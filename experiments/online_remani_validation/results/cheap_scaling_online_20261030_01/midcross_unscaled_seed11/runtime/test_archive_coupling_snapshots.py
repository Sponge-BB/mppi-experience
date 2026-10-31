"""Temporary fixtures only; never archive real trial output or start ROS."""
import errno
import hashlib
import io
import json
import os
from pathlib import Path
import tarfile
import tempfile
import unittest
from unittest.mock import patch

import archive_coupling_snapshots as archive


class SnapshotArchiveContracts(unittest.TestCase):
    def fixture(self, root, group='engineering'):
        trial = root / 'trial'
        trial.mkdir()
        (trial / 'manifest.json').write_text(json.dumps(dict(
            coupling_experiment=dict(group=group, seed=1))))
        (trial / 'run_status.json').write_text(json.dumps(dict(
            returncode=0, monitored_node_pids={'/base_mppi': 99999999})))
        (trial / 'compression_audit.json').write_text('{"untouched":"existing gzip record"}\n')
        if group != 'native':
            directory = trial / 'collision_snapshots'
            directory.mkdir()
            (directory / '00001.npz').write_bytes(b'original bytes one\x00' * 128)
            (directory / '00002.npz').write_bytes(b'original bytes two\x00' * 128)
        return trial

    def test_roundtrip_verifies_every_byte_and_preserves_existing_manifests(self):
        with tempfile.TemporaryDirectory(prefix='coupling_archive_contract_') as directory:
            root = Path(directory)
            trial = self.fixture(root)
            originals = {p.name: p.read_bytes() for p in (trial / 'collision_snapshots').iterdir()}
            before = {name: (trial / name).read_bytes() for name in
                      ('manifest.json', 'run_status.json', 'compression_audit.json')}
            with patch.object(archive, 'assert_quiet'):
                ledger = archive.archive_trial(trial, preset=0)
                verified = archive.verify_trial(trial)
                receipt = archive.restore_trial(trial, root / 'restored')
            self.assertTrue(ledger['originals_removed'])
            self.assertEqual(verified['archive_sha256'], ledger['archive_sha256'])
            self.assertEqual(ledger['file_count'], 2)
            self.assertTrue(receipt['all_original_bytes_verified'])
            self.assertFalse((trial / 'collision_snapshots').exists())
            self.assertFalse((trial / archive.LOCK).exists())
            for name, content in originals.items():
                self.assertEqual((root / 'restored' / name).read_bytes(), content)
                self.assertEqual(ledger['entries'][name]['sha256'], hashlib.sha256(content).hexdigest())
            for name, content in before.items():
                self.assertEqual((trial / name).read_bytes(), content)

    def test_old_experiment_and_incomplete_status_are_rejected(self):
        with tempfile.TemporaryDirectory(prefix='coupling_archive_contract_') as directory:
            trial = self.fixture(Path(directory))
            (trial / 'manifest.json').write_text('{"factorial":{"group":"A"}}')
            with self.assertRaisesRegex(ValueError, 'coupling-experiment'):
                archive.archive_trial(trial)
            (trial / 'manifest.json').write_text('{"coupling_experiment":{"group":"engineering"}}')
            (trial / 'run_status.json').write_text('{"returncode":null}')
            with self.assertRaisesRegex(ValueError, 'completed run_status'):
                archive.archive_trial(trial)

    def test_existing_archive_and_ledger_are_never_overwritten(self):
        for name in (archive.ARCHIVE, archive.LEDGER):
            with self.subTest(name=name), tempfile.TemporaryDirectory(prefix='coupling_archive_contract_') as directory:
                trial = self.fixture(Path(directory))
                target = trial / name
                target.write_bytes(b'existing output')
                with patch.object(archive, 'assert_quiet'), self.assertRaises(FileExistsError):
                    archive.archive_trial(trial)
                self.assertEqual(target.read_bytes(), b'existing output')
                self.assertEqual(len(list((trial / 'collision_snapshots').iterdir())), 2)

    def test_unexpected_entry_and_symlink_are_rejected(self):
        for kind in ('symlink', 'directory', 'unexpected'):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory(prefix='coupling_archive_contract_') as directory:
                trial = self.fixture(Path(directory))
                snapshots = trial / 'collision_snapshots'
                if kind == 'symlink':
                    (snapshots / '00003.npz').symlink_to(snapshots / '00001.npz')
                elif kind == 'directory':
                    (snapshots / '00003.npz').mkdir()
                else:
                    (snapshots / 'notes.txt').write_text('not a snapshot')
                with patch.object(archive, 'assert_quiet'), self.assertRaises(ValueError):
                    archive.archive_trial(trial)
                self.assertFalse((trial / archive.ARCHIVE).exists())
                self.assertTrue((snapshots / '00001.npz').exists())

    def test_snapshot_directory_symlink_is_rejected(self):
        with tempfile.TemporaryDirectory(prefix='coupling_archive_contract_') as directory:
            root = Path(directory)
            trial = self.fixture(root)
            source = trial / 'collision_snapshots'
            source.rename(root / 'other_data')
            source.symlink_to(root / 'other_data', target_is_directory=True)
            with patch.object(archive, 'assert_quiet'), self.assertRaises(ValueError):
                archive.archive_trial(trial)
            self.assertEqual(len(list((root / 'other_data').iterdir())), 2)

    def test_verification_failure_preserves_all_originals(self):
        with tempfile.TemporaryDirectory(prefix='coupling_archive_contract_') as directory:
            trial = self.fixture(Path(directory))
            with patch.object(archive, 'assert_quiet'), patch.object(archive, 'verify_content', side_effect=RuntimeError('injected readback failure')):
                with self.assertRaisesRegex(RuntimeError, 'readback failure'):
                    archive.archive_trial(trial, preset=0)
            self.assertEqual(len(list((trial / 'collision_snapshots').iterdir())), 2)
            self.assertFalse((trial / archive.ARCHIVE).exists())
            self.assertFalse((trial / archive.LEDGER).exists())
            self.assertFalse((trial / archive.LOCK).exists())

    def test_new_activity_before_removal_retains_archive_and_originals(self):
        with tempfile.TemporaryDirectory(prefix='coupling_archive_contract_') as directory:
            trial = self.fixture(Path(directory))
            with patch.object(archive, 'assert_quiet', side_effect=[None, RuntimeError('ROS restarted')]):
                with self.assertRaisesRegex(RuntimeError, 'ROS restarted'):
                    archive.archive_trial(trial, preset=0)
            self.assertEqual(len(list((trial / 'collision_snapshots').iterdir())), 2)
            self.assertTrue((trial / archive.ARCHIVE).exists())
            self.assertEqual(archive.read_json(trial / archive.LEDGER)['status'], 'verified_originals_retained')

    def test_changed_source_at_final_gate_is_not_removed(self):
        with tempfile.TemporaryDirectory(prefix='coupling_archive_contract_') as directory:
            trial = self.fixture(Path(directory))
            source = trial / 'collision_snapshots/00002.npz'
            calls = []

            def quiet(*_):
                calls.append(True)
                if len(calls) == 2:
                    source.write_bytes(b'new unexpected writer data')

            with patch.object(archive, 'assert_quiet', side_effect=quiet):
                with self.assertRaisesRegex(RuntimeError, 'set changed'):
                    archive.archive_trial(trial, preset=0)
            self.assertEqual(source.read_bytes(), b'new unexpected writer data')
            self.assertTrue((trial / 'collision_snapshots/00001.npz').exists())
            self.assertEqual(archive.read_json(trial / archive.LEDGER)['status'], 'verified_originals_retained')

    def test_partial_removal_failure_still_restores_all_originals(self):
        with tempfile.TemporaryDirectory(prefix='coupling_archive_contract_') as directory:
            root = Path(directory)
            trial = self.fixture(root)
            originals = {p.name: p.read_bytes() for p in (trial / 'collision_snapshots').iterdir()}
            original_unlink = Path.unlink

            def unlink(path, *args, **kwargs):
                if path.name == '00002.npz':
                    raise OSError('injected removal error')
                return original_unlink(path, *args, **kwargs)

            with patch.object(archive, 'assert_quiet'), patch.object(Path, 'unlink', new=unlink):
                with self.assertRaisesRegex(OSError, 'removal error'):
                    archive.archive_trial(trial, preset=0)
            ledger = archive.read_json(trial / archive.LEDGER)
            self.assertEqual(ledger['status'], 'verified_archive_partial_original_removal')
            self.assertEqual(ledger['removed_files'], ['00001.npz'])
            with patch.object(archive, 'assert_quiet'):
                archive.restore_trial(trial, root / 'restored')
            for name, content in originals.items():
                self.assertEqual((root / 'restored' / name).read_bytes(), content)

    def test_corrupt_archive_or_changed_trial_is_rejected(self):
        for change in ('archive', 'manifest'):
            with self.subTest(change=change), tempfile.TemporaryDirectory(prefix='coupling_archive_contract_') as directory:
                trial = self.fixture(Path(directory))
                with patch.object(archive, 'assert_quiet'):
                    archive.archive_trial(trial, preset=0)
                    if change == 'archive':
                        with (trial / archive.ARCHIVE).open('r+b') as stream:
                            stream.write(b'bad')
                    else:
                        (trial / 'manifest.json').write_text('{"coupling_experiment":{"group":"coupled"}}')
                    with self.assertRaises(RuntimeError):
                        archive.verify_trial(trial)

    def test_restore_requires_new_explicit_directory(self):
        with tempfile.TemporaryDirectory(prefix='coupling_archive_contract_') as directory:
            root = Path(directory)
            trial = self.fixture(root)
            target = root / 'existing'
            target.mkdir()
            (target / 'keep.txt').write_text('keep')
            with patch.object(archive, 'assert_quiet'):
                archive.archive_trial(trial, preset=0)
                with self.assertRaises(FileExistsError):
                    archive.restore_trial(trial, target)
            self.assertEqual((target / 'keep.txt').read_text(), 'keep')

    def test_native_missing_snapshots_is_explicit_skip(self):
        with tempfile.TemporaryDirectory(prefix='coupling_archive_contract_') as directory:
            trial = self.fixture(Path(directory), group='native')
            with patch.object(archive, 'assert_quiet'):
                result = archive.archive_trial(trial)
            self.assertTrue(result['skipped'])
            self.assertFalse((trial / archive.ARCHIVE).exists())

    def test_tar_path_traversal_links_and_duplicates_are_rejected(self):
        for kind in ('path', 'symlink', 'duplicate'):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory(prefix='coupling_archive_contract_') as directory:
                target = Path(directory) / 'bad.tar.xz'
                payload = b'data'
                with tarfile.open(target, 'w:xz', preset=0) as stream:
                    item = tarfile.TarInfo('../00001.npz' if kind == 'path' else '00001.npz')
                    if kind == 'symlink':
                        item.type = tarfile.SYMTYPE
                        item.linkname = '../../outside'
                        stream.addfile(item)
                    else:
                        item.size = len(payload)
                        stream.addfile(item, io.BytesIO(payload))
                        if kind == 'duplicate':
                            stream.addfile(item, io.BytesIO(payload))
                records = {'00001.npz': dict(bytes=len(payload), sha256=hashlib.sha256(payload).hexdigest())}
                with self.assertRaises(ValueError):
                    archive.verify_content(target, records)

    def test_live_process_and_master_are_never_stopped_or_archived(self):
        with tempfile.TemporaryDirectory(prefix='coupling_archive_contract_') as directory:
            trial = self.fixture(Path(directory))
            status = dict(monitored_node_pids={'/fake': os.getpid()})
            with self.assertRaisesRegex(RuntimeError, 'still active'):
                archive.assert_quiet(trial, status)
            status = dict(monitored_node_pids={'/fake': 99999999})
            with patch.object(archive.os, 'kill', side_effect=ProcessLookupError), patch.object(archive.socket, 'socket') as socket_factory:
                socket_factory.return_value.__enter__.return_value.connect_ex.return_value = 0
                with self.assertRaisesRegex(RuntimeError, '11441 is active'):
                    archive.assert_quiet(trial, status)
                socket_factory.return_value.__enter__.return_value.connect_ex.return_value = errno.ECONNREFUSED
                archive.assert_quiet(trial, status)

    def test_ambiguous_port_state_fails_closed(self):
        with patch.object(archive.os, 'kill', side_effect=ProcessLookupError), patch.object(archive.socket, 'socket') as socket_factory:
            socket_factory.return_value.__enter__.return_value.connect_ex.return_value = errno.EPERM
            with self.assertRaisesRegex(RuntimeError, 'Ambiguous'):
                archive.assert_quiet(Path('/unused'), dict(monitored_node_pids={'/fake': 99999999}))


if __name__ == '__main__':
    unittest.main()
