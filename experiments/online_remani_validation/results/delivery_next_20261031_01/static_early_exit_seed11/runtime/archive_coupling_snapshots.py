#!/usr/bin/env python3
"""Lossless post-run tar.xz archival of NEW coupling-trial snapshot files.

No ROS node is started. Originals are removed only after every archive entry
has been read back and matched against the original file SHA. Existing trial
manifests and compression_audit.json are never modified. This CPU-intensive
postprocess must not run concurrently with a measured ROS experiment.
"""
import argparse
from contextlib import contextmanager
from datetime import datetime, timedelta, timezone
import errno
import hashlib
import json
import os
from pathlib import Path
import re
import socket
import stat
import tarfile
import tempfile


ARCHIVE = 'collision_snapshots.tar.xz'
LEDGER = 'collision_snapshot_archive.json'
LOCK = '.collision_snapshot_archive.lock'
GROUPS = {'native', 'engineering', 'execution', 'handover', 'coupled'}
SAFE_NAME = re.compile(r'^[0-9]{5,}\.npz$')


def sha_stream(stream):
    digest = hashlib.sha256()
    for block in iter(lambda: stream.read(1024 * 1024), b''):
        digest.update(block)
    return digest.hexdigest()


def regular(path):
    """lstat first: a dangling symlink must not look like a missing file."""
    info = Path(path).lstat()
    if not stat.S_ISREG(info.st_mode):
        raise ValueError('Refusing nonregular file or symlink: ' + str(path))
    return info


def file_sha(path):
    regular(path)
    fd = os.open(str(path), os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0))
    with os.fdopen(fd, 'rb') as stream:
        if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
            raise ValueError('File changed to nonregular input: ' + str(path))
        return sha_stream(stream)


def fsync_directory(path):
    fd = os.open(str(path), os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0) | getattr(os, 'O_NOFOLLOW', 0))
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def exists(path):
    return os.path.lexists(str(path))


def read_json(path):
    regular(path)
    with Path(path).open() as stream:
        return json.load(stream)


def checked_trial(path):
    trial = Path(path).absolute()
    if trial.is_symlink() or not trial.is_dir() or trial.resolve() != trial:
        raise ValueError('Trial must be an explicit existing directory without symlink/.. traversal')
    manifest = read_json(trial / 'manifest.json')
    coupling = manifest.get('coupling_experiment')
    if not isinstance(coupling, dict) or coupling.get('group') not in GROUPS:
        raise ValueError('Only newly generated named coupling-experiment trials may be archived')
    status = read_json(trial / 'run_status.json')
    if status.get('returncode') is None:
        raise ValueError('A completed run_status with a process returncode is required')
    pids = status.get('monitored_node_pids')
    if not isinstance(pids, dict) or not pids:
        raise ValueError('Recorded node PIDs are required to establish post-run scope')
    return trial, manifest, status


def assert_quiet(trial, status):
    """Fail closed if the trial's ROS processes or isolated master may be live."""
    live = []
    for name, pid in status['monitored_node_pids'].items():
        if not isinstance(pid, int) or isinstance(pid, bool) or pid <= 1:
            raise ValueError('Invalid monitored PID for ' + str(name))
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            continue
        except PermissionError as exc:
            raise RuntimeError('Cannot establish that monitored process is inactive: ' + str(pid)) from exc
        # A zombie has exited and cannot be writing. Otherwise even possible
        # PID reuse is conservatively rejected; no unrelated process is killed.
        try:
            raw = (Path('/proc') / str(pid) / 'stat').read_text()
            state = raw[raw.rfind(')') + 2:].split()[0]
        except FileNotFoundError:
            continue
        if state != 'Z':
            live.append((name, pid))
    if live:
        raise RuntimeError('Monitored ROS processes still active; no archive mutation: ' + str(live))
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(.5)
            result = sock.connect_ex(('127.0.0.1', 11441))
    except OSError as exc:
        raise RuntimeError('Cannot prove isolated ROS master is inactive; refusing archival') from exc
    if result == 0:
        raise RuntimeError('Isolated ROS master 11441 is active; do not archive during timing')
    if result != errno.ECONNREFUSED:
        raise RuntimeError('Ambiguous isolated-master state (errno %s); refusing archival' % result)


@contextmanager
def trial_lock(trial):
    path = trial / LOCK
    fd = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, 'O_NOFOLLOW', 0), 0o600)
    info = os.fstat(fd)
    try:
        os.write(fd, ('pid=%d\n' % os.getpid()).encode())
        yield
    finally:
        os.close(fd)
        try:
            current = path.lstat()
            if current.st_ino == info.st_ino and current.st_dev == info.st_dev:
                path.unlink()  # only the exact lock created by this operation
        except FileNotFoundError:
            pass


def write_json(path, value, exclusive):
    fd, temporary = tempfile.mkstemp(prefix='.' + path.name + '.', suffix='.partial', dir=str(path.parent))
    temporary = Path(temporary)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        if exclusive:
            os.link(str(temporary), str(path))  # atomic, never replace a conflict
        else:
            regular(path)
            os.replace(str(temporary), str(path))  # update only this operation's ledger
        fsync_directory(path.parent)
    finally:
        if temporary.exists():
            temporary.unlink()


def signature(info):
    return (info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns)


def snapshot_files(directory):
    if directory.is_symlink() or not directory.is_dir():
        raise ValueError('Snapshot source must be a real directory')
    records = {}
    for path in sorted(directory.iterdir()):
        if not SAFE_NAME.fullmatch(path.name):
            raise ValueError('Unexpected snapshot entry: ' + str(path))
        before = regular(path)
        digest = file_sha(path)
        after = regular(path)
        if signature(before) != signature(after):
            raise RuntimeError('Snapshot changed during initial hashing: ' + str(path))
        records[path.name] = dict(bytes=after.st_size, sha256=digest,
                                  source_signature=list(signature(after)))
    return records


def verify_content(archive, records, archive_sha=None):
    """Read entries without extraction; reject duplicate names and all links."""
    if archive_sha is not None and file_sha(archive) != archive_sha:
        raise RuntimeError('Archive SHA mismatch')
    if not records:
        raise ValueError('Empty snapshot archive is not a valid completed archive')
    for name, record in records.items():
        if not SAFE_NAME.fullmatch(name) or not isinstance(record.get('bytes'), int) or record['bytes'] < 0:
            raise ValueError('Invalid snapshot name/size in archive ledger')
        if not re.fullmatch(r'[0-9a-f]{64}', str(record.get('sha256'))):
            raise ValueError('Invalid original SHA in archive ledger')
    seen = set()
    with tarfile.open(archive, mode='r:xz') as stream:
        for member in stream:
            if not member.isfile() or member.name not in records or member.name in seen:
                raise ValueError('Unexpected, duplicate or nonregular tar entry: ' + member.name)
            record = records[member.name]
            if member.size != record['bytes']:
                raise RuntimeError('Archived size mismatch: ' + member.name)
            extracted = stream.extractfile(member)
            if extracted is None:
                raise RuntimeError('Cannot read archive entry: ' + member.name)
            with extracted:
                if sha_stream(extracted) != record['sha256']:
                    raise RuntimeError('Archived entry SHA mismatch: ' + member.name)
            seen.add(member.name)
    if seen != set(records):
        raise RuntimeError('Archive is missing recorded snapshot entries')
    return len(seen)


def check_sources(directory, records):
    # Rehash the entire source set before deleting ANY original. Also rejects
    # newly added entries, symlinks and rename/replacement races at this gate.
    current = snapshot_files(directory)
    if current != records:
        raise RuntimeError('Original snapshot set changed; verified archive retained, originals not removed')


def archive_trial(path, preset=6):
    if not isinstance(preset, int) or not 0 <= preset <= 9:
        raise ValueError('XZ preset must be in 0..9')
    trial, manifest, status = checked_trial(path)
    assert_quiet(trial, status)
    archive, ledger_path, directory = trial / ARCHIVE, trial / LEDGER, trial / 'collision_snapshots'
    if exists(archive) or exists(ledger_path):
        raise FileExistsError('Archive/ledger already exists; use verify or restore, never overwrite')
    if not exists(directory):
        if manifest['coupling_experiment']['group'] == 'native':
            return dict(skipped=True, reason='native group has no shared collision snapshots', trial=str(trial))
        raise FileNotFoundError('Snapshot source is missing: ' + str(directory))
    with trial_lock(trial):
        records = snapshot_files(directory)
        if not records:
            return dict(skipped=True, reason='snapshot directory is empty', trial=str(trial))
        fd, temporary = tempfile.mkstemp(prefix='.collision_snapshots.', suffix='.tar.xz.partial', dir=str(trial))
        os.close(fd)
        temporary = Path(temporary)
        try:
            with tarfile.open(temporary, mode='w:xz', preset=preset) as stream:
                for name in records:
                    source = directory / name
                    regular(source)
                    stream.add(source, arcname=name, recursive=False)
            verify_content(temporary, records)
            archive_sha = file_sha(temporary)
            check_sources(directory, records)
            fd = os.open(str(temporary), os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0))
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
            # Link installation is atomic and refuses an output installed by
            # another process after the conflict check.
            os.link(str(temporary), str(archive))
            fsync_directory(trial)
        finally:
            if temporary.exists():
                temporary.unlink()  # only this invocation's temporary archive
        ledger = dict(format='coupling_collision_snapshots_tar_xz_v1',
            created=datetime.now(timezone(timedelta(hours=8))).isoformat(), timezone='Asia/Shanghai',
            post_run_only=True, scope='lossless storage postprocess; not online timing or algorithm result',
            trial=str(trial), group=manifest['coupling_experiment']['group'],
            trial_manifest_sha256=file_sha(trial / 'manifest.json'),
            run_status_sha256=file_sha(trial / 'run_status.json'),
            source_directory='collision_snapshots', archive=ARCHIVE,
            archive_sha256=archive_sha, archive_bytes=regular(archive).st_size,
            original_bytes=sum(v['bytes'] for v in records.values()), file_count=len(records),
            entries=records, xz_preset=preset, status='verified_originals_retained',
            originals_removed=False,
            recovery='verify archive, then restore into an explicit new directory; all original bytes recoverable')
        ledger['space_saved_bytes'] = ledger['original_bytes'] - ledger['archive_bytes']
        write_json(ledger_path, ledger, exclusive=True)
        # No destructive step occurs until the complete installed archive and
        # durable recovery ledger have been verified.
        verify_content(archive, records, archive_sha)
        assert_quiet(trial, status)
        check_sources(directory, records)
        removed = []
        try:
            for name, record in records.items():
                source = directory / name
                info = regular(source)
                if list(signature(info)) != record['source_signature'] or file_sha(source) != record['sha256']:
                    raise RuntimeError('Snapshot changed before removal: ' + name)
                source.unlink()  # recoverable byte-for-byte from verified tar.xz
                removed.append(name)
            directory.rmdir()  # fails closed if an unexpected new file appeared
        except Exception as exc:
            ledger.update(status='verified_archive_partial_original_removal', removed_files=removed,
                          removal_error='%s: %s' % (type(exc).__name__, exc))
            write_json(ledger_path, ledger, exclusive=False)
            raise
        ledger.update(status='archived', originals_removed=True, removed_files=removed)
        write_json(ledger_path, ledger, exclusive=False)
        return ledger


def verify_trial(path):
    trial, _, status = checked_trial(path)
    assert_quiet(trial, status)
    if exists(trial / LOCK):
        raise RuntimeError('Archive operation lock exists; refusing concurrent verification')
    ledger = read_json(trial / LEDGER)
    if ledger.get('format') != 'coupling_collision_snapshots_tar_xz_v1' or ledger.get('archive') != ARCHIVE:
        raise ValueError('Unexpected archive ledger format/path')
    if not re.fullmatch(r'[0-9a-f]{64}', str(ledger.get('archive_sha256'))):
        raise ValueError('Archive ledger must contain an explicit SHA256')
    if ledger.get('trial_manifest_sha256') != file_sha(trial / 'manifest.json') or ledger.get('run_status_sha256') != file_sha(trial / 'run_status.json'):
        raise RuntimeError('Trial manifest/run status changed after archival')
    archive = trial / ARCHIVE
    if regular(archive).st_size != ledger.get('archive_bytes'):
        raise RuntimeError('Archive byte count mismatch')
    count = verify_content(archive, ledger.get('entries', {}), ledger.get('archive_sha256'))
    if count != ledger.get('file_count'):
        raise RuntimeError('Archive ledger file count mismatch')
    if sum(v['bytes'] for v in ledger['entries'].values()) != ledger.get('original_bytes'):
        raise RuntimeError('Archive ledger original-byte total mismatch')
    return ledger


def restore_trial(path, destination):
    ledger = verify_trial(path)
    trial = Path(path).absolute()
    target = Path(destination).absolute()
    if exists(target):
        raise FileExistsError('Restore destination must be an explicit new directory; no overwrite')
    if target.parent.resolve() != target.parent or not target.parent.is_dir():
        raise ValueError('Restore parent must already exist without symlink/.. traversal')
    target.mkdir(mode=0o700, exist_ok=False)
    restored = []
    with tarfile.open(trial / ARCHIVE, mode='r:xz') as stream:
        for member in stream:
            if member.name not in ledger['entries'] or not member.isfile() or not SAFE_NAME.fullmatch(member.name):
                raise ValueError('Unexpected archive entry during restore')
            output = target / member.name
            source = stream.extractfile(member)
            digest = hashlib.sha256()
            size = 0
            with source, output.open('xb') as dest:
                for block in iter(lambda: source.read(1024 * 1024), b''):
                    dest.write(block)
                    digest.update(block)
                    size += len(block)
            expected = ledger['entries'][member.name]
            if digest.hexdigest() != expected['sha256'] or size != expected['bytes']:
                raise RuntimeError('Restored entry failed byte verification: ' + member.name)
            restored.append(member.name)
    if set(restored) != set(ledger['entries']):
        raise RuntimeError('Incomplete restore; partial new directory retained for inspection')
    if file_sha(trial / ARCHIVE) != ledger['archive_sha256']:
        raise RuntimeError('Archive changed during restore; new directory retained for inspection')
    receipt = dict(trial=str(trial), archive_sha256=ledger['archive_sha256'],
                   files=restored, all_original_bytes_verified=True)
    write_json(target / 'RESTORED_FROM.json', receipt, exclusive=True)
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('archive', 'verify', 'restore'))
    parser.add_argument('--trial', type=Path, required=True)
    parser.add_argument('--restore-to', type=Path)
    parser.add_argument('--preset', type=int, default=6)
    args = parser.parse_args()
    if args.action == 'restore' and args.restore_to is None:
        parser.error('restore requires --restore-to <new directory>')
    if args.action != 'restore' and args.restore_to is not None:
        parser.error('--restore-to only applies to restore')
    result = archive_trial(args.trial, args.preset) if args.action == 'archive' else verify_trial(args.trial) if args.action == 'verify' else restore_trial(args.trial, args.restore_to)
    print(json.dumps({key: value for key, value in result.items() if key not in ('entries', 'removed_files')},
                     ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
