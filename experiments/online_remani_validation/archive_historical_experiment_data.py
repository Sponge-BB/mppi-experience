#!/usr/bin/env python3
"""Lossless, streaming archival of selected historical experiment evidence.

Only the three documented evidence classes below are eligible. The latest arm
experiments, frozen runtime trees, source trees, images and summaries are never
changed. ``plan`` only inventories metadata; ``apply`` requires that saved plan.
Compressed siblings stay beside their original paths, with a durable recovery
ledger under experiments/archives/cleanup_20261105.

Examples::
    python3 archive_historical_experiment_data.py plan
    python3 archive_historical_experiment_data.py apply --plan PLAN.json
    python3 archive_historical_experiment_data.py restore --group native_v12

Consumers expecting .f64/.jsonl need ``restore --group ...`` before replaying.
No ROS process is started, and no full data file is loaded into memory.
"""

import argparse
from contextlib import contextmanager
import ctypes
import errno
import gzip
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import stat
import sys
import tempfile
from datetime import datetime, timezone


WORKSPACE = Path(__file__).resolve().parents[2]
RESULTS = WORKSPACE / 'experiments/online_remani_validation/results'
ARCHIVES = WORKSPACE / 'experiments/archives/cleanup_20261105'
DEFAULT_PLAN = ARCHIVES / 'historical_archive_plan.json'
MANIFEST = ARCHIVES / 'historical_archive_manifest.jsonl'
LOCK = ARCHIVES / '.historical_archive.lock'
EXCLUDED = {'active_arm_20261104_01', 'standby_arm_influence_20261105_01'}
PRUNED = {'runtime', 'ros_pkgs', '__pycache__', '.git'}
MINIMUM_SIZE = 128 * 1024
CHUNK = 1024 * 1024
SCHEMA = 'historical_experiment_gzip_v1'


def now():
    return datetime.now(timezone.utc).isoformat()


def emit(value):
    print(json.dumps(value, ensure_ascii=False), flush=True)


def fsync_directory(path):
    fd = os.open(str(path), os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0))
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def snapshot(info):
    return {key: getattr(info, 'st_' + key) for key in
            ('dev', 'ino', 'size', 'mtime_ns', 'ctime_ns', 'nlink', 'mode', 'atime_ns')}


def identity(info):
    # Reads may update atime. Every other identity/mutation field must be stable.
    data = snapshot(info)
    data.pop('atime_ns')
    return data


def regular(path):
    info = path.lstat()
    if not stat.S_ISREG(info.st_mode):
        raise ValueError('Expected regular non-symlink file: ' + str(path))
    return info


def check_components(path, base):
    path = Path(path).absolute()
    base = Path(base).absolute()
    try:
        relative = path.relative_to(base)
    except ValueError as exc:
        raise ValueError('Path outside permitted root: ' + str(path)) from exc
    current = base
    if current.is_symlink() or not current.is_dir():
        raise ValueError('Root is not a real directory: ' + str(current))
    for part in relative.parts[:-1]:
        current = current / part
        info = current.lstat()
        if not stat.S_ISDIR(info.st_mode):
            raise ValueError('Non-directory/symlink ancestor: ' + str(current))
    return path


def evidence_kind(parts):
    if any(part in PRUNED for part in parts):
        return None
    if parts[-1] in ('execution_rollouts.jsonl', 'selected_rollouts.jsonl'):
        return parts[-1]
    if len(parts) >= 4 and parts[-3:-1] == ('native3d_snapshots', 'maps') \
            and parts[-1].endswith('.f64'):
        return 'native3d_maps_f64'
    return None


def checked_evidence(relative):
    rel = PurePosixPath(relative)
    if rel.is_absolute() or '..' in rel.parts or len(rel.parts) < 2:
        raise ValueError('Invalid evidence relative path: ' + str(relative))
    if rel.parts[0] in EXCLUDED or not evidence_kind(rel.parts):
        raise ValueError('Path is outside eligible historical evidence: ' + str(rel))
    return check_components(RESULTS.joinpath(*rel.parts), RESULTS)


def open_regular(path):
    fd = os.open(str(path), os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0))
    stream = os.fdopen(fd, 'rb')
    try:
        if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
            raise ValueError('Not a regular file: ' + str(path))
    except BaseException:
        stream.close()
        raise
    return stream


def digest_stream(stream):
    digest, length = hashlib.sha256(), 0
    while True:
        block = stream.read(CHUNK)
        if not block:
            break
        digest.update(block)
        length += len(block)
    return digest.hexdigest(), length


def digest_file(path):
    with open_regular(path) as stream:
        before = identity(os.fstat(stream.fileno()))
        result = digest_stream(stream)
        if identity(os.fstat(stream.fileno())) != before or identity(regular(path)) != before:
            raise RuntimeError('File changed while hashing: ' + str(path))
        return result


def verify_gzip(path, expected_sha, expected_size):
    with open_regular(path) as raw:
        before = identity(os.fstat(raw.fileno()))
        with gzip.GzipFile(fileobj=raw, mode='rb') as compressed:
            digest, length = digest_stream(compressed)
        if (digest, length) != (expected_sha, expected_size):
            raise RuntimeError('Decompressed SHA/size mismatch: ' + str(path))
        if identity(os.fstat(raw.fileno())) != before or identity(regular(path)) != before:
            raise RuntimeError('Gzip changed during verification: ' + str(path))
    return before


def publish_no_replace(temporary, target):
    """Atomically publish a completed sibling; never replace an existing file."""
    # Linux renameat2 provides the desired rename-with-no-overwrite primitive.
    libc = ctypes.CDLL(None, use_errno=True)
    renameat2 = getattr(libc, 'renameat2', None)
    if renameat2 is not None:
        renameat2.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int,
                             ctypes.c_char_p, ctypes.c_uint]
        renameat2.restype = ctypes.c_int
        result = renameat2(-100, os.fsencode(temporary), -100, os.fsencode(target), 1)
        if result == 0:
            fsync_directory(target.parent)
            return
        error = ctypes.get_errno()
        if error not in (errno.ENOSYS, errno.EINVAL, errno.EOPNOTSUPP):
            raise OSError(error, os.strerror(error), str(target))
    # Atomic no-overwrite fallback on filesystems lacking renameat2 support.
    os.link(str(temporary), str(target), follow_symlinks=False)
    temporary.unlink()
    fsync_directory(target.parent)


def ensure_archives():
    ARCHIVES.mkdir(parents=True, exist_ok=True)
    check_components(MANIFEST, ARCHIVES)
    if ARCHIVES.is_symlink():
        raise ValueError('Archive directory cannot be a symlink')


@contextmanager
def operation_lock():
    ensure_archives()
    fd = os.open(str(LOCK), os.O_WRONLY | os.O_CREAT | os.O_EXCL |
                 getattr(os, 'O_NOFOLLOW', 0), 0o600)
    owned = os.fstat(fd)
    try:
        os.write(fd, json.dumps({'pid': os.getpid(), 'created': now()}).encode())
        os.fsync(fd)
        yield
    finally:
        os.close(fd)
        try:
            info = LOCK.lstat()
            if (info.st_dev, info.st_ino) == (owned.st_dev, owned.st_ino):
                LOCK.unlink()
        except FileNotFoundError:
            pass


@contextmanager
def ledger_writer():
    flags = os.O_WRONLY | os.O_APPEND | os.O_CREAT | getattr(os, 'O_NOFOLLOW', 0)
    fd = os.open(str(MANIFEST), flags, 0o600)
    with os.fdopen(fd, 'a', encoding='utf-8') as stream:
        if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
            raise ValueError('Manifest must be a regular file')
        fsync_directory(ARCHIVES)

        def append(record):
            line = {'schema': SCHEMA, 'recorded_at': now(), **record}
            stream.write(json.dumps(line, ensure_ascii=False, sort_keys=True) + '\n')
            stream.flush()
            os.fsync(stream.fileno())

        yield append


def selected_groups(names):
    if names:
        if any('/' in name or name in ('', '.', '..') for name in names):
            raise ValueError('--group accepts a direct results directory name')
        groups = sorted(set(names))
    else:
        groups = sorted(p.name for p in RESULTS.iterdir()
                        if p.is_dir() and not p.is_symlink() and p.name not in EXCLUDED)
    for name in groups:
        if name in EXCLUDED:
            raise ValueError('Latest experiment group is protected: ' + name)
        group = RESULTS / name
        if group.is_symlink() or not group.is_dir():
            raise ValueError('Not a historical result directory: ' + name)
    return groups


def plan_command(args):
    groups = selected_groups(args.group)
    entries, summary, seen = [], {}, set()
    total_allocated_unique = 0
    for group in groups:
        count, size, allocated = 0, 0, 0
        for parent, directories, files in os.walk(RESULTS / group, followlinks=False):
            directories[:] = sorted(d for d in directories if d not in PRUNED
                                    and not (Path(parent) / d).is_symlink())
            for name in sorted(files):
                path = Path(parent) / name
                rel = path.relative_to(RESULTS).as_posix()
                kind = evidence_kind(PurePosixPath(rel).parts)
                if kind is None:
                    continue
                info = path.lstat()
                if not stat.S_ISREG(info.st_mode) or info.st_size < MINIMUM_SIZE:
                    continue
                checked_evidence(rel)
                entries.append({'path': rel, 'kind': kind, 'stat': snapshot(info)})
                count += 1
                size += info.st_size
                allocated += info.st_blocks * 512
                key = (info.st_dev, info.st_ino)
                if key not in seen:
                    total_allocated_unique += info.st_blocks * 512
                    seen.add(key)
        if count:
            summary[group] = {'files': count, 'bytes': size, 'allocated_bytes': allocated}
    document = {'schema': SCHEMA, 'created': now(), 'results_root': str(RESULTS),
                'excluded_groups': sorted(EXCLUDED), 'minimum_size': MINIMUM_SIZE,
                'groups': groups, 'entries': entries, 'summary': summary,
                'totals': {'files': len(entries), 'bytes': sum(e['stat']['size'] for e in entries),
                           'allocated_unique_bytes': total_allocated_unique},
                'note': 'Plan only; bytes are not guaranteed savings. Other hardlinks may retain blocks.'}
    ensure_archives()
    output = Path(args.output).absolute()
    check_components(output, ARCHIVES)
    fd, name = tempfile.mkstemp(prefix='.historical-plan.', dir=str(output.parent))
    temporary = Path(name)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            json.dump(document, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        publish_no_replace(temporary, output)
    finally:
        if temporary.exists():
            temporary.unlink()
    emit({'action': 'plan', 'plan': str(output), **document['totals'], 'groups': summary})


def load_ready_records():
    records = {}
    if not MANIFEST.exists():
        return records
    with open_regular(MANIFEST) as stream:
        for number, raw in enumerate(stream, 1):
            if not raw.endswith(b'\n'):
                raise RuntimeError('Incomplete ledger line %d; inspect before continuing' % number)
            row = json.loads(raw)
            if row.get('schema') != SCHEMA:
                raise ValueError('Unexpected ledger schema')
            if row.get('event') == 'archive_ready':
                checked_evidence(row['path'])
                records[row['path']] = row
    return records


def archive_one(entry, append, ready, level):
    path = checked_evidence(entry['path'])
    gz_path = path.with_name(path.name + '.gz')
    if not path.exists():
        record = ready.get(entry['path'])
        if not record or not gz_path.exists():
            raise RuntimeError('Original absent without durable archive evidence: ' + str(path))
        if record.get('gzip_path') != gz_path.relative_to(RESULTS).as_posix():
            raise ValueError('Ledger gzip path mismatch')
        verify_gzip(gz_path, record['sha256'], record['bytes'])
        return 'already_archived', 0
    temporary = None
    with open_regular(path) as source:
        before = os.fstat(source.fileno())
        expected = entry['stat']
        if any(getattr(before, 'st_' + key) != expected[key]
               for key in ('dev', 'ino', 'size', 'mtime_ns')):
            raise RuntimeError('Source no longer matches plan: ' + str(path))
        if before.st_size < MINIMUM_SIZE:
            raise ValueError('Source below minimum eligible size')
        reused = os.path.lexists(gz_path)
        try:
            if reused:
                original_sha, length = digest_stream(source)
                verify_gzip(gz_path, original_sha, length)
            else:
                fd, name = tempfile.mkstemp(prefix='.' + path.name + '.', suffix='.gzip.partial',
                                            dir=str(path.parent))
                temporary = Path(name)
                digest, length = hashlib.sha256(), 0
                with os.fdopen(fd, 'wb') as raw:
                    with gzip.GzipFile(filename='', mode='wb', fileobj=raw,
                                       compresslevel=level, mtime=0) as compressed:
                        while True:
                            block = source.read(CHUNK)
                            if not block:
                                break
                            digest.update(block)
                            length += len(block)
                            compressed.write(block)
                    raw.flush()
                    os.fsync(raw.fileno())
                original_sha = digest.hexdigest()
                verify_gzip(temporary, original_sha, length)
                if identity(os.fstat(source.fileno())) != identity(before) \
                        or identity(regular(path)) != identity(before):
                    raise RuntimeError('Source changed during compression: ' + str(path))
                publish_no_replace(temporary, gz_path)
                temporary = None
            gzip_before = verify_gzip(gz_path, original_sha, length)
            gzip_sha, gzip_size = digest_file(gz_path)
            if identity(os.fstat(source.fileno())) != identity(before) \
                    or identity(regular(path)) != identity(before):
                raise RuntimeError('Source changed before archival commit: ' + str(path))
            record = {'event': 'archive_ready', 'path': entry['path'],
                      'gzip_path': gz_path.relative_to(RESULTS).as_posix(),
                      'bytes': length, 'sha256': original_sha, 'gzip_bytes': gzip_size,
                      'gzip_sha256': gzip_sha, 'original_stat': snapshot(before),
                      'gzip_stat': gzip_before, 'reused_existing_gzip': reused,
                      'compression_level': None if reused else level}
            # A fully verified recovery record must be durable BEFORE unlink.
            append(record)
            if identity(regular(path)) != identity(before) \
                    or identity(regular(gz_path)) != gzip_before:
                raise RuntimeError('File changed after durable record; original retained')
            path.unlink()
            fsync_directory(path.parent)
            append({'event': 'original_removed', 'path': entry['path'], 'sha256': original_sha})
            ready[entry['path']] = record
            return 'archived', length - gzip_size
        finally:
            if temporary is not None and temporary.exists():
                temporary.unlink()


def apply_command(args):
    plan_path = check_components(Path(args.plan).absolute(), ARCHIVES)
    with open_regular(plan_path) as stream:
        plan = json.load(stream)
    if plan.get('schema') != SCHEMA or plan.get('results_root') != str(RESULTS):
        raise ValueError('Plan schema/root mismatch')
    entries = plan['entries']
    if args.group:
        groups = set(selected_groups(args.group))
        entries = [e for e in entries if PurePosixPath(e['path']).parts[0] in groups]
    if args.max_files is not None:
        entries = entries[:args.max_files]
    counts, errors, logical_saved = {}, [], 0
    with operation_lock(), ledger_writer() as append:
        ready = load_ready_records()
        for index, entry in enumerate(entries, 1):
            try:
                status, saved = archive_one(entry, append, ready, args.level)
                counts[status] = counts.get(status, 0) + 1
                logical_saved += saved
            except Exception as exc:
                error = {'path': entry.get('path'), 'error': str(exc)}
                errors.append(error)
                emit({'action': 'file_failed_original_retained_if_present', **error})
            if index % 100 == 0:
                emit({'action': 'apply_progress', 'processed': index, 'total': len(entries),
                      'counts': counts, 'failed': len(errors)})
    emit({'action': 'apply_complete', 'counts': counts, 'failed': len(errors),
          'logical_bytes_reduced': logical_saved, 'manifest': str(MANIFEST),
          'note': 'Actual released blocks depend on remaining hardlinks.'})
    return 1 if errors else 0


def restore_one(record, append):
    path = checked_evidence(record['path'])
    gz_path = path.with_name(path.name + '.gz')
    if record.get('gzip_path') != gz_path.relative_to(RESULTS).as_posix():
        raise ValueError('Ledger gzip path mismatch')
    expected = record['sha256'], record['bytes']
    if not os.path.lexists(gz_path):
        if os.path.lexists(path) and digest_file(path) == expected:
            return 'already_restored'
        raise RuntimeError('Neither verified gzip nor matching original exists: ' + str(path))
    gzip_before = verify_gzip(gz_path, *expected)
    if digest_file(gz_path) != (record['gzip_sha256'], record['gzip_bytes']):
        raise RuntimeError('Compressed-file SHA/size differs from recovery ledger')
    temporary = None
    try:
        if os.path.lexists(path):
            if digest_file(path) != expected:
                raise RuntimeError('Existing original has different content; refusing: ' + str(path))
        else:
            fd, name = tempfile.mkstemp(prefix='.' + path.name + '.', suffix='.restore.partial',
                                        dir=str(path.parent))
            temporary = Path(name)
            digest, length = hashlib.sha256(), 0
            with os.fdopen(fd, 'wb') as restored, open_regular(gz_path) as raw:
                with gzip.GzipFile(fileobj=raw, mode='rb') as compressed:
                    while True:
                        block = compressed.read(CHUNK)
                        if not block:
                            break
                        digest.update(block)
                        length += len(block)
                        restored.write(block)
                restored.flush()
                os.fchmod(restored.fileno(), stat.S_IMODE(record['original_stat']['mode']))
                os.fsync(restored.fileno())
            if (digest.hexdigest(), length) != expected or digest_file(temporary) != expected:
                raise RuntimeError('Restored-file verification failed')
            os.utime(temporary, ns=(record['original_stat']['atime_ns'],
                                   record['original_stat']['mtime_ns']))
            publish_no_replace(temporary, path)
            temporary = None
        original_before = identity(regular(path))
        if digest_file(path) != expected:
            raise RuntimeError('Restored original SHA/size mismatch')
        append({'event': 'restore_ready', 'path': record['path'],
                'sha256': record['sha256'], 'bytes': record['bytes']})
        if identity(regular(path)) != original_before \
                or identity(regular(gz_path)) != gzip_before:
            raise RuntimeError('File changed before gzip removal; both retained')
        gz_path.unlink()
        fsync_directory(path.parent)
        append({'event': 'gzip_removed_after_restore', 'path': record['path'],
                'sha256': record['sha256']})
        return 'restored'
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def restore_command(args):
    groups = set(selected_groups(args.group))
    counts, failures = {}, 0
    with operation_lock(), ledger_writer() as append:
        records = [row for rel, row in sorted(load_ready_records().items())
                   if PurePosixPath(rel).parts[0] in groups]
        for index, record in enumerate(records, 1):
            try:
                status = restore_one(record, append)
                counts[status] = counts.get(status, 0) + 1
            except Exception as exc:
                failures += 1
                emit({'action': 'restore_failed', 'path': record['path'], 'error': str(exc)})
            if index % 100 == 0:
                emit({'action': 'restore_progress', 'processed': index, 'total': len(records),
                      'counts': counts, 'failed': failures})
    emit({'action': 'restore_complete', 'groups': sorted(groups),
          'counts': counts, 'failed': failures})
    return 1 if failures else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest='command', required=True)
    inventory = commands.add_parser('plan', help='Write metadata-only eligible-file inventory')
    inventory.add_argument('--group', action='append')
    inventory.add_argument('--output', default=str(DEFAULT_PLAN))
    inventory.set_defaults(function=plan_command)
    archive = commands.add_parser('apply', help='Verify, gzip and archive only saved-plan files')
    archive.add_argument('--plan', required=True)
    archive.add_argument('--group', action='append')
    archive.add_argument('--level', type=int, choices=(1, 3), default=1)
    archive.add_argument('--max-files', type=int)
    archive.set_defaults(function=apply_command)
    restore = commands.add_parser('restore', help='Restore original paths for explicit historical groups')
    restore.add_argument('--group', action='append', required=True)
    restore.set_defaults(function=restore_command)
    args = parser.parse_args()
    if getattr(args, 'max_files', None) is not None and args.max_files < 1:
        parser.error('--max-files must be positive')
    try:
        return args.function(args) or 0
    except Exception as exc:
        emit({'action': 'refused_or_failed', 'error': str(exc)})
        return 1


if __name__ == '__main__':
    sys.exit(main())
