#!/usr/bin/env python3
"""Post-run shared-input provenance audit; never starts ROS or grants execution."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import numpy as np


NATIVE_LOG = re.compile(
    r'\[INFO\].*?\[[0-9.]+, ([0-9.]+)\]: '
    r'\[SharedCollision\] native_reference sequence=(\d+) input_age=([0-9.]+)')


def integer(value, low, high):
    return math.isfinite(float(value)) and low <= value <= high and int(value) == value


def validate_payload(payload, record, body, limits, config):
    """Validate archived transport against producer metadata and trial geometry."""
    a = np.asarray(payload)
    if a.dtype != np.dtype('float64') or a.ndim != 1 or len(a) < 16 or not np.isfinite(a).all():
        raise ValueError('malformed or nonfinite transport')
    if a[0] != 1 or not integer(a[1], 1, 10**9) or a[2] <= 0:
        raise ValueError('invalid version/sequence/stamp')
    if not all(integer(a[i], low, high) for i, low, high in
               ((3, 2, 2000), (4, 2, 2000), (11, 1, 16), (12, 0, 16))):
        raise ValueError('invalid map/body/obstacle counts')
    nx, ny, nb, no = map(int, a[[3, 4, 11, 12]])
    if len(a) != 16 + 3 * nb + 6 * no + nx * ny:
        raise ValueError('incomplete transport')
    if (nx, ny) != (300, 200) or not np.array_equal(a[5:8], [-7.5, -5., .05]):
        raise ValueError('trial grid geometry drift')
    if nb != len(body) or not np.array_equal(a[16:16 + 3 * nb].reshape((-1, 3)), body):
        raise ValueError('body cover drift')
    if a[15] not in (0., 1.) or bool(a[15]) != bool(record['healthy']):
        raise ValueError('health metadata drift')
    if a[1] != record['sequence'] or a[2] != record['stamp']:
        raise ValueError('producer sequence/stamp drift')
    expected = [config['margin'], config['comfort'], config['weight']]
    if not np.allclose(a[8:11], expected, rtol=0, atol=1e-12):
        raise ValueError('common cost configuration drift')
    if abs(a[8] - limits['margin']) > 1e-12 or a[8] < .10:
        raise ValueError('native/local hard margin drift')
    if not np.allclose(a[13:15], [limits['max_age'], limits['max_prediction']], rtol=0, atol=1e-12):
        raise ValueError('freshness/prediction contract drift')
    if not 0 < a[13] <= .35 or not 1 <= a[14] <= 4 or a[9] <= a[8] or a[10] <= 0:
        raise ValueError('invalid common cost/freshness bounds')
    obs = a[16 + 3 * nb:16 + 3 * nb + 6 * no].reshape((-1, 6))
    if len(obs) and (np.any(obs[:, 4] <= 0) or not np.isin(obs[:, 5], [0., 1.]).all()):
        raise ValueError('invalid obstacle shape/membership')
    if np.any(a[16 + 3 * nb + 6 * no:] < 0):
        raise ValueError('negative distance field')
    wanted = record.get('payload_sha256', '')
    if not re.fullmatch(r'[0-9a-f]{64}', wanted) or hashlib.sha256(a.tobytes()).hexdigest() != wanted:
        raise ValueError('transport SHA256 mismatch')
    return dict(sequence=int(a[1]), stamp=float(a[2]), healthy=bool(a[15]),
                max_age=float(a[13]), margin=float(a[8]), dynamic_tracks=no)


def age_summary(values):
    if not values:
        return None
    return dict(min=float(min(values)), median=float(np.median(values)),
                p95=float(np.percentile(values, 95)), max=float(max(values)))


def audit(out, write=True):
    out = Path(out).resolve()
    manifest = json.loads((out / 'manifest.json').read_text())
    # Do not audit an in-progress trial or race its growing record files.
    if not (out / 'result.json').is_file() or not (out / 'run_status.json').is_file():
        raise RuntimeError('post-run audit requires result.json and run_status.json')
    status = json.loads((out / 'run_status.json').read_text())
    if not status.get('result_written', False):
        raise RuntimeError('trial has no completed result')
    report = dict(applicable=bool(manifest.get('shared_collision', False)),
                  scope='Archived input geometry/provenance/freshness only; not a collision replay, safety certificate, real-time guarantee, or controller-comparison outcome.')
    if not report['applicable']:
        report.update(status='not_applicable', passed=None,
                      reason='Trial does not use the shared collision interface.')
        if write:
            (out / 'shared_collision_audit.json').write_text(json.dumps(report, indent=2) + '\n')
        return report
    limits = json.loads((out / 'limits.json').read_text())
    config = json.loads((out / 'shared_collision_config.json').read_text())
    body = np.atleast_2d(np.loadtxt(out / 'geometry.csv', delimiter=','))
    errors, warnings, records, models = [], [], {}, {}
    producer_errors = []
    for line, text in enumerate((out / 'collision_snapshots.jsonl').read_text().splitlines(), 1):
        try:
            record = json.loads(text)
            if 'sequence' not in record:
                producer_errors.append(record)
                continue
            seq = record['sequence']
            if not integer(seq, 1, 10**9):
                raise ValueError('invalid producer sequence')
            seq = int(seq)
            if seq in records or (records and seq <= max(records)):
                raise ValueError('nonmonotonic producer sequence')
            records[seq] = record
            with np.load(out / 'collision_snapshots' / ('%05d.npz' % seq), allow_pickle=False) as archive:
                models[seq] = validate_payload(archive['payload'], record, body, limits, config)
        except (ValueError, KeyError, TypeError, OSError) as exc:
            errors.append(dict(check='provider_transport', line=line, error=str(exc)))
    native = []
    # ROS's latest symlink must not double-count the same physical log.
    log_paths = sorted(set(p.resolve() for p in (out / 'roslogs').glob('*/remani_planner_node-*-stdout.log')))
    for log in log_paths:
        for line, text in enumerate(log.read_text(errors='replace').splitlines(), 1):
            match = NATIVE_LOG.search(text)
            if not match:
                continue
            when, seq, printed_age = float(match[1]), int(match[2]), float(match[3])
            record, model = records.get(seq), models.get(seq)
            item = dict(sequence=seq, publication_t=when - 1000., printed_input_age=printed_age,
                        log=str(log.relative_to(out)), line=line)
            native.append(item)
            if record is None or model is None:
                errors.append(dict(check='native_reference', **item, error='unknown or invalid producer snapshot'))
                continue
            age = when - model['stamp']
            item.update(exact_input_age=age, producer_healthy=model['healthy'])
            if not model['healthy'] or age < -.05 or abs(age - printed_age) > .000501:
                errors.append(dict(check='native_reference', **item, error='health/time/log-age mismatch'))
    if not native:
        warnings.append('No native shared-reference publication was observed; native hook delivery coverage is absent.')
    executions, approvals, ages = 0, 0, []
    execution_path = out / 'execution_rollouts.jsonl'
    if execution_path.is_file():
        for line, text in enumerate(execution_path.read_text().splitlines(), 1):
            try:
                row = json.loads(text)
                executions += 1
                if row.get('reason') != 'ok' or not row.get('validation', [0])[0]:
                    continue
                approvals += 1
                seq = row.get('shared_collision_sequence')
                if not integer(seq, 1, 10**9):
                    raise ValueError('approved execution has invalid sequence')
                model = models.get(int(seq))
                if model is None or not model['healthy'] or row['cloud_stamp'] != model['stamp']:
                    raise ValueError('approved execution uses unknown/unhealthy/mismatched snapshot')
                age = row['t'] + 1000. - row['cloud_stamp']
                ages.append(age)
                if not -.05 <= age <= model['max_age'] + 1e-9:
                    raise ValueError('approved execution uses stale/future snapshot')
                command = np.asarray(row.get('command', []), dtype=float)
                validation = np.asarray(row['validation'], dtype=float)
                if (not row.get('rollout') or command.shape != (2,) or not np.isfinite(command).all()
                        or validation.shape != (6,) or not np.isfinite(validation).all() or validation[0] != 1.):
                    raise ValueError('approved execution lacks finite command or validated rollout')
            except (ValueError, KeyError, TypeError, IndexError) as exc:
                errors.append(dict(check='local_execution_input', line=line, error=str(exc)))
        execution_scope = ('Input provenance/freshness for approved local rollouts; '
                           'does not independently recalculate rollout collisions or braking dynamics.')
        if not approvals:
            warnings.append('No approved local execution was observed; execution coverage is absent.')
    else:
        execution_scope = 'Native-only trial has no local execution records; local execution safety is NOT verified.'
    native_ages = [v['exact_input_age'] for v in native if 'exact_input_age' in v]
    report.update(passed=not errors, status='checked', run_status=status,
                  producer_snapshots=len(records), validated_transports=len(models),
                  producer_error_events=len(producer_errors), provider_error_samples=producer_errors[:5],
                  native_reference_segments=len(native), native_unique_snapshots=len(set(v['sequence'] for v in native)),
                  native_input_age_s=age_summary(native_ages),
                  native_segments_over_max_age=sum(v > limits['max_age'] for v in native_ages),
                  native_age_policy='Frozen optimization advisory may be older than max_age at publication; this is reported, not silently rejected or called latest-frame execution approval.',
                  native_reference_records=native,
                  local_execution_records=executions, local_approved_records=approvals,
                  local_input_age_s=age_summary(ages), local_execution_scope=execution_scope,
                  body_sha256=hashlib.sha256(np.ascontiguousarray(body).tobytes()).hexdigest(),
                  common_hard_margin_m=limits['margin'], warnings=warnings, errors=errors)
    if write:
        (out / 'shared_collision_audit.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def self_test():
    body = np.array([[0., 0., .3]])
    limits = dict(margin=.1, max_age=.35, max_prediction=3.)
    config = dict(margin=.1, comfort=.12, weight=50000.)
    header = [1., 1., 1000., 300., 200., -7.5, -5., .05, .1, .12, 50000., 1., 0., .35, 3., 1.]
    payload = np.concatenate([header, body.ravel(), np.ones(60000)])
    record = dict(sequence=1, stamp=1000., healthy=True,
                  payload_sha256=hashlib.sha256(payload.tobytes()).hexdigest())
    assert validate_payload(payload, record, body, limits, config)['healthy']
    for index, value in ((1, 1.5), (2, 1001.), (8, .11), (15, 0.), (18, .4), (-1, -1.)):
        changed = payload.copy(); changed[index] = value
        try:
            validate_payload(changed, record, body, limits, config)
        except ValueError:
            pass
        else:
            raise AssertionError('accepted invalid payload at index %d' % index)
    bad = dict(record, payload_sha256='0' * 64)
    try:
        validate_payload(payload, bad, body, limits, config)
    except ValueError:
        pass
    else:
        raise AssertionError('accepted altered digest')
    assert age_summary([]) is None
    assert age_summary([.1, .2])['max'] == .2
    return dict(passed=True, invalid_payload_cases=7)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('out', type=Path, nargs='?')
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        print(json.dumps(self_test(), indent=2))
    elif args.out is not None:
        result = audit(args.out)
        print(json.dumps({k: result[k] for k in ('applicable', 'status', 'passed')}, indent=2))
        if result.get('passed') is False:
            raise SystemExit(1)
    else:
        parser.error('provide a completed trial directory or --self-test')
