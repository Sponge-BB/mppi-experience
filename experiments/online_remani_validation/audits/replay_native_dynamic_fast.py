#!/usr/bin/env python3
"""Same recorded inputs, seed-1 reset-state replay; no ROS or actuation.

The selected inputs cover real intervention and late infeasibility, not just
first/middle/last rows. This is an isolated compute/equivalence gate, never a
replacement for the new online closed-loop experiment. Profile calls are run
separately and are excluded from every runtime statistic.
"""
import argparse
from collections import OrderedDict
import ctypes as ct
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import replay_native_realtime as base
from native_collision_runtime import load_recorded_snapshot
from report_coupled_objective import lines, read

DEFAULT_BATCH = HERE/'results/native_realtime_formal_20261027_01'
DEFAULT_TRIALS = ('static_demand_seed11', 'crossing_demand_seed11',
                  'dynamic_head_on_center_demand_seed11')
VARIANTS = ('parent', 'off', 'on')


def joined_cycles(trial):
    """Only successful *recorded* solver calls, joined to their diagnostics."""
    cycles = [row for row in lines(Path(trial)/'state_mppi_cycles.jsonl')
              if row.get('online_step_succeeded')]
    diagnostics = {round(row['t']+1000., 8): row
                   for row in read(Path(trial)/'mppi_diagnostics.json', [])}
    if not cycles:
        raise ValueError('no successful recorded CABI inputs: '+str(trial))
    return [(ordinal, cycle, diagnostics[round(cycle['solve_ros_time'], 8)])
            for ordinal, cycle in enumerate(cycles)]


def choose_cycles(joined):
    """Deterministic semantic selection; retain all labels when rows coincide."""
    chosen = OrderedDict()

    def add(label, rows, last=False):
        if not rows:
            return
        row = rows[-1] if last else rows[0]
        key = int(row[1]['cycle_id'])
        if key not in chosen:
            chosen[key] = dict(ordinal=row[0], cycle_id=key, labels=[])
        chosen[key]['labels'].append(label)

    def mode(row):
        return int(row[2].get('local_phase', {}).get('decision', [0])[0])

    add('first_solver_input', joined)
    moving = [r for r in joined if r[2].get('obstacle_tracks', 0) > 0]
    add('first_observed_dynamic', moving)
    intervention = [r for r in joined if mode(r) in (1, 2)]
    add('first_intervention', intervention)
    add('last_intervention_validation_passed',
        [r for r in intervention if r[2].get('validation_code') == 0], last=True)
    infeasible = [r for r in joined if mode(r) == 4 or
                  r[2].get('reason') == 'no_feasible_candidate']
    add('first_no_feasible_candidate', infeasible)
    add('last_no_feasible_candidate', infeasible, last=True)
    if not moving:
        add('late_rejoin_or_cusp', [r for r in joined if mode(r) == 3], last=True)
    return sorted(chosen.values(), key=lambda item: item['ordinal'])


def check_state_identity(cycle, name):
    """Do not silently replace solve/validation state by a convenient state."""
    value = np.ascontiguousarray(cycle[name+'_state'], dtype=np.float64)
    identity = cycle.get(name+'_odom_identity')
    if value.shape != (5,) or not np.isfinite(value).all():
        raise ValueError(name+' state is incomplete or non-finite')
    if not identity or not np.array_equal(value, np.asarray(identity['state'])):
        raise ValueError(name+' state differs from its recorded odometry identity')
    return value


def recorded_inputs(trial, selection=None):
    """Generalize the old loader's identity checks to semantic cycle selection.

    The snapshot decoder/hash functions and CABI runner are reused unchanged.
    This loader changes which original rows are selected, not any input value.
    """
    trial = Path(trial).resolve()
    joined = joined_cycles(trial)
    selection = choose_cycles(joined) if selection is None else selection
    cycles = {int(row[1]['cycle_id']): row for row in joined}
    native = {round(row['bindings']['solve']['ros_time'], 8): row
              for row in lines(trial/'native_collision_cycles.jsonl')
              if 'solve' in row.get('bindings', {})}
    refs = {row['sha256']: row for row in lines(trial/'consumed_references.jsonl')}
    raw_root = trial/'native3d_snapshots'
    snapshots = {int(row['header'][2]): row for row in lines(raw_root/'index.jsonl')
                 if row['kind'] == 'snapshot'}
    shared_index = {int(row['sequence']): row for row in lines(trial/'collision_snapshots.jsonl')}
    result = []
    for chosen in selection:
        ordinal, cycle, diag = cycles[int(chosen['cycle_id'])]
        if ordinal != chosen['ordinal']:
            raise ValueError('cycle ordinal identity mismatch')
        now = float(cycle['solve_ros_time'])
        entry = native[round(now, 8)]
        reference = np.ascontiguousarray(refs[cycle['reference_array_sha256']]['reference'], dtype=np.float64)
        if base.digest(reference) != cycle['reference_array_sha256']:
            raise ValueError('reference identity differs from actual solver input')
        provenance = {}

        def payload(phase):
            binding = entry['bindings'][phase]
            header = binding['header']
            record = snapshots[int(header[2])]
            if record['header'][:20] != header[:20] or record['header'][21:] != header[21:]:
                raise ValueError('snapshot does not match the recorded '+phase+' binding')
            provenance[phase] = dict(sequence=int(header[2]), raw_sha256=record['raw_sha256'],
                map_sha256=record['map_sha256'], prefix_sha256=record['prefix_sha256'])
            return load_recorded_snapshot(raw_root, record, self_contained=True)

        seq = int(diag['solve_collision_sequence'])
        with np.load(trial/'collision_snapshots'/('%05d.npz' % seq), allow_pickle=False) as archive:
            shared = np.ascontiguousarray(archive['payload'])
        if base.digest(shared) != shared_index[seq]['payload_sha256']:
            raise ValueError('planar proposal frame hash mismatch')
        nb, no = int(shared[11]), int(shared[12])
        offset = 16+3*nb
        obs = np.ascontiguousarray(shared[offset:offset+6*no].reshape((-1, 6)))
        field = np.ascontiguousarray(shared[offset+6*no:].reshape((int(shared[4]), int(shared[3]))))
        if not np.array_equal(obs, np.asarray(diag['observed_obstacles']).reshape((-1, 6))):
            raise ValueError('proposal obstacle input differs from recorded solver input')
        validation_recorded = 'validation' in entry['bindings']
        if bool(cycle.get('validation_invoked')) != validation_recorded:
            raise ValueError('validation invocation/binding identity mismatch')
        sample = dict(cycle_id=cycle['cycle_id'], ordinal=ordinal, source=str(trial),
            labels=chosen['labels'], state=check_state_identity(cycle, 'solve'), reference=reference,
            validation_state=check_state_identity(cycle, 'validation'),
            native=payload('solve'), now=now, validation_recorded=validation_recorded,
            obs=obs, field=field, gx=float(shared[5]), gy=float(shared[6]), resolution=float(shared[7]))
        sample['age'] = max(0., now-sample['native'][18])
        if validation_recorded:
            sample['validation_native'] = payload('validation')
            sample['validation_now'] = entry['bindings']['validation']['ros_time']
            if sample['validation_now'] != cycle['validation_snapshot_ros_time']:
                raise ValueError('validation snapshot clock differs from the recorded cycle')
            sample['validation_state_age'] = max(0., sample['validation_now']-cycle['validation_odom_stamp'])
            sample['validation_age'] = max(0., sample['validation_now']-sample['validation_native'][18])
        else:
            provenance['validation'] = None
        sample['identities'] = {name: base.digest(sample[name]) for name in
            ('state', 'reference', 'validation_state', 'native', 'validation_native', 'obs', 'field') if name in sample}
        sample['identities']['clock'] = base.digest(np.array([now, sample['age'],
            cycle['validation_snapshot_ros_time'], cycle['validation_ros_time'], cycle['validation_odom_stamp'],
            sample.get('validation_age', 0.), sample.get('validation_state_age', 0.),
            sample['gx'], sample['gy'], sample['resolution']]))
        sample['provenance'] = dict(native=provenance, shared_sequence=seq,
            shared_payload_sha256=shared_index[seq]['payload_sha256'],
            observed_reason=diag['reason'], observed_validation_code=diag['validation_code'],
            observed_mode=int(diag['local_phase']['decision'][0]), validation_recorded=validation_recorded,
            note='all input arrays are original captured values; full ESDF reconstructed and hash-checked')
        result.append(sample)
    return result


def summarize_trace(values):
    """Keep every candidate's classification; discarded costs are not actions."""
    candidates = []
    for ident in np.unique(values[:, 0]) if len(values) else []:
        rows = values[values[:, 0] == ident]
        if not np.isfinite(rows).all() or not np.all(rows[:, 6:] == rows[0, 6:]):
            raise ValueError('malformed search trace')
        candidates.append(dict(id=int(ident), feasible=bool(rows[0, 6]), selected=bool(rows[0, 7]),
            rows=rows.tolist()))
    return candidates


class CaptureBeforeValidation:
    """Reuse the established runner, capture untimed debug before validation."""
    def __init__(self, lib):
        self.lib = lib
        self.extra = None
        lib.online_search_trace.argtypes = [ct.c_void_p, base.DP, ct.c_int]
        lib.online_search_trace.restype = ct.c_int
        lib.online_avoidance_debug.argtypes = [ct.c_void_p, base.DP]

    def __getattr__(self, name):
        return getattr(self.lib, name)

    def capture(self, handle):
        count = self.lib.online_search_trace(handle, None, 0)
        if not 0 <= count <= 200000:
            raise ValueError('invalid search trace length')
        values = np.zeros((count, 8), dtype=np.float64)
        if self.lib.online_search_trace(handle, base.ptr(values), count) != count:
            raise ValueError('search trace truncated')
        avoidance = np.zeros(8, dtype=np.float64)
        self.lib.online_avoidance_debug(handle, base.ptr(avoidance))
        self.extra = dict(search_candidates=summarize_trace(values), avoidance_debug=avoidance.tolist())

    def online_validate_execution(self, handle, *args):
        self.capture(handle)
        return self.lib.online_validate_execution(handle, *args)


def run_variant(lib, sample, params, geometry, fast, profile=False):
    os.environ['NATIVE_DYNAMIC_FAST'] = str(int(fast))
    capture = CaptureBeforeValidation(lib)
    if sample['validation_recorded']:
        record = base.run_one(capture, sample, params, geometry, demand=True, profile=profile)
    else:
        record = run_plan_only(capture, sample, params, geometry, profile)
    record.update(capture.extra)
    record['dynamic_fast'] = bool(fast)
    record['validation_recorded'] = sample['validation_recorded']
    return record


def run_plan_only(lib, sample, params, geometry, profile):
    """Replay a real no-feasible cycle that never invoked final validation.

    Do not substitute the solve snapshot for a nonexistent validation snapshot.
    The planning calls/reset/timer boundaries are the same as base.run_one.
    """
    os.environ.update(NATIVE_COLLISION_3D='1', NATIVE_REALTIME_DEMAND='1',
        NATIVE_REALTIME_PROFILE=str(int(profile)), NATIVE_REALTIME_REPLAY='1')
    handle = lib.online_create(base.ptr(params), base.ptr(geometry), len(geometry), 1)
    if not handle:
        raise ValueError('CABI creation failed')
    try:
        lib.online_reset(handle)
        lib.online_reference_changed(handle)
        if lib.online_set_native_collision_snapshot(handle, base.ptr(sample['native']), len(sample['native'])):
            raise ValueError('solve snapshot refused')
        if lib.online_set_native_collision_clock(handle, sample['now']):
            raise ValueError('solve clock refused')
        out = np.zeros(8, dtype=np.float64)
        begin, cpu = time.perf_counter_ns(), time.thread_time_ns()
        code = lib.online_step(handle, base.ptr(sample['state']), base.ptr(sample['reference']),
            len(sample['reference']), base.ptr(sample['obs']), len(sample['obs']), sample['age'],
            base.ptr(sample['field']), sample['field'].shape[1], sample['field'].shape[0],
            sample['gx'], sample['gy'], sample['resolution'], 0, base.ptr(out))
        elapsed, cpu = (time.perf_counter_ns()-begin)/1e6, (time.thread_time_ns()-cpu)/1e6
        if code != 0 or not np.isfinite(out).all():
            raise ValueError('offline plan failed: '+str(code))
        count = lib.online_selected_controls(handle, None, 0)
        controls = np.zeros((count, 2), dtype=np.float64)
        lib.online_selected_controls(handle, base.ptr(controls), count)
        diagnostics = {}
        for name, size in [('online_debug', 8), ('online_decision_debug', 12), ('online_recovery_debug', 10)]:
            values = np.zeros(size, dtype=np.float64)
            getattr(lib, name)(handle, base.ptr(values))
            diagnostics[name] = values.tolist()
        stats = None
        if hasattr(lib, 'online_native_realtime_profile'):
            values = np.zeros(len(base.PROFILE_NAMES), dtype=np.float64)
            if lib.online_native_realtime_profile(handle, base.ptr(values), len(values)) != len(values):
                raise ValueError('wrong profiling schema')
            stats = dict(zip(base.PROFILE_NAMES, values.tolist()))
        lib.capture(handle)
        return dict(code=int(code), action=out[:7].tolist(), controls=controls.tolist(), diagnostics=diagnostics,
            kernel_ms=float(out[7]), wall_ms=elapsed, thread_cpu_ms=cpu, profile=stats,
            validation_code=None, validation_report=None, input_sha256=sample['identities'], demand=True,
            profiling=profile)
    finally:
        lib.online_destroy(handle)


def core_output(record):
    keys = ('code', 'action', 'controls', 'diagnostics', 'validation_recorded', 'validation_code',
            'validation_report', 'avoidance_debug')
    return {key: record[key] for key in keys}


def compare_records(left, right, allow_pruned_cost=False):
    """Never ignore actions/controls/audit codes, even when both are infeasible."""
    checks = dict(same_inputs=left['input_sha256'] == right['input_sha256'],
                  core_output_exact=core_output(left) == core_output(right))
    lhs, rhs = left['search_candidates'], right['search_candidates']
    checks['candidate_classifications_exact'] = [(r['id'], r['feasible'], r['selected']) for r in lhs] == [
        (r['id'], r['feasible'], r['selected']) for r in rhs]
    traces_equal, ignored = len(lhs) == len(rhs), []
    for a, b in zip(lhs, rhs):
        av, bv = np.asarray(a['rows']), np.asarray(b['rows'])
        if av.shape != bv.shape:
            traces_equal = False
            continue
        if allow_pruned_cost and not a['feasible'] and not b['feasible']:
            if not np.array_equal(av[:, 4], bv[:, 4]):
                ignored.append(dict(candidate=a['id'], field='discarded_candidate_cost',
                    before=float(av[0, 4]), after=float(bv[0, 4])))
            av, bv = av[:, [0, 1, 2, 3, 5, 6, 7]], bv[:, [0, 1, 2, 3, 5, 6, 7]]
        traces_equal = traces_equal and np.array_equal(av, bv)
    checks['candidate_traces_exact_except_declared_pruned_cost'] = bool(traces_equal)
    return dict(passed=all(checks.values()), checks=checks, allowed_differences=ignored)


def trial_order(repeat):
    """Rotate starting variant and reverse alternate repetitions."""
    names = list(VARIANTS)
    shift = repeat % len(names)
    names = names[shift:]+names[:shift]
    return list(reversed(names)) if repeat % 2 else names


def timing_summary(records):
    return {name: {metric: dict(n=len(rows), p50=float(np.median([r[metric] for r in rows])),
        p95=float(np.percentile([r[metric] for r in rows], 95)),
        min=float(min(r[metric] for r in rows)), max=float(max(r[metric] for r in rows)))
        for metric in ('kernel_ms', 'wall_ms', 'thread_cpu_ms')}
        for name, rows in records.items()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch', type=Path, default=DEFAULT_BATCH)
    parser.add_argument('--trial', type=Path, action='append', help='override default three trial directories')
    parser.add_argument('--parent', type=Path, default=HERE/'builds/native_realtime_20261027_01')
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--warmup', type=int, default=1, help='unrecorded repetitions per sample/variant')
    parser.add_argument('--no-profile', action='store_true')
    args = parser.parse_args()
    if args.repeats < 1 or args.warmup < 0:
        parser.error('repeats must be positive; warmup must be nonnegative')
    if args.out.exists():
        raise FileExistsError('Fresh output directory required')
    trials = args.trial or [args.batch/name for name in DEFAULT_TRIALS]
    libraries = dict(parent=base.configure(args.parent/'build/libonline_mppi.so'),
                     child=base.configure(args.build/'build/libonline_mppi.so'))
    records, all_checks = [], []
    pooled = {name: [] for name in VARIANTS}
    for trial in trials:
        samples = recorded_inputs(trial)
        params = np.asarray(list(read(trial/'limits.json').values()), dtype=np.float64)
        geometry = np.ascontiguousarray(np.loadtxt(trial/'geometry.csv', delimiter=','), dtype=np.float64)
        for sample in samples:
            sample['identities']['controller_parameters'] = base.digest(params)
            sample['identities']['proposal_geometry'] = base.digest(geometry)
            def run(name, profile=False):
                return run_variant(libraries['parent' if name == 'parent' else 'child'],
                    sample, params, geometry, fast=name == 'on', profile=profile)

            for _ in range(args.warmup):
                for name in VARIANTS:
                    run(name)
            timed = {name: [] for name in VARIANTS}
            schedule = []
            for repeat in range(args.repeats):
                order = trial_order(repeat)
                schedule.append(order)
                for name in order:
                    value = run(name)
                    timed[name].append(value)
                    pooled[name].append(value)
            checks = []
            for repeat in range(args.repeats):
                checks.append(dict(kind='parent_equals_child_off', repeat=repeat,
                    **compare_records(timed['parent'][repeat], timed['off'][repeat])))
                checks.append(dict(kind='off_equals_on_except_discarded_cost', repeat=repeat,
                    **compare_records(timed['off'][repeat], timed['on'][repeat], allow_pruned_cost=True)))
            for name in VARIANTS:
                for repeat in range(1, args.repeats):
                    checks.append(dict(kind=name+'_reset_deterministic', repeat=repeat,
                        **compare_records(timed[name][0], timed[name][repeat])))
            profiles = {}
            if not args.no_profile:
                for name in VARIANTS:
                    profiles[name] = run(name, profile=True)
                    checks.append(dict(kind=name+'_profiling_output_invariant',
                        **compare_records(timed[name][0], profiles[name])))
            all_checks.extend(checks)
            record = dict(source=sample['source'], cycle_id=sample['cycle_id'], ordinal=sample['ordinal'],
                labels=sample['labels'], provenance=sample['provenance'], input_sha256=sample['identities'],
                timed_order=schedule, timed=timed, timing=timing_summary(timed), profiles=profiles, checks=checks)
            records.append(record)
            print(Path(sample['source']).name, sample['cycle_id'], ','.join(sample['labels']),
                {name: round(record['timing'][name]['kernel_ms']['p50'], 3) for name in VARIANTS},
                'passed', all(check['passed'] for check in checks), flush=True)
    result = dict(schema_version=1, passed=all(check['passed'] for check in all_checks), records=records,
        aggregate_sample_timing=timing_summary(pooled),
        scope='only this selected batch of exact recorded inputs; seed 1 reset per call; not a historical warm-state replay, navigation success rate, or closed-loop result',
        timing_scope='only unprofiled online_step calls; capture/validation outside timers; wall_ms includes CABI input construction, kernel_ms covers plan, neither is the full ROS callback',
        state_scope='reset loses historical controller state; original solve and validation state/snapshots are retained and independently hash-checked',
        equality_scope='parent/off strict; on may differ only in cost of already-infeasible discarded candidates; every action, selected control, candidate feasible class and validation output must match',
        environment=dict(NATIVE_COLLISION_3D='1', NATIVE_REALTIME_DEMAND='1', NATIVE_REALTIME_REPLAY='1',
            NATIVE_REALTIME_PROFILE='0 for all timing; separate 1 for profiling', NATIVE_DYNAMIC_FAST='parent/off 0, on 1'),
        repeats=args.repeats, warmup_per_sample_variant=args.warmup,
        libraries={name: dict(path=str(path.resolve()), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
            for name, path in [('parent', args.parent/'build/libonline_mppi.so'), ('child', args.build/'build/libonline_mppi.so')]},
        runner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    args.out.mkdir(parents=True, exist_ok=False)
    (args.out/'replay.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print('PASSED', result['passed'], 'samples', len(records), flush=True)
    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
