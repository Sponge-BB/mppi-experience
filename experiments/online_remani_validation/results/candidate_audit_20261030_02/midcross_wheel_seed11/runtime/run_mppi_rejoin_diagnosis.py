#!/usr/bin/env python3
"""Isolate fixed-native-reference MPPI recovery; no ROS or robot publication.

The C API kernel and publication validator are the unchanged V12 binaries.
World states/obstacles are ideal measurements. This is a synchronous numerical
diagnosis, NOT native online replanning, Gazebo, perception or a hardware test.
"""
import argparse
import ast
import collections
import ctypes as ct
import hashlib
import json
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import numpy as np

from dynamic_scenarios import CASES, body_clearance
from native_reference import reference_polynomial, sampled_reference, fingerprint

HERE = Path(__file__).resolve().parent
DP = ct.POINTER(ct.c_double)
PARAMETERS = ('vmax', 'wmax', 'accel', 'angular_accel', 'track', 'wheel_v',
              'wheel_a', 'margin', 'max_age', 'max_prediction', 'pos_unc',
              'vel_unc', 'acc_unc', 'goal_pos', 'goal_yaw', 'goal_speed')
DEFAULT_CASES = ('native_turn', 'offset_p010', 'offset_n010', 'offset_p020',
                 'offset_n020', 'offset_p030', 'offset_n030', 'static_rejoin',
                 'dynamic_crossing')
CRITERIA = dict(rejoin_distance_m=.05, rejoin_heading_rad=.15,
                rejoin_continuous_s=2., endpoint_distance_m=.05,
                endpoint_heading_rad=.1, endpoint_speed_mps=.05,
                endpoint_continuous_s=1., physical_collision_gap_m=0.,
                deadline_ms=50., alternative_deadline_ms=45.,
                max_duration_s=55., static_appearance_s=4.,
                static_reference_arclength_fraction=.62,
                static_radius_m=.20, dynamic_appearance_s=2.,
                dynamic_reference_arclength_fraction=.60,
                dynamic_crossing_s=9., dynamic_velocity_mps=.24,
                dynamic_radius_m=.20)


def ptr(array):
    return array.ctypes.data_as(DP)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False,
                               allow_nan=False)+'\n')


def extract_plant():
    """Compile the exact Simulator.integrate AST without importing ROS modules."""
    path = HERE/'simulator.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Simulator')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'integrate')
    source = ast.get_source_segment(path.read_text(), method)
    namespace = dict(np=np, math=math, CASES=CASES, body_clearance=body_clearance)
    unit = ast.Module(body=[method], type_ignores=[])
    exec(compile(ast.fix_missing_locations(unit), str(path), 'exec'), namespace)
    return namespace['integrate'], hashlib.sha256(source.encode()).hexdigest()


def load_reference(source):
    """Use the entire last native replacement transaction, including reversals."""
    messages = json.loads((source/'trajectories.json').read_text())
    starts = [i for i, m in enumerate(messages) if m['action'] == 1 and m['id'] == 1]
    if len(starts) < 2:
        raise ValueError('Require a native replacement after the initial reference')
    first = starts[-1]
    bundle = [m for m in messages[first:] if m['action'] == 1]
    if [m['id'] for m in bundle] != list(range(1, len(bundle)+1)):
        raise ValueError('Incomplete/out-of-order native reference bundle')
    if len({m['stamp'] for m in bundle}) != 1:
        raise ValueError('Native bundle start stamps differ')
    references, clocks = [], []
    arm_expected = np.deg2rad(json.loads((source/'geometry.json').read_text())['arm_q_deg'])
    for m in bundle:
        for piece in m['polynomials']:
            if (piece['num_dim'] != 8 or piece['num_order'] != 7 or
                    piece['duration'] <= 0 or len(piece['data']) != 64 or
                    not np.isfinite(piece['data']).all()):
                raise ValueError('Unexpected native polynomial shape')
            coefficients = np.asarray(piece['data']).reshape((8, 8), order='F')
            ts = np.linspace(0., piece['duration'], max(2, int(piece['duration']/.04)+1))
            arm = np.array([np.polyval(c, ts) for c in coefficients[2:]])
            if np.max(abs(arm-arm_expected[:, None])) > 1e-4:
                raise ValueError('Native reference arm differs from fixed-arm geometry')
        msg = SimpleNamespace(singul=m['singul'], trajectory=[SimpleNamespace(**p) for p in m['polynomials']])
        rows = reference_polynomial(msg, step=.04)
        references.append(rows[:, 1:5])
        clocks.append(rows[:, 0]+(clocks[-1][-1] if clocks else 0.))
    states = np.genfromtxt(source/'state.csv', names=True, delimiter=',')
    index = int(np.argmin(abs(states['t']-messages[first]['t'])))
    initial = np.array([states[index][name] for name in ('x', 'y', 'yaw', 'v', 'w')])
    ref = sampled_reference(references, initial[2])
    times = np.concatenate(clocks)
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(ref[:, :2], axis=0), axis=1))]
    if not np.isfinite(ref).all() or len(ref) < 3 or arc[-1] <= 0:
        raise ValueError('Invalid/degenerate native reference')
    return ref, times, arc, initial, bundle


def project_continuous(state, reference, times, arcs, previous):
    """Read-only local continuous, same-body-heading projection (not controller)."""
    edges = np.diff(reference[:, :2], axis=0)
    length2 = np.einsum('ij,ij->i', edges, edges)
    fraction = np.clip(np.einsum('ij,ij->i', state[:2]-reference[:-1, :2], edges)
                       / np.maximum(1e-15, length2), 0., 1.)
    nearest = reference[:-1, :2]+fraction[:, None]*edges
    distances = np.linalg.norm(nearest-state[:2], axis=1)
    target_times = times[:-1]+fraction*np.diff(times)
    heading_delta = np.arctan2(np.sin(reference[1:, 2]-reference[:-1, 2]),
                               np.cos(reference[1:, 2]-reference[:-1, 2]))
    headings = reference[:-1, 2]+fraction*heading_delta
    heading_error = np.arctan2(np.sin(headings-state[2]), np.cos(headings-state[2]))
    # Do not "recover" by jumping from a reverse segment to a nearby forward
    # segment merely because their geometric projections overlap. A phase
    # transition requires reaching the existing segment endpoint in space/time.
    signs = np.sign(reference[:, 3])
    signs[signs == 0.] = 1.
    current = min(len(reference)-2, max(0, int(np.searchsorted(times, previous, side='right')-1)))
    later = np.flatnonzero(signs[current+1:] != signs[current])
    phase_end = current+1+int(later[0]) if len(later) else len(reference)-1
    allow_next = (phase_end < len(reference)-1 and previous >= times[phase_end]-.3
                  and np.linalg.norm(state[:2]-reference[phase_end, :2]) < .08)
    if allow_next:
        later = np.flatnonzero(signs[phase_end+1:] != signs[phase_end])
        phase_end = phase_end+1+int(later[0]) if len(later) else len(reference)-1
    eligible = ((target_times >= max(0., previous-.3)) & (target_times <= previous+2.)
                & (length2 > 1e-12) & (np.abs(heading_error) < math.pi/2))
    eligible &= np.arange(len(edges)) < phase_end
    if not np.any(eligible):
        return dict(valid=False, time=previous, s=float(np.interp(previous, times, arcs)),
                    error=None, lateral=None, yaw_error=None, point=None)
    i = int(np.argmin(np.where(eligible, distances+1e-7*np.abs(target_times-previous), np.inf)))
    delta = nearest[i]-state[:2]
    lateral = -math.sin(state[2])*delta[0]+math.cos(state[2])*delta[1]
    return dict(valid=True, time=float(target_times[i]),
                s=float(arcs[i]+fraction[i]*(arcs[i+1]-arcs[i])),
                error=float(distances[i]), lateral=float(lateral),
                yaw_error=float(heading_error[i]), point=nearest[i].tolist())


def project_geometric(state, reference, times, arcs):
    """Nearest same-heading segment, independent of clock/phase bookkeeping.

    A local controller may safely skip an old reverse maneuver and join the
    forward segment. Keep this error separate rather than falsely calling a
    conservative diagnostic progress estimate an execution freeze.
    """
    edges = np.diff(reference[:, :2], axis=0)
    length2 = np.einsum('ij,ij->i', edges, edges)
    fraction = np.clip(np.einsum('ij,ij->i', state[:2]-reference[:-1, :2], edges)
                       / np.maximum(1e-15, length2), 0., 1.)
    nearest = reference[:-1, :2]+fraction[:, None]*edges
    delta = nearest-state[:2]
    distance = np.linalg.norm(delta, axis=1)
    headings = reference[:-1, 2]+fraction*np.arctan2(
        np.sin(np.diff(reference[:, 2])), np.cos(np.diff(reference[:, 2])))
    yaw_error = np.arctan2(np.sin(headings-state[2]), np.cos(headings-state[2]))
    eligible = (length2 > 1e-12) & (abs(yaw_error) < math.pi/2)
    if not np.any(eligible):
        return dict(valid=False, error=None, yaw_error=None, s=0., point=None)
    i = int(np.argmin(np.where(eligible, distance, np.inf)))
    return dict(valid=True, error=float(distance[i]), yaw_error=float(yaw_error[i]),
                s=float(arcs[i]+fraction[i]*np.diff(arcs)[i]), point=nearest[i].tolist())


class World:
    gx, gy, res, nx, ny = -7.5, -5., .05, 300, 200

    def __init__(self, name, reference, arcs):
        self.name = name
        self.xx = self.gx+np.arange(self.nx)[None, :]*self.res
        self.yy = self.gy+np.arange(self.ny)[:, None]*self.res
        # Same arena and conservative half-cell diagonal allowance as unit SIL.
        self.wall_field = np.ascontiguousarray(np.maximum(0., np.minimum(
            2.-abs(self.yy), np.minimum(self.xx+1.5, 4.5-self.xx))-.071))
        fraction = CRITERIA['static_reference_arclength_fraction'] if name == 'static_rejoin' else CRITERIA['dynamic_reference_arclength_fraction']
        index = min(len(reference)-2, int(np.searchsorted(arcs, fraction*arcs[-1])))
        self.center = reference[index, :2].copy()
        edge = reference[index+1, :2]-reference[index, :2]
        self.tangent = edge/max(1e-12, np.linalg.norm(edge))
        self.normal = np.array([-self.tangent[1], self.tangent[0]])
        self.actor_s = float(arcs[index])
        self.static_field = np.ascontiguousarray(np.maximum(0., np.minimum(
            self.wall_field, np.hypot(self.xx-self.center[0], self.yy-self.center[1])
            -CRITERIA['static_radius_m']-.071)))

    def actors(self, t):
        if self.name == 'static_rejoin' and t >= CRITERIA['static_appearance_s']:
            return [(*self.center, CRITERIA['static_radius_m'], 1, 0., 0.)]
        if self.name == 'dynamic_crossing' and t >= CRITERIA['dynamic_appearance_s']:
            velocity = self.normal*CRITERIA['dynamic_velocity_mps']
            center = self.center+(t-CRITERIA['dynamic_crossing_s'])*velocity
            return [(*center, CRITERIA['dynamic_radius_m'], 1, *velocity)]
        return []

    def inputs(self, t):
        actors = self.actors(t)
        obs = [[x-.04*vx, y-.04*vy, vx, vy, r, 1.] for x, y, r, _, vx, vy in actors
               if self.name == 'dynamic_crossing']
        field = self.static_field if actors and self.name == 'static_rejoin' else self.wall_field
        return field, np.ascontiguousarray(obs, dtype=float).reshape(-1, 6)

    def static_rejoin_certificate(self, reference, arcs, body, margin):
        """Reject a geometrically impossible success predicate before solving."""
        reach = float(np.max(np.linalg.norm(body[:, :2], axis=1)+body[:, 2]))
        required = self.actor_s+CRITERIA['static_radius_m']+reach+.1
        actors = self.actors(CRITERIA['static_appearance_s'])
        gaps = np.array([body_clearance(np.r_[row[:3], 0., 0.], body, actors) for row in reference])
        safe = (arcs > required) & (gaps >= margin)
        indices = np.flatnonzero(safe)
        if not (required < arcs[-1] and gaps[-1] >= margin and len(indices) >= 2):
            raise ValueError('Static rejoin predicate/endpoint has no verified safe reference segment')
        suffix_start = len(reference)-1
        while suffix_start > 0 and safe[suffix_start-1]:
            suffix_start -= 1
        return dict(required_arclength_m=required, total_arclength_m=float(arcs[-1]),
                    physical_endpoint_gap_m=float(gaps[-1]), safe_reference_samples=int(len(indices)),
                    contiguous_safe_suffix_arclength_m=float(arcs[-1]-arcs[suffix_start]),
                    robot_cover_reach_m=reach,
                    scope='Discrete full-body projected-disk geometric check of arena and static actor, not a path-existence proof.')


class Kernel:
    def __init__(self, build, body, limits, seed):
        self.lib = ct.CDLL(str(build/'build/libonline_mppi.so'))
        lib = self.lib
        lib.online_create.argtypes = [DP, DP, ct.c_int, ct.c_int]
        lib.online_create.restype = ct.c_void_p
        lib.online_step.argtypes = [ct.c_void_p, DP, DP, ct.c_int, DP, ct.c_int, ct.c_double,
                                   DP, ct.c_int, ct.c_int, ct.c_double, ct.c_double, ct.c_double, ct.c_int, DP]
        lib.online_step.restype = ct.c_int
        lib.online_selected_controls.argtypes = [ct.c_void_p, DP, ct.c_int]
        lib.online_selected_controls.restype = ct.c_int
        lib.online_execution_candidates.argtypes = [ct.c_void_p, DP, ct.c_int, ct.c_int, DP]
        lib.online_execution_candidates.restype = ct.c_int
        lib.online_accept_execution_controls.argtypes = [ct.c_void_p, DP, ct.c_int]
        lib.online_accept_execution_controls.restype = ct.c_int
        lib.online_validate_execution.argtypes = [ct.c_void_p, DP, DP, ct.c_int, DP, ct.c_int,
            ct.c_double, ct.c_double, DP, ct.c_int, ct.c_int, ct.c_double, ct.c_double, ct.c_double, DP, ct.c_int, DP]
        lib.online_validate_execution.restype = ct.c_int
        lib.online_feedback.argtypes = [ct.c_void_p, ct.c_double, ct.c_double]
        lib.online_destroy.argtypes = [ct.c_void_p]
        lib.online_reset.argtypes = [ct.c_void_p]
        lib.online_reference_changed.argtypes = [ct.c_void_p]
        lib.online_search_trace.argtypes = [ct.c_void_p, DP, ct.c_int]
        lib.online_search_trace.restype = ct.c_int
        lib.online_decision_debug.argtypes = [ct.c_void_p, DP]
        lib.online_recovery_debug.argtypes = [ct.c_void_p, DP]
        values = np.asarray([limits[k] for k in PARAMETERS], dtype=float)
        self.handle = lib.online_create(ptr(values), ptr(body), len(body), seed)
        if not self.handle:
            raise RuntimeError('Frozen V12 configuration rejected')
        lib.online_reset(self.handle)
        lib.online_reference_changed(self.handle)

    def solve_validate(self, state, reference, obs, field):
        start = time.monotonic()
        result, report = np.zeros(8), np.zeros(6)
        checked = np.zeros((650, 8))
        lib, handle = self.lib, self.handle
        code = lib.online_step(handle, ptr(state), ptr(reference), len(reference), ptr(obs), len(obs),
                               .04, ptr(field), World.nx, World.ny, World.gx, World.gy, World.res, 0, ptr(result))
        reason = 'kernel_error' if code or not np.isfinite(result).all() else 'no_feasible_candidate' if result[6] else 'ok'
        candidates, attempts, rank_accepted, valid_count = [], [], -1, 0
        if reason == 'ok':
            count = lib.online_selected_controls(handle, None, 0)
            controls = np.zeros((count, 2))
            lib.online_selected_controls(handle, ptr(controls), count)
            if count < 1 or not np.allclose(controls[0], result[:2], rtol=0., atol=1e-9):
                reason = 'selected_control_mismatch'
            else:
                pool, lengths = np.zeros((8, 64, 2)), np.zeros(8)
                n = lib.online_execution_candidates(handle, ptr(pool), 8, 64, ptr(lengths))
                if n < 1 or int(lengths[0]) != count or not np.array_equal(pool[0, :count], controls):
                    reason = 'candidate_export_mismatch'
                else:
                    candidates = [np.ascontiguousarray(pool[i, :int(lengths[i])]) for i in range(n)]
        if reason == 'ok':
            reason = 'local_validation_failed'
            for rank, candidate in enumerate(candidates):
                if rank > 0 and (time.monotonic()-start)*1000 > CRITERIA['alternative_deadline_ms']:
                    break
                check_code = lib.online_validate_execution(handle, ptr(state), ptr(candidate), len(candidate),
                    ptr(obs), len(obs), .04, 0., ptr(field), World.nx, World.ny, World.gx, World.gy, World.res,
                    ptr(checked), len(checked), ptr(report))
                attempts.append(dict(candidate=rank, code=int(check_code)))
                if check_code:
                    continue
                result[:2] = candidate[0]
                rank_accepted, valid_count, reason = rank, int(report[3]), 'ok'
                if lib.online_accept_execution_controls(handle, ptr(candidate), len(candidate)):
                    reason, valid_count = 'candidate_adoption_failed', 0
                break
        wall_ms = (time.monotonic()-start)*1000
        pre_deadline_reason = reason
        if wall_ms > CRITERIA['deadline_ms']:
            reason = 'deadline_miss'
        issued = result[:2].copy() if reason == 'ok' else np.zeros(2)
        if reason != 'ok':
            valid_count = 0
        lib.online_feedback(handle, *issued)  # Exactly once, only issued command.
        decision, recovery = np.zeros(12), np.zeros(10)
        lib.online_decision_debug(handle, ptr(decision))
        lib.online_recovery_debug(handle, ptr(recovery))
        return dict(command=issued, reason=reason, active_zero=bool(reason == 'ok' and np.max(abs(issued)) < 1e-6),
                    pre_deadline_reason=pre_deadline_reason, candidate_count=len(candidates),
                    kernel_no_feasible=bool(code == 0 and result[6]),
                    kernel_code=int(code), kernel_ms=float(result[7]), callback_ms=wall_ms,
                    attempts=attempts, accepted_candidate=rank_accepted, validation=report.tolist(),
                    route=checked[:valid_count].copy(), decision=decision.tolist(), recovery=recovery.tolist())

    def search(self, max_candidates=24):
        start = time.monotonic()
        n = self.lib.online_search_trace(self.handle, None, 0)
        rows = np.zeros((n, 8))
        self.lib.online_search_trace(self.handle, ptr(rows), n)
        all_count = len(np.unique(rows[:, 0])) if len(rows) else 0
        if all_count > max_candidates:
            ids = np.unique(rows[:, 0])
            # Read-only visualization subset, never a search/command change.
            selected = np.unique(rows[rows[:, 7] > .5, 0]).tolist()
            representative = [rows[np.flatnonzero(rows[:, 0] == i)[0]] for i in ids]
            ranked = sorted(representative, key=lambda r: (not bool(r[6]), r[4]))
            keep = list(selected)
            for row in ranked[:max_candidates//2]:
                if row[0] not in keep:
                    keep.append(row[0])
            for i in ids[np.linspace(0, len(ids)-1, max_candidates, dtype=int)]:
                if i not in keep:
                    keep.append(i)
                if len(keep) >= max_candidates:
                    break
            rows = rows[np.isin(rows[:, 0], keep)]
        return rows, all_count, (time.monotonic()-start)*1000

    def close(self):
        self.lib.online_destroy(self.handle)


def trial(build, source, out, case, seed=1, duration=55., search_stride=5,
          kernel_type=None, experiment_context=None):
    out.mkdir(parents=True, exist_ok=False)
    reference, times, arcs, initial, bundle = load_reference(source)
    limits = json.loads((source/'limits.json').read_text())
    body = np.ascontiguousarray(np.atleast_2d(np.loadtxt(source/'geometry.csv', delimiter=',')))
    if set(limits) != set(PARAMETERS):
        raise ValueError('C API parameter set drift')
    integrate, plant_hash = extract_plant()
    state = initial.copy()
    moved = np.flatnonzero(np.linalg.norm(reference[:, :2]-reference[0, :2], axis=1) > 1e-4)
    tangent = reference[moved[0], :2]-reference[0, :2]
    tangent /= np.linalg.norm(tangent)
    offset = 0.
    if case.startswith('offset_'):
        sign = 1. if case[7] == 'p' else -1.
        offset = sign*int(case[8:])/100.
        state[:2] += offset*np.array([-tangent[1], tangent[0]])
    world = World(case, reference, arcs)
    certificate = world.static_rejoin_certificate(reference, arcs, body, limits['margin']) if case == 'static_rejoin' else None
    initial_gap = body_clearance(state, body, world.actors(0.))
    if initial_gap <= limits['margin']:
        raise ValueError('Initial state lacks physical safety margin')
    plant = SimpleNamespace(s=state, command=np.zeros(2), command_time=0., command_lease=.075,
                            limits=limits, lock=threading.Lock(), scene='fixed_reference_diagnosis', t=0.)
    kernel = (kernel_type or Kernel)(build, body, limits, seed)
    for name in ('geometry.csv', 'geometry.json', 'limits.json'):
        (out/name).write_bytes((source/name).read_bytes())
    np.savetxt(out/'reference.csv', np.column_stack((times, reference, arcs)), delimiter=',',
               header='t,x,y,yaw,v,s', comments='')
    write_json(out/'reference_bundle.json', bundle)
    lib_path = build/'build/libonline_mppi.so'
    metadata = dict(case=case, seed=seed, initial_state=state.tolist(), initial_offset_m=offset,
        reference_source=str(source), reference_bundle_ids=[m['id'] for m in bundle],
        reference_sha256=fingerprint(reference), reference_messages_sha256=sha(source/'trajectories.json'),
        library=str(lib_path), library_sha256=sha(lib_path), geometry=json.loads((source/'geometry.json').read_text()),
        geometry_sha256=sha(source/'geometry.csv'), limits=limits, plant_method_sha256=plant_hash,
        criteria=CRITERIA, duration_s=duration, dt_s=.05, plant_substep_s=.005,
        obstacle_center=world.center.tolist(), obstacle_reference_arclength_m=world.actor_s,
        initial_physical_gap_m=initial_gap, search_stride=search_stride,
        static_rejoin_certificate=certificate,
        search_visualization='Read-only trace every stride cycles; at most 24 candidate trajectories, including selected, low-cost and uniformly sampled IDs; not every MPPI sample.',
        scope='Unchanged frozen V12 MPPI and execution validator; full fixed-arm projected disks; '
              'entire accepted native polynomial bundle held fixed. Ideal synchronous observations, '
              'no ROS, no detector, no native online replan, no Gazebo, no vehicle publication. '
              'World advances in fixed simulated steps, not real wall time; observation age .04s '
              'and measured-state age 0s are controlled fixtures. Actual 50ms decision deadline '
              'is enforced, but host timing is not ROS real-time performance. Post-command search '
              'export/render/log overhead excluded from deadline and separately recorded.')
    if experiment_context is not None:
        metadata['experiment_context'] = experiment_context
        metadata['scope'] = experiment_context['scope']
    write_json(out/'metadata.json', metadata)
    files = {name: (out/(name+'.jsonl')).open('w') for name in
             ('selected_rollouts', 'obstacles', 'search_trace', 'decisions')}
    rows, previous_time, minimum_gap = [], 0., initial_gap
    dwell, first_rejoin, endpoint_dwell, goal_time = 0., None, 0., None
    counts, validation_counts, pre_deadline_counts = collections.Counter(), collections.Counter(), collections.Counter()
    rejoin_samples = []
    start_wall = time.monotonic()
    try:
        for k in range(int(math.ceil(duration/.05))):
            t = k*.05
            plant.t = t
            field, obs = world.inputs(t)
            solved = kernel.solve_validate(plant.s, reference, obs, field)
            projection = project_continuous(plant.s, reference, times, arcs, previous_time)
            geometric = project_geometric(plant.s, reference, times, arcs)
            if projection['valid']:
                previous_time = projection['time']
            cmd = solved['command']
            half_track = .5*limits['track']
            command_bound_ratio = max(abs(cmd[0])/limits['vmax'], abs(cmd[1])/limits['wmax'],
                abs(cmd[0]-half_track*cmd[1])/limits['wheel_v'],
                abs(cmd[0]+half_track*cmd[1])/limits['wheel_v'])
            counts[solved['reason']] += 1
            pre_deadline_counts[solved['pre_deadline_reason']] += 1
            if solved['active_zero']:
                counts['active_zero'] += 1
            for attempt in solved['attempts']:
                validation_counts[str(attempt['code'])] += 1
            # Rejoin is sustained, not the final closest point, and must be
            # beyond the blocked interval in a static-obstacle trial.
            safe_ahead = True
            if case == 'static_rejoin':
                safe_ahead = (t >= CRITERIA['static_appearance_s'] and
                              geometric['s'] > world.actor_s+CRITERIA['static_radius_m']+
                              np.max(np.linalg.norm(body[:, :2], axis=1)+body[:, 2])+.1)
            elif case == 'dynamic_crossing':
                safe_ahead = t >= CRITERIA['dynamic_crossing_s']+1.
            # Success uses geometric joining of a safe reference segment, not
            # a diagnostic clock that may conservatively retain a skipped cusp.
            safe_join = False
            if geometric['valid']:
                pose = plant.s.copy()
                pose[:2] = geometric['point']
                safe_join = body_clearance(pose, body, world.actors(t)) >= limits['margin']
            near = (geometric['valid'] and geometric['error'] <= CRITERIA['rejoin_distance_m']
                    and abs(geometric['yaw_error']) <= CRITERIA['rejoin_heading_rad']
                    and safe_ahead and safe_join)
            dwell = dwell+.05 if near else 0.
            if dwell >= CRITERIA['rejoin_continuous_s']-1e-9 and first_rejoin is None:
                first_rejoin = t+.05-dwell
            rejoin_samples.append(bool(near))
            goal_distance = float(np.linalg.norm(plant.s[:2]-reference[-1, :2]))
            goal_heading = abs(wrap(plant.s[2]-reference[-1, 2]))
            endpoint = (goal_distance <= CRITERIA['endpoint_distance_m'] and
                        goal_heading <= CRITERIA['endpoint_heading_rad'] and
                        abs(plant.s[3]) < CRITERIA['endpoint_speed_mps'])
            endpoint_dwell = endpoint_dwell+.05 if endpoint else 0.
            if endpoint_dwell >= CRITERIA['endpoint_continuous_s']-1e-9 and goal_time is None:
                goal_time = t+.05-endpoint_dwell
            export_ms = 0.
            if search_stride > 0 and k % search_stride == 0:
                trace, all_count, export_ms = kernel.search()
                files['search_trace'].write(json.dumps(dict(t=t, rows=np.round(trace, 6).tolist(),
                    all_candidate_count=all_count, visual_subset=True))+'\n')
            route = solved.pop('route')
            files['selected_rollouts'].write(json.dumps(dict(t=t, reason=solved['reason'],
                path=route[:, 1:6].tolist(), relative_times=route[:, 0].tolist()))+'\n')
            files['obstacles'].write(json.dumps(dict(t=t, obstacles=[dict(x=x, y=y, radius=r, id=i, vx=vx, vy=vy)
                for x, y, r, i, vx, vy in world.actors(t)]))+'\n')
            files['decisions'].write(json.dumps(dict(t=t, **{key: value.tolist() if isinstance(value, np.ndarray) else value
                for key, value in solved.items()}), allow_nan=False)+'\n')
            rows.append([t, *plant.s, *cmd, projection['error'], projection['lateral'], projection['yaw_error'],
                projection['s'], projection['time'], int(projection['valid']), minimum_gap, solved['kernel_ms'],
                solved['callback_ms'], export_ms, solved['reason'], int(solved['active_zero']),
                solved['accepted_candidate'], goal_distance, int(near),
                geometric['error'], geometric['yaw_error'], geometric['s'], command_bound_ratio])
            plant.command, plant.command_time = cmd, t
            # Exact production-SIL integrator, 5ms physical audit of full cover.
            for j in range(10):
                plant.t = t+(j+1)*.005
                integrate(plant, .005)
                gap = body_clearance(plant.s, body, world.actors(plant.t))
                minimum_gap = min(minimum_gap, gap)
            if minimum_gap < CRITERIA['physical_collision_gap_m'] or (goal_time is not None and first_rejoin is not None):
                break
    finally:
        kernel.close()
        for f in files.values():
            f.close()
    import csv
    with (out/'state.csv').open('w') as f:
        writer = csv.writer(f)
        writer.writerow(('t', 'x', 'y', 'yaw', 'v', 'w', 'cmd_v', 'cmd_w', 'reference_error', 'lateral_error',
            'yaw_error', 'progress_s', 'progress_time', 'projection_valid', 'min_gap_m', 'kernel_ms',
            'callback_ms', 'search_export_ms', 'reason', 'active_zero', 'accepted_candidate', 'goal_distance', 'near_reference',
            'geometric_reference_error', 'geometric_yaw_error', 'geometric_progress_s', 'command_bound_ratio'))
        writer.writerows(rows)
    finite_errors = [row[8] for row in rows if row[8] is not None]
    report = dict(case=case, seed=seed, scope=metadata['scope'], steps=len(rows), simulated_duration_s=len(rows)*.05,
        actual_wall_duration_s=time.monotonic()-start_wall, min_physical_gap_m=minimum_gap,
        collision=bool(minimum_gap < 0.), sustained_rejoin=first_rejoin is not None,
        first_sustained_rejoin_s=first_rejoin, endpoint_reached=goal_time is not None,
        endpoint_settled_s=goal_time, final_state=plant.s.tolist(), final_progress_time_s=previous_time,
        reference_duration_s=float(times[-1]), final_reference_error_m=rows[-1][8],
        max_reference_error_m=max(finite_errors) if finite_errors else None,
        p95_reference_error_m=float(np.percentile(finite_errors, 95)) if finite_errors else None,
        final_geometric_reference_error_m=rows[-1][23],
        max_geometric_reference_error_m=max(r[23] for r in rows if r[23] is not None),
        rejoin_metric='Nearest same-body-heading safe reference segment, not diagnostic clock progress; static case must pass obstacle arclength plus full body reach.',
        near_reference_fraction=float(np.mean(rejoin_samples)), reason_counts=dict(counts),
        pre_deadline_reason_counts=dict(pre_deadline_counts),
        validator_code_counts=dict(validation_counts), deadline_misses=counts['deadline_miss'],
        kernel_p95_ms=float(np.percentile([r[15] for r in rows], 95)),
        callback_p95_ms=float(np.percentile([r[16] for r in rows], 95)),
        search_export_total_ms=sum(r[17] for r in rows), library_sha256=metadata['library_sha256'],
        reference_sha256=metadata['reference_sha256'],
        max_command_bound_ratio=max(r[26] for r in rows),
        passed=bool(minimum_gap >= 0. and first_rejoin is not None and goal_time is not None
                    and max(r[26] for r in rows) <= 1.+1e-8))
    write_json(out/'result.json', report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, default=HERE/'builds/failure_diagnosis_v12_04')
    parser.add_argument('--source', type=Path, default=HERE/'results/resource_contention/validation_02/static_remani_01')
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--cases', nargs='+', choices=DEFAULT_CASES, default=list(DEFAULT_CASES))
    parser.add_argument('--duration', type=float, default=CRITERIA['max_duration_s'])
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--search-stride', type=int, default=5)
    parser.add_argument('--allow-different-kernel', action='store_true', help='Explicitly mark non-V12 control investigation')
    args = parser.parse_args()
    if not 0 < args.duration <= 120:
        parser.error('Duration must be (0,120] seconds')
    args.build, args.source, args.out = args.build.resolve(), args.source.resolve(), args.out.resolve()
    expected = sha(HERE/'builds/native_v12_07/build/libonline_mppi.so')
    actual = sha(args.build/'build/libonline_mppi.so')
    if expected != actual and not args.allow_different_kernel:
        raise RuntimeError('Frozen V12 binary SHA mismatch; refuse an accidental algorithm change')
    args.out.mkdir(parents=True, exist_ok=False)
    write_json(args.out/'protocol.json', dict(criteria=CRITERIA, cases=args.cases,
        seed=args.seed, duration_s=args.duration, actual_kernel_sha256=actual,
        original_native_v12_07_sha256=expected, kernel_identical=expected == actual,
        paired_source=str(args.source), script_sha256=sha(__file__),
        formal_success_criteria='Collision-free AND sustained rejoin AND settled endpoint; no success-rate inference from few fixtures.'))
    results = []
    for case in args.cases:
        result = trial(args.build, args.source, args.out/case, case, args.seed, args.duration, args.search_stride)
        results.append(result)
        print(json.dumps(result, ensure_ascii=False), flush=True)
        write_json(args.out/'summary.json', dict(scope='Fixed native reference / ideal-observation diagnosis only',
            completed=len(results), requested=len(args.cases), cases=results))


if __name__ == '__main__':
    main()
