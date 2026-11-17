#!/usr/bin/env python3
"""Captured-joint geometry and synthetic planning timing; no ROS/device access."""
import importlib.util
import json
from pathlib import Path
import resource
import sys
import time

import numpy as np

RELEASE = Path(__file__).resolve().parents[1]
for directory in ('core', 'runtime', 'tools'):
    sys.path.insert(0, str(RELEASE/directory))
from active_arm_model import PiperArmModel
from standby_whole_body_mppi import StandbyWholeBodyMPPI
from mppi_command_gateway import NativeLeaseValidator
from run_offline_validation import make_fixture


def main():
    evidence = RELEASE.parents[1]/'mesh_audit_20261106'
    asset = json.loads((RELEASE/'model/shoulder_wrist_proxy.json').read_text())
    q = np.asarray(asset['certificate']['q_radians'])
    model = PiperArmModel.from_parameters(RELEASE/'model/parameters.yaml', RELEASE/'model/mm.urdf')
    previous = RELEASE.parent/'standby_20261106_alignment_05'
    spec = importlib.util.spec_from_file_location('previous05', previous/'core/active_arm_model.py')
    old = importlib.util.module_from_spec(spec); spec.loader.exec_module(old)
    coarse = old.PiperArmModel.from_parameters(RELEASE/'model/parameters.yaml', RELEASE/'model/mm.urdf')
    cfg = json.loads((RELEASE/'config/hardware_shadow.json').read_text())
    a = cfg['arm_limits']
    kernel = StandbyWholeBodyMPPI(model, cfg['limits'],
                                 [a[k] for k in ('lower', 'upper', 'velocity', 'acceleration')],
                                 cfg['postures'], horizon=cfg['horizon'], step=cfg['step'],
                                 random_samples=cfg['random_samples'], seed=cfg['seed'])
    # Both observations are explicit synthetic fixtures; no real cloud is
    # treated as free merely because it contains no returns at some location.
    fixture_model, _, fixture_kernel, snapshot, extended_state, reference = make_fixture()
    state = np.r_[np.zeros(5), q, np.zeros(6)]
    timings = []; validation_ms = []; plans = []
    for _ in range(3):
        kernel.previous = None
        begin = time.perf_counter()
        plan, info = kernel.plan(state, reference, q, snapshot, 1000.05)
        timings.append((time.perf_counter()-begin)*1000.)
        if plan is None:
            raise RuntimeError('captured joint state has no synthetic-fixture plan: '+str(info))
        control, prediction = plan
        validator = NativeLeaseValidator(kernel)
        begin = time.perf_counter()
        valid = validator(state, control[0], snapshot, 1000.05, .1)
        validation_ms.append((time.perf_counter()-begin)*1000.)
        if not valid:
            raise RuntimeError('captured joint plan failed final validation')
        plans.append({'first_control': control[0].tolist(), 'diagnostics': info,
                      'base_displacement_m': float(np.linalg.norm(prediction[-1, :2]-state[:2]))})
    # Existing extended-arm moving-point fixture provides a nonzero MPPI
    # avoidance check; its pose is synthetic, not the actual folded arm.
    extended_plan, extended_info = fixture_kernel.plan(
        extended_state, reference, extended_state[5:11], snapshot, 1000.05)
    if extended_plan is None:
        raise RuntimeError('extended-arm synthetic avoidance regression failed')
    sweep = json.loads((evidence/'nearby_mesh_cases.json').read_text())['cases']
    rows = []
    for case in sweep:
        qi = np.asarray(case['q'])
        result = model.check(qi)
        if case['surface_intersection'] and result['valid']:
            raise RuntimeError('original mesh intersection was accepted')
        rows.append(dict(case, coarse=coarse.check(qi), refined=result))
    report = {'schema': 'captured-joint-mesh-refinement-v1', 'release': RELEASE.name,
              'hardware_motion_tested': False, 'ros_or_can_opened': False,
              'q_radians': q.tolist(), 'coarse_geometry': coarse.check(q),
              'refined_geometry': model.check(q), 'collision_policy': model.collision_policy(),
              'mesh_sweep': rows, 'captured_joint_synthetic_plans': plans,
              'timing': {'plan_ms': timings, 'final_validation_ms': validation_ms,
                         'configured_plan_budget_ms': cfg['deadline_s']*1000.,
                         'configured_final_validation_budget_ms': 20.,
                         'scope': 'three offline calls with synthetic cloud; not sustained real-time proof'},
              'extended_arm_synthetic_avoidance': {
                  'diagnostics': extended_info,
                  'predicted_base_displacement_m': float(np.linalg.norm(extended_plan[1][-1, :2])),
                  'predicted_joint_change_rad': (extended_plan[1][-1, 5:11]-extended_state[5:11]).tolist()},
              'limitations': ['Original mounting assumptions; no physical calibration.',
                              'Numerical padding is not a physical safety clearance.',
                              'Near high-obstacle coverage and actual actuator response remain unvalidated.'],
              'peak_rss_mib_before_plot': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.}
    (RELEASE/'reports/mesh_refinement.json').write_text(json.dumps(report, indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    x = [r['joint5_delta_rad'] for r in rows]
    fig, ax = plt.subplots(figsize=(9, 4.6))
    ax.plot(x, [r['coarse']['min_self_gap']*1000 for r in rows], label='05 coarse spheres', color='#b56332')
    ax.plot(x, [r['refined']['min_self_gap']*1000 for r in rows], 'o-', label='06 with conservative mesh check', color='#147a69')
    ax.plot(x, [r['surface_distance_m']*1000 for r in rows], '--', label='Original shoulder/wrist triangle surface gap', color='#425ab5')
    ax.axhline(0, color='black', lw=.8); ax.axvline(0, color='#777777', lw=.8, ls=':')
    collision_x = [r['joint5_delta_rad'] for r in rows if r['surface_intersection']]
    ax.axvspan(min(collision_x), max(collision_x), color='#cc5555', alpha=.12, label='Original meshes intersect')
    ax.set(xlabel='Joint 5 offset from captured pose (rad)', ylabel='Model gap (mm)',
           title='Offline geometry check — actual captured joints, no robot motion')
    ax.legend(fontsize=8); ax.grid(alpha=.2); fig.tight_layout()
    fig.savefig(RELEASE/'reports/mesh_refinement.png', dpi=150); plt.close(fig)
    print(json.dumps({'current_pose_valid': report['refined_geometry']['valid'],
                      'plan_ms': timings, 'final_validation_ms': validation_ms,
                      'peak_rss_mib': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.}))


if __name__ == '__main__':
    main()
