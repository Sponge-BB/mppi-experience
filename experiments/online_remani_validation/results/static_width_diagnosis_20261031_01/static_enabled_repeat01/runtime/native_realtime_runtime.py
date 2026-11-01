"""Common runtime-copy instrumentation for native-3D demand-sampling trials.

No new controller, collision model, budget, actuator or reference changes.
Both groups execute identical Python sources; only the child's launch switch
chooses the original free-path sampler or the existing bounded candidate set.
"""
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np

from native_collision_runtime import patch_trial as patch_native_trial, replace_once

STALL_WINDOW_S = 5.
MIN_STOP_SIM_S = 17.


def deadline_zero_pattern(row):
    report = row.get('validation', [])
    return (row.get('reference_points', 0) > 0 and row.get('reason') == 'deadline_miss'
        and row.get('code') == 0 and row.get('validation_code') == 0
        and len(report) >= 6 and report[0] == 1 and report[5] == 1
        and abs(row.get('nominal_v', 0)) + abs(row.get('nominal_w', 0)) > 1e-9
        and abs(row.get('applied_v', 0)) + abs(row.get('applied_w', 0)) <= 1e-9)


def update_trial_evidence(sim):
    """Read-only task evidence and common early termination, not control logic.

    Called after integration/recording. True actor locations are evaluation
    evidence only and never copied into MPPI or REMANI inputs.
    """
    if not hasattr(sim, 'realtime_evidence'):
        sim.realtime_evidence = dict(first_truth_actor_t=None, first_observed_moving_t=None,
            deadline_zero_window_start=None, deadline_zero_window_end=None,
            deadline_zero_cycles=0, deadline_zero_deadlock=False,
            early_stop_window_s=STALL_WINDOW_S, minimum_stop_sim_s=MIN_STOP_SIM_S,
            scope='evaluation only; real motion-triggered scene exposure; no fabricated dynamic encounter')
        sim.realtime_seen_diagnostics = 0
        sim.realtime_window_pose = None
    evidence = sim.realtime_evidence
    if evidence['first_truth_actor_t'] is None and sim.obstacles():
        evidence['first_truth_actor_t'] = float(sim.t)
        sim.events.append(dict(t=sim.t, kind='realtime_truth_actor_appeared'))
    new_rows = sim.diagnostics[sim.realtime_seen_diagnostics:]
    sim.realtime_seen_diagnostics = len(sim.diagnostics)
    for row in new_rows:
        if evidence['first_observed_moving_t'] is None and any(
                len(ob) >= 6 and ob[5] > .5 for ob in row.get('observed_obstacles', [])):
            evidence['first_observed_moving_t'] = float(row['t'])
        if deadline_zero_pattern(row):
            previous = evidence['deadline_zero_window_end']
            if previous is not None and float(row['t'])-previous > .5:
                evidence['deadline_zero_window_start'] = None
                evidence['deadline_zero_cycles'] = 0
            if evidence['deadline_zero_window_start'] is None:
                evidence['deadline_zero_window_start'] = float(row['t'])
                sim.realtime_window_pose = np.asarray(sim.s[:3]).copy()
            evidence['deadline_zero_window_end'] = float(row['t'])
            evidence['deadline_zero_cycles'] += 1
        else:
            evidence['deadline_zero_window_start'] = None
            evidence['deadline_zero_window_end'] = None
            evidence['deadline_zero_cycles'] = 0
            sim.realtime_window_pose = None
    start, end = evidence['deadline_zero_window_start'], evidence['deadline_zero_window_end']
    if start is None or end is None:
        return False
    pose = sim.realtime_window_pose
    angle = abs(np.arctan2(np.sin(sim.s[2]-pose[2]), np.cos(sim.s[2]-pose[2])))
    moving = np.linalg.norm(np.asarray(sim.s[:2])-pose[:2]) > .001 or angle > .001 or \
        abs(sim.s[3]) > .001 or abs(sim.s[4]) > .001
    if moving:
        evidence['deadline_zero_window_start'] = None
        evidence['deadline_zero_window_end'] = None
        evidence['deadline_zero_cycles'] = 0
        sim.realtime_window_pose = None
        return False
    stop = (not getattr(sim, 'native_truth_collision', False)
            and sim.t >= MIN_STOP_SIM_S and end-start >= STALL_WINDOW_S
            and sim.t-end < .5 and evidence['deadline_zero_cycles'] >= 10)
    if stop:
        evidence['deadline_zero_deadlock'] = True
        evidence['early_stop_t'] = float(sim.t)
        sim.events.append(dict(t=sim.t, kind='deadline_zero_deadlock',
            window_start=start, window_end=end, cycles=evidence['deadline_zero_cycles']))
    return stop


def patch_trial(out, demand):
    """Patch only NEW trial snapshots; original 04 files/builds stay untouched."""
    out = Path(out).resolve()
    patch_native_trial(out, enabled=True, parity=False)
    runtime = out/'runtime'
    text = (runtime/'simulator.py').read_text()
    text = replace_once(text, '            self.record()',
        "            self.record()\n            from native_realtime_runtime import update_trial_evidence\n            if update_trial_evidence(self):reason='deadline_zero_deadlock';break")
    anchor = "        (self.out/'result.json').write_text(json.dumps(result,indent=2)+'\\n')"
    text = replace_once(text, anchor,
        "        result['native_realtime_evidence']=getattr(self,'realtime_evidence',{})\n"+anchor)
    (runtime/'simulator.py').write_text(text)
    if Path(__file__).resolve() != runtime/Path(__file__).name:
        shutil.copy2(Path(__file__), runtime/Path(__file__).name)
    tree = ET.parse(out/'experiment.launch')
    for node in tree.getroot().findall('node'):
        if node.get('name') in ('base_mppi', 'remani_planner_node'):
            for name, value in (('NATIVE_REALTIME_DEMAND', str(int(demand))),
                                ('NATIVE_REALTIME_PROFILE', '0'), ('NATIVE_REALTIME_REPLAY', '0')):
                ET.SubElement(node, 'env', name=name, value=value)
    tree.write(out/'experiment.launch', encoding='unicode')
    path = out/'manifest.json'
    manifest = json.loads(path.read_text())
    for name in ('simulator.py', 'native_realtime_runtime.py'):
        manifest['experiment_sources_sha256'][name] = hashlib.sha256((runtime/name).read_bytes()).hexdigest()
    manifest['native_realtime'] = dict(demand=bool(demand), common_native_collision_3d=True,
        profile=False, replay=False, binary_switch='NATIVE_REALTIME_DEMAND',
        changed='skip only native3D V7 free-path full sampling, use existing bounded candidates',
        collision_and_dynamic_branches_unchanged=True, early_stop_window_s=STALL_WINDOW_S,
        minimum_stop_sim_s=MIN_STOP_SIM_S, early_stop_reason='deadline_zero_deadlock',
        early_stop_is_navigation_success=False, actual_scene_exposure_reported_separately=True)
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')

