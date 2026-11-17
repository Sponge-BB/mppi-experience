#!/usr/bin/python3
"""Summarize and audit read-only replay; no ROS."""
import hashlib,json
from pathlib import Path
P=Path(__file__).resolve().parent
b=json.loads((P/'baseline_replay.json').read_text());be=json.loads((P/'baseline_extra.json').read_text());f=json.loads((P/'fixed_replay.json').read_text())
root=Path('/home/l/remani_ws');rows=[]
for old,new in zip(b['replay']['summary'],f['replay']['summary']):
 rows.append(dict(window=old['label'],rows=old['rows'],old_samephase_flips=old['samephase_flips'],fixed_samephase_flips=new['samephase_flips'],old_crossing_2_5cm_flips=old['samephase_flips_crossing_2_5cm'],fixed_crossing_2_5cm_flips=new['samephase_flips_crossing_2_5cm']))
source_names=['active_arm_mppi.py','native_reference_adapter.py']
checks=dict(baseline_first_control_matches_all_680=b['replay']['summary'][0]['observed_command_matches']==680,
 input_hashes_match=len({d['input_sha256'] for d in (b,be,f)})==1 and f['input_sha256']==hashlib.sha256((P/'inputs.json').read_bytes()).hexdigest(),
 all_proposals_finite=f['replay']['all_finite'],no_2_5cm_crossing_flip=f['replay']['summary'][0]['samephase_flips_crossing_2_5cm']==0,
 yaw_freeze_windows_no_flip=all(r['fixed_samephase_flips']==0 for r in rows if r['window'] in ('long_yaw_freeze','third_yaw_freeze')),
 boundary_no_flip=f['extra']['boundary']['w_flips_ge_005']==0,
 all_33_numerical_closed_loops_reach_final_5cm_015rad=len(f['closed_loop']['cases'])==33 and all(c['final_freeze_exit_ready'] for c in f['closed_loop']['cases']),
 all_6_motion_fixtures_keep_explicit_gear=len(f['extra']['motion'])==6 and all(m['all_command_direction_correct'] for m in f['extra']['motion']),
 tested_runtime_sources_match_current=all(f['source_sha256'][name]==hashlib.sha256((root/'src/mm_remani/scripts/mppi'/name).read_bytes()).hexdigest() for name in source_names))
assert all(checks.values()),checks
flips=[]
for old,new in zip(f['replay']['rows'],f['replay']['rows'][1:]):
 if old['phase']==new['phase'] and old['command'][1]*new['command'][1]<0 and min(abs(old['command'][1]),abs(new['command'][1]))>=.05:flips.append(dict(previous=old,current=new))
summary=dict(status='OFFLINE_VERIFIED_HARDWARE_PENDING',checks=checks,method=dict(input='680 recorded nominal delivered states, exact source-stamp equality for odom and joint samples, recorded native phase/gears/polynomials; all 17 measured state coordinates retained.',replay='Actual WholeBodyMPPI.proposals nominal first command, actual base_command saturation and recorded safety speed limit. Baseline exactly reproduces all recorded (v,w). Fixed output is counterfactual at the old measurements, not a newly executed trajectory.',closed_loop=f['closed_loop']['scope'],environment='System /usr/bin/python3 with PYTHONNOUSERSITE=1, PYTHONDONTWRITEBYTECODE=1 and OPENBLAS/OMP/MKL thread limits 1. No ROS nodes or publishers. No dependency changes.',limits=['Recorded bag lacks full collision-map payload, so recorded collision acceptance and physical closed-loop outcomes are not replayed.','Simple clear geometry is only for independent numeric base response; no new real-vehicle calibration or safety claim.','Ordinary speed minimum .08 m/s remains existing behavior. Low scaled-speed tests verify explicit gear, not time-scaling speed compliance.','The three residual counterfactual flips are single approach-to-yaw stage entries across 3.5 cm; no remaining 2.5 cm branch flips.']),replay_comparison=rows,boundary_flips=dict(baseline=be['extra']['boundary']['w_flips_ge_005'],fixed=f['extra']['boundary']['w_flips_ge_005']),closed_loop_cases=33,recorded_start_closed_loop=[{k:v for k,v in c.items() if k!='trace'} for c in f['closed_loop']['cases'][:3]],motion=[{k:v for k,v in c.items() if k!='trace'} for c in f['extra']['motion']],residual_flips=flips,baseline_version='20261114-155510-before-frozen-guidance',input_sha256=f['input_sha256'])
(P/'comparison.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(dict(status=summary['status'],checks=checks,comparison=rows),indent=2))
