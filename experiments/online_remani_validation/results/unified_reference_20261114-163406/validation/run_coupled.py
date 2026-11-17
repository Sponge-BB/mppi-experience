#!/usr/bin/python3
import argparse,sys,importlib.util,json,time,hashlib,copy
from pathlib import Path
import numpy as np
ROOT=Path('/home/l/remani_ws'); WORK=Path('/tmp/remani_unified_validation_20261114')
parser=argparse.ArgumentParser();parser.add_argument('--variant',choices=['baseline','current'],required=True);parser.add_argument('--output',required=True);args=parser.parse_args()
source_root=WORK/'baseline' if args.variant=='baseline' else ROOT
sys.path.insert(0,str(source_root/'src/mm_remani/scripts/mppi'))
import active_arm_mppi,native_reference_adapter,native_reference
original=ROOT/'experiments/online_remani_validation/results/frozen_guidance_capture_20261114-161009/analysis_rootcause'
spec=importlib.util.spec_from_file_location('coupled_env',original/'coupled_native_clock.py');env=importlib.util.module_from_spec(spec);spec.loader.exec_module(env)
output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
fixture=env.load_fixture();gate=json.loads((original/'coupled_recorded_delivery_gate.json').read_text())
cases=[('ideal',fixture,None,120.),('recorded_gate',fixture,gate,float(gate[-1]['t']))]
summary=[]
for name,fixture,events,seconds in cases:
 began=time.monotonic();result=env.simulate(fixture=fixture,delivery_events=events,max_seconds=seconds)
 trace=result.pop('trace');result['wall_seconds']=time.monotonic()-began;result['variant']=args.variant
 result['source_sha256']={str(p.relative_to(source_root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [source_root/'src/mm_remani/scripts/mppi/active_arm_mppi.py',source_root/'src/mm_remani/scripts/mppi/native_reference_adapter.py']}
 (output/(name+'_report.json')).write_text(json.dumps(result,indent=2)+'\n')
 (output/(name+'_trace.jsonl')).write_text(''.join(json.dumps(row,separators=(',',':'))+'\n' for row in trace))
 selected={k:result[k] for k in ['clock_completed','elapsed','final_phase','final_goal_position_error','freeze_events','resume_events','frozen_seconds','scale_transition_command_flips','nonzero_sign_changes','nonzero_sign_changes_scale_mode','nonzero_sign_changes_endpoint_2_5cm','wall_seconds']}; selected['name']=name;summary.append(selected);print(json.dumps(selected),flush=True)
(output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
