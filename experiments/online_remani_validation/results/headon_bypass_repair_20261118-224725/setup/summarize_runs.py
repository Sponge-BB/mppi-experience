"""Small recorded-data summary; no ROS or control outputs."""
from pathlib import Path
import collections,csv,json,sys
import numpy as np
CASE=Path(__file__).resolve().parents[1]
summary={}
for group in sys.argv[1:] or ['native','mppi','fast_only']:
 d=CASE/group
 if not (d/'result.json').is_file():continue
 r=json.loads((d/'result.json').read_text())
 row={k:r.get(k) for k in ['reason','goal_reached','elapsed','motion_anchor','min_external_gap_m','min_base_box_gap_m','min_arm_box_gap_m','final_state','trajectory_bundles']}
 states=list(csv.DictReader((d/'state.csv').open()))
 row['max_abs_y_m']=max(abs(float(s['y'])) for s in states)
 f=d/'active_arm_cycles.jsonl'
 if f.is_file():
  c=[json.loads(s) for s in f.read_text().splitlines()]
  c=[s for s in c if s['t']>r['motion_anchor']]
  row['controller_reasons']=dict(collections.Counter(s['reason'] for s in c))
  row['effective_horizons_s']=dict(collections.Counter(round(s['effective_horizon_s'],2) for s in c if 'effective_horizon_s' in s))
  row['cycle_ms_p50_p95_max']=np.percentile([s['callback_ms'] for s in c],[50,95,100]).tolist()
  avoid=[s for s in c if s['reason']=='ok' and s.get('intervention')]
  row['delivered_interventions']=len(avoid)
  row['first_delivered_intervention_t']=avoid[0]['t'] if avoid else None
  row['time_sentinel_candidates']=sum(s.get('min_gap',0) is not None and s.get('min_gap',0)<-999 for s in c)
 summary[group]=row
print(json.dumps(summary,ensure_ascii=False,indent=2))
(CASE/'run_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
