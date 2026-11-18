"""Minimal pure-function counterexample: no nearby obstacle or ROS processes."""
from pathlib import Path
import json,sys,hashlib
import numpy as np
from scipy.spatial import cKDTree
case=Path(__file__).resolve().parents[1];trial=case/'mppi'
sys.path.insert(0,str(trial/'runtime'))
from active_arm_model import PiperArmModel
from active_arm_mppi import Snapshot, WholeBodyMPPI
cfg=json.loads((trial/'active_arm_config.json').read_text())
model=PiperArmModel.from_parameters(trial/'parameters.yaml',trial/'model/mm.urdf')
limits=json.loads((trial/'limits.json').read_text())
al=cfg['arm_limits'];state=np.r_[np.zeros(5),cfg['q_start'],np.zeros(6)]
reference=np.array([[0.,0.,0.,0.],[1.,0.,0.,.2]])
h=np.zeros(32);h[4]=h[19]=h[28]=1.;h[3]=h[18]=10.;h[5:8]=[80,80,40];h[8:11]=[-4.,-4.,-.5];h[11]=.1;h[15]=.35;h[16]=2.
field=np.full((80,80,40),10.)
far=[(np.array([-.18,0.,0.]),cKDTree(np.array([[3.,3.,1.]])))]
rows=[]
for name,groups,prediction in [('no_dynamic_track',[],2.),('dynamic_track_2s',far,2.),('counterexample_coverage_3s',far,3.)]:
 header=h.copy();header[16]=prediction;snapshot=Snapshot(header,field,groups)
 kernel=WholeBodyMPPI(model,limits,[al[k] for k in ['lower','upper','velocity','acceleration']],cfg['postures'],horizon=20,step=.1,random_samples=6)
 cost,valid,gap,states=kernel.evaluate(state,np.zeros((1,20,8)),reference,np.asarray(cfg['q_start']),snapshot,10.)
 rows.append(dict(case=name,valid=bool(valid[0]),min_gap=float(gap[0]),H=20,dt=.1,robot_horizon_s=2.,checked_end_offset_s=2.05,dynamic_coverage_s=prediction,all_external_obstacles_far=True))
assert rows[0]['valid'] and not rows[1]['valid'] and rows[1]['min_gap'] < -1000 and rows[2]['valid']
report=dict(scope='Pure-function controlled example, not historical map replay or a runtime fix',source_sha256=hashlib.sha256((trial/'runtime/active_arm_mppi.py').read_bytes()).hexdigest(),rows=rows,conclusion='A nonempty distant dynamic track invalidates a safe stationary candidate solely because 2.05s check exceeds 2.0s coverage.',limitation='The 3s condition is only a counterexample; it is not a recommendation to extend prediction trust blindly.')
(case/'failure_analysis/prediction_window_check.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
