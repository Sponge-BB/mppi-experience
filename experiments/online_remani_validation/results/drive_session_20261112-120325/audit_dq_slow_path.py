#!/usr/bin/env python3
"""Check that the same recorded velocity reaches the existing slow-layer topic."""
import os,sys,json
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
sys.path[:0]=['/opt/ros/noetic/lib/python3/dist-packages','/home/l/remani_ws/devel/lib/python3/dist-packages']
from pathlib import Path
import rosbag,numpy as np
root=Path(__file__).resolve().parent;p=root/'dq_root_cause_analysis.json';doc=json.loads(p.read_text());keys={e['matched_sample']['source_ros']:e for e in doc['events']};matched={}
with rosbag.Bag(str(root/'capture.bag')) as bag:
 for _,msg,t in bag.read_messages(topics=['/mm/joint_states']):
  stamp=msg.header.stamp.to_sec()
  if stamp not in keys:continue
  e=keys[stamp];ids=[msg.name.index('joint'+str(i)) for i in range(1,7)];q=np.array(msg.position)[ids];v=np.array(msg.velocity)[ids]
  matched[stamp]={'receive_ros':t.to_sec(),'source_ros':stamp,'positions_identical':np.array_equal(q,e['matched_sample']['q']),'velocities_identical':np.array_equal(v,e['matched_sample']['dq']),'q':q.tolist(),'dq':v.tolist()}
for stamp,e in keys.items():e['same_input_on_slow_mm_joint_states']=matched.get(stamp,{'missing':True})
doc['repair_location_assessment']={
 'driver': 'The derivative is generated in the driver, but changing SDK/CAN/raw position acquisition is not necessary to resolve how planning consumes derivative estimates; touching the hardware interface would widen this fix. SDK pair stamps would need raw CAN evidence to prove the specific timing distortion.',
 'mppi_only': 'A source-time velocity estimator only in whole_body_mppi.on_joints is a narrow experiment, but leaves REMANI consuming exactly the same pulsed velocity on /mm/joint_states and permits inconsistent slow/fast initial states.',
 'shared_existing_input': 'Preferred small integration point: use the existing filter_joint_states path to provide one explicit source-time velocity estimate to /mm/joint_states, and route MPPI to that same feedback topic. Preserve raw /joint_states_single and hardware driver; keep position/stamp freshness independent; validate the estimator against recorded motion rather than treating a 50/100ms mean as truth. This is a proposed location, not an implemented or validated filter.'}
p.write_text(json.dumps(doc,ensure_ascii=False,indent=2));print(json.dumps({str(keys[s]['seq']):v for s,v in matched.items()},ensure_ascii=False,indent=2))
