import json,re,math,datetime,resource,gc,time
from pathlib import Path
import rosbag
case=Path('/home/l/remani_ws/experiments/online_remani_validation/results/dynamic_crossing_20261116-195203/navigation_segment')
j=json.load(open(case/'analysis/perception_navigation_audit.json'))
start=j['scope']['navigation_start'];end=datetime.datetime.fromisoformat('2026-11-16T20:07:50.841+08:00').timestamp()
rx=re.compile(r'([A-Za-z_][A-Za-z0-9_]*)=([^\s]+)');commits=[];regions=[];timeouts=[];count=0;t0=time.monotonic()
with rosbag.Bag(str(case/'core.bag'),'r') as bag:
 bag._connection_indexes={cid:ix for cid,ix in bag._connection_indexes.items() if bag._connections[cid].topic=='/rosout_agg'};gc.collect()
 for _,m,t in bag.read_messages(topics=['/rosout_agg']):
  stamp=m.header.stamp.to_sec()
  if not start<=stamp<=end or m.name!='/remani_planner_node':continue
  count+=1;s=m.msg
  if '[GridMap][LiDAR]' in s and 'timed out' in s:timeouts.append({'stamp':stamp,'text':s})
  if '[MapTrace]' not in s:continue
  d=dict(rx.findall(s))
  if d.get('stage')=='commit':commits.append(d)
  elif d.get('stage')=='region':regions.append(d)
def stats(a):
 a=sorted(map(float,a))
 if not a:return {'n':0}
 def q(f):
  i=(len(a)-1)*f;lo=int(i);hi=min(lo+1,len(a)-1);return a[lo]+(a[hi]-a[lo])*(i-lo)
 return {'n':len(a),'min':a[0],'p50':q(.5),'p95':q(.95),'max':a[-1]}
r={'schema':'perception_strict_navigation_v1','scope':{'start':start,'end':end,'end_local':'2026-11-16 20:07:50.841 +08:00','end_evidence':'Parent audit determined final planning/finish at20:07:50.841; no messages after this time included.','interval':'first goal to last finish; may include intermediate idle/replanning, excludes final post-finish idle'},'commit_count':len(commits),'compute_ms':stats(d['compute_ms'] for d in commits),'compute_cells':stats(d['compute_cells'] for d in commits),'changed_voxels':stats(d['changed'] for d in regions if 'changed' in d),'regions_eq_1':sum(float(d.get('regions',0))==1 for d in commits),'compute_equals_input_count':sum(d.get('compute_cells')==d.get('input_cells') for d in commits),'compute_gt_50ms_count':sum(float(d.get('compute_ms',0))>50 for d in commits),'lidar_timeout_count':len(timeouts),'lidar_timeout_examples':timeouts,'peak_rss_MiB':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,'elapsed_s':time.monotonic()-t0,'original_full_interval_statistics_preserved':True}
out=case/'analysis/perception_strict_navigation.json';out.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n');print(json.dumps(r,ensure_ascii=False))
