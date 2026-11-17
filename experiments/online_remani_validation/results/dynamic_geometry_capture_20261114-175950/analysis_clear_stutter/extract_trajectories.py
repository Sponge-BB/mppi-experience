import json, math, time
from pathlib import Path
import rosbag, rospy
p=Path('/home/l/remani_ws/experiments/online_remani_validation/results/dynamic_geometry_capture_20261114-175950')
out=p/'analysis_clear_stutter'
count=0
start=time.monotonic()
with rosbag.Bag(str(p/'dynamic_capture.bag')) as bag, (out/'trajectories.jsonl').open('w') as ft, (out/'odom_full_pose.jsonl').open('w') as fo:
 for top,m,t in bag.read_messages(topics=['/planning/trajectory','/odom'],start_time=rospy.Time.from_sec(1794650906.)):
  r=dict(bag=t.to_sec(),source=m.header.stamp.to_sec())
  if top=='/planning/trajectory':
   r.update(action=m.action,id=m.trajectory_id,gear=m.singul,pieces=[dict(duration=q.duration,data=list(q.data),num_dim=q.num_dim,num_order=q.num_order) for q in m.trajectory])
   ft.write(json.dumps(r)+'\n'); count+=1
  else:
   q=m.pose.pose.orientation
   r.update(x=m.pose.pose.position.x,y=m.pose.pose.position.y,z=m.pose.pose.position.z,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),v=m.twist.twist.linear.x,w=m.twist.twist.angular.z)
   fo.write(json.dumps(r)+'\n')
print(json.dumps(dict(trajectories=count,elapsed=time.monotonic()-start)))
