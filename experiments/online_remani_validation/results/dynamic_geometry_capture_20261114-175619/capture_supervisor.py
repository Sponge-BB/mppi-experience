"""Read-only bounded rosbag capture. No robot commands or parameter changes."""
import json,os,signal,subprocess,sys,threading,time
from pathlib import Path
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
out=Path(sys.argv[1]);began=time.monotonic();stop=threading.Event();lock=threading.Lock()
first_goal=None;goal_stamp=None;seen={};counts={};children=[];diag_started=False;reason=''
TOPICS=['/mppi/status','/mppi/prediction_rollouts','/mppi/collision_snapshot',
 '/cmd_vel','/odom','/move_base_simple/goal','/planning/trajectory','/planning/finish',
 '/planning/safety_speed_limit','/planning/mppi_refresh_request','/tracking_fault',
 '/piper_execution_state','/joint_states_single','/mm/joint_states','/joint_ctrl_single',
 '/mm_controller_node/trajectory_progress','/mm_controller_node/base_tracking_error',
 '/mm_controller_node/arm_tracking_error','/rosout_agg','/dynamic_points',
 '/velodyne_points_mapping','/velodyne_points','/cloud_registered','/tf','/tf_static',
 '/map_generator/global_cloud','/dynamic_obstacle_tracker/confirmed_track_count',
 '/dynamic_obstacle_tracker/motion_qualified_track_count','/dynamic_obstacle_tracker/maximum_motion_qualified_speed',
 '/dynamic_obstacle_tracker/perception_healthy','/dynamic_obstacle_tracker/processing_ms',
 '/dynamic_obstacle_tracker/input_age']
REQUIRED=['/mppi/status','/cmd_vel','/odom','/mm/joint_states','/dynamic_points','/velodyne_points_mapping']

def launch(name,topics,buffer,size,duration):
 cmd=['/opt/ros/noetic/lib/rosbag/record','--lz4','--buffsize='+str(buffer),'--chunksize=512',
      '--size='+str(size),'--duration='+str(duration),'--min-space=8G','-O',str(out/name)]+topics+['__name:='+name+'_recorder']
 log=(out/(name+'.log')).open('w')
 p=subprocess.Popen(cmd,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);log.close()
 children.append((name,p));return p

def observe(m,topic):
 with lock:seen[topic]=time.monotonic();counts[topic]=counts.get(topic,0)+1

def goal(m):
 global first_goal,goal_stamp
 with lock:
  if first_goal is None:first_goal=time.monotonic();goal_stamp=m.header.stamp.to_sec()
 with (out/'goal_events.jsonl').open('a') as f:f.write(json.dumps(dict(received_ros=rospy.Time.now().to_sec(),received_wall=time.time(),source=m.header.stamp.to_sec(),x=m.pose.position.x,y=m.pose.position.y))+'\n')

def save(phase):
 with lock:
  now=time.monotonic();fresh={t:round(now-s,3) for t,s in seen.items()}
  d=dict(phase=phase,reason=reason,pid=os.getpid(),elapsed_s=round(now-began,2),first_goal_stamp=goal_stamp,
   since_first_goal_s=None if first_goal is None else round(now-first_goal,2),sample_counts=dict(counts),input_age_s=fresh,
   required_missing_or_stale=[t for t in REQUIRED if fresh.get(t,999)>2.],diagnostic_started=diag_started,
   diagnostic_limit='Goal-triggered optional exact occupancy diagnostic: producer at most 16 frames at about 1 Hz.',
   children=[dict(name=n,pid=p.pid,returncode=p.poll()) for n,p in children],control_publications=False,
   main_buffer_mib=32,max_main_file_mib=4096,after_goal_limit_s=90,topics=TOPICS)
 temp=out/'status.tmp';temp.write_text(json.dumps(d,indent=2)+'\n');temp.replace(out/'status.json')
for s in [signal.SIGINT,signal.SIGTERM]:signal.signal(s,lambda *_:stop.set())
save('starting')
rospy.init_node('dynamic_capture_monitor',anonymous=True,disable_signals=True,disable_rosout=True)
main=launch('dynamic_capture',TOPICS,32,4096,900)
subs=[rospy.Subscriber(t,rospy.AnyMsg,observe,callback_args=t,queue_size=1,buff_size=262144) for t in REQUIRED]
subs.append(rospy.Subscriber('/move_base_simple/goal',PoseStamped,goal,queue_size=1))
try:
 while not stop.is_set() and not rospy.is_shutdown():
  now=time.monotonic()
  if main.poll() is not None:reason='main_recorder_finished';break
  if first_goal is not None and not diag_started:
   launch('occupancy_evidence',['/mppi/collision_diagnostic'],8,256,20);diag_started=True
  if first_goal is not None and now-first_goal>=90:reason='90_seconds_after_first_goal';break
  if now-began>=900:reason='duration_limit';break
  save('capturing_navigation' if first_goal is not None else ('ready_for_user_goal' if all(now-seen.get(t,0)<2 for t in REQUIRED) else 'waiting_for_robot_topics'))
  stop.wait(.5)
finally:
 for sub in subs:sub.unregister()
 for name,p in children:
  if p.poll() is None:p.send_signal(signal.SIGINT)
 for name,p in children:
  try:p.wait(timeout=12)
  except subprocess.TimeoutExpired:reason=(reason+'; recorder_close_timeout:'+name)
 save('closed');rospy.signal_shutdown('capture complete')
