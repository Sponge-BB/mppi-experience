#!/usr/bin/python3
"""Bounded hai-side observation and optional isolated, nonexecuted goal."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time
import xmlrpc.client
import os

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry,Path as RosPath
from sensor_msgs.msg import JointState,PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

NS='/standby_trial_20261106_distributed_08'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=float,default=65.)
    parser.add_argument('--output',required=True)
    parser.add_argument('--publish-goal',action='store_true')
    args=parser.parse_args()
    if not 10<=args.duration<=120 or os.environ.get('ROS_MASTER_URI')!='http://192.168.137.128:11462':
        raise ValueError('Bounded08 private-master probe required')
    rospy.init_node('hai_distributed_probe',anonymous=True,disable_signals=True)
    counts=Counter();latest={};ages={}; tf_success=0;goals=[]
    buffer=tf2_ros.Buffer(cache_time=rospy.Duration(5.),debug=False);listener=tf2_ros.TransformListener(buffer)
    def receive(key):
        def callback(msg):
            counts[key]+=1;latest[key]=msg
            if hasattr(msg,'header'):
                ages[key]=rospy.get_time()-msg.header.stamp.to_sec()
        return callback
    specs=[(NS+'/perception/filtered_points',PointCloud2,'filtered'),(NS+'/markers',MarkerArray,'markers'),
           (NS+'/native/path',RosPath,'path'),(NS+'/status',String,'status'),
           (NS+'/native/reference_status',String,'reference_status'),
           ('/standby_trial_20261105_feedback_02/sensors/odom',Odometry,'odom'),
           ('/standby_trial_20261105_feedback_02/sensors/joint_states',JointState,'joints')]
    subs=[rospy.Subscriber(topic,kind,receive(key),queue_size=1) for topic,kind,key in specs]
    pub=rospy.Publisher(NS+'/goal',PoseStamped,queue_size=1)
    start=time.monotonic();last_print=-10.; errors=[]
    while not rospy.is_shutdown() and time.monotonic()-start<args.duration:
        elapsed=time.monotonic()-start
        try:
            buffer.lookup_transform('world','standby_feedback_02_base_link',rospy.Time(0),rospy.Duration(.03))
            tf_success+=1
        except tf2_ros.TransformException:pass
        if args.publish_goal and not goals and elapsed>=20. and all(k in latest for k in ('odom','joints','filtered','status')):
            status=json.loads(latest['status'].data)
            # No goal before valid current model/input state; only this isolated
            # graph has the native planner, with no controller or driver.
            if status.get('proposed_controls') is not None and pub.get_num_connections()>=2:
                odom=latest['odom'];q=odom.pose.pose.orientation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                goal=PoseStamped();goal.header.stamp=rospy.Time.now();goal.header.frame_id='world'
                goal.pose.position.x=odom.pose.pose.position.x+.2*math.cos(yaw)
                goal.pose.position.y=odom.pose.pose.position.y+.2*math.sin(yaw)
                goal.pose.orientation.z=math.sin(yaw/2.);goal.pose.orientation.w=math.cos(yaw/2.)
                pub.publish(goal);goals.append(dict(stamp=goal.header.stamp.to_sec(),x=goal.pose.position.x,
                    y=goal.pose.position.y,yaw=yaw,scope='isolated nonexecuted planner probe from hai'))
        if elapsed-last_print>=5.:
            print(json.dumps({'elapsed_s':round(elapsed,1),'counts':dict(counts),'goals':len(goals)}),flush=True)
            last_print=elapsed
        time.sleep(.2)
    graph=[]
    try:
        master=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        graph=master.getSystemState('/hai_probe_audit')[2]
    except Exception as exc:errors.append(str(exc))
    result=dict(counts=dict(counts),latest_age_s=ages,world_to_base_tf_successes=tf_success,
        goals=goals,latest_status=json.loads(latest['status'].data) if 'status' in latest else None,
        latest_reference_status=json.loads(latest['reference_status'].data) if 'reference_status' in latest else None,
        final_path_points=len(latest['path'].poses) if 'path' in latest else 0,ros_graph=graph,errors=errors,
        hardware_commands_sent=False)
    Path(args.output).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('counts','world_to_base_tf_successes','goals','final_path_points')}),flush=True)
    rospy.signal_shutdown('bounded hai probe complete')


if __name__=='__main__':main()
