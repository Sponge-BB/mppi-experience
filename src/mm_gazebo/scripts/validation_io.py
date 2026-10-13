#!/usr/bin/env python3
"""Gazebo-only physical wheel/arm bridge, ground truth feedback, and scene map."""
import math
import os
import threading
import time
from urllib.parse import urlparse
import numpy as np
import rospy
import tf2_ros
from tf.transformations import quaternion_matrix
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float64, Header
from control_msgs.msg import JointTrajectoryControllerState


def require_isolated():
    uri=urlparse(os.environ.get('ROS_MASTER_URI',''))
    if uri.hostname not in ('localhost','127.0.0.1') or uri.port!=11431:
        raise RuntimeError('Validation requires ROS_MASTER_URI=http://127.0.0.1:11431')
    if not rospy.get_param('/use_sim_time',False) or not rospy.get_param('/gazebo_validation/enabled',False):
        raise RuntimeError('Missing simulation-only guards')


class IO:
    def __init__(self):
        require_isolated();self.lock=threading.Lock();self.command=Twist();self.received=0.
        self.v=0.;self.w=0.;self.last_time=None;self.last_odom=0.
        base='/real_base_tracker/';mm='/remani_planner_node/mm/'
        self.vmax=rospy.get_param(base+'tracker_max_v');self.wmax=rospy.get_param(base+'tracker_max_w')
        self.a=rospy.get_param(base+'tracker_max_linear_accel');self.aw=rospy.get_param(base+'tracker_max_angular_accel')
        self.timeout=rospy.get_param(base+'command_timeout',.3)
        self.arm_v=rospy.get_param(mm+'manipulator_max_vel');self.arm_a=rospy.get_param(mm+'manipulator_max_acc')
        self.arm=np.r_[np.deg2rad(rospy.get_param('/remani_planner_node/fsm/init_state')[2:]),0.,0.]
        self.arm_goal=self.arm.copy();self.arm_velocity=np.zeros(8)
        self.wheels=[rospy.Publisher('/gazebo/'+w+'_wheel_velocity_controller/command',Float64,queue_size=1) for w in ('fl','bl','fr','br')]
        self.joints=[rospy.Publisher(f'/gazebo/joint{i}_position_controller/command',Float64,queue_size=1) for i in range(1,9)]
        self.odom=rospy.Publisher('/odom',Odometry,queue_size=1);self.tf=tf2_ros.TransformBroadcaster()
        self.applied=rospy.Publisher('/sim/applied_cmd_vel',Twist,queue_size=1)
        self.gripper=rospy.Publisher('/mm/mani/gripper_state',Bool,queue_size=1,latch=True)
        self.gripper.publish(False)
        rospy.Subscriber('/sim/cmd_vel',Twist,self.cmd,queue_size=1)
        rospy.Subscriber('/mm_controller_node/joint_cmd',JointTrajectoryControllerState,self.arm_cmd,queue_size=1)
        rospy.Subscriber('/gazebo/model_states',ModelStates,self.feedback,queue_size=1)
        cloud=point_cloud2.create_cloud_xyz32(Header(frame_id='world'),np.load(rospy.get_param('~static_map_file'),allow_pickle=False).tolist())
        self.map_pub=rospy.Publisher('/map_generator/global_cloud',PointCloud2,queue_size=1,latch=True)
        self.map_pub.publish(cloud)
        self.map_timer=rospy.Timer(rospy.Duration(2),lambda _:self.map_pub.publish(cloud))
        rospy.on_shutdown(self.stop)
        self.timer=rospy.Timer(rospy.Duration(.02),self.tick)
        # Wall-time watchdog still stops wheel commands when ROS time stops.
        self.watchdog=threading.Thread(target=self.watch,daemon=True);self.watchdog.start()

    def cmd(self,msg):
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):return
        with self.lock:self.command=msg;self.received=time.monotonic()

    def arm_cmd(self,msg):
        values=dict(zip(msg.joint_names,msg.desired.positions))
        with self.lock:
            for i in range(8):
                value=values.get(f'joint{i+1}',self.arm_goal[i])
                if math.isfinite(value):self.arm_goal[i]=value
            self.arm_goal[5]=np.clip(self.arm_goal[5],-.35,.35)

    def feedback(self,msg):
        try:i=msg.name.index('mm_robot')
        except ValueError:return
        stamp=rospy.Time.now();now=stamp.to_sec()
        if now-self.last_odom<.009:return
        self.last_odom=now
        od=Odometry();od.header.stamp=stamp;od.header.frame_id='odom';od.child_frame_id='base_link'
        od.pose.pose=msg.pose[i]
        q=msg.pose[i].orientation;rot=quaternion_matrix([q.x,q.y,q.z,q.w])[:3,:3]
        linear=msg.twist[i].linear;angular=msg.twist[i].angular
        vel=rot.T@np.array([linear.x,linear.y,linear.z]);omega=rot.T@np.array([angular.x,angular.y,angular.z])
        od.twist.twist.linear.x,od.twist.twist.linear.y,od.twist.twist.linear.z=vel
        od.twist.twist.angular.x,od.twist.twist.angular.y,od.twist.twist.angular.z=omega
        tr=TransformStamped();tr.header=od.header;tr.child_frame_id='base_link'
        tr.transform.translation.x=od.pose.pose.position.x;tr.transform.translation.y=od.pose.pose.position.y;tr.transform.translation.z=od.pose.pose.position.z
        tr.transform.rotation=q;self.tf.sendTransform(tr);self.odom.publish(od)

    def tick(self,event):
        now=rospy.get_time();dt=.02 if self.last_time is None else max(0.,min(.05,now-self.last_time));self.last_time=now
        with self.lock:
            fresh=time.monotonic()-self.received<=self.timeout
            v=np.clip(self.command.linear.x,-self.vmax,self.vmax) if fresh else 0.
            w=np.clip(self.command.angular.z,-self.wmax,self.wmax) if fresh else 0.
            self.v+=float(np.clip(v-self.v,-self.a*dt,self.a*dt));self.w+=float(np.clip(w-self.w,-self.aw*dt,self.aw*dt))
            # Wheel conversion must match physical URDF, not arbitrarily copy planner wheel geometry.
            left=(self.v-.411132*self.w/2)/.08;right=(self.v+.411132*self.w/2)/.08
            for pub,value in zip(self.wheels,[left,left,right,right]):pub.publish(value)
            applied=Twist();applied.linear.x=self.v;applied.angular.z=self.w;self.applied.publish(applied)
            for i,pub in enumerate(self.joints):
                err=self.arm_goal[i]-self.arm[i]
                speed=math.copysign(min(self.arm_v,math.sqrt(2*self.arm_a*abs(err))),err)
                self.arm_velocity[i]+=float(np.clip(speed-self.arm_velocity[i],-self.arm_a*dt,self.arm_a*dt))
                delta=self.arm_velocity[i]*dt
                if abs(delta)>=abs(err):self.arm[i]=self.arm_goal[i];self.arm_velocity[i]=0.
                else:self.arm[i]+=delta
                pub.publish(float(self.arm[i]))

    def stop(self):
        for pub in self.wheels:pub.publish(0.)

    def watch(self):
        while not rospy.is_shutdown():
            if time.monotonic()-self.received>self.timeout+.2:
                with self.lock:self.v=0.;self.w=0.;self.stop()
            time.sleep(.05)


if __name__=='__main__':
    rospy.init_node('validation_io');node=IO();rospy.spin()
