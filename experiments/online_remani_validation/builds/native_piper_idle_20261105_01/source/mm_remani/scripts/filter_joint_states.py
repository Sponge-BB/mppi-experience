#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

def joint_cb(msg):
    # Expect 6 joints exactly in this order for remani_planner's arrays
    target_joints = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    
    pos_map = dict(zip(msg.name, msg.position))
    vel_map = dict(zip(msg.name, msg.velocity))
    
    # Just to be safe, grab effort if available
    effort_map = {}
    if len(msg.effort) == len(msg.name):
        effort_map = dict(zip(msg.name, msg.effort))
        
    out_msg = JointState()
    out_msg.header = msg.header
    
    for j in target_joints:
        out_msg.name.append(j)
        out_msg.position.append(pos_map.get(j, 0.0))
        out_msg.velocity.append(vel_map.get(j, 0.0))
        out_msg.effort.append(effort_map.get(j, 0.0))
        
    pub.publish(out_msg)

if __name__ == '__main__':
    rospy.init_node('filter_joint_states')
    pub = rospy.Publisher('/mm/joint_states', JointState, queue_size=10)
    rospy.Subscriber('/gazebo/joint_states', JointState, joint_cb, queue_size=10)
    rospy.loginfo("Filtering /gazebo/joint_states -> /mm/joint_states (arm joints only)")
    rospy.spin()
