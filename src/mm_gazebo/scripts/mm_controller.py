#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from rospy.msg import AnyMsg
from control_msgs.msg import JointTrajectoryControllerState
from rospy.msg import AnyMsg


class MMController(object):
    def __init__(self):
        rospy.init_node('mm_controller', anonymous=False)

        # Base drive params
        self.wheel_radius = float(rospy.get_param('~wheel_radius', 0.08))
        self.wheel_separation = float(rospy.get_param('~wheel_separation', 0.411))

        # Arm joint params
        self.arm_joint_names = rospy.get_param(
            '~arm_joint_names',
            ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7', 'joint8']
        )
        self.arm_command_topic = rospy.get_param('~arm_command_topic', '/mm_controller_node/joint_cmd')
        self.publish_joint_state = bool(rospy.get_param('~publish_joint_state', True))
        self.arm_joint_state_topic = rospy.get_param('~arm_joint_state_topic', '/joint_states_cmd')

        # Wheel controllers
        self.pub_fl = rospy.Publisher('/gazebo/fl_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_fr = rospy.Publisher('/gazebo/fr_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_bl = rospy.Publisher('/gazebo/bl_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_br = rospy.Publisher('/gazebo/br_wheel_velocity_controller/command', Float64, queue_size=1)

        # Arm output topics: pub a JointState-like command topic for external IK/controller,
        # and also publish per-joint position controllers if available.
        self.arm_js_pub = rospy.Publisher(self.arm_joint_state_topic, JointState, queue_size=1)
        self.arm_joint_pubs = []
        for joint_name in self.arm_joint_names:
            topic = '/gazebo/%s_position_controller/command' % joint_name
            self.arm_joint_pubs.append(rospy.Publisher(topic, Float64, queue_size=1))

        self.last_joint_cmd = JointState()
        self.last_joint_cmd.name = list(self.arm_joint_names)
        self.last_joint_cmd.position = [0.0] * len(self.arm_joint_names)

        rospy.Subscriber('/cmd_vel', Twist, self.cb_cmd_vel, queue_size=1)
        rospy.Subscriber(self.arm_command_topic, AnyMsg, self.cb_any_msg, queue_size=1)

        rospy.loginfo('mm_controller started. base(r=%.3f, track=%.3f), arm_cmd=%s',
                      self.wheel_radius, self.wheel_separation, self.arm_command_topic)
        rospy.spin()

    def cb_cmd_vel(self, msg):
        v = msg.linear.x
        wz = msg.angular.z
        half_track = self.wheel_separation * 0.5
        v_left = v - wz * half_track
        v_right = v + wz * half_track
        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius

        self.pub_fl.publish(Float64(omega_left))
        self.pub_bl.publish(Float64(omega_left))
        self.pub_fr.publish(Float64(omega_right))
        self.pub_br.publish(Float64(omega_right))

    def cb_any_msg(self, msg):
        topic_type = msg._connection_header['type']
        if topic_type == 'sensor_msgs/JointState':
            real_msg = JointState().deserialize(msg._buff)
            pos_map = dict(zip(real_msg.name, real_msg.position))
        elif topic_type == 'control_msgs/JointTrajectoryControllerState':
            real_msg = JointTrajectoryControllerState().deserialize(msg._buff)
            pos_map = dict(zip(real_msg.joint_names, real_msg.desired.positions))
        else:
            return

        joint_positions = []
        for i, joint_name in enumerate(self.arm_joint_names):
            if joint_name in pos_map:
                val = pos_map[joint_name]
                self.last_joint_cmd.position[i] = val
            else:
                val = self.last_joint_cmd.position[i]
            joint_positions.append(val)

        out_js = JointState()
        out_js.header.stamp = rospy.Time.now()
        out_js.name = list(self.arm_joint_names)
        out_js.position = joint_positions
        self.arm_js_pub.publish(out_js)

        for pub, pos in zip(self.arm_joint_pubs, joint_positions):
            pub.publish(Float64(pos))


if __name__ == '__main__':
    try:
        MMController()
    except rospy.ROSInterruptException:
        pass
