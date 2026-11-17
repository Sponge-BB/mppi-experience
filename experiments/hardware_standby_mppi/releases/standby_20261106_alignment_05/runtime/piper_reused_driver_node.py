#!/usr/bin/env python3
"""Isolated Piper driver fork; importing it never constructs a real SDK.

The frozen original node supplies feedback publication methods. Its original
constructor, subscription threads and every original control callback are
deliberately unused. All live actuator calls go through PiperLeaseGate.
"""
import importlib.util
import json
import os
from pathlib import Path
import threading
import time
from urllib.parse import urlparse

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String, UInt8MultiArray
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import PoseStamped
from piper_msgs.msg import PiperStatusMsg, PiperEulerPose
from piper_msgs.srv import Enable, EnableResponse

from piper_lease_gate import PiperLeaseGate, open_existing_sdk
from piper_command_pair import PiperCommandPair
from bounded_can_transport import install_bounded_can_transport
from base_lease_gate import validate_readiness


RELEASE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "frozen_original_piper_driver", RELEASE/"driver_reference/piper_ctrl_single_node.py")
original = importlib.util.module_from_spec(spec)
spec.loader.exec_module(original)


class ReusedPiperNode(original.C_PiperRosNode):
    def __init__(self):
        # Never call the original __init__: it performs initialization TX,
        # creates direct-control bypasses and starts automatic enable threads.
        rospy.init_node("piper_reused_driver", anonymous=False)
        if rospy.get_name() != "/standby_trial_20261106_alignment_05/arm/piper_reused_driver":
            raise RuntimeError("driver must use its exact isolated launch namespace")
        parsed = urlparse(os.environ.get("ROS_MASTER_URI", ""))
        if parsed.hostname not in ("127.0.0.1", "localhost") or parsed.port != 11461:
            raise RuntimeError("control trial requires existing localhost master 11461")
        if rospy.get_param("/use_sim_time", False):
            raise RuntimeError("real control refuses simulated ROS time")
        connect = rospy.get_param("~connect_transport", False)
        allow = rospy.get_param("~allow_motion", False)
        if type(connect) is not bool or type(allow) is not bool:
            raise ValueError("connect_transport and allow_motion must be bool")
        if rospy.get_param("~auto_enable", False) or rospy.get_param("~auto_reenable_on_drop", False):
            raise ValueError("automatic enabling is prohibited in this fork")
        self.piper = None
        if connect:
            ready, reason = validate_readiness(RELEASE/"config/hardware_readiness.json")
            if not ready:
                raise RuntimeError("hardware connection blocked: "+reason)
            # The SDK class and its protocol come from the exact installed
            # 0.6.1 copy. No initialization/mode/enable packet is sent here.
            self.piper = open_existing_sdk(original.C_PiperInterface,
                                          rospy.get_param("~can_port", "can1"),
                                          transport_installer=install_bounded_can_transport)
        self.gate = PiperLeaseGate(self.piper, allow_motion=allow,
                                  fixed_vel=rospy.get_param("~fixed_vel", 5))
        self.command_pair = PiperCommandPair(self.gate)
        self.joint_states = JointState()
        self.joint_states.name = ["joint%d" % i for i in range(1, 7)]+["gripper"]
        self.joint_states.position = [0.]*7
        self.joint_states.velocity = [0.]*7
        self.joint_states.effort = [0.]*7
        # All outputs are private/remapped in launch. SDK's aggregate feedback
        # stamp is not an all-six-joints freshness guarantee and MUST NOT be
        # wired back into verified_feedback or planner measurements.
        self.joint_pub = rospy.Publisher("joint_states_single", JointState, queue_size=1)
        self.arm_status_pub = rospy.Publisher("arm_status", PiperStatusMsg, queue_size=1)
        self.end_pose_pub = rospy.Publisher("end_pose", PoseStamped, queue_size=1)
        self.end_pose_euler_pub = rospy.Publisher("end_pose_euler", PiperEulerPose, queue_size=1)
        self.motor_enable_pub = rospy.Publisher("piper_motor_enable_states", UInt8MultiArray, queue_size=1)
        self.status_pub = rospy.Publisher("gate_status", String, queue_size=1)
        self.feedback_sub = rospy.Subscriber("verified_feedback", JointState, self.feedback_callback,
                                             queue_size=1, tcp_nodelay=True)
        self.command_sub = rospy.Subscriber("joint_ctrl_single", JointState, self.joint_callback,
                                            queue_size=1, tcp_nodelay=True)
        self.lease_sub = rospy.Subscriber("command_lease", String, self.lease_callback,
                                          queue_size=4, tcp_nodelay=True)
        self.enable_sub = rospy.Subscriber("enable_flag", Bool, self.enable_callback, queue_size=1)
        self.enable_service = rospy.Service("enable_srv", Enable, self.handle_enable_service)
        self.stop_service = rospy.Service("stop_srv", Trigger, self.handle_stop_service)
        self._exit = threading.Event()
        self._watchdog = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._watchdog.start()
        rospy.on_shutdown(self.shutdown)

    def feedback_callback(self, msg):
        # This must be the independent receive-only assembler: it timestamps
        # the OLDEST of all three CAN pairs and refuses incomplete snapshots.
        self.gate.update_feedback(msg.name, msg.position, msg.header.stamp.to_sec(),
                                  time.monotonic(), time.time())

    def GetEnableFlag(self):
        return self.gate.status()["enabled"]

    def joint_callback(self, msg):
        self.command_pair.joint(msg.name, msg.position, msg.header.seq,
                                msg.header.stamp.to_sec(), time.monotonic(), time.time(),
                                velocities=msg.velocity, efforts=msg.effort)

    def lease_callback(self, msg):
        self.command_pair.lease(msg.data, time.monotonic(), time.time())

    def enable_callback(self, msg):
        self.command_pair.enable(msg.data, time.monotonic(), time.time())

    def handle_enable_service(self, req):
        return EnableResponse(self.command_pair.enable(req.enable_request, time.monotonic(), time.time()))

    def handle_stop_service(self, _req):
        self.gate.stop()
        state = self.gate.status()
        return TriggerResponse(success="stop_transport_error" not in state["reason"],
                               message=state["reason"])

    def _watchdog_loop(self):
        # ROS /clock and callback traffic cannot pause this watchdog.
        while not self._exit.wait(.01):
            self.command_pair.tick(time.monotonic(), time.time())

    def Pubilsh(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.piper is not None:
                try:
                    # These are the unmodified original feedback methods.
                    self.PublishMotorEnableStates()
                    self.PublishArmState()
                    self.PublishArmEndPose()
                    self.PublishArmJointAndGripper()
                except Exception as exc:
                    self.gate.stop("sdk_feedback_error:"+type(exc).__name__)
                    rospy.logerr_throttle(2., "SDK feedback publication failed: %s", exc)
            self.status_pub.publish(String(data=json.dumps(self.gate.status(), sort_keys=True)))
            rate.sleep()

    def shutdown(self):
        self.gate.stop("shutdown")
        self._exit.set()
        # Frozen SDK DisconnectPort only stops read threads/closes socket;
        # it does not send arm-disable/gripper/reset packets.
        if self.piper is not None:
            self.piper.DisconnectPort()

    # Explicit overrides ensure even accidental direct Python calls cannot
    # enter old bypasses inherited from the original class.
    def pos_callback(self, _msg):
        return False

    def handle_gripper_service(self, _req):
        raise RuntimeError("gripper bypass disabled")

    def handle_go_zero_service(self, _req):
        raise RuntimeError("go-zero bypass disabled")

    def handle_reset_service(self, _req):
        raise RuntimeError("reset bypass disabled")

    def handle_block_arm_service(self, _req):
        raise RuntimeError("use stop_srv; unblock bypass disabled")

    def MaintainAutoEnable(self):
        return None

    def SubPosThread(self):
        raise RuntimeError("pos subscriber disabled")

    def SubJointThread(self):
        raise RuntimeError("original unguarded subscriber disabled")

    def SubEnableThread(self):
        raise RuntimeError("original unguarded enable thread disabled")


if __name__ == "__main__":
    ReusedPiperNode().Pubilsh()
