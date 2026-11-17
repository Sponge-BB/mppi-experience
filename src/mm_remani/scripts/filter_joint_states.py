#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

import math
from collections import deque

_previous = None
_velocity_estimator = None
TARGET_JOINTS = tuple('joint%d' % i for i in range(1, 7))


class SourceTimeVelocityWindow:
    """Causal position difference over a source-time window, without clipping.

    Only velocity is estimated: the caller publishes the latest position and
    original stamp. For constant acceleration the velocity lags by window/2.
    A feedback gap requires a new complete window, never a zero-velocity guess.
    """
    def __init__(self, window_s=0.05, max_gap_s=0.15):
        if not (math.isfinite(window_s) and math.isfinite(max_gap_s)
                and 0 < window_s <= max_gap_s):
            raise ValueError('velocity window must be positive and <= max gap')
        self.window_s = window_s
        self.max_gap_s = max_gap_s
        self.samples = deque(maxlen=128)

    def update(self, stamp, q):
        if not (math.isfinite(stamp) and stamp > 0 and len(q) == 6
                and all(math.isfinite(value) for value in q)):
            return None
        if self.samples:
            dt = stamp - self.samples[-1][0]
            if dt <= 0:
                return None
            if dt > self.max_gap_s:
                self.samples.clear()
        self.samples.append((stamp, tuple(q)))
        cutoff = stamp - self.window_s
        while len(self.samples) > 1 and self.samples[1][0] <= cutoff:
            self.samples.popleft()
        if len(self.samples) < 2 or self.samples[0][0] > cutoff:
            return None
        t0, q0 = self.samples[0]
        t1, q1 = self.samples[1]
        alpha = (cutoff - t0) / (t1 - t0)
        old = [a + alpha * (b - a) for a, b in zip(q0, q1)]
        return [(value - start) / self.window_s for value, start in zip(q, old)]


def joint_cb(msg):
    global _previous
    if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
        return
    positions = dict(zip(msg.name, msg.position))
    if not all(j in positions and math.isfinite(positions[j]) for j in TARGET_JOINTS):
        rospy.logwarn_throttle(2.0, 'Drop joint feedback missing a valid arm position.')
        return
    stamp = msg.header.stamp.to_sec()
    if not math.isfinite(stamp) or stamp <= 0:
        return
    q = [positions[j] for j in TARGET_JOINTS]
    previous = _previous
    if previous is not None and stamp <= previous[0]:
        return
    _previous = (stamp, q)
    velocities = dict(zip(msg.name, msg.velocity)) if len(msg.velocity) == len(msg.name) else {}
    if _velocity_estimator is not None:
        v = _velocity_estimator.update(stamp, q)
        if v is None:
            return
    elif all(j in velocities and math.isfinite(velocities[j]) for j in TARGET_JOINTS):
        v = [velocities[j] for j in TARGET_JOINTS]
    elif previous is not None and 0 < stamp - previous[0] <= 0.15:
        v = [(value - old) / (stamp - previous[0]) for value, old in zip(q, previous[1])]
    else:
        # A missing velocity is unknown, not zero; wait for the next sample.
        return
    out_msg = JointState()
    out_msg.header = msg.header
    out_msg.name = list(TARGET_JOINTS)
    out_msg.position = q
    out_msg.velocity = v
    efforts = dict(zip(msg.name, msg.effort)) if len(msg.effort) == len(msg.name) else {}
    if all(j in efforts and math.isfinite(efforts[j]) for j in TARGET_JOINTS):
        out_msg.effort = [efforts[j] for j in TARGET_JOINTS]
    pub.publish(out_msg)

if __name__ == '__main__':
    rospy.init_node('filter_joint_states')
    window_s = float(rospy.get_param('~velocity_window_s', 0.0))
    if window_s != 0.0:
        _velocity_estimator = SourceTimeVelocityWindow(window_s)
    pub = rospy.Publisher('/mm/joint_states', JointState, queue_size=10)
    rospy.Subscriber('/gazebo/joint_states', JointState, joint_cb, queue_size=10)
    rospy.loginfo("Filtering /gazebo/joint_states -> /mm/joint_states; velocity window %.3fs",
                  window_s)
    rospy.spin()
