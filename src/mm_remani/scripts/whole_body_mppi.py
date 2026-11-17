#!/usr/bin/python3
"""REMANI reference -> whole-body MPPI -> existing ROS actuator topics."""
import os
from contextlib import contextmanager
for _key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_key] = '1'
import json
import math
from pathlib import Path
import sys
import threading
import time
from urllib.parse import urlparse
sys.path.insert(0, str(Path(__file__).resolve().parent/'mppi'))

import numpy as np
import rosgraph
import rospy
from rospy.numpy_msg import numpy_msg
import rospkg
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray, String
from quadrotor_msgs.msg import PolynomialTraj
from piper_msgs.msg import PiperExecutionState
from active_arm_model import PiperArmModel
from active_arm_mppi import Snapshot, wrap
from standby_whole_body_mppi import StandbyWholeBodyMPPI
from native_reference_adapter import NativeReferenceAdapter
from prediction_visualization import encode_prediction
from mppi_refresh_event import AvoidanceEpisode, encode_event

JOINTS = tuple('joint'+str(i) for i in range(1, 7))
# Discard an obsolete plan locally while a valid reference transition settles.
# Actual collision/invalid-command failures still request planner recovery. Normal
# reference expiry is already handled by the FSM's end-of-trajectory replan.
REFERENCE_TRANSITION_REJECTIONS = (
    'reference_changed_during_plan', 'reference_changed_before_delivery',
    'waiting_native_reference_after_goal', 'waiting_native_progress',
    'waiting_native_progress_during_plan', 'waiting_native_progress_before_delivery',
    'native_reference_quiet_interval', 'native_reference_quiet_interval_before_delivery',
    'native_reference_not_started', 'native_reference_not_started_before_delivery',
    'native_reference_cancelled', 'native_reference_cancelled_before_delivery',
    'native_gear_changed_during_plan', 'native_gear_changed_before_delivery',
    'native_reference_expired', 'native_reference_expired_during_plan',
    'native_reference_expired_before_delivery',
)

# These failures reject this delivery, not the spatial reference. Keep the
# existing reference for a later cycle with fresh inputs and full validation.
# No fault-clear is sent here: only successful delivery may report recovery.
# Invalid geometry/commands, collision failures and a latched driver stop are
# deliberately absent. Do not broaden this list with prefix matching.
LOCAL_RETRY_REJECTIONS = frozenset((
    'planning_deadline_exceeded', 'delivery_deadline_exceeded',
    'collision_snapshot_unavailable', 'collision_snapshot_expired_during_plan',
    'collision_snapshot_decode_failed',
    'state_feedback_stale', 'feedback_expired_during_plan',
    'inputs_expired_before_delivery',
    'waiting_explicit_arm_enable_or_normal_status',
))

# These are reference transport/activation waits, not feedback, collision,
# driver, or cancellation failures. A new zero-velocity intent may maintain
# the driver's lease only after the ordinary latest-state/map command checks.
REFERENCE_WAIT_HOLD_REASONS = frozenset((
    'waiting_native_reference_after_goal', 'waiting_native_progress',
    'native_reference_quiet_interval', 'native_reference_not_started',
    'native_reference_expired',
))


def fresh(stamp, now, age=.15):
    return math.isfinite(stamp) and stamp > 0 and -.005 <= now-stamp <= age


def checked_input_evidence(node, stage, now):
    """Capture the inputs at the actual gate, while its original lock is held.

    Diagnostic values never grant freshness or replace the controller checks.
    Only scalar metadata is copied; the ESDF array is never copied here.
    """
    record = dict(stage=stage, checked_at=float(now))
    for key in ('base', 'joint'):
        stamp = getattr(node, key+'_stamp', 0.)
        record[key+'_stamp'] = float(stamp)
        record[key+'_age_ms'] = round((now-stamp)*1000., 3)
        record[key+'_fresh'] = fresh(stamp, now)
        trace = getattr(node, key+'_input_trace', None)
        if trace is not None:
            record[key+'_callback'] = dict(trace)
    header = getattr(getattr(node, 'snapshot', None), 'header', None)
    if isinstance(header, np.ndarray) and header.shape == (32,):
        record['snapshot_header'] = header.tolist()
        record['map_age_ms'] = round((now-header[3])*1000., 3)
        record['dynamic_age_ms'] = round((now-header[18])*1000., 3)
        record['map_age_ok'] = bool(-.05 <= now-header[3] <= header[15])
        record['dynamic_age_ok'] = bool(header[28] == 0 or -.05 <= now-header[18] <= header[15])
        record['snapshot_valid'] = bool(header[4] == 1)
        record['dynamic_valid'] = bool(header[28] == 0 or header[19] == 1)
    trace = getattr(node, 'snapshot_input_trace', None)
    if trace is not None:
        record['snapshot_callback'] = dict(trace)
    return record


@contextmanager
def measured_delivery_lock(node, diagnostics):
    """Measure the existing lock scope without changing its ordering/lifetime."""
    began = time.monotonic()
    with node.reference.lock, node.lock:
        acquired = time.monotonic()
        diagnostics['delivery_lock_wait_ms'] = round((acquired-began)*1000., 3)
        try:
            yield
        finally:
            diagnostics['delivery_lock_hold_ms'] = round((time.monotonic()-acquired)*1000., 3)


def failure_recording_context(state, snapshot, info, plan_time):
    """Small original-cycle evidence; no geometry recomputation or field copy."""
    result = dict(plan_time=float(plan_time))
    if isinstance(state, np.ndarray) and state.shape == (17,):
        result['measured_state'] = state.tolist()
    header = getattr(snapshot, 'header', None)
    if isinstance(header, np.ndarray) and header.shape == (32,):
        result['snapshot_header'] = header.tolist()
    for key, value in info.items():
        if (key.startswith(('structured_', 'random_', 'native_')) or key in
                ('mode', 'feasible', 'candidates', 'min_gap', 'external_min_gap',
                 'reference_source', 'reference_revision', 'reference_status',
                 'selected_family', 'selection')):
            result[key] = value
    return result


def validate_simulation_environment(use_sim_time, master_uri, base_topic, arm_topic):
    """Simulation may only execute on this experiment's local, remapped graph."""
    master = urlparse(master_uri)
    if (use_sim_time is not True or master.scheme != 'http' or
            master.hostname not in ('localhost', '127.0.0.1', '::1') or
            master.port != 11466 or master.username is not None or
            master.password is not None or master.path not in ('', '/') or
            master.query or master.fragment or base_topic != '/sim/cmd_vel' or
            arm_topic != '/sim/joint_ctrl_single'):
        raise ValueError('simulation_requires_sim_time_local_master_11466_and_sim_command_remaps')


def base_command(control, limits):
    """Same coupled wheel/linear/angular saturation used by the rollout."""
    v, w = map(float, control[:2]); h = limits['track']/2
    scale = max(1., abs(v)/limits['vmax'], abs(w)/limits['wmax'],
                abs(v-h*w)/limits['wheel_v'], abs(v+h*w)/limits['wheel_v'])
    return v/scale, w/scale


def checked_first_step(kernel, state, control, snapshot, now):
    """Check the delivered action, its driver hold and braking on the new map.

    Base receives velocity; Piper receives the first joint position target.
    The numerical position-servo mode checks the actual held goal followed by
    watchdog braking. The legacy mode checks velocity and position envelopes.
    These are model predictions; hardware response still needs measurement.
    """
    half = kernel.dt/2
    if kernel.arm_position_servo:
        target = kernel.position_target(state, control)
        states = kernel.position_lease_brake_states(state, control, lease=.30)
        times = now+np.arange(len(states))*half
        # The ROS sink consumes a position goal, not the predicted first pose.
        end = states[2].copy()
        end[5:11] = target
    else:
        # MOVEJ consumes a position intent, not the inertial predicted pose.
        # Keep measured velocity in the separate stopping envelope below.
        lead = np.clip(np.clip(control[2:], -kernel.vmax, kernel.vmax)*kernel.dt, -.03, .03)
        target = np.clip(state[5:11]+lead, kernel.qmin, kernel.qmax)
        velocity_states = kernel.lease_brake_states(state, control[None], lease=.30)
        end = velocity_states[2].copy()
        end[5:11] = target
        # Check the start, midpoint and delivered target at every base/time
        # sample: fixed_vel does not establish when MOVEJ reaches the target.
        # This is a discrete geometric envelope, not a calibrated servo model.
        position_states = np.repeat(velocity_states[None], 3, axis=0)
        position_states[:, :, 5:11] = np.stack((state[5:11],
            (state[5:11]+target)/2, target))[:, None, :]
        position_states[:, :, 11:17] = 0.
        states = np.concatenate((velocity_states, position_states.reshape((-1, 17))))
        times = now + np.tile(np.arange(len(velocity_states))*half, 4)
    spheres = kernel.spheres(states)
    gaps, _ = snapshot.query(spheres, times)
    # Self/ground geometry depends only on q; repeated held poses need one
    # check. External obstacles above still use every base/time sample.
    unique_q = np.unique(states[:, 5:11], axis=0)
    if (np.any(gaps < 0) or np.any(kernel.model.configuration_gaps_batch(unique_q) < 0)
            or np.any(states[:, 5:11] < kernel.qmin) or np.any(states[:, 5:11] > kernel.qmax)):
        raise ValueError('command_hold_or_braking_rejected_by_latest_state_or_map')
    return end


def reference_wait_hold(kernel, state):
    """Fresh braking intent, never a replay of the last commanded arm target.

    The caller must still run checked_first_step using latest feedback/map and
    all execution-authority/deadline gates. Prediction is for display only.
    """
    controls = np.zeros((kernel.H, 8))
    prediction = kernel.lease_brake_states(state, controls, lease=.30)
    return controls, prediction


def startup_plan(kernel, state, target, snapshot, now, preflight=False):
    """Original standby target, zero base commands, existing collision checks.

    The optional full joint path checks current observed geometry, not future
    dynamic safety. The horizon/held command are rechecked every delivery.
    """
    if np.max(np.abs(state[3:5])) > .01:
        raise ValueError('startup_waiting_stationary_base')
    reference = np.tile(np.r_[state[:3], 0.], (2, 1))
    if preflight:
        count = max(2, int(np.ceil(np.max(np.abs(target-state[5:11]))/.02))+1)
        path = np.tile(state, (count, 1))
        path[:, 5:11] = np.linspace(state[5:11], target, count)
        if (np.any(path[:, 5:11] < kernel.qmin) or np.any(path[:, 5:11] > kernel.qmax)
                or np.any(kernel.model.configuration_gaps_batch(path[:, 5:11]) < 0)):
            raise ValueError('startup_joint_path_configuration_rejected')
        gaps, _ = snapshot.query(kernel.spheres(path), np.full(count, now))
        if np.any(gaps < 0):
            raise ValueError('startup_joint_path_obstacle_rejected')
    controls = np.zeros((1, kernel.H, 8)); z = state.copy()
    for k in range(kernel.H):
        controls[0, k, 2:] = np.clip(1.8*(target-z[5:11])-.2*z[11:17], -kernel.vmax, kernel.vmax)
        z = kernel.advance(z, controls[0, k])
    _, valid, gaps, states = kernel.evaluate(state, controls, reference, target, snapshot, now)
    info = dict(mode='startup_to_standby', feasible=int(valid[0]), min_gap=float(gaps[0]))
    if not valid[0]:
        info.update(kernel.last_evaluation_diagnostics)
    return ((controls[0], states[0]) if valid[0] else None), info


class WholeBodyNode:
    def __init__(self):
        self.lock = threading.RLock()
        self.execute = rospy.get_param('~execute', False)
        self.simulation = rospy.get_param('~simulation', False)
        if self.simulation:
            validate_simulation_environment(rospy.get_param('/use_sim_time', False),
                rosgraph.get_master_uri(), rospy.resolve_name('/cmd_vel'),
                rospy.resolve_name('/joint_ctrl_single'))
        self.arm_enabled = rospy.get_param('~arm_enabled', True)
        self.seed = int(rospy.get_param('~seed', 11))
        self.rate = float(rospy.get_param('~rate_hz', 10.))
        self.deadline = float(rospy.get_param('~deadline_s', .095))
        self.limits = rospy.get_param('~base_limits')
        self.arm_velocity = np.asarray(rospy.get_param('~arm_velocity', [.15]*6))
        self.arm_acceleration = np.asarray(rospy.get_param('~arm_acceleration', [.35]*6))
        packages = rospkg.RosPack()
        model_file = Path(packages.get_path('my_description'))/'robot/mm.urdf'
        proxy_file = Path(packages.get_path('mm_remani'))/'config/mppi_shoulder_wrist_proxy.json'
        self.model = PiperArmModel.from_parameters(
            rospy.get_param('/remani_planner_node'), model_file, proxy_file)
        self.model.lower[5] = max(self.model.lower[5], -.35)
        self.model.upper[5] = min(self.model.upper[5], .35)
        if len(self.model.fixed_spheres) != 47:
            raise ValueError('MPPI requires the current 47-sphere fixed structure')
        # Nominal constants belong to this numerical plant, not real hardware.
        arm_position_servo = bool(rospy.get_param('~arm_position_servo', False))
        if arm_position_servo and not self.simulation:
            raise ValueError('nominal_arm_position_servo_requires_simulation')
        self.kernel = StandbyWholeBodyMPPI(self.model, self.limits,
            (self.model.lower, self.model.upper, self.arm_velocity, self.arm_acceleration),
            rospy.get_param('~postures'), enabled=self.arm_enabled, seed=self.seed,
            horizon=int(rospy.get_param('~horizon', 8)),
            step=1./self.rate, random_samples=int(rospy.get_param('~random_samples', 4)),
            arm_position_servo=arm_position_servo,
            config=rospy.get_param('~standby', {}))
        # Match MMControllerFSM's clamp and its existing recovery parameter.
        # Native pose capture must enter the envelope that releases its clock.
        base_slow_distance = max(.01, float(rospy.get_param(
            '/mm_controller_node/controller/base_tracking_slow_distance', .08)))
        base_resume_distance = max(0., min(base_slow_distance, float(rospy.get_param(
            '/mm_controller_node/controller/base_tracking_resume_distance', .05))))
        self.reference = NativeReferenceAdapter(self.kernel, clock=rospy.get_time,
            resume_distance=base_resume_distance, yaw_hold_speed=max(0., float(rospy.get_param(
                '/mm_controller_node/controller/yaw_hold_speed', .04))))
        # Reuse the original FSM arrival envelope and tracker alignment gain.
        self.terminal_alignment = {
            'position_tolerance': max(.01, float(rospy.get_param('/remani_planner_node/fsm/goal_position_tolerance', .10))),
            'yaw_tolerance': max(.01, float(rospy.get_param('/remani_planner_node/fsm/goal_yaw_tolerance', .15))),
            'arm_tolerance': max(.01, float(rospy.get_param('/remani_planner_node/fsm/goal_arm_tolerance', .12))),
            'timeout': max(.5, float(rospy.get_param('/remani_planner_node/fsm/goal_yaw_alignment_timeout', 8.))),
            'gain': max(.1, float(rospy.get_param('/real_base_tracker/final_alignment_k_yaw', 1.))),
            'max_w': min(self.limits['wmax'], max(.05, float(rospy.get_param('/real_base_tracker/final_alignment_max_angular_speed', .25)))),
        }
        self.native_arm_startup = rospy.get_param('~native_arm_startup', False)
        planned_startup = rospy.get_param('~startup_to_standby', False)
        if (type(self.native_arm_startup) is not bool or type(planned_startup) is not bool
                or (self.native_arm_startup and (planned_startup or self.simulation))):
            raise ValueError('invalid_or_conflicting_arm_startup_modes')
        self.startup_complete = not (self.native_arm_startup or planned_startup)
        self.native_handoff_sent = False
        self.native_handoff_stamp = 0.
        self.startup_target = None
        if not self.startup_complete:
            initial = np.asarray(rospy.get_param('/remani_planner_node/fsm/init_state'), dtype=float)
            if initial.shape != (8,) or not np.isfinite(initial).all() or not self.arm_enabled:
                raise ValueError('invalid_startup_standby_configuration')
            self.startup_target = np.deg2rad(initial[2:])
            if np.any(self.startup_target < self.model.lower) or np.any(self.startup_target > self.model.upper):
                raise ValueError('startup_standby_outside_joint_limits')
        self.startup_enable_requested = False
        self.startup_settled_since = None
        self.enable_feedback_stamp = self.enable_feedback_until = 0.
        self.base = self.joints = self.snapshot = None
        # Keep field ownership separate from the current frame's validity. A
        # malformed/missing delta revokes execution, not the last known field.
        self.snapshot_cache = None
        self.snapshot_error_detail = None
        self.snapshot_resync_last = -math.inf
        self.base_stamp = self.joint_stamp = 0.
        self.anchor = self.arm_anchor = None
        self.execution_stamp = self.execution_valid_until = 0.
        self.execution_enabled = self.execution_latched = False
        self.execution_command_active = False
        self.execution_revision = 0
        self.sent_command = False
        self.sent_navigation_goal = None
        self.safety_speed_limit = None
        self.input_errors = {}; self.last_visual = 0.
        self.visual_hz = float(rospy.get_param('~prediction_visualization_hz', 2.))
        if not math.isfinite(self.visual_hz) or not 0. <= self.visual_hz <= 5.:
            raise ValueError('invalid prediction visualization rate')
        self.tf = tf2_ros.Buffer(cache_time=rospy.Duration(3.))
        self.listener = tf2_ros.TransformListener(self.tf)
        # Only this node owns these topics in MPPI mode. No SDK/CAN is opened.
        self.base_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) if self.execute else None
        self.arm_pub = rospy.Publisher('/joint_ctrl_single', JointState, queue_size=1) if self.execute else None
        self.enable_pub = (rospy.Publisher('/enable_flag', Bool, queue_size=1)
                           if self.execute and not self.simulation and not self.startup_complete
                           and not self.native_arm_startup else None)
        self.fault_pub = rospy.Publisher('/tracking_fault', Bool, queue_size=1) if self.execute else None
        self.info_pub = rospy.Publisher('/mppi/status', String, queue_size=1)
        self.snapshot_resync_pub = rospy.Publisher('/mppi/collision_snapshot_resync',
                                                  Empty, queue_size=1)
        self.rollout_pub = rospy.Publisher('/mppi/prediction_rollouts', Float64MultiArray, queue_size=1)
        self.refresh_episode = AvoidanceEpisode()
        self.refresh_pub = (rospy.Publisher('/planning/mppi_refresh_request',
            Float64MultiArray, queue_size=1) if self.execute else None)
        rospy.Subscriber('/odom', Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber('/joint_states_single', JointState, self.on_joints, queue_size=1)
        rospy.Subscriber('/mppi/collision_snapshot', numpy_msg(Float64MultiArray), self.on_snapshot,
                         queue_size=None, buff_size=8*1024**2)
        rospy.Subscriber('/planning/trajectory', PolynomialTraj,
                         lambda m: self.reference.feed_trajectory(m, rospy.get_time()), queue_size=20)
        rospy.Subscriber('/mm_controller_node/trajectory_progress', Vector3Stamped,
                         lambda m: self.reference.feed_progress(m, rospy.get_time()), queue_size=1)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.on_goal, queue_size=1)
        rospy.Subscriber('/planning/finish', Bool, self.on_finish, queue_size=1)
        rospy.Subscriber('/planning/safety_speed_limit', Float64, self.on_speed_limit, queue_size=1)
        rospy.Subscriber('/piper_execution_state', PiperExecutionState,
                         self.on_execution_state, queue_size=1)
        rospy.on_shutdown(self.stop_base)

    def on_odom(self, msg):
        callback_enter = rospy.get_time()
        p, q, twist = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        stamp = msg.header.stamp.to_sec()
        try:
            values = np.asarray([p.x, p.y, q.x, q.y, q.z, q.w, twist.linear.x, twist.angular.z])
            if (not np.isfinite(values).all() or abs(np.linalg.norm(values[2:6])-1.) > .01 or
                    not fresh(stamp, rospy.get_time())):
                raise ValueError('invalid_odometry')
            # Original planner/controller consume /odom numerically as world.
            # Refuse a changed TF convention rather than correcting only MPPI.
            if msg.header.frame_id != 'world':
                tr = self.tf.lookup_transform('world', msg.header.frame_id, msg.header.stamp,
                                              rospy.Duration(.01)).transform
                t, r = tr.translation, tr.rotation
                if (np.linalg.norm([t.x, t.y, t.z]) > .001 or
                        np.linalg.norm([r.x, r.y, r.z]) > .001):
                    raise ValueError('native_world_odom_alignment_changed')
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            lock_began = time.monotonic()
            with self.lock:
                lock_acquired = time.monotonic()
                if stamp <= self.base_stamp: return
                self.base = np.array([p.x, p.y, yaw, twist.linear.x, twist.angular.z])
                self.base_stamp = stamp
                self.base_input_trace = dict(enter=callback_enter, adopted=rospy.get_time(),
                    lock_wait_ms=round((lock_acquired-lock_began)*1000., 3))
                self.input_errors.pop('odometry', None)
        except (ValueError, tf2_ros.TransformException) as exc:
            with self.lock: self.base = None; self.input_errors['odometry'] = str(exc)

    def on_joints(self, msg):
        callback_enter = rospy.get_time()
        try:
            if len(set(msg.name)) != len(msg.name): raise ValueError('duplicate_joint_names')
            ids = [msg.name.index(name) for name in JOINTS]
            if any(i >= len(msg.position) or i >= len(msg.velocity) for i in ids):
                raise ValueError('joint_feedback_incomplete')
            q = np.asarray([msg.position[i] for i in ids])
            dq = np.asarray([msg.velocity[i] for i in ids])
            stamp = msg.header.stamp.to_sec()
            if not np.isfinite(np.r_[q, dq]).all():
                raise ValueError('joint_feedback_invalid_or_stale')
            if np.any(q < self.model.lower) or np.any(q > self.model.upper):
                raise ValueError('joint_feedback_outside_limits')
            now = rospy.get_time()
            if math.isfinite(stamp) and stamp > 0 and now-stamp > .15:
                # A queued valid packet cannot invalidate newer accepted state.
                # Retain its real source stamp: capture/delivery still pause as
                # soon as the last accepted feedback itself exceeds 150 ms.
                return
            if not fresh(stamp, now):
                raise ValueError('joint_feedback_invalid_or_stale')
            lock_began = time.monotonic()
            with self.lock:
                lock_acquired = time.monotonic()
                if stamp <= self.joint_stamp: return
                self.joints = np.r_[q, dq]; self.joint_stamp = stamp
                self.joint_input_trace = dict(enter=callback_enter, adopted=rospy.get_time(),
                    lock_wait_ms=round((lock_acquired-lock_began)*1000., 3))
                self.input_errors.pop('joint_feedback', None)
        except (ValueError, IndexError) as exc:
            with self.lock: self.joints = None; self.input_errors['joint_feedback'] = str(exc)

    def on_snapshot(self, msg):
        callback_enter = rospy.get_time()
        began = time.monotonic()
        try:
            with self.lock:
                read_acquired = time.monotonic()
                previous = getattr(self, 'snapshot_cache', None)
                if previous is None:
                    previous = self.snapshot
            decode_began = time.monotonic()
            candidate = Snapshot.decode(msg.data, previous)
            decode_end = time.monotonic()
            with self.lock:
                adopted = time.monotonic()
                self.snapshot = candidate
                self.snapshot_cache = candidate
                self.snapshot_error_detail = None
                self.snapshot_input_trace = dict(enter=callback_enter, adopted=rospy.get_time(),
                    read_lock_wait_ms=round((read_acquired-began)*1000., 3),
                    decode_ms=round((decode_end-decode_began)*1000., 3),
                    write_lock_wait_ms=round((adopted-decode_end)*1000., 3),
                    wire_seq=float(candidate.header[2]))
                self.input_errors.pop('collision_snapshot', None)
        except (ValueError, OverflowError) as exc:
            with self.lock:
                self.snapshot = None
                self.snapshot_error_detail = str(exc)
                self.input_errors['collision_snapshot'] = 'collision_snapshot_decode_failed'
            self.request_snapshot_resync()

    def request_snapshot_resync(self):
        """Ask the existing exporter to resend a full field, at most 4 Hz."""
        now = time.monotonic()
        with self.lock:
            publisher = getattr(self, 'snapshot_resync_pub', None)
            if (publisher is None or
                    now-getattr(self, 'snapshot_resync_last', -math.inf) < .25):
                return
            self.snapshot_resync_last = now
        try:
            publisher.publish(Empty())
        except Exception as exc:
            rospy.logwarn_throttle(5., 'MPPI snapshot resync request failed: %s', exc)

    def on_goal(self, msg):
        if not getattr(self, 'startup_complete', True):
            rospy.logwarn_throttle(2., 'Arm startup is not complete; send the navigation goal after startup_ready.')
            return
        try:
            if msg.header.frame_id != 'world':
                original_stamp = msg.header.stamp
                transform = self.tf.lookup_transform('world', msg.header.frame_id,
                    msg.header.stamp, rospy.Duration(.02))
                msg = do_transform_pose(msg, transform)
                msg.header.stamp = original_stamp
            self.reference.feed_goal(msg, rospy.get_time())
        except (ValueError, tf2_ros.TransformException):
            # Invalid new goals must not leave the old reference executable.
            with self.reference.lock:
                self.reference.active = True
                self.reference.goal_stamp = self.reference.goal_received = None
                self.reference.goal_pose = None
                self.reference.min_start = self.reference.last_start = 0.
                self.reference._clear('goal_transform_unavailable')

    def on_finish(self, msg):
        if msg.data and getattr(self, 'startup_complete', True):
            with self.reference.lock, self.lock:
                self.reference.finish()
                self.anchor = self.arm_anchor = None

    def on_execution_state(self, msg):
        """Use the driver's source expiry; a queued packet gains no new lease."""
        stamp, until = msg.header.stamp.to_sec(), msg.valid_until.to_sec()
        with self.lock:
            # Disabled motors may still provide fresh normal feedback for a
            # one-shot startup enable. This never grants command authority.
            now = rospy.get_time()
            if (msg.stop_latched or not msg.feedback_valid or not fresh(stamp, now)
                    or not math.isfinite(until) or not max(stamp, now) < until <= stamp+.15+1e-6):
                self.enable_feedback_until = 0.
                if math.isfinite(stamp):
                    self.enable_feedback_stamp = max(getattr(self, 'enable_feedback_stamp', 0.), stamp)
            elif stamp > getattr(self, 'enable_feedback_stamp', 0.):
                self.enable_feedback_stamp, self.enable_feedback_until = stamp, until
            else:
                self.enable_feedback_until = min(getattr(self, 'enable_feedback_until', 0.), until)
            # Stop applies even without a new CAN frame. Once observed it
            # cannot be undone by delayed healthy packets or driver restart.
            self.execution_latched |= bool(msg.stop_latched)
            if self.execution_latched or not (msg.enabled and msg.feedback_valid):
                if self.execution_enabled: self.execution_revision += 1
                if math.isfinite(stamp): self.execution_stamp = max(self.execution_stamp, stamp)
                self.execution_enabled = False
                self.execution_command_active = False
                self.execution_valid_until = 0.
                self.stop_base()
                return
            now = rospy.get_time()
            if not (fresh(stamp, now) and math.isfinite(until)
                    and max(stamp, now) < until <= stamp+.15+1e-6):
                if self.execution_enabled: self.execution_revision += 1
                if math.isfinite(stamp): self.execution_stamp = max(self.execution_stamp, stamp)
                self.execution_enabled = False
                self.execution_command_active = False
                self.execution_valid_until = 0.
                self.stop_base()
                return
            if stamp <= self.execution_stamp:
                # Repeats cannot turn a disabled state back on or extend a
                # source's earliest expiry. A new CAN frame may authorize it.
                self.execution_valid_until = min(self.execution_valid_until, until)
                return
            if not self.execution_ready(now): self.execution_revision += 1
            self.execution_stamp, self.execution_valid_until = stamp, until
            self.execution_enabled = True
            self.execution_command_active = bool(msg.command_active)

    def execution_ready(self, now):
        return (not self.execution_latched and self.execution_enabled and
                fresh(self.execution_stamp, now) and now < self.execution_valid_until)

    def native_startup_gate(self, now):
        """The driver owns native startup; this node only checks its handoff.

        No map or MPPI solve is needed while the driver moves to standby.
        The first MPPI command still receives normal map/geometry validation;
        navigation stays blocked until source-stamped driver acknowledgement.
        """
        with self.reference.lock, self.lock:
            if not self.execution_ready(now):
                raise ValueError('waiting_native_arm_startup')
            if self.joints is None or not fresh(self.joint_stamp, now):
                raise ValueError('native_startup_joint_feedback_unavailable')
            if self.execution_command_active and not self.native_handoff_sent:
                raise ValueError('native_startup_driver_already_claimed')
            if (self.native_handoff_sent and self.execution_command_active
                    and self.execution_stamp > self.native_handoff_stamp):
                self.startup_complete = True
                self.reference.finish()
                self.anchor = self.arm_anchor = None
                rospy.loginfo('startup_ready: original standby reached; Piper confirmed MPPI takeover.')
                return
            if (np.max(np.abs(self.joints[:6]-self.startup_target)) >= .015
                    or np.max(np.abs(self.joints[6:])) >= .02):
                raise ValueError('native_startup_standby_not_reached')

    def on_speed_limit(self, msg):
        # Preserve real_base_tracker's latched limit: negative/nonfinite clears,
        # positive caps translation, and zero stops translation and rotation.
        with self.lock:
            self.safety_speed_limit = (float(msg.data) if math.isfinite(msg.data) and msg.data >= 0.
                                       else None)

    def stop_base(self):
        if self.base_pub is not None: self.base_pub.publish(Twist())
        # No stale arm publication: on a planner pause, Piper retains its last
        # bounded position target while continuing hardware feedback checks.

    def reject(self, reason, diagnostics=None):
        self.stop_base()
        episode = getattr(self, 'refresh_episode', None)
        if episode is not None:
            episode.clear_count = 0  # A failed cycle is not evidence of clearance.
        if not getattr(self, 'startup_complete', True):
            self.startup_settled_since = None
        # The native tracker fault requests a new navigation trajectory before
        # releasing its latched stop. Standby has no such trajectory to recover;
        # its rejected commands stop the base and pause the arm command stream.
        # A delivered standby command or an older goal cannot establish a
        # tracking failure for a new goal still waiting for its first plan.
        with self.reference.lock:
            if (reason not in REFERENCE_TRANSITION_REJECTIONS
                    and reason not in LOCAL_RETRY_REJECTIONS
                    and self.sent_command and self.fault_pub is not None and self.reference.active
                    and self.reference.goal_stamp is not None
                    and self.sent_navigation_goal == self.reference.goal_stamp):
                self.fault_pub.publish(Bool(data=True))
        record = dict(state=reason, execute=self.execute,
            cycle_ms=round((time.monotonic()-self.cycle_began)*1000, 2))
        if reason == 'collision_snapshot_decode_failed':
            record['snapshot_error'] = getattr(self, 'snapshot_error_detail', None)
        if reason.startswith(('collision_snapshot', 'inputs_expired')):
            h = getattr(getattr(self, 'snapshot', None), 'header', None)
            if isinstance(h, np.ndarray) and h.shape == (32,):
                now = rospy.get_time()
                record['snapshot'] = dict(valid=bool(h[4]),
                    map_age_ms=round((now-h[3])*1000, 2),
                    dynamic_age_ms=round((now-h[18])*1000, 2),
                    cached_joint_age_ms=round((now-h[24])*1000, 2),
                    dynamic_valid=bool(h[19]), ground_valid=bool(h[22]),
                    self_collision_valid=bool(h[23]), field_version=int(h[14]))
        # Keep the first rejection in the node log. Otherwise a later driver
        # stop replaces /mppi/status and hides why command delivery ceased.
        if reason != getattr(self, '_last_rejection_reason', None):
            rospy.logwarn('MPPI command rejected: %s (cycle=%.2f ms; snapshot=%s)',
                          reason, record['cycle_ms'], record.get('snapshot', {}))
            self._last_rejection_reason = reason
        if diagnostics: record['candidate_diagnostics'] = diagnostics
        self.info_pub.publish(String(data=json.dumps(record)))

    def publish_refresh(self, info, checked_at, startup):
        """Called after both commands were published, under their original locks."""
        if getattr(self, 'refresh_pub', None) is None:
            return
        # Optional refresh transport must not reject an already delivered action.
        try:
            eligible = (self.execute and not startup and self.reference.active
                        and self.reference.goal_stamp is not None
                        and not info.get('reference_wait_hold')
                        and not info.get('terminal_pose_alignment'))
            episode = self.refresh_episode.observe(self.reference.goal_stamp,
                eligible, info.get('intervention'), checked_at)
            if episode is None:
                return
            h = self.snapshot.header
            packet = Float64MultiArray(data=encode_event(checked_at,
                self.reference.goal_stamp, self.reference.native.start,
                self.base_stamp, self.joint_stamp, h[3], h[18], h[28] != 0, episode))
            self.refresh_pub.publish(packet)
        except Exception as exc:
            rospy.logwarn_throttle(1., 'MPPI refresh request unavailable: %s', exc)

    def capture(self, now):
        began = time.monotonic()
        with self.lock:
            acquired = time.monotonic()
            self.capture_input_trace = checked_input_evidence(self, 'capture', now)
            self.capture_input_trace['lock_wait_ms'] = round((acquired-began)*1000., 3)
            for value, source in ((self.base, 'odometry'), (self.joints, 'joint_feedback'),
                                  (self.snapshot, 'collision_snapshot')):
                if value is None:
                    raise ValueError(self.input_errors.get(source, 'waiting_'+source))
            if not fresh(self.base_stamp, now) or not fresh(self.joint_stamp, now):
                raise ValueError('state_feedback_stale')
            if not self.snapshot.fresh(now): raise ValueError('collision_snapshot_unavailable')
            state = np.r_[self.base, self.joints]
            if self.anchor is None:
                self.anchor = state[:3].copy(); self.arm_anchor = state[5:11].copy()
            return state, self.snapshot, self.anchor.copy(), self.arm_anchor.copy()

    def cycle(self):
        began = time.monotonic(); now = rospy.get_time()
        self.cycle_began = began
        diagnostics = {}
        self.capture_input_trace = None
        kernel = None; prediction = None; prediction_step = None
        state = None; snapshot = None; info = {}
        visual_due = self.visualization_due(began)
        startup = not getattr(self, 'startup_complete', True)
        native_startup = startup and getattr(self, 'native_arm_startup', False)
        try:
            with self.lock:
                execution_revision = self.execution_revision
                if self.execute and not self.simulation and self.execution_latched:
                    raise ValueError('arm_stop_latched')
                if not startup and self.execute and not self.simulation and not self.execution_ready(now):
                    raise ValueError('waiting_explicit_arm_enable_or_normal_status')
            if native_startup:
                self.native_startup_gate(now)
                startup = not self.startup_complete
                native_startup = startup
            with self.reference.lock:
                revision = self.reference.revision
                state, snapshot, anchor, arm_anchor = self.capture(now)
            if self.capture_input_trace is not None:
                diagnostics['capture_inputs'] = self.capture_input_trace
            if self.simulation:
                diagnostics.update(plan_time=now, measured_state=state.tolist(),
                    snapshot_header=snapshot.header.tolist(), arm_reference=arm_anchor.tolist())
            standby_reference = np.tile(np.r_[anchor, 0.], (2, 1))
            with self.reference.lock:
                if revision != self.reference.revision:
                    raise ValueError('reference_changed_during_plan')
                kernel = self.reference.navigation if self.reference.active and not startup else self.kernel
                kernel.capture_rollouts = visual_due
                kernel.visual_rollouts = None
            if startup and not native_startup:
                result, info = startup_plan(kernel, state, self.startup_target, snapshot, now,
                    preflight=not self.startup_enable_requested and not self.simulation)
            else:
                result, info = self.reference.plan(state, standby_reference, arm_anchor, snapshot, now)
                if (result is None and not startup and
                        info.get('reference_status') in REFERENCE_WAIT_HOLD_REASONS):
                    wait_reason = info['reference_status']
                    info = dict(info, mode='reference_wait_hold', feasible=1,
                                reference_wait_hold=True, hold_reason=wait_reason)
                    alignment = None
                    alignment_config = getattr(self, 'terminal_alignment', None)
                    if wait_reason == 'native_reference_expired' and alignment_config is not None:
                        alignment = self.reference.terminal_pose_command(state, rospy.get_time(), alignment_config)
                    if alignment is None:
                        result = reference_wait_hold(kernel, state)
                    else:
                        controls = np.tile(alignment, (kernel.H, 1))
                        result = controls, kernel.lease_brake_states(state, controls, lease=.30)
                        info.update(mode='terminal_pose_alignment', terminal_pose_alignment=True)
                    if visual_due:
                        prediction_step = kernel.dt/2
            elapsed = time.monotonic()-began
            # This is the cycle budget through plan return, not pure solver
            # time. Preserve the exact decision point before rejection work.
            diagnostics.update(planning_elapsed_ms=round(elapsed*1000, 3),
                               deadline_ms=round(self.deadline*1000, 3))
            if self.simulation:
                diagnostics.update({key:info[key] for key in (
                    'structured_labels', 'structured_valid', 'structured_costs',
                    'structured_external_gaps', 'structured_configuration_gaps',
                    'structured_bounds_valid',
                    'structured_braking_stopped', 'random_valid', 'random_gaps',
                    'structured_first_external_violation_s', 'structured_terminal_arm_delta',
                    'structured_braking_tail_s', 'structured_checked_duration_s',
                    'selected_family', 'selection', 'min_gap', 'external_min_gap',
                    'mode', 'reference_source', 'native_phase', 'native_time_scale',
                    'native_arm_reference', 'source_stamp', 'reference_hash') if key in info})
                diagnostics['solve_ms'] = round(elapsed*1000, 2)
            # A late normal return cannot qualify this cycle for delivery,
            # whether or not the solver found a candidate. Keep it a local
            # retry instead of aborting the navigation reference as infeasible.
            if elapsed > self.deadline: raise ValueError('planning_deadline_exceeded')
            with self.reference.lock:
                if (revision != self.reference.revision or
                        info.get('reference_revision', revision) != revision):
                    raise ValueError('reference_changed_during_plan')
            if result is None:
                if startup:
                    diagnostics.update(info)
                reason = 'no_feasible_candidate' if info.get('feasible') == 0 else info.get('reference_status', 'no_feasible_candidate')
                raise ValueError(reason)
            controls, prediction = result
            command = np.asarray(controls[0], dtype=float).copy()
            # Hold the reference lock through delivery: cancel/replace cannot
            # race between the solver's revision check and its ROS publication.
            with measured_delivery_lock(self, diagnostics):
                end = rospy.get_time()
                diagnostics['checked_inputs'] = checked_input_evidence(self, 'post_plan', end)
                if revision != self.reference.revision: raise ValueError('reference_changed_before_delivery')
                if self.base is None or self.joints is None:
                    raise ValueError('feedback_invalidated_during_plan')
                if not fresh(self.base_stamp, end) or not fresh(self.joint_stamp, end):
                    raise ValueError('feedback_expired_during_plan')
                if self.snapshot is None or not self.snapshot.fresh(end):
                    raise ValueError('collision_snapshot_expired_during_plan')
                if not np.isfinite(command).all():
                    raise ValueError('invalid_selected_arm_target')
                if info.get('terminal_pose_alignment'):
                    command = self.reference.terminal_pose_command(
                        np.r_[self.base, self.joints], end, self.terminal_alignment)
                    if command is None:
                        raise ValueError('reference_changed_before_delivery')
                if native_startup:
                    # The driver may still be holding its native target. Keep
                    # the base stationary until it acknowledges MPPI ownership.
                    command[:2] = 0.
                command[:2] = base_command(command, self.limits)
                limit = self.safety_speed_limit
                if limit is not None:
                    if limit <= 1e-3:
                        command[:2] = 0.
                    else:
                        command[0] = np.clip(command[0], -limit, limit)
                # Recheck the command that will actually be sent, including
                # the planner's limit; arm avoidance remains available at v=w=0.
                current = np.r_[self.base, self.joints]
                check_began = time.monotonic()
                try:
                    first_step = checked_first_step(kernel, current, command, self.snapshot, end)
                finally:
                    diagnostics['first_step_check_ms'] = round((time.monotonic()-check_began)*1000., 3)
                target = first_step[5:11]
                if np.max(np.abs(target-self.joints[:6])) > .03:
                    raise ValueError('arm_target_exceeds_current_feedback_lead')
                v, w = base_command(command, self.limits)
                delivery_time = rospy.get_time()
                diagnostics['checked_inputs'] = checked_input_evidence(self, 'before_delivery', delivery_time)
                if (not fresh(self.base_stamp, delivery_time) or
                        not fresh(self.joint_stamp, delivery_time) or
                        not self.snapshot.fresh(delivery_time)):
                    raise ValueError('inputs_expired_before_delivery')
                if self.reference.active and not startup:
                    latest = self.reference.status(delivery_time, float(current[2]))
                    if revision != self.reference.revision:
                        raise ValueError('reference_changed_before_delivery')
                    if info.get('reference_wait_hold'):
                        if info.get('terminal_pose_alignment') and (
                                latest['reference_status'] != 'native_reference_expired' or
                                self.reference.terminal_pose_command(current, delivery_time, self.terminal_alignment) is None):
                            raise ValueError('reference_changed_before_delivery')
                        if (latest['reference_status'] not in REFERENCE_WAIT_HOLD_REASONS
                                and latest['reference_status'] != 'native_reference_ready'):
                            raise ValueError(latest['reference_status']+'_before_delivery')
                    else:
                        if latest['reference_status'] != 'native_reference_ready':
                            raise ValueError(latest['reference_status']+'_before_delivery')
                        if latest['native_gear_block'] != info['native_gear_block']:
                            raise ValueError('native_gear_changed_before_delivery')
                    # Progress is a tracking clock, not a stop command. Its
                    # normal slowdown is consumed next cycle; explicit planner
                    # speed limits were already applied before collision checking.
                    if self.simulation:
                        diagnostics['delivery_native_phase'] = latest['native_phase']
                        diagnostics['delivery_native_time_scale'] = latest['native_time_scale']
                delivery_elapsed = time.monotonic()-began
                diagnostics['delivery_elapsed_ms'] = round(delivery_elapsed*1000, 3)
                if delivery_elapsed > self.deadline:
                    raise ValueError('delivery_deadline_exceeded')
                if self.execute:
                    if not self.simulation and (not self.execution_ready(delivery_time)
                            or execution_revision != self.execution_revision):
                        if (startup and not native_startup and not self.startup_enable_requested and not self.execution_latched
                                and not self.execution_ready(delivery_time)
                                and fresh(self.enable_feedback_stamp, delivery_time)
                                and delivery_time < self.enable_feedback_until
                                and self.enable_pub.get_num_connections() > 0):
                            # Validate before enabling; discard this pre-enable
                            # plan. A later cycle needs a new authority epoch
                            # and a command stamp newer than driver enable.
                            self.startup_enable_requested = True
                            self.enable_pub.publish(Bool(data=True))
                            self.reject('startup_enable_requested')
                            return
                        raise ValueError('waiting_explicit_arm_enable_or_normal_status')
                    base = Twist(); base.linear.x = v; base.angular.z = w
                    arm = JointState(); arm.header.stamp = rospy.Time.from_sec(now)
                    arm.name = list(JOINTS); arm.position = target.tolist()
                    arm.velocity = first_step[11:17].tolist()
                    self.arm_pub.publish(arm); self.base_pub.publish(base)
                    self.publish_refresh(info, end, startup)
                    self.sent_command = True
                    self._last_rejection_reason = None
                    if not info.get('reference_wait_hold'):
                        # A validated hold renews actuator authority, but does
                        # not certify tracking alignment to a recovery curve.
                        self.fault_pub.publish(Bool(data=False))
                        self.sent_navigation_goal = self.reference.goal_stamp if self.reference.active else None
                startup_ready = False
                if native_startup:
                    if self.execute:
                        if not self.native_handoff_sent:
                            self.native_handoff_stamp = delivery_time
                        self.native_handoff_sent = True
                elif startup:
                    settled = (np.max(np.abs(self.joints[:6]-self.startup_target)) < .015
                               and np.max(np.abs(self.joints[6:])) < .02)
                    self.startup_settled_since = ((self.startup_settled_since if self.startup_settled_since is not None
                                                   else delivery_time) if settled else None)
                    if (self.execute and self.startup_settled_since is not None
                            and delivery_time-self.startup_settled_since >= .3):
                        self.startup_complete = startup_ready = True
                        self.reference.finish()
                        self.anchor = self.base[:3].copy(); self.arm_anchor = self.joints[:6].copy()
                        rospy.loginfo('startup_ready: original standby pose reached; MPPI accepts navigation goals.')
            compact = {k: info[k] for k in ('mode', 'reference_source', 'native_phase',
                'native_time_scale', 'feasible', 'min_gap', 'selection', 'arm_motion',
                'structured_candidates', 'random_samples', 'nominal_external_min_gap',
                'external_min_gap', 'selected_family', 'hold_reason', 'pending_start_stamp',
                'native_phase_source') if k in info}
            compact.update(state='command_delivered' if self.execute else 'candidate_only',
                simulation=self.simulation, arm_enabled=self.arm_enabled, seed=self.seed,
                arm_execution_model='position_servo' if kernel.arm_position_servo else 'velocity_legacy',
                arm_servo_native=kernel._servo_native is not None,
                solve_ms=round(elapsed*1000, 2), base_v=v, base_w=w,
                cycle_ms=round((time.monotonic()-began)*1000, 2),
                base_safety_speed_limit=limit,
                arm_target=target.tolist(), base_anchor_error_m=float(np.linalg.norm(state[:2]-anchor[:2])))
            if startup:
                compact.update(startup_ready=startup_ready,
                    startup_error_rad=float(np.max(np.abs(current[5:11]-self.startup_target))))
                if native_startup:
                    compact['startup_mode'] = 'native_waiting_driver_takeover_ack'
            if diagnostics: compact['candidate_diagnostics'] = diagnostics
            self.info_pub.publish(String(data=json.dumps(compact)))
            if visual_due:
                self.publish_prediction(kernel, prediction, now, 1 if self.execute else 2, prediction_step)
        except (ValueError, IndexError, FloatingPointError) as exc:
            # Values below already belonged to this failed cycle. Exporting
            # them must not recompute candidates, refresh stamps or copy ESDF.
            diagnostics.update(failure_recording_context(state, snapshot, info, now))
            if self.capture_input_trace is not None:
                diagnostics.setdefault('capture_inputs', self.capture_input_trace)
                diagnostics.setdefault('checked_inputs', self.capture_input_trace)
            self.reject(str(exc), diagnostics)
            if visual_due and kernel is not None:
                self.publish_prediction(kernel, prediction, now, 0, prediction_step)

    def visualization_due(self, now):
        publisher = getattr(self, 'rollout_pub', None)
        hz = getattr(self, 'visual_hz', 0.)
        return bool(publisher is not None and hz > 0. and
                    now-self.last_visual >= 1./hz and publisher.get_num_connections() > 0)

    def publish_prediction(self, kernel, prediction, stamp, status, prediction_step=None):
        # This runs AFTER delivery/rejection. No FK, markers, map or actuator
        # work is added to the checked control path. Diagnostic failure cannot
        # reject an already delivered command.
        try:
            step = kernel.dt if prediction_step is None else prediction_step
            wire = encode_prediction(stamp, step, status,
                                     kernel.visual_rollouts, prediction)
            if wire is not None:
                self.rollout_pub.publish(Float64MultiArray(data=wire.tolist()))
                self.last_visual = time.monotonic()
        except Exception as exc:
            rospy.logwarn_throttle(5., 'MPPI prediction visualization skipped: %s', exc)

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.cycle(); rate.sleep()


if __name__ == '__main__':
    rospy.init_node('whole_body_mppi')
    WholeBodyNode().run()
