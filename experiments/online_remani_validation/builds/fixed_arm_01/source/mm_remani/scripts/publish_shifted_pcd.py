#!/usr/bin/env python3
"""Load a PCD map and publish a transformable copy for RViz, ICP and ESDF."""
import threading

import numpy as np
import rospy
import tf.transformations as tft
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField


def make_xyz_yaw_matrix(x, y, z, yaw):
    mat = np.eye(4)
    c = np.cos(yaw)
    s = np.sin(yaw)
    mat[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    mat[:3, 3] = [x, y, z]
    return mat


def pose_yaw(orientation):
    return tft.euler_from_quaternion(
        [orientation.x, orientation.y, orientation.z, orientation.w])[2]


def odom_to_mat(msg):
    trans = np.eye(4)
    q = msg.pose.pose.orientation
    trans[:3, :3] = tft.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
    p = msg.pose.pose.position
    trans[:3, 3] = [p.x, p.y, p.z]
    return trans


def mat_to_odom_msg(mat, frame_id):
    msg = Odometry()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    xyz = tft.translation_from_matrix(mat)
    quat = tft.quaternion_from_matrix(mat)
    msg.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
    return msg


def planar_xy_yaw_from_mat(mat):
    return mat[0, 3], mat[1, 3], tft.euler_from_matrix(mat)[2]


def normalize_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def make_pointcloud_msg(frame_id, seq, points, fields, point_step):
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(points)
    msg.fields = fields
    msg.point_step = point_step
    msg.row_step = msg.point_step * msg.width
    msg.is_bigendian = False
    msg.is_dense = True
    msg.data = points.astype(np.float32).tobytes()
    return msg


class ShiftedPcdPublisher:
    def __init__(self):
        self.pcd_file = rospy.get_param("~pcd_file")
        self.input_is_preprocessed = rospy.get_param("~input_is_preprocessed", False)
        if not isinstance(self.input_is_preprocessed, bool):
            raise RuntimeError("~input_is_preprocessed must be a ROS bool")
        self.robot_x = rospy.get_param("~robot_map_x", 0.0)
        self.robot_y = rospy.get_param("~robot_map_y", 0.0)
        self.robot_z = rospy.get_param("~robot_map_z", 0.0)
        self.map_roll = rospy.get_param("~map_roll", 0.0)
        self.map_pitch = rospy.get_param("~map_pitch", 0.0)
        self.map_yaw = rospy.get_param("~map_yaw", 0.0)
        self.viz_stride = max(1, int(rospy.get_param("~viz_stride", 5)))
        self.voxel_res = rospy.get_param("~voxel_res", 0.08)
        self.min_voxel_points = rospy.get_param("~min_voxel_points", 2)
        self.outlier_filter_enable = rospy.get_param("~outlier_filter_enable", False)
        self.outlier_filter_radius = max(
            1e-3, float(rospy.get_param("~outlier_filter_radius", 0.16)))
        self.outlier_filter_min_neighbors = max(
            1, int(rospy.get_param("~outlier_filter_min_neighbors", 4)))
        self.ground_min = rospy.get_param("~ground_min", 0.05)
        self.height_max = rospy.get_param("~height_max", 1.70)
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.adjust_pose_topic = rospy.get_param("~adjust_pose_topic", "/map_adjust_pose")
        self.manual_adjust_yaw = rospy.get_param("~manual_adjust_yaw", True)
        self.auto_icp_adjust = rospy.get_param("~auto_icp_adjust", False)
        self.icp_correction_topic = rospy.get_param("~icp_correction_topic", "/map_to_odom")
        self.applied_correction_topic = rospy.get_param(
            "~applied_correction_topic", "/map_correction_applied")
        self.auto_icp_adjust_mode = rospy.get_param("~auto_icp_adjust_mode", "absolute")
        self.auto_icp_adjust_gain = rospy.get_param("~auto_icp_adjust_gain", 0.12)
        self.auto_icp_adjust_min_interval = rospy.get_param("~auto_icp_adjust_min_interval", 2.0)
        self.auto_icp_adjust_min_samples = int(rospy.get_param("~auto_icp_adjust_min_samples", 4))
        self.auto_icp_adjust_min_fitness = rospy.get_param("~auto_icp_adjust_min_fitness", 0.78)
        self.auto_icp_adjust_min_translation = rospy.get_param("~auto_icp_adjust_min_translation", 0.003)
        self.auto_icp_adjust_min_yaw = rospy.get_param("~auto_icp_adjust_min_yaw", 0.002)
        self.auto_icp_adjust_max_residual_translation = rospy.get_param(
            "~auto_icp_adjust_max_residual_translation", 0.60)
        self.auto_icp_adjust_max_residual_yaw = rospy.get_param(
            "~auto_icp_adjust_max_residual_yaw", 0.12)
        self.auto_icp_adjust_max_translation_std = rospy.get_param(
            "~auto_icp_adjust_max_translation_std", 0.06)
        self.auto_icp_adjust_max_yaw_std = rospy.get_param("~auto_icp_adjust_max_yaw_std", 0.015)
        self.auto_icp_adjust_max_translation_step = rospy.get_param(
            "~auto_icp_adjust_max_translation_step", 0.012)
        self.auto_icp_adjust_max_yaw_step = rospy.get_param("~auto_icp_adjust_max_yaw_step", 0.006)
        self.auto_icp_adjust_converged_translation = rospy.get_param(
            "~auto_icp_adjust_converged_translation", 0.02)
        self.auto_icp_adjust_converged_yaw = rospy.get_param(
            "~auto_icp_adjust_converged_yaw", 0.004)
        self.auto_icp_capture_reference_on_first_match = rospy.get_param(
            "~auto_icp_capture_reference_on_first_match", False)
        self.auto_icp_adjust_worse_tolerance = rospy.get_param(
            "~auto_icp_adjust_worse_tolerance", 0.03)
        self.auto_icp_adjust_overshoot_hold_sec = rospy.get_param(
            "~auto_icp_adjust_overshoot_hold_sec", 3.0)
        self.auto_icp_adjust_yaw_error_weight = rospy.get_param(
            "~auto_icp_adjust_yaw_error_weight", 1.0)
        self.auto_icp_adjust_max_linear_speed = rospy.get_param(
            "~auto_icp_adjust_max_linear_speed", 0.08)
        self.auto_icp_adjust_max_angular_speed = rospy.get_param(
            "~auto_icp_adjust_max_angular_speed", 0.15)
        self.auto_icp_adjust_validate = rospy.get_param("~auto_icp_adjust_validate", True)
        self.auto_icp_adjust_min_improvement = rospy.get_param(
            "~auto_icp_adjust_min_improvement", 0.001)
        self.auto_icp_adjust_max_fitness_drop = rospy.get_param(
            "~auto_icp_adjust_max_fitness_drop", 0.02)

        self.lock = threading.RLock()
        self.latest_odom = None
        self.map_version = max(1, rospy.Time.now().to_nsec() & 0xffffffff)
        self.map_stamp = rospy.Time(0)
        self.last_auto_icp_adjust_time = rospy.Time(0)
        self.last_auto_icp_stamp = rospy.Time(0)
        self.icp_adjust_samples = []
        self.icp_adjust_sample_stamp_ns = None
        self.last_icp_mat = None
        self.last_icp_map_stamp_ns = None
        self.auto_icp_reference_mat = None
        self.last_auto_icp_error_vec = None
        self.pending_auto_icp_validation = None
        self.auto_icp_hold_until = rospy.Time(0)
        self.remni_msg = None
        self.viz_msg = None

        self.raw_arr, self.field_names = self.load_binary_pcd(self.pcd_file)
        if self.input_is_preprocessed and self.field_names != ["x", "y", "z"]:
            raise RuntimeError(
                "~input_is_preprocessed=true requires an XYZ-only binary PCD; "
                "got fields: %s" % self.field_names)
        if self.input_is_preprocessed and not np.isfinite(self.raw_arr).all():
            raise RuntimeError("Preprocessed PCD contains non-finite XYZ values")
        self.intensity_idx = self.find_intensity_index(self.field_names)
        self.T_world_map = self.initial_transform()

        self.pub_remni = rospy.Publisher(
            "/map_generator/global_cloud", PointCloud2, queue_size=1, latch=True)
        self.pub_icp_map = rospy.Publisher("/map", PointCloud2, queue_size=1, latch=True)
        self.pub_viz = rospy.Publisher(
            "/map_generator/global_cloud_viz", PointCloud2, queue_size=1, latch=True)
        self.pub_applied_correction = rospy.Publisher(
            self.applied_correction_topic, Odometry, queue_size=1)

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber(
            self.adjust_pose_topic, PoseWithCovarianceStamped, self.adjust_pose_cb, queue_size=1)
        rospy.Subscriber(self.icp_correction_topic, Odometry, self.icp_correction_cb, queue_size=1)

        self.rebuild_messages_locked()
        self.publish_locked()
        rospy.loginfo(
            "2D Pose Estimate map adjustment enabled on %s; odom topic: %s",
            self.adjust_pose_topic, self.odom_topic)
        rospy.loginfo(
            "Auto ICP map adjustment: enabled=%s correction_topic=%s applied_topic=%s gain=%.2f",
            self.auto_icp_adjust, self.icp_correction_topic,
            self.applied_correction_topic, self.auto_icp_adjust_gain)
        rospy.loginfo(
            "Auto ICP gate: samples=%d min_fitness=%.2f max_step=%.3fm max_yaw=%.3frad max_std=%.3fm/%.3frad",
            self.auto_icp_adjust_min_samples, self.auto_icp_adjust_min_fitness,
            self.auto_icp_adjust_max_translation_step, self.auto_icp_adjust_max_yaw_step,
            self.auto_icp_adjust_max_translation_std, self.auto_icp_adjust_max_yaw_std)
        rospy.loginfo(
            "Auto ICP mode=%s capture_first=%s converged_deadband=%.3fm/%.3frad",
            self.auto_icp_adjust_mode, self.auto_icp_capture_reference_on_first_match,
            self.auto_icp_adjust_converged_translation,
            self.auto_icp_adjust_converged_yaw)
        rospy.loginfo(
            "Auto ICP validation=%s min_improvement=%.3fm max_fitness_drop=%.3f",
            self.auto_icp_adjust_validate, self.auto_icp_adjust_min_improvement,
            self.auto_icp_adjust_max_fitness_drop)
        if self.input_is_preprocessed:
            rospy.loginfo(
                "Preprocessed XYZ map enabled: %d points; runtime only applies the "
                "robot_map/map_rpy rigid transform. Height, outlier, density and "
                "voxel filters are bypassed. Input coordinates must be in the "
                "pre-transform map-input frame.",
                len(self.raw_arr))
        if self.intensity_idx is None:
            rospy.loginfo(
                "PCD has no intensity field; /map_generator/global_cloud_viz "
                "uses world Z as a synthetic intensity for RViz compatibility.")

    @staticmethod
    def load_binary_pcd(pcd_file):
        with open(pcd_file, "rb") as f:
            raw = f.read()

        marker = b"DATA binary\n"
        header_end = raw.rfind(marker)
        if header_end < 0:
            raise RuntimeError("Not a binary PCD file")
        header_end += len(marker)
        header = raw[:header_end].decode()
        data = raw[header_end:]

        n_fields = 3
        field_names = []
        for line in header.split("\n"):
            if line.startswith("FIELDS"):
                field_names = line.split()[1:]
                n_fields = len(field_names)
                break

        arr = np.frombuffer(data, dtype=np.float32).copy()
        extra = len(arr) % n_fields
        if extra:
            arr = arr[:-extra]
        arr = arr.reshape(-1, n_fields)
        rospy.loginfo("Loaded %d points, fields: %s", len(arr), field_names)
        return arr, field_names

    @staticmethod
    def find_intensity_index(field_names):
        for i, name in enumerate(field_names):
            if name in ("intensity", "scalar_intensity", "Intensity"):
                return i
        return None

    def initial_transform(self):
        r, p, y = np.radians(self.map_roll), np.radians(self.map_pitch), np.radians(self.map_yaw)
        rx = tft.euler_matrix(r, 0.0, 0.0)
        ry = tft.euler_matrix(0.0, p, 0.0)
        rz = tft.euler_matrix(0.0, 0.0, y)
        transform = rz @ ry @ rx
        transform[:3, 3] = [-float(self.robot_x), -float(self.robot_y), -float(self.robot_z)]
        rospy.loginfo(
            "Initial map offset: robot_map=(%.3f, %.3f, %.3f), rpy=(%.2f, %.2f, %.2f) deg",
            self.robot_x, self.robot_y, self.robot_z,
            self.map_roll, self.map_pitch, self.map_yaw)
        return transform

    def odom_cb(self, msg):
        with self.lock:
            self.latest_odom = msg

    def adjust_pose_cb(self, msg):
        with self.lock:
            if self.latest_odom is None:
                rospy.logwarn("Ignore map adjustment: waiting for odom on %s", self.odom_topic)
                return

            odom_pose = self.latest_odom.pose.pose
            robot_yaw = pose_yaw(odom_pose.orientation)
            clicked_yaw = pose_yaw(msg.pose.pose.orientation)

            if not self.manual_adjust_yaw:
                clicked_yaw = robot_yaw

            T_robot = make_xyz_yaw_matrix(
                odom_pose.position.x, odom_pose.position.y, 0.0, robot_yaw)
            T_clicked = make_xyz_yaw_matrix(
                msg.pose.pose.position.x, msg.pose.pose.position.y, 0.0, clicked_yaw)
            correction = T_robot @ np.linalg.inv(T_clicked)
            self.apply_map_correction_locked(
                correction, "manual " + self.adjust_pose_topic, reset_icp_reference=True)

    def icp_correction_cb(self, msg):
        with self.lock:
            if self.latest_odom is None:
                return

            self.update_auto_icp_params_locked()
            if not self.auto_icp_adjust:
                return

            mode = str(self.auto_icp_adjust_mode).lower()
            now = rospy.Time.now()
            if self.auto_icp_hold_until > now:
                return

            if msg.header.stamp != rospy.Time(0) and msg.header.stamp <= self.last_auto_icp_stamp:
                return

            try:
                icp_map_stamp_ns = int(msg.child_frame_id)
            except (TypeError, ValueError):
                rospy.logwarn_throttle(
                    2.0,
                    "Skip auto ICP map adjust: /map_to_odom has no map-stamp tag")
                return

            current_map_stamp_ns = self.map_stamp.to_nsec()
            if icp_map_stamp_ns != current_map_stamp_ns:
                rospy.loginfo_throttle(
                    2.0,
                    "Skip stale auto ICP map adjust: icp_map_stamp=%d current_map_stamp=%d",
                    icp_map_stamp_ns, current_map_stamp_ns)
                return

            fitness = msg.pose.covariance[0]
            if self.auto_icp_adjust_min_fitness > 0:
                if fitness <= 0.0:
                    rospy.logwarn_throttle(
                        2.0,
                        "Skip auto ICP map adjust: /map_to_odom has no fitness tag")
                    return
                if fitness < self.auto_icp_adjust_min_fitness:
                    rospy.loginfo_throttle(
                        2.0,
                        "Skip auto ICP map adjust: fitness %.3f < %.3f",
                        fitness, self.auto_icp_adjust_min_fitness)
                    return

            current_icp_mat = odom_to_mat(msg)
            self.last_icp_mat = current_icp_mat.copy()
            self.last_icp_map_stamp_ns = icp_map_stamp_ns

            if mode == "reference" and self.auto_icp_reference_mat is None:
                if self.auto_icp_capture_reference_on_first_match:
                    self.auto_icp_reference_mat = current_icp_mat.copy()
                    dx_ref, dy_ref, dyaw_ref = planar_xy_yaw_from_mat(self.auto_icp_reference_mat)
                    rospy.logwarn(
                        "Capture auto ICP reference: map_to_odom=(%.3fm, %.3fm, %.3frad). "
                        "Future auto map corrections will keep this reference instead of driving it to zero.",
                        dx_ref, dy_ref, dyaw_ref)
                return

            if mode == "reference":
                correction_full = self.auto_icp_reference_mat @ np.linalg.inv(current_icp_mat)
            else:
                correction_full = np.linalg.inv(current_icp_mat)
            dx_full, dy_full, dyaw_full = planar_xy_yaw_from_mat(correction_full)
            dyaw_full = normalize_angle(dyaw_full)
            current_error_vec = np.array([
                dx_full,
                dy_full,
                dyaw_full * self.auto_icp_adjust_yaw_error_weight,
            ])
            current_error_norm = float(np.linalg.norm(current_error_vec))

            if self.pending_auto_icp_validation is not None:
                pending = self.pending_auto_icp_validation
                if pending["map_stamp_ns"] == current_map_stamp_ns:
                    improvement = pending["before_error_norm"] - current_error_norm
                    fitness_drop = pending["before_fitness"] - fitness
                    converged = (
                        np.hypot(dx_full, dy_full) < self.auto_icp_adjust_converged_translation and
                        abs(dyaw_full) < self.auto_icp_adjust_converged_yaw)
                    enough_improvement = (
                        improvement >= self.auto_icp_adjust_min_improvement or converged)
                    fitness_ok = (
                        self.auto_icp_adjust_max_fitness_drop <= 0.0 or
                        fitness_drop <= self.auto_icp_adjust_max_fitness_drop)

                    if not enough_improvement or not fitness_ok:
                        rollback = np.linalg.inv(pending["correction"])
                        self.pending_auto_icp_validation = None
                        self.last_auto_icp_error_vec = None
                        self.auto_icp_hold_until = now + rospy.Duration(
                            self.auto_icp_adjust_overshoot_hold_sec)
                        rospy.logwarn(
                            "Rollback auto ICP map correction: improvement=%.3fm fitness_drop=%.3f "
                            "(before=%.3fm after=%.3fm), hold %.1fs",
                            improvement, fitness_drop, pending["before_error_norm"],
                            current_error_norm, self.auto_icp_adjust_overshoot_hold_sec)
                        self.apply_map_correction_locked(rollback, "rollback auto ICP")
                        return

                    rospy.loginfo(
                        "Validate auto ICP map correction: improvement=%.3fm fitness_drop=%.3f "
                        "(before=%.3fm after=%.3fm)",
                        improvement, fitness_drop, pending["before_error_norm"], current_error_norm)
                    self.pending_auto_icp_validation = None
                    self.last_auto_icp_error_vec = current_error_vec
                    self.last_auto_icp_stamp = msg.header.stamp
                    self.icp_adjust_samples = []
                    return
                elif current_map_stamp_ns > pending["map_stamp_ns"]:
                    self.pending_auto_icp_validation = None

            twist = self.latest_odom.twist.twist
            linear_speed = np.hypot(twist.linear.x, twist.linear.y)
            angular_speed = abs(twist.angular.z)
            if (linear_speed > self.auto_icp_adjust_max_linear_speed or
                    angular_speed > self.auto_icp_adjust_max_angular_speed):
                rospy.loginfo_throttle(
                    2.0,
                    "Skip auto ICP map adjust: robot moving v=%.3f w=%.3f",
                    linear_speed, angular_speed)
                return

            dx, dy, dyaw = dx_full, dy_full, dyaw_full
            trans_norm = np.hypot(dx, dy)

            if (trans_norm < self.auto_icp_adjust_converged_translation and
                    abs(dyaw) < self.auto_icp_adjust_converged_yaw):
                self.icp_adjust_samples = []
                rospy.loginfo_throttle(
                    2.0,
                    "Auto ICP map adjust converged: reference error %.3fm %.3frad",
                    trans_norm, dyaw)
                return

            if (self.auto_icp_adjust_max_residual_translation > 0 and
                    trans_norm > self.auto_icp_adjust_max_residual_translation):
                self.icp_adjust_samples = []
                rospy.loginfo_throttle(
                    2.0,
                    "Skip auto ICP map adjust: residual %.3fm too large",
                    trans_norm)
                return
            if (self.auto_icp_adjust_max_residual_yaw > 0 and
                    abs(dyaw) > self.auto_icp_adjust_max_residual_yaw):
                self.icp_adjust_samples = []
                rospy.loginfo_throttle(
                    2.0,
                    "Skip auto ICP map adjust: residual yaw %.3frad too large",
                    dyaw)
                return

            if trans_norm < self.auto_icp_adjust_min_translation and abs(dyaw) < self.auto_icp_adjust_min_yaw:
                self.icp_adjust_samples = []
                return

            if self.icp_adjust_sample_stamp_ns != icp_map_stamp_ns:
                self.icp_adjust_samples = []
                self.icp_adjust_sample_stamp_ns = icp_map_stamp_ns
            self.icp_adjust_samples.append([dx, dy, dyaw])
            self.icp_adjust_samples = self.icp_adjust_samples[-max(1, self.auto_icp_adjust_min_samples):]

            if len(self.icp_adjust_samples) < self.auto_icp_adjust_min_samples:
                return

            samples = np.array(self.icp_adjust_samples, dtype=np.float64)
            yaw_samples = np.unwrap(samples[:, 2])
            trans_std = float(np.linalg.norm(np.std(samples[:, :2], axis=0)))
            yaw_std = float(np.std(yaw_samples))
            if (trans_std > self.auto_icp_adjust_max_translation_std or
                    yaw_std > self.auto_icp_adjust_max_yaw_std):
                rospy.loginfo_throttle(
                    2.0,
                    "Wait for stable auto ICP map adjust: trans_std=%.3fm yaw_std=%.3frad",
                    trans_std, yaw_std)
                return

            if (now - self.last_auto_icp_adjust_time).to_sec() < self.auto_icp_adjust_min_interval:
                return

            mean_dx = float(np.mean(samples[:, 0]))
            mean_dy = float(np.mean(samples[:, 1]))
            mean_dyaw = normalize_angle(float(np.mean(yaw_samples)))
            current_error_vec = np.array([
                mean_dx,
                mean_dy,
                mean_dyaw * self.auto_icp_adjust_yaw_error_weight,
            ])
            current_error_norm = float(np.linalg.norm(current_error_vec))

            if self.last_auto_icp_error_vec is not None:
                last_error_norm = float(np.linalg.norm(self.last_auto_icp_error_vec))
                if (last_error_norm > 1e-6 and
                        current_error_norm > last_error_norm + self.auto_icp_adjust_worse_tolerance):
                    rospy.logwarn_throttle(
                        2.0,
                        "Hold auto ICP map adjust: residual got worse %.3f -> %.3f, avoid over-correction",
                        last_error_norm, current_error_norm)
                    self.icp_adjust_samples = []
                    self.auto_icp_hold_until = now + rospy.Duration(self.auto_icp_adjust_overshoot_hold_sec)
                    return

            dx = mean_dx * self.auto_icp_adjust_gain
            dy = mean_dy * self.auto_icp_adjust_gain
            dyaw = mean_dyaw * self.auto_icp_adjust_gain
            trans_norm = np.hypot(dx, dy)

            if self.auto_icp_adjust_max_translation_step > 0 and trans_norm > self.auto_icp_adjust_max_translation_step:
                scale = self.auto_icp_adjust_max_translation_step / trans_norm
                dx *= scale
                dy *= scale
            if self.auto_icp_adjust_max_yaw_step > 0:
                dyaw = float(np.clip(
                    dyaw,
                    -self.auto_icp_adjust_max_yaw_step,
                    self.auto_icp_adjust_max_yaw_step))

            if np.hypot(dx, dy) < 1e-4 and abs(dyaw) < 1e-4:
                return

            correction = make_xyz_yaw_matrix(dx, dy, 0.0, dyaw)
            before_error_norm = current_error_norm
            before_fitness = fitness
            self.apply_map_correction_locked(correction, "auto ICP")
            if self.auto_icp_adjust_validate:
                self.pending_auto_icp_validation = {
                    "map_stamp_ns": self.map_stamp.to_nsec(),
                    "before_error_norm": before_error_norm,
                    "before_fitness": before_fitness,
                    "correction": correction.copy(),
                }
                self.last_auto_icp_error_vec = None
            else:
                self.last_auto_icp_error_vec = current_error_vec
            self.last_auto_icp_adjust_time = now
            self.last_auto_icp_stamp = msg.header.stamp
            self.icp_adjust_samples = []

    def update_auto_icp_params_locked(self):
        self.auto_icp_adjust = rospy.get_param("~auto_icp_adjust", self.auto_icp_adjust)
        self.auto_icp_adjust_mode = rospy.get_param("~auto_icp_adjust_mode", self.auto_icp_adjust_mode)
        self.auto_icp_adjust_gain = rospy.get_param("~auto_icp_adjust_gain", self.auto_icp_adjust_gain)
        self.auto_icp_adjust_min_interval = rospy.get_param(
            "~auto_icp_adjust_min_interval", self.auto_icp_adjust_min_interval)
        self.auto_icp_adjust_min_samples = max(
            1, int(rospy.get_param("~auto_icp_adjust_min_samples", self.auto_icp_adjust_min_samples)))
        self.auto_icp_adjust_min_fitness = rospy.get_param(
            "~auto_icp_adjust_min_fitness", self.auto_icp_adjust_min_fitness)
        self.auto_icp_adjust_min_translation = rospy.get_param(
            "~auto_icp_adjust_min_translation", self.auto_icp_adjust_min_translation)
        self.auto_icp_adjust_min_yaw = rospy.get_param(
            "~auto_icp_adjust_min_yaw", self.auto_icp_adjust_min_yaw)
        self.auto_icp_adjust_max_residual_translation = rospy.get_param(
            "~auto_icp_adjust_max_residual_translation", self.auto_icp_adjust_max_residual_translation)
        self.auto_icp_adjust_max_residual_yaw = rospy.get_param(
            "~auto_icp_adjust_max_residual_yaw", self.auto_icp_adjust_max_residual_yaw)
        self.auto_icp_adjust_max_translation_std = rospy.get_param(
            "~auto_icp_adjust_max_translation_std", self.auto_icp_adjust_max_translation_std)
        self.auto_icp_adjust_max_yaw_std = rospy.get_param(
            "~auto_icp_adjust_max_yaw_std", self.auto_icp_adjust_max_yaw_std)
        self.auto_icp_adjust_max_translation_step = rospy.get_param(
            "~auto_icp_adjust_max_translation_step", self.auto_icp_adjust_max_translation_step)
        self.auto_icp_adjust_max_yaw_step = rospy.get_param(
            "~auto_icp_adjust_max_yaw_step", self.auto_icp_adjust_max_yaw_step)
        self.auto_icp_adjust_converged_translation = rospy.get_param(
            "~auto_icp_adjust_converged_translation", self.auto_icp_adjust_converged_translation)
        self.auto_icp_adjust_converged_yaw = rospy.get_param(
            "~auto_icp_adjust_converged_yaw", self.auto_icp_adjust_converged_yaw)
        self.auto_icp_capture_reference_on_first_match = rospy.get_param(
            "~auto_icp_capture_reference_on_first_match",
            self.auto_icp_capture_reference_on_first_match)
        self.auto_icp_adjust_worse_tolerance = rospy.get_param(
            "~auto_icp_adjust_worse_tolerance", self.auto_icp_adjust_worse_tolerance)
        self.auto_icp_adjust_overshoot_hold_sec = rospy.get_param(
            "~auto_icp_adjust_overshoot_hold_sec", self.auto_icp_adjust_overshoot_hold_sec)
        self.auto_icp_adjust_yaw_error_weight = rospy.get_param(
            "~auto_icp_adjust_yaw_error_weight", self.auto_icp_adjust_yaw_error_weight)
        self.auto_icp_adjust_max_linear_speed = rospy.get_param(
            "~auto_icp_adjust_max_linear_speed", self.auto_icp_adjust_max_linear_speed)
        self.auto_icp_adjust_max_angular_speed = rospy.get_param(
            "~auto_icp_adjust_max_angular_speed", self.auto_icp_adjust_max_angular_speed)
        self.auto_icp_adjust_validate = rospy.get_param(
            "~auto_icp_adjust_validate", self.auto_icp_adjust_validate)
        self.auto_icp_adjust_min_improvement = rospy.get_param(
            "~auto_icp_adjust_min_improvement", self.auto_icp_adjust_min_improvement)
        self.auto_icp_adjust_max_fitness_drop = rospy.get_param(
            "~auto_icp_adjust_max_fitness_drop", self.auto_icp_adjust_max_fitness_drop)

    def update_runtime_params_locked(self):
        yaw = np.degrees(tft.euler_from_matrix(self.T_world_map)[2])
        rospy.set_param("~robot_map_x", -float(self.T_world_map[0, 3]))
        rospy.set_param("~robot_map_y", -float(self.T_world_map[1, 3]))
        rospy.set_param("~robot_map_z", -float(self.T_world_map[2, 3]))
        rospy.set_param("~map_yaw", float(yaw))
        rospy.logwarn(
            "Current map launch-equivalent offset: robot_map_x=%.3f robot_map_y=%.3f robot_map_z=%.3f map_yaw=%.3fdeg",
            -self.T_world_map[0, 3], -self.T_world_map[1, 3],
            -self.T_world_map[2, 3], yaw)

    def apply_map_correction_locked(self, correction, reason, reset_icp_reference=False):
        previous_map_stamp_ns = self.map_stamp.to_nsec()
        if reset_icp_reference:
            if self.last_icp_mat is not None and self.last_icp_map_stamp_ns == previous_map_stamp_ns:
                self.auto_icp_reference_mat = correction @ self.last_icp_mat
                dx_ref, dy_ref, dyaw_ref = planar_xy_yaw_from_mat(self.auto_icp_reference_mat)
                rospy.logwarn(
                    "Reset auto ICP reference after %s: map_to_odom=(%.3fm, %.3fm, %.3frad)",
                    reason, dx_ref, dy_ref, dyaw_ref)
            else:
                self.auto_icp_reference_mat = None
                rospy.logwarn(
                    "Reset auto ICP reference after %s: waiting for next valid /map_to_odom",
                    reason)

        self.T_world_map = correction @ self.T_world_map
        self.map_version = (self.map_version + 1) & 0xffffffff
        if self.map_version == 0:
            self.map_version = 1

        dx, dy, dyaw = planar_xy_yaw_from_mat(correction)
        rospy.logwarn(
            "Apply %s map correction: delta=(%.3fm, %.3fm, %.3frad), version=%d",
            reason, dx, dy, dyaw, self.map_version)

        self.update_runtime_params_locked()
        self.rebuild_messages_locked()
        self.publish_locked()
        self.pub_applied_correction.publish(
            mat_to_odom_msg(correction, "world"))
        self.last_auto_icp_adjust_time = rospy.Time.now()
        self.icp_adjust_samples = []
        self.icp_adjust_sample_stamp_ns = None
        if reset_icp_reference:
            self.last_auto_icp_error_vec = None
            self.pending_auto_icp_validation = None
            self.auto_icp_hold_until = rospy.Time(0)

    def transform_raw_points_locked(self):
        arr = self.raw_arr.copy()
        xyz = arr[:, :3]
        arr[:, :3] = (self.T_world_map[:3, :3] @ xyz.T).T + self.T_world_map[:3, 3]
        return arr

    def filter_and_downsample(self, arr):
        n_before = len(arr)
        mask = (arr[:, 2] >= self.ground_min) & (arr[:, 2] <= self.height_max)
        arr = arr[mask]
        rospy.loginfo(
            "Height filter: %d -> %d points (z in [%.2f, %.2f])",
            n_before, len(arr), self.ground_min, self.height_max)

        if self.outlier_filter_enable and len(arr) > 0:
            n_before_outlier = len(arr)
            arr = self.filter_sparse_outliers(arr)
            rospy.loginfo(
                "PCD outlier filter: %d -> %d points (radius %.2fm, min %d neighbors)",
                n_before_outlier, len(arr), self.outlier_filter_radius,
                self.outlier_filter_min_neighbors)

        if self.min_voxel_points > 1 and len(arr) > 0:
            inv_res = 1.0 / self.voxel_res
            voxel_keys = (arr[:, :3] * inv_res).astype(np.int32)
            max_coord = int(1.0 / self.voxel_res * 100)
            keys = (voxel_keys[:, 0].astype(np.int64) * max_coord * max_coord +
                    voxel_keys[:, 1].astype(np.int64) * max_coord +
                    voxel_keys[:, 2].astype(np.int64))
            _, inverse, counts = np.unique(keys, return_inverse=True, return_counts=True)
            dense_mask = counts[inverse] >= self.min_voxel_points
            n_before_dense = len(arr)
            arr = arr[dense_mask]
            rospy.loginfo(
                "Voxel density filter: %d -> %d points (min %d pts / %.2fm voxel)",
                n_before_dense, len(arr), self.min_voxel_points, self.voxel_res)

        inv_res = 1.0 / self.voxel_res
        voxel_keys = (arr[:, :3] * inv_res).astype(np.int32)
        max_coord = int(1.0 / self.voxel_res * 100)
        keys = (voxel_keys[:, 0].astype(np.int64) * max_coord * max_coord +
                voxel_keys[:, 1].astype(np.int64) * max_coord +
                voxel_keys[:, 2].astype(np.int64))
        _, unique_idx = np.unique(keys, return_index=True)
        remni_arr = arr[unique_idx]
        rospy.loginfo(
            "Voxel downsampled %d -> %d points (%.2fm grid), version=%d",
            len(arr), len(remni_arr), self.voxel_res, self.map_version)
        return arr, remni_arr

    def filter_sparse_outliers(self, arr):
        voxel_keys = np.floor(arr[:, :3] / self.outlier_filter_radius).astype(np.int32)
        unique_keys, inverse, counts = np.unique(
            voxel_keys, axis=0, return_inverse=True, return_counts=True)
        count_by_key = {tuple(key): int(count) for key, count in zip(unique_keys, counts)}
        keep_voxel = np.zeros(len(unique_keys), dtype=bool)

        neighbor_offsets = [
            (dx, dy, dz)
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
            for dz in (-1, 0, 1)
        ]
        for idx, key in enumerate(unique_keys):
            key_tuple = tuple(key)
            neighbor_count = 0
            for offset in neighbor_offsets:
                neighbor_key = (
                    key_tuple[0] + offset[0],
                    key_tuple[1] + offset[1],
                    key_tuple[2] + offset[2],
                )
                neighbor_count += count_by_key.get(neighbor_key, 0)
                if neighbor_count >= self.outlier_filter_min_neighbors:
                    keep_voxel[idx] = True
                    break

        return arr[keep_voxel[inverse]]

    def rebuild_messages_locked(self):
        arr = self.transform_raw_points_locked()
        if self.input_is_preprocessed:
            # The offline artifact already contains the exact planner points.  Keep
            # all runtime corrections cheap and rigid: transform, serialize, publish.
            remni_arr = arr[:, :3]
        else:
            arr, remni_arr = self.filter_and_downsample(arr)

        xyz_fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
        ]
        self.remni_msg = make_pointcloud_msg(
            "world", self.map_version, remni_arr[:, :3], xyz_fields, 12)

        viz = arr[::self.viz_stride]
        if self.intensity_idx is not None:
            viz_data = np.zeros((len(viz), 4), dtype=np.float32)
            viz_data[:, :3] = viz[:, :3]
            viz_data[:, 3] = viz[:, self.intensity_idx]
            viz_fields = xyz_fields + [PointField("intensity", 12, PointField.FLOAT32, 1)]
            self.viz_msg = make_pointcloud_msg(
                "world", self.map_version, viz_data, viz_fields, 16)
        else:
            # Keep the planner and ICP clouds compact XYZ-only, but retain the
            # historical RViz contract on the visualization-only topic.  The
            # existing distributed RViz setup selects the Intensity transformer;
            # using Z as its scalar keeps that setup usable without pretending
            # that the offline planner artifact contains reflectivity.
            viz_data = np.zeros((len(viz), 4), dtype=np.float32)
            viz_data[:, :3] = viz[:, :3]
            viz_data[:, 3] = viz[:, 2]
            viz_fields = xyz_fields + [
                PointField("intensity", 12, PointField.FLOAT32, 1)]
            self.viz_msg = make_pointcloud_msg(
                "world", self.map_version, viz_data, viz_fields, 16)

    def publish_locked(self):
        now = rospy.Time.now()
        self.map_stamp = now
        self.remni_msg.header.stamp = now
        self.viz_msg.header.stamp = now
        self.pub_remni.publish(self.remni_msg)
        self.pub_icp_map.publish(self.remni_msg)
        self.pub_viz.publish(self.viz_msg)
        rospy.loginfo(
            "Published shifted map version %d to /map_generator/global_cloud, /map and /map_generator/global_cloud_viz",
            self.map_version)

    def spin(self):
        rospy.spin()


def main():
    rospy.init_node("publish_shifted_pcd")
    try:
        node = ShiftedPcdPublisher()
    except Exception as exc:
        rospy.logerr("Failed to start shifted PCD publisher: %s", exc)
        return
    node.spin()


if __name__ == "__main__":
    main()
