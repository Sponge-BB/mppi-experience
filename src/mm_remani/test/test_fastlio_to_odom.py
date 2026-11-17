#!/usr/bin/env python3
"""Offline bridge checks; no ROS master, publishers, CAN, or NumPy required."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


def quaternion(x=0.0, y=0.0, z=0.0, w=1.0):
    return SimpleNamespace(x=x, y=y, z=z, w=w)


def vector():
    return SimpleNamespace(x=0.0, y=0.0, z=0.0)


class Time(float):
    @staticmethod
    def now():
        return Time(100.0)

    def __sub__(self, other):
        return Time(float(self) - float(other))

    def to_sec(self):
        return float(self)


def transform():
    return SimpleNamespace(
        header=SimpleNamespace(stamp=Time(0), frame_id=''), child_frame_id='',
        transform=SimpleNamespace(translation=vector(), rotation=quaternion()))


def odometry():
    return SimpleNamespace(
        header=SimpleNamespace(stamp=Time(1), frame_id='camera_init'),
        child_frame_id='body',
        pose=SimpleNamespace(
            pose=SimpleNamespace(position=vector(), orientation=quaternion()),
            covariance=[0.0] * 36),
        twist=SimpleNamespace(
            twist=SimpleNamespace(linear=vector(), angular=vector()),
            covariance=[0.0] * 36))


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def rpy_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return quaternion(sr * cp * cy - cr * sp * sy,
                      cr * sp * cy + sr * cp * sy,
                      cr * cp * sy - sr * sp * cy,
                      cr * cp * cy + sr * sp * sy)


def rotate(matrix, point):
    return tuple(sum(row[k] * point[k] for k in range(3)) for row in matrix)


class FastLIOBridgeTests(unittest.TestCase):
    def setUp(self):
        self.params = {}
        self.publisher = Mock()
        self.dynamic_tf = Mock()
        self.static_tf = Mock()
        self.static_factory = Mock(return_value=self.static_tf)

        def get_param(name, *default):
            if name in self.params:
                return self.params[name]
            if default:
                return default[0]
            raise KeyError(name)

        rospy = SimpleNamespace(
            Time=Time, get_param=get_param,
            Publisher=Mock(return_value=self.publisher), Subscriber=Mock(),
            loginfo=Mock(), logerr_throttle=Mock())
        source = Path(__file__).resolve().parents[1] / 'scripts/fastlio_to_odom.py'
        tree = ast.parse(source.read_text())
        bridge_class = next(node for node in tree.body
                            if isinstance(node, ast.ClassDef))
        namespace = {
            'math': math, 'rospy': rospy, 'Quaternion': quaternion,
            'TransformStamped': transform, 'Odometry': odometry,
            'tf2_ros': SimpleNamespace(
                TransformBroadcaster=Mock(return_value=self.dynamic_tf),
                StaticTransformBroadcaster=self.static_factory)}
        exec(compile(ast.Module(body=[bridge_class], type_ignores=[]),
                     str(source), 'exec'), namespace)
        self.bridge_class = namespace['FastLIOToOdomBridge']

    def enable_calibration(self, translation, rotation):
        self.params.update({
            '~publish_lidar_tf': True,
            '/mapping/extrinsic_T': list(translation),
            '/mapping/extrinsic_R': [value for row in rotation for value in row]})

    def test_default_entry_keeps_original_planar_pose_and_publishes_no_lidar_tf(self):
        bridge = self.bridge_class()
        message = odometry()
        message.pose.pose.position.x = 2.0
        message.pose.pose.position.y = 3.0
        message.pose.pose.orientation = rpy_quaternion(0.2, -0.1, 0.7)
        bridge.cb(message)
        pose = self.publisher.publish.call_args[0][0].pose.pose
        self.assertAlmostEqual(pose.position.x, 2.0 - 0.28 * math.cos(0.7))
        self.assertAlmostEqual(pose.position.y, 3.0 - 0.28 * math.sin(0.7))
        self.assertEqual(pose.position.z, 0.115)
        self.assertAlmostEqual(pose.orientation.z, math.sin(0.35))
        self.static_factory.assert_not_called()
        self.assertEqual(message.pose.pose.position.x, 2.0)

    def test_full_imu_pose_and_calibration_preserve_world_surface_and_planar_base(self):
        translation = (-0.011, -0.02329, 0.04412)
        rotation = rpy_matrix(0.12, -0.08, 0.19)
        self.enable_calibration(translation, rotation)
        bridge = self.bridge_class()
        message = odometry()
        body_position = (1.2, -2.1, 0.06)
        body_rotation = rpy_matrix(0.18, -0.13, -1.1)
        message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = body_position
        message.pose.pose.orientation = rpy_quaternion(0.18, -0.13, -1.1)
        bridge.cb(message)

        static = self.static_tf.sendTransform.call_args[0][0]
        self.assertEqual((static.header.frame_id, static.child_frame_id),
                         ('body', 'livox_frame'))
        point_lidar = (2.4, -0.8, 0.9)
        calibrated = rotate(rotation, point_lidar)
        point_imu = tuple(a + b for a, b in zip(calibrated, translation))
        expected_world = tuple(a + b for a, b in
                               zip(rotate(body_rotation, point_imu), body_position))
        tf_calibrated = bridge.rotate_vector(static.transform.rotation, point_lidar)
        tf_point_imu = tuple(a + b for a, b in zip(tf_calibrated, translation))
        actual_world = tuple(a + b for a, b in zip(
            bridge.rotate_vector(message.pose.pose.orientation, tf_point_imu), body_position))
        for actual, expected in zip(actual_world, expected_world):
            self.assertAlmostEqual(actual, expected, places=10)

        # The chassis uses the LiDAR heading, including the mounting rotation,
        # while the perception TF retains roll/pitch and the actual ray origin.
        lidar_origin = tuple(a + b for a, b in
                             zip(body_position, rotate(body_rotation, translation)))
        forward = rotate(body_rotation, rotate(rotation, (1.0, 0.0, 0.0)))
        yaw = math.atan2(forward[1], forward[0])
        result = self.publisher.publish.call_args[0][0]
        self.assertAlmostEqual(result.pose.pose.position.x,
                               lidar_origin[0] - 0.28 * math.cos(yaw))
        self.assertAlmostEqual(result.pose.pose.position.y,
                               lidar_origin[1] - 0.28 * math.sin(yaw))
        self.assertEqual(result.pose.pose.position.z, 0.115)
        self.assertEqual(result.pose.pose.orientation.x, 0.0)
        self.assertEqual(result.pose.pose.orientation.y, 0.0)
        self.assertAlmostEqual(result.pose.pose.orientation.z, math.sin(yaw / 2))
        self.assertEqual(result.header.stamp, message.header.stamp)
        dynamic = self.dynamic_tf.sendTransform.call_args[0][0]
        self.assertEqual((dynamic.header.frame_id, dynamic.child_frame_id),
                         ('odom', 'base_link'))
        self.assertEqual(self.static_tf.sendTransform.call_count, 1)

    def test_calibration_handles_half_turns_without_changing_rotation(self):
        for angles in ((math.pi, 0.0, 0.0), (0.0, math.pi, 0.0),
                       (0.0, 0.0, math.pi), (0.0, 0.0, 0.0)):
            with self.subTest(angles=angles):
                matrix = rpy_matrix(*angles)
                _, q = self.bridge_class.validate_lidar_extrinsic(
                    (0.0, 0.0, 0.0), [value for row in matrix for value in row])
                point = (0.3, -0.7, 1.2)
                for actual, expected in zip(
                        self.bridge_class.rotate_vector(q, point), rotate(matrix, point)):
                    self.assertAlmostEqual(actual, expected, places=10)

    def test_invalid_or_missing_calibration_cannot_publish_an_invented_transform(self):
        identity = rpy_matrix(0.0, 0.0, 0.0)
        cases = [((float('nan'), 0, 0), identity),
                 ((0, 0, 0), ((1, 0, 0), (0, 1, 0), (0, 0, -1))),
                 ((0, 0, 0), ((2, 0, 0), (0, 1, 0), (0, 0, 1))),
                 ((0, 0), identity)]
        for translation, rotation in cases:
            with self.subTest(translation=translation, rotation=rotation):
                self.enable_calibration(translation, rotation)
                with self.assertRaises(ValueError):
                    self.bridge_class()
        self.params = {'~publish_lidar_tf': True}
        with self.assertRaises(KeyError):
            self.bridge_class()
        self.static_tf.sendTransform.assert_not_called()

    def test_existing_parent_frame_cannot_be_reused_as_lidar_child(self):
        self.enable_calibration((0, 0, 0), rpy_matrix(0, 0, 0))
        for frame in ('body', 'base_link', 'odom', 'camera_init', 'world', 'map', ''):
            with self.subTest(frame=frame):
                self.params['~lidar_frame'] = frame
                with self.assertRaises(ValueError):
                    self.bridge_class()
        self.static_tf.sendTransform.assert_not_called()


if __name__ == '__main__':
    unittest.main()
