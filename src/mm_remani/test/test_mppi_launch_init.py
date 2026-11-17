"""Construct the real MPPI object from launch parameters without ROS/CAN I/O.

ROSLaunchConfig only parses XML. The local parameter mapping models namespace
deletion before resolved parameter upload; no parameter server is contacted.
"""
from contextlib import contextmanager, ExitStack
from copy import deepcopy
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

from roslaunch.config import ROSLaunchConfig
from roslaunch.xmlloader import XmlLoader

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))
from whole_body_mppi import WholeBodyNode


_MISSING = object()


class LocalParameters:
    def __init__(self, values):
        self.values = values

    def get_param(self, name, default=_MISSING):
        name = '/whole_body_mppi/'+name[1:] if name.startswith('~') else name
        if name in self.values:
            return deepcopy(self.values[name])
        prefix = name.rstrip('/')+'/'
        nested = {}
        for key, value in self.values.items():
            if not key.startswith(prefix):
                continue
            parts = key[len(prefix):].split('/'); current = nested
            for part in parts[:-1]:
                current = current.setdefault(part, {})
            current[parts[-1]] = deepcopy(value)
        if nested:
            return nested
        if default is _MISSING:
            raise KeyError(name)
        return default


class MPPIRealLaunchConstructionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = ROSLaunchConfig()
        # Parsing is read-only: no ROSLaunchParent, init_node, master or driver.
        XmlLoader().load(str(PACKAGE/'launch/exp0_real.launch'), cls.config,
                        argv=['hardware_enabled:=false'], verbose=False)
        cls.resolved = {key: deepcopy(value.value) for key, value in cls.config.params.items()}

    def parameters(self, residual, apply_clear):
        values = deepcopy(residual)
        if apply_clear:
            for namespace in self.config.clear_params:
                prefix = namespace.rstrip('/')+'/'
                values = {key: value for key, value in values.items()
                          if key != namespace.rstrip('/') and not key.startswith(prefix)}
        values.update(deepcopy(self.resolved))
        return LocalParameters(values)

    @contextmanager
    def blocked_ros(self, parameters):
        with ExitStack() as stack:
            stack.enter_context(patch('whole_body_mppi.rospy.get_param', side_effect=parameters.get_param))
            publishers = stack.enter_context(patch('whole_body_mppi.rospy.Publisher', side_effect=lambda *a, **k: Mock()))
            subscribers = stack.enter_context(patch('whole_body_mppi.rospy.Subscriber', side_effect=lambda *a, **k: Mock()))
            stack.enter_context(patch('whole_body_mppi.tf2_ros.Buffer', return_value=Mock()))
            stack.enter_context(patch('whole_body_mppi.tf2_ros.TransformListener', return_value=Mock()))
            stack.enter_context(patch('whole_body_mppi.rospy.on_shutdown'))
            stack.enter_context(patch('whole_body_mppi.rospy.init_node', side_effect=AssertionError('ROS init forbidden')))
            yield publishers, subscribers

    def test_clear_private_namespace_removes_stale_modes_and_real_constructor_succeeds(self):
        self.assertIn('/whole_body_mppi/', self.config.clear_params)
        residual = {
            '/whole_body_mppi/startup_to_standby': True,
            '/whole_body_mppi/simulation': True,
            '/whole_body_mppi/arm_position_servo': True,
            '/whole_body_mppi/obsolete_private_mode': True,
            '/unrelated_existing_node/retained': 123,
        }
        parameters = self.parameters(residual, apply_clear=True)
        for name in ('startup_to_standby', 'simulation', 'arm_position_servo', 'obsolete_private_mode'):
            self.assertIs(parameters.get_param('~'+name, False), False)
        self.assertEqual(parameters.get_param('/unrelated_existing_node/retained'), 123)
        with self.blocked_ros(parameters) as (publishers, subscribers):
            node = WholeBodyNode()
        self.assertFalse(node.execute)
        self.assertFalse(node.simulation)
        self.assertFalse(node.kernel.arm_position_servo)
        self.assertTrue(node.native_arm_startup)
        self.assertFalse(node.startup_complete)
        self.assertEqual(len(node.model.fixed_spheres), 47)
        self.assertEqual(node.kernel.postures.shape, (1, 6))
        self.assertIsNone(node.base_pub)
        self.assertIsNone(node.arm_pub)
        self.assertIsNone(node.enable_pub)
        self.assertGreater(publishers.call_count, 0)  # Status/visualization only, all mocked.
        self.assertGreater(subscribers.call_count, 0)

    def test_same_residual_without_namespace_clear_reproduces_observed_startup_conflict(self):
        parameters = self.parameters({'/whole_body_mppi/startup_to_standby': True}, apply_clear=False)
        with self.blocked_ros(parameters) as (publishers, subscribers):
            with self.assertRaisesRegex(ValueError, '^invalid_or_conflicting_arm_startup_modes$'):
                WholeBodyNode()
        # The failure occurs before constructing any command/status publisher
        # or subscription; merely rebuilding binaries cannot remove this flag.
        publishers.assert_not_called()
        subscribers.assert_not_called()


if __name__ == '__main__':
    unittest.main()
