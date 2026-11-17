"""Parse the real launch only; never start ROS nodes, SDK, CAN or motors."""
from pathlib import Path
import unittest

from roslaunch.config import ROSLaunchConfig
from roslaunch.xmlloader import XmlLoader


LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'exp0_real.launch'


class OriginalArmStartupLaunchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.configurations = {}
        for use_mppi in (False, True):
            for hardware in (False, True):
                config = ROSLaunchConfig()
                # XmlLoader resolves XML and parameters. It does not launch
                # processes or connect to a master, even with hardware=true.
                XmlLoader().load(str(LAUNCH), config, argv=[
                    'use_mppi:=' + str(use_mppi).lower(),
                    'hardware_enabled:=' + str(hardware).lower(),
                ], verbose=False)
                cls.configurations[use_mppi, hardware] = config

    def test_original_bridge_and_single_driver_are_present_in_each_mode(self):
        for (use_mppi, hardware), config in self.configurations.items():
            with self.subTest(use_mppi=use_mppi, hardware=hardware):
                bridges = [n for n in config.nodes
                           if n.package == 'mm_controller' and n.type == 'joint_cmd_bridge.py']
                drivers = [n for n in config.nodes
                           if n.package == 'piper' and n.type == 'piper_ctrl_single_node.py']
                self.assertEqual(len(bridges), 1)
                self.assertEqual(len(drivers), int(hardware))
                self.assertIs(config.params['/joint_cmd_bridge/execute'].value, hardware)
                remaps = dict(bridges[0].remap_args)
                self.assertEqual(remaps.get('/joint_ctrl_single', '/joint_ctrl_single'),
                                 '/piper_startup_joint_ctrl' if use_mppi else '/joint_ctrl_single')

    def test_original_enable_and_standby_with_separate_mppi_execution_limits(self):
        for use_mppi in (False, True):
            with self.subTest(use_mppi=use_mppi):
                config = self.configurations[use_mppi, True]
                prefix = '/piper_ctrl_single_node/'
                expected = {
                    'can_port': 'can1',
                    'auto_enable': True,
                    'auto_reenable_on_drop': True,
                    'gripper_exist': True,
                    'native_startup_to_standby': use_mppi,
                    'fixed_vel': 1 if use_mppi else 15,
                    'command_timeout': .30 if use_mppi else 0.,
                }
                for name, value in expected.items():
                    self.assertEqual(config.params[prefix + name].value, value, name)

    def test_mppi_is_the_only_post_startup_command_path(self):
        for (use_mppi, hardware), config in self.configurations.items():
            with self.subTest(use_mppi=use_mppi, hardware=hardware):
                nodes = [n for n in config.nodes
                         if n.package == 'mm_remani' and n.type == 'whole_body_mppi.py']
                self.assertEqual(len(nodes), int(use_mppi))
                if use_mppi:
                    self.assertIs(config.params['/whole_body_mppi/execute'].value, hardware)
                    self.assertIs(config.params['/whole_body_mppi/native_arm_startup'].value, True)
                    remaps = dict(nodes[0].remap_args)
                    self.assertEqual(remaps.get('/joint_ctrl_single', '/joint_ctrl_single'),
                                     '/joint_ctrl_single')
                self.assertIs(config.params['/mm_controller_node/fsm/capture_startup_standby'].value,
                              False)


if __name__ == '__main__':
    unittest.main()
