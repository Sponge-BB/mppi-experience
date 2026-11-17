"""No sockets, ROS, CAN operations or subprocesses are executed by these tests."""
import importlib.util
import json
from pathlib import Path
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('managed_session', ROOT/'bin/session.py')
SESSION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SESSION)


def state(flags):
    return json.dumps([dict(ifname='can1', flags=flags,
                            linkinfo=dict(info_kind='can'))]).encode()


class ReceiveOnlyGateTests(unittest.TestCase):
    def test_default_requires_listen_only(self):
        with patch.object(SESSION.subprocess, 'check_output', side_effect=[state(['UP']), b'can1: can state ERROR-ACTIVE']):
            with self.assertRaisesRegex(RuntimeError, 'LISTEN-ONLY'):
                SESSION.verify_receive_only_can('can1')

    def test_listen_only_records_no_application_transmit(self):
        with patch.object(SESSION.subprocess, 'check_output', side_effect=[state(['UP']), b'can1: <LISTEN-ONLY>']):
            result = SESSION.verify_receive_only_can('can1')
        self.assertTrue(result['listen_only'])
        self.assertFalse(result['hardware_ack_allowed'])
        self.assertFalse(result['application_can_tx_enabled'])
        self.assertEqual(result['controller_mode'], 'listen_only')

    def test_normal_ack_requires_explicit_flag_and_remains_application_receive_only(self):
        with patch.object(SESSION.subprocess, 'check_output', side_effect=[state(['UP']), b'can1: can state ERROR-ACTIVE']) as probe:
            result = SESSION.verify_receive_only_can('can1', allow_hardware_ack=True)
        self.assertTrue(result['hardware_ack_allowed'])
        self.assertFalse(result['listen_only'])
        self.assertFalse(result['application_can_tx_enabled'])
        self.assertEqual(result['controller_mode'], 'normal_ack_application_receive_only')
        for call in probe.call_args_list:
            command = call.args[0]
            self.assertIn('show', command)
            self.assertNotIn('set', command)

    def test_down_interface_is_rejected_even_with_ack_permission(self):
        with patch.object(SESSION.subprocess, 'check_output', return_value=state([])):
            with self.assertRaisesRegex(RuntimeError, 'UP CAN'):
                SESSION.verify_receive_only_can('can1', allow_hardware_ack=True)

    def test_unexpected_interface_cannot_invoke_ip(self):
        with patch.object(SESSION.subprocess, 'check_output') as probe:
            with self.assertRaisesRegex(RuntimeError, 'existing canN'):
                SESSION.verify_receive_only_can('can1;anything')
            probe.assert_not_called()


if __name__ == '__main__':
    unittest.main()
