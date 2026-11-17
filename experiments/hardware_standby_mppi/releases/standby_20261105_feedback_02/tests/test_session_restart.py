"""Observer restart ownership tests; no ROS, signals or processes are used."""
import contextlib
import importlib.util
from pathlib import Path
import signal
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('restart_session', ROOT/'bin/session.py')
SESSION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SESSION)


class ShadowRestartTests(unittest.TestCase):
    def data(self):
        return dict(master_uri='http://127.0.0.1:11461', run_dir=str(ROOT/'logs/unexecuted_unit_test'),
                    processes=[dict(pid=10, component='master'), dict(pid=20, component='shadow'),
                               dict(pid=30, component='sensors'), dict(pid=40, component='feedback')])

    def test_unverified_shadow_cannot_be_signalled_or_replaced(self):
        data = self.data()
        with patch.object(SESSION, 'owned', side_effect=lambda entry: entry['pid'] != 20), \
                patch.object(SESSION, 'master_ok', return_value=True), \
                patch.object(SESSION.os, 'killpg') as kill, patch.object(SESSION, 'launch') as launch:
            with self.assertRaisesRegex(RuntimeError, 'owned shadow'):
                SESSION.restart_shadow(data, data['processes'])
            kill.assert_not_called(); launch.assert_not_called()

    def test_restart_signals_only_shadow_and_preserves_other_entries(self):
        data = self.data()
        others = [dict(entry) for entry in data['processes'] if entry['component'] != 'shadow']
        stopped = set()
        def kill(pid, sig):
            self.assertEqual((pid, sig), (20, signal.SIGINT)); stopped.add(pid)
        replacement = dict(pid=50, start_ticks='new', log='unused_test_log')
        with contextlib.ExitStack() as stack:
            stack.enter_context(patch.object(SESSION, 'owned', side_effect=lambda entry: entry['pid'] not in stopped))
            stack.enter_context(patch.object(SESSION, 'master_ok', return_value=True))
            stack.enter_context(patch.object(SESSION.os, 'killpg', side_effect=kill))
            stack.enter_context(patch.object(SESSION, 'launch', return_value=replacement))
            stack.enter_context(patch.object(SESSION, 'save'))
            stack.enter_context(patch.object(SESSION.time, 'sleep'))
            stack.enter_context(patch('builtins.print'))
            SESSION.restart_shadow(data, data['processes'])
        self.assertEqual(others, [entry for entry in data['processes'] if entry['component'] != 'shadow'])
        self.assertTrue(data['processes'][1]['replaced_by_restart'])
        self.assertEqual(data['processes'][-1]['replaces_pid'], 20)

    def test_stuck_shadow_prevents_duplicate_replacement(self):
        data = self.data()
        with patch.object(SESSION, 'owned', return_value=True), \
                patch.object(SESSION, 'master_ok', return_value=True), \
                patch.object(SESSION.time, 'monotonic', side_effect=[0., 9., 0., 3.]), \
                patch.object(SESSION.os, 'killpg') as kill, patch.object(SESSION, 'launch') as launch:
            with self.assertRaisesRegex(RuntimeError, 'duplicate observer'):
                SESSION.restart_shadow(data, data['processes'])
            self.assertEqual([call.args for call in kill.call_args_list], [(20, signal.SIGINT), (20, signal.SIGTERM)])
            launch.assert_not_called()


if __name__ == '__main__':
    unittest.main()
