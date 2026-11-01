#!/usr/bin/env python3
from pathlib import Path
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

import run_candidate_audit as runner


class CandidateAuditRunnerTests(unittest.TestCase):
    def test_audit_enables_optimized_path_and_disables_old_replay(self):
        tree = ET.ElementTree(ET.fromstring('<launch><node name="remani_planner_node">'
            '<env name="DELIVERY_REPLAY_KIND" value="first_refresh"/>'
            '<env name="CANDIDATE_AUDIT_TARGET" value="wheel"/>'
            '</node><node name="base_mppi"/></launch>'))
        out = Path('/tmp/candidate-audit-test')
        runner.set_planner_environment(tree, out, 'collision')
        planner = tree.getroot().find("node[@name='remani_planner_node']")
        for name, expected in dict(DELIVERY_REPLAY_KIND='', DELIVERY_FAST_QUERY='1',
                DELIVERY_ACCEPT_CANDIDATE='1', CANDIDATE_AUDIT_TARGET='collision',
                CANDIDATE_AUDIT_DIR=str(out/'candidate_audit'),
                CANDIDATE_AUDIT_DONE_PATH=str(out/'diagnostic_complete.json')).items():
            self.assertEqual(len(planner.findall("env[@name='%s']" % name)), 1)
            self.assertEqual(planner.find("env[@name='%s']" % name).get('value'), expected)
        self.assertEqual(tree.getroot().find("node[@name='base_mppi']").findall('env'), [])

    def test_failure_restores_lifecycle(self):
        previous = runner.lifecycle.prepare_capture
        with mock.patch.object(runner.lifecycle, 'run_capture', side_effect=RuntimeError('test')):
            with self.assertRaises(RuntimeError):
                runner.run_capture(None, None, 'wheel')
        self.assertIs(runner.lifecycle.prepare_capture, previous)


if __name__ == '__main__':
    unittest.main()
