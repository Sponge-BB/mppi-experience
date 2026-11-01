"""Small synthetic records only; never a navigation/performance test."""
import gzip
import json
from pathlib import Path
import tempfile
import unittest

from PIL import Image

import report_native_dynamic_fast as report
from test_report_native_realtime import realtime_fixture


def fixture(path, enabled):
    realtime_fixture(path, True)
    manifest = json.loads((path/'manifest.json').read_text())
    manifest['native_dynamic_fast'] = dict(enabled=enabled)
    (path/'manifest.json').write_text(json.dumps(manifest))
    row = dict(t=1., reference_points=5, code=0, reason='deadline_miss',
        kernel_ms=70., callback_ms=80., authorized=False,
        applied_v=0., applied_w=0., nominal_v=.1, nominal_w=0.,
        validation_code=0, validation=[1., .2, .2, 301., 0., 1.],
        local_phase=dict(decision=[2., 0., 2., .1, 0., 0., 1.]), observed_obstacles=[])
    with gzip.open(path/'mppi_diagnostics.json.gz', 'wt') as stream:
        json.dump([row], stream)
    (path/'state_mppi_cycles.jsonl').write_text('\n'.join(json.dumps(r) for r in [
        dict(command_publish_after_ros_time=1001.1, reason='deadline_miss', command_v=0., command_w=0.),
        dict(command_publish_after_ros_time=1001.4, reason='ok', command_v=.1, command_w=0.)])+'\n')


class ReportTests(unittest.TestCase):
    def test_phase_and_timing_do_not_treat_missing_as_zero(self):
        rows = [dict(reference_points=10, code=0, kernel_ms=5., callback_ms=8.,
                     local_phase={'decision': [0, 0, 0, 0, 0, 0, 0]}, observed_obstacles=[]),
                dict(reference_points=10, code=0, kernel_ms=40., callback_ms=48.,
                     local_phase={'decision': [0, 0, 0, 0, 0, 0, 1]}, observed_obstacles=[]),
                dict(reference_points=0, code=0, kernel_ms=999., callback_ms=999.)]
        value = report.timing_by_phase(rows)
        self.assertEqual(value['overall']['cycles'], 2)
        self.assertEqual(value['intervention']['callback_ms']['p50'], 48.)
        self.assertIsNone(value['monitoring']['callback_ms'])
        self.assertEqual(report.phase_of({}), 'unknown')

    def test_new_switch_validated_and_publication_zero_interval(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'trial'
            fixture(path, True)
            value = report.summarize_trial(path, 'fast')
            self.assertEqual(value['group'], 'fast')
            self.assertEqual(value['deadline_zero']['cycles'], 1)
            self.assertAlmostEqual(value['deadline_zero']['issued_zero_duration_s'], .3)
            self.assertTrue(value['deadline_zero']['publication_timestamp_based'])
            self.assertFalse(value['remani_delivery']['available'])
            self.assertIsNone(value['remani_delivery']['accepted_refresh_requests'])
            with self.assertRaisesRegex(ValueError, 'enabled'):
                report.Trial(path, 'baseline')
            with self.assertRaises(ValueError):
                report.Trial(path, 'original')

    def test_render_true_recorded_geometry_and_four_frames(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            fixture(root/'baseline', False)
            fixture(root/'fast', True)
            before = (root/'baseline/manifest.json').read_bytes()
            value = report.render_pair(root/'baseline', root/'fast', root/'viz', max_frames=4)
            self.assertTrue(value['actual_robot_projection'])
            self.assertTrue(value['actual_remani_references'])
            self.assertTrue(value['actual_recorded_search_subset_only'])
            self.assertEqual(before, (root/'baseline/manifest.json').read_bytes())
            with Image.open(root/'viz/comparison.gif') as image:
                self.assertEqual(image.n_frames, 4)
                for i in range(image.n_frames):
                    image.seek(i)
                    image.load()

    def test_complete_gate_report_and_readable_summary(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            batch = root/'batch'
            batch.mkdir()
            replay = root/'replay.json'
            replay.write_text(json.dumps(dict(passed=True, scope='synthetic test only')))
            plan = dict(groups=['baseline', 'fast'], scenes=['static'], seed=11,
                        replay_gate_sha256=report.sha(replay))
            (batch/'plan.json').write_text(json.dumps(plan))
            fixture(batch/'static_baseline_seed11', False)
            fixture(batch/'static_fast_seed11', True)
            with self.assertRaisesRegex(ValueError, 'finish'):
                report.report_batch(batch, replay, root/'blocked', render=False)
            (batch/'complete.json').write_text('{}')
            result = report.report_batch(batch, replay, root/'out', render=False)
            self.assertEqual(len(result['trials']), 2)
            self.assertFalse(result['statistical_success_rate_claim'])
            self.assertTrue((root/'out/online_comparison.png').is_file())
            text = (root/'out/实验总结.txt').read_text()
            self.assertIn('不是只播放旧运动轨迹', text)
            self.assertIn('不是Gazebo', text)
            self.assertIn('修改前', text)
            self.assertIn('省重复计算后', text)


if __name__ == '__main__':
    unittest.main()
