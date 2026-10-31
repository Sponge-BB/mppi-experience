"""Recorded fixture tests only; no navigation performance claims."""
import gzip
import json
from pathlib import Path
import tempfile
import unittest

from PIL import Image
import numpy as np

from test_render_native_collision_3d import fixture
from test_native_realtime import diagnostic
import render_native_realtime as render
import report_native_realtime as report
import run_native_realtime as runner
from native_collision_runtime import SnapshotRecorder
from test_native_collision_runtime import payload


def realtime_fixture(path, demand, stationary=False):
    fixture(path, True)
    manifest = json.loads((path/'manifest.json').read_text())
    manifest['native_realtime'] = {'demand': demand}
    (path/'manifest.json').write_text(json.dumps(manifest))
    result = json.loads((path/'result.json').read_text())
    result['native_realtime_evidence'] = {'first_truth_actor_t': None, 'first_observed_moving_t': None}
    if stationary:
        result['motion_anchor'] = None
        result['reason'] = 'deadline_zero_deadlock'
        (path/'state.csv').write_text('t,x,y,yaw,v\n0,0,0,0,0\n1,0,0,0,0\n2,0,0,0,0\n')
    (path/'result.json').write_text(json.dumps(result))
    row = diagnostic(1.)
    row.update(kernel_ms=70., callback_ms=75., authorized=False)
    with gzip.open(path/'mppi_diagnostics.json.gz', 'wt') as stream:
        json.dump([row], stream)


class Reports(unittest.TestCase):
    def test_point_presence_is_not_moving_prediction_or_static_dynamic_exposure(self):
        for scene, speed, expected in (('static', .2, False), ('crossing', 0., False), ('crossing', .2, True)):
            with self.subTest(scene=scene, speed=speed), tempfile.TemporaryDirectory() as temp:
                path = Path(temp)/'trial'
                realtime_fixture(path, True)
                manifest = json.loads((path/'manifest.json').read_text())
                manifest['scene'] = scene
                (path/'manifest.json').write_text(json.dumps(manifest))
                result = json.loads((path/'result.json').read_text())
                result['native_realtime_evidence']['first_truth_actor_t'] = 2.
                (path/'result.json').write_text(json.dumps(result))
                values = payload()
                values[13] = 1
                values[3] = values[18] = 1002.
                offset = 32+7*int(values[12])
                values = np.concatenate((values[:offset], [1., 0., .5, speed, 0., 0., 1.], values[offset:]))
                recorder = SnapshotRecorder(path)
                recorder.submit(values.astype('<f8').tobytes(), values)
                recorder.close()
                value = report.summarize_trial(path, 'demand')
                self.assertTrue(value['native_prediction_points_received'])
                self.assertEqual(value['native_nonzero_velocity_snapshot_count'], int(speed > 0))
                self.assertEqual(value['dynamic_exposure_valid'], expected)

    def test_untriggered_scene_never_counts_dynamic_exposure(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'trial'
            realtime_fixture(path, False, stationary=True)
            value = report.summarize_trial(path, 'original')
            self.assertFalse(value['scene_entered'])
            self.assertFalse(value['dynamic_exposure_valid'])
            self.assertTrue(value['early_deadline_zero_deadlock'])
            self.assertEqual(value['deadline_miss_cycles'], 1)
            self.assertEqual(value['actual_nonzero_command_cycles'], 0)
            self.assertFalse(runner.startup_check(path)['passed'])

    def test_both_native3d_renderer_labels_and_goal_alignment(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            realtime_fixture(root/'old', False, stationary=True)
            realtime_fixture(root/'new', True)
            value = render.render_pair(root/'old', root/'new', root/'viz', max_frames=4)
            self.assertTrue(value['both_native_collision_3d'])
            self.assertIn('both goal_sent', value['alignment'])
            self.assertTrue(value['no_future_actor_fabrication'])
            with Image.open(root/'viz/comparison.gif') as image:
                for index in range(image.n_frames):
                    image.seek(index)
                    image.load()
                self.assertEqual(image.n_frames, 4)


if __name__ == '__main__':
    unittest.main()
