"""Synthetic RECORD fixtures check rendering, not navigation performance."""
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from PIL import Image

import render_native_collision_3d as renderer


def fixture(path, enabled):
    path.mkdir()
    def write(name, value):
        (path/name).write_text(json.dumps(value)+'\n')
    write('manifest.json', {'scene': 'static', 'scenario_start': 'motion', 'native_collision_3d': {'enabled': enabled}})
    write('coupling_parameters.json', {'seed': 11})
    write('result.json', {'reason': 'timeout', 'motion_anchor': 1., 'goal_sent': .5, 'elapsed': 2.,
        'min_clearance': -.01, 'collision_conservative_projection': True,
        'native3d_truth_monitor': {'geometry_available': True, 'checks': 100, 'error': None, 'sample_collision': False}})
    write('run_status.json', {'result_written': True})
    write('trial_audit.json', {'valid_experiment': True})
    header = np.zeros(32)
    header[11] = .1
    write('native_collision_3d_body.json', {'header': header.tolist(), 'body': [[0, 0, .3, .3, .35, 1, 0], [.2, 0, .8, .05, .1, 1, 1]]})
    write('geometry.json', {'base_length': .5, 'base_width': .4})
    (path/'state.csv').write_text('t,x,y,yaw,v\n0,0,0,0,0\n1,0.1,0,0,.1\n2,.2,0,0,.1\n')
    (path/'consumed_references.jsonl').write_text(json.dumps({'t': .8, 'revision': 1, 'reference': [[0, 0, 0, .1], [3, 0, 0, 0]]})+'\n')
    (path/'reference_events.jsonl').write_text(json.dumps({'t': .8, 'event': 'replace', 'revision': 1})+'\n')
    (path/'search').mkdir()
    np.savez(path/'search/000000.npz', published=1., rows=np.array([[0, 0, .1, 0, 1, .1, 1, 1], [0, .1, .11, 0, 1, .1, 1, 1]]))


class Rendering(unittest.TestCase):
    def test_real_search_and_separate_collision_metrics(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            fixture(root/'off', False)
            fixture(root/'on', True)
            value = renderer.render_pair(root/'off', root/'on', root/'viz', max_frames=4)
            self.assertEqual(value['frames'], 4)
            self.assertTrue(value['real_search_subset_only'])
            report = value['trials'][0]
            self.assertTrue(report['projection_2d_overlap'])
            self.assertFalse(report['native3d_independent_sample_collision'])
            self.assertTrue(report['native3d_monitor_ready'])
            with Image.open(root/'viz/comparison.gif') as image:
                for i in range(image.n_frames):
                    image.seek(i)
                    image.load()
                self.assertEqual(image.n_frames, 4)

    def test_query_truth_vertical_geometry_is_not_2d_overlap(self):
        body = np.array([[1., 0., 2., .05, .1, 1., 1.]])
        gap = renderer.sampled_truth_clearance(np.zeros(3), body, .1, [(1., 0., .2, 1, 0, 0)])
        self.assertGreater(gap, 0)


if __name__ == '__main__':
    unittest.main()
