"""Pure protocol/runtime-copy/analytic truth tests; no ROS or robot commands."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np

import native_collision_runtime as runtime

HERE = Path(__file__).resolve().parent


def payload(full=True):
    header = np.zeros(32)
    header[[0, 1, 2, 3, 4]] = [3, 1, 10, 1001., 1]
    header[5:8] = [2, 2, 2]
    header[11:17] = [.1, 2, 0, 4, .35, 3.]
    header[17:25] = [999., 1001., 1, int(full), .05, 1, 1, 999.]
    body = np.array([[0, 0, .3, .3, .35, 50000, 0], [.4, 0, .9, .05, .1, 50000, 1]])
    return np.concatenate([header, body.ravel(), np.ones(8) if full else []])


class Protocol(unittest.TestCase):
    def test_binary_roundtrip_and_malformed_fail_closed(self):
        values = payload()
        np.testing.assert_array_equal(runtime.decode_snapshot(values.astype('<f8').tobytes()), values)
        for raw in (b'bad', values[:-1].astype('<f8').tobytes()):
            with self.assertRaises(ValueError):
                runtime.decode_snapshot(raw)
        values[12] = 2.5
        with self.assertRaises(ValueError):
            runtime.decode_snapshot(values.astype('<f8').tobytes())

    def test_dynamic_freshness_not_static_or_fixed_q_cache_age(self):
        values = payload()
        self.assertIsNone(runtime.freshness_reason(values, 1001.1))
        self.assertEqual(runtime.freshness_reason(None, 1001.), 'native3d_missing_snapshot')
        self.assertIn('stale', runtime.freshness_reason(values, 1001.4))
        values[3] = 1001.4
        self.assertEqual(runtime.freshness_reason(values, 1001.4), 'native3d_dynamic_stale')
        values[18] = 1001.4
        values[4] = 0
        self.assertEqual(runtime.freshness_reason(values, 1001.4), 'native3d_unhealthy')

    def test_first_field_survives_later_delta_before_first_bind(self):
        full, cached = runtime.attach_cached_field(payload(), None)
        delta = payload(False)
        delta[2] += 1
        merged, _ = runtime.attach_cached_field(delta, cached)
        self.assertEqual(merged[20], 1)
        np.testing.assert_array_equal(merged[-8:], full[-8:])
        delta[14] += 1
        merged, _ = runtime.attach_cached_field(delta, cached)
        self.assertEqual(merged[20], 0)

    def test_native_missing_or_rejected_never_binds_2d(self):
        lib = SimpleNamespace(online_set_collision_snapshot=Mock(return_value=0),
            online_set_native_collision_snapshot=Mock(return_value=3), online_set_native_collision_clock=Mock(return_value=0))
        node = SimpleNamespace(lib=lib, handle=1, native_collision_3d=True,
            native_collision_payload=None, native_collision_bindings={}, shared_collision=True,
            collision_payload=np.zeros(16))
        self.assertEqual(runtime.bind_collision(node, 1001., 'solve'), 'native3d_missing_snapshot')
        lib.online_set_collision_snapshot.assert_not_called()
        lib.online_set_native_collision_snapshot.assert_not_called()
        node.native_collision_payload = payload()
        self.assertEqual(runtime.bind_collision(node, 1001., 'validation'), 'native3d_snapshot_binding_failed')
        lib.online_set_native_collision_clock.assert_not_called()
        lib.online_set_collision_snapshot.assert_not_called()
        lib.online_set_native_collision_snapshot.return_value = 0
        self.assertIsNone(runtime.bind_collision(node, 1001., 'validation'))
        lib.online_set_native_collision_clock.assert_called_once_with(1, 1001.)


class TruthAndPatching(unittest.TestCase):
    def test_background_raw_snapshots_reconstruct_exactly_and_share_map(self):
        with tempfile.TemporaryDirectory() as temp:
            out = Path(temp)
            recorder = runtime.SnapshotRecorder(out)
            full, delta = payload(), payload(False)
            delta[2] += 1
            repeat = full.copy()
            repeat[2] += 2
            repeat[14] += 1  # new native sequence, identical raw field
            for values in (full, delta, repeat):
                recorder.submit(values.astype('<f8').tobytes(), values)
            recorder.close()
            records = [json.loads(line) for line in (out/'native3d_snapshots/index.jsonl').read_text().splitlines()]
            self.assertEqual(len(list((out/'native3d_snapshots/maps').glob('*.f64'))), 1)
            for record, values in zip(records, (full, delta, repeat)):
                self.assertEqual(runtime.load_recorded_snapshot(out/'native3d_snapshots', record), values.astype('<f8').tobytes())
            merged = runtime.load_recorded_snapshot(out/'native3d_snapshots', records[1], self_contained=True)
            self.assertEqual(merged[20], 1.)
            np.testing.assert_array_equal(merged[-8:], full[-8:])
            self.assertTrue((out/'native_collision_3d_body.json').exists())
            self.assertEqual(recorder.stats['errors'], [])

    def test_native_physical_radius_and_vertical_separation(self):
        body = np.array([[0., 0., .3, .3, .35, 1., 0.]])
        actor = [(1., 0., .2, 1., 0., 0.)]
        self.assertAlmostEqual(runtime.sampled_truth_clearance(np.zeros(3), body, .1, actor), .6)
        self.assertLess(runtime.sampled_truth_clearance(np.array([.7, 0., 0.]), body, .1, actor), 0)
        body[0, 2] = 2.
        self.assertGreater(runtime.sampled_truth_clearance(np.array([1., 0., 0.]), body, .1, actor), 0)

    def test_trial_patches_only_copies_both_safety_bindings_and_truth_stop(self):
        with tempfile.TemporaryDirectory() as temp:
            out = Path(temp)
            (out/'runtime').mkdir()
            originals = {}
            for name in ('mppi_node.py', 'local_execution.py', 'simulator.py'):
                originals[name] = (HERE/name).read_text()
                (out/'runtime'/name).write_text(originals[name])
            (out/'experiment.launch').write_text('<launch><node name="base_mppi"/><node name="remani_planner_node"/></launch>')
            (out/'manifest.json').write_text(json.dumps({'experiment_sources_sha256': {}}))
            runtime.patch_trial(out, True)
            for name in originals:
                self.assertEqual((HERE/name).read_text(), originals[name])
                compile((out/'runtime'/name).read_text(), name, 'exec')
            text = (out/'runtime/local_execution.py').read_text()
            self.assertEqual(text.count('bind_collision(node,'), 2)
            self.assertNotIn('online_set_collision_snapshot(', text)
            self.assertIn('publication_reason(node,final_time)', text)
            sim = (out/'runtime/simulator.py').read_text()
            self.assertIn("reason='native3d_sample_collision'", sim)
            self.assertIn('check_truth(self,self.t-dt+(kstep+1)*step)', sim)
            self.assertIn('if self.native_truth_body is None:u=np.zeros(2)', sim)
            self.assertNotIn("if self.scene in CASES and self.collided:reason='collision'", sim)
            manifest = json.loads((out/'manifest.json').read_text())
            self.assertFalse(manifest['native_collision_3d']['fallback_2d'])
            with self.assertRaises(ValueError):
                runtime.patch_trial(out, True)


if __name__ == '__main__':
    unittest.main()
