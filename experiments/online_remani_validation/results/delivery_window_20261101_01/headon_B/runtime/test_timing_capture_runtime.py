import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from timing_capture_runtime import (BodyOnlySnapshotRecorder, LightCycleCapture,
                                    patch_sources, patch_trial)

HERE = Path(__file__).resolve().parent


class TimingCaptureTests(unittest.TestCase):
    def test_same_publisher_and_collision_receivers_remain(self):
        names = ('native_collision_runtime.py', 'shared_collision_node.py',
                 'search_recording.py', 'local_execution.py')
        sources = {name: (HERE / name).read_text() for name in names}
        changed = patch_sources(sources)
        self.assertIn('message.data=payload.tolist();self.pub.publish(message)',
                      changed['shared_collision_node.py'])
        self.assertNotIn('np.savez_compressed(target,payload=payload)',
                         changed['shared_collision_node.py'])
        self.assertIn('payload = decode_snapshot(raw)', changed['native_collision_runtime.py'])
        self.assertIn('sim.native_truth_body = body', changed['native_collision_runtime.py'])
        self.assertIn('sampled_truth_clearance(np.zeros(5)', changed['native_collision_runtime.py'])
        self.assertIn("'wall_ms':1000*", changed['local_execution.py'])
        self.assertIn('if computation_ms>50:', changed['local_execution.py'])
        self.assertIn('node.state_contract_cycles=LightCycleCapture(', changed['local_execution.py'])
        self.assertIn('if node.state_contract_cycles.enabled:rospy.on_shutdown(',
                      changed['local_execution.py'])

    def test_compact_cycle_keeps_report_fields_without_identity_or_live_io(self):
        from report_sim_crop_bypass import precontact_pairs
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'state_mppi_cycles.jsonl'
            capture = LightCycleCapture(path)
            original = dict(schema_version=1, event='mppi_cycle', cycle_id=7,
                solve_ros_time=1004., reason='ok', validation_code=0,
                online_step_invoked=True, online_step_succeeded=True,
                command_publish_after_monotonic=47., command_publish_after_ros_time=1004.1,
                cycle_start_monotonic=46.9, command_v=.1, command_w=0.,
                validation_attempts=[dict(candidate=0, code=0, wall_ms=1.2, thread_cpu_ms=1.1)],
                solve_state=np.zeros(5), solve_odom_identity=dict(large='omitted'))
            self.assertTrue(capture.enabled)
            self.assertTrue(capture.write(original))
            self.assertFalse(path.exists())
            capture.close()
            capture.close()
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(len(rows), 1)
            self.assertNotIn('solve_state', rows[0])
            self.assertNotIn('solve_odom_identity', rows[0])
            self.assertEqual(rows[0]['validation_attempts'][0]['wall_ms'], 1.2)
            pairs, alignment = precontact_pairs([
                dict(t=4., reason='ok', validation_code=0, kernel_ms=10.)], rows, 10.)
            self.assertEqual(len(pairs), 1)
            self.assertEqual(pairs[0][0]['kernel_ms'], 10.)
            self.assertEqual(alignment['missing_diagnostics'], 0)

    def test_body_export_without_large_maps(self):
        with tempfile.TemporaryDirectory() as temp:
            out = Path(temp)
            recorder = BodyOnlySnapshotRecorder(out)
            payload = np.zeros(32+7*67+100000)
            payload[12] = 67
            body = payload[32:32+7*67].reshape((-1, 7))
            body[:, 3:5] = .1
            for _ in range(10):
                recorder.submit(b'x', payload)
            recorder.close()
            export = json.loads((out / 'native_collision_3d_body.json').read_text())
            self.assertEqual(len(export['body']), 67)
            self.assertEqual(recorder.stats['frames'], 10)
            self.assertFalse((out / 'native3d_snapshots/maps').exists())
            self.assertFalse((out / 'native3d_snapshots/frames.bin').exists())
            self.assertLess(sum(p.stat().st_size for p in out.rglob('*') if p.is_file()), 20000)

    def test_per_node_overrides_keep_signed_planner_startup(self):
        with tempfile.TemporaryDirectory() as temp:
            out = Path(temp)
            runtime = out / 'runtime'
            runtime.mkdir()
            names = ('native_collision_runtime.py', 'shared_collision_node.py',
                     'search_recording.py', 'local_execution.py')
            for name in names:
                (runtime / name).write_text((HERE / name).read_text())
            (out / 'manifest.json').write_text(json.dumps(dict(experiment_sources_sha256={})))
            (out / 'experiment.launch').write_text('<launch>' + ''.join(
                '<node name="%s" />' % name for name in (
                    'remani_planner_node', 'base_mppi', 'dual_timescale_observer')) + '</launch>')
            patch_trial(out)
            tree = ET.parse(out / 'experiment.launch')
            envs = {n.get('name'): {e.get('name'): e.get('value') for e in n.findall('env')}
                    for n in tree.getroot().findall('node')}
            self.assertEqual(envs['remani_planner_node']['STATE_CONTRACT_CAPTURE'], '1')
            self.assertEqual(envs['remani_planner_node']['STATE_CONTRACT_SIGNED_LOW_SPEED'], '1')
            self.assertEqual(envs['remani_planner_node']['COUPLING_DIAGNOSTIC_DIR'], '')
            self.assertEqual(envs['base_mppi']['STATE_CONTRACT_CAPTURE'], '0')
            self.assertFalse(json.loads((out/'manifest.json').read_text())[
                'timing_only_capture']['complete_input_replay_available'])


if __name__ == '__main__':
    unittest.main()
