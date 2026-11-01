"""Only perception routing is changed; simulation parameters are preserved."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from run_sim_crop_bypass import bypass_crop

HERE = Path(__file__).resolve().parent
BASELINE = HERE / 'results/execution_timing_diagnosis_20261028_01/online_probe'


class SimCropBypassTests(unittest.TestCase):
    def test_only_crop_node_and_height_output_change(self):
        with tempfile.TemporaryDirectory(prefix='simulation_crop_') as directory:
            out = Path(directory)
            launch_text = (BASELINE / 'experiment.launch').read_text()
            manifest = json.loads((BASELINE / 'manifest.json').read_text())
            (out / 'experiment.launch').write_text(launch_text)
            (out / 'manifest.json').write_text(json.dumps(manifest))
            expected = ET.fromstring(launch_text)
            crop = next(n for n in expected.findall('node') if n.get('name') == 'crop_box_filter')
            expected.remove(crop)
            height = next(n for n in expected.findall('node') if n.get('name') == 'passthrough_z')
            next(r for r in height.findall('remap') if r.get('from') == '~output').set('to', '/velodyne_points')
            bypass_crop(out)
            actual = ET.parse(out / 'experiment.launch').getroot()
            self.assertEqual(ET.tostring(actual), ET.tostring(expected))
            patched = json.loads((out / 'manifest.json').read_text())
            extra = patched.pop('simulation_perception_adjustment')
            original = dict(manifest)
            original['nodes'] = [n for n in original['nodes'] if n['name'] != 'crop_box_filter']
            self.assertEqual(patched, original)
            self.assertEqual(extra['launch_sha256'], hashlib.sha256((out / 'experiment.launch').read_bytes()).hexdigest())


if __name__ == '__main__':
    unittest.main()
