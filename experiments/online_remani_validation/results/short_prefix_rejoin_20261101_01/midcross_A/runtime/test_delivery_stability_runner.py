import json
from pathlib import Path
import tempfile
import unittest

from run_delivery_stability import set_diagnostic_settings


class DiagnosticDeclaration(unittest.TestCase):
    def test_private_settings_are_not_navigation(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);(p/'manifest.json').write_text('{}')
            settings={'execution_weight':0.,'handover_weight':0.}
            set_diagnostic_settings(p,settings,2)
            m=json.loads((p/'manifest.json').read_text())
            self.assertFalse(m['navigation_metrics_valid'])
            self.assertFalse(m['coupling_experiment']['genuine_online_closed_loop'])
            self.assertTrue(m['coupling_experiment']['diagnostic_pause'])
            self.assertFalse(m['coupling_experiment']['boundary_admission'])
            self.assertEqual(m['coupling_experiment']['refresh_ordinal'],2)
            self.assertEqual(json.loads((p/'coupling_parameters.json').read_text()),settings)

    def test_no_relaxed_constraints_or_diagnostic_publication_path(self):
        source=Path(__file__).with_name('run_delivery_stability.py').read_text()
        self.assertNotIn('max_wheel_omega',source)
        self.assertIn("COUPLING_EXEC_WEIGHT='0'",source)
        self.assertIn("BOUNDARY_ADMISSION='0'",source)
        self.assertIn('audit_graph(out)',source)
        self.assertIn("if '__name:='+node[1:] in raw",source)


if __name__=='__main__':unittest.main()
