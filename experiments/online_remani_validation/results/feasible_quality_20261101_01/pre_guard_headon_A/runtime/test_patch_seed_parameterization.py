#!/usr/bin/env python3
import unittest
from patch_seed_parameterization import DEFAULT_SOURCE,FUNCTION_SHA256,REPLACEMENTS,_extract,project_off,transform_search
from verify_delivery_seed_geometry import extract_function

class SeedTransformTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.native=DEFAULT_SOURCE.read_text()
    def test_exact_off(self):self.assertEqual(project_off(transform_search(self.native)),self.native)
    def test_duplicate_rejected(self):
        with self.assertRaises(ValueError):transform_search(transform_search(self.native))
    def test_each_function_drift_rejected(self):
        for name in FUNCTION_SHA256:
            old=_extract(self.native,name)['text']
            with self.assertRaises(ValueError):transform_search(self.native.replace(old,old.replace('{','{ /* drift */',1),1))
    def test_off_drift_rejected(self):
        text=transform_search(self.native)
        with self.assertRaises(ValueError):project_off(text.replace('totalTrajTime=0.;','totalTrajTime=1.;',1))
    def test_other_functions_unchanged(self):
        text=transform_search(self.native)
        for name in ('getFlatState','getSampleTrajs','getKinoNode'):
            self.assertEqual(extract_function(text,name)['text'],extract_function(self.native,name)['text'])
    def test_scoped_changes(self):
        self.assertEqual(len(REPLACEMENTS),5)
        self.assertNotIn('has_path_',REPLACEMENTS['getTrajsWithTime'])
        self.assertNotIn('#define inf',transform_search(self.native)[:100])
    def test_runtime_switch_on_all_changed_functions(self):
        text=transform_search(self.native)
        for name in FUNCTION_SHA256:
            self.assertIn('if(seed_parameterization::enabled())',_extract(text,name)['text'])
    def test_true_endpoint_only_on(self):
        self.assertIn('seed_parameterization::enabled()?0.:1.0e-2',transform_search(self.native))

if __name__=='__main__':unittest.main()
