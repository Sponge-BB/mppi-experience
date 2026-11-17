#!/usr/bin/env python3
"""Run the candidate and existing filter tests without installing the candidate."""
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
import importlib.util
from pathlib import Path
import sys
import unittest

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
ROOT = next(p for p in HERE.parents if (p/'src/mm_remani/test/test_robot_self_filter.py').is_file())
spec = importlib.util.spec_from_file_location('velodyne_transformer', HERE/'source/velodyne_transformer.py')
module = importlib.util.module_from_spec(spec)
sys.modules['velodyne_transformer'] = module
spec.loader.exec_module(module)
sys.path[:0] = [str(HERE/'tests'), str(ROOT/'src/mm_remani/test')]
suite = unittest.defaultTestLoader.loadTestsFromNames([
    'test_self_filter_padding', 'test_robot_self_filter', 'test_mapping_cloud_sparse'])
with (HERE/'test_results.txt').open('w') as output:
    result = unittest.TextTestRunner(stream=output, verbosity=2).run(suite)
print('tests=%d failures=%d errors=%d' % (result.testsRun, len(result.failures), len(result.errors)))
raise SystemExit(not result.wasSuccessful())
