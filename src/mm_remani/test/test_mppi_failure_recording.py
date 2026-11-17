"""Failure telemetry must retain original inputs without copying a map."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from whole_body_mppi import failure_recording_context


class FailureRecordingTests(unittest.TestCase):
    def test_original_state_version_and_candidate_reasons_are_retained(self):
        state = np.arange(17, dtype=float)
        header = np.arange(32, dtype=float)
        info = dict(feasible=0, structured_bounds_valid=[False, False],
                    structured_configuration_gaps=[.003, .003],
                    structured_external_gaps=[.43, .43], native_phase=3.5,
                    unrelated_large_object=object())
        result = failure_recording_context(state, SimpleNamespace(header=header), info, 100.)
        np.testing.assert_array_equal(result['measured_state'], state)
        np.testing.assert_array_equal(result['snapshot_header'], header)
        self.assertEqual(result['structured_bounds_valid'], [False, False])
        self.assertEqual(result['native_phase'], 3.5)
        self.assertNotIn('unrelated_large_object', result)
        self.assertNotIn('distance', result)
        np.testing.assert_array_equal(state, np.arange(17))

    def test_pre_capture_failure_does_not_invent_inputs(self):
        self.assertEqual(failure_recording_context(None, None, {}, 100.), {'plan_time': 100.})


if __name__ == '__main__':
    unittest.main()
