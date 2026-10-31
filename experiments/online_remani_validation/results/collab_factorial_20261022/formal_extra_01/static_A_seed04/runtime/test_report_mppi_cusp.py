import unittest
from pathlib import Path
from report_mppi_cusp_adjustment import longest,audit_case,replay_equal

HERE=Path(__file__).resolve().parent
ROOT=HERE/'results/mppi_cusp_20261018'


class CuspReportTests(unittest.TestCase):
    def test_consecutive_stop(self):
        self.assertAlmostEqual(longest([False,True,True,False,True]),.1)
        self.assertEqual(longest([]),0.)

    def test_known_failure_stop_not_silently_dropped(self):
        path=ROOT/'focus_02/source_02/seed_01/baseline/dynamic_crossing'
        if not path.exists():self.skipTest('archived fixture unavailable')
        data=audit_case(path)
        self.assertEqual(data['source'],'source_02')
        self.assertAlmostEqual(data['longest_unfinished_active_zero_s'],34.9)
        self.assertFalse(data['passed'])

    def test_known_success_has_measured_phase_event(self):
        path=ROOT/'focus_02/source_02/seed_01/phase/dynamic_crossing'
        if not path.exists():self.skipTest('archived fixture unavailable')
        data=audit_case(path)
        self.assertTrue(data['passed'])
        self.assertEqual(len(data['phase_events']),1)

    def test_original_failure_replayed_exactly(self):
        now=ROOT/'focus_02/source_02/seed_01/baseline/dynamic_crossing'
        old=HERE/'results/dual_timescale_20261018/local_rejoin_01/source_02/dynamic_crossing'
        if not now.exists():self.skipTest('archived fixture unavailable')
        self.assertTrue(replay_equal(now,old))


if __name__=='__main__':unittest.main()
