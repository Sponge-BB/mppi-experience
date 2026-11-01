import unittest

from report_watchdog_off import cycle_spans


def cycle(ident, published, started, mono_published):
    return dict(cycle_id=ident, command_publish_after_ros_time=1000.+published,
        cycle_start_monotonic=started, command_publish_after_monotonic=mono_published,
        reason='ok')


class TimingWindowTests(unittest.TestCase):
    def test_separates_compute_from_between_cycle_wait(self):
        rows = [cycle(1, 10., 100., 100.1), cycle(2, 13.5, 103.4, 103.5)]
        value = cycle_spans(rows, 9., 14.)
        self.assertAlmostEqual(value['start_to_publish']['maximum_s'], .1)
        self.assertAlmostEqual(value['previous_publish_to_next_start']['maximum_s'], 3.3)
        self.assertEqual(value['previous_publish_to_next_start']['over_250ms_count'], 1)

    def test_publication_end_excluded_and_no_tail_invented(self):
        rows = [cycle(1, 9., 99., 99.1), cycle(2, 10., 100., 100.1),
                cycle(3, 11., 101., 101.1)]
        value = cycle_spans(rows, 10., 11.)
        self.assertEqual(value['start_to_publish']['count'], 1)
        self.assertEqual(value['previous_publish_to_next_start']['count'], 0)
        self.assertIsNone(value['previous_publish_to_next_start']['maximum_s'])


if __name__ == '__main__':
    unittest.main()
