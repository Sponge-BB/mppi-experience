"""Minimal checks for missing diagnostics and rejected-vs-late commands."""
import unittest

from report_sim_crop_bypass import precontact_pairs, window_metrics


def diagnostic(t, reason, code, proposal=None):
    return dict(t=t, reason=reason, validation_code=code, reference_points=3,
                validated_proposal=proposal, authorized=reason == 'ok',
                applied_v=.1 if reason == 'ok' else 0., applied_w=0.,
                code=0, callback_ms=1., kernel_ms=.5)


def cycle(ident, t, published, reason='ok', code=0):
    return dict(cycle_id=ident, solve_ros_time=1000.+t,
                command_publish_after_ros_time=1000.+published,
                reason=reason, validation_code=code,
                validation_attempts=[dict(candidate=0, code=code)])


class CropReportTests(unittest.TestCase):
    def test_missing_initial_diagnostic_does_not_shift_alignment(self):
        first = diagnostic(1., 'ok', 0, [.1, 0.])
        second = diagnostic(1., 'ok', 0, [.2, 0.])
        late = diagnostic(2., 'ok', 0, [.3, 0.])
        cycles = [cycle(1, 0., .1, 'no_active_task', -1),
                  cycle(2, 1., 1.1), cycle(3, 1., 1.2), cycle(4, 2., 3.)]
        pairs, stats = precontact_pairs([first, second, late], cycles, end=3.)
        self.assertIsNone(pairs[0][0])
        self.assertIs(pairs[1][0], first)
        self.assertIs(pairs[2][0], second)
        self.assertEqual(stats['missing_diagnostics'], 1)
        self.assertEqual(stats['excluded_contact_or_later_rows'], 1)
        self.assertEqual(stats['unmatched_diagnostics'], 0)

    def test_failed_braking_is_not_a_safe_action_lost_only_to_deadline(self):
        failed = diagnostic(1., 'deadline_miss', 5)
        late_safe = diagnostic(2., 'deadline_miss', 0, [.1, 0.])
        pairs = [(failed, cycle(1, 1., 1.1, 'deadline_miss', 5), 1.1),
                 (late_safe, cycle(2, 2., 2.1, 'deadline_miss', 0), 2.1)]
        metrics = window_metrics(pairs)
        self.assertEqual(metrics['deadline']['label_count'], 2)
        self.assertEqual(metrics['deadline']['already_failed_validation'], 1)
        self.assertEqual(metrics['deadline']['validated_nonzero_proposals_cleared'], 1)
        self.assertEqual(metrics['cycles_with_braking_rejection_code5'], 1)


if __name__ == '__main__':
    unittest.main()
