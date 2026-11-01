import unittest

from fast_refresh_runtime import AvoidanceEpisode, phase_state


class TestAvoidanceFeedback(unittest.TestCase):
    def test_no_repeated_episode_on_new_reference_or_invalid_output(self):
        state = AvoidanceEpisode()
        self.assertEqual(state.observe(1, True, False), (0, False))
        self.assertEqual(state.observe(1, True, True), (1, True))
        # A new reference is deliberately not an input to episode detection.
        self.assertEqual(state.observe(1, True, True), (1, True))
        self.assertEqual(state.observe(1, False, False), (1, False))
        self.assertEqual(state.observe(1, True, True), (1, True))
        for _ in range(4):
            self.assertEqual(state.observe(1, True, False), (1, False))
        self.assertEqual(state.observe(1, True, True), (2, True))
        self.assertEqual(state.observe(2, True, True), (1, True))

    def test_existing_dynamic_state_not_turn_or_ttc_threshold(self):
        self.assertEqual(phase_state(None), (None, 0.))
        self.assertEqual(phase_state(dict(decision=[0, -1, 3, .5],
                                         strategy=[1, 0, 0, 1])), (False, .5))
        self.assertEqual(phase_state(dict(decision=[2, -1, 3, .17],
                                         strategy=[1, 1, 1, 1])), (True, .17))
        # Crossing avoidance also counts; frontal lateral-needed is not required.
        self.assertEqual(phase_state(dict(decision=[2, -1, 3, .21],
                                         strategy=[1, 1, 0, 1])), (True, .21))
        self.assertEqual(phase_state(dict(decision=[3, -1, 3, .4],
                                         strategy=[1, 0, 0, 1])), (None, .4))
        self.assertEqual(phase_state(dict(decision=[0, -1, 3, .17],
                                         strategy=[1, 0, 0, 1])), (None, .17))

    def test_reference_reset_does_not_start_another_episode(self):
        state = AvoidanceEpisode()
        self.assertEqual(state.observe(1, True, True), (1, True))
        reset = phase_state(dict(decision=[3, -1, 3, .18], strategy=[1, 0, 0, 1]))[0]
        self.assertEqual(state.observe(1, True, reset), (1, False))
        self.assertEqual(state.observe(1, True, True), (1, True))
        # Even one safe normal cycle is not enough to rearm across handover.
        self.assertEqual(state.observe(1, True, False), (1, False))
        self.assertEqual(state.observe(1, True, True), (1, True))


if __name__ == '__main__':
    unittest.main()
