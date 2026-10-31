#!/usr/bin/env python3
"""Source-contract tests only; root owns compiled/numerical regressions."""
import unittest

from patch_mppi_terminal_phase import (
    BASE_SHA256, DEFAULT_SOURCE, PATCHES, SWITCH,
    project_off, sha256, transform_mppi,
)


def function_body(text, marker):
    """Small brace parser sufficient for these attested inline functions."""
    if text.count(marker) != 1:
        raise ValueError('Missing/ambiguous function marker: ' + marker)
    start = text.index(marker)
    opening = text.index('{', start)
    depth = 0
    for index in range(opening, len(text)):
        if text[index] == '{':
            depth += 1
        elif text[index] == '}':
            depth -= 1
            if not depth:
                return text[start:index+1]
    raise ValueError('Unclosed function')


class TerminalPhaseTransformTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.native = DEFAULT_SOURCE.read_text()
        cls.patched = transform_mppi(cls.native)

    def test_attested_parent(self):
        self.assertEqual(sha256(self.native), BASE_SHA256)

    def test_exact_inverse(self):
        self.assertEqual(project_off(self.patched), self.native)

    def test_double_patch_rejected(self):
        with self.assertRaises(ValueError):
            transform_mppi(self.patched)

    def test_unrelated_parent_drift_rejected(self):
        with self.assertRaises(ValueError):
            transform_mppi(self.native + '\n')

    def test_every_anchor_drift_rejected(self):
        for old, _ in PATCHES:
            with self.subTest(anchor=old):
                with self.assertRaises(ValueError):
                    transform_mppi(self.native.replace(old, old + ' ', 1))

    def test_patched_branch_drift_rejected(self):
        text = self.patched.replace('!terminal_phase_fix&&!side_pass',
                                    '!terminal_phase_fix||!side_pass', 1)
        with self.assertRaises(ValueError):
            project_off(text)

    def test_unrelated_patched_drift_rejected(self):
        with self.assertRaises(ValueError):
            project_off(self.patched.replace('return norm(', 'return .1+norm(', 1))

    def test_inverse_requires_patch(self):
        with self.assertRaises(ValueError):
            project_off(self.native)

    def test_constructor_only_switch_read(self):
        self.assertEqual(self.patched.count('std::getenv('), 1)
        helper = function_body(self.patched, '  static bool readTerminalPhaseFix()')
        self.assertIn('std::getenv("' + SWITCH + '")', helper)
        self.assertIn("value && value[0]=='1' && value[1]=='\\0'", helper)
        self.assertIn('terminal_phase_fix(readTerminalPhaseFix())', self.patched)
        for marker in ('  DynamicMerit dynamicScore(', '  V plan('):
            body = function_body(self.patched, marker)
            self.assertNotIn('getenv', body)
            self.assertNotIn('readTerminalPhaseFix()', body)

    def test_scoring_only_disables_one_yaw_branch(self):
        marker = '  DynamicMerit dynamicScore('
        before = function_body(self.native, marker)
        after = function_body(self.patched, marker)
        old, new = PATCHES[-1]
        self.assertEqual(before.replace(old, new, 1), after)
        self.assertIn('const bool endpoint=goalCapture(s,sc);', after)
        self.assertIn('else if(!side_pass&&(p.distance>.08||goal_distance<.25))', after)

    def test_proposals_lifecycle_and_thresholds_unchanged(self):
        for marker in ('  bool goalCapture(', '  double endpointApproachYaw(',
                       '  V plan(', '  void referenceChanged()', '  void reset()'):
            with self.subTest(function=marker):
                self.assertEqual(function_body(self.native, marker),
                                 function_body(self.patched, marker))

    def test_only_three_exact_edits(self):
        expected = self.native
        for old, new in PATCHES:
            expected = expected.replace(old, new, 1)
        self.assertEqual(expected, self.patched)
        self.assertEqual(len(PATCHES), 3)


if __name__ == '__main__':
    unittest.main()
