"""Accounting and reference-version matching contracts, no ROS."""
import unittest

from report_dual_timescale import union_duration, pair_events, publication_chain, integrate_diagnostics


class ReportContracts(unittest.TestCase):
    def test_nested_intervals_not_double_counted(self):
        self.assertEqual(union_duration([(0,4),(1,3),(3,5)]),5)
        self.assertEqual(union_duration([(0,4),(1,3),(3,5)],(2,4.5)),2.5)
        self.assertEqual(union_duration([(5,6)],(0,4)),0)

    def test_unclosed_wait_not_extended(self):
        rows=[dict(event='begin',mono=1,key='a'),dict(event='begin',mono=2,key='b'),dict(event='end',mono=4,key='a')]
        closed,unmatched=pair_events(rows,'begin','end',lambda r:r['key'])
        self.assertEqual(len(closed),1);self.assertEqual(unmatched[0]['event']['key'],'b')
        self.assertEqual(union_duration([(a['mono'],b['mono']) for a,b in closed]),3)

    def test_source_consumption_not_tail_receipt_time(self):
        publication=dict(transaction_fnv64='42',sim=1010,mono=90,request_id=2)
        observer=[dict(event='reference_observer_receipt',transaction_fnv64='42',sim=1010.01,mono=90.01,message_hashes=['abc']),
                  dict(event='mppi_reference_consumption_observed',source_ros=1010.03,sim=1018,mono=98)]
        refs=[dict(t=10.03,message_hashes=['abc'],sha256='pathhash',revision=3)]
        events=[dict(t=10.02,message_hashes=['abc'],event='replace',revision=3)]
        diag=[dict(t=10.05,reference_sha256='pathhash',reason='ok',applied_v=.1,applied_w=0)]
        chain=publication_chain(publication,observer,refs,events,diag)
        self.assertAlmostEqual(chain['publish_to_consume_sim_s'],.03)
        self.assertAlmostEqual(chain['publish_to_first_control_sim_s'],.05)
        self.assertTrue(chain['complete_exact_chain'])

    def test_time_proximity_does_not_replace_hash_match(self):
        publication=dict(transaction_fnv64='42',sim=1010,mono=90)
        observer=[dict(event='reference_observer_receipt',transaction_fnv64='43',sim=1010.001,mono=90.001,message_hashes=['abc'])]
        chain=publication_chain(publication,observer,[],[],[])
        self.assertFalse(chain['observer_receipt_matched'])
        self.assertNotIn('consume_sim_s',chain)

    def test_turning_not_both_zero(self):
        rows=[dict(t=0,reason='turn',applied_v=0,applied_w=.2),dict(t=1,reason='blocked',applied_v=0,applied_w=0),dict(t=2,reason='ok',applied_v=.1,applied_w=0)]
        out=integrate_diagnostics(rows,0,3)
        self.assertEqual(out['zero_command_duration_s'],{'blocked':1})
        self.assertEqual(out['low_linear_command_duration_s'],{'turn':1,'blocked':1})


if __name__=='__main__':unittest.main()
