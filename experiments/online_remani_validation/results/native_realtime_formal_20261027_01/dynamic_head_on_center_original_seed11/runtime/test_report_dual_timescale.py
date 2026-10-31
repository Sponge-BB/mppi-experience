"""Accounting and reference-version matching contracts, no ROS."""
import unittest
import numpy as np

from report_dual_timescale import (union_duration, pair_events, publication_chain, integrate_diagnostics,
                                  terminal_settling_start,phase_stops)


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

    def test_republication_does_not_reuse_old_consume(self):
        publication=dict(transaction_fnv64='42',sim=1020,mono=100)
        observer=[dict(event='reference_observer_receipt',transaction_fnv64='42',sim=1020.01,mono=100.01,message_hashes=['abc'])]
        refs=[dict(t=10,message_hashes=['abc'],sha256='old',revision=3)]
        chain=publication_chain(publication,observer,refs,[],[])
        self.assertFalse(chain['consumption_matched'])

    def test_turning_not_both_zero(self):
        rows=[dict(t=0,reason='turn',applied_v=0,applied_w=.2),dict(t=1,reason='blocked',applied_v=0,applied_w=0),dict(t=2,reason='ok',applied_v=.1,applied_w=0)]
        out=integrate_diagnostics(rows,0,3)
        self.assertEqual(out['zero_command_duration_s'],{'blocked':1})
        self.assertEqual(out['low_linear_command_duration_s'],{'turn':1,'blocked':1})

    def terminal_fixture(self):
        state=np.array([(0,0,0,0,0,0),(1,2.99,0,0,0,0),(2,2.94,0,0,0,0),
                        (3,2.98,0,0,0,99),(4,2.99,0,0,0,99)],
                       dtype=[(k,float) for k in ('t','x','y','yaw','v','w')])
        return state,dict(goal_pos=.05,goal_yaw=.1,goal_speed=.05)

    def test_terminal_backtrack_not_fixed_one_second(self):
        state,limits=self.terminal_fixture()
        self.assertEqual(terminal_settling_start(state,limits,4),3)
        state['t'][-1]=3.6
        self.assertEqual(terminal_settling_start(state,limits,3.6),3)
        # Simulator uses raw yaw, not wrapped yaw; w is not a goal condition.
        state['yaw'][-1]=2*np.pi
        self.assertIsNone(terminal_settling_start(state,limits,3.6))

    def test_phase_clipping_and_missing_refresh(self):
        state,limits=self.terminal_fixture()
        diag=[dict(t=0,reason='ok',applied_v=.1,applied_w=0),dict(t=1,reason='blocked',applied_v=0,applied_w=0)]
        result=phase_stops(state,diag,limits,0,4,1,None)
        phases=result['phases']
        self.assertEqual(phases['obstacle_until_new_reference']['duration_sim_s'],2)
        self.assertEqual(phases['after_new_reference_before_goal']['duration_sim_s'],0)
        self.assertEqual(phases['terminal_settling']['duration_sim_s'],1)
        self.assertEqual(sum(p['duration_sim_s'] for p in phases.values()),4)
        self.assertEqual(sum(phases['terminal_settling']['zero_command_duration_s'].values()),1)
        clipped=phase_stops(state,diag,limits,0,4,1,9)['phases']
        self.assertEqual(clipped['after_new_reference_before_goal']['duration_sim_s'],0)


if __name__=='__main__':unittest.main()
