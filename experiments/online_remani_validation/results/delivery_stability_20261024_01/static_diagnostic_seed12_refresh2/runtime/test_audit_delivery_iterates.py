#!/usr/bin/env python3
import unittest
import numpy as np

import audit_delivery_iterates as a
from recheck_delivery_diag import snapshot_arrays
from test_recheck_delivery_diag import frame


def boundary(x=.8,v=.1):
    values=np.zeros((8,4));values[0,0]=x;values[0,1]=v
    return dict(rows=8,cols=4,data_column_major=values.flatten(order='F').tolist())


def record(stage='initial',iteration=0):
    b=np.zeros((8,8));b[0,0]=.8;b[1,0]=.1
    return dict(kind='canonical_candidate',phase=a.PHASES[0],request_id=2,reconstructed=True,
        stage=stage,iteration=iteration,solver_elapsed_s=.1,canonical_parts=[dict(gear=1,times=[1.],b_rows_ascending_power=b.tolist())],
        x=[1.,2.],g=[3.,4.],cost=10.,native_IsTrajSafe_pass=True,native_reason='ok',native_audit_wall_s=.01,
        native_gate_skips_first_0_05_s=True,future_FSM_live_acceptance_certified=False)


class IterateAuditTests(unittest.TestCase):
    def test_coefficient_conversion_and_derivatives(self):
        r=record();r['canonical_parts'][0]['b_rows_ascending_power'][2][0]=.5
        c=a.canonical_candidate(r,1000.);sample=list(a.candidate_samples(c,'dense',.5))[-1]
        self.assertAlmostEqual(sample['x'],1.4)
        self.assertAlmostEqual(sample['vx'],1.1)
        self.assertEqual(c['parts'][0]['gear'],1)
        self.assertEqual(sample['query_time'],1001.)

    def test_invalid_coefficients_and_unknown(self):
        r=record();r['reconstructed']=False
        self.assertIsNone(a.canonical_candidate(r,1000.))
        r=record();r['canonical_parts'][0]['times']=[-1.]
        with self.assertRaises(ValueError):a.canonical_candidate(r,1000.)
        self.assertIsNone(a.conjunction([True,None]))
        self.assertFalse(a.conjunction([False,None]))

    def test_effective_head_not_raw_capture(self):
        c=a.canonical_candidate(record(),1000.)
        s=dict(effective_initial_states=[boundary()])
        self.assertTrue(a.head_evidence(c,s)['matches'])
        s['effective_initial_states']=[boundary(v=-.1)]
        self.assertFalse(a.head_evidence(c,s)['matches'])
        self.assertIsNone(a.head_evidence(c,{})['matches'])

    def test_signature_detects_substitution(self):
        r=record();s=dict(first_seen=True,first_cost=10.,first_x_hash=a.vector_hash(r['x']),first_gradient_hash=a.vector_hash(r['g']))
        self.assertTrue(a.first_signature(s,r)['verified'])
        r['g'][0]+=1.
        self.assertFalse(a.first_signature(s,r)['verified'])

    def test_phase_counts_only_actual_booleans_and_full_identity(self):
        r=record();phase=a.PHASES[0]
        s=dict(kind='solve_summary',phase=phase,request_id=2,first_seen=True,first_cost=10.,
            first_x_hash=a.vector_hash(r['x']),first_gradient_hash=a.vector_hash(r['g']),
            effective_initial_states=[boundary()],accepted_total=0,accepted_truncated=False,
            input_hash='5',raw_seq=3,esdf_seq=3,sim=1000.,sim_end=1000.,frame_now=1000.,frame_sequence=4,native_return_success=False)
        end=dict(kind='post_solver_audit_end',phase=phase,request_id=2,audited_candidates=1,not_part_of_lbfgs_budget=True,audit_wall_s=.02)
        ret=dict(kind='recursive_return',phase=phase,request_id=2,success=False)
        capture=dict(request_id=2,input_hash='5',raw_seq=3,esdf_seq=3,sim=1000.,frame=dict(now=1000.))
        result=a.analyze_phase(type('Trial',(),dict(name='synthetic'))(),phase,[s,r,end,ret],capture,snapshot_arrays(frame()),dict(sequence=4))
        self.assertTrue(result['phase_evidence_valid'])
        self.assertEqual(result['counts']['all']['both_pass'],1)
        self.assertEqual(result['first_both_pass']['iteration'],0)
        r['native_IsTrajSafe_pass']=None
        result=a.analyze_phase(type('Trial',(),dict(name='synthetic'))(),phase,[s,r,end,ret],capture,snapshot_arrays(frame()),dict(sequence=4))
        self.assertIsNone(result['candidates'][0]['both_pass'])
        self.assertEqual(result['counts']['all']['both_pass'],0)
        self.assertFalse(result['phase_evidence_valid'])


if __name__=='__main__':unittest.main()
