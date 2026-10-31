"""Synthetic contracts; no actual archive analysis, ROS, compilation or plot."""
import copy
import unittest
from report_delivery_stability import summarize_search,summarize_paused,LABELS


class SearchTests(unittest.TestCase):
    def test_native_failure_can_be_rescued_and_final_failure_can_follow_success(self):
        rows=[]
        for i,(native,final,route) in enumerate(((3,2,'rrt_fallback'),(2,3,'astar_then_arm_sampling')),1):
            base=dict(event='delivery_search_exit',pid=42,request_id=i)
            rows+=[dict(base,solver='kino_astar',probe_call_id=100+i,probe_parent_call_id=i,
                        return_code=native,exit_reason='time_budget_exhausted' if native==3 else 'oneshot_reach_end'),
                   dict(base,solver='frontend_wrapper',probe_call_id=i,return_code=final,
                        exit_reason='sampling_success_end' if final==2 else 'arm_sampling_or_rrt_failed',sampling_route=route)]
        answer=summarize_search(rows)
        self.assertEqual(answer['wrapper_count'],2);self.assertEqual(answer['wrapper_successes'],1)
        self.assertEqual(answer['fallback_rescues'],1);self.assertEqual(answer['astar_success_then_sampling_failures'],1)

    def test_missing_log_is_not_zero_failure(self):
        self.assertIsNone(summarize_search(None)['wrapper_count'])

    def test_same_call_id_other_pid_not_attached(self):
        rows=[dict(event='delivery_search_exit',solver='frontend_wrapper',pid=1,request_id=3,probe_call_id=4,return_code=3),
              dict(event='delivery_search_exit',solver='kino_astar',pid=2,request_id=3,probe_parent_call_id=4,return_code=2)]
        self.assertFalse(summarize_search(rows)['records'][0]['astar_success_then_sampling_failed'])


class PausedTests(unittest.TestCase):
    def fixture(self):
        rows=[dict(kind='capture',input_hash='123',request_id=2)];failures=[]
        for label,budget in zip(LABELS,(.6,.6,1.5)):
            rows+=[dict(kind='solve_summary',phase=label,input_hash='123',budget_s=budget,first_seen=True,first_cost=2.,
                first_x_hash='55',first_gradient_hash='99',effective_initial_states=[1],effective_final_states=[2],
                raw_seq=1,esdf_seq=1,frame_now=1001.,frame_sequence=8,prediction_hash='a',sim=1001.,sim_end=1001.,
                wheel_limit=3.2,native_return_success=False,reason='budget_timeout'),
                dict(kind='recursive_return',phase=label,success=False)]
            failures.append(dict(kind='solve',label=label,input_hash='123',budget_s=budget,success=False,reason='budget_timeout',
                lbfgs_wall_ms=budget*1000,wall_ms=99999.))
        done=dict(complete=True,frozen_stable=True,same_first_fg=True,audits_complete=True,
            native_dynamic_observed_stable=True,diagnostic_candidates_published=False)
        return rows,failures,done

    def analyze(self,rows,failures,done,timeline=None):
        if timeline is None:timeline=[dict(request_id=1,event='state_contract_reference_publish')]
        return summarize_paused(rows,failures,done,timeline)

    def test_solver_time_never_uses_post_audit_total(self):
        answer=self.analyze(*self.fixture())
        self.assertTrue(answer['diagnosis_record_contract_valid']);self.assertFalse(answer['navigation_metrics_valid'])
        self.assertEqual([p['solver_wall_ms'] for p in answer['passes']],[600.,600.,1500.])
        self.assertFalse(answer['complete_native_map_identity_proven'])

    def test_same_first_fg_alone_insufficient(self):
        for field in ('raw_seq','effective_initial_states','prediction_hash','sim_end','wheel_limit'):
            rows,failures,done=self.fixture();rows[3][field]='changed'
            self.assertFalse(self.analyze(rows,failures,done)['diagnosis_record_contract_valid'])

    def test_missing_duplicate_mutated_input_and_publication_rejected(self):
        rows,failures,done=self.fixture()
        for damaged in (rows[:-1],rows+[copy.deepcopy(rows[1])]):
            self.assertFalse(self.analyze(damaged,failures,done)['diagnosis_record_contract_valid'])
        failures[0]['input_hash']='124'
        self.assertFalse(self.analyze(rows,failures,done)['diagnosis_record_contract_valid'])
        rows,failures,done=self.fixture();done['diagnostic_candidates_published']=True
        self.assertFalse(self.analyze(rows,failures,done)['diagnosis_record_contract_valid'])

    def test_no_capture_no_invented_completion(self):
        answer=self.analyze(None,None,{})
        self.assertFalse(answer['diagnosis_record_contract_valid']);self.assertEqual(answer['capture_count'],0)
        self.assertTrue(all(p['solver_wall_ms'] is None for p in answer['passes']))

    def test_failure_verdict_disagreement_rejected(self):
        rows,failures,done=self.fixture();failures[1]['success']=True
        self.assertFalse(self.analyze(rows,failures,done)['diagnosis_record_contract_valid'])

    def test_new_native_dynamic_proof_required_and_wrong_budget_rejected(self):
        rows,failures,done=self.fixture();done.pop('native_dynamic_observed_stable')
        self.assertFalse(self.analyze(rows,failures,done)['diagnosis_record_contract_valid'])
        rows,failures,done=self.fixture();failures[0]['budget_s']=2.
        self.assertFalse(self.analyze(rows,failures,done)['diagnosis_record_contract_valid'])

    def test_captured_request_publication_not_allowed(self):
        answer=self.analyze(*self.fixture(),timeline=[dict(request_id=2,event='state_contract_reference_publish')])
        self.assertFalse(answer['diagnosis_record_contract_valid'])
        self.assertFalse(answer['observed_no_captured_request_publication'])

if __name__=='__main__':unittest.main()
