// Ensure a rejected trial cannot replace an actual accepted iterate, and the
// recovery switch does not generalize to other errors, timeouts or no iterate.
#include "coverage_collision.hpp"
#include "candidate_exit_audit.hpp"
#include <iostream>

int main(){
  candidate_exit_audit::Scope recorded(true);
  Eigen::VectorXd a(2),b(2),g(2);a<<1.,2.;b<<3.,4.;g<<.1,.2;
  candidate_exit_audit::completed(a,g,10.);
  const bool initial=recorded.accepted.valid&&recorded.accepted.x_hash==delivery_candidate::hash(a);
  candidate_exit_audit::completed(b,2.*g,20.);
  candidate_exit_audit::accepted(a,10.);
  const bool trial_is_not_accepted=recorded.accepted.x_hash==delivery_candidate::hash(a);
  candidate_exit_audit::accepted(b,20.);
  const bool accepted_updated=recorded.accepted.x_hash==delivery_candidate::hash(b)&&recorded.accepted.cost==20.;
  {candidate_exit_audit::Scope excluded(false);candidate_exit_audit::completed(a,g,5.);}
  const bool nesting=recorded.accepted.x_hash==delivery_candidate::hash(b)&&recorded.evaluated.cost==20.;
  setenv("CANDIDATE_EXIT_RECOVER","1",1);
  const bool narrow=candidate_exit_audit::canRecover(true,domain_lbfgs::LBFGSERR_MAXIMUMLINESEARCH,0,true)&&
    !candidate_exit_audit::canRecover(true,domain_lbfgs::LBFGSERR_INVALID_FUNCVAL,0,true)&&
    !candidate_exit_audit::canRecover(true,domain_lbfgs::LBFGSERR_MAXIMUMLINESEARCH,3,true)&&
    !candidate_exit_audit::canRecover(true,domain_lbfgs::LBFGSERR_MAXIMUMLINESEARCH,0,false)&&
    !candidate_exit_audit::canRecover(false,domain_lbfgs::LBFGSERR_MAXIMUMLINESEARCH,0,true);
  unsetenv("CANDIDATE_EXIT_RECOVER");
  const bool off=!candidate_exit_audit::canRecover(true,domain_lbfgs::LBFGSERR_MAXIMUMLINESEARCH,0,true);
  const bool pass=initial&&trial_is_not_accepted&&accepted_updated&&nesting&&narrow&&off;
  std::cout<<"{\"test\":\"actual accepted versus rejected trial; nested diagnostic isolation; narrow independent recovery switch\",\"initial_retained\":"<<(initial?"true":"false")
    <<",\"rejected_trial_not_accepted\":"<<(trial_is_not_accepted?"true":"false")<<",\"accepted_updated\":"<<(accepted_updated?"true":"false")
    <<",\"diagnostic_scope_isolated\":"<<(nesting?"true":"false")<<",\"narrow_recovery_conditions\":"<<(narrow?"true":"false")
    <<",\"default_off\":"<<(off?"true":"false")<<",\"pass\":"<<(pass?"true":"false")<<"}\n";
  return pass?0:1;
}
