#include "short_prefix_rejoin.hpp"
#include <iostream>
#include <map>

int main(){
  namespace sr=short_prefix_rejoin;
  std::map<std::string,bool> tests;
  std::vector<std::array<double,8>> rows(4);
  rows[0]={0.,0.,0.,0.,.2,0.,0.,0.};
  rows[1]={.1,0.,0.,0.,.1,0.,0.,0.};
  rows[2]={.3,0.,0.,0.,-.2,0.,0.,0.};
  rows[3]={1.6,0.,0.,0.,-.3,0.,0.,0.};
  const auto hint=sr::reverseHint(rows,.05,1);
  tests["future_reverse_hint"]=hint.valid&&hint.gear==-1&&hint.row_index==2&&
    std::abs(hint.dt_s-.25)<1e-12;
  tests["inactive_and_missing_hint_keep_original"]=sr::switchPenalty(10.)==10.&&
    !sr::activate(false,true,true,1,hint)&&!sr::activate(true,true,true,1,sr::Hint());
  tests["only_short_prefix_fallback_activates"]=sr::activate(true,true,true,1,hint)&&
    !sr::activate(true,false,true,1,hint)&&!sr::activate(true,true,false,1,hint);
  tests["zero_current_gear_disables"]=!sr::reverseHint(rows,.05,0).valid&&
    !sr::activate(true,true,true,0,hint);
  tests["past_or_beyond_one_second_not_used"]=!sr::reverseHint(rows,.5,1).valid&&
    !sr::reverseHint(rows,-1.,1).valid;
  sr::Context outer;outer.active=false;
  {
    sr::ScopedContext bind_outer(outer);
    tests["inactive_scope_keeps_penalty"]=sr::switchPenalty(10.)==10.&&
      outer.switch_penalty_evaluations==1&&outer.zeroed_switch_penalty_evaluations==0;
    sr::Context active;active.active=true;
    {
      sr::ScopedContext bind_active(active);
      const bool repeated=sr::switchPenalty(10.)==0.&&sr::switchPenalty(10.)==0.;
      tests["active_uniform_coefficient_is_nonnegative"]=repeated&&
        active.switch_penalty_evaluations==2&&active.zeroed_switch_penalty_evaluations==2;
    }
    tests["nested_exit_restores_previous_scope"]=sr::current()==&outer&&sr::switchPenalty(10.)==10.;
  }
  tests["search_exit_restores_original"]=sr::current()==nullptr&&sr::switchPenalty(10.)==10.;
  bool pass=true;for(const auto& entry:tests)pass=pass&&entry.second;
  std::cout<<"{\"pass\":"<<(pass?"true":"false");
  for(const auto& entry:tests)
    std::cout<<",\""<<entry.first<<"\":"<<(entry.second?"true":"false");
  std::cout<<",\"scope\":\"hint selection and uniform per-search coefficient policy; no map-safety claim\"}\n";
  return pass?0:1;
}
