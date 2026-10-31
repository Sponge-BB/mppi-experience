#!/usr/bin/env python3
"""Fail-closed, reversible isolated seed geometry/time repair; no file writes."""
from pathlib import Path
import hashlib
from verify_delivery_seed_geometry import extract_function

HERE=Path(__file__).resolve().parent
DEFAULT_SOURCE=HERE/'builds/delivery_stability_20261024_02/source/kino_astar.cpp'
INCLUDE='#include "seed_parameterization.hpp"\n'
FUNCTION_SHA256={
    'getTrajsWithTime':'f0764a72da61eb08b491f37c45d54ea044551d4ec76fc473d85e16460e39574f',
    'evaluateCurvatureWeightedDistance':'db4f9fc70439032b2fb95d2c10b33acfafc69e4d29d4edaf42280dea9051a595',
    'evaluateDuration':'974c6cd67da4ea7bb47f096f18ef44b6ba716ad9f6c577bee0e4558323272772',
    'evaluateLength':'c90a018dcaf9547960f9214acb623934f6d26bfd87bf5ae4d6f6bde1de256241',
    'evaluatePos':'71b5a73c628787907a6a222491e2eed2cd4e083fa5525e68e6355eab1d7bb13d',
}
REPLACEMENTS={
 'getTrajsWithTime':'''void KinoAstar::getTrajsWithTime(){
  // Transactional: no stale or partial seed may survive a failed parameterization.
  flat_trajs_.clear(); shot_lengthList.clear(); shot_timeList.clear();
  shotindex.clear(); shot_SList.clear(); totalTrajTime=0.;
  auto& seed_status=seed_parameterization::status();
  seed_status=seed_parameterization::Status();seed_status.applied=true;
  seed_status.input_points=sample_trajs_.size();seed_status.reason="invalid_geometry_or_time_profile";
  if(start_state_.size()<4||end_state_.size()<4||!start_state_.allFinite()||!end_state_.allFinite()) return;
  seed_status.signed_start_speed=start_state_(3);
  const auto result=seed_parameterization::build<CarFlatTrajData>(sample_trajs_,
    std::abs(start_state_(3)),std::abs(end_state_(3)),start_ctrl_,non_siguav_,sample_time_,
    max_vel_,max_acc_,min_turning_radius_,curvatureDisCoe_,
    [this](const Eigen::Vector4d& state,const Eigen::Vector2d& control,int gear,Eigen::MatrixXd& flat){
      getFlatState(state,control,gear,flat);
    });
  if(!result.valid) return;
  if(start_state_(3)!=0. && result.gears.front()*(start_state_(3))<0.) {
    seed_status.reason="initial_gear_conflict";return;
  }
  if(end_state_(3)!=0. && result.gears.back()*(end_state_(3))<0.) {
    seed_status.reason="final_gear_conflict";return;
  }
  sample_trajs_=result.path; shot_lengthList=result.lengths; shot_timeList=result.times;
  shotindex=result.indices; shot_SList=result.gears; flat_trajs_=result.flats;
  totalTrajTime=result.total;
  seed_status.valid=true;seed_status.reason="ok";seed_status.output_segments=result.flats.size();
  seed_status.first_gear=result.gears.front();
  return;
}''',
 'evaluateCurvatureWeightedDistance':'''double KinoAstar::evaluateCurvatureWeightedDistance(const Eigen::Vector2d &state1, const Eigen::Vector2d &state2, const double &yaw1, const double &yaw2){
  return seed_parameterization::distance(state1,state2,yaw1,yaw2,min_turning_radius_,curvatureDisCoe_);
}''',
 'evaluateDuration':'''double KinoAstar::evaluateDuration(const double &length, const double &startV, const double &endV){
  return seed_parameterization::duration(length,startV,endV,max_vel_,max_acc_);
}''',
 'evaluateLength':'''double KinoAstar::evaluateLength(const double &curt, const double &locallength, const double &localtime, const double &startV, const double &endV){
  return seed_parameterization::lengthAt(curt,locallength,localtime,startV,endV,max_vel_,max_acc_);
}''',
 'evaluatePos':'''Eigen::Vector3d KinoAstar::evaluatePos(const double &input_t){
  if(start_state_.size()<4||end_state_.size()<4) return Eigen::Vector3d::Constant(seed_parameterization::invalid());
  return seed_parameterization::position(sample_trajs_,shot_lengthList,shot_timeList,shotindex,
    input_t,std::abs(start_state_(3)),std::abs(end_state_(3)),non_siguav_,max_vel_,max_acc_,
    min_turning_radius_,curvatureDisCoe_);
}''',
}

WRAPPER_EVENT='''
    {
      const bool seed_enabled=seed_parameterization::enabled();
      const auto seed_status=seed_parameterization::status();
      const auto input_points=seed_enabled?seed_status.input_points:sample_trajs_.size();
      const auto segments=flat_trajs_.size();
      const bool seed_valid=segments>0&&std::isfinite(totalTrajTime)&&totalTrajTime>0.;
      const auto reason=seed_enabled?seed_status.reason:"legacy_not_repaired";
      const auto first_gear=segments?flat_trajs_.front().singul:0;
      // search() may overwrite its wrapper argument with the end state.
      // start_state_ retains the actual native initial boundary.
      const auto signed_start=start_state_(3);
      auto record=state_contract_trace::record("seed_parameterization",dual_probe::Fields());
      record.postsolve=[seed_enabled,seed_status,input_points,segments,seed_valid,reason,first_gear,signed_start]
          (dual_probe::Fields& fields) {
        fields.str("seed_parameterization_schema","seed_parameterization_v1")
          .add("enabled",seed_enabled?"true":"false")
          .add("applied",seed_enabled&&seed_status.applied?"true":"false")
          .add("valid",seed_valid?"true":"false").str("reason",reason)
          .add("input_points",input_points).add("output_segments",segments)
          .add("signed_start_speed",signed_start).add("first_gear",first_gear)
          .add("probe_call_id_available","false")
          .str("association_scope","pid/request only; no public wrapper probe id accessor");
      };
      state_contract_trace::enqueue(std::move(record));
    }
    if(seed_parameterization::enabled() &&
       (flat_trajs_.empty()||!std::isfinite(totalTrajTime)||totalTrajTime<=0.)) {
      delivery_search_scope.finish("invalid_seed_parameterization",KinoAstar::NO_PATH,iter_num_,use_node_num_,open_set_.size());
      return KinoAstar::NO_PATH;
    }
'''
WRAPPER_PATCHES=[
 ('    getKinoNode();\n','    getKinoNode();\n'+WRAPPER_EVENT),
 ('        pos = evaluatePos(basetime + res_time - 1.0e-2);',
  '        pos = evaluatePos(basetime + res_time - (seed_parameterization::enabled()?0.:1.0e-2));'),
]

def _extract(text,name):
    if name!='evaluatePos':return extract_function(text,name)
    native='Eigen::Vector3d KinoAstar::evaluatePos('
    if text.count(native)!=1:raise ValueError('Missing/ambiguous evaluatePos')
    modified=text.replace(native,'double KinoAstar::evaluatePos(',1)
    result=extract_function(modified,name)
    result['text']=result['text'].replace('double KinoAstar::evaluatePos(',native,1)
    result['sha256']=hashlib.sha256(result['text'].encode()).hexdigest()
    return result

def _wrapped(original,replacement):
    opening=original.index('{');new_opening=replacement.index('{')
    return original[:opening+1]+'\n  if(seed_parameterization::enabled()) {\n'+replacement[new_opening+1:-1]+\
        '\n  }\n'+original[opening+1:]

def _functions(text):
    result={name:_extract(text,name) for name in FUNCTION_SHA256}
    for name,function in result.items():
        if function['sha256']!=FUNCTION_SHA256[name]:
            raise ValueError('Unattested seed function: '+name)
    return result

def transform_search(text):
    if INCLUDE in text or 'seed_parameterization::' in text:
        raise ValueError('Seed repair already applied')
    functions=_functions(text)
    result=text
    for name in FUNCTION_SHA256:
        old=functions[name]['text']
        if result.count(old)!=1:raise ValueError('Ambiguous seed function: '+name)
        result=result.replace(old,_wrapped(old,REPLACEMENTS[name]),1)
    for old,new in WRAPPER_PATCHES:
        if result.count(old)!=1:raise ValueError('Wrapper seed anchor drift: '+repr(old))
        result=result.replace(old,new,1)
    return INCLUDE+result

def project_off(text,source=DEFAULT_SOURCE):
    original=_functions(Path(source).read_text())
    if not text.startswith(INCLUDE):raise ValueError('Missing seed repair include')
    result=text[len(INCLUDE):]
    for name in reversed(tuple(FUNCTION_SHA256)):
        new=_wrapped(original[name]['text'],REPLACEMENTS[name])
        if result.count(new)!=1:raise ValueError('Patched seed function drift: '+name)
        result=result.replace(new,original[name]['text'],1)
    for old,new in reversed(WRAPPER_PATCHES):
        if result.count(new)!=1:raise ValueError('Wrapper seed source drift')
        result=result.replace(new,old,1)
    return result

def self_test(source=DEFAULT_SOURCE):
    original=Path(source).read_text();patched=transform_search(original)
    assert project_off(patched,source)==original
    assert extract_function(patched,'getFlatState')['text']==extract_function(original,'getFlatState')['text']
    return dict(exact_off_projection=True,native_flat_state_unchanged=True,
                changes=['translation-invariant positive reverse/forward metric',
                         'same 0.7 acceleration in duration and length',
                         'finite positive transactional segment output'],
                production_modified=False,ros_started=False)

if __name__=='__main__':print(self_test())
