"""Exact, isolated active-arm child transforms for optional MPPI refreshes.

No production or parent file is written. transform_sources receives the four
named original source strings and returns four modified strings. The child
builder must also copy arm_box_refresh_policy.hpp into its source directory.
"""
from pathlib import Path


def once(text, old, new, name):
    if text.count(old) != 1:
        raise ValueError('%s: expected one source anchor, found %d' % (name, text.count(old)))
    return text.replace(old, new, 1)


SEED_CONTEXT = r'''
// One immutable whole-body suggestion and epoch per optional request. The
// live subscriber store remains separate and is never overwritten by a scope.
inline arm_box_refresh::Identity& currentSeedIdentity(){
  static arm_box_refresh::Identity value;return value;
}
struct SeedContext {
  const Seed* value=nullptr;double epoch=0.;bool strict_prefix=false;
  arm_box_refresh::Identity identity;
};
inline SeedContext& seedContext(){static thread_local SeedContext value;return value;}
inline const Seed& seedView(){return seedContext().value?*seedContext().value:seed();}
inline double seedEpoch(){return seedContext().value?seedContext().epoch:ros::Time::now().toSec();}
inline const arm_box_refresh::Identity& seedIdentity(){
  return seedContext().value?seedContext().identity:currentSeedIdentity();
}
inline bool strictPrefix(){return seedContext().value&&seedContext().strict_prefix;}
struct SeedScope {
  Seed frozen;SeedContext previous;
  SeedScope(const Seed& source,double epoch,const arm_box_refresh::Identity& identity,bool strict=true)
      :frozen(source),previous(seedContext()){
    seedContext().value=&frozen;seedContext().epoch=epoch;
    seedContext().identity=identity;seedContext().strict_prefix=strict;
  }
  ~SeedScope(){seedContext()=previous;}
  SeedScope(const SeedScope&)=delete;SeedScope& operator=(const SeedScope&)=delete;
};
inline bool seedSourcesFresh(const Seed& s,double epoch){
  arm_box_refresh::SourceTimes stamps;
  stamps.stamp=s.stamp;stamps.odom=s.odom;stamps.cloud=s.cloud;stamps.dynamic=s.dynamic;
  stamps.joint=active_arm_coupling::armEnabled()?s.arm.joint_stamp:s.odom;
  return arm_box_refresh::freshSources(stamps,epoch);
}
inline bool seedIdentityMatches(const Seed& s){
  const auto& identity=seedIdentity();
  return identity.generation>0&&identity.revision>0&&
    s.generation==double(identity.generation)&&s.revision==double(identity.revision)&&
    s.goal==double(identity.goal);
}
inline std::uint64_t seedFingerprint(const Seed& s){
  std::uint64_t h=1469598103934665603ULL;
  const auto add=[&](double value){const auto* p=reinterpret_cast<const unsigned char*>(&value);
    for(std::size_t i=0;i<sizeof(value);++i){h^=p[i];h*=1099511628211ULL;}};
  for(double v:{s.stamp,s.generation,s.revision,s.goal,s.odom,s.cloud,s.dynamic,s.arm.joint_stamp})add(v);
  for(const auto& row:s.rows)for(double v:row)add(v);
  for(const auto& row:s.arm.rows)for(int j=0;j<6;++j){add(row.arm.q[j]);add(row.arm.dq[j]);}
  return h;
}
'''

PREFIX_ROWS = r'''inline std::array<double,8> baseAt(const Seed& s,double time){
  if(s.rows.size()<2||time<s.rows.front()[0]||time>s.rows.back()[0])
    throw std::invalid_argument("prefix origin outside checked interval");
  std::size_t k=1;while(k<s.rows.size()&&s.rows[k][0]<time)++k;
  if(k==s.rows.size())k=s.rows.size()-1;
  const auto&a=s.rows[k-1];const auto&b=s.rows[k];
  const double fraction=(time-a[0])/(b[0]-a[0]);std::array<double,8> row;
  for(int i=0;i<8;++i)row[i]=a[i]+fraction*(b[i]-a[i]);
  row[0]=time;row[3]=a[3]+fraction*angle(b[3]-a[3]);return row;
}
inline bool prefixRows(const Eigen::VectorXd& start,double yaw,int direction,
                       std::vector<std::array<double,8>>& rows,std::string& reason,double* source_origin=nullptr) {
  const auto& s=seedView();const double now=seedEpoch(),age=now-s.stamp;
  reason="stale_or_missing";
  if(s.rows.size()<3||!seedSourcesFresh(s,now))return false;
  if(!seedIdentityMatches(s)){reason="reference_epoch_mismatch";return false;}
  const double origin=std::max(s.rows.front()[0],std::max(0.,age));
  if(origin>s.rows.back()[0]){reason="short_prefix";return false;}
  const auto first=baseAt(s,origin);
  if(std::hypot(first[1]-start(0),first[2]-start(1))>.08||std::abs(angle(first[3]-yaw))>.25){
    reason="start_mismatch";return false;
  }
  const int sign=direction==0?(first[4]<0?-1:1):direction;
  rows.clear();double previous=-1.;
  const auto append=[&](std::array<double,8> row){
    row[0]-=origin;
    if(std::abs(row[4])<.015||row[4]*sign<=0){reason="gear_or_low_speed";return false;}
    if(row[0]-previous<.18&&!rows.empty())return true;
    rows.push_back(row);previous=row[0];return true;
  };
  if(!append(first))return false;
  for(const auto& row:s.rows){
    if(row[0]<=origin+1e-10)continue;
    if(row[0]-origin>1.0+1e-6)break;
    if(!append(row))break;
  }
  if(rows.size()<3||rows.back()[0]<.4||
     std::hypot(rows.back()[1]-start(0),rows.back()[2]-start(1))<.06){reason="short_prefix";return false;}
  if(source_origin)*source_origin=origin;
  reason="eligible";return true;
}
'''

OPTIONAL_REFRESH = r'''  // A required blocked-guide repair or partial continuation keeps its native
  // fallback. Only a pure fast-event refresh requires a usable MPPI prefix.
  static arm_box_refresh::Pending optional_pending;
  const arm_box_refresh::Identity identity{feedback_identity.generation,
      feedback_identity.revision,feedback_identity.goal_token};
  const auto packet=arm_box_refresh::packet(feedback.latest);
  const bool required=blocked||continue_partial;
  const auto action=arm_box_refresh::decide(optional_pending,packet,identity,now.toSec(),
      planner_manager_->grid_map_->getDynamicObservationSeq(),
      planner_manager_->grid_map_->getESDFObservationSeq(),required,fast_feedback_request,false);
  if(required){
    if(!refresh_ready)return;
    const char* source=blocked?"guide_blocked":"partial";
    dual_probe::emit("refresh_trigger",dual_probe::Fields().add("guide_id",info.traj_id)
      .add("blocked",blocked?"true":"false").add("continue_partial",continue_partial?"true":"false")
      .add("fast_feedback_trigger","false").str("source",source));
    last_cooperative_replan_=now;flag_relan_astar_=true;
    changeFSMExecState(REPLAN_TRAJ,"LOCAL_EXECUTION_REQUIRED_REFRESH");return;
  }
  if(action==arm_box_refresh::Action::CancelIdentity||action==arm_box_refresh::Action::CancelResolved){
    dual_probe::emit("optional_refresh_cancel",dual_probe::Fields().str("reason",
      action==arm_box_refresh::Action::CancelIdentity?"identity_changed":"avoidance_resolved"));return;
  }
  if(action==arm_box_refresh::Action::Idle)return;
  if(action==arm_box_refresh::Action::WaitMap){
    dual_probe::emit("optional_refresh_defer",dual_probe::Fields().str("reason","esdf_pending")
      .add("required_seq",optional_pending.required_esdf)
      .add("esdf_seq",planner_manager_->grid_map_->getESDFObservationSeq())
      .add("episode",optional_pending.event.episode));return;
  }
  if(!refresh_ready||now<next_cooperative_attempt_)return;
  // Stay in EXEC_TRAJ while inputs are missing: no zero-speed owner and no
  // suspension of the next blocked/partial scan. This scope freezes one seed
  // and one epoch for the preflight, initializer and objective.
  collab::SeedScope frozen(collab::seed(),now.toSec(),identity,true);
  const auto source_fields=[&](){
    const auto& source=collab::seedView();const double epoch=collab::seedEpoch();
    dual_probe::Fields fields;
    fields.add("seed_stamp",source.stamp).add("seed_epoch",epoch)
      .str("seed_hash",std::to_string(collab::seedFingerprint(source)))
      .add("goal_token",source.goal).add("generation",source.generation).add("revision",source.revision)
      .add("episode",optional_pending.event.episode).add("required_seq",optional_pending.required_esdf)
      .add("seed_age_s",epoch-source.stamp).add("odom_age_s",epoch-source.odom)
      .add("joint_age_s",epoch-source.arm.joint_stamp)
      .add("cloud_age_s",epoch-source.cloud).add("dynamic_age_s",epoch-source.dynamic);
    return fields;
  };
  bool valid=false;std::string reason;
  {
    coupling_bridge::Request request(execution_goal_key_,mm_state_pos_,mm_car_yaw_,mm_car_singul_);
    valid=request.value.valid&&request.value.targets.has_delivery&&
      (!active_arm_coupling::enabled()||request.value.arm_valid);
    reason=request.value.reason;
    std::vector<std::array<double,8>> prefix;double origin=0.;
    if(valid)valid=collab::settings().prefix&&collab::prefixRows(mm_state_pos_,mm_car_yaw_,
      mm_car_singul_,prefix,reason,&origin);
    if(valid&&active_arm_coupling::armEnabled()){
      const auto arm=active_arm_coupling::interpolate(collab::seedView().arm,origin);
      valid=(arm.q-mm_state_pos_.tail(6)).cwiseAbs().maxCoeff()<=.10&&
            (arm.dq-mm_state_vel_.tail(6)).cwiseAbs().maxCoeff()<=.30;
      if(!valid)reason="active_arm_prefix_measurement_mismatch";
    }
  }
  if(!valid){
    auto fields=source_fields();fields.str("reason",reason).add("valid","false");
    dual_probe::emit("optional_refresh_defer",fields);return;
  }
  auto input_fields=source_fields();input_fields.add("valid","true");
  dual_probe::emit("arm_box_refresh_input",input_fields);
  arm_box_refresh::Packet attempt;
  if(!arm_box_refresh::beginAttempt(optional_pending,attempt))return;
  feedback.consumed_generation=attempt.identity.generation;
  feedback.consumed_episode=attempt.episode;
  last_cooperative_replan_=now;flag_relan_astar_=true;
  dual_probe::emit("refresh_trigger",dual_probe::Fields().add("guide_id",info.traj_id)
    .add("blocked","false").add("continue_partial","false").add("fast_feedback_trigger","true")
    .add("episode",attempt.episode).str("source","fast_avoidance_feedback")
    .add("seed_stamp",collab::seedView().stamp).add("seed_epoch",collab::seedEpoch())
    .str("seed_hash",std::to_string(collab::seedFingerprint(collab::seedView()))));
  const bool refreshed=planFromLocalTraj(true,false,dynamic_astar_replan_wall_time_,false);
  next_cooperative_attempt_=ros::Time::now()+ros::Duration(.75);
  flag_relan_astar_=false;
  if(refreshed)replan_fail_time_=0;
  dual_probe::emit("refresh_attempt_result",dual_probe::Fields().add("success",refreshed?"true":"false")
    .add("optional","true").add("episode",attempt.episode)
    .add("next_attempt_sim",next_cooperative_attempt_.toSec()));
  // Failure keeps the last published guide (restored by callReboundReplan).
  // No automatic retry of this optional episode; required repairs stay live.
'''


def transform_sources(original):
    result = dict(original)
    name = 'collab_factorial.hpp'
    text = result[name]
    text = once(text, '#include "active_arm_coupling.hpp"',
                '#include "active_arm_coupling.hpp"\n#include "arm_box_refresh_policy.hpp"', name+' include')
    text = once(text, 'inline Seed& seed(){static Seed value;return value;}\n',
                'inline Seed& seed(){static Seed value;return value;}\n'+SEED_CONTEXT, name+' context')
    start = text.index('inline bool prefixRows(')
    end = text.index('\ntemplate<class Manager>\nint search(', start)
    text = text[:start] + PREFIX_ROWS + text[end:]
    start = text.index('bool prefixSearch(')
    end = text.index('// Diagnostic-only paired requests.', start)
    section = text[start:end].replace('seed()', 'seedView()').replace('ros::Time::now().toSec()', 'seedEpoch()')
    section = once(section, '.add("seed_revision",seedView().revision)',
        '.add("seed_epoch",seedEpoch()).str("seed_hash",std::to_string(seedFingerprint(seedView())))\n    .add("seed_origin",arm_prefix_origin).add("seed_generation",seedView().generation)\n    .add("seed_revision",seedView().revision)', name+' prefix evidence')
    result[name] = text[:start] + section + text[end:]

    name = 'coupling_bridge.hpp'
    text = result[name]
    text = once(text,
        '    activePrediction()=&value;value.seed=collab::seed();value.request_time=ros::Time::now().toSec();',
        '''    collab::currentSeedIdentity()={goalGeneration(),referenceRevision(),dual_probe::hash(goal)&0xffffffffULL};
    activePrediction()=&value;value.seed=collab::seedView();value.request_time=collab::seedEpoch();''', name+' request seed')
    text = once(text, '''    if(age<-.05||age>.25||value.request_time-s.odom>.25||value.request_time-s.cloud>.35||
       value.request_time-s.dynamic>.35){value.reason="stale";return;}''',
        '''    if(!collab::seedSourcesFresh(s,value.request_time)){value.reason="stale";return;}''', name+' all source stamps')
    text = once(text, '.add("seed_stamp",prediction.seed.stamp).add("seed_age_s",prediction.request_time-prediction.seed.stamp)',
        '.add("seed_stamp",prediction.seed.stamp).add("seed_epoch",prediction.request_time)\n      .str("seed_hash",std::to_string(collab::seedFingerprint(prediction.seed)))\n      .add("seed_age_s",prediction.request_time-prediction.seed.stamp)', name+' objective evidence')
    result[name] = text

    name = 'planner_manager.cpp'
    text = result[name]
    text = once(text, '      if(prefix_used)status=KinoAstar::REACH_END;\n      else {',
        '''      if(prefix_used)status=KinoAstar::REACH_END;
      else if(collab::strictPrefix()){
        collab::note("optional_prefix_rejected",dual_probe::Fields().str("fallback","disabled_for_optional_attempt"));
        return false;
      }
      else {''', name+' optional fallback')
    result[name] = text

    name = 'remani_replan_fsm.cpp'
    text = result[name]
    start = text.index('  if((blocked||continue_partial||fast_feedback_request)&&refresh_ready){')
    end = text.index('\n}\n\n  REMANIReplanFSM::~REMANIReplanFSM()', start)
    text = text[:start] + OPTIONAL_REFRESH + text[end:]
    result[name] = text
    return result
