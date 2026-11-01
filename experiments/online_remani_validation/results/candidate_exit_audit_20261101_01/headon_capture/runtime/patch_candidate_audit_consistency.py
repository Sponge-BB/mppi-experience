"""Patch a NEW child only: observe the moving overlay and pin its current KD.

No refresh is called by a diagnostic pin. Historical source and manifests remain
unchanged; root applies these functions while preparing the next child.
"""
from build_state_contract import Transformer


def transform_helper(text):
    t = Transformer(text)
    t.after('#include <fstream>', '\n#include <functional>')
    t.after('  int iteration=0,line_search=0;\n  bool cost_seen=false;\n  std::array<double,8> costs{};', '''
  std::string evaluation_begin_key="null",evaluation_end_key="null";
''')
    t.after('  std::vector<Step> steps;', '''
  std::function<std::string()> context;
  std::string solver_entry_key="null",solver_exit_key="null";
  std::string evaluation_begin_key="null",evaluation_end_key="null";
  unsigned long long evaluate_complete_count=0,evaluate_overlay_changed_count=0;
''')
    t.change('  void begin(){if(enabled)start_wall=wall();}', '''
  void begin(){if(enabled){start_wall=wall();solver_entry_key=context?context():"null";}}
  void evaluationBegin(){if(enabled)evaluation_begin_key=context?context():"null";}
''')
    for label, fields in (
            ('initial', '0,0'), ('accepted', 'iteration,line_search')):
        t.after('    steps.push_back({"'+label+'",x,cost,wall()-start_wall,'+fields+'});', '''
    steps.back().evaluation_begin_key=evaluation_begin_key;
    steps.back().evaluation_end_key=evaluation_end_key;''')
    t.change('      vectorJson(out,s.x);out<<\'}\';', '''
      vectorJson(out,s.x);out<<",\\\"evaluation_begin_key\\\":"<<s.evaluation_begin_key
        <<",\\\"evaluation_end_key\\\":"<<s.evaluation_end_key<<'}';''')
    t.before('inline void recordInitial(', '''
inline void recordEvaluationBegin(){if(active())active()->evaluationBegin();}
inline std::string overlayContentKey(std::string key){
  for(const char* marker:{",\\\"frozen\\\":true",",\\\"frozen\\\":false"}){
    const std::string token=marker;const auto at=key.find(token);
    if(at!=std::string::npos)key.erase(at,token.size());
  }
  return key;
}
''')
    t.after('  auto*s=active();if(!s)return;s->cost_seen=true;s->costs[0]=smooth;', '''
  s->evaluation_end_key=s->context?s->context():"null";
  ++s->evaluate_complete_count;
  s->evaluate_overlay_changed_count+=overlayContentKey(s->evaluation_begin_key)!=overlayContentKey(s->evaluation_end_key);''')
    t.after('class GridMapParallelTestAccessor {\n public:', '''
  static std::string readOverlayKey(GridMap& map){
    std::lock_guard<std::mutex> lock(map.dynamic_point_overlay_mutex_);
    std::ostringstream out;
    out<<"{\\\"sequence\\\":"<<map.dynamic_point_overlay_sequence_
      <<",\\\"refresh_nsec\\\":"<<map.dynamic_point_overlay_refresh_time_.toNSec()
      <<",\\\"size\\\":"<<(map.dynamic_point_overlay_cloud_?map.dynamic_point_overlay_cloud_->size():0)
      <<",\\\"radii_size\\\":"<<map.dynamic_point_overlay_radii_.size()
      <<",\\\"cloud_address\\\":"<<candidate_audit::quote(std::to_string(
          reinterpret_cast<std::uintptr_t>(map.dynamic_point_overlay_cloud_.get())))
      <<",\\\"enabled\\\":"<<(map.mp_.dynamic_point_overlay_enable_?"true":"false")
      <<",\\\"ready\\\":"<<(map.dynamic_point_overlay_ready_?"true":"false")
      <<",\\\"frozen\\\":"<<(map.dynamic_point_overlay_frozen_?"true":"false")<<'}';
    return out.str();
  }
  static bool freezeCurrentOverlay(GridMap& map){
    std::lock_guard<std::mutex> lock(map.dynamic_point_overlay_mutex_);
    if(!map.mp_.dynamic_point_overlay_enable_||map.dynamic_point_overlay_frozen_)return false;
    // Pin exactly the present query state, including an empty/unready layer.
    // beginDynamicPointPlanningSnapshot is deliberately not called: it can refresh.
    map.dynamic_point_overlay_frozen_=true;
    map.dynamic_point_overlay_frozen_sequence_=map.dynamic_point_overlay_sequence_;
    return true;
  }
  static void releaseCurrentOverlay(GridMap& map){
    std::lock_guard<std::mutex> lock(map.dynamic_point_overlay_mutex_);
    map.dynamic_point_overlay_frozen_=false;
    map.dynamic_point_overlay_frozen_sequence_=0;
  }
''')
    return t.finish()


def transform_optimizer(text):
    t = Transformer(text)
    t.before('    candidate_record.begin();', '''
    if(candidate_record.enabled)candidate_record.context=[&](){
      return GridMapParallelTestAccessor::readOverlayKey(*grid_map_);
    };
    const char* candidate_solver_freeze_env=std::getenv("CANDIDATE_AUDIT_FREEZE_SOLVER");
    const bool candidate_solver_freeze_requested=candidate_record.enabled && candidate_solver_freeze_env &&
      std::strcmp(candidate_solver_freeze_env,"1")==0;
    const bool candidate_solver_overlay_owned=candidate_solver_freeze_requested &&
      GridMapParallelTestAccessor::freezeCurrentOverlay(*grid_map_);
    delivery_replay::Finally candidate_solver_overlay_release([&](){
      if(candidate_solver_overlay_owned)GridMapParallelTestAccessor::releaseCurrentOverlay(*grid_map_);
    });
''')
    t.before('    delivery_candidate::beginEvaluation();',
             '    candidate_audit::recordEvaluationBegin();\n')
    t.before('    const double candidate_solver_wall=', '''
    if(candidate_record.enabled && candidate_record.context)
      candidate_record.solver_exit_key=candidate_record.context();
''')
    t.after('    const auto native_validation_start=ros::WallTime::now();', '''
    const std::string candidate_native_gate_begin=candidate_record.context?candidate_record.context():"null";''')
    t.after('    good_traj = IsTrajSafe(singul_traj_data);', '''
    const std::string candidate_native_gate_end=candidate_record.context?candidate_record.context():"null";''')
    return t.finish()


def transform_finish(text):
    t = Transformer(text)
    t.after('      failure_probe::PauseOwned audit_pause;', '''
      const std::string audit_overlay_before_pin=GridMapParallelTestAccessor::readOverlayKey(*grid_map_);
      const bool audit_overlay_owned=GridMapParallelTestAccessor::freezeCurrentOverlay(*grid_map_);
      delivery_replay::Finally audit_overlay_release([&](){
        if(audit_overlay_owned)GridMapParallelTestAccessor::releaseCurrentOverlay(*grid_map_);
      });
      const std::string audit_overlay_key=GridMapParallelTestAccessor::readOverlayKey(*grid_map_);
''')
    t.after('        <<",\\\"sim_after_pause\\\":"<<audit_sim<<",\\\"map_metadata\\\":\\\"map.json\\\",\\\"map\\\":"<<map_metadata', '''
        <<",\\\"dynamic_map_history\\\":{\\\"solver_entry\\\":"<<candidate_record.solver_entry_key
        <<",\\\"solver_exit\\\":"<<candidate_record.solver_exit_key
        <<",\\\"native_gate_begin\\\":"<<candidate_native_gate_begin
        <<",\\\"native_gate_end\\\":"<<candidate_native_gate_end
        <<",\\\"before_audit_pin\\\":"<<audit_overlay_before_pin<<",\\\"audit_begin\\\":"<<audit_overlay_key
        <<",\\\"solver_freeze_requested\\\":"<<(candidate_solver_freeze_requested?"true":"false")
        <<",\\\"solver_pin_owned\\\":"<<(candidate_solver_overlay_owned?"true":"false")
        <<",\\\"audit_pin_owned\\\":"<<(audit_overlay_owned?"true":"false")
        <<",\\\"evaluate_complete_count\\\":"<<candidate_record.evaluate_complete_count
        <<",\\\"evaluate_overlay_changed_count\\\":"<<candidate_record.evaluate_overlay_changed_count
        <<",\\\"solver_entry_exit_key_equal\\\":"<<(candidate_record.solver_entry_key==candidate_record.solver_exit_key?"true":"false")
        <<",\\\"scope\\\":\\\"each recorded candidate is rechecked against the one current post-pause pinned overlay; this is not its original-time map, and initial collision here does not prove initial-time collision; ordinary online objectives may change with the independent dynamic worker\\\"}"
''')
    t.change('              Eigen::Matrix<double,8,1> beta;beta.setZero();\n'
              '              for(int power=1;power<8;++power)beta[power]=power*std::pow(local,power-1);', '''
              const double s1=local,s2=s1*s1,s3=s2*s1,s4=s2*s2,s5=s4*s1,s6=s3*s3;
              Eigen::Matrix<double,8,1> beta;
              beta<<0.,1.,2.*s1,3.*s2,4.*s3,5.*s4,6.*s5,7.*s6;''')
    t.change('              if(smoothedLog(.01-velocity.head(2).squaredNorm(),.001,factor,df)){',
              '              if(smoothedLog(.1*.1-velocity.head(2).squaredNorm(),.001,factor,df)){')
    t.after('          <<",\\\"cost_components_recorded\\\":"<<(step.cost_seen?"true":"false")', '''
          <<",\\\"evaluation_begin_key\\\":"<<step.evaluation_begin_key
          <<",\\\"evaluation_end_key\\\":"<<step.evaluation_end_key''')
    t.before('      const bool frozen_stable=', '''
      const std::string audit_overlay_end=GridMapParallelTestAccessor::readOverlayKey(*grid_map_);
      const bool audit_overlay_stable=audit_overlay_key==audit_overlay_end;
''')
    t.after('        <<",\\\"solver_map_generations_stable\\\":"<<(solver_map_stable?"true":"false")', '''
        <<",\\\"overlay_key_stable\\\":"<<(audit_overlay_stable?"true":"false")
        <<",\\\"audit_overlay_begin\\\":"<<audit_overlay_key<<",\\\"audit_overlay_end\\\":"<<audit_overlay_end''')
    t.change('<<(frozen_stable&&solver_map_stable?"true":"false")',
              '<<(frozen_stable&&solver_map_stable&&audit_overlay_stable?"true":"false")')
    return t.finish()
