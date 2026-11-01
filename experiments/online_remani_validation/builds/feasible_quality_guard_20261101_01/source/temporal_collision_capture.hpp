#pragma once
// Isolated SIL same-input evidence. Copies only observations, overlay metadata,
// native fixed-arm sphere geometry and one final polynomial per solve. File I/O
// is deferred until the request has made its publication/failure decision.
#include "temporal_collision_source.hpp"
#include "delivery_replay_probe.hpp"
#include "static_width_trace.hpp"
#include <memory>
#include <cstdint>
#include <cstring>

namespace temporal_capture {
inline bool enabled(){const char* p=std::getenv("TEMPORAL_COLLISION_CAPTURE_PATH");return p&&*p;}
struct Overlay {
  std::shared_ptr<const temporal_capture_source::Source> source;
  bool source_exact=false,enabled=false,ready=false,frozen=false;
  std::uint64_t sequence=0,frozen_sequence=0,cloud_hash=14695981039346656037ULL;
  std::size_t count=0;
  ros::Time refresh;
  double horizon=0,step=0,extra=0,cap=0,influence=0,maximum_prediction=0;
  double uncertainty0=0,uncertainty1=0,uncertainty2=0;
  Eigen::Vector3d minimum=Eigen::Vector3d::Zero(),maximum=Eigen::Vector3d::Zero();
};
inline void bytes(std::uint64_t& h,const void* data,std::size_t n){
  const auto* p=static_cast<const unsigned char*>(data);
  for(std::size_t i=0;i<n;++i){h^=p[i];h*=1099511628211ULL;}
}
}

// Child GridMap header grants this class friendship; no object layout changes.
class TemporalCollisionCaptureMapAccessor {
public:
  static temporal_capture::Overlay capture(GridMap& map){
    temporal_capture::Overlay out;
    std::lock_guard<std::mutex> lock(map.dynamic_point_overlay_mutex_);
    out.source=temporal_capture_source::publishedSource(&map);
    out.enabled=map.mp_.dynamic_point_overlay_enable_;
    out.ready=map.dynamic_point_overlay_ready_;out.frozen=map.dynamic_point_overlay_frozen_;
    out.sequence=map.dynamic_point_overlay_sequence_;out.frozen_sequence=map.dynamic_point_overlay_frozen_sequence_;
    out.refresh=map.dynamic_point_overlay_refresh_time_;
    out.source_exact=out.source&&out.source->snapshot.sequence==out.sequence&&out.source->refresh_epoch==out.refresh;
    out.horizon=map.mp_.dynamic_point_overlay_horizon_;out.step=map.mp_.dynamic_point_overlay_time_step_;
    out.extra=map.mp_.dynamic_point_overlay_extra_inflation_;out.cap=map.mp_.dynamic_point_overlay_max_inflation_;
    out.influence=map.mp_.dynamic_point_overlay_influence_distance_;
    out.maximum_prediction=map.dynamic_point_buffer_.maxPredictionTime();
    out.uncertainty0=map.dynamic_point_buffer_.configuredPredictionUncertainty(0.);
    out.uncertainty1=map.dynamic_point_buffer_.configuredPredictionUncertainty(1.);
    out.uncertainty2=map.dynamic_point_buffer_.configuredPredictionUncertainty(2.);
    out.minimum=map.mp_.map_min_boundary_;out.maximum=map.mp_.map_max_boundary_;
    out.count=map.dynamic_point_overlay_cloud_?map.dynamic_point_overlay_cloud_->size():0;
    if(out.count!=map.dynamic_point_overlay_radii_.size())throw std::runtime_error("temporal overlay point/radius mismatch");
    for(std::size_t i=0;i<out.count;++i){const auto& p=map.dynamic_point_overlay_cloud_->points[i];
      const float row[]={p.x,p.y,p.z,map.dynamic_point_overlay_radii_[i]};
      temporal_capture::bytes(out.cloud_hash,row,sizeof(row));}
    return out;
  }
};

namespace temporal_capture {
inline std::string overlayJson(const Overlay& o){
  std::ostringstream s;s<<std::setprecision(17)
    <<"{\"enabled\":"<<(o.enabled?"true":"false")<<",\"ready\":"<<(o.ready?"true":"false")
    <<",\"frozen\":"<<(o.frozen?"true":"false")<<",\"sequence\":"<<o.sequence
    <<",\"frozen_sequence\":"<<o.frozen_sequence<<",\"refresh_nsec\":"<<o.refresh.toNSec()
    <<",\"source_exact\":"<<(o.source_exact?"true":"false")<<",\"point_count\":"<<o.count
    <<",\"float32_xyzw_fnv1a64\":"<<delivery_diag::quote(std::to_string(o.cloud_hash))
    <<",\"horizon_s\":"<<o.horizon<<",\"step_s\":"<<o.step<<",\"extra_inflation_m\":"<<o.extra
    <<",\"maximum_inflation_m\":"<<o.cap<<",\"influence_distance_m\":"<<o.influence
    <<",\"maximum_prediction_s\":"<<o.maximum_prediction
    <<",\"uncertainty_at_0_1_2_s\":["<<o.uncertainty0<<','<<o.uncertainty1<<','<<o.uncertainty2<<']'
    <<",\"map_minimum\":"<<delivery_diag::vector(o.minimum)
    <<",\"map_maximum\":"<<delivery_diag::vector(o.maximum)
    <<",\"source_snapshot\":"<<(o.source?delivery_replay::nativeDynamicText(o.source->snapshot):"null")<<'}';
  return s.str();
}
inline void enqueue(const std::string& stage,std::function<std::string()> serialize){
  if(!enabled())return;
  auto record=state_contract_trace::record("temporal_collision_capture",dual_probe::Fields().str("stage",stage));
  const auto request=record.request_id;const double sim=record.capture_sim,mono=record.capture_mono;
  const std::string path=std::getenv("TEMPORAL_COLLISION_CAPTURE_PATH");
  record.postsolve=[stage,serialize,request,sim,mono,path](dual_probe::Fields& fields){
    const double begin=dual_probe::mono();
    const std::string row="{\"schema\":\"temporal_collision_capture_v1\",\"stage\":"+delivery_diag::quote(stage)+
      ",\"request_id\":"+std::to_string(request)+",\"capture_sim\":"+delivery_diag::number(sim)+
      ",\"capture_mono\":"+delivery_diag::number(mono)+","+serialize()+"}\n";
    std::ofstream out(path,std::ios::app);if(!out)throw std::runtime_error("temporal evidence log open failed");
    out<<row;if(!out)throw std::runtime_error("temporal evidence log write failed");
    fields.add("serialized_bytes",row.size()).add("serialization_wall_ms",1000.*(dual_probe::mono()-begin));
  };
  state_contract_trace::enqueue(std::move(record));
}
template<class Data> std::string candidateJson(const Data& data){
  std::string text="{\"start_epoch\":"+delivery_diag::number(data.start_time)+
    ",\"duration_s\":"+delivery_diag::number(data.duration)+",\"parts\":[";
  for(std::size_t i=0;i<data.singul_traj.size();++i){const auto& part=data.singul_traj[i];
    if(i)text+=',';text+="{\"gear\":"+std::to_string(part.singul)+",\"duration\":"+
      delivery_diag::number(part.duration)+",\"start_epoch\":"+delivery_diag::number(part.start_time)+",\"pieces\":[";
    for(int j=0;j<part.traj.getPieceNum();++j){if(j)text+=',';const auto& piece=part.traj.getPiece(j);
      text+="{\"T\":"+delivery_diag::number(piece.getDuration())+",\"basis\":\"descending\",\"coeffs\":"+
        delivery_diag::matrix(piece.getCoeffMat())+"}";}
    text+="]}";
  }return text+"]}";
}
struct Plan {
  unsigned long long request=0,input_hash=0,ordinal=0;double requested_epoch=0.;Overlay overlay;
  std::function<Overlay()> current_overlay;
};
inline std::shared_ptr<Plan>& currentPlan(){static thread_local std::shared_ptr<Plan> value;return value;}
inline unsigned long long& serial(){static thread_local unsigned long long value=0;return value;}
template<class Map,class Model,class Matrices>
void planning(Map& map,Model& model,const Matrices& initial,const Matrices& final,
              const std::vector<int>& signs,unsigned long long input_hash,double requested_epoch,
              const std::string& mode,double arm_radius){
  if(!enabled())return;
  const double began=dual_probe::mono();
  auto plan=std::make_shared<Plan>();plan->request=dual_probe::request();plan->input_hash=input_hash;
  plan->ordinal=++serial();plan->requested_epoch=requested_epoch;
  plan->overlay=TemporalCollisionCaptureMapAccessor::capture(map);currentPlan()=plan;
  plan->current_overlay=[&map](){return TemporalCollisionCaptureMapAccessor::capture(map);};
  Eigen::VectorXd arm=initial.empty()?Eigen::VectorXd():initial.front().col(0).tail(model.getManiDof()).eval();
  std::vector<Eigen::Vector3d> base;model.getCarPts(Eigen::Vector3d::Zero(),base);
  std::vector<std::array<double,5>> body;body.reserve(base.size()+32);
  for(std::size_t i=0;i<base.size();++i){const auto& p=base[i];
    body.push_back({{p.x(),p.y(),p.z(),model.getBaseSphereRadius(i)+model.getMapResolution(),0.}});}
  if(arm.size()==model.getManiDof()){
    std::vector<Eigen::Matrix4d> joints,unused;model.getJointTrans(arm,joints,unused);
    const auto links=model.getLinkPoint();Eigen::Matrix4d transform=model.getTq0();
    for(int i=0;i<model.getManiDof();++i){transform*=joints[i];for(int j=0;j<links[i].cols();++j){
      const Eigen::Vector3d p=(transform*links[i].col(j)).template head<3>();
      body.push_back({{p.x(),p.y(),p.z(),arm_radius,1.}});}}
  }
  const double capture_ms=1000.*(dual_probe::mono()-began);
  enqueue("planning",[plan,initial,final,signs,body,arm,mode,capture_ms](){
    std::ostringstream s;s<<std::setprecision(17)<<"\"plan_ordinal\":"<<plan->ordinal
      <<",\"input_hash\":"<<delivery_diag::quote(std::to_string(plan->input_hash))
      <<",\"requested_epoch\":"<<delivery_diag::number(plan->requested_epoch)<<",\"mode\":"<<delivery_diag::quote(mode)
      <<",\"overlay\":"<<overlayJson(plan->overlay)<<",\"initial_states\":"<<state_contract_trace::matricesJson(initial)
      <<",\"final_states\":"<<state_contract_trace::matricesJson(final)<<",\"gears\":"<<state_contract_trace::signsJson(signs)
      <<",\"fixed_arm_q\":"<<delivery_diag::vector(arm)<<",\"fixed_arm_body_columns\":\"x,y,z,hard_radius,type_base0_arm1\",\"fixed_arm_body\":[";
    for(std::size_t i=0;i<body.size();++i){if(i)s<<',';s<<'[';for(int j=0;j<5;++j){if(j)s<<',';s<<body[i][j];}s<<']';}
    s<<"],\"capture_copy_wall_ms\":"<<capture_ms
      <<",\"scope\":\"native hard sphere geometry; body is exact only while arm configuration stays fixed; no ESDF dump; dynamic model collision is not a physical-contact measurement\"";
    return s.str();
  });
}
template<class Data> void finalCandidate(const Data& data,bool success,const std::string& reason){
  if(!enabled())return;const auto plan=currentPlan();
  if(!plan||plan->request!=dual_probe::request())return;
  const auto copied=data;const auto final_overlay=plan->current_overlay();
  enqueue("final_candidate",[plan,copied,success,reason,final_overlay](){return std::string("\"plan_ordinal\":")+
    std::to_string(plan->ordinal)+",\"native_success\":"+(success?"true":"false")+
    ",\"native_reason\":"+delivery_diag::quote(reason)+",\"final_native_overlay\":"+overlayJson(final_overlay)+
    ",\"candidate\":"+candidateJson(copied);});
}
template<class Data,class Map>
void acceptance(const Data& data,const plan_env::DynamicPointSnapshot& snapshot,const ros::Time& now,
                Map& map,double resume_scale,double step){
  if(!enabled())return;
  const auto copied=data;const auto plan=currentPlan();
  const auto overlay=TemporalCollisionCaptureMapAccessor::capture(map);
  enqueue("acceptance",[copied,snapshot,now,plan,overlay,resume_scale,step](){
    return std::string("\"plan_ordinal\":")+std::to_string(plan?plan->ordinal:0)+
      ",\"validation_now_nsec\":"+std::to_string(now.toNSec())+
      ",\"resume_scale\":"+delivery_diag::number(resume_scale)+",\"check_step_s\":"+delivery_diag::number(step)+
      ",\"latest_snapshot\":"+delivery_replay::nativeDynamicText(snapshot)+
      ",\"acceptance_overlay\":"+overlayJson(overlay)+",\"candidate\":"+candidateJson(copied)+
      ",\"time_mapping\":\"branch-dependent; candidate_phase_alignment event is authoritative; COVERAGE_ALIGNMENT=1 uses measured projected phase, actual pose independently checked; legacy retains original restart-zero mapping\"";
  });
}
} // namespace temporal_capture
