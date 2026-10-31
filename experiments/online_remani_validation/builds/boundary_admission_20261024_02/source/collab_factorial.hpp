#pragma once
// Isolated SIL experiment only. A prediction is an INITIALIZATION SUGGESTION,
// never a committed future state or a replacement execution safety certificate.
#include "dual_timescale_probe.hpp"
#include "failure_probe.hpp"
#include <std_msgs/Float64MultiArray.h>
#include <array>
#include <fstream>
#include <memory>
#include <numeric>

namespace collab {
inline int envInt(const char* name, int fallback=0) {
  const char* value=std::getenv(name);return value?std::stoi(value):fallback;
}
struct Settings { bool rolling=envInt("COLLAB_ROLLING"), prefix=envInt("COLLAB_PREFIX"); };
inline Settings& settings(){static Settings value;return value;}
inline bool& refresh(){static bool value=false;return value;}
inline unsigned long long& goalToken(){static unsigned long long value=0;return value;}
struct Seed {
  double stamp=0, revision=0, generation=0, goal=0, odom=0, cloud=0, dynamic=0;
  std::vector<std::array<double,8>> rows;
};
inline Seed& seed(){static Seed value;return value;}
struct Field {
  double stamp=0,cloud=0,dynamic=0,gx=0,gy=0,res=0,margin=0;
  int nx=0,ny=0;std::vector<std::array<double,3>> body;
  std::vector<std::array<double,6>> obstacles;std::vector<double> distances;
  bool fresh(double now)const{return now-stamp>=-.05&&now-stamp<=.35&&now-cloud<=.35&&now-dynamic<=.35;}
  double distance(double x,double y)const{
    double fx=(x-gx)/res,fy=(y-gy)/res;int ix=std::floor(fx),iy=std::floor(fy);
    if(ix<0||iy<0||ix>=nx-1||iy>=ny-1)return 0.;
    double a=fx-ix,b=fy-iy;
    return (1-b)*((1-a)*distances[iy*nx+ix]+a*distances[iy*nx+ix+1])+
      b*((1-a)*distances[(iy+1)*nx+ix]+a*distances[(iy+1)*nx+ix+1]);
  }
  double gap(double x,double y,double yaw,double now)const{
    if(!fresh(now))return -std::numeric_limits<double>::infinity();
    double answer=std::numeric_limits<double>::infinity(),c=std::cos(yaw),s=std::sin(yaw);
    for(const auto& disk:body){double px=x+c*disk[0]-s*disk[1],py=y+s*disk[0]+c*disk[1];
      answer=std::min(answer,distance(px,py)-disk[2]-margin);
      for(const auto& obstacle:obstacles){double age=std::max(0.,now-dynamic),moving=obstacle[5]>.5?1.:0.;
        double ox=obstacle[0]+moving*obstacle[2]*age,oy=obstacle[1]+moving*obstacle[3]*age;
        answer=std::min(answer,std::hypot(px-ox,py-oy)-disk[2]-obstacle[4]-margin);}
    }return answer;
  }
};
inline std::shared_ptr<const Field>& field(){static std::shared_ptr<const Field> value;return value;}
inline bool parseField(const std::vector<double>& v,std::shared_ptr<const Field>& output){
  output.reset();if(v.size()<12||v[0]!=20261022||v[6]<=0||v[7]<2||v[8]<2||
    v[7]>1000||v[8]>1000||v[10]<1||v[10]>1000||v[11]<0||v[11]>1000)return false;
  for(int i:{7,8,10,11})if(!std::isfinite(v[i])||v[i]!=std::floor(v[i]))return false;
  for(double x:v)if(!std::isfinite(x))return false;
  int nb=v[10],no=v[11],nx=v[7],ny=v[8];
  if(v.size()!=12+3*nb+6*no+std::size_t(nx)*ny)return false;
  auto f=std::make_shared<Field>();f->stamp=v[1];f->cloud=v[2];f->dynamic=v[3];
  f->gx=v[4];f->gy=v[5];f->res=v[6];f->nx=nx;f->ny=ny;f->margin=v[9];
  std::size_t index=12;
  for(int i=0;i<nb;++i){std::array<double,3> row;std::copy_n(v.begin()+index,3,row.begin());
    if(row[2]<=0)return false;f->body.push_back(row);index+=3;}
  for(int i=0;i<no;++i){std::array<double,6> row;std::copy_n(v.begin()+index,6,row.begin());f->obstacles.push_back(row);index+=6;}
  f->distances.assign(v.begin()+index,v.end());output=f;return true;
}
inline bool parse(const std::vector<double>& values,Seed& output) {
  output=Seed();
  if(values.size()<10||values[0]!=1||values[8]!=8||values[9]<2||values[9]>1000||
     values[9]!=std::floor(values[9])||values.size()!=10+8*std::size_t(values[9]))return false;
  for(double x:values)if(!std::isfinite(x))return false;
  output.stamp=values[1];output.generation=values[2];output.revision=values[3];output.goal=values[4];
  output.odom=values[5];output.cloud=values[6];output.dynamic=values[7];
  double previous=-1;
  for(std::size_t i=10;i<values.size();i+=8){
    std::array<double,8> row;std::copy_n(values.begin()+i,8,row.begin());
    if(row[0]<0||row[0]<=previous){output=Seed();return false;}
    previous=row[0];output.rows.push_back(row);
  }
  return true;
}
inline ros::Subscriber& subscriber(){static ros::Subscriber value;return value;}
inline ros::Subscriber& fieldSubscriber(){static ros::Subscriber value;return value;}
inline void seedMessage(const std_msgs::Float64MultiArray::ConstPtr& message){parse(message->data,seed());}
inline void fieldMessage(const std_msgs::Float64MultiArray::ConstPtr& message){parseField(message->data,field());}
inline void configure(ros::NodeHandle& nh) {
  subscriber()=nh.subscribe("/sim/mppi_seed_states",1,seedMessage);
  fieldSubscriber()=nh.subscribe("/sim/mppi_target_field",1,fieldMessage);
}
inline void note(const std::string& event,const dual_probe::Fields& fields=dual_probe::Fields()) {
  dual_probe::emit("collab_"+event,fields);
}
inline double angle(double a){return std::atan2(std::sin(a),std::cos(a));}
inline bool goodSearch(int status){return status!=3&&status!=5&&status!=6;}

// Rolling goal is sampled on the existing GLOBAL reference, not on obstacle
// truth and not on a fabricated free-space line. Final navigation goal survives.
template<class Manager>
bool rollingTarget(Manager& m,const Eigen::VectorXd& start,double yaw,
                   const Eigen::VectorXd& final,double final_yaw,
                   Eigen::VectorXd& target,Eigen::VectorXd& velocity,
                   Eigen::VectorXd& acceleration,bool& horizon) {
  dual_probe::Scope timer("rolling_target");
  auto& global=m.traj_container_.global_traj;
  double best_score=std::numeric_limits<double>::infinity(),projection=0;
  for(double t=0;t<=global.duration+1e-6;t+=.05){
    double u=std::min(t,global.duration);auto p=global.traj.getPos(u);auto v=global.traj.getVel(u);
    double e=v.head(2).norm()>1e-3?angle(std::atan2(v(1),v(0))-yaw):0;
    double score=(p.head(2)-start.head(2)).squaredNorm()+.02*(1-std::cos(e));
    if(score<best_score){best_score=score;projection=u;}
  }
  int final_kind=-1;
  const bool final_occupied=m.mm_config_->checkPlanningCollision(
    Eigen::Vector3d(final(0),final(1),final_yaw),final.tail(m.pp_.manipulator_dim_),false,final_kind);
  const auto collision=field();const double query_time=ros::Time::now().toSec();
  if(!collision||!collision->fresh(query_time)){
    note("target",dual_probe::Fields().add("rolling","true").add("success","false").str("reason","projected_snapshot_unavailable"));return false;
  }
  const double final_projected_gap=collision->gap(final(0),final(1),final_yaw,query_time);
  target=start;velocity=Eigen::VectorXd::Zero(start.size());acceleration=velocity;horizon=true;
  double selected=-1,selected_yaw=yaw,selected_distance=0;
  if((final.head(2)-start.head(2)).norm()<=1.20&&!final_occupied&&final_projected_gap>=0){
    selected=global.duration;target=final;selected_yaw=final_yaw;horizon=false;
  }else{
    // Prefer about one metre of route progress; never search arbitrarily far
    // past an occupied subgoal. 1.6m is a declared bounded fallback, not 1000m.
    double best_preference=std::numeric_limits<double>::infinity();
    for(double t=projection+.05;t<global.duration-1e-6;t+=.05){
      auto p=global.traj.getPos(t);auto v=global.traj.getVel(t);
      double distance=(p.head(2)-start.head(2)).norm();
      if(distance<.55||distance>1.60)continue;
      double candidate_yaw=v.head(2).norm()>1e-3?std::atan2(v(1),v(0)):final_yaw;
      int kind=-1;
      if(m.mm_config_->checkPlanningCollision(Eigen::Vector3d(p(0),p(1),candidate_yaw),
          p.tail(m.pp_.manipulator_dim_),false,kind))continue;
      if(collision->gap(p(0),p(1),candidate_yaw,query_time)<0)continue;
      double preference=std::abs(distance-1.0);
      if(preference<best_preference){best_preference=preference;selected=t;
        target=p;selected_yaw=candidate_yaw;selected_distance=distance;}
    }
  }
  if(selected>=0){
    velocity.head(2)=m.pp_.mobile_base_non_singul_vel_*Eigen::Vector2d(std::cos(selected_yaw),std::sin(selected_yaw));
    global.last_glb_t_of_lc_tgt=global.glb_t_of_lc_tgt;
    global.glb_t_of_lc_tgt=global.global_start_time+selected;
  }
  note("target",dual_probe::Fields().add("rolling","true").add("success",selected>=0?"true":"false")
    .add("final_occupied",final_occupied?"true":"false").add("final_collision_type",final_kind)
    .add("final_projected_gap",final_projected_gap).add("field_age",query_time-collision->stamp)
    .add("projection_t",projection).add("selected_t",selected).add("selected_distance",selected_distance)
    .add("target_x",target(0)).add("target_y",target(1)));
  timer.success=selected>=0;return selected>=0;
}

// A local prefix must have current observations, the same task, one gear and
// useful displacement. Reanchor ONLY its suggested geometry to measured start;
// no guessed future pose is installed as the slow planner's starting state.
inline bool prefixRows(const Eigen::VectorXd& start,double yaw,int direction,
                       std::vector<std::array<double,8>>& rows,std::string& reason) {
  const auto& s=seed();double now=ros::Time::now().toSec(),age=now-s.stamp;
  reason="stale_or_missing";
  if(s.rows.size()<3||age<-.05||age>.25||now-s.cloud>.35||now-s.dynamic>.35||
     now-s.odom>.25||s.goal!=double(goalToken()))return false;
  std::size_t begin=0;
  while(begin+1<s.rows.size()&&s.rows[begin][0]<std::max(0.,age))++begin;
  const auto& first=s.rows[begin];
  if(std::hypot(first[1]-start(0),first[2]-start(1))>.08||std::abs(angle(first[3]-yaw))>.25){
    reason="start_mismatch";return false;
  }
  const int sign=direction==0?(first[4]<0?-1:1):direction;
  rows.clear();double previous=-1;
  for(std::size_t i=begin;i<s.rows.size();++i){
    auto row=s.rows[i];row[0]-=first[0];
    if(row[0]>1.0+1e-6)break;
    if(std::abs(row[4])<.015||row[4]*sign<=0){reason="gear_or_low_speed";break;}
    if(row[0]-previous<.18&&i!=begin)continue;
    rows.push_back(row);previous=row[0];
  }
  if(rows.size()<3||rows.back()[0]<.4||
     std::hypot(rows.back()[1]-start(0),rows.back()[2]-start(1))<.06){reason="short_prefix";return false;}
  reason="eligible";return true;
}

template<class Manager>
int search(Manager& m,const Eigen::MatrixXd& head,const Eigen::MatrixXd& tail,
           double yaw,int sign,bool gripper,double end_yaw,bool end_gripper,int failures,
           std::vector<std::vector<Eigen::VectorXd>>& paths,std::vector<std::vector<double>>& yaws,
           std::vector<poly_traj::MinSnapOpt<8>>& trajectories,std::vector<int>& directions) {
  dual_probe::Scope timer("search");
  int status=m.ploy_traj_opt_->astarWithMinTraj(head,tail,yaw,sign,gripper,end_yaw,end_gripper,
    Eigen::Vector2d::Zero(),failures,paths,yaws,trajectories,directions);
  timer.fields.add("status",status);timer.success=goodSearch(status);return status;
}

// Same-gear prefix and first native continuation are refit into ONE MinSnap
// block. Adding an artificial block would make the native optimizer treat the
// join as a gear transition, silently changing the problem.
template<class Manager>
bool prefixSearch(Manager& m,const Eigen::MatrixXd& head,const Eigen::MatrixXd& tail,
                  double yaw,int sign,bool gripper,double end_yaw,bool end_gripper,int failures,
                  std::vector<std::vector<Eigen::VectorXd>>& paths,std::vector<std::vector<double>>& yaws,
                  std::vector<poly_traj::MinSnapOpt<8>>& trajectories,std::vector<int>& directions) {
  dual_probe::Scope timer("prefix_conversion_and_search");
  std::vector<std::array<double,8>> rows;std::string reason;
  auto reject=[&](const std::string& why){note("prefix",dual_probe::Fields().str("reason",why).add("used","false"));return false;};
  if(!prefixRows(head.col(0),yaw,sign,rows,reason))return reject(reason);
  const std::size_t raw_samples=rows.size();
  // A 1s suggestion does not need five extra polynomial pieces. Preserve one
  // interior bend and the endpoint, then let the original optimizer refine it.
  rows={rows.front(),rows[rows.size()/2],rows.back()};
  const auto& last=rows.back();const auto& before=rows[rows.size()-2];
  const int gear=sign==0?(last[4]<0?-1:1):sign;
  Eigen::MatrixXd middle=head;middle.col(0)(0)=last[1];middle.col(0)(1)=last[2];
  middle.col(1).setZero();middle.col(2).setZero();middle.col(3).setZero();
  Eigen::Vector2d h(std::cos(last[3]),std::sin(last[3])),n(-std::sin(last[3]),std::cos(last[3]));
  middle.col(1).head(2)=last[4]*h;
  middle.col(2).head(2)=(last[4]-before[4])/(last[0]-before[0])*h+last[4]*last[5]*n;
  const int np=rows.size()-1;
  Eigen::MatrixXd prefix_points(head.rows(),np-1);Eigen::VectorXd prefix_times(np);
  for(int i=0;i<np;++i){prefix_times(i)=rows[i+1][0]-rows[i][0];
    if(i<np-1){prefix_points.col(i)=head.col(0);prefix_points(0,i)=rows[i+1][1];prefix_points(1,i)=rows[i+1][2];}}
  poly_traj::MinSnapOpt<8> prefix;prefix.reset(head,middle,np);prefix.generate(prefix_points,prefix_times);
  auto prefix_traj=prefix.getTraj(gear);
  for(double t=0;t<=last[0]+1e-8;t+=.02){
    double u=std::min(t,last[0]);auto p=prefix_traj.getPos(u);auto v=prefix_traj.getVel(u);
    if(!p.allFinite()||!v.allFinite()||v.head(2).norm()<1e-5)return reject("prefix_fit_singularity");
    double a=std::atan2(gear*v(1),gear*v(0));int kind=-1;
    if(m.mm_config_->checkPlanningCollision(Eigen::Vector3d(p(0),p(1),a),p.tail(m.pp_.manipulator_dim_),false,kind))
      return reject("prefix_fit_collision");
  }
  int status=search(m,middle,tail,last[3],gear,gripper,end_yaw,end_gripper,failures,paths,yaws,trajectories,directions);
  if(!goodSearch(status)||trajectories.empty()||directions.front()!=gear)return reject("continuation_failed_or_gear_changed");
  auto first=trajectories.front().getTraj(gear);const int nt=first.getPieceNum();
  auto native_points=first.getPositions();auto native_times=first.getDurations();
  Eigen::MatrixXd end(head.rows(),4);end<<first.getJuncPos(nt),first.getJuncVel(nt),first.getJuncAcc(nt),first.getJuncJerk(nt);
  Eigen::MatrixXd merged_points(head.rows(),np+nt-1);Eigen::VectorXd merged_times(np+nt);
  for(int i=0;i<np;++i){merged_points.col(i)=head.col(0);merged_points(0,i)=rows[i+1][1];merged_points(1,i)=rows[i+1][2];merged_times(i)=prefix_times(i);}
  for(int i=1;i<nt;++i)merged_points.col(np+i-1)=native_points.col(i);
  merged_times.tail(nt)=native_times;
  trajectories.front().reset(head,end,np+nt);trajectories.front().generate(merged_points,merged_times);
  // The merged initializer is only a proposal. Native optimization and its
  // ORIGINAL full-trajectory wheel/collision acceptance remain mandatory.
  note("prefix",dual_probe::Fields().str("reason","used").add("used","true").add("prefix_s",last[0])
    .add("prefix_points",rows.size()).add("raw_prefix_samples",raw_samples)
    .add("merged_first_pieces",np+nt).add("native_continuation_pieces",nt)
    .add("seed_revision",seed().revision).add("seed_age",ros::Time::now().toSec()-seed().stamp)
    .add("anchor_x",last[1]).add("anchor_y",last[2]));
  timer.success=true;return true;
}

// Diagnostic-only paired requests. Pause only THIS runner's numerical plant
// and MPPI; never attach to or pause a vehicle. No reference is published from
// these calls. The surrounding run is excluded from online navigation scores.
template<class Manager>
void pairedOnce(Manager& m,const Eigen::VectorXd& start,const Eigen::VectorXd& velocity,
                const Eigen::VectorXd& acceleration,const Eigen::VectorXd& jerk,
                double yaw,int sign,bool gripper,const Eigen::VectorXd& final,
                double final_yaw,bool final_gripper) {
  static bool done=false;
  if(done||!refresh()||!envInt("COLLAB_PAIR"))return;
  done=true;failure_probe::PauseOwned paused;
  const double now=ros::Time::now().toSec();
  const auto raw=m.grid_map_->getDynamicObservationSeq(),esdf=m.grid_map_->getESDFObservationSeq();
  const auto global=m.traj_container_.global_traj;
  const auto local=m.traj_container_.singul_traj_data;
  const int failures=m.experimentFailureCount();
  const Settings original=settings();const Seed original_seed=seed();
  unsigned long long common=1469598103934665603ULL;
  for(const auto* v:{&start,&velocity,&acceleration,&jerk,&final})
    resource_probe::hashBytes(common,v->data(),v->size()*sizeof(double));
  note("pair_begin",dual_probe::Fields().str("task_hash",std::to_string(common)).add("raw_seq",raw)
    .add("esdf_seq",esdf).add("seed_stamp",seed().stamp).add("frozen_sim",now));
  const int order[]={0,1,2,3,3,2,1,0};
  for(int pass=0;pass<8;++pass){
    int group=order[pass];settings().rolling=group>=2;settings().prefix=group%2;
    m.traj_container_.global_traj=global;m.traj_container_.singul_traj_data=local;
    m.experimentSetFailureCount(failures);seed()=original_seed;
    const std::string label="paired_"+std::string(1,char('A'+group))+"_"+std::to_string(pass);
    resource_probe::label()=label;
    dual_probe::Scope scope("paired_request",dual_probe::Fields().str("group",std::string(1,char('A'+group))));
    Eigen::VectorXd target,target_v,target_a;bool horizon=true;
    double began=dual_probe::mono(),init_ms=0,opt_ms=0;
    bool target_ok=m.getLocalTarget(1000.,start,yaw,final,final_yaw,target,target_v,target_a,horizon,false);
    bool valid=false;unsigned long long target_hash=0;
    if(target_ok){
      double target_yaw=std::atan2(target_v(1),target_v(0));
      target_v.setZero();target_v.head(2)=m.pp_.mobile_base_non_singul_vel_*Eigen::Vector2d(std::cos(target_yaw),std::sin(target_yaw));
      target_a.setZero();target_hash=1469598103934665603ULL;
      for(const auto* v:{&target,&target_v,&target_a})resource_probe::hashBytes(target_hash,v->data(),v->size()*sizeof(double));
      valid=m.reboundReplan(start,velocity,acceleration,jerk,yaw,sign,gripper,now,
        target,target_v,target_a,target_yaw,horizon?gripper:final_gripper,true,false,true,init_ms,opt_ms);
    }
    const bool stable=std::abs(ros::Time::now().toSec()-now)<1e-9&&
      raw==m.grid_map_->getDynamicObservationSeq()&&esdf==m.grid_map_->getESDFObservationSeq();
    note("pair_result",dual_probe::Fields().str("group",std::string(1,char('A'+group))).str("label",label)
      .str("task_hash",std::to_string(common)).str("target_hash",std::to_string(target_hash))
      .add("target_ok",target_ok?"true":"false").add("valid",valid?"true":"false")
      .add("wall_ms",(dual_probe::mono()-began)*1000).add("stable",stable?"true":"false")
      .add("raw_seq",m.grid_map_->getDynamicObservationSeq()).add("esdf_seq",m.grid_map_->getESDFObservationSeq()));
    scope.success=valid;
  }
  settings()=original;seed()=original_seed;resource_probe::label()="normal";
  m.traj_container_.global_traj=global;m.traj_container_.singul_traj_data=local;m.experimentSetFailureCount(failures);
  const char* path=std::getenv("COLLAB_PAIR_DONE");
  if(path){std::ofstream f(path);f<<"{\"done\":true,\"task_hash\":\""<<common<<"\"}\n";}
  note("pair_end");
}
} // namespace collab
