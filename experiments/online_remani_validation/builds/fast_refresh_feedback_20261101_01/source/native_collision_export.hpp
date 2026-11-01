#pragma once
// Experimental read-only native collision export. No duplicate GridMap/model.
// The legacy queue alone reads ESDF/model/odometry. The private queue only reads
// an immutable cache and the explicitly thread-safe DynamicPointSnapshot API.
#include <mm_config/mm_config.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/UInt8MultiArray.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

namespace native_collision_export {
inline bool switchOn(const char* name) {
  const char* v=std::getenv(name);return v && v[0]=='1' && v[1]=='\0';
}
inline bool enabled(){return switchOn("NATIVE_COLLISION_3D");}
inline bool exportEnabled(){return switchOn("NATIVE_COLLISION_EXPORT") || switchOn("NATIVE_COLLISION_3D_EXPORT");}
// v1 wire: little-endian IEEE doubles; header32, body7, dynamic7, optional ESDF.
// Body: x,y,z,hard_radius,soft_radius,weight,type(base0/arm1), in base frame.
// Base hard radius includes native map_resolution; physical proxy radius is
// hard_radius-header[11]. Arm hard radius is its physical sphere thickness.
// ESDF: native x-major ((ix*ny)+iy)*nz+iz, origin at cropped voxel EDGE.
struct Cache {
  std::array<double,32> header{};
  std::vector<double> body;
  std::shared_ptr<const std::vector<double>> esdf;
  std::uint64_t generation=0;
};
class Exporter {
  ros::NodeHandle nh_;
  std::shared_ptr<GridMap> map_;
  remani_planner::MMConfig::Ptr model_;
  Eigen::VectorXd fixed_q_;
  double arm_radius_=0.,car_margin_=0.,arm_margin_=0.,weight_=0.,ground_=0.;
  double max_age_=.35,max_prediction_=2.,pos_unc_=0.,vel_unc_=0.,acc_unc_=0.;
  bool dynamic_enabled_=false,q_matches_=false,have_odom_=false;
  Eigen::Vector2d position_=Eigen::Vector2d::Zero();
  double joint_stamp_=0.;
  Eigen::Vector3i crop_lo_=Eigen::Vector3i::Constant(-1),crop_hi_=crop_lo_;
  std::uint64_t esdf_seq_=std::numeric_limits<std::uint64_t>::max(),generation_=0,sequence_=0;
  std::uint64_t sent_generation_=0;
  int last_subscribers_=-1;
  bool last_health_=false;
  double last_full_stamp_=0.;
  bool parity_sent_=false;
  std::shared_ptr<const Cache> cache_;
  // Destruction order matters: spinner must stop before queue/cache/map die.
  ros::CallbackQueue output_queue_;
  ros::Publisher publisher_,parity_publisher_;
  ros::Subscriber odom_sub_,joint_sub_;
  ros::Timer sample_timer_,output_timer_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;

  void odom(const nav_msgs::Odometry::ConstPtr& msg){
    position_=Eigen::Vector2d(msg->pose.pose.position.x,msg->pose.pose.position.y);
    have_odom_=position_.allFinite();
  }
  void joints(const sensor_msgs::JointState::ConstPtr& msg){
    q_matches_=msg->position.size()==std::size_t(fixed_q_.size());
    for(Eigen::Index i=0;q_matches_ && i<fixed_q_.size();++i)
      q_matches_=std::isfinite(msg->position[i]) && std::abs(msg->position[i]-fixed_q_[i])<=1e-5;
    joint_stamp_=msg->header.stamp.toSec();
  }
  void sample(const ros::TimerEvent&){
    auto next=std::make_shared<Cache>();
    const auto previous=std::atomic_load(&cache_);
    auto& h=next->header;
    h[0]=3.;h[1]=1.;h[11]=map_->getResolution();h[15]=max_age_;h[16]=max_prediction_;
    h[21]=ground_;h[24]=joint_stamp_;h[25]=pos_unc_;h[26]=vel_unc_;h[27]=acc_unc_;
    h[28]=dynamic_enabled_?1.:0.;
    std::vector<Eigen::Vector3d> car;
    model_->getCarPts(Eigen::Vector3d::Zero(),car);
    for(std::size_t i=0;i<car.size();++i){
      const auto& p=car[i];
      const double radius=model_->getBaseSphereRadius(i);
      const double hard=radius+model_->getMapResolution();
      next->body.insert(next->body.end(),{p.x(),p.y(),p.z(),hard,radius+car_margin_,weight_,0.});
    }
    std::vector<Eigen::Matrix4d> transforms,unused;
    model_->getJointTrans(fixed_q_,transforms,unused);
    const auto points=model_->getLinkPoint();
    Eigen::Matrix4d transform=model_->getTq0();
    bool ground_ok=true;
    for(int i=0;i<model_->getManiDof();++i){
      transform*=transforms[i];
      for(Eigen::Index j=0;j<points[i].cols();++j){
        const Eigen::Vector3d p=(transform*points[i].col(j)).head<3>();
        ground_ok=ground_ok && p.allFinite() && p.z()>=ground_;
        next->body.insert(next->body.end(),{p.x(),p.y(),p.z(),arm_radius_,arm_radius_+arm_margin_,weight_/5.,1.});
      }
    }
    double unused_distance=0.;
    const bool self_ok=!model_->checkCarManiCollision(fixed_q_,false,unused_distance) &&
                       !model_->checkManiManiCollision(fixed_q_,false,unused_distance);
    h[12]=next->body.size()/7;h[22]=ground_ok?1.:0.;h[23]=self_ok?1.:0.;
    Eigen::Vector3i dimensions;map_->getVoxelNum(dimensions);
    const auto origin=map_->getOrigin();const double res=h[11];
    Eigen::Vector3d lower(position_.x()-2.5,position_.y()-2.5,std::numeric_limits<double>::infinity());
    Eigen::Vector3d upper(position_.x()+2.5,position_.y()+2.5,-std::numeric_limits<double>::infinity());
    for(std::size_t i=0;i<next->body.size();i+=7){
      lower.z()=std::min(lower.z(),next->body[i+2]-next->body[i+3]-2*res);
      upper.z()=std::max(upper.z(),next->body[i+2]+next->body[i+3]+2*res);
    }
    Eigen::Vector3i lo,hi;
    for(int a=0;a<3;++a){
      lo[a]=std::max(0,std::min(dimensions[a]-1,int(std::floor((lower[a]-origin[a])/res))));
      hi[a]=std::max(0,std::min(dimensions[a]-1,int(std::ceil((upper[a]-origin[a])/res))));
    }
    const auto seq=map_->getESDFObservationSeq();
    const bool refresh=!previous || seq!=esdf_seq_ || lo!=crop_lo_ || hi!=crop_hi_;
    if(refresh){
      auto field=std::make_shared<std::vector<double>>();
      const Eigen::Vector3i shape=hi-lo+Eigen::Vector3i::Ones();
      field->reserve(std::size_t(shape.x())*shape.y()*shape.z());
      for(int x=lo.x();x<=hi.x();++x)for(int y=lo.y();y<=hi.y();++y)for(int z=lo.z();z<=hi.z();++z)
        field->push_back(map_->getDistance(Eigen::Vector3i(x,y,z)));
      // This is acquisition time, NOT a fabricated original observation time.
      h[17]=ros::Time::now().toSec();next->esdf=field;
      esdf_seq_=seq;crop_lo_=lo;crop_hi_=hi;++generation_;
    }else{next->esdf=previous->esdf;h[17]=previous->header[17];}
    for(int a=0;a<3;++a){h[5+a]=hi[a]-lo[a]+1;h[8+a]=origin[a]+lo[a]*res;}
    h[14]=double(seq);next->generation=generation_;
    const bool finite=std::all_of(next->body.begin(),next->body.end(),[](double v){return std::isfinite(v);}) &&
      std::all_of(next->esdf->begin(),next->esdf->end(),[](double v){return std::isfinite(v);});
    h[4]=have_odom_ && q_matches_ && map_->mapReadyForPlanning() && finite && ground_ok && self_ok?1.:0.;
    std::shared_ptr<const Cache> immutable=next;std::atomic_store(&cache_,immutable);
    if(switchOn("NATIVE_COLLISION_PARITY") && !parity_sent_ && h[4]==1.){
      // Separate preflight only: native static ESDF, same legacy callback and
      // same immutable acquisition. Never run in formal timing trials.
      std::vector<double> parity{4.,1.,h[14],h[17],double(next->generation),28.};
      const std::array<Eigen::Vector2d,7> offsets{{{0.,0.},{.3,0.},{-.3,0.},
        {0.,.3},{0.,-.3},{.8,0.},{-.8,0.}}};
      for(const auto& delta:offsets)for(double yaw:{0.,M_PI/2.,M_PI,-M_PI/2.}){
        const Eigen::Vector3d pose(position_.x()+delta.x(),position_.y()+delta.y(),yaw);
        int type=-1;const bool collision=model_->checkcollision(pose,fixed_q_,false,type);
        parity.insert(parity.end(),{pose.x(),pose.y(),pose.z(),collision?1.:0.,double(type)});
      }
      std_msgs::UInt8MultiArray message;message.data.resize(parity.size()*sizeof(double));
      std::memcpy(message.data.data(),parity.data(),message.data.size());parity_publisher_.publish(message);
      parity_sent_=true;
    }
  }
  void output(const ros::TimerEvent&){
    const auto cached=std::atomic_load(&cache_);if(!cached)return;
    const ros::Time now=ros::Time::now();
    // The only GridMap call on this queue: documented thread-safe point holder.
    const auto dynamic=map_->getDynamicPointSnapshot(now);
    auto h=cached->header;h[2]=double(++sequence_);h[3]=now.toSec();
    h[18]=dynamic.stamp.toSec();h[19]=(!dynamic_enabled_ || dynamic.valid)?1.:0.;
    h[13]=dynamic_enabled_ && dynamic.valid?dynamic.points.size():0.;
    h[4]=h[4]==1. && h[19]==1.?1.:0.;
    const int subscribers=publisher_.getNumSubscribers();
    // A small periodic resynchronization also recovers a subscriber that lost
    // its first full packet while inputs were not yet healthy. Re-sending the
    // same immutable field never rewrites its acquisition timestamp/sequence.
    const bool health=h[4]==1.;
    const bool full=cached->generation!=sent_generation_ || subscribers!=last_subscribers_ ||
                    (health && !last_health_) || now.toSec()-last_full_stamp_>=1.;
    h[20]=full?1.:0.;
    std::vector<double> wire(h.begin(),h.end());
    wire.insert(wire.end(),cached->body.begin(),cached->body.end());
    if(dynamic_enabled_ && dynamic.valid)for(const auto& p:dynamic.points)
      wire.insert(wire.end(),{p.position.x(),p.position.y(),p.position.z(),p.velocity.x(),p.velocity.y(),p.velocity.z(),double(p.track_id)});
    if(full)wire.insert(wire.end(),cached->esdf->begin(),cached->esdf->end());
    std_msgs::UInt8MultiArray message;message.data.resize(wire.size()*sizeof(double));
    std::memcpy(message.data.data(),wire.data(),message.data.size());publisher_.publish(message);
    sent_generation_=cached->generation;last_subscribers_=subscribers;last_health_=health;
    if(full)last_full_stamp_=now.toSec();
  }
public:
  Exporter(ros::NodeHandle nh,const std::shared_ptr<GridMap>& map,
           const remani_planner::MMConfig::Ptr& model):nh_(nh),map_(map),model_(model){
    bool sil=false,sim=false;ros::param::param("/online_validation/enabled",sil,false);
    ros::param::param("/use_sim_time",sim,false);
    if(!sil || !sim)throw std::runtime_error("native collision export requires isolated SIL");
    std::vector<double> lo,hi;nh_.getParam("mm/manipulator_min_pos",lo);nh_.getParam("mm/manipulator_max_pos",hi);
    if(lo.empty() || lo!=hi || lo.size()!=std::size_t(model_->getManiDof()) ||
       !std::all_of(lo.begin(),lo.end(),[](double q){return std::isfinite(q);}))
      throw std::runtime_error("native collision export requires exactly fixed finite arm limits");
    fixed_q_=Eigen::Map<const Eigen::VectorXd>(lo.data(),lo.size());
    nh_.param("mm/manipulator_thickness",arm_radius_,.034);
    nh_.param("optimization/safe_margin",car_margin_,.1);nh_.param("optimization/safe_margin_mani",arm_margin_,.1);
    nh_.param("optimization/weight_obstacle",weight_,-1.);nh_.param("optimization/ground_safe_dis",ground_,.03);
    nh_.param("dynamic_point_flow/enable",dynamic_enabled_,false);nh_.param("dynamic_point_flow/max_age",max_age_,.35);
    nh_.param("dynamic_point_flow/max_prediction_time",max_prediction_,2.);
    nh_.param("dynamic_point_flow/position_uncertainty",pos_unc_,.03);
    nh_.param("dynamic_point_flow/velocity_uncertainty",vel_unc_,.15);
    nh_.param("dynamic_point_flow/acceleration_bound",acc_unc_,.5);
    if(!(arm_radius_>0.) || !(weight_>=0.) || !(map_->getResolution()>0.))
      throw std::runtime_error("invalid native collision geometry/objective parameters");
    std::uint16_t endian=1;
    if(*reinterpret_cast<unsigned char*>(&endian)!=1 || sizeof(double)!=8)
      throw std::runtime_error("native collision wire requires little-endian 64-bit doubles");
    publisher_=nh_.advertise<std_msgs::UInt8MultiArray>("/sim/native_collision_3d",1,false);
    if(switchOn("NATIVE_COLLISION_PARITY"))
      parity_publisher_=nh_.advertise<std_msgs::UInt8MultiArray>("/sim/native_collision_parity",1,true);
    odom_sub_=nh_.subscribe("odom_world",1,&Exporter::odom,this);
    joint_sub_=nh_.subscribe("joint_state",1,&Exporter::joints,this);
    sample_timer_=nh_.createTimer(ros::Duration(.05),&Exporter::sample,this);
    ros::NodeHandle output_nh(nh_);output_nh.setCallbackQueue(&output_queue_);
    output_timer_=output_nh.createTimer(ros::Duration(.05),&Exporter::output,this);
    spinner_.reset(new ros::AsyncSpinner(1,&output_queue_));spinner_->start();
    ROS_INFO("[NativeCollision3D] read-only exporter initialized; native self-collision policy retained (Piper self-check remains disabled)");
  }
  ~Exporter(){if(spinner_)spinner_->stop();output_timer_.stop();sample_timer_.stop();}
};
inline std::shared_ptr<Exporter>& instance(){static std::shared_ptr<Exporter> value;return value;}
inline void initialize(ros::NodeHandle& nh,const std::shared_ptr<GridMap>& map,
                       const remani_planner::MMConfig::Ptr& model){
  if(exportEnabled() && !instance())instance()=std::make_shared<Exporter>(nh,map,model);
}
} // namespace native_collision_export
