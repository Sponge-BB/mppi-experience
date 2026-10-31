#pragma once
// Fixed-arm SIL experiment only. Include after the native optimizer declaration
// has been exposed to the diagnostic main. This never changes the native opt.
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <vector>
#include "delivery_accepted_candidate.hpp"

namespace cheap_variable_scaling {
using Opt = remani_planner::PolyTrajOptimizer;
struct Value { double cost=0.; Eigen::VectorXd gradient; };
struct Result {
  Eigen::VectorXd scale;
  std::vector<int> free_indices;
  int evaluations=0;
  double elapsed_s=0.;
};

class Context {
  Opt* native_;
  std::vector<poly_traj::MinSnapOpt<8>> snaps_;
  std::vector<int> pieces_,signs_,free_;
  int point_end_=0,time_end_=0,angle_begin_=0,nx_=0;

  // Native primary and low-speed-triggered dense feasibility quadrature.
  // Wheel formula, penalty smoothing and all native weights are delegated to
  // the actual optimizer; only environment/arm costs are excluded from proxy.
  double integral(int id,Eigen::VectorXd& gdT) {
    const int K=native_->cps_num_prePiece_,D=native_->dense_sample_resolution_;
    auto& snap=snaps_[id];double total=0.;
    Eigen::Matrix<double,8,1> b1,b2,b3,b4,d1,d2,d3,d4;
    for(int i=0;i<gdT.size();++i){
      const Eigen::Matrix<double,8,8> c=snap.get_b().block(i*8,0,8,8);
      const double step=snap.get_T1()[i]/K;double s1=0.;
      for(int j=0;j<=K;++j){
        double s2=s1*s1,s3=s2*s1,s4=s2*s2,s5=s4*s1,s6=s3*s3;
        b1<<0.,1.,2.*s1,3.*s2,4.*s3,5.*s4,6.*s5,7.*s6;
        b2<<0.,0.,2.,6.*s1,12.*s2,20.*s3,30.*s4,42.*s5;
        b3<<0.,0.,0.,6.,24.*s1,60.*s2,120.*s3,210.*s4;
        b4<<0.,0.,0.,0.,24.,120.*s1,360.*s2,840.*s3;
        const Eigen::VectorXd v=c.transpose()*b1,a=c.transpose()*b2,
          jerk=c.transpose()*b3,snap4=c.transpose()*b4;
        const double alpha=1./K*j,omg=(j==0||j==K)?0.5:1.;
        Eigen::Vector2d gv2=Eigen::Vector2d::Zero(),ga2=gv2,gj2=gv2;
        double cost=0.;
        if(native_->feasibilityGradCostCar(v.head(2),a.head(2),jerk.head(2),id,
                                         gv2,ga2,gj2,cost,false)){
          Eigen::VectorXd gv=Eigen::VectorXd::Zero(8),ga=gv,gj=gv;
          gv.head(2)=gv2;ga.head(2)=ga2;gj.head(2)=gj2;
          snap.get_gdC().block(i*8,0,8,8)+=omg*step*(b1*gv.transpose());
          gdT[i]+=omg*(cost/K+step*alpha*gv.dot(a));
          snap.get_gdC().block(i*8,0,8,8)+=omg*step*(b2*ga.transpose());
          gdT[i]+=omg*(step*alpha*ga.dot(jerk));
          snap.get_gdC().block(i*8,0,8,8)+=omg*step*(b3*gj.transpose());
          gdT[i]+=omg*(step*alpha*gj.dot(snap4));
          total+=omg*step*cost;
        }
        double gate=0.,dgate=0.;
        if(native_->smoothedLog(.1*.1-v.head(2).squaredNorm(),.001,gate,dgate)){
          const double small_step=step/D;
          double local=s1,dalpha;int quantity;
          if(j==0){quantity=D/2;dalpha=1./K*j-1./K/D;}
          else if(j==K){quantity=D/2;local-=step/2.;dalpha=1./K*j-.5/K-1./K/D;}
          else{quantity=D;local-=step/2.;dalpha=1./K*j-.5/K-1./K/D;}
          for(int l=0;l<=quantity;++l){
            s2=local*local;s3=s2*local;s4=s2*s2;s5=s4*local;s6=s3*s3;
            d1<<0.,1.,2.*local,3.*s2,4.*s3,5.*s4,6.*s5,7.*s6;
            d2<<0.,0.,2.,6.*local,12.*s2,20.*s3,30.*s4,42.*s5;
            d3<<0.,0.,0.,6.,24.*local,60.*s2,120.*s3,210.*s4;
            d4<<0.,0.,0.,0.,24.,120.*local,360.*s2,840.*s3;
            dalpha+=1./K/D;local+=small_step;
            const Eigen::VectorXd dv=c.transpose()*d1,da=c.transpose()*d2,
              dj=c.transpose()*d3,ds=c.transpose()*d4;
            const double domg=(l==0||l==quantity)?0.5:1.;
            gv2.setZero();ga2.setZero();gj2.setZero();cost=0.;
            if(native_->feasibilityGradCostCar(dv.head(2),da.head(2),dj.head(2),id,
                                             gv2,ga2,gj2,cost,true)){
              const double gated_cost=gate*cost;
              Eigen::VectorXd parent=Eigen::VectorXd::Zero(8),gv=parent,ga=parent,gj=parent;
              parent.head(2)=-2.*dgate*cost*v.head(2);
              gv.head(2)=gate*gv2;ga.head(2)=gate*ga2;gj.head(2)=gate*gj2;
              snap.get_gdC().block(i*8,0,8,8)+=domg*small_step*
                (b1*parent.transpose()+d1*gv.transpose()+d2*ga.transpose()+d3*gj.transpose());
              gdT[i]+=domg*(gated_cost/K/D+small_step*(alpha*parent.dot(a)+
                dalpha*gv.dot(da)+dalpha*ga.dot(dj)+dalpha*gj.dot(ds)));
              total+=domg*small_step*gated_cost;
            }
          }
        }
        s1+=step;
      }
    }
    return total;
  }

 public:
  explicit Context(Opt* native):native_(native),pieces_(native->piece_num_container_),
      signs_(native->singul_container_) {
    if(native_->traj_dim_!=8 || native_->manipulator_dof_!=6 || pieces_.empty() ||
       native_->cps_num_prePiece_<1 || native_->dense_sample_resolution_<2)
      throw std::invalid_argument("cheap scaling requires configured fixed-arm 8D optimizer");
    snaps_.resize(pieces_.size());
    for(std::size_t i=0;i<pieces_.size();++i){point_end_+=8*(pieces_[i]-1);time_end_+=pieces_[i];snaps_[i].reset(pieces_[i]);}
    time_end_+=point_end_;angle_begin_=time_end_+8*(pieces_.size()-1);
    nx_=angle_begin_+pieces_.size()-1;
    for(int i=0;i<nx_;++i){
      const bool free=i<point_end_?i%8<2:(i<time_end_?true:(i<angle_begin_?(i-time_end_)%8<2:true));
      if(free)free_.push_back(i);
    }
  }
  Context(const Context&)=delete;
  Context& operator=(const Context&)=delete;

  Value evaluate(const Eigen::VectorXd& x) {
    std::vector<delivery_candidate::Part> parts;
    if(x.size()!=nx_ || !delivery_candidate::decode(x,pieces_,8,
        native_->iniState_container_,native_->finState_container_,native_->non_singul_v_,parts))
      throw std::invalid_argument("cheap scaling input layout mismatch");
    Value out;out.gradient=Eigen::VectorXd::Zero(nx_);
    double smooth=0.,wheel=0.,time=0.;int po=0,to=point_end_;
    for(std::size_t id=0;id<parts.size();++id){
      auto& snap=snaps_[id];auto& part=parts[id];
      snap.generate(part.points,part.times,part.initial,part.final);
      Eigen::VectorXd gt=Eigen::VectorXd::Zero(pieces_[id]);double s=0.;
      snap.initGradCost(gt,s);smooth+=s;wheel+=integral(int(id),gt);
      Eigen::Map<Eigen::MatrixXd> gp(out.gradient.data()+po,8,pieces_[id]-1);po+=8*(pieces_[id]-1);
      Eigen::MatrixXd gi,gf;snap.getGrad2TP(gt,gp,gi,gf);
      if(native_->opt_gear_){
        if(id>0){const double a=x[angle_begin_+id-1];
          out.gradient.segment(time_end_+8*(id-1),8)+=gi.col(0);
          out.gradient[angle_begin_+id-1]+=gi.topRows(2).col(1).dot(
            Eigen::Vector2d(native_->non_singul_v_*std::sin(a),-native_->non_singul_v_*std::cos(a)));}
        if(id+1<parts.size()){const double a=x[angle_begin_+id];
          out.gradient.segment(time_end_+8*id,8)+=gf.col(0);
          out.gradient[angle_begin_+id]+=gf.topRows(2).col(1).dot(
            Eigen::Vector2d(-native_->non_singul_v_*std::sin(a),native_->non_singul_v_*std::cos(a)));}
      }
      for(int k=0;k<pieces_[id];++k){
        const double vt=x[to+k];double chain;
        if(vt>0.)chain=vt+1.;
        else{const double den=(.5*vt-1.)*vt+1.;chain=(1.-vt)/(den*den);}
        out.gradient[to+k]=(gt[k]+native_->wei_time_)*chain;
      }
      time+=part.times.sum()*native_->wei_time_;to+=pieces_[id];
      gp.bottomRows(6).setZero();
    }
    for(std::size_t id=0;id+1<parts.size();++id)out.gradient.segment(time_end_+8*id+2,6).setZero();
    out.cost=smooth+wheel+time;
    if(!std::isfinite(out.cost)||!out.gradient.allFinite())
      throw std::runtime_error("nonfinite cheap scaling proxy");
    return out;
  }

  Result estimate(const Eigen::VectorXd& x,double h_multiplier=3e-8) {
    const auto start=std::chrono::steady_clock::now();
    Result r;r.scale=Eigen::VectorXd::Zero(nx_);r.free_indices=free_;
    for(int i:free_){
      const double h=h_multiplier*std::max(1.,std::abs(x[i]));
      Eigen::VectorXd a=x,b=x;a[i]+=h;b[i]-=h;
      const auto ga=evaluate(a),gb=evaluate(b);r.evaluations+=2;
      const double diagonal=std::abs((ga.gradient[i]-gb.gradient[i])/(2*h));
      r.scale[i]=1./std::sqrt(std::max(1.,diagonal));
    }
    r.elapsed_s=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    return r;
  }
};
} // namespace cheap_variable_scaling
