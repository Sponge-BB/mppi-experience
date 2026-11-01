#ifndef REMANI_EXPERIMENT_SEED_PARAMETERIZATION_HPP
#define REMANI_EXPERIMENT_SEED_PARAMETERIZATION_HPP
// Isolated seed repair only. No ROS, collision query or optimization policy.
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <vector>

namespace seed_parameterization {
inline bool enabled() {
  const char* value=std::getenv("SEED_PARAMETERIZATION_FIX");
  return value&&value[0]=='1'&&value[1]=='\0';
}
struct Status {
  bool applied=false,valid=false;
  const char* reason="not_called";
  std::size_t input_points=0,output_segments=0;
  double signed_start_speed=0.;
  int first_gear=0;
};
inline Status& status() { static thread_local Status value;return value; }
inline double invalid() { return std::numeric_limits<double>::quiet_NaN(); }
inline double yawDelta(double a, double b) { return std::atan2(std::sin(b-a), std::cos(b-a)); }
inline double distance(const Eigen::Vector2d& from, const Eigen::Vector2d& to,
                       double yaw0, double yaw1, double min_radius, double coefficient) {
  if (!from.allFinite() || !to.allFinite() || !std::isfinite(yaw0) ||
      !std::isfinite(yaw1) || !std::isfinite(min_radius) || min_radius < 0. ||
      !std::isfinite(coefficient) || coefficient < 0.) return invalid();
  const Eigen::Vector2d delta=to-from;
  const double chord=delta.norm(), angle=std::abs(yawDelta(yaw0,yaw1));
  if (!std::isfinite(chord) || !std::isfinite(angle)) return invalid();
  // A stationary heading change is not representable by this translating seed.
  // Do not silently delete it or turn it into a free/zero-length maneuver.
  if (chord==0.) return angle <= 1e-12 ? 0. : invalid();
  const Eigen::Vector2d heading(std::cos(yaw0),std::sin(yaw0));
  const double cross=std::abs(heading.x()*delta.y()-heading.y()*delta.x());
  // Parallel forward AND reverse steps have an infinite turning radius.
  if (cross <= 1e-12*chord) return chord;
  // Circle center relative to `from`, not an origin-dependent linear solve.
  const double radius=(.5*chord)*(chord/cross);
  if (!std::isfinite(radius)) return chord;
  if (radius < min_radius) return std::max(chord,coefficient*angle);
  return chord;
}

struct Profile {
  bool valid=false;
  double length=0., start=0., end=0., acceleration=0., peak=0.;
  double accelerate_time=0., cruise_time=0., decelerate_time=0., duration=0.;
};
inline Profile profile(double length,double start,double end,double vmax,double amax) {
  Profile p;
  if (!std::isfinite(length)||!std::isfinite(start)||!std::isfinite(end)||
      !std::isfinite(vmax)||!std::isfinite(amax)||length<=0.||start<0.||end<0.||
      vmax<=0.||amax<=0.||start>vmax||end>vmax) return p;
  p.length=length;p.start=start;p.end=end;p.acceleration=.7*amax;
  const double a=p.acceleration;
  const double minimum=std::abs(end*end-start*start)/(2.*a);
  // An infeasible endpoint-speed jump must not manufacture negative time.
  if (length < minimum) return p;
  const double critical=(vmax*vmax-start*start)/(2.*a)+(vmax*vmax-end*end)/(2.*a);
  if (length>=critical) {
    p.peak=vmax;p.cruise_time=(length-critical)/vmax;
  } else p.peak=std::sqrt(a*length+.5*(start*start+end*end));
  if (p.peak<std::max(start,end)) return Profile();
  p.accelerate_time=(p.peak-start)/a;p.decelerate_time=(p.peak-end)/a;
  p.duration=p.accelerate_time+p.cruise_time+p.decelerate_time;
  p.valid=std::isfinite(p.duration)&&p.duration>0.;
  return p;
}
inline double duration(double length,double start,double end,double vmax,double amax) {
  const auto p=profile(length,start,end,vmax,amax);return p.valid?p.duration:invalid();
}
inline double lengthAt(double time,double length,double total,double start,double end,
                       double vmax,double amax) {
  const auto p=profile(length,start,end,vmax,amax);
  if (!p.valid||!std::isfinite(time)||!std::isfinite(total)||total<=0.||time<0.||
      time>total||std::abs(total-p.duration)>1e-10*std::max(1.,p.duration)) return invalid();
  if (time==0.) return 0.;
  if (time==total) return length;
  if (time<=p.accelerate_time) return start*time+.5*p.acceleration*time*time;
  const double x1=start*p.accelerate_time+.5*p.acceleration*p.accelerate_time*p.accelerate_time;
  const double remaining=time-p.accelerate_time;
  if (remaining<=p.cruise_time) return x1+p.peak*remaining;
  const double t=remaining-p.cruise_time;
  return std::max(0.,std::min(length,x1+p.peak*p.cruise_time+p.peak*t-.5*p.acceleration*t*t));
}

template<class Flat> struct Output {
  bool valid=false;
  std::vector<Eigen::Vector3d> path;
  std::vector<double> lengths,times;
  std::vector<int> indices,gears;
  std::vector<Flat> flats;
  double total=0.;
};

template<class Flat,class GetFlat> Output<Flat> build(
    const std::vector<Eigen::Vector3d>& input,double start_speed,double end_speed,
    const Eigen::Vector2d& start_control,double non_singular,double sample_time,
    double vmax,double amax,double min_radius,double coefficient,GetFlat get_flat) {
  Output<Flat> out;
  if(input.size()<2||!std::isfinite(start_speed)||!std::isfinite(end_speed)||
     start_speed<0.||end_speed<0.||!start_control.allFinite()||
     !std::isfinite(non_singular)||non_singular<=0.||non_singular>vmax||
     !std::isfinite(sample_time)||sample_time<=0.) return out;
  for(const auto& q:input) {
    if(!q.allFinite())return Output<Flat>();
    if(!out.path.empty()&&(q.head<2>()-out.path.back().template head<2>()).norm()==0.) {
      if(std::abs(yawDelta(out.path.back()[2],q[2]))>1e-12) return Output<Flat>();
      continue;
    }
    out.path.push_back(q);
  }
  if(out.path.size()<2)return Output<Flat>();
  out.indices.push_back(0);
  double accumulated=0.;int previous=0;
  for(std::size_t i=0;i+1<out.path.size();++i) {
    const auto& a=out.path[i];const auto& b=out.path[i+1];
    const Eigen::Vector2d delta=b.template head<2>()-a.template head<2>();
    const int gear=delta.dot(Eigen::Vector2d(std::cos(a[2]),std::sin(a[2])))>=0.?1:-1;
    const double step=distance(a.template head<2>(),b.template head<2>(),a[2],b[2],min_radius,coefficient);
    if(!std::isfinite(step)||step<=0.)return Output<Flat>();
    if(previous!=0&&gear!=previous) {
      if(!std::isfinite(accumulated)||accumulated<=0.)return Output<Flat>();
      out.indices.push_back(static_cast<int>(i));out.lengths.push_back(accumulated);
      out.gears.push_back(previous);accumulated=0.;
    }
    accumulated+=step;previous=gear;
  }
  out.indices.push_back(static_cast<int>(out.path.size()-1));
  out.lengths.push_back(accumulated);out.gears.push_back(previous);
  for(std::size_t block=0;block<out.lengths.size();++block) {
    const double v0=block==0?start_speed:non_singular;
    const double v1=block+1==out.lengths.size()?end_speed:non_singular;
    const double total=duration(out.lengths[block],v0,v1,vmax,amax);
    if(!std::isfinite(total)||total<=0.)return Output<Flat>();
    out.times.push_back(total);out.total+=total;
    if(!std::isfinite(out.total))return Output<Flat>();
    Flat flat;flat.singul=out.gears[block];
    const int first=out.indices[block],last=out.indices[block+1];
    if(first<0||last<=first||static_cast<std::size_t>(last)>=out.path.size())return Output<Flat>();
    Eigen::Vector2d control=Eigen::Vector2d::Zero();if(block==0)control=start_control;
    const auto& q0=out.path[first];const auto& q1=out.path[last];
    get_flat(Eigen::Vector4d(q0[0],q0[1],q0[2],v0),control,flat.singul,flat.start_state);
    get_flat(Eigen::Vector4d(q1[0],q1[1],q1[2],v1),Eigen::Vector2d::Zero(),flat.singul,flat.final_state);
    if(!flat.start_state.allFinite()||!flat.final_state.allFinite())return Output<Flat>();
    const double step_time=total<=sample_time?total/2.:sample_time;
    if(!std::isfinite(step_time)||step_time<=0.)return Output<Flat>();
    // Bound pathological input work. This does not affect normal search seeds.
    if(total/step_time>1000000.)return Output<Flat>();
    double last_time=0.;
    for(std::size_t sample=1;;++sample) {
      const double time=std::min(total,static_cast<double>(sample)*step_time);
      if(time<=last_time)return Output<Flat>();
      Eigen::Vector3d q=q1;
      if(time<total) {
        const double arc=lengthAt(time,out.lengths[block],total,v0,v1,vmax,amax);
        if(!std::isfinite(arc))return Output<Flat>();
        double passed=0.;bool found=false;
        for(int k=first;k<last;++k) {
          const double length=distance(out.path[k].template head<2>(),out.path[k+1].template head<2>(),
              out.path[k][2],out.path[k+1][2],min_radius,coefficient);
          if(!std::isfinite(length)||length<=0.)return Output<Flat>();
          if(arc<=passed+length||k+1==last) {
            const double u=std::max(0.,std::min(1.,(arc-passed)/length));
            q=(1.-u)*out.path[k]+u*out.path[k+1];
            q[2]=out.path[k][2]+u*yawDelta(out.path[k][2],out.path[k+1][2]);
            found=true;break;
          }
          passed+=length;
        }
        if(!found)return Output<Flat>();
      }
      if(!q.allFinite())return Output<Flat>();
      flat.traj_pts.emplace_back(q[0],q[1],time-last_time);flat.thetas.push_back(q[2]);
      last_time=time;if(time==total)break;
    }
    out.flats.push_back(flat);
  }
  out.valid=!out.flats.empty();return out;
}

inline Eigen::Vector3d position(const std::vector<Eigen::Vector3d>& path,
    const std::vector<double>& lengths,const std::vector<double>& times,
    const std::vector<int>& indices,double time,double start_speed,double end_speed,
    double non_singular,double vmax,double amax,double min_radius,double coefficient) {
  const auto bad=[](){return Eigen::Vector3d::Constant(invalid()).eval();};
  if(path.size()<2||lengths.empty()||times.size()!=lengths.size()||
     indices.size()!=lengths.size()+1||!std::isfinite(time))return bad();
  double total=0.;for(double t:times){if(!std::isfinite(t)||t<=0.)return bad();total+=t;}
  if(!std::isfinite(total)||total<=0.)return bad();
  time=std::max(0.,std::min(total,time));
  if(time==0.)return path.front().allFinite()?path.front():bad();
  if(time==total)return path.back().allFinite()?path.back():bad();
  std::size_t block=0;double passed_time=0.;
  while(block+1<times.size()&&time>passed_time+times[block])passed_time+=times[block++];
  const int first=indices[block],last=indices[block+1];
  if(first<0||last<=first||static_cast<std::size_t>(last)>=path.size())return bad();
  const double v0=block==0?start_speed:non_singular;
  const double v1=block+1==times.size()?end_speed:non_singular;
  const double local_time=std::max(0.,std::min(times[block],time-passed_time));
  if(local_time==0.)return path[first].allFinite()?path[first]:bad();
  if(local_time==times[block])return path[last].allFinite()?path[last]:bad();
  const double arc=lengthAt(local_time,lengths[block],times[block],v0,v1,vmax,amax);
  if(!std::isfinite(arc))return bad();
  double passed=0.;
  for(int k=first;k<last;++k) {
    const double length=distance(path[k].head<2>(),path[k+1].head<2>(),path[k][2],path[k+1][2],min_radius,coefficient);
    if(!std::isfinite(length)||length<=0.)return bad();
    if(arc<=passed+length||k+1==last) {
      const double u=std::max(0.,std::min(1.,(arc-passed)/length));
      Eigen::Vector3d q=(1.-u)*path[k]+u*path[k+1];
      q[2]=path[k][2]+u*yawDelta(path[k][2],path[k+1][2]);
      return q.allFinite()?q:bad();
    }
    passed+=length;
  }
  return bad();
}
} // namespace seed_parameterization
#endif
