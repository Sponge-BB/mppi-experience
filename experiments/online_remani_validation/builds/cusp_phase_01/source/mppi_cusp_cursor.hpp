#pragma once
// Experiment-only measured-state phase association. A rollout may read its
// immutable phase bounds, but must NEVER advance this cursor.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

namespace mppi_cusp {
struct Sample {double x,y,yaw,v;};
struct State {double x,y,yaw,v,w;};
struct Phase {int begin,end,gear;};
class Cursor {
 public:
  static constexpr double position_tolerance=.04, yaw_tolerance=.15;
  static constexpr double linear_tolerance=.025, angular_tolerance=.15;
  static constexpr double remaining_tolerance=.05, speed_epsilon=.005;
  void reset(){phases.clear();phase=confirm=transitions=event=0;hash=0;distance=yaw_error=remaining=0;}
  bool update(const std::vector<Sample>&ref,const State&s){
    if(ref.size()<2||!finite(s))throw std::invalid_argument("invalid actual cusp state");
    const auto next=fingerprint(ref);event=0;
    if(phases.empty()||next!=hash){
      partition(ref);phase=confirm=transitions=0;hash=next;event=1;measure(ref,s);return true;
    }
    measure(ref,s);
    const bool at_cusp=phase+1<int(phases.size())&&distance<=position_tolerance&&
      yaw_error<=yaw_tolerance&&remaining<=remaining_tolerance&&
      std::abs(s.v)<=linear_tolerance&&std::abs(s.w)<=angular_tolerance;
    confirm=at_cusp?confirm+1:0;
    if(confirm>=2){++phase;++transitions;confirm=0;event=2;measure(ref,s);return true;}
    return false;
  }
  const Phase&active()const{return phases.at(phase);}
  void debug(double*out)const{
    if(!out)return;
    double values[]={double(!phases.empty()),double(phase),double(phases.size()),
      phases.empty()?-1.:double(active().begin),phases.empty()?-1.:double(active().end),
      phases.empty()?0.:double(active().gear),distance,yaw_error,remaining,
      double(transitions),double(event),double(confirm),double(std::uint32_t(hash>>32)),
      double(std::uint32_t(hash))};
    std::copy(values,values+14,out);
  }
 private:
  static bool finite(const State&s){return std::isfinite(s.x)&&std::isfinite(s.y)&&
    std::isfinite(s.yaw)&&std::isfinite(s.v)&&std::isfinite(s.w);}
  static int sign(double v){return v>speed_epsilon?1:v<-speed_epsilon?-1:0;}
  void partition(const std::vector<Sample>&ref){
    phases.clear();int gear=1,begin=0;
    for(const auto&r:ref)if(sign(r.v)){gear=sign(r.v);break;}
    for(int i=1;i<int(ref.size());++i){int next=sign(ref[i].v);
      if(next&&next!=gear){phases.push_back({begin,i-1,gear});begin=i;gear=next;}}
    phases.push_back({begin,int(ref.size())-1,gear});
  }
  void measure(const std::vector<Sample>&ref,const State&s){
    const auto&p=active();const auto&end=ref[p.end];
    distance=std::hypot(end.x-s.x,end.y-s.y);
    yaw_error=std::abs(std::atan2(std::sin(end.yaw-s.yaw),std::cos(end.yaw-s.yaw)));
    double arc=0.,best=std::numeric_limits<double>::infinity(),at=0.;
    for(int i=p.begin;i<p.end;++i){double dx=ref[i+1].x-ref[i].x,dy=ref[i+1].y-ref[i].y;
      double length=std::hypot(dx,dy),f=length>1e-9?
        std::clamp(((s.x-ref[i].x)*dx+(s.y-ref[i].y)*dy)/(length*length),0.,1.):0.;
      double d=std::hypot(s.x-ref[i].x-f*dx,s.y-ref[i].y-f*dy);
      if(d<best){best=d;at=arc+f*length;}arc+=length;
    }
    remaining=std::max(0.,arc-at);
  }
  static std::uint64_t fingerprint(const std::vector<Sample>&ref){
    std::uint64_t h=14695981039346656037ull;
    for(const auto&r:ref)for(double value:{r.x,r.y,r.yaw,r.v}){
      if(!std::isfinite(value))throw std::invalid_argument("nonfinite cusp reference");
      if(value==0.)value=0.;std::uint64_t bits;std::memcpy(&bits,&value,sizeof bits);
      for(int j=0;j<8;++j){h^=(bits>>(8*j))&255ull;h*=1099511628211ull;}}
    return h;
  }
  std::vector<Phase>phases;int phase=0,confirm=0,transitions=0,event=0;
  std::uint64_t hash=0;double distance=0,yaw_error=0,remaining=0;
};
} // namespace mppi_cusp
