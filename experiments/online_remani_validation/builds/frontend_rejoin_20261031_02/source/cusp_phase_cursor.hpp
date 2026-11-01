#pragma once
// Experiment-only native-reference gear association. Only measured API states
// advance phases; sampled rollout states only read an immutable phase range.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

namespace cusp_phase {
inline bool enabled(){const char* p=std::getenv("MPPI_CUSP_PHASE_ASSOCIATION");return p&&std::strcmp(p,"1")==0;}
struct Sample {double x,y,yaw,v;};
struct State {double x,y,yaw,v,w;};
struct Phase {int begin=0,end=0,gear=1;};
class Cursor {
  std::vector<Phase> phases_;
  int phase_=0,confirm_=0,transitions_=0,event_=0;
  std::uint64_t fingerprint_=0;
  double distance_=0.,yaw_error_=0.,remaining_=0.;
  static int sign(double v){return v>.005?1:v<-.005?-1:0;}
  static std::uint64_t fingerprint(const std::vector<Sample>& ref){
    std::uint64_t h=14695981039346656037ULL;
    for(const auto& r:ref)for(double d:{r.x,r.y,r.yaw,r.v}){
      if(!std::isfinite(d))throw std::invalid_argument("nonfinite cusp reference");
      if(d==0.)d=0.;std::uint64_t bits;std::memcpy(&bits,&d,sizeof bits);
      for(int j=0;j<8;++j){h^=(bits>>(8*j))&255ULL;h*=1099511628211ULL;}
    }return h;
  }
  void partition(const std::vector<Sample>& ref){
    phases_.clear();int begin=0,gear=1;
    for(const auto& r:ref)if(sign(r.v)){gear=sign(r.v);break;}
    for(int i=1;i<int(ref.size());++i){const int next=sign(ref[i].v);
      if(next&&next!=gear){phases_.push_back({begin,i-1,gear});begin=i;gear=next;}}
    phases_.push_back({begin,int(ref.size())-1,gear});
    for(const auto& p:phases_)if(p.end<=p.begin)throw std::invalid_argument("cusp phase has fewer than two reference points");
  }
  static double projection(const std::vector<Sample>& ref,const Phase& p,
                           const State& s,double* remaining=nullptr){
    double arc=0.,at=0.,best=std::numeric_limits<double>::infinity();
    for(int i=p.begin;i<p.end;++i){
      const double dx=ref[i+1].x-ref[i].x,dy=ref[i+1].y-ref[i].y;
      const double length=std::hypot(dx,dy);
      if(length<=1e-9)continue;
      const double f=std::clamp(((s.x-ref[i].x)*dx+(s.y-ref[i].y)*dy)/(length*length),0.,1.);
      const double distance=std::hypot(s.x-ref[i].x-f*dx,s.y-ref[i].y-f*dy);
      if(distance<best){best=distance;at=arc+f*length;}arc+=length;
    }
    if(remaining)*remaining=std::max(0.,arc-at);return best;
  }
  void initialize(const std::vector<Sample>& ref,const State& s){
    // A delivered reference can already be underway. Initialize by actual
    // geometric association; measured nonzero gear breaks nearby ambiguity.
    int best=0;double distance=std::numeric_limits<double>::infinity();
    std::vector<double> gaps;gaps.reserve(phases_.size());
    for(int i=0;i<int(phases_.size());++i){const double d=projection(ref,phases_[i],s);gaps.push_back(d);if(d<distance){distance=d;best=i;}}
    const int actual=std::abs(s.v)>.025?sign(s.v):0;
    if(actual){double matched=std::numeric_limits<double>::infinity();int index=best;
      for(int i=0;i<int(phases_.size());++i)if(phases_[i].gear==actual&&gaps[i]<matched){matched=gaps[i];index=i;}
      if(matched<=distance+.05)best=index;
    }
    phase_=best;
  }
  void measure(const std::vector<Sample>& ref,const State& s){
    const auto& p=active();const auto& end=ref[p.end];
    distance_=std::hypot(end.x-s.x,end.y-s.y);
    yaw_error_=std::abs(std::atan2(std::sin(end.yaw-s.yaw),std::cos(end.yaw-s.yaw)));
    projection(ref,p,s,&remaining_);
  }
 public:
  void reset(){phases_.clear();phase_=confirm_=transitions_=event_=0;fingerprint_=0;distance_=yaw_error_=remaining_=0.;}
  bool update(const std::vector<Sample>& ref,const State& s){
    if(ref.size()<2||!std::isfinite(s.x)||!std::isfinite(s.y)||!std::isfinite(s.yaw)||
       !std::isfinite(s.v)||!std::isfinite(s.w))throw std::invalid_argument("invalid measured cusp state");
    const auto hash=fingerprint(ref);event_=0;
    if(phases_.empty()||hash!=fingerprint_){partition(ref);initialize(ref,s);confirm_=transitions_=0;fingerprint_=hash;event_=1;measure(ref,s);return true;}
    measure(ref,s);
    const bool reached=phase_+1<int(phases_.size())&&distance_<=.05&&remaining_<=.05&&
      std::abs(s.v)<=.025&&std::abs(s.w)<=.20;
    confirm_=reached?confirm_+1:0;
    if(confirm_>=2){++phase_;++transitions_;confirm_=0;event_=2;measure(ref,s);return true;}
    return false;
  }
  const Phase& active()const{return phases_.at(phase_);}
  void debug(double* out)const{
    const double v[]={double(!phases_.empty()),double(phase_),double(phases_.size()),
      phases_.empty()?-1.:double(active().begin),phases_.empty()?-1.:double(active().end),
      phases_.empty()?0.:double(active().gear),distance_,yaw_error_,remaining_,
      double(transitions_),double(event_),double(confirm_),double(std::uint32_t(fingerprint_>>32)),double(std::uint32_t(fingerprint_))};
    std::copy(v,v+14,out);
  }
};

struct Range {const void* token=nullptr;std::size_t size=0;Phase phase;};
inline const Range*& current(){static thread_local const Range* p=nullptr;return p;}
inline const Phase* bound(const void* token,std::size_t size){const auto* p=current();return p&&p->token==token&&p->size==size?&p->phase:nullptr;}
struct Scope {
  Range range;const Range* previous;
  Scope(const void* token,std::size_t size,const Phase* phase):previous(current()){
    if(phase){range={token,size,*phase};current()=&range;}
  }
  ~Scope(){current()=previous;}
  Scope(const Scope&)=delete;Scope& operator=(const Scope&)=delete;
};
} // namespace cusp_phase
