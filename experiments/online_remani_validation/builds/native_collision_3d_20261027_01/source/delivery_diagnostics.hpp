#pragma once
// Opt-in, post-decision diagnostics for isolated SIL. This header performs no
// ROS publication and never changes an objective, a gate, or a returned result.
#include "coupling_collision.hpp"
#include <Eigen/Dense>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

namespace delivery_diag {
using Snapshot=shared_collision::Snapshot;
inline const char* directory() {
  const char* p=std::getenv("COUPLING_DIAGNOSTIC_DIR");return p&&*p?p:nullptr;
}
inline std::string quote(const std::string& s) {
  std::ostringstream o;o<<'"';for(unsigned char c:s){
    if(c=='"'||c=='\\')o<<'\\'<<char(c);else if(c=='\n')o<<"\\n";
    else if(c<32)o<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;
    else o<<char(c);
  }o<<'"';return o.str();
}
inline std::string number(double v) {
  if(!std::isfinite(v))return "null";
  std::ostringstream o;o<<std::setprecision(17)<<v;return o.str();
}
template<class Derived> std::string vector(const Eigen::MatrixBase<Derived>& v) {
  std::string s="[";for(Eigen::Index i=0;i<v.size();++i){if(i)s+=',';s+=number(v.derived().coeff(i));}return s+"]";
}
template<class Derived> std::string matrix(const Eigen::MatrixBase<Derived>& m) {
  std::string s="[";for(Eigen::Index i=0;i<m.rows();++i){if(i)s+=',';s+='[';
    for(Eigen::Index j=0;j<m.cols();++j){if(j)s+=',';s+=number(m(i,j));}s+=']';}return s+"]";
}
inline double monotonic() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline std::string clock(double sim) {
  return "{\"mono\":"+number(monotonic())+",\"sim\":"+number(sim)+",\"pid\":"+std::to_string(getpid())+"}";
}
inline std::string hash(const std::string& text) {
  unsigned long long h=14695981039346656037ULL;
  for(unsigned char c:text){h^=c;h*=1099511628211ULL;}
  std::ostringstream s;s<<std::hex<<std::setw(16)<<std::setfill('0')<<h;return s.str();
}
inline void mkdirChecked(const std::string& path) {
  if(::mkdir(path.c_str(),0700)&&errno!=EEXIST)throw std::runtime_error("mkdir: "+path);
  struct stat s{};if(::stat(path.c_str(),&s)||!S_ISDIR(s.st_mode))throw std::runtime_error("not directory: "+path);
}
inline void writeExclusive(const std::string& path,const std::string& text) {
  const int fd=::open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL,0600);
  if(fd<0)throw std::runtime_error("exclusive diagnostic open: "+path);
  size_t n=0;while(n<text.size()){
    const auto got=::write(fd,text.data()+n,text.size()-n);
    if(got<0&&errno==EINTR)continue;
    if(got<=0){::close(fd);throw std::runtime_error("diagnostic write: "+path);}n+=size_t(got);
  }
  if(::close(fd))throw std::runtime_error("diagnostic close: "+path);
}
inline std::string snapshotText(const Snapshot& m) {
  std::string s="{\"schema\":\"delivery_snapshot_v1\"";
#define DELIVERY_SCALAR(key) s+=",\"" #key "\":"+number(m.key)
  DELIVERY_SCALAR(sequence);DELIVERY_SCALAR(stamp);DELIVERY_SCALAR(gx);DELIVERY_SCALAR(gy);
  DELIVERY_SCALAR(res);DELIVERY_SCALAR(margin);DELIVERY_SCALAR(comfort);DELIVERY_SCALAR(weight);
  DELIVERY_SCALAR(max_age);DELIVERY_SCALAR(max_prediction);
#undef DELIVERY_SCALAR
  s+=",\"healthy\":"+std::string(m.healthy?"true":"false")+",\"nx\":"+std::to_string(m.nx)+",\"ny\":"+std::to_string(m.ny)+",\"body\":[";
  for(size_t i=0;i<m.body.size();++i){if(i)s+=',';const auto& b=m.body[i];s+='['+number(b.x)+','+number(b.y)+','+number(b.r)+']';}
  s+="],\"obstacles\":[";for(size_t i=0;i<m.obstacles.size();++i){if(i)s+=',';const auto& b=m.obstacles[i];
    s+='['+number(b.x)+','+number(b.y)+','+number(b.vx)+','+number(b.vy)+','+number(b.r)+','+number(b.moving)+']';}
  s+="],\"field\":[";for(size_t i=0;i<m.field.size();++i){if(i)s+=',';s+=number(m.field[i]);}return s+"]}";
}
inline std::string snapshotFile(const std::shared_ptr<const Snapshot>& p) {
  if(!p)return "null";
  const std::string text=snapshotText(*p),digest=hash(text),relative="snapshots/"+digest+".json";
  const std::string root=directory();mkdirChecked(root);mkdirChecked(root+"/snapshots");
  const std::string absolute=root+'/'+relative;
  std::ifstream old(absolute);if(old.good()){
    const std::string existing((std::istreambuf_iterator<char>(old)),std::istreambuf_iterator<char>());
    if(existing!=text)throw std::runtime_error("snapshot hash collision/corruption: "+absolute);
  }else writeExclusive(absolute,text);
  return "{\"snapshot\":"+quote(relative)+",\"snapshot_payload_hash\":"+quote(digest)+",\"hash_algorithm\":\"fnv1a64_json\"}";
}
struct Frame {std::shared_ptr<const Snapshot> snapshot;double now=0.;};
struct Solve {
  Frame frame;unsigned long long request_id=0,scope_id=0;bool first_invalid_saved=false;
};
inline Solve& solve() {static thread_local Solve s;return s;}
inline void beginSolve(unsigned long long request_id,unsigned long long scope_id) noexcept {
  if(!directory())return;auto& s=solve();s=Solve{};s.request_id=request_id;s.scope_id=scope_id;
}
inline void rememberSolve(const std::shared_ptr<const Snapshot>& p,double now) noexcept {
  if(!directory())return;solve().frame={p,now};
}
inline std::string frameJson(const Frame& f) {
  const std::string ref=snapshotFile(f.snapshot);
  return "{\"now\":"+number(f.now)+","+(ref=="null"?"\"snapshot\":null":ref.substr(1,ref.size()-2))+"}";
}
inline std::string evaluationJson(const coupling_collision::Evaluation& e) {
  return "{\"valid\":"+std::string(e.valid?"true":"false")+",\"physical\":"+number(e.physical)+
    ",\"gap\":"+number(e.gap)+",\"cost\":"+number(e.cost)+",\"gx\":"+number(e.gx)+
    ",\"gy\":"+number(e.gy)+",\"gyaw\":"+number(e.gyaw)+",\"gt\":"+number(e.gt)+"}";
}
// Hard returns still come from the unchanged original evaluators. This copy
// explains their first invalid condition or closest projected body disk.
inline std::string leaf(const Snapshot& m,double x,double y,double yaw,double forecast,bool stat,bool dyn) {
  auto plain=[](const std::string& r){return "{\"reason\":"+quote(r)+"}";};
  if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw))return plain("nonfinite_pose");
  if(!std::isfinite(forecast)||forecast<0.)return plain("negative_or_nonfinite_forecast");
  if(m.body.empty())return plain("empty_body");
  if(!std::isfinite(m.margin)||m.margin<0.||!std::isfinite(m.comfort)||m.comfort<m.margin||
     !std::isfinite(m.weight)||m.weight<=0.)return plain("invalid_margin_comfort_weight");
  if(stat&&(m.nx<2||m.ny<2||!std::isfinite(m.res)||m.res<=0.||!std::isfinite(m.gx)||
     !std::isfinite(m.gy)||m.field.size()!=size_t(m.nx)*size_t(m.ny)))return plain("invalid_field_metadata");
  if(dyn&&!m.obstacles.empty()&&(!std::isfinite(m.max_prediction)||m.max_prediction<=0.||forecast>m.max_prediction))
    return plain("dynamic_forecast_outside_horizon");
  double best=std::numeric_limits<double>::infinity();std::string closest=plain("no_geometry");
  const double co=std::cos(yaw),si=std::sin(yaw);
  for(size_t i=0;i<m.body.size();++i){const auto& b=m.body[i];
    const std::string prefix="\"body_index\":"+std::to_string(i);
    if(!std::isfinite(b.x)||!std::isfinite(b.y)||!std::isfinite(b.r)||b.r<=0.)return "{\"reason\":\"invalid_body_disk\","+prefix+"}";
    const double px=x+co*b.x-si*b.y,py=y+si*b.x+co*b.y;
    if(stat){const double fx=(px-m.gx)/m.res,fy=(py-m.gy)/m.res;
      const std::string coords=","+prefix+",\"body_center\":["+number(px)+','+number(py)+"],\"grid\":["+number(fx)+','+number(fy)+']';
      if(!std::isfinite(fx)||!std::isfinite(fy)||fx<0.||fy<0.||fx>=m.nx-1||fy>=m.ny-1)
        return "{\"reason\":\"static_field_out_of_bounds\""+coords+"}";
      const int ix=int(std::floor(fx)),iy=int(std::floor(fy));
      const double qs[]={m.field[iy*m.nx+ix],m.field[iy*m.nx+ix+1],m.field[(iy+1)*m.nx+ix],m.field[(iy+1)*m.nx+ix+1]};
      for(double q:qs)if(!std::isfinite(q)||q<0.)return "{\"reason\":\"invalid_field_cell_value\""+coords+"}";
      const auto distance=shared_collision::mapDistance(m,px,py);const double physical=distance.d-b.r;
      if(physical<best){best=physical;closest="{\"reason\":\"static_minimum\""+coords+",\"distance\":"+number(distance.d)+
        ",\"physical\":"+number(physical)+",\"gap\":"+number(physical-m.margin)+"}";}
    }
    if(dyn)for(size_t j=0;j<m.obstacles.size();++j){const auto& o=m.obstacles[j];
      const std::string indices=","+prefix+",\"obstacle_index\":"+std::to_string(j);
      if(!std::isfinite(o.x)||!std::isfinite(o.y)||!std::isfinite(o.vx)||!std::isfinite(o.vy)||!std::isfinite(o.r)||
          o.r<=0.||(o.moving!=0.&&o.moving!=1.))return "{\"reason\":\"invalid_obstacle\""+indices+"}";
      const double ox=o.x+forecast*o.vx*o.moving,oy=o.y+forecast*o.vy*o.moving;
      const double distance=std::hypot(px-ox,py-oy),physical=distance-o.r-b.r;
      if(!std::isfinite(distance))return "{\"reason\":\"nonfinite_dynamic_distance\""+indices+"}";
      if(physical<best){best=physical;closest="{\"reason\":\"dynamic_minimum\""+indices+",\"obstacle_center\":["+
        number(ox)+','+number(oy)+"],\"distance\":"+number(distance)+",\"physical\":"+number(physical)+
        ",\"gap\":"+number(physical-m.margin)+"}";}
    }
  }return closest;
}
inline void unavailable(const char* what) noexcept {
  const char prefix[]="COUPLING_DIAGNOSTIC_UNAVAILABLE ";
  const auto a=::write(STDERR_FILENO,prefix,sizeof(prefix)-1);(void)a;
  size_t n=0;while(what[n])++n;
  const auto b=::write(STDERR_FILENO,what,n);(void)b;
  const auto c=::write(STDERR_FILENO,"\n",1);(void)c;
}
inline std::string nextName(const std::string& group) {
  static std::atomic<unsigned long long> serial{0};
  return std::string(directory())+'/'+group+'/'+std::to_string(getpid())+'_'+std::to_string(++serial)+".json";
}
template<class Data> std::string partsJson(const Data& data) {
  std::string text="[";size_t j=0;for(const auto& part:data.singul_traj){if(j++)text+=',';
    text+="{\"gear\":"+std::to_string(part.singul)+",\"duration\":"+number(part.duration)+",\"pieces\":[";
    for(int i=0;i<part.traj.getPieceNum();++i){if(i)text+=',';const auto& p=part.traj.getPiece(i);
      text+="{\"T\":"+number(p.getDuration())+",\"basis\":\"descending\",\"coeffs\":"+matrix(p.getCoeffMat())+"}";}
    text+="]}";
  }return text+"]";
}
template<class Data> class CandidateRecord {
  const Data& data_;bool enabled_=false,finished_=false,accepted_=false;
  unsigned long long request_id_=0,scope_id_=0;Frame solve_,accept_;std::string failure_="null";
public:
  CandidateRecord(const Data& data,unsigned long long request_id):data_(data),enabled_(directory()),request_id_(request_id) {
    if(enabled_){const auto& s=solve();scope_id_=s.scope_id;if(s.request_id==request_id)solve_=s.frame;}
  }
  void acceptance(const std::shared_ptr<const Snapshot>& p,double now) noexcept {if(enabled_)accept_={p,now};}
  void accept() noexcept {finished_=true;accepted_=true;}
  void reject(const char* reason) noexcept {finished_=true;accepted_=false;if(!enabled_)return;
    try{failure_="{\"reason\":"+quote(reason)+"}";}catch(...){unavailable("rejection metadata allocation");}}
  template<class P,class V>
  void rejectSample(const char* reason,size_t part,int sample,double q,double query_time,
                    const P& p,const V& v,double yaw,const coupling_collision::Evaluation& value,bool dynamic) noexcept {
    finished_=true;accepted_=false;if(!enabled_)return;
    try {double remaining=q;int piece=0;const auto& traj=data_.singul_traj[part].traj;
      while(piece+1<traj.getPieceNum()&&remaining>traj.getPiece(piece).getDuration())remaining-=traj.getPiece(piece++).getDuration();
      const std::string detail=accept_.snapshot?leaf(*accept_.snapshot,p(0),p(1),yaw,dynamic?query_time-accept_.snapshot->stamp:0.,!dynamic,dynamic):"null";
      failure_="{\"reason\":"+quote(reason)+",\"part\":"+std::to_string(part)+",\"piece\":"+std::to_string(piece)+
        ",\"sample\":"+std::to_string(sample)+",\"q\":"+number(q)+",\"piece_time\":"+number(remaining)+
        ",\"query_time\":"+number(query_time)+",\"p\":"+vector(p)+",\"v\":"+vector(v)+",\"yaw\":"+number(yaw)+
        ",\"physical\":"+number(value.physical)+",\"gap\":"+number(value.gap)+",\"evaluation\":"+evaluationJson(value)+",\"leaf\":"+detail+"}";
    }catch(const std::exception& e){unavailable(e.what());}catch(...){unavailable("sample metadata allocation");}
  }
  ~CandidateRecord() noexcept {
    if(!enabled_)return;try{
      const std::string root=directory();mkdirChecked(root);mkdirChecked(root+"/candidates");
      const std::string text="{\"schema\":\"delivery_candidate_v1\",\"request_id\":"+std::to_string(request_id_)+
        ",\"scope_id\":"+std::to_string(scope_id_)+",\"clock\":"+clock(accept_.now)+
        ",\"start_time\":"+number(data_.start_time)+",\"solve_frame\":"+frameJson(solve_)+",\"accept_frame\":"+frameJson(accept_)+
        ",\"finished\":"+(finished_?"true":"false")+",\"accepted\":"+(accepted_?"true":"false")+
        ",\"failure\":"+failure_+",\"parts\":"+partsJson(data_)+"}";
      writeExclusive(nextName("candidates"),text);
    }catch(const std::exception& e){unavailable(e.what());}catch(...){unavailable("unknown candidate logging failure");}
  }
};
template<class Opts,class Times,class Signs,class B,class P>
void geometryError(const Opts& opts,const Times& times,const Signs& signs,const Frame& frame,
                   size_t block,int piece,int sample,double prefix,double s,
                   const B& beta0,const B& beta1,const B& beta2,const P& p,const P& v,const P& a) noexcept {
  if(!directory()||solve().first_invalid_saved)return;solve().first_invalid_saved=true;
  try {
    const double query=frame.now+prefix+s,forecast=frame.snapshot?query-frame.snapshot->stamp:std::numeric_limits<double>::quiet_NaN();
    const double yaw=std::atan2(signs[block]*v.y(),signs[block]*v.x());
    std::string detail="null";
    if(!p.allFinite()||!v.allFinite())detail="{\"reason\":\"nonfinite_position_or_velocity\"}";
    else if(frame.snapshot){
      const auto stat=coupling_collision::evaluate(*frame.snapshot,p.x(),p.y(),yaw,0.,{true,false,.25});
      if(!stat.valid)detail=leaf(*frame.snapshot,p.x(),p.y(),yaw,0.,true,false);
      else if(!std::isfinite(forecast)||forecast<0.)detail="{\"reason\":\"negative_or_nonfinite_forecast\"}";
      else if(forecast<=frame.snapshot->max_prediction){
        const auto dyn=coupling_collision::evaluate(*frame.snapshot,p.x(),p.y(),yaw,forecast,{false,true,std::min(.25,frame.snapshot->max_prediction)});
        detail=dyn.valid?"{\"reason\":\"nonfinite_aggregate_or_velocity_chain_gradient\"}":leaf(*frame.snapshot,p.x(),p.y(),yaw,forecast,false,true);
      }else detail="{\"reason\":\"nonfinite_static_aggregate_or_velocity_chain_gradient\"}";
    }
    std::string blocks="[";for(size_t i=0;i<opts.size();++i){if(i)blocks+=',';
      blocks+="{\"gear\":"+std::to_string(signs[i])+",\"times\":"+vector(times[i])+",\"basis\":\"ascending_rows_per_8\",\"coeffs\":"+matrix(opts[i].get_b())+"}";}
    blocks+=']';const std::string root=directory();mkdirChecked(root);mkdirChecked(root+"/geometry_errors");
    const auto& ids=solve();const std::string text="{\"schema\":\"delivery_geometry_error_v1\",\"request_id\":"+std::to_string(ids.request_id)+
      ",\"scope_id\":"+std::to_string(ids.scope_id)+",\"clock\":"+clock(frame.now)+",\"frame\":"+frameJson(frame)+
      ",\"block\":"+std::to_string(block)+",\"piece\":"+std::to_string(piece)+",\"sample\":"+std::to_string(sample)+
      ",\"prefix\":"+number(prefix)+",\"s\":"+number(s)+",\"T\":"+number(times[block][piece])+",\"query_time\":"+number(query)+
      ",\"forecast\":"+number(forecast)+",\"p\":"+vector(p)+",\"v\":"+vector(v)+",\"a\":"+vector(a)+",\"yaw\":"+number(yaw)+
      ",\"beta0\":"+vector(beta0)+",\"beta1\":"+vector(beta1)+",\"beta2\":"+vector(beta2)+",\"leaf\":"+detail+",\"blocks\":"+blocks+"}";
    writeExclusive(nextName("geometry_errors"),text);
  }catch(const std::exception& e){unavailable(e.what());}catch(...){unavailable("unknown geometry logging failure");}
}
} // namespace delivery_diag
