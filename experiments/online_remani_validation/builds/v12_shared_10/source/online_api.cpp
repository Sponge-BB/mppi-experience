#include "online_kernel.hpp"
#include "directed_reference.hpp"
#include <limits>
using namespace experiment;
namespace {
const double* grid=nullptr; int nx=0,ny=0;double grid_x0=0,grid_y0=0,res=.05;
double distance(double x,double y){
  double fx=(x-grid_x0)/res,fy=(y-grid_y0)/res;int ix=std::floor(fx),iy=std::floor(fy);
  if(!grid||ix<0||iy<0||ix>=nx-1||iy>=ny-1)return 0.;
  double a=fx-ix,b=fy-iy;
  return (1-b)*((1-a)*grid[iy*nx+ix]+a*grid[iy*nx+ix+1])+b*((1-a)*grid[(iy+1)*nx+ix]+a*grid[(iy+1)*nx+ix+1]);
}
struct Online {V actual{};directed_reference::Cursor cursor;const double* classified_static=nullptr;int static_width=0,static_height=0;C c;MPPI m;Online(const double*p,int seed):c{p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7],p[8],p[9],p[10],p[11],p[12],p[13],p[14],p[15]},m(c,seed){}};

// A phase switch is a reference association change, not an actuator reset.
// In particular h.actual and the mutable Cursor are deliberately preserved.
void clearDirectedPhaseState(Online&h){
  auto&m=h.m;m.warm.clear();m.route=Scene{};m.published_route.clear();
  m.side=0;m.cycles=0;m.failed=false;m.clear_ticks=0;m.strategy=0;m.phase=0;
  m.recovering=false;m.recovery_clear=0;m.committed_side=0;m.clear_count=0;
  m.d_lateral_needed=false;m.d_using_new=false;m.referenceChanged();
  m.search_trace.clear();m.status={};m.debug={};m.avoidance_debug={};
  m.decision_debug={};m.recovery_debug={};m.margin_debug={};
}
}
extern "C" void* online_create(const double*p,const double*body,int n,int seed){
  try{auto*h=new Online(p,seed);for(int i=0;i<n;++i)h->c.body.push_back({{body[3*i],body[3*i+1]},body[3*i+2]});h->m.trace_enabled=true;return h;}catch(...){return nullptr;}
}
extern "C" void online_destroy(void*ptr){delete static_cast<Online*>(ptr);}
extern "C" int online_step(void*ptr,const double*state,const double*refs,int nr,
    const double*obs,int no,double age,const double*field,int width,int height,
    double gx,double gy,double resolution,int cbf,double*out){
  try{
    if(!ptr||nr<2||age<-.01||age>.35||cbf){if(ptr){auto&h=*static_cast<Online*>(ptr);h.classified_static=nullptr;h.m.search_trace.clear();h.m.recovery_debug={};}return 1;}
    auto&h=*static_cast<Online*>(ptr);grid=(h.classified_static&&h.static_width==width&&h.static_height==height&&no>0)?h.classified_static:field;nx=width;ny=height;grid_x0=gx;grid_y0=gy;res=resolution;map_distance=distance;h.classified_static=nullptr; // Classified layer is one-step only; never silently reuse stale membership.
    S s{state[0],state[1],state[2],state[3],state[4]};Scene sc;sc.ymax=100.;sc.name="online";
    for(int i=0;i<nr;++i)sc.path.push_back({{refs[4*i],refs[4*i+1]},refs[4*i+2],refs[4*i+3]});

    std::vector<directed_reference::Sample>cursor_reference;cursor_reference.reserve(sc.path.size());
    for(const auto&r:sc.path)cursor_reference.push_back({r.p.x,r.p.y,r.yaw,r.v});
    // The ONLY mutable cursor update. Candidates below have only this Scene's
    // value snapshot, and cannot see or advance h.cursor.
    const auto cursor_event=h.cursor.update(cursor_reference,{s.x,s.y,s.v});
    const auto&directed_phase=h.cursor.active();
    sc.directed=true;sc.active_begin=directed_phase.begin;sc.active_end=directed_phase.end;
    sc.active_gear=directed_phase.gear;sc.local_end=-1;sc.local_gear=0;
    if(cursor_event.started||cursor_event.switched){
      clearDirectedPhaseState(h);
      auto collision=shared_collision::snapshot();
      double event_time=collision?collision->stamp+std::max(0.,age)-1000.:std::numeric_limits<double>::quiet_NaN();
      std::ostringstream line;line<<std::setprecision(17)<<"[DirectedReference] t="<<event_time
        <<" event="<<(cursor_event.started?"phase_start":"phase_switch")
        <<" ref_fnv64="<<std::hex<<h.cursor.reference_hash()<<std::dec
        <<" from="<<cursor_event.previous_phase<<" phase="<<h.cursor.phase_id()
        <<" count="<<h.cursor.phase_count()<<" gear="<<sc.active_gear
        <<" begin="<<sc.active_begin<<" end="<<sc.active_end
        <<" measured="<<s.x<<","<<s.y<<","<<s.yaw<<","<<s.v<<","<<s.w
        <<" checked_cusp_distance="<<cursor_event.checked_cusp_distance;
      std::cout<<line.str()<<std::endl;
    }
    Z z;z.t=std::max(0.,age);z.stamp=0;z.fresh=true;
    for(int i=0;i<no;++i)z.os.push_back({{obs[6*i],obs[6*i+1]},{obs[6*i+2],obs[6*i+3]},obs[6*i+4],obs[6*i+5]>.5});
    auto start=std::chrono::steady_clock::now();auto u=h.m.plan(s,z,sc);F f{u};if(cbf)f=filter(s,u,z,h.c,sc.ymax);
    out[0]=f.u.x;out[1]=f.u.y;out[2]=u.x;out[3]=u.y;out[4]=f.intervention;out[5]=f.qp_failed;out[6]=h.m.avoidance_debug[7];
    out[7]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();return 0;
  }catch(...){return 2;}
}
extern "C" void online_feedback(void*ptr,double v,double w){if(ptr){auto&h=*static_cast<Online*>(ptr);h.actual={v,w};h.m.feedback({v,w});}}
// Offline forensic sweep for a recorded failed QP. Not used to select controls.
extern "C" int online_audit(void*ptr,const double*state,const double*obs,int no,double age,
    const double*field,int width,int height,double gx,double gy,double resolution,
    const double*nominal,double*out){
  try{
    auto&h=*static_cast<Online*>(ptr);grid=field;nx=width;ny=height;grid_x0=gx;grid_y0=gy;res=resolution;map_distance=distance;
    S s{state[0],state[1],state[2],state[3],state[4]};Z z;z.t=age;
    for(int i=0;i<no;++i)z.os.push_back({{obs[6*i],obs[6*i+1]},{obs[6*i+2],obs[6*i+3]},obs[6*i+4],obs[6*i+5]>.5});
    auto h0=barrier(s,z,0,h.c,100.);out[0]=*std::min_element(h0.begin(),h0.end());
    auto residual=[&](V u){auto hs=barrier(step(s,u,h.c.dt,h.c),z,h.c.dt,h.c,100.);double r=1e9;for(size_t i=0;i<hs.size();++i)r=std::min(r,hs[i]-std::exp(-h.c.alpha*h.c.dt)*h0[i]);return r;};
    out[1]=clearance(s,predict(z,0,h.c),h.c,100.,h.c.margin);out[2]=residual({nominal[0],nominal[1]});out[3]=residual({0,0});
    out[4]=0;out[5]=-1e9;out[6]=out[7]=0;
    for(int iv=-20;iv<=20;++iv)for(int iw=-20;iw<=20;++iw){V u=bound({iv*h.c.vmax/20,iw*h.c.wmax/20},h.c);double r=residual(u);
      bool valid=r>=-1e-7;S p=s;for(int k=1;k<=10;++k){p=step(p,u,h.c.dt/10,h.c,-1,h.c.dt/10);if(clearance(p,predict(z,k*h.c.dt/10,h.c),h.c,100.,h.c.margin)<-1e-7)valid=false;}
      if(valid)out[4]+=1;
      if(r>out[5]){out[5]=r;out[6]=u.x;out[7]=u.y;}
    }return 0;
  }catch(...){return 1;}
}

extern "C" void online_reset(void*p){if(p){auto&h=*static_cast<Online*>(p);h.cursor.reset();h.m.reset();}}
extern "C" void online_debug(void*p,double*out){if(p)std::copy(static_cast<Online*>(p)->m.debug.begin(),static_cast<Online*>(p)->m.debug.end(),out);}
extern "C" void online_status(void*p,double*out){if(p)std::copy(static_cast<Online*>(p)->m.status.begin(),static_cast<Online*>(p)->m.status.end(),out);}
extern "C" int online_local_route(void*p,double*out,int capacity){if(!p)return 0;auto&r=static_cast<Online*>(p)->m.published_route;int n=std::min(capacity,int(r.size()));for(int i=0;i<n;++i){out[2*i]=r[i].p.x;out[2*i+1]=r[i].p.y;}return n;}
extern "C" int online_revision(){return 8;}
extern "C" double online_pose_clearance(void*p,double x,double y,double yaw){return static_cast<Online*>(p)->m.poseDistance({x,y},yaw);}
extern "C" void online_set_classified_static(void*p,const double*field,int width,int height){if(p){auto&h=*static_cast<Online*>(p);h.classified_static=field;h.static_width=width;h.static_height=height;}}
extern "C" void online_avoidance_debug(void*p,double*out){if(p)std::copy(static_cast<Online*>(p)->m.avoidance_debug.begin(),static_cast<Online*>(p)->m.avoidance_debug.end(),out);}
extern "C" void online_decision_debug(void*p,double*out){if(p)std::copy(static_cast<Online*>(p)->m.decision_debug.begin(),static_cast<Online*>(p)->m.decision_debug.end(),out);}
extern "C" void online_search_enable(void*p,int enabled){if(p){auto&m=static_cast<Online*>(p)->m;m.trace_enabled=enabled!=0;m.search_trace.clear();}}
extern "C" int online_search_trace(void*p,double*out,int capacity){
          if(!p)return 0;auto&h=*static_cast<Online*>(p);auto&m=h.m;int n=0;
          if(!out){for(const auto&r:m.search_trace)n+=int(r.controls.size())+1;return n;}
          for(size_t id=0;id<m.search_trace.size();++id){const auto&r=m.search_trace[id];auto s=m.trace_start;
            for(size_t k=0;k<=r.controls.size();++k){
              if(out&&n<capacity){double row[]={double(id),k*h.c.rdt,s.x,s.y,r.cost,r.gap,double(r.feasible),double(r.selected)};std::copy(row,row+8,out+8*n);}++n;
              if(k<r.controls.size()){s=experiment::step(s,r.controls[k],h.c.rdt/2,h.c);s=experiment::step(s,r.controls[k],h.c.rdt/2,h.c);}
            }
          }return n;}
extern "C" void online_recovery_debug(void*p,double*out){if(p)std::copy(static_cast<Online*>(p)->m.recovery_debug.begin(),static_cast<Online*>(p)->m.recovery_debug.end(),out);}

extern "C" void online_margin_debug(void*p,double*out){if(p)std::copy(static_cast<Online*>(p)->m.margin_debug.begin(),static_cast<Online*>(p)->m.margin_debug.end(),out);}

// V10: verify the exact selected controls on the publication-time snapshot.
// Output rows: relative time, x, y, yaw, v, w, target_v, target_w.
extern "C" int online_selected_controls(void*p,double*out,int capacity){
  if(!p)return 0;auto&us=static_cast<Online*>(p)->m.warm;
  if(!out)return us.size();int n=std::min(capacity,int(us.size()));
  for(int i=0;i<n;++i){out[2*i]=us[i].x;out[2*i+1]=us[i].y;}return n;
}
extern "C" void online_reference_changed(void*p){
  if(p){auto&h=*static_cast<Online*>(p);h.cursor.reset();clearDirectedPhaseState(h);}
}
extern "C" int online_validate_execution(void*p,const double*state,
    const double*controls,int count,const double*obs,int no,double age,double state_age,
    const double*field,int width,int height,double gx,double gy,double resolution,
    double*out,int capacity,double*report){
  // report: valid, selected min gap, brake min gap, rows, restoration, brake valid.
  std::fill(report,report+6,0.);
  try{
    if(!p||!field||count<1||count>64||capacity<2||age<0||age>.35||state_age<0||state_age>.2)return 1;
    for(int i=0;i<5;++i)if(!std::isfinite(state[i]))return 2;
    for(int i=0;i<no*6;++i)if(!std::isfinite(obs[i]))return 2;
    auto&h=*static_cast<Online*>(p);auto&c=h.c;
    // Solver time consumes part of the CV horizon. Validate the remaining
    // selected prefix; never extrapolate stale tail points beyond validity.
    if(no>0)count=std::min(count,int(std::floor((c.max_prediction-age-.02)/c.rdt)));
    if(count<1)return 4;
    grid=field;nx=width;ny=height;grid_x0=gx;grid_y0=gy;res=resolution;map_distance=distance;
    S initial{state[0],state[1],state[2],state[3],state[4]};
    // The previous issued command, rather than a reference derivative, bridges
    // the measured state to the current publication time.
    initial=step(initial,h.actual,state_age,c);
    Z z;z.t=age;z.fresh=true;
    for(int i=0;i<no;++i)z.os.push_back({{obs[6*i],obs[6*i+1]},{obs[6*i+2],obs[6*i+3]},obs[6*i+4],obs[6*i+5]>.5});
    double initial_gap=clearance(initial,predict(z,0,c),c,100.,c.margin);
    bool restoration=no==0&&initial_gap<0&&initial_gap+c.margin>1e-6;
    if(initial_gap<0&&!restoration)return 3;
    report[4]=restoration;
    auto check=[&](double d,double&previous,bool&restore){
      if(!std::isfinite(d))return false;
      bool ok=restore ? d+c.margin>1e-6&&d>=previous-1e-9 : d>=-1e-9;
      if(d>=0)restore=false;previous=d;return ok;
    };
    auto row=[&](int n,double t,const S&s,V u){
      if(n>=capacity)return false;
      double values[]={t,s.x,s.y,s.yaw,s.v,s.w,u.x,u.y};
      std::copy(values,values+8,out+8*n);return true;
    };
    S s=initial;double t=0,previous=initial_gap,min_gap=initial_gap;
    bool restore=restoration;int n=0;V first{controls[0],controls[1]};
    if(!row(n++,0,s,first))return 1;
    for(int k=0;k<count;++k){
      V u{controls[2*k],controls[2*k+1]};
      if(!std::isfinite(u.x)||!std::isfinite(u.y)||norm(bound(u,c)-u)>1e-9)return 2;
      for(int j=0;j<10;++j){
        t+=c.rdt/10;if(no>0&&age+t>c.max_prediction-1e-6)return 4;
        s=step(s,u,c.rdt/10,c);double gap=clearance(s,predict(z,t,c),c,100.,c.margin);
        min_gap=std::min(min_gap,gap);
        if(!check(gap,previous,restore)){report[1]=min_gap;return 3;}
        if(!row(n++,t,s,u))return 1;
      }
    }
    if(restore)return 3; // A margin-restoration rollout must restore full margin.
    report[1]=min_gap;report[3]=n;
    // Independently check the fallback available after the next issued cycle:
    // execute first action for the 75ms actuator lease, then brake with the same lag/wheel model.
    s=initial;t=0;previous=initial_gap;restore=restoration;double brake_gap=initial_gap,settled=0;
    for(int j=0;j<200;++j){
      V u=t<.075-1e-8?first:V{};t+=.01;
      if(no>0&&age+t>c.max_prediction-1e-6)break;
      s=step(s,u,.01,c);double d=clearance(s,predict(z,t,c),c,100.,c.margin);
      brake_gap=std::min(brake_gap,d);
      // Static restoration may safely hold its positive physical clearance
      // while braking; it does not require braking back out to full margin.
      bool ok=restoration ? d+c.margin>1e-6 : check(d,previous,restore);
      if(!ok){report[2]=brake_gap;return 5;}
      if(t>=.075&&std::abs(s.v)<.01&&std::abs(s.w)<.03)settled+=.01;else settled=0;
      if(settled>=.10){report[5]=1;break;}
    }
    report[2]=brake_gap;if(!report[5])return 5;
    report[0]=1;return 0;
  }catch(...){return 6;}
}

// Export bounded, distinct, already kernel-feasible alternatives. This is NOT
// execution approval: every attempted sequence needs the fresh rollout/brake
// validator, including the original winner at index zero.
extern "C" int online_execution_candidates(void*p,double*out,int capacity,int stride,double*lengths){
  if(!p||!out||!lengths||capacity<1||capacity>16||stride<1||stride>64)return 0;
  auto&m=static_cast<Online*>(p)->m;
  std::vector<const std::vector<V>*> candidates;
  if(!m.warm.empty())candidates.push_back(&m.warm);
  std::vector<const MPPI::SearchTrace*> ranked;
  for(const auto&r:m.search_trace)if(r.feasible&&!r.controls.empty())ranked.push_back(&r);
  std::stable_sort(ranked.begin(),ranked.end(),[](const auto*a,const auto*b){return a->cost<b->cost;});
  for(auto*r:ranked){
    if(int(candidates.size())>=capacity)break;
    if(int(r->controls.size())>stride)continue;
    bool duplicate=false;
    for(auto*prior:candidates){
      if(prior->size()!=r->controls.size())continue;
      double difference=0;
      for(size_t i=0;i<prior->size();++i)difference=std::max(difference,norm((*prior)[i]-r->controls[i]));
      if(difference<1e-5){duplicate=true;break;}
    }
    if(!duplicate)candidates.push_back(&r->controls);
  }
  int n=0;
  for(auto*us:candidates){
    if(int(us->size())>stride)continue;
    lengths[n]=us->size();
    for(size_t k=0;k<us->size();++k){out[2*(n*stride+k)]=(*us)[k].x;out[2*(n*stride+k)+1]=(*us)[k].y;}
    ++n;
  }return n;
}

extern "C" int online_accept_execution_controls(void*p,const double*controls,int count){
  if(!p||count<1||count>64)return 1;
  auto&h=*static_cast<Online*>(p);std::vector<V> checked;
  for(int i=0;i<count;++i){
    V u{controls[2*i],controls[2*i+1]};
    if(!std::isfinite(u.x)||!std::isfinite(u.y)||norm(bound(u,h.c)-u)>1e-9)return 2;
    checked.push_back(u);
  }
  h.m.warm=std::move(checked);return 0;
}

extern "C" void online_dynamic_strategy_debug(void*p,double*out){if(p){auto&m=static_cast<Online*>(p)->m;double values[]={double(m.d_using_new),double(m.d_active),double(m.d_lateral_needed),double(m.d_had_dynamic)};std::copy(values,values+4,out);}}

extern "C" int online_set_collision_snapshot(void*p,const double*data,int count){
  if(!p||!data||count<16)return 1;
  auto&c=static_cast<Online*>(p)->c;
  if(std::abs(data[8]-c.margin)>1e-9||data[11]!=c.body.size()||count<16+3*int(c.body.size()))return 3;
  for(size_t i=0;i<c.body.size();++i)if(std::abs(data[16+3*i]-c.body[i].p.x)>1e-9||
      std::abs(data[17+3*i]-c.body[i].p.y)>1e-9||std::abs(data[18+3*i]-c.body[i].r)>1e-9)return 3;
  shared_collision::enabled()=true;
  return shared_collision::bind(data,count)?0:2;
}
extern "C" int online_common_pose(double x,double y,double yaw,double forecast,double*out){
  auto model=shared_collision::snapshot();if(!model)return 1;
  auto value=shared_collision::evaluate(*model,x,y,yaw,forecast);
  double v[]={value.physical,value.gap,value.cost,value.gx,value.gy,value.gyaw,model->sequence};
  std::copy(v,v+7,out);return value.valid?0:2;
}

// Caller owns a buffer of 12 doubles; see directed_reference::Cursor::debug.
extern "C" void online_directed_cursor_debug(void*p,double*out){
  if(!out)return;if(!p){std::fill(out,out+12,0.);return;}
  static_cast<Online*>(p)->cursor.debug(out);
}
