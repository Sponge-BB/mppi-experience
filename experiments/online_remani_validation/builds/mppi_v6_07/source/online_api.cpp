#include "online_kernel.hpp"
using namespace experiment;
namespace {
const double* grid=nullptr; int nx=0,ny=0;double grid_x0=0,grid_y0=0,res=.05;
double distance(double x,double y){
  double fx=(x-grid_x0)/res,fy=(y-grid_y0)/res;int ix=std::floor(fx),iy=std::floor(fy);
  if(!grid||ix<0||iy<0||ix>=nx-1||iy>=ny-1)return 0.;
  double a=fx-ix,b=fy-iy;
  return (1-b)*((1-a)*grid[iy*nx+ix]+a*grid[iy*nx+ix+1])+b*((1-a)*grid[(iy+1)*nx+ix]+a*grid[(iy+1)*nx+ix+1]);
}
struct Online {const double* classified_static=nullptr;int static_width=0,static_height=0;C c;MPPI m;Online(const double*p,int seed):c{p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7],p[8],p[9],p[10],p[11],p[12],p[13],p[14],p[15]},m(c,seed){}};
}
extern "C" void* online_create(const double*p,const double*body,int n,int seed){
  try{auto*h=new Online(p,seed);for(int i=0;i<n;++i)h->c.body.push_back({{body[3*i],body[3*i+1]},body[3*i+2]});return h;}catch(...){return nullptr;}
}
extern "C" void online_destroy(void*ptr){delete static_cast<Online*>(ptr);}
extern "C" int online_step(void*ptr,const double*state,const double*refs,int nr,
    const double*obs,int no,double age,const double*field,int width,int height,
    double gx,double gy,double resolution,int cbf,double*out){
  try{
    if(!ptr||nr<2||age<-.01||age>.35||cbf)return 1;
    auto&h=*static_cast<Online*>(ptr);grid=(h.classified_static&&h.static_width==width&&h.static_height==height&&no>0)?h.classified_static:field;nx=width;ny=height;grid_x0=gx;grid_y0=gy;res=resolution;map_distance=distance;h.classified_static=nullptr; // Classified layer is one-step only; never silently reuse stale membership.
    S s{state[0],state[1],state[2],state[3],state[4]};Scene sc;sc.ymax=100.;sc.name="online";
    for(int i=0;i<nr;++i)sc.path.push_back({{refs[4*i],refs[4*i+1]},refs[4*i+2],refs[4*i+3]});
    Z z;z.t=std::max(0.,age);z.stamp=0;z.fresh=true;
    for(int i=0;i<no;++i)z.os.push_back({{obs[6*i],obs[6*i+1]},{obs[6*i+2],obs[6*i+3]},obs[6*i+4],obs[6*i+5]>.5});
    auto start=std::chrono::steady_clock::now();auto u=h.m.plan(s,z,sc);F f{u};if(cbf)f=filter(s,u,z,h.c,sc.ymax);
    out[0]=f.u.x;out[1]=f.u.y;out[2]=u.x;out[3]=u.y;out[4]=f.intervention;out[5]=f.qp_failed;out[6]=h.m.avoidance_debug[7];
    out[7]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();return 0;
  }catch(...){return 2;}
}
extern "C" void online_feedback(void*ptr,double v,double w){if(ptr)static_cast<Online*>(ptr)->m.feedback({v,w});}
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

extern "C" void online_reset(void*p){if(p)static_cast<Online*>(p)->m.reset();}
extern "C" void online_debug(void*p,double*out){if(p)std::copy(static_cast<Online*>(p)->m.debug.begin(),static_cast<Online*>(p)->m.debug.end(),out);}
extern "C" void online_status(void*p,double*out){if(p)std::copy(static_cast<Online*>(p)->m.status.begin(),static_cast<Online*>(p)->m.status.end(),out);}
extern "C" int online_local_route(void*p,double*out,int capacity){if(!p)return 0;auto&r=static_cast<Online*>(p)->m.published_route;int n=std::min(capacity,int(r.size()));for(int i=0;i<n;++i){out[2*i]=r[i].p.x;out[2*i+1]=r[i].p.y;}return n;}
extern "C" int online_revision(){return 6;}
extern "C" double online_pose_clearance(void*p,double x,double y,double yaw){return static_cast<Online*>(p)->m.poseDistance({x,y},yaw);}
extern "C" void online_set_classified_static(void*p,const double*field,int width,int height){if(p){auto&h=*static_cast<Online*>(p);h.classified_static=field;h.static_width=width;h.static_height=height;}}
extern "C" void online_avoidance_debug(void*p,double*out){if(p)std::copy(static_cast<Online*>(p)->m.avoidance_debug.begin(),static_cast<Online*>(p)->m.avoidance_debug.end(),out);}
