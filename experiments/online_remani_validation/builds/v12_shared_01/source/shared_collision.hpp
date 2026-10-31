#pragma once
// Fixed-arm SIL contract. Both the native planner and the MPPI library compile
// this exact implementation. Input is detector-labelled measurement, not truth.
#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>

namespace shared_collision {
struct Disk { double x,y,r; };
struct Obstacle { double x,y,vx,vy,r,moving; };
struct Snapshot {
  double sequence=0,stamp=0,gx=0,gy=0,res=.05,margin=.10,comfort=.12,weight=50000;
  double max_age=.35,max_prediction=3; bool healthy=false;
  int nx=0,ny=0;std::vector<Disk>body;std::vector<Obstacle>obstacles;std::vector<double>field;
};
inline std::shared_ptr<const Snapshot>& store(){static std::shared_ptr<const Snapshot>p;return p;}
inline bool& enabled(){static bool value=false;return value;}
inline std::shared_ptr<const Snapshot> snapshot(){return std::atomic_load(&store());}
inline bool bind(const double*p,int n){
  if(!p||n<16||n>2000000)return false;
  for(int i=0;i<n;++i)if(!std::isfinite(p[i]))return false;
  auto integer=[](double x,int lo,int hi){return x>=lo&&x<=hi&&std::floor(x)==x;};
  if(p[0]!=1||!integer(p[1],1,1e9)||p[2]<=0||!integer(p[3],2,2000)||!integer(p[4],2,2000)||
      p[7]<=0||p[8]<.10||p[9]<=p[8]||p[10]<=0||!integer(p[11],1,16)||
      !integer(p[12],0,16)||p[13]<=0||p[13]>.35||p[14]<1||p[14]>4||
      (p[15]!=0&&p[15]!=1))return false;
  const int nx=int(p[3]),ny=int(p[4]),nb=int(p[11]),no=int(p[12]);
  if(n!=16+3*nb+6*no+nx*ny)return false;
  auto value=std::make_shared<Snapshot>();value->sequence=p[1];value->stamp=p[2];
  value->nx=nx;value->ny=ny;value->gx=p[5];value->gy=p[6];value->res=p[7];
  value->margin=p[8];value->comfort=p[9];value->weight=p[10];
  value->max_age=p[13];value->max_prediction=p[14];value->healthy=p[15]>0;
  int i=16;
  for(int k=0;k<nb;++k,i+=3){if(p[i+2]<=0)return false;value->body.push_back({p[i],p[i+1],p[i+2]});}
  for(int k=0;k<no;++k,i+=6){if(p[i+4]<=0||(p[i+5]!=0&&p[i+5]!=1))return false;
    value->obstacles.push_back({p[i],p[i+1],p[i+2],p[i+3],p[i+4],p[i+5]});}
  value->field.assign(p+i,p+n);
  if(std::any_of(value->field.begin(),value->field.end(),[](double v){return v<0;}))return false;
  std::shared_ptr<const Snapshot> immutable=value;std::atomic_store(&store(),immutable);
  return true;
}
struct Distance {double d=0,dx=0,dy=0;};
inline Distance mapDistance(const Snapshot&m,double x,double y){
  double fx=(x-m.gx)/m.res,fy=(y-m.gy)/m.res;
  int ix=int(std::floor(fx)),iy=int(std::floor(fy));
  if(ix<0||iy<0||ix>=m.nx-1||iy>=m.ny-1)return {};
  double a=fx-ix,b=fy-iy;
  double q00=m.field[iy*m.nx+ix],q10=m.field[iy*m.nx+ix+1];
  double q01=m.field[(iy+1)*m.nx+ix],q11=m.field[(iy+1)*m.nx+ix+1];
  return {(1-b)*((1-a)*q00+a*q10)+b*((1-a)*q01+a*q11),
    ((1-b)*(q10-q00)+b*(q11-q01))/m.res,
    ((1-a)*(q01-q00)+a*(q11-q10))/m.res};
}
struct Evaluation {double physical=1e6,gap=1e6,cost=0,gx=0,gy=0,gyaw=0;bool valid=true;};
inline Evaluation evaluate(const Snapshot&m,double x,double y,double yaw,double forecast,
                           bool include_static=true,bool include_dynamic=true,
                           const Obstacle*override_obstacles=nullptr,int override_count=-1){
  Evaluation out;
  if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw)||forecast<-.05||
      (!m.obstacles.empty()&&forecast>m.max_prediction+1e-8)){
    out.valid=false;out.physical=out.gap=-1e6;out.cost=1e12;return out;
  }
  forecast=std::max(0.,forecast);double co=std::cos(yaw),si=std::sin(yaw);
  for(const auto&body:m.body){
    double px=x+co*body.x-si*body.y,py=y+si*body.x+co*body.y;
    Distance nearest=include_static?mapDistance(m,px,py):Distance{1e6,0,0};
    const Obstacle*obstacles=override_count>=0?override_obstacles:m.obstacles.data();
    const int count=override_count>=0?override_count:int(m.obstacles.size());
    if(include_dynamic)for(int i=0;i<count;++i){const auto&o=obstacles[i];
      double dx=px-o.x-forecast*o.vx*o.moving,dy=py-o.y-forecast*o.vy*o.moving;
      double distance=std::hypot(dx,dy),surface=distance-o.r;
      if(surface<nearest.d)nearest={surface,distance>1e-9?dx/distance:1.,distance>1e-9?dy/distance:0.};
    }
    double physical=nearest.d-body.r;
    out.physical=std::min(out.physical,physical);out.gap=std::min(out.gap,physical-m.margin);
    double error=std::max(0.,m.comfort-physical);
    out.cost+=m.weight*error*error*error;
    double factor=-3*m.weight*error*error;
    out.gx+=factor*nearest.dx;out.gy+=factor*nearest.dy;
    out.gyaw+=factor*(nearest.dx*(-si*body.x-co*body.y)+nearest.dy*(co*body.x-si*body.y));
  }return out;
}
inline bool fresh(const Snapshot&m,double now){return m.healthy&&now-m.stamp>=-.05&&now-m.stamp<=m.max_age;}
}
