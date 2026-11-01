#include "shared_collision.hpp"
extern "C" int bind_native(const double*p,int n){return shared_collision::bind(p,n)?0:1;}
extern "C" int pose_native(double x,double y,double yaw,double t,double*out){
 auto m=shared_collision::snapshot();if(!m)return 1;
 auto v=shared_collision::evaluate(*m,x,y,yaw,t);
 double a[]={v.physical,v.gap,v.cost,v.gx,v.gy,v.gyaw,m->sequence};
 std::copy(a,a+7,out);return v.valid?0:2;}
