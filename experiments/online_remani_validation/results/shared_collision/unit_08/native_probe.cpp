#include "shared_collision.hpp"
extern "C" int bind_native(const double*p,int n){return shared_collision::bind(p,n)?0:1;}
extern "C" int pose_native(double x,double y,double yaw,double t,double*out){
  auto model=shared_collision::snapshot();if(!model)return 1;
  auto value=shared_collision::evaluate(*model,x,y,yaw,t);
  double values[]={value.physical,value.gap,value.cost,value.gx,value.gy,value.gyaw,model->sequence};
  std::copy(values,values+7,out);return value.valid?0:2;
}
