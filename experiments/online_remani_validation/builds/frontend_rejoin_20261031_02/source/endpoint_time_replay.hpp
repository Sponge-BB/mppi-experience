#pragma once
// Diagnostic seed transformation plus a default-off private-child front-end gate.
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace endpoint_time {
inline bool requested(){const char* value=std::getenv("ENDPOINT_TIME_REPLAY");return value&&std::strcmp(value,"1")==0;}
inline bool fixRequested(){const char* value=std::getenv("ENDPOINT_TIME_FIX");return value&&std::strcmp(value,"1")==0;}
inline std::vector<Eigen::VectorXd> factors(const std::vector<Eigen::VectorXd>& input,bool enabled){
  std::vector<Eigen::VectorXd> result;
  for(const auto& times:input){
    if(times.size()<1||!times.allFinite()||(times.array()<=0.).any())
      throw std::runtime_error("endpoint-time requires finite positive nonempty segment durations");
    result.push_back(Eigen::VectorXd::Ones(times.size()));
    if(enabled){result.back()[0]=1.5;result.back()[times.size()-1]=1.5;}
  }
  if(result.empty())throw std::runtime_error("endpoint-time requires at least one gear segment");
  return result;
}
inline std::vector<Eigen::VectorXd> initialTimes(const std::vector<Eigen::VectorXd>& input,bool enabled){
  const auto scales=factors(input,enabled);auto result=input;
  for(std::size_t n=0;n<result.size();++n)result[n].array()*=scales[n].array();
  return result;
}
template<class Result> bool endpointCoordinates(const Result& original,const Result& changed){
  if(original.dim!=changed.dim||original.pieces!=changed.pieces||original.first_x.size()==0||
     original.first_x.size()!=changed.first_x.size())return false;
  int time_begin=0,time_count=0;
  for(int count:original.pieces){if(count<1)return false;time_begin+=original.dim*(count-1);time_count+=count;}
  for(int k=0;k<original.first_x.size();++k)if(k<time_begin||k>=time_begin+time_count)
    if(original.first_x[k]!=changed.first_x[k])return false;
  int at=time_begin;
  const auto real=[](double v){return v>0.?((.5*v+1.)*v+1.):1./((.5*v-1.)*v+1.);};
  for(int count:original.pieces)for(int k=0;k<count;++k,++at){
    const double a=real(original.first_x[at]),b=real(changed.first_x[at]);
    const double scale=(k==0||k+1==count)?1.5:1.;
    if(!std::isfinite(a)||!std::isfinite(b)||std::abs(b-scale*a)>1e-12*std::max(1.,scale*a))return false;
  }
  return true;
}
inline void emit(const std::string& path,const std::string& body){
  std::ofstream out(path,std::ios::app);if(!out)throw std::runtime_error("cannot open endpoint-time log");
  out<<"{\"schema\":\"endpoint_time_replay_v1\","<<body<<"}\n";
  if(!out)throw std::runtime_error("cannot write endpoint-time log");
}
} // namespace endpoint_time
