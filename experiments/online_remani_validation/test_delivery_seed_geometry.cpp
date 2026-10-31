// Offline extracted-native-function harness: no ROS, search or map is mocked.
// native_functions.hpp and native_fixture.hpp are generated with provenance.
#include <Eigen/Dense>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// Only the original error logger is stubbed; all supplied boundary speeds are
// <= max_vel_, so its branch is not exercised. No numerical expression changes.
#define ROS_ERROR(...) std::fprintf(stderr,__VA_ARGS__)
#include "native_functions.hpp"
#include "native_fixture.hpp"

std::string number(double value){
  if(!std::isfinite(value))return "null";
  std::ostringstream s;s<<std::setprecision(17)<<value;return s.str();
}
std::string pointsJson(const std::vector<Eigen::Vector3d>& points){
  std::string s="[";
  for(std::size_t i=0;i<points.size();++i){if(i)s+=',';s+='[';
    for(int j=0;j<3;++j){if(j)s+=',';s+=number(points[i][j]);}s+=']';}
  return s+"]";
}

void runCase(const std::string& name,const std::vector<Eigen::Vector3d>& path,double signed_speed){
  KinoAstar solver;configure(solver);
  solver.sample_trajs_=path;
  solver.start_state_<<path.front()[0],path.front()[1],path.front()[2],signed_speed;
  solver.end_state_<<path.back()[0],path.back()[1],path.back()[2],0.;
  solver.start_ctrl_.setZero();solver.totalTrajTime=-12345.;
  double euclidean=0.,weighted=0.;std::string weights="[";
  for(std::size_t i=1;i<path.size();++i){
    euclidean+=(path[i]-path[i-1]).head(2).norm();
    const double w=solver.evaluateCurvatureWeightedDistance(path[i-1].head(2),path[i].head(2),path[i-1][2],path[i][2]);
    weighted+=w;if(i>1)weights+=',';weights+=number(w);
  }
  weights+=']';
  const double duration=solver.evaluateDuration(weighted,std::abs(signed_speed),0.);
  const double half_arc=solver.evaluateLength(.5*duration,weighted,duration,std::abs(signed_speed),0.);
  solver.getTrajsWithTime();
  bool output_finite=true,times_positive=true;std::size_t count=0;
  for(const auto& block:solver.flat_trajs_){
    output_finite=output_finite&&block.start_state.allFinite()&&block.final_state.allFinite();
    for(const auto& p:block.traj_pts){++count;output_finite=output_finite&&p.allFinite();times_positive=times_positive&&p[2]>0.;}
    for(double theta:block.thetas)output_finite=output_finite&&std::isfinite(theta);
  }
  std::cout<<"{\"name\":\""<<name<<"\",\"scope\":\"constructed geometry only; not historical search-path replay\""
    <<",\"native_inf_macro_value\":"<<(inf)<<",\"signed_start_speed\":"<<number(signed_speed)
    <<",\"input_points\":"<<pointsJson(path)<<",\"euclidean_length\":"<<number(euclidean)
    <<",\"weighted_step_lengths\":"<<weights<<",\"weighted_length\":"<<number(weighted)
    <<",\"native_duration\":"<<number(duration)<<",\"native_half_time_arc\":"<<number(half_arc)
    <<",\"parameterization_early_return\":"<<(solver.totalTrajTime==-12345.?"true":"false")
    <<",\"flat_block_count\":"<<solver.flat_trajs_.size()<<",\"flat_sample_count\":"<<count
    <<",\"flat_outputs_finite\":"<<(output_finite?"true":"false")
    <<",\"flat_sample_times_positive\":"<<(times_positive?"true":"false")
    <<",\"native_gear_block_lengths\":[";
  for(std::size_t i=0;i<solver.shot_lengthList.size();++i){if(i)std::cout<<',';std::cout<<number(solver.shot_lengthList[i]);}
  std::cout<<"],\"native_gear_block_times\":[";
  for(std::size_t i=0;i<solver.shot_timeList.size();++i){if(i)std::cout<<',';std::cout<<number(solver.shot_timeList[i]);}
  std::cout<<"]}\n";
  if(name=="short_forward_origin"){
    assert(std::abs(weighted-euclidean)<1e-12);assert(solver.flat_trajs_.size()==1);assert(output_finite);
  }
}

std::vector<Eigen::Vector3d> straight(const Eigen::Vector3d& q,const std::vector<double>& offsets){
  std::vector<Eigen::Vector3d> out;
  for(double s:offsets)out.emplace_back(q[0]+s*std::cos(q[2]),q[1]+s*std::sin(q[2]),q[2]);
  return out;
}
int main(){
  assert((inf)==0);
  runCase("short_forward_origin",straight({0.,0.,0.},{0.,.01,.02}),.05);
  runCase("short_reverse_origin",straight({0.,0.,0.},{0.,-.01,-.02}),-.05);
  runCase("reverse_then_forward_origin",straight({0.,0.,0.},{0.,-.01,-.02,-.01,0.,.01}),-.05);
  runCase("short_reverse_translated_y",straight({0.,1.,0.},{0.,-.01,-.02}),-.05);
  runCapturedBoundaryConstructedCases();
  return 0;
}
