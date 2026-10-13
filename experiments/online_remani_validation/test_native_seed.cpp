#include <path_searching/sample_mani_RRT.h>
#include <iostream>

// Exercise the actual native translation unit, without a ROS master, map or
// initialized robot model: invalid seeds must be rejected BEFORE using them.
int main(){
  ros::Time::init();
  mani_sample::SampleMani sampler;
  Eigen::VectorXd arm=Eigen::VectorXd::Zero(6);
  int checks=0;
  for(int n: {0,1,3}){
    std::vector<Eigen::Vector3d> base(n,Eigen::Vector3d::Zero()),dense;
    std::vector<double> times;
    std::vector<int> directions;
    std::vector<std::vector<Eigen::VectorXd>> paths(1);
    std::vector<int> output_directions(1);
    std::vector<std::vector<double>> yaws(1);
    std::vector<Eigen::VectorXd> durations(1);
    if(sampler.sampleManiSearch(true,arm,arm,base,dense,times,directions,1,
        paths,output_directions,yaws,durations))return 1;
    if(!paths.empty()||!output_directions.empty()||!yaws.empty()||!durations.empty())return 2;
    ++checks;
  }
  for(int n: {0,1}){
    sampler.init(std::vector<Eigen::Vector3d>(n,Eigen::Vector3d::Zero()),{},{});
    if(sampler.search(arm,arm))return 3;
    ++checks;
  }
  std::cout<<"{\"passed\": true, \"native_invalid_seed_checks\": "<<checks<<"}"<<std::endl;
}
