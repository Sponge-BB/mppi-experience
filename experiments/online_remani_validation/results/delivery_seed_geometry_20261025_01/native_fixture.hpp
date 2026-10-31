void runCase(const std::string&,const std::vector<Eigen::Vector3d>&,double);
std::vector<Eigen::Vector3d> straight(const Eigen::Vector3d&,const std::vector<double>&);
void configure(KinoAstar& s){
s.max_vel_=0.2296;
s.max_acc_=0.29750000000000004;
s.non_siguav_=0.050000000000000003;
s.mobile_base_wheel_base_=0.37;
s.min_turning_radius_=0.20000000000000001;
s.curvatureDisCoe_=0.40000000000000002;
s.sample_time_=0.10000000000000001;
}
void runCapturedBoundaryConstructedCases(){
runCase("constructed_reverse_forward_at_seed11_request3_boundary",straight({0.87958756065929111,-0.021030543371894653,0.19108487075299016},{0.,-.01,-.02,-.01,0.,.01}),-0.05000000000000001);
runCase("constructed_reverse_forward_at_seed12_request3_boundary",straight({0.88049678848714041,-0.00017018985570234249,0.14723121264732733},{0.,-.01,-.02,-.01,0.,.01}),-0.05000000000000001);
}
