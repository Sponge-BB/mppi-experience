using namespace Eigen;
using namespace std;
#define inf 1 >> 30
  struct CarFlatTrajData{
    int singul;
    std::vector<Eigen::Vector3d> traj_pts; //某单个方向上均匀采样后的所有坐标点：x,y,t  有终点没有起点
    std::vector<double> thetas;    // 储存均匀采样后的yaw
    Eigen::MatrixXd start_state;   // start flat state pva
    Eigen::MatrixXd final_state;   // end flat state (2, 3)
  };

class KinoAstar {
public:
 double max_vel_=0.,max_acc_=0.,min_turning_radius_=0.,curvatureDisCoe_=0.;
 double non_siguav_=0.,mobile_base_wheel_base_=0.,sample_time_=0.,totalTrajTime=0.;
 Eigen::Vector4d start_state_,end_state_;Eigen::Vector2d start_ctrl_;
 std::vector<Eigen::Vector3d> sample_trajs_;
 std::vector<double> shot_lengthList,shot_timeList;
 std::vector<int> shotindex,shot_SList;
 std::vector<CarFlatTrajData> flat_trajs_;
 Eigen::Vector3d evaluatePos(const double&);
 void getTrajsWithTime();
 double evaluateCurvatureWeightedDistance(const Eigen::Vector2d&,const Eigen::Vector2d&,const double&,const double&);
 double evaluateDuration(const double&,const double&,const double&);
 double evaluateLength(const double&,const double&,const double&,const double&,const double&);
 void getFlatState(const Eigen::Vector4d&,const Eigen::Vector2d&,const int&,Eigen::MatrixXd&);
};
void KinoAstar::getTrajsWithTime(){
  if(seed_parameterization::enabled()) {

  // Transactional: no stale or partial seed may survive a failed parameterization.
  flat_trajs_.clear(); shot_lengthList.clear(); shot_timeList.clear();
  shotindex.clear(); shot_SList.clear(); totalTrajTime=0.;
  auto& seed_status=seed_parameterization::status();
  seed_status=seed_parameterization::Status();seed_status.applied=true;
  seed_status.input_points=sample_trajs_.size();seed_status.reason="invalid_geometry_or_time_profile";
  if(start_state_.size()<4||end_state_.size()<4||!start_state_.allFinite()||!end_state_.allFinite()) return;
  seed_status.signed_start_speed=start_state_(3);
  const auto result=seed_parameterization::build<CarFlatTrajData>(sample_trajs_,
    std::abs(start_state_(3)),std::abs(end_state_(3)),start_ctrl_,non_siguav_,sample_time_,
    max_vel_,max_acc_,min_turning_radius_,curvatureDisCoe_,
    [this](const Eigen::Vector4d& state,const Eigen::Vector2d& control,int gear,Eigen::MatrixXd& flat){
      getFlatState(state,control,gear,flat);
    });
  if(!result.valid) return;
  if(start_state_(3)!=0. && result.gears.front()*(start_state_(3))<0.) {
    seed_status.reason="initial_gear_conflict";return;
  }
  if(end_state_(3)!=0. && result.gears.back()*(end_state_(3))<0.) {
    seed_status.reason="final_gear_conflict";return;
  }
  sample_trajs_=result.path; shot_lengthList=result.lengths; shot_timeList=result.times;
  shotindex=result.indices; shot_SList=result.gears; flat_trajs_=result.flats;
  totalTrajTime=result.total;
  seed_status.valid=true;seed_status.reason="ok";seed_status.output_segments=result.flats.size();
  seed_status.first_gear=result.gears.front();
  return;

  }

  double startvel = fabs(start_state_(3));
  double endvel = fabs(end_state_(3));
  std::vector<Eigen::Vector3d> traj_pts;
  std::vector<double> thetas;
  shot_lengthList.clear();
  shot_timeList.clear();
  shotindex.clear();
  shot_SList.clear();
  double tmpl = 0;
  int lastS = (sample_trajs_[1] - sample_trajs_[0]).head(2).dot(Eigen::Vector2d(cos(sample_trajs_[0][2]), sin(sample_trajs_[0][2]))) >= 0 ? 1 : -1;
  shotindex.push_back(0);

  for(unsigned int i = 0; i < sample_trajs_.size() - 1; i++){
    Eigen::Vector3d state1 = sample_trajs_[i];
    Eigen::Vector3d state2 = sample_trajs_[i + 1];
    int curS;
    if((state2 - state1).head(2).norm() < 1e-4){
      curS = lastS;
    }
    else{
      curS = (state2 - state1).head(2).dot(Eigen::Vector2d(cos(state1[2]), sin(state1[2]))) >=0 ? 1 : -1;
    }
    if(curS * lastS >= 0){
      tmpl += evaluateCurvatureWeightedDistance(state1.head(2), state2.head(2), state1[2], state2[2]); // change
    }else{
      shotindex.push_back(i);
      shot_SList.push_back(lastS);
      shot_lengthList.push_back(tmpl);
      if(tmpl == inf) return;
      shot_timeList.push_back(evaluateDuration(tmpl, non_siguav_, non_siguav_));
      tmpl = evaluateCurvatureWeightedDistance(state1.head(2), state2.head(2), state1[2], state2[2]);
    }
    lastS = curS;
  }
  shot_SList.push_back(lastS);
  shot_lengthList.push_back(tmpl);
  shot_timeList.push_back(evaluateDuration(tmpl, non_siguav_, non_siguav_));
  shotindex.push_back(sample_trajs_.size() - 1);

  if((int)shot_timeList.size() >= 2){
    shot_timeList[0] = evaluateDuration(shot_lengthList[0], startvel, non_siguav_);
    shot_timeList[shot_timeList.size() - 1] = evaluateDuration(shot_lengthList.back(), non_siguav_, endvel);
  }
  else{
    shot_timeList[0] = evaluateDuration(shot_lengthList[0], startvel, endvel);
  }
  // 
  for(unsigned int i = 0; i < shot_lengthList.size(); i++){
    double initv = non_siguav_, finv = non_siguav_;
    Eigen::Vector2d Initctrlinput, Finctrlinput;
    Initctrlinput<<0,0;Finctrlinput<<0,0;
    if(i == 0){
      initv = startvel; 
      Initctrlinput = start_ctrl_;
    }
    if(i == shot_lengthList.size() - 1)
      finv = endvel;

    double locallength = shot_lengthList[i];
    int sig = shot_SList[i];
    std::vector<Eigen::Vector3d> localTraj;
    localTraj.assign(sample_trajs_.begin() + shotindex[i], sample_trajs_.begin() + shotindex[i + 1] + 1);
    traj_pts.clear();
    thetas.clear();        
    double samplet;
    double tmparc = 0;
    int index = 0;
    double sample_time = sample_time_;
    if(shot_timeList[i] <= sample_time){
      sample_time = shot_timeList[i] / 2.0;
    }
    for(samplet = sample_time; samplet < shot_timeList[i]; samplet += sample_time){
      double arc = evaluateLength(samplet, locallength, shot_timeList[i], initv, finv);
      for(unsigned int k = index; k < localTraj.size() - 1; k++){
        tmparc += evaluateCurvatureWeightedDistance(localTraj[k].head(2),localTraj[k+1].head(2),localTraj[k][2],localTraj[k+1][2]); //change
        if(tmparc>=arc){
          index = k;
          double l1 = tmparc-arc;
          // double l = (localTraj[k+1]-localTraj[k]).head(2).norm();
          double l = evaluateCurvatureWeightedDistance(localTraj[k].head(2),localTraj[k+1].head(2),localTraj[k][2],localTraj[k+1][2]); //change
          
          double l2 = l-l1;//l2
          double px = (l1/l*localTraj[k]+l2/l*localTraj[k+1])[0]; // FIXME l == 0
          double py = (l1/l*localTraj[k]+l2/l*localTraj[k+1])[1];
          double yaw= (l1/l*localTraj[k]+l2/l*localTraj[k+1])[2];

          if(fabs(localTraj[k + 1][2] - localTraj[k][2]) >= M_PI){   
            double normalize_yaw = 0;
            if(localTraj[k + 1][2] <= 0){
              normalize_yaw = l1 / l * localTraj[k][2] + l2 / l * (localTraj[k + 1][2] + 2 * M_PI);
            }else if(localTraj[k][2] <=0){
              normalize_yaw = l1 / l * (localTraj[k][2] + 2 * M_PI) + l2 / l * localTraj[k + 1][2];
            }
            yaw = normalize_yaw;
          }

          traj_pts.emplace_back(px, py, sample_time);
          thetas.push_back(yaw);
          // tmparc -=(localTraj[k+1]-localTraj[k]).head(2).norm();
          tmparc -= evaluateCurvatureWeightedDistance(localTraj[k].head(2),localTraj[k+1].head(2),localTraj[k][2],localTraj[k+1][2]); //change
          break;
        }
      }
    }
    traj_pts.emplace_back(localTraj.back()[0], localTraj.back()[1], shot_timeList[i] - (samplet - sample_time_));
    thetas.push_back(localTraj.back()[2]);
    // flat_trajs_
    Eigen::MatrixXd startS;
    Eigen::MatrixXd endS;
    getFlatState(Eigen::Vector4d(localTraj.front()[0],localTraj.front()[1],localTraj.front()[2], initv), Initctrlinput, sig, startS);
    getFlatState(Eigen::Vector4d(localTraj.back()[0],localTraj.back()[1],localTraj.back()[2], finv), Finctrlinput, sig, endS);
    
    CarFlatTrajData flat_traj;
    flat_traj.traj_pts = traj_pts;
    flat_traj.thetas = thetas;
    flat_traj.start_state = startS;
    flat_traj.final_state = endS;
    flat_traj.singul = sig;
    flat_trajs_.push_back(flat_traj);
  }
  totalTrajTime = 0;
  for(unsigned int i = 0; i < shot_timeList.size(); i++){
    totalTrajTime += shot_timeList[i];
  }
}

double KinoAstar::evaluateCurvatureWeightedDistance(const Eigen::Vector2d &state1, const Eigen::Vector2d &state2, const double &yaw1, const double &yaw2){
  if(seed_parameterization::enabled()) {

  return seed_parameterization::distance(state1,state2,yaw1,yaw2,min_turning_radius_,curvatureDisCoe_);

  }

  Eigen::Vector2d diff = state2 - state1;
  double EuDis = diff.norm();
  double siny = sin(yaw1), cosy = cos(yaw1);
  if( (Eigen::Vector2d(state1 + EuDis*Eigen::Vector2d(cos(yaw1),sin(yaw1)))-state2).norm()/EuDis < 0.01 ){
    return EuDis;
  }
  
  Eigen::Matrix2d A;
  Eigen::Vector2d b;

  A << cosy,siny,2*diff[0],2*diff[1];
  b << state1[0]*cosy+state1[1]*siny,diff.dot(state1+state2);
  b = A.colPivHouseholderQr().solve(b);
  double radius = (b-state1).norm();
  // std::cout<<"state1:"<<state1.transpose()<<"  "<<yaw1<<"  state2:"<<state2.transpose()<<"  "<<yaw2<<"   radius"<<radius<<std::endl;
  if(radius < min_turning_radius_){
    // std::cout<<"radius<min_turning_radius:"<<curvatureDisCoe_ * std::min(std::min(fabs(yaw2-yaw1),fabs(yaw2-yaw1+M_PI*2)),fabs(yaw2-yaw1-M_PI*2))<<std::endl;
    return curvatureDisCoe_ * std::min(std::min(fabs(yaw2-yaw1),fabs(yaw2-yaw1+M_PI*2)),fabs(yaw2-yaw1-M_PI*2));
  }
  // std::cout<<"noline EuDis:"<<EuDis<<std::endl;
  return EuDis;
}

double KinoAstar::evaluateDuration(const double &length, const double &startV, const double &endV){
  if(seed_parameterization::enabled()) {

  return seed_parameterization::duration(length,startV,endV,max_vel_,max_acc_);

  }

  double critical_len; 
  if(startV > max_vel_ || endV > max_vel_){
    ROS_ERROR("kinoAstar:evaluateDuration:start or end vel is larger that the limit!");
  }
  double temp_max_acc = max_acc_ * 0.7;
  double temp_max_vel = max_vel_ * 1.0;
  double startv2 = pow(startV, 2);
  double endv2   = pow(endV, 2);
  double maxv2   = pow(temp_max_vel, 2);
  critical_len = (maxv2 - startv2) /(2 * temp_max_acc) + (maxv2 - endv2) / (2 * temp_max_acc);
  if(length >= critical_len){
    return (temp_max_vel - startV) / temp_max_acc + (temp_max_vel - endV) / temp_max_acc + (length - critical_len) / temp_max_vel;
  }
  else{
    double tmpv = sqrt(0.5 * (startv2 + endv2 + 2 * temp_max_acc * length));
    return (tmpv - startV) / temp_max_acc + (tmpv - endV) / temp_max_acc;
  }
}

double KinoAstar::evaluateLength(const double &curt, const double &locallength, const double &localtime, const double &startV, const double &endV){
  if(seed_parameterization::enabled()) {

  return seed_parameterization::lengthAt(curt,locallength,localtime,startV,endV,max_vel_,max_acc_);

  }

  double critical_len; 
  if(startV > max_vel_ || endV > max_vel_){
    // std::cout << startV << ", " << endV << ", " << max_vel_ << std::endl;
    // ROS_ERROR("kinoAstar:evaluateLength:start or end vel is larger that the limit!");
  }
  double startv2 = pow(startV, 2);
  double endv2 = pow(endV, 2);
  double maxv2 = pow(max_vel_, 2);
  critical_len = (maxv2 - startv2) / (2 * max_acc_) + (maxv2 - endv2) / (2 * max_acc_);
  if(locallength >= critical_len){// 如果locallength距离比critical_len大，则为加速到到最大速度再减速
    double t1 = (max_vel_-startV)/max_acc_;
    double t2 = t1+(locallength-critical_len)/max_vel_;
    if(curt<=t1){
      return startV*curt + 0.5*max_acc_*pow(curt,2);
    }else if(curt<=t2){
      return startV*t1 + 0.5*max_acc_*pow(t1,2)+(curt-t1)*max_vel_;
    }else{
      return startV*t1 + 0.5*max_acc_*pow(t1,2) + (t2-t1)*max_vel_ + max_vel_*(curt-t2)-0.5*max_acc_*pow(curt-t2,2);
    }
  }else{
    double tmpv = sqrt(0.5*(startv2+endv2+2*max_acc_*locallength));
    double tmpt = (tmpv-startV)/max_acc_;
    if(curt<=tmpt){
      return startV*curt+0.5*max_acc_*pow(curt,2);
    }
    else{
      return startV*tmpt+0.5*max_acc_*pow(tmpt,2) + tmpv*(curt-tmpt)-0.5*max_acc_*pow(curt-tmpt,2);
    }
  }
}

void KinoAstar::getFlatState(const Eigen::Vector4d &state, const Eigen::Vector2d &control_input,const int &singul, 
                             Eigen::MatrixXd &flat_state){

    flat_state.resize(2, 3);

    double angle = state(2);
    double vel   = state(3); // vel > 0 

    Eigen::Matrix2d init_R;
    init_R << cos(angle),  -sin(angle),
              sin(angle),   cos(angle);

    if (abs(vel) <= non_siguav_){
      vel = singul * non_siguav_;
    }
    else{
      vel = singul * vel;
    }
    flat_state << state.head(2), init_R * Eigen::Vector2d(vel, 0.0),
                  init_R * Eigen::Vector2d(control_input(1), std::tan(control_input(0)) / mobile_base_wheel_base_ * std::pow(vel, 2));
}

Eigen::Vector3d KinoAstar::evaluatePos(const double &input_t){
  if(seed_parameterization::enabled()) {

  if(start_state_.size()<4||end_state_.size()<4) return Eigen::Vector3d::Constant(seed_parameterization::invalid());
  return seed_parameterization::position(sample_trajs_,shot_lengthList,shot_timeList,shotindex,
    input_t,std::abs(start_state_(3)),std::abs(end_state_(3)),non_siguav_,max_vel_,max_acc_,
    min_turning_radius_,curvatureDisCoe_);

  }

  double t = input_t;
  if(t < 0 || t > totalTrajTime){
    // ROS_ERROR("In evaluatePos, t<0 || t>totalTrajTime");
    // std::cout << "t: " << t << ", totol: " << totalTrajTime << std::endl;
    t = std::min<double>(std::max<double>(0, t), totalTrajTime);
  }
  double startvel = fabs(start_state_(3));
  double endvel = fabs(end_state_(3));
  int index = -1;
  double tmpT = 0;
  double CutTime;
  for(unsigned int i = 0; i < shot_timeList.size(); i++){
    tmpT += shot_timeList[i];
    if(tmpT >= t) {
      index = i; 
      CutTime = t - tmpT + shot_timeList[i];
      break;
    }
  }

  double initv = non_siguav_, finv = non_siguav_;
  if(index==0) {initv = startvel;}
  if(index == (int)shot_lengthList.size() - 1) finv = endvel;
  double localtime = shot_timeList[index];
  double locallength = shot_lengthList[index];
  int front = shotindex[index];
  int back  = shotindex[index+1];
  std::vector<Eigen::Vector3d> localTraj;
  localTraj.assign(sample_trajs_.begin() + front, sample_trajs_.begin() + back + 1);
  double arclength = evaluateLength(CutTime, locallength, localtime, initv, finv);
  double tmparc = 0;
  for(unsigned int i = 0; i < localTraj.size() - 1; i++){
    // tmparc += (localTraj[i+1]-localTraj[i]).head(2).norm();// change
    tmparc += evaluateCurvatureWeightedDistance(localTraj[i].head(2), localTraj[i + 1].head(2), localTraj[i][2], localTraj[i + 1][2]); //change
    if(tmparc >= arclength){
      double l1 = tmparc - arclength;
      // double l = (localTraj[i+1]-localTraj[i]).head(2).norm();// change
      // FIXME l = 0
      double l = evaluateCurvatureWeightedDistance(localTraj[i].head(2), localTraj[i + 1].head(2), localTraj[i][2], localTraj[i + 1][2]); //change
      if(l < 1e-3) return localTraj[i];
      double l2 = l - l1;//l2
      Eigen::Vector3d state = l1 / l * localTraj[i] + l2 / l * localTraj[i + 1];
      if(fabs(localTraj[i + 1][2]-localTraj[i][2]) >= M_PI){   
        double normalize_yaw = 0;
        if(localTraj[i + 1][2] <= 0){
          normalize_yaw = l1 / l * localTraj[i][2] + l2 / l * (localTraj[i + 1][2] + 2 * M_PI);
        }else if(localTraj[i][2] <= 0){
          normalize_yaw = l1 / l * (localTraj[i][2] + 2 * M_PI) + l2 / l * localTraj[i + 1][2];
        }
        state[2] = normalize_yaw;
      }
      return state;
    }
  }
  return localTraj.back();
}
