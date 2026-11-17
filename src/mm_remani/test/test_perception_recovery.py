"""Execute extracted production recovery methods; no ROS master or devices.

Only transport, map, solver and collision-query boundaries are stubbed. The
recovery state machine, source-age checks, stop-owner clamp, success arbitration
and trajectory timestamp rebasing are the current C++ production method bodies.
"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / 'src/mm_remani_planner/plan_manage/src/remani_replan_fsm.cpp'


def function(text, signature):
    """Keep the complete real definition, counting balanced C++ braces."""
    start = text.index(signature)
    opening = text.index('{', start)
    depth = 1
    cursor = opening + 1
    while depth:
        depth += (text[cursor] == '{') - (text[cursor] == '}')
        cursor += 1
    return text[start:cursor]


PREAMBLE = r'''
#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#define ROS_INFO(...) ((void)0)
#define ROS_WARN_THROTTLE(...) ((void)0)
namespace ros {
double wall_now = 100., steady_now = 100.;
struct Time {
  double value;
  explicit Time(double v=0.):value(v){}
  static Time now(){return Time(wall_now);}
  double toSec() const{return value;}
  bool isZero() const{return value == 0.;}
};
struct WallDuration {double value; explicit WallDuration(double v):value(v){}};
struct SteadyTime {
  double value;
  explicit SteadyTime(double v=0.):value(v){}
  static SteadyTime now(){return SteadyTime(steady_now);}
  bool operator<(const SteadyTime& other) const{return value<other.value;}
  SteadyTime operator+(const WallDuration& duration) const{
    return SteadyTime(value+duration.value);
  }
};
}
namespace std_msgs {struct Float64 {double data=0.;};}
struct Publisher {
  std::vector<double> values;
  void publish(const std_msgs::Float64& msg){values.push_back(msg.data);}
};
struct Segment {double start_time=0.,end_time=0.,duration=1.;};
struct SingulTrajData {
  double start_time=0.,duration=3.;
  ros::Time start_time_ros;
  Eigen::VectorXd first=Eigen::VectorXd::Zero(8);
  std::vector<Segment> singul_traj{{0.,1.,1.},{1.,3.,2.}};
  Eigen::VectorXd getPos(double) const{return first;}
};
struct Map {
  bool timeout=false,ready=true;
  int refreshes=0;
  std::uint64_t epoch=1,esdf=31,observation=31;
  bool getOdomDepthTimeout() const{return timeout;}
  bool mapReadyForPlanning() const{return ready && !timeout;}
  void refreshReadSnapshot(){++refreshes;}
  std::pair<std::uint64_t,std::uint64_t> getReadESDFVersion() const{
    return {epoch,esdf};
  }
  std::uint64_t getESDFObservationSeq() const{return esdf;}
  std::uint64_t getDynamicObservationSeq() const{return observation;}
};
struct Planner {
  std::shared_ptr<Map> grid_map_=std::make_shared<Map>();
  struct {SingulTrajData singul_traj_data;} traj_container_;
};
enum State {GEN_NEW_TRAJ,EXEC_TRAJ,EMERGENCY_STOP};
class REMANIReplanFSM {
public:
  bool perception_recovery_pending_=true;
  bool perception_recovery_candidate_ready_=false;
  bool transient_obstacle_recovery_=false;
  std::pair<std::uint64_t,std::uint64_t> recovery_last_attempt_map_{0,0};
  std::uint64_t accepted_goal_revision_=0,recovery_candidate_goal_revision_=0;
  bool goal_feedback_recheck_pending_=false;
  ros::Time goal_yaw_alignment_start_time_;
  bool enable_fail_safe_=true,have_target_=true,have_odom_=true;
  bool have_local_traj_=false,try_plan_after_emergency_=false;
  bool tracking_safety_stop_active_=false,tracking_fault_active_=false;
  bool tracking_recovery_candidate_published_=false,tracking_recovery_pending_=false;
  bool dynamic_safety_speed_limit_active_=false,dynamic_point_flow_active_mode_=true;
  bool dynamic_point_flow_hazard_active_=false,flag_escape_emergency_=false;
  std::uint64_t dynamic_event_required_esdf_seq_=0;
  int traj_dim_=8,mobile_base_dim_=2,manipulator_dim_=6;
  int rear_goal_plan_trial_times_=3,global_plan_trial_times_=5;
  int new_traj_fail_time_=4;
  double mm_car_yaw_=0.,mm_car_yaw_rate_=0.,start_yaw_=0.;
  double replan_feedback_position_threshold_=.08;
  double replan_feedback_arm_threshold_=.10,replan_feedback_yaw_threshold_=.15;
  double current_safety_speed_limit_=std::numeric_limits<double>::quiet_NaN();
  Eigen::VectorXd mm_state_pos_=Eigen::VectorXd::Zero(8);
  Eigen::VectorXd mm_state_vel_=Eigen::VectorXd::Zero(8);
  ros::Time perception_odom_stamp_{100.},perception_joint_stamp_{100.};
  ros::SteadyTime perception_retry_after_;
  Publisher safety_speed_limit_pub_;
  std::shared_ptr<Planner> planner_manager_=std::make_shared<Planner>();
  State state=EMERGENCY_STOP;
  bool esdf_ready=true,solve_ok=true,rear=false;
  bool static_clear=true,dynamic_clear=true,predicted_clear=true;
  int solves=0,validations=0,deliveries=0,esdf_waits=0,dynamic_clears=0;
  int goal_id=17,solved_goal=-1,delivered_goal=-1,trial_count=0;
  double solve_elapsed=.35,validate_elapsed=0.;
  std::function<void()> on_validate;
  std::uint64_t sent_validation=0,sent_prediction=0;
  bool hasUnmetRequiredESDF() const{return !esdf_ready;}
  bool waitForRequiredESDF(const char*,bool){++esdf_waits;return esdf_ready;}
  bool isRearDistantGoal() const{return rear;}
  void changeFSMExecState(State next,const char*){state=next;}
  bool planFromGlobalTraj(int trials){
    ++solves;trial_count=trials;
    ros::wall_now+=solve_elapsed;ros::steady_now+=solve_elapsed;
    if(!solve_ok)return false;
    auto& candidate=planner_manager_->traj_container_.singul_traj_data;
    candidate=SingulTrajData();candidate.first=mm_state_pos_;
    candidate.start_time=ros::wall_now;candidate.start_time_ros=ros::Time::now();
    start_yaw_=mm_car_yaw_;solved_goal=goal_id;
    perception_recovery_candidate_ready_=true;
    return true;
  }
  bool validateTrajectoryForPublication(SingulTrajData&,std::uint64_t& raw,
      std::uint64_t& predicted){
    ++validations;ros::wall_now+=validate_elapsed;ros::steady_now+=validate_elapsed;
    if(on_validate)on_validate();
    raw=31;predicted=41;
    return static_clear && dynamic_clear && predicted_clear;
  }
  void sendPolyTrajROSMsg(std::uint64_t raw,std::uint64_t predicted){
    ++deliveries;delivered_goal=solved_goal;have_local_traj_=true;
    sent_validation=raw;sent_prediction=predicted;
    // These checks happen exactly at the transport boundary.
    if(!perception_recovery_pending_ || !perception_recovery_candidate_ready_)
      throw std::runtime_error("recovery owner released before candidate delivery");
    if(!perceptionRecoveryReady())
      throw std::runtime_error("stale candidate reached publisher");
    if(current_safety_speed_limit_!=0.)
      throw std::runtime_error("moving while delivering recovery candidate");
  }
  void clearDynamicSafetySpeedLimit(const ros::Time&,bool){
    ++dynamic_clears;
    if(!esdf_ready)return;
    dynamic_safety_speed_limit_active_=false;
    publishSafetySpeedLimit(tracking_safety_stop_active_?0.:-1.);
  }
  bool perceptionRecoveryReady() const;
  void beginTransientObstacleRecovery();
  void recoverPerceptionStop();
  void publishSafetySpeedLimit(double);
  void finishNewTrajectory();
};
'''


CASES = r'''
void require(bool condition,const char* detail){
  if(!condition)throw std::runtime_error(detail);
}
void refresh(REMANIReplanFSM& f){
  f.perception_odom_stamp_=ros::Time::now();
  f.perception_joint_stamp_=ros::Time::now();
}
void tick(double seconds){ros::wall_now+=seconds;ros::steady_now+=seconds;}
void staged(REMANIReplanFSM& f){
  f.recoverPerceptionStop();
  require(f.solves==1 && f.deliveries==0,"first phase must solve without publishing");
  require(f.perception_recovery_pending_ && f.perception_recovery_candidate_ready_,
      "first phase lost recovery ownership/candidate");
  require(f.have_target_ && f.new_traj_fail_time_==4,"staging consumed goal/failure budget");
}
void run(const std::string& name){
  REMANIReplanFSM f;
  auto map=f.planner_manager_->grid_map_;
  if(name=="two_phase"){
    staged(f);
    require(!f.perceptionRecoveryReady(),"350ms solve did not expire source feedback");
    f.recoverPerceptionStop();
    require(f.solves==1 && f.deliveries==0,"stale feedback caused publish/reoptimize");
    refresh(f);f.recoverPerceptionStop();
    require(f.deliveries==1 && f.solves==1,"fresh callback did not deliver staged candidate");
    require(f.delivered_goal==17 && f.have_target_,"recovery changed accepted goal");
    require(!f.perception_recovery_pending_ && !f.perception_recovery_candidate_ready_,
        "successful publication failed to release own stop");
    require(f.state==EXEC_TRAJ && f.current_safety_speed_limit_==-1.,
        "published recovery did not enable reference execution");
    require(f.sent_validation==31 && f.sent_prediction==41,"lost validation generations");
  }else if(name=="source_validity"){
    require(f.perceptionRecoveryReady(),"fresh valid input rejected");
    for(double age:{.1501,1.}){
      refresh(f);f.perception_odom_stamp_=ros::Time(ros::wall_now-age);
      require(!f.perceptionRecoveryReady(),"stale odometry accepted");
      refresh(f);f.perception_joint_stamp_=ros::Time(ros::wall_now-age);
      require(!f.perceptionRecoveryReady(),"stale joints accepted");
    }
    for(double future:{.0049,.0051}){
      refresh(f);f.perception_joint_stamp_=ros::Time(ros::wall_now+future);
      require(f.perceptionRecoveryReady()==(future<.005),"future tolerance incorrect");
    }
    refresh(f);f.perception_joint_stamp_=ros::Time();
    require(!f.perceptionRecoveryReady(),"zero joint source accepted");
    refresh(f);f.perception_odom_stamp_=ros::Time();
    require(!f.perceptionRecoveryReady(),"zero odom source accepted");
    refresh(f);f.have_odom_=false;
    require(!f.perceptionRecoveryReady(),"no odometry accepted");
    f.have_odom_=true;f.mm_state_pos_(3)=std::numeric_limits<double>::quiet_NaN();
    require(!f.perceptionRecoveryReady(),"nonfinite state accepted");
    f.mm_state_pos_.setZero();f.mm_state_vel_(4)=std::numeric_limits<double>::infinity();
    require(!f.perceptionRecoveryReady(),"nonfinite velocity accepted");
    f.mm_state_vel_.setZero();map->ready=false;
    require(!f.perceptionRecoveryReady(),"map not ready accepted");
  }else if(name=="sustained_loss"){
    f.perception_recovery_candidate_ready_=true;map->timeout=true;
    for(int n=0;n<8;++n){refresh(f);f.recoverPerceptionStop();tick(.2);}
    require(f.solves==0 && f.deliveries==0,"continued sensor loss invoked solve/publish");
    require(!f.perception_recovery_candidate_ready_ && f.perception_recovery_pending_,
        "sensor loss did not invalidate staged candidate");
    require(f.have_target_ && f.enable_fail_safe_ && f.new_traj_fail_time_==4,
        "sensor loss mutated config/goal/failure budget");
  }else if(name=="config_disabled"){
    f.enable_fail_safe_=false;f.perception_recovery_candidate_ready_=true;
    f.recoverPerceptionStop();
    require(f.solves==0 && f.deliveries==0 && f.perception_recovery_pending_,
        "fail_safe=false automatically recovered");
  }else if(name=="esdf_wait"){
    f.esdf_ready=false;
    for(int n=0;n<10;++n){refresh(f);f.recoverPerceptionStop();tick(.2);}
    require(f.solves==0 && f.deliveries==0 && f.new_traj_fail_time_==4,
        "ESDF wait consumed solve/budget or published");
    f.esdf_ready=true;refresh(f);staged(f);refresh(f);f.recoverPerceptionStop();
    require(f.deliveries==1,"ESDF completion did not unblock recovery");
  }else if(name=="no_goal_or_moving"){
    f.have_target_=false;f.recoverPerceptionStop();
    require(f.solves==0 && f.deliveries==0,"no-target recovery moved");
    f.have_target_=true;f.mm_state_vel_(0)=.11;f.recoverPerceptionStop();
    require(f.solves==0 && f.deliveries==0,"moving state was treated as stopped");
  }else if(name=="failure_retry"){
    f.solve_ok=false;f.recoverPerceptionStop();
    require(f.solves==1 && f.have_target_ && f.new_traj_fail_time_==4,
        "failed recovery discarded accepted goal/budget");
    refresh(f);f.recoverPerceptionStop();require(f.solves==1,"retry did not wait .5s");
    tick(.51);refresh(f);f.solve_ok=true;f.recoverPerceptionStop();
    require(f.solves==2 && f.perception_recovery_candidate_ready_,"retry never resumed");
  }else if(name=="collision_reject"){
    for(int collision=0;collision<3;++collision){
      REMANIReplanFSM g;refresh(g);staged(g);refresh(g);
      g.static_clear=collision!=0;g.dynamic_clear=collision!=1;g.predicted_clear=collision!=2;
      g.recoverPerceptionStop();
      require(g.deliveries==0 && !g.perception_recovery_candidate_ready_ &&
          g.perception_recovery_pending_,"unsafe final candidate published/reused");
      require(g.have_target_ && g.new_traj_fail_time_==4,"collision discarded goal");
    }
  }else if(name=="start_drift"){
    for(int drift=0;drift<3;++drift){
      REMANIReplanFSM g;refresh(g);staged(g);refresh(g);
      if(drift==0)g.mm_state_pos_(0)=.081;
      if(drift==1)g.mm_state_pos_(2)=.101;
      if(drift==2)g.mm_car_yaw_=.151;
      g.recoverPerceptionStop();
      require(g.deliveries==0 && g.validations==0 &&
          !g.perception_recovery_candidate_ready_,"candidate start drift accepted");
    }
  }else if(name=="final_feedback_expires"){
    staged(f);refresh(f);f.validate_elapsed=.151;
    f.recoverPerceptionStop();
    require(f.deliveries==0 && f.perception_recovery_pending_ &&
        f.perception_recovery_candidate_ready_,"freshness not checked after validation");
    f.validate_elapsed=0.;refresh(f);f.recoverPerceptionStop();
    require(f.deliveries==1 && f.solves==1,"fresh retry unnecessarily reoptimized");
  }else if(name=="loss_during_validation"){
    staged(f);refresh(f);f.on_validate=[&](){map->timeout=true;};
    f.recoverPerceptionStop();
    require(f.deliveries==0 && f.perception_recovery_pending_,
        "sensor failed during final acceptance but candidate was published");
    f.on_validate=nullptr;f.recoverPerceptionStop();
    require(!f.perception_recovery_candidate_ready_,"renewed outage retained candidate");
  }else if(name=="other_owners"){
    staged(f);refresh(f);
    f.tracking_safety_stop_active_=true;f.tracking_fault_active_=true;
    f.dynamic_safety_speed_limit_active_=true;f.dynamic_point_flow_hazard_active_=true;
    f.dynamic_event_required_esdf_seq_=12;f.recoverPerceptionStop();
    require(f.deliveries==1 && !f.perception_recovery_pending_,"own recovery never finished");
    require(f.tracking_safety_stop_active_ && f.tracking_fault_active_ &&
        f.dynamic_safety_speed_limit_active_ && f.dynamic_point_flow_hazard_active_ &&
        f.dynamic_event_required_esdf_seq_==12 && f.dynamic_clears==0,
        "recovery released another stop owner");
    require(f.current_safety_speed_limit_==0.,"other owners lost zero-speed hold");
  }else if(name=="stop_clamp"){
    f.publishSafetySpeedLimit(-1.);f.publishSafetySpeedLimit(.2);
    require(f.current_safety_speed_limit_==0. &&
        f.safety_speed_limit_pub_.values.size()==1,"pending perception stop was bypassed");
    f.perception_recovery_pending_=false;f.publishSafetySpeedLimit(.2);
    require(f.current_safety_speed_limit_==.2,"normal speed publishing changed");
    f.publishSafetySpeedLimit(-1.);require(f.current_safety_speed_limit_==-1.,"release changed");
  }else if(name=="rebase_all_segments"){
    SingulTrajData candidate;
    candidate.first=Eigen::VectorXd::LinSpaced(8,-.2,.5);
    const auto geometry=candidate.first;
    rebaseStoppedTrajectory(candidate,ros::Time(202.25));
    require(candidate.start_time==202.25 && candidate.start_time_ros.toSec()==202.25,
        "whole-trajectory start not rebased");
    require(candidate.singul_traj[0].start_time==202.25 &&
        candidate.singul_traj[0].end_time==203.25 &&
        candidate.singul_traj[1].start_time==203.25 &&
        candidate.singul_traj[1].end_time==205.25,"segment timestamps inconsistent");
    require(candidate.duration==3. && candidate.first==geometry,"rebase changed geometry/duration");
  }else if(name=="rear_goal_trials"){
    f.rear=true;staged(f);
    require(f.trial_count==3,"rear goal no longer uses configured retry count");
  }else throw std::runtime_error("unknown test case");
}
int main(int argc,char** argv){
  try{require(argc==2,"case argument required");run(argv[1]);std::cout<<"PASS "<<argv[1]<<'\n';}
  catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
'''


class PerceptionRecoveryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not shutil.which('g++'):
            raise RuntimeError('g++ is required for the actual C++ recovery tests')
        cls.source = SOURCE.read_text()
        cls.temporary = tempfile.TemporaryDirectory(prefix='remani-perception-recovery-')
        cls.addClassCleanup(cls.temporary.cleanup)
        folder = Path(cls.temporary.name)
        cls.binary = folder / 'recovery'
        signatures = (
            'void rebaseStoppedTrajectory(',
            'bool REMANIReplanFSM::perceptionRecoveryReady()',
            'void REMANIReplanFSM::recoverPerceptionStop()',
            'void REMANIReplanFSM::publishSafetySpeedLimit(',
            'void REMANIReplanFSM::finishNewTrajectory()',
        )
        program = PREAMBLE + '\n'.join(function(cls.source, item) for item in signatures) + CASES
        cpp = folder / 'recovery.cpp'
        cpp.write_text(program)
        process = subprocess.run(
            ['g++', '-std=c++14', '-O0', '-g0', '-DEIGEN_DONT_VECTORIZE',
             '-I/usr/include/eigen3', str(cpp), '-o', str(cls.binary)],
            capture_output=True, text=True, timeout=90,
            env=dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1'))
        if process.returncode:
            raise AssertionError('Recovery harness compile failed:\n' + process.stderr)

    def test_production_recovery_transitions(self):
        cases = (
            'two_phase', 'source_validity', 'sustained_loss', 'config_disabled',
            'esdf_wait', 'no_goal_or_moving', 'failure_retry', 'collision_reject',
            'start_drift', 'final_feedback_expires', 'loss_during_validation',
            'other_owners', 'stop_clamp', 'rebase_all_segments', 'rear_goal_trials',
        )
        for case in cases:
            with self.subTest(case=case):
                process = subprocess.run([str(self.binary), case], capture_output=True,
                                         text=True, timeout=5)
                self.assertEqual(process.returncode, 0, process.stdout + process.stderr)

    def test_integration_stages_before_transport_and_invalidates_new_goal(self):
        # These short wiring assertions supplement the executable method tests.
        # They do not stand in for the recovery-transition tests above.
        rebound = function(self.source, 'bool REMANIReplanFSM::callReboundReplan(')
        staged = rebound.index('perception_recovery_candidate_ready_ = true;')
        delivered = rebound.index('sendPolyTrajROSMsg(')
        self.assertLess(staged, delivered)
        self.assertIn('return true;', rebound[staged:delivered])
        goal = function(self.source, 'void REMANIReplanFSM::waypointCallback(')
        accepted = goal.index('if (!planNextWaypoint(')
        invalidated = goal.index('perception_recovery_candidate_ready_ = false;', accepted)
        self.assertIn('return;', goal[accepted:invalidated])
        self.assertNotIn('enable_fail_safe_ = false;', self.source)
        safety = function(self.source, 'void REMANIReplanFSM::checkCollisionCallback(')
        guard = safety.index('if (perception_recovery_pending_)')
        ordinary = safety.index('if (!have_target_)')
        self.assertLess(guard, ordinary)
        self.assertIn('return;', safety[guard:ordinary])
        validate = function(self.source,
                            'bool REMANIReplanFSM::validateTrajectoryForPublication(')
        for check in ('validateStaticTrajectory(', 'validateDynamicTrajectory(',
                      'validatePredictedDynamicTrajectory('):
            self.assertIn(check, validate)
        execute = function(self.source, 'void REMANIReplanFSM::execFSMCallback(')
        generation = execute[execute.index('case GEN_NEW_TRAJ:'):execute.index('case REPLAN_TRAJ:')]
        self.assertLess(generation.index('recoverPerceptionStop();'),
                        generation.index('planFromGlobalTraj('))


if __name__ == '__main__':
    unittest.main(verbosity=2)
