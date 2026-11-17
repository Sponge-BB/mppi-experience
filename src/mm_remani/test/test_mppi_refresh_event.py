"""Execute production delivery and one-shot refresh logic without ROS/hardware."""
from pathlib import Path
import os
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_mppi_deadline_priority as delivery
from mppi_refresh_event import AvoidanceEpisode, encode_event

ROOT = Path(__file__).resolve().parents[3]
CPP = ROOT/'src/mm_remani_planner/plan_manage/src/remani_replan_fsm.cpp'
INCLUDE = ROOT/'src/mm_remani_planner/plan_manage/include'


class DeliveredRefreshTests(unittest.TestCase):
    def setUp(self):
        self.f = delivery.DeadlinePriorityTests(methodName='runTest')
        self.f.setUp()
        self.node = self.f.node
        self.node.refresh_episode = AvoidanceEpisode()
        self.node.refresh_pub = Mock()
        self.node.reference.native = NS(start=98.5)
        header = np.zeros(32)
        header[3], header[4], header[15] = 99.8, 1., .35
        header[18], header[19], header[28] = 99.9, 1., 1.
        self.node.snapshot.header = header
        self.intervention = True
        self.hold = False
        self.node.reference.plan = Mock(side_effect=self.solve)

    def solve(self, *args, **kwargs):
        result, info = self.f.solve(*args)
        info['intervention'] = self.intervention
        if self.hold:
            info['reference_wait_hold'] = True
        return result, info

    def test_event_follows_both_published_commands_and_keeps_source_times(self):
        def on_event(packet):
            self.node.arm_pub.publish.assert_called_once()
            self.node.base_pub.publish.assert_called_once()
            self.assertEqual(packet.data, [1.,100.,99.,98.5,99.99,99.99,99.8,99.9,1.,100.])
        self.node.refresh_pub.publish.side_effect = on_event
        result = self.f.cycle()
        self.assertEqual(result['state'], 'command_delivered')
        self.node.refresh_pub.publish.assert_called_once()

    def test_no_request_for_rejection_deadlines_or_observe_only(self):
        for failure in ('solve_deadline','delivery_deadline','infeasible','observe','stale'):
            with self.subTest(failure=failure):
                self.setUp()
                if failure == 'solve_deadline': self.f.solve_seconds = .120
                if failure == 'delivery_deadline': self.f.check_seconds = .060
                if failure == 'infeasible':
                    self.f.has_result = False
                    self.f.feasible = 0
                if failure == 'observe': self.node.execute = False
                if failure == 'stale': self.node.joint_stamp = 99.
                self.f.cycle()
                self.node.refresh_pub.publish.assert_not_called()

    def test_successful_wait_hold_is_not_an_avoidance_event(self):
        self.hold = True
        self.assertEqual(self.f.cycle()['state'], 'command_delivered')
        self.node.refresh_pub.publish.assert_not_called()

    def test_successful_startup_is_not_an_avoidance_event(self):
        n = self.node
        n.startup_complete = False
        n.native_arm_startup = False
        n.startup_enable_requested = True
        n.startup_target = np.zeros(6)
        n.startup_settled_since = None
        with patch('whole_body_mppi.startup_plan', side_effect=self.solve):
            self.assertEqual(self.f.cycle()['state'], 'command_delivered')
        n.refresh_pub.publish.assert_not_called()

    def test_successful_terminal_alignment_is_not_an_avoidance_event(self):
        self.f.reference_status = 'native_reference_expired'
        self.f.has_result = False
        self.node.terminal_alignment = object()
        self.node.kernel.H = 2
        self.node.kernel.lease_brake_states = Mock(return_value=self.f.fixture.predicted)
        self.node.reference.terminal_pose_command = Mock(return_value=np.zeros(8))
        self.node.reference.status.return_value = {'reference_status':'native_reference_expired'}
        self.assertEqual(self.f.cycle()['state'], 'command_delivered')
        self.node.refresh_pub.publish.assert_not_called()

    def test_optional_notification_failure_does_not_stop_delivered_action(self):
        for failure in ('transport','episode'):
            with self.subTest(failure=failure):
                self.setUp()
                if failure == 'transport':
                    self.node.refresh_pub.publish.side_effect = ValueError('transport failed')
                else:
                    self.node.refresh_episode.observe = Mock(side_effect=ValueError('bad notification'))
                with patch('whole_body_mppi.rospy.logwarn_throttle'):
                    result = self.f.cycle()
                self.assertEqual(result['state'], 'command_delivered')
                self.node.base_pub.publish.assert_called_once()
                self.assertGreater(self.node.base_pub.publish.call_args.args[0].linear.x, 0.)

    def test_real_rejection_interrupts_four_clear_delivery_streak(self):
        self.f.cycle()
        self.intervention = False
        for _ in range(3): self.f.cycle()
        self.assertEqual(self.node.refresh_episode.clear_count, 3)
        self.f.solve_seconds = .120
        self.f.cycle()
        self.assertEqual(self.node.refresh_episode.clear_count, 0)
        self.f.solve_seconds = .05
        self.intervention = True
        self.f.cycle()
        self.assertEqual(self.node.refresh_episode.episode, 100.)


class EpisodeTests(unittest.TestCase):
    def test_reference_change_restart_and_four_explicit_clear_cycles(self):
        e = AvoidanceEpisode()
        self.assertEqual(e.observe(10.,True,True,101.),101.)
        # Native-reference identity never resets a continuing episode.
        self.assertEqual(e.observe(10.,True,True,102.),101.)
        for _ in range(3): self.assertIsNone(e.observe(10.,True,False,103.))
        self.assertIsNone(e.observe(10.,False,None,104.))
        self.assertEqual(e.observe(10.,True,True,105.),101.)
        for _ in range(4): self.assertIsNone(e.observe(10.,True,False,106.))
        self.assertEqual(e.observe(10.,True,True,107.),107.)
        self.assertEqual(AvoidanceEpisode().observe(10.,True,True,108.),108.)
        self.assertEqual(e.observe(11.,True,True,109.),109.)

    def test_no_prefix_or_unused_commands_and_malformed_times_rejected(self):
        self.assertEqual(len(encode_event(100.,90.,99.,99.98,99.99,99.8,99.9,True,99.95)),10)
        for stamp, episode in ((float('nan'),99.95),(100.,101.)):
            with self.assertRaises(ValueError):
                encode_event(stamp,90.,99.,99.98,99.99,99.8,99.9,True,episode)


def definition(source, signature):
    start = source.index(signature)
    opening = source.index('{',start)
    depth, cursor = 1, opening+1
    while depth:
        depth += (source[cursor]=='{')-(source[cursor]=='}')
        cursor += 1
    return source[start:cursor]


PREAMBLE = r'''
#include <plan_manage/mppi_refresh_event.hpp>
#include <Eigen/Core>
#include <algorithm>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
namespace ros {
double wall=100., steady=100.;
struct Duration {double value; double toSec()const{return value;}};
struct Time {
  double value; explicit Time(double v=0):value(v){}
  static Time now(){return Time(wall);}
  bool isZero()const{return value==0.;} double toSec()const{return value;}
};
struct SteadyTime {
  double value; explicit SteadyTime(double v=0):value(v){}
  static SteadyTime now(){return SteadyTime(steady);}
  bool isZero()const{return value==0.;}
  Duration operator-(const SteadyTime& r)const{return {value-r.value};}
};
}
int logged_success=-1; double logged_elapsed=-1.;
void logResult(const char*,double,double,double,int success,double elapsed){
  logged_success=success;logged_elapsed=elapsed;
}
#define ROS_INFO(...) logResult(__VA_ARGS__)
namespace std_msgs {struct Float64MultiArray {
  using ConstPtr=std::shared_ptr<const Float64MultiArray>;
  std::vector<double> data;
};}
struct SingulTrajData {
  double start_time=99.;std::vector<int> singul_traj{7};
  void clearSingulTraj(){singul_traj.clear();}
};
struct Map {bool ready=true;bool mapReadyForPlanning()const{return ready;}};
struct Optimizer {double getMaxWallTime()const{return 1.5;}};
struct Planner {
  std::shared_ptr<Map> grid_map_=std::make_shared<Map>();
  std::shared_ptr<Optimizer> ploy_traj_opt_=std::make_shared<Optimizer>();
  struct {
    struct {double glb_t_of_lc_tgt=12.,last_glb_t_of_lc_tgt=11.;} global_traj;
    SingulTrajData singul_traj_data;
  } traj_container_;
};
enum State {EXEC_TRAJ,REPLAN_TRAJ,EMERGENCY_STOP};
class REMANIReplanFSM {
public:
  mppi_refresh::Event mppi_refresh_request_;
  double mppi_goal_stamp_=90.,mppi_consumed_goal_=0.,mppi_consumed_episode_=0.;
  ros::SteadyTime mppi_refresh_attempt_;
  State exec_state_=EXEC_TRAJ;
  bool have_target_=true,have_local_traj_=true,have_published_trajectory_=true;
  bool have_new_target_=false,perception_recovery_pending_=false;
  bool tracking_fault_active_=false,tracking_safety_stop_active_=false;
  bool tracking_recovery_pending_=false,try_plan_after_emergency_=false;
  bool dynamic_replan_retry_pending_=false,dynamic_safety_speed_limit_active_=false;
  double current_safety_speed_limit_=.2;
  bool required=false,have_odom_=true,solve_ok=true,safety_during_solve=false;
  ros::Time perception_odom_stamp_{99.99},perception_joint_stamp_{99.99};
  Eigen::VectorXd mm_state_pos_=Eigen::VectorXd::Zero(8),mm_state_vel_=Eigen::VectorXd::Zero(8);
  double mm_car_yaw_=0.,mm_car_yaw_rate_=0.;
  double global_plan_dynamic_replan_hold_time_=.6,dynamic_astar_replan_wall_time_=.6;
  SingulTrajData last_published_trajectory_;
  std::shared_ptr<Planner> planner_manager_=std::make_shared<Planner>();
  Eigen::VectorXd local_target_pt_=Eigen::VectorXd::Ones(8);
  Eigen::VectorXd local_target_vel_=Eigen::VectorXd::Ones(8)*2.;
  Eigen::VectorXd local_target_acc_=Eigen::VectorXd::Ones(8)*3.;
  Eigen::VectorXd start_pos_=Eigen::VectorXd::Ones(8)*4.;
  Eigen::VectorXd start_vel_=Eigen::VectorXd::Ones(8)*5.;
  Eigen::VectorXd start_acc_=Eigen::VectorXd::Ones(8)*6.;
  Eigen::VectorXd start_jer_=Eigen::VectorXd::Ones(8)*7.;
  double start_yaw_=.3;int start_singul_=-1;
  int attempts=0,replan_fail_time_=17,deliveries=0;
  bool hasUnmetRequiredESDF()const{return required;}
  bool perceptionRecoveryReady()const;
  void mppiRefreshCallback(const std_msgs::Float64MultiArray::ConstPtr&);
  bool tryMppiRefresh();
  bool restoreProductionTrajectory();
  bool progress_ready=true;
  // Controllable solver boundary: real source-time/clock/queue behavior is
  // exercised separately by test_fsm_feedback_phase.py's extracted methods.
  bool waitForControllerProgress(const SingulTrajData&,const ros::Time&) const {
    return progress_ready;
  }
  bool planFromLocalTraj(bool poly,bool random,double budget,bool dynamic){
    if(poly || random || !dynamic || budget!=.6)
      throw std::runtime_error("not one bounded warm dynamic attempt");
    ++attempts;ros::wall+=.1;ros::steady+=.1;
    planner_manager_->traj_container_.global_traj.glb_t_of_lc_tgt=70.;
    planner_manager_->traj_container_.global_traj.last_glb_t_of_lc_tgt=69.;
    local_target_pt_.setConstant(88.);local_target_vel_.setConstant(88.);
    local_target_acc_.setConstant(88.);start_pos_.setConstant(88.);
    start_vel_.setConstant(88.);start_acc_.setConstant(88.);start_jer_.setConstant(88.);
    start_yaw_=88.;start_singul_=1;have_new_target_=false;
    planner_manager_->traj_container_.singul_traj_data.start_time=100.7;
    if(safety_during_solve){exec_state_=EMERGENCY_STOP;tracking_safety_stop_active_=true;
      current_safety_speed_limit_=0.;}
    // Solver/final-validation boundary is stubbed; failure executes the real
    // existing callReboundReplan restore lambda extracted below.
    if(!solve_ok)return restoreProductionTrajectory();
    last_published_trajectory_=planner_manager_->traj_container_.singul_traj_data;
    ++deliveries;return true;
  }
};
'''

CASES = r'''
void require(bool value,const char* why){if(!value)throw std::runtime_error(why);}
void request(REMANIReplanFSM& f,double episode=99.95){
  auto msg=std::make_shared<std_msgs::Float64MultiArray>();
  const double t=ros::wall;
  msg->data={1,t,f.mppi_goal_stamp_,f.last_published_trajectory_.start_time,
    t-.01,t-.01,t-.10,t-.05,1,episode};
  f.perception_odom_stamp_=ros::Time(t);f.perception_joint_stamp_=ros::Time(t);
  f.mppiRefreshCallback(msg);
}
void run(const std::string& name){
  REMANIReplanFSM f;request(f);
  if(name=="success_duplicate_reference_restart"){
    require(f.attempts==0,"callback must only cache");
    require(f.tryMppiRefresh() && f.attempts==1 && f.deliveries==1,"no native attempt");
    require(logged_success==1 && std::abs(logged_elapsed-100.)<1e-5,"timing/outcome missing");
    ros::wall+=1.;ros::steady+=1.;request(f); // new reference, same episode
    require(!f.tryMppiRefresh() && f.attempts==1,"reference change reopened episode");
    request(f,ros::wall); // later process/source episode on the same goal
    require(f.tryMppiRefresh() && f.attempts==2,"restart episode collided");
  }else if(name=="failure_restores_metadata_and_trajectory"){
    f.solve_ok=false;require(f.tryMppiRefresh(),"no attempted refresh");
    require(logged_success==0 && f.attempts==1 && f.deliveries==0,"wrong failure outcome");
    auto& data=f.planner_manager_->traj_container_;
    require(data.singul_traj_data.start_time==99.,"failed candidate replaced reference");
    require(data.global_traj.glb_t_of_lc_tgt==12. && data.global_traj.last_glb_t_of_lc_tgt==11.,"global cursors changed");
    require(f.local_target_pt_(0)==1. && f.local_target_vel_(0)==2. && f.local_target_acc_(0)==3.,"local target changed");
    require(f.start_pos_(0)==4. && f.start_vel_(0)==5. && f.start_acc_(0)==6. && f.start_jer_(0)==7.,"start derivatives changed");
    require(f.start_yaw_==.3 && f.start_singul_==-1 && !f.have_new_target_,"start bookkeeping changed");
    require(f.have_target_ && f.have_local_traj_ && f.exec_state_==EXEC_TRAJ && f.replan_fail_time_==17,"failed optional request entered ordinary failure loop");
    ros::wall+=1.;ros::steady+=1.;request(f);require(!f.tryMppiRefresh(),"failed episode repeated");
  }else if(name=="positive_limit_preserved_success_and_failure"){
    for(bool success:{true,false}){
      REMANIReplanFSM g;g.solve_ok=success;g.dynamic_safety_speed_limit_active_=true;
      g.current_safety_speed_limit_=.08;request(g,ros::wall);
      require(g.tryMppiRefresh(),"positive slowdown incorrectly blocks update");
      require(g.dynamic_safety_speed_limit_active_ && g.current_safety_speed_limit_==.08,"refresh released speed owner");
    }
  }else if(name=="independent_stop_survives_failure"){
    f.solve_ok=false;f.safety_during_solve=true;require(f.tryMppiRefresh(),"not attempted");
    require(f.exec_state_==EMERGENCY_STOP && f.tracking_safety_stop_active_ && f.current_safety_speed_limit_==0.,"failure rollback cleared independent stop");
  }else if(name=="owners_and_readiness_block"){
    for(int owner=0;owner<13;++owner){
      REMANIReplanFSM g;request(g);
      if(owner==0)g.perception_recovery_pending_=true;
      if(owner==1)g.tracking_fault_active_=true;
      if(owner==2)g.tracking_safety_stop_active_=true;
      if(owner==3)g.tracking_recovery_pending_=true;
      if(owner==4)g.try_plan_after_emergency_=true;
      if(owner==5)g.dynamic_replan_retry_pending_=true;
      if(owner==6)g.required=true;
      if(owner==7){g.dynamic_safety_speed_limit_active_=true;g.current_safety_speed_limit_=0.;}
      if(owner==8)g.exec_state_=EMERGENCY_STOP;
      if(owner==9)g.planner_manager_->grid_map_->ready=false;
      if(owner==10)g.perception_joint_stamp_=ros::Time(99.7);
      if(owner==11)g.have_target_=false;
      if(owner==12)g.have_new_target_=true;
      require(!g.tryMppiRefresh() && g.attempts==0 && g.mppi_consumed_episode_==0.,"guard allowed optional update or consumed episode");
    }
  }else if(name=="unknown_progress_keeps_episode_pending"){
    f.progress_ready=false;
    require(!f.tryMppiRefresh() && f.attempts==0 && f.deliveries==0 &&
        f.mppi_consumed_episode_==0. && f.mppi_consumed_goal_==0. &&
        f.mppi_refresh_attempt_.isZero() && f.have_target_,
        "unknown progress consumed the optional event or cooldown");
    f.progress_ready=true;
    require(f.tryMppiRefresh() && f.attempts==1 && f.deliveries==1,
        "fresh matching progress could not reuse the unconsumed event");
  }else if(name=="identity_sources_schema"){
    for(int field:{1,2,3,4,5,6,7}){
      REMANIReplanFSM g;request(g);g.mppi_refresh_request_.data[field]-=1.;
      require(!g.tryMppiRefresh(),"stale/wrong identity accepted");
    }
    auto msg=std::make_shared<std_msgs::Float64MultiArray>();msg->data={1,2};
    f.mppiRefreshCallback(msg);require(!f.tryMppiRefresh(),"invalid packet retained valid request");
    request(f);f.mppi_refresh_request_.data[8]=0.;f.mppi_refresh_request_.data[7]=0.;
    require(f.tryMppiRefresh(),"no dynamic overlay incorrectly blocks event");
  }else if(name=="cooldown_pending_expiry"){
    require(f.tryMppiRefresh(),"first attempt missing");
    request(f,ros::wall);require(!f.tryMppiRefresh(),"cooldown bypassed");
    ros::wall+=.7;ros::steady+=.7;
    require(!f.tryMppiRefresh(),"expired queued request accepted");
    request(f,ros::wall);require(f.tryMppiRefresh(),"new fresh episode blocked");
  }else throw std::runtime_error("unknown case");
}
int main(int argc,char** argv){
  try{require(argc==2,"case required");run(argv[1]);}
  catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
'''


class NativeRefreshTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = CPP.read_text()
        methods = '\n'.join(definition(source, signature) for signature in (
            'void REMANIReplanFSM::mppiRefreshCallback(',
            'bool REMANIReplanFSM::tryMppiRefresh()',
            'bool REMANIReplanFSM::perceptionRecoveryReady()'))
        rebound = definition(source,'bool REMANIReplanFSM::callReboundReplan(')
        restore_start = rebound.index('    const bool preserve_executing_trajectory =')
        restore_lambda = definition(rebound,'    auto finish_failed_planning_attempt =')
        restore_end = rebound.index(restore_lambda)+len(restore_lambda)
        restore = ('bool REMANIReplanFSM::restoreProductionTrajectory(){\n' +
                   rebound[restore_start:restore_end] + ';return finish_failed_planning_attempt();}\n')
        cls.temporary = tempfile.TemporaryDirectory(prefix='remani-refresh-test-')
        cls.addClassCleanup(cls.temporary.cleanup)
        folder = Path(cls.temporary.name)
        program = folder/'refresh.cpp'; cls.binary = folder/'refresh'
        program.write_text(PREAMBLE+methods+restore+CASES)
        result = subprocess.run(['g++','-std=c++14','-O0','-g0','-DEIGEN_DONT_VECTORIZE',
            '-I/usr/include/eigen3','-I'+str(INCLUDE),str(program),'-o',str(cls.binary)],
            capture_output=True,text=True,timeout=60,
            env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1'))
        if result.returncode: raise AssertionError(result.stderr)

    def test_production_fsm_attempt_rollback_and_stop_owners(self):
        for case in ('success_duplicate_reference_restart','failure_restores_metadata_and_trajectory',
                     'positive_limit_preserved_success_and_failure','independent_stop_survives_failure',
                     'owners_and_readiness_block','identity_sources_schema','cooldown_pending_expiry',
                     'unknown_progress_keeps_episode_pending'):
            with self.subTest(case=case):
                result = subprocess.run([str(self.binary),case],capture_output=True,text=True,timeout=5)
                self.assertEqual(result.returncode,0,result.stderr)


if __name__ == '__main__': unittest.main()
