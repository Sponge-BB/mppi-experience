"""Exercise extracted production FSM phase logic and a real ROS callback queue.

This does not launch a ROS master, nodes or hardware. The solver/trajectory and
TF math boundaries are small fixtures; the queue and all tested FSM methods are
the production implementations. Assertions use require(), not C assert/NDEBUG.
Use --emit-harness FILE to inspect the generated translation unit without g++.
"""
from pathlib import Path
import hashlib
import json
import os
import shlex
import shutil
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]
SOURCE = Path(os.environ.get(
    'FSM_FEEDBACK_SOURCE',
    str(ROOT / 'src/mm_remani_planner/plan_manage/src/remani_replan_fsm.cpp')))


def function(source, signature):
    begin = source.index(signature)
    opening = source.index('{', begin)
    level = 1
    end = opening + 1
    while level:
        if source[end] == '{':
            level += 1
        elif source[end] == '}':
            level -= 1
        end += 1
    return source[begin:end]


PREAMBLE = r'''
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/make_shared.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace tf {
// A pure math boundary, unrelated to source-time/queue behavior under test.
double getYaw(const geometry_msgs::Quaternion& q) {
  return std::atan2(2.*(q.w*q.z+q.x*q.y),1.-2.*(q.y*q.y+q.z*q.z));
}
}
struct SingulTrajData { double start_time=100.; double duration=20.; };
struct MovingHandoff { SingulTrajData executing; };
struct Planner {
  struct {SingulTrajData singul_traj_data;} traj_container_;
};
class REMANIReplanFSM {
public:
  bool mppi_terminal_control_=true;
  bool have_controller_progress_=false;
  double controller_trajectory_start_time_=0.;
  double controller_trajectory_time_=0.;
  double controller_trajectory_scale_=0.;
  double controller_progress_timeout_=.3;
  ros::Time controller_progress_stamp_;
  ros::CallbackQueue feedback_callback_queue_;
  int mobile_base_dim_=2, manipulator_dim_=6;
  double mobile_base_non_singul_vel_=.01;
  Eigen::VectorXd mm_state_pos_=Eigen::VectorXd::Zero(8);
  Eigen::VectorXd mm_state_vel_=Eigen::VectorXd::Zero(8);
  Eigen::VectorXd mm_state_acc_=Eigen::VectorXd::Zero(8);
  Eigen::VectorXd default_arm_config_=Eigen::VectorXd::Zero(6);
  Eigen::Quaterniond mm_car_orient_=Eigen::Quaterniond::Identity();
  double mm_car_yaw_=0., mm_car_yaw_rate_=0.;
  int mm_car_singul_=0;
  ros::Time perception_odom_stamp_, perception_joint_stamp_;
  bool have_odom_=false, has_default_arm_=true, default_arm_from_config_=true;
  bool have_target_=true, have_new_target_=true, restored_old_reference_=false;
  bool dynamic_safety_speed_limit_active_=false, esdf_ready_=true;
  double current_safety_speed_limit_=.2;
  int ordinary_budget_=17, dynamic_budget_=2, generation_=45;
  std::shared_ptr<Planner> planner_manager_=std::make_shared<Planner>();
  bool waitForRequiredESDF(const char*)const{return esdf_ready_;}
  void drainFeedbackCallbacks();
  bool currentTrajectoryPhase(const SingulTrajData&,const ros::Time&,
                              double&,double&) const;
  bool waitForControllerProgress(const SingulTrajData&,const ros::Time&) const;
  double getCurrentTrajectoryTime(const SingulTrajData&,const ros::Time&) const;
  double getCurrentTrajectoryScale(const SingulTrajData&,const ros::Time&) const;
  void trajectoryProgressCallback(const geometry_msgs::Vector3Stamped::ConstPtr&);
  void mmCarOdomCallback(const nav_msgs::OdometryConstPtr&);
  void mmManiOdomCallback(const sensor_msgs::JointStateConstPtr&);
  bool postSolve(const MovingHandoff* handoff);
  bool replanEntry();
};
'''


CASES = r'''
void require(bool value,const char* detail) {
  if(!value)throw std::runtime_error(detail);
}
bool close(double a,double b){return std::fabs(a-b)<1e-8;}
ros::Time stamp(double value){ros::Time t;t.fromSec(value);return t;}
void packet(REMANIReplanFSM& f,double reference,double phase,double scale,
            double source_stamp) {
  auto p=boost::make_shared<geometry_msgs::Vector3Stamped>();
  p->header.stamp=stamp(source_stamp);
  p->vector.x=reference;p->vector.y=phase;p->vector.z=scale;
  f.trajectoryProgressCallback(p);
}
class FeedbackCall: public ros::CallbackInterface {
  std::function<void()> body_;
public:
  explicit FeedbackCall(std::function<void()> body):body_(std::move(body)){}
  CallResult call() override {body_();return Success;}
};
void enqueue(REMANIReplanFSM& f,std::function<void()> body) {
  f.feedback_callback_queue_.addCallback(
      boost::make_shared<FeedbackCall>(std::move(body)));
}
void run(const std::string& name) {
  REMANIReplanFSM f;
  SingulTrajData traj;
  const ros::Time now=stamp(108.);
  ros::Time::setNow(now);
  double phase=-1.,scale=-1.;
  if(name=="stale_never_becomes_wall_clock") {
    packet(f,100.,6.242451821,.4,107.5);
    require(!f.currentTrajectoryPhase(traj,now,phase,scale),"stale progress valid");
    require(close(phase,6.242451821)&&scale==0.,"stale progress advanced to wall time");
    require(!f.waitForControllerProgress(traj,now),"stale progress passed decision gate");
    require(f.have_target_&&f.have_new_target_,"wait deleted accepted target");
    require(close(f.getCurrentTrajectoryTime(traj,now),phase)&&
            f.getCurrentTrajectoryScale(traj,now)==0.,"wrappers restored wall-clock permission");
  } else if(name=="fresh_slow_and_zero_scale") {
    packet(f,100.,2.,.4,107.8);
    require(f.currentTrajectoryPhase(traj,now,phase,scale)&&close(phase,2.08)&&
        close(scale,.4),"slow scale not respected");
    packet(f,100.,2.,0.,107.8);
    require(f.currentTrajectoryPhase(traj,now,phase,scale)&&phase==2.&&scale==0.,
        "zero scale advanced despite fresh stopped controller clock");
  } else if(name=="first_reference_waits_for_progress") {
    require(!f.currentTrajectoryPhase(traj,now,phase,scale)&&phase==0.&&scale==0.,
        "first missing progress became wall-clock completion");
    require(!f.waitForControllerProgress(traj,now)&&f.have_target_,
        "first missing progress passed or deleted target");
    packet(f,100.,.01,.5,108.);
    require(f.currentTrajectoryPhase(traj,now,phase,scale)&&close(phase,.01),
        "first valid progress failed to resume readiness");
  } else if(name=="clock_skew_matches_mppi_adapter") {
    packet(f,100.,2.,.4,108.003);
    require(f.currentTrajectoryPhase(traj,now,phase,scale)&&close(phase,1.9988),
        "permitted future source time did not back-project like MPPI adapter");
    packet(f,100.,2.,.4,108.006);
    require(!f.currentTrajectoryPhase(traj,now,phase,scale),"future source outside tolerance passed");
  } else if(name=="mismatched_reference_waits") {
    packet(f,100.002,2.,.4,108.);
    require(!f.currentTrajectoryPhase(traj,now,phase,scale)&&phase==0.&&scale==0.,
        "different reference reused controller phase or wall clock");
    packet(f,100.,2.,.4,108.);
    require(f.currentTrajectoryPhase(traj,now,phase,scale),"matching reference did not recover");
  } else if(name=="nonfinite_packet_cannot_be_clamped_valid") {
    for(int field=0;field<3;++field)for(int kind=0;kind<2;++kind) {
      packet(f,100.,2.,.4,108.);
      double values[3]={100.,2.,.4};
      values[field]=kind?std::numeric_limits<double>::infinity():
                         std::numeric_limits<double>::quiet_NaN();
      packet(f,values[0],values[1],values[2],108.);
      require(!f.have_controller_progress_&&
          !f.currentTrajectoryPhase(traj,now,phase,scale),
          "nonfinite raw progress became valid after clamp");
    }
    for(const auto& bad: {std::pair<double,double>{-.001,.4},
        {300.001,.4},{2.,-.001},{2.,1.001}}) {
      packet(f,100.,bad.first,bad.second,108.);
      require(!f.have_controller_progress_&&
          !f.currentTrajectoryPhase(traj,now,phase,scale),
          "out-of-range raw progress became valid after clamp");
    }
  } else if(name=="future_reference_stays_pending") {
    traj.start_time=108.1;
    packet(f,108.1,.02,.4,108.);
    require(!f.currentTrajectoryPhase(traj,now,phase,scale)&&phase==0.&&scale==0.,
        "unactivated future reference advanced");
    require(f.currentTrajectoryPhase(traj,stamp(108.1),phase,scale)&&close(phase,.06),
        "activated matching reference still pending");
  } else if(name=="native_compatibility") {
    f.mppi_terminal_control_=false;
    require(f.currentTrajectoryPhase(traj,now,phase,scale)&&phase==8.&&scale==1.,
        "native first-reference wall clock changed");
    packet(f,100.,2.,.4,107.5);
    require(f.currentTrajectoryPhase(traj,now,phase,scale)&&phase==8.&&scale==1.,
        "native expired-progress compatibility changed");
    packet(f,101.,2.,.4,108.);
    require(f.currentTrajectoryPhase(traj,now,phase,scale)&&phase==8.&&scale==1.,
        "native mismatched-progress compatibility changed");
    packet(f,100.,2.,.4,108.03);
    require(f.currentTrajectoryPhase(traj,now,phase,scale)&&phase==2.&&close(scale,.4),
        "native negative-age handling changed");
  } else if(name=="real_queue_adopts_progress_odom_and_joints") {
    packet(f,100.,6.24,.4,107.5);
    require(!f.currentTrajectoryPhase(traj,now,phase,scale),"fixture not stale");
    enqueue(f,[&](){packet(f,100.,6.28,.2,107.99);});
    auto odom=boost::make_shared<nav_msgs::Odometry>();
    odom->header.stamp=stamp(107.98);odom->pose.pose.position.x=1.2;
    odom->pose.pose.position.y=-.3;odom->pose.pose.orientation.w=1.;
    odom->twist.twist.linear.x=.1;
    enqueue(f,[&](){f.mmCarOdomCallback(odom);});
    auto joints=boost::make_shared<sensor_msgs::JointState>();
    joints->header.stamp=stamp(107.99);
    joints->position={.1,.2,.3,.4,.5,.6};joints->velocity={0.,0.,0.,0.,0.,0.};
    enqueue(f,[&](){f.mmManiOdomCallback(joints);});
    require(f.mm_state_pos_(0)==0.&&!f.feedback_callback_queue_.isEmpty(),
        "queue did not retain unread source feedback");
    f.drainFeedbackCallbacks();
    require(f.currentTrajectoryPhase(traj,now,phase,scale)&&close(phase,6.282)&&
        close(scale,.2),"decision did not use drained controller progress");
    require(close(f.mm_state_pos_(0),1.2)&&close(f.mm_state_pos_(1),-.3)&&
        close(f.mm_state_pos_(7),.6)&&f.perception_odom_stamp_==stamp(107.98)&&
        f.perception_joint_stamp_==stamp(107.99)&&f.feedback_callback_queue_.isEmpty(),
        "source odom/joint feedback not adopted before decision");
    require(f.have_target_&&f.have_new_target_,"feedback queue changed target/plan metadata");
  } else if(name=="postsolve_expired_progress_restores_old_reference") {
    MovingHandoff handoff;packet(f,100.,6.24,.4,107.5);
    require(!f.postSolve(&handoff)&&f.restored_old_reference_&&f.have_target_&&
        f.have_new_target_,"expired post-solve handoff accepted or deleted target");
  } else if(name=="postsolve_wrong_reference_restores_old_reference") {
    MovingHandoff handoff;packet(f,101.,6.24,.4,108.);
    require(!f.postSolve(&handoff)&&f.restored_old_reference_&&f.have_target_,
        "different post-solve reference accepted frozen old context");
  } else if(name=="postsolve_uses_drained_matching_feedback") {
    MovingHandoff handoff;packet(f,100.,6.24,.4,107.5);
    enqueue(f,[&](){packet(f,100.,6.28,.2,108.);});
    require(f.postSolve(&handoff)&&!f.restored_old_reference_&&!f.have_new_target_&&
        f.currentTrajectoryPhase(handoff.executing,now,phase,scale)&&close(phase,6.28),
        "post-solve acceptance used pre-solve stale inbox");
  } else if(name=="unknown_replan_preserves_all_budgets_and_goal") {
    packet(f,100.,6.24,.4,107.5);
    for(int n=0;n<40;++n)require(!f.replanEntry(),"unknown progress entered local solve");
    require(f.ordinary_budget_==17&&f.dynamic_budget_==2&&f.generation_==45&&
        f.have_target_&&f.have_new_target_,
        "waiting consumed retry budget/generation or accepted goal");
  } else if(name=="fresh_replan_is_allowed") {
    packet(f,100.,6.28,.2,108.);
    require(f.replanEntry()&&f.ordinary_budget_==18&&f.dynamic_budget_==3&&
        f.generation_==46&&f.have_target_,"fresh clock failed to permit a local attempt");
  } else if(name=="guarded_stopped_replan_bypasses_progress") {
    f.dynamic_safety_speed_limit_active_=true;f.current_safety_speed_limit_=0.;
    require(!f.currentTrajectoryPhase(traj,now,phase,scale),"fixture unexpectedly ready");
    require(f.replanEntry()&&f.have_target_,"true stopped guard blocked measured-state recovery");
    REMANIReplanFSM positive;positive.dynamic_safety_speed_limit_active_=true;
    positive.current_safety_speed_limit_=.08;
    require(!positive.replanEntry(),"positive slowdown bypassed unknown phase");
    REMANIReplanFSM native;native.mppi_terminal_control_=false;
    require(native.replanEntry(),"native local-replan compatibility changed");
  } else throw std::runtime_error("unknown case");
}
int main(int argc,char** argv) {
  try {ros::Time::init();require(argc==2,"case required");run(argv[1]);
       std::cout<<"PASS "<<argv[1]<<'\n';}
  catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
'''


CASE_NAMES = (
    'stale_never_becomes_wall_clock', 'fresh_slow_and_zero_scale',
    'first_reference_waits_for_progress', 'clock_skew_matches_mppi_adapter',
    'mismatched_reference_waits', 'nonfinite_packet_cannot_be_clamped_valid',
    'future_reference_stays_pending', 'native_compatibility',
    'real_queue_adopts_progress_odom_and_joints',
    'postsolve_expired_progress_restores_old_reference',
    'postsolve_wrong_reference_restores_old_reference',
    'postsolve_uses_drained_matching_feedback',
    'unknown_replan_preserves_all_budgets_and_goal',
    'fresh_replan_is_allowed', 'guarded_stopped_replan_bypasses_progress',
)


def harness(source):
    signatures = (
        'void REMANIReplanFSM::drainFeedbackCallbacks()',
        'bool REMANIReplanFSM::currentTrajectoryPhase(',
        'bool REMANIReplanFSM::waitForControllerProgress(',
        'double REMANIReplanFSM::getCurrentTrajectoryTime(',
        'double REMANIReplanFSM::getCurrentTrajectoryScale(',
        'void REMANIReplanFSM::trajectoryProgressCallback(',
        'void REMANIReplanFSM::mmCarOdomCallback(',
        'void REMANIReplanFSM::mmManiOdomCallback(',
    )
    rebound = function(source, 'bool REMANIReplanFSM::callReboundReplan(')
    # Exercise the actual post-manager-return fragment, including drain order
    # and rollback branch; solver internals are intentionally outside this test.
    begin = rebound.index('    // Solve-time traffic is already queued')
    end = rebound.index('    have_new_target_ = false;', begin)
    end += len('    have_new_target_ = false;')
    post_solve = ('''bool REMANIReplanFSM::postSolve(const MovingHandoff* handoff){
      auto finish_failed_planning_attempt=[&](){
        restored_old_reference_=true;return false;
      };
''' + rebound[begin:end] + '\nreturn true;\n}\n')
    execution = function(source, 'void REMANIReplanFSM::execFSMCallback(')
    branch = execution.index('    case REPLAN_TRAJ:')
    opening = execution.index('{', branch)
    branch_end = execution.index('      if (dynamic_replan_retry_pending_)', opening)
    # The accounting is a fake downstream boundary. The preceding ESDF and
    # progress/stopped-guard entry is extracted verbatim from the real case.
    replan_entry = ('''bool REMANIReplanFSM::replanEntry(){
      {
''' + execution[opening+1:branch_end] + '''
        ++ordinary_budget_;++dynamic_budget_;++generation_;return true;
      }
    force_return:return false;
    }
''')
    return (PREAMBLE + '\n'.join(function(source, item) for item in signatures)
            + '\n' + post_solve + replan_entry + CASES)


class FSMFeedbackPhaseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not shutil.which('g++'):
            raise RuntimeError('g++ required for extracted production C++ tests')
        cls.source = SOURCE.read_text()
        cls.temporary = tempfile.TemporaryDirectory(prefix='remani-feedback-phase-')
        cls.addClassCleanup(cls.temporary.cleanup)
        folder = Path(cls.temporary.name)
        cls.binary = folder / 'phase'
        cpp = folder / 'phase.cpp'
        cpp.write_text(harness(cls.source))
        available_kib = next(int(line.split()[1]) for line in
            Path('/proc/meminfo').read_text().splitlines()
            if line.startswith('MemAvailable:'))
        if available_kib < 1536 * 1024:
            raise RuntimeError('Refusing compiler: MemAvailable below 1.5 GiB')
        flags = subprocess.run(['pkg-config', '--cflags', '--libs', 'roscpp'],
            capture_output=True, text=True, check=True).stdout
        result = subprocess.run(
            ['g++', '-std=c++14', '-O0', '-g0', '-DEIGEN_DONT_VECTORIZE',
             '-I/usr/include/eigen3', str(cpp), '-o', str(cls.binary)]
             + shlex.split(flags), capture_output=True, text=True, timeout=90,
            env=dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1'))
        if result.returncode:
            raise AssertionError('FSM phase harness compile failed:\n'+result.stderr)

    def test_phase_and_real_callback_queue_behaviour(self):
        for case in CASE_NAMES:
            with self.subTest(case=case):
                result = subprocess.run([str(self.binary), case],
                    capture_output=True, text=True, timeout=5)
                self.assertEqual(result.returncode, 0, result.stdout+result.stderr)

    def test_only_feedback_subscribers_move_to_queue(self):
        # Supplemental routing checks; behavioral cases above use the actual
        # queue and extracted production callbacks rather than token assertions.
        self.assertEqual(self.source.count('feedback_node_->subscribe('), 3)
        for name in ('execFSMCallback', 'checkCollisionCallback', 'planFromLocalTraj'):
            body = function(self.source, 'REMANIReplanFSM::'+name+'(')
            self.assertTrue(body[body.index('{')+1:].lstrip().startswith(
                'drainFeedbackCallbacks();'), name)


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--emit-harness':
        output = Path(sys.argv[2])
        output.write_text(harness(SOURCE.read_text()))
        print(json.dumps({'source': str(SOURCE),
            'source_sha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
            'harness': str(output), 'behaviour_cases': len(CASE_NAMES),
            'compiler_invoked': False}))
    else:
        unittest.main()
