"""Execute production terminal-goal helpers and GEN wiring without ROS/devices."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

import test_perception_recovery as recovery


PREAMBLE = recovery.PREAMBLE.replace(
    '#include <Eigen/Core>', '#include <Eigen/Core>\n#include <algorithm>').replace(
    '#define ROS_INFO(...) ((void)0)',
    '#define ROS_INFO(...) ((void)0)\n#define ROS_INFO_THROTTLE(...) ((void)0)').replace(
    'bool isZero() const{return value == 0.;}',
    'bool isZero() const{return value == 0.;}\n'
    '  Time operator-(const Time& other) const{return Time(value-other.value);}').replace(
    'namespace std_msgs {struct Float64 {double data=0.;};}',
    'namespace std_msgs {struct Float64 {double data=0.;}; struct Bool {bool data=false;};}').replace(
    'void publish(const std_msgs::Float64& msg){values.push_back(msg.data);}',
    'void publish(const std_msgs::Float64& msg){values.push_back(msg.data);}\n'
    '  void publish(const std_msgs::Bool& msg){values.push_back(msg.data);}').replace(
    'enum State {GEN_NEW_TRAJ,EXEC_TRAJ,EMERGENCY_STOP};',
    'enum State {WAIT_TARGET,GEN_NEW_TRAJ,EXEC_TRAJ,EMERGENCY_STOP};\n'
    'struct TARGET_TYPE {enum {MANUAL_TARGET=1, PRESET_TARGET=2};};').replace(
    'class REMANIReplanFSM {\npublic:',
    '''class REMANIReplanFSM {
public:
  bool mppi_terminal_control_=true;
  bool have_trigger_=true,manual_restore_running_=false;
  bool manual_restore_standby_after_goal_=false;
  int target_type_=TARGET_TYPE::MANUAL_TARGET;
  State exec_state_=EXEC_TRAJ;
  double goal_position_tolerance_=.15,goal_yaw_tolerance_=.15;
  double goal_arm_tolerance_=.12,goal_yaw_alignment_timeout_=8.;
  double end_yaw_=0.;
  Eigen::VectorXd end_pt_=Eigen::VectorXd::Zero(8);
  Publisher reached_pub_;
  int perception_recoveries=0;
  bool goalTerminalFeedbackReady() const;
  bool waitForGoalTerminal(const ros::Time&,double,double,double);
  void finishManualGoal();
  void generationTerminalStep();
''').replace(
    'void changeFSMExecState(State next,const char*){state=next;}',
    'void changeFSMExecState(State next,const char*){state=exec_state_=next;}')


CASES = r'''
void require(bool value,const char* detail){if(!value)throw std::runtime_error(detail);}
void ready(REMANIReplanFSM& f){
  f.perception_recovery_pending_=false;f.exec_state_=EXEC_TRAJ;
  f.perception_odom_stamp_=f.perception_joint_stamp_=ros::Time::now();
}
void tick(double seconds){ros::wall_now+=seconds;ros::steady_now+=seconds;}
void REMANIReplanFSM::recoverPerceptionStop(){++perception_recoveries;}
void run(const std::string& name){
  REMANIReplanFSM f;ready(f);
  if(name=="bounded_settle"){
    require(f.waitForGoalTerminal(ros::Time::now(),.14,.25,.20),
        "near final state skipped bounded yaw/arm settling");
    const double start=f.goal_yaw_alignment_start_time_.toSec();
    tick(7.99);ready(f);
    require(f.waitForGoalTerminal(ros::Time::now(),.14,.25,.20),"settle ended before8s");
    tick(.02);ready(f);
    require(!f.waitForGoalTerminal(ros::Time::now(),.14,.25,.20),"settle exceeded8s");
    require(f.goal_yaw_alignment_start_time_.toSec()==start,"same terminal attempt renewed timeout");
    require(!f.waitForGoalTerminal(ros::Time::now(),.14,.01,.20),
        "arm outside formal tolerance extended or finished terminal attempt");
    require(f.reached_pub_.values.empty() && f.have_target_,"wait falsely finished goal");
  }else if(name=="capture_envelope"){
    require(!f.waitForGoalTerminal(ros::Time::now(),.1501,.3,.2),"outside XY envelope waited");
    require(!f.waitForGoalTerminal(ros::Time::now(),.1,.3,.3001),"outside arm envelope waited");
    require(f.waitForGoalTerminal(ros::Time::now(),.15,.3,.30),"capture boundary rejected");
    f.goal_arm_tolerance_=.35;
    require(f.waitForGoalTerminal(ros::Time::now(),.1,.3,.34),"configured arm tolerance was tightened");
    require(!f.waitForGoalTerminal(ros::Time::now(),.1,.01,.20),"fully reached state kept waiting");
    f.target_type_=TARGET_TYPE::PRESET_TARGET;
    require(!f.waitForGoalTerminal(ros::Time::now(),.1,.3,.2),"preset entered MPPI manual settle");
    f.target_type_=TARGET_TYPE::MANUAL_TARGET;f.mppi_terminal_control_=false;
    require(!f.waitForGoalTerminal(ros::Time::now(),.1,.3,.2),"legacy entered MPPI manual settle");
  }else if(name=="source_validity"){
    require(f.goalTerminalFeedbackReady(),"fresh valid goal feedback rejected");
    f.perception_odom_stamp_=ros::Time(ros::wall_now-.151);
    require(!f.goalTerminalFeedbackReady(),"stale odometry completed goal");ready(f);
    f.perception_joint_stamp_=ros::Time(ros::wall_now-.151);
    require(!f.goalTerminalFeedbackReady(),"stale joints completed goal");ready(f);
    f.planner_manager_->grid_map_->timeout=true;
    require(!f.goalTerminalFeedbackReady(),"map timeout completed goal");
    f.planner_manager_->grid_map_->timeout=false;f.planner_manager_->grid_map_->ready=false;
    require(!f.goalTerminalFeedbackReady(),"unready map completed goal");
    f.planner_manager_->grid_map_->ready=true;f.esdf_ready=false;
    require(!f.goalTerminalFeedbackReady(),"unmet map boundary completed goal");
    f.esdf_ready=true;f.mm_state_pos_(3)=std::numeric_limits<double>::quiet_NaN();
    require(!f.goalTerminalFeedbackReady(),"nonfinite feedback completed goal");
  }else if(name=="stop_owners"){
    for(int owner=0;owner<9;++owner){
      REMANIReplanFSM g;ready(g);
      if(owner==0)g.perception_recovery_pending_=true;
      if(owner==1)g.tracking_fault_active_=true;
      if(owner==2)g.tracking_safety_stop_active_=true;
      if(owner==3)g.tracking_recovery_pending_=true;
      if(owner==4)g.dynamic_safety_speed_limit_active_=true;
      if(owner==5)g.dynamic_point_flow_hazard_active_=true;
      if(owner==6)g.dynamic_event_required_esdf_seq_=12;
      if(owner==7)g.try_plan_after_emergency_=true;
      if(owner==8)g.exec_state_=EMERGENCY_STOP;
      require(!g.goalTerminalFeedbackReady(),"terminal check bypassed a stop owner");
      require(!g.waitForGoalTerminal(ros::Time::now(),.1,.3,.2),"unsafe terminal wait accepted");
      g.goal_feedback_recheck_pending_=true;g.generationTerminalStep();
      require(g.reached_pub_.values.empty() && g.have_target_,"GEN bypassed stop owner");
    }
  }else if(name=="late_arrival_recheck"){
    f.exec_state_=GEN_NEW_TRAJ;f.goal_feedback_recheck_pending_=true;
    f.mm_state_pos_(0)=.14;f.mm_car_yaw_=.14;f.mm_state_pos_(2)=.11;
    f.goal_yaw_alignment_start_time_=ros::Time(91.);
    f.generationTerminalStep();
    require(!f.have_target_ && !f.have_trigger_ && f.state==WAIT_TARGET,
        "late valid feedback did not finish goal in GEN");
    require(f.reached_pub_.values.size()==1 && f.reached_pub_.values[0]==1.,"missing finish message");
    require(!f.goal_feedback_recheck_pending_ && f.goal_yaw_alignment_start_time_.isZero(),
        "completed goal retained terminal state");
    require(f.solves==0 && f.deliveries==0,"completed goal invoked planner/trajectory publication");
    require(f.mm_state_pos_(0)==.14 && f.mm_car_yaw_==.14,
        "finish overwrote measured endpoint with goal/old standby anchor");
    f.generationTerminalStep();require(f.reached_pub_.values.size()==1,"duplicate finish");
  }else if(name=="recheck_scope_and_timeout"){
    f.exec_state_=GEN_NEW_TRAJ;f.generationTerminalStep();
    require(f.reached_pub_.values.empty(),"ordinary GEN accepted arrival shortcut");
    for(int boundary=0;boundary<6;++boundary){
      REMANIReplanFSM g;ready(g);g.exec_state_=GEN_NEW_TRAJ;
      g.goal_feedback_recheck_pending_=true;g.goal_yaw_alignment_start_time_=ros::Time(91.);
      if(boundary==0)g.mm_state_pos_(2)=.121;
      if(boundary==1)g.mm_car_yaw_=.151;
      if(boundary==2)g.mm_state_pos_(0)=.151;
      if(boundary==3)g.target_type_=TARGET_TYPE::PRESET_TARGET;
      if(boundary==4)g.manual_restore_standby_after_goal_=true;
      if(boundary==5)g.mppi_terminal_control_=false;
      g.generationTerminalStep();
      require(g.reached_pub_.values.empty() && g.have_target_,"GEN relaxed formal goal tolerances/scope");
      require(g.goal_feedback_recheck_pending_,"GEN retry lost accepted-goal provenance");
      require(g.goal_yaw_alignment_start_time_.toSec()==91.,"GEN restarted terminal timeout");
    }
    f.goal_feedback_recheck_pending_=true;f.mm_state_pos_(2)=.13;
    f.generationTerminalStep();
    require(f.goal_feedback_recheck_pending_ && f.reached_pub_.values.empty(),
        "first GEN retry consumed unfinished goal");
    f.mm_state_pos_(2)=.11;f.generationTerminalStep();
    require(f.reached_pub_.values.size()==1,"later same-goal feedback could not complete GEN retry");
    REMANIReplanFSM stale;ready(stale);stale.exec_state_=GEN_NEW_TRAJ;
    stale.goal_feedback_recheck_pending_=true;
    stale.perception_joint_stamp_=ros::Time(ros::wall_now-.151);
    stale.generationTerminalStep();
    require(stale.goal_feedback_recheck_pending_ && stale.reached_pub_.values.empty(),
        "temporary stale feedback consumed arrival retry");
    ready(stale);stale.exec_state_=GEN_NEW_TRAJ;stale.generationTerminalStep();
    require(stale.reached_pub_.values.size()==1,"fresh late arrival did not finish accepted retry");
  }else throw std::runtime_error("unknown test");
}
int main(int argc,char** argv){
  try{require(argc==2,"case required");run(argv[1]);std::cout<<"PASS "<<argv[1]<<'\n';}
  catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
'''


class GoalTerminalTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not shutil.which('g++'):
            raise RuntimeError('g++ is required for production terminal-goal tests')
        cls.source = recovery.SOURCE.read_text()
        cls.temporary = tempfile.TemporaryDirectory(prefix='remani-goal-terminal-')
        cls.addClassCleanup(cls.temporary.cleanup)
        folder = Path(cls.temporary.name)
        cls.binary = folder / 'terminal'
        signatures = ('bool REMANIReplanFSM::perceptionRecoveryReady()',
                      'bool REMANIReplanFSM::goalTerminalFeedbackReady()',
                      'bool REMANIReplanFSM::waitForGoalTerminal(',
                      'void REMANIReplanFSM::finishManualGoal()')
        execute = recovery.function(cls.source, 'void REMANIReplanFSM::execFSMCallback(')
        generation = execute[execute.index('case GEN_NEW_TRAJ:'):execute.index('case REPLAN_TRAJ:')]
        prefix = generation[generation.index('{')+1:generation.index('// Global/recovery planning')]
        generation_method = ('void REMANIReplanFSM::generationTerminalStep(){\n' +
                             prefix + '\nforce_return:;\n}\n')
        program = PREAMBLE + '\n'.join(recovery.function(cls.source, s) for s in signatures)
        cpp = folder / 'terminal.cpp'
        cpp.write_text(program + generation_method + CASES)
        result = subprocess.run(
            ['g++', '-std=c++14', '-O0', '-g0', '-DEIGEN_DONT_VECTORIZE',
             '-I/usr/include/eigen3', str(cpp), '-o', str(cls.binary)],
            capture_output=True, text=True, timeout=90,
            env=dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1'))
        if result.returncode:
            raise AssertionError('Terminal harness compilation failed:\n'+result.stderr)

    def test_production_terminal_transitions(self):
        for case in ('bounded_settle', 'capture_envelope', 'source_validity',
                     'stop_owners', 'late_arrival_recheck', 'recheck_scope_and_timeout'):
            with self.subTest(case=case):
                result = subprocess.run([str(self.binary), case], capture_output=True,
                                        text=True, timeout=5)
                self.assertEqual(result.returncode, 0, result.stdout+result.stderr)

    def test_fsm_wiring_preserves_legacy_and_resets_goal_state(self):
        execute = recovery.function(self.source, 'void REMANIReplanFSM::execFSMCallback(')
        block = execute[execute.index('case EXEC_TRAJ:'):execute.index('case EMERGENCY_STOP:')]
        self.assertIn('mppi_terminal_control_ && target_type_ == TARGET_TYPE::MANUAL_TARGET', block)
        wait = block.index('waitForGoalTerminal(')
        self.assertIn('trajectory_finished && touch_the_goal', block[:wait])
        self.assertLess(block.index('goalTerminalFeedbackReady()'), wait)
        self.assertIn('}else if(trajectory_finished && touch_the_goal &&', block)
        self.assertIn('goal_feedback_recheck_pending_ = mppi_terminal_control_', block)
        init = recovery.function(self.source, 'void REMANIReplanFSM::init(')
        self.assertIn('nh.param("mppi/enable", mppi_terminal_control_, false);', init)
        goal = recovery.function(self.source, 'void REMANIReplanFSM::waypointCallback(')
        self.assertIn('goal_feedback_recheck_pending_ = false;', goal)
        new = recovery.function(self.source, 'void REMANIReplanFSM::finishNewTrajectory()')
        self.assertIn('goal_feedback_recheck_pending_ = false;', new)
        sent = recovery.function(self.source, 'void REMANIReplanFSM::sendPolyTrajROSMsg(')
        self.assertIn('goal_feedback_recheck_pending_ = false;', sent)
        changed = recovery.function(self.source, 'void REMANIReplanFSM::changeFSMExecState(')
        self.assertIn('if (new_state == EMERGENCY_STOP)', changed)
        self.assertIn('goal_feedback_recheck_pending_ = false;', changed)


if __name__ == '__main__':
    unittest.main(verbosity=2)
