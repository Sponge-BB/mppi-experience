"""Run the actual C++ transient-stop recovery with bounded fake boundaries."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

from test_perception_recovery import PREAMBLE, SOURCE, function


CASES = r'''
void require(bool condition,const char* detail){
  if(!condition)throw std::runtime_error(detail);
}
void refresh(REMANIReplanFSM& f){
  f.perception_odom_stamp_=ros::Time::now();
  f.perception_joint_stamp_=ros::Time::now();
}
void tick(double seconds){ros::wall_now+=seconds;ros::steady_now+=seconds;}
void new_map(REMANIReplanFSM& f){
  ++f.planner_manager_->grid_map_->esdf;
}
void run(const std::string& name){
  REMANIReplanFSM f;
  auto map=f.planner_manager_->grid_map_;
  f.perception_recovery_pending_=false;
  f.beginTransientObstacleRecovery();
  require(f.have_target_ && f.perception_recovery_pending_ &&
      f.transient_obstacle_recovery_ && f.flag_escape_emergency_ &&
      f.state==EMERGENCY_STOP && f.current_safety_speed_limit_==0.,
      "raw-live stop did not retain the accepted goal and zero-speed owner");
  if(name=="failed_more_than_five"){
    f.solve_ok=false;
    for(int n=0;n<8;++n){
      tick(.51);refresh(f);new_map(f);f.recoverPerceptionStop();
      require(f.have_target_ && f.new_traj_fail_time_==4 && f.deliveries==0,
          "transient failure consumed the ordinary goal/failure budget");
    }
    require(f.solves==8,"new complete maps did not permit bounded retries");
    tick(.51);refresh(f);new_map(f);f.solve_ok=true;f.recoverPerceptionStop();
    require(f.deliveries==0,"solver completion skipped fresh feedback phase");
    refresh(f);f.recoverPerceptionStop();
    require(f.deliveries==1 && f.delivered_goal==17 && f.have_target_ &&
        !f.perception_recovery_pending_ && !f.transient_obstacle_recovery_,
        "accepted goal failed to recover after more than five failures");
  }else if(name=="complete_map_and_throttle"){
    f.solve_ok=false;f.recoverPerceptionStop();
    for(int n=0;n<4;++n){tick(.6);refresh(f);f.recoverPerceptionStop();}
    require(f.solves==1,"unchanged complete map retried repeatedly");
    new_map(f);f.recoverPerceptionStop();
    require(f.solves==2,"new complete map failed to permit retry");
    new_map(f);refresh(f);f.recoverPerceptionStop();
    require(f.solves==2,"new map bypassed the retry interval");
    tick(.51);refresh(f);f.esdf_ready=false;f.recoverPerceptionStop();
    require(f.solves==2,"incomplete map invoked a solve");
    f.esdf_ready=true;f.recoverPerceptionStop();
    require(f.solves==3,"completed map was consumed before becoming ready");
    tick(.51);refresh(f);++map->epoch;map->esdf=1;f.recoverPerceptionStop();
    require(f.solves==4,"new epoch with reset sequence could not recover");
  }else if(name=="fresh_two_phase_with_new_maps"){
    f.recoverPerceptionStop();
    require(f.solves==1 && f.deliveries==0 &&
        f.perception_recovery_candidate_ready_,"first phase did not stage");
    new_map(f);f.recoverPerceptionStop();
    require(f.deliveries==0,"350ms solve published with expired feedback");
    refresh(f);
    // Validation owns its read scope. A newer incoming map is not itself a
    // reason to reject a candidate that passes validation and final freshness.
    f.on_validate=[&](){new_map(f);};
    f.recoverPerceptionStop();
    require(f.deliveries==1 && f.solves==1,
        "continuing observations starved an otherwise valid candidate");
  }else if(name=="new_goal_between_phases"){
    f.recoverPerceptionStop();refresh(f);
    ++f.accepted_goal_revision_;f.goal_id=18;
    f.recovery_last_attempt_map_={0,0};
    f.recoverPerceptionStop();
    require(f.deliveries==0 && !f.perception_recovery_candidate_ready_,
        "old staged goal survived accepted goal revision change");
    f.recoverPerceptionStop();refresh(f);f.recoverPerceptionStop();
    require(f.deliveries==1 && f.delivered_goal==18,
        "new goal was overwritten by old recovery");
  }else if(name=="new_goal_during_validation"){
    f.recoverPerceptionStop();refresh(f);
    f.on_validate=[&](){++f.accepted_goal_revision_;f.goal_id=18;};
    f.recoverPerceptionStop();
    require(f.deliveries==0 && !f.perception_recovery_candidate_ready_,
        "goal changed during validation but old trajectory was published");
  }else if(name=="cancel_does_not_resume"){
    f.recoverPerceptionStop();refresh(f);f.have_target_=false;
    for(int n=0;n<4;++n){
      new_map(f);refresh(f);f.recoverPerceptionStop();tick(.6);
    }
    require(f.solves==1 && f.deliveries==0 && !f.have_target_ &&
        !f.perception_recovery_candidate_ready_ &&
        f.current_safety_speed_limit_==0.,"cancelled target resumed");
  }else if(name=="collision_and_drift_still_reject"){
    for(int kind=0;kind<5;++kind){
      REMANIReplanFSM g;refresh(g);g.beginTransientObstacleRecovery();
      g.recoverPerceptionStop();refresh(g);
      g.static_clear=kind!=0;g.dynamic_clear=kind!=1;g.predicted_clear=kind!=2;
      if(kind==3)g.on_validate=[&](){g.mm_state_pos_(0)=.081;};
      if(kind==4)g.on_validate=[&](){g.mm_state_pos_(2)=.101;};
      g.recoverPerceptionStop();
      require(g.deliveries==0 && g.have_target_ &&
          g.perception_recovery_pending_ &&
          !g.perception_recovery_candidate_ready_,
          "collision or latest-feedback drift bypassed final validation");
    }
  }else if(name=="other_stop_owners"){
    f.recoverPerceptionStop();refresh(f);
    f.tracking_safety_stop_active_=true;f.tracking_fault_active_=true;
    f.dynamic_safety_speed_limit_active_=true;
    f.dynamic_point_flow_hazard_active_=true;
    f.dynamic_event_required_esdf_seq_=12;
    f.recoverPerceptionStop();
    require(f.deliveries==1 && !f.perception_recovery_pending_ &&
        f.tracking_safety_stop_active_ && f.tracking_fault_active_ &&
        f.dynamic_safety_speed_limit_active_ &&
        f.dynamic_point_flow_hazard_active_ &&
        f.dynamic_event_required_esdf_seq_==12 &&
        f.current_safety_speed_limit_==0. && f.dynamic_clears==0,
        "transient recovery released another stop owner");
  }else throw std::runtime_error("unknown case");
}
int main(int argc,char** argv){
  try{require(argc==2,"case required");run(argv[1]);std::cout<<"PASS "<<argv[1]<<'\n';}
  catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
'''


class TransientObstacleRecoveryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not shutil.which('g++'):
            raise RuntimeError('g++ is required for production C++ recovery tests')
        cls.source = SOURCE.read_text()
        cls.temporary = tempfile.TemporaryDirectory(prefix='remani-obstacle-recovery-')
        cls.addClassCleanup(cls.temporary.cleanup)
        folder = Path(cls.temporary.name)
        cls.binary = folder / 'recovery'
        signatures = (
            'void rebaseStoppedTrajectory(',
            'bool REMANIReplanFSM::perceptionRecoveryReady()',
            'void REMANIReplanFSM::beginTransientObstacleRecovery()',
            'void REMANIReplanFSM::recoverPerceptionStop()',
            'void REMANIReplanFSM::publishSafetySpeedLimit(',
            'void REMANIReplanFSM::finishNewTrajectory()',
        )
        cpp = folder / 'recovery.cpp'
        cpp.write_text(PREAMBLE + '\n'.join(function(cls.source, item)
                                            for item in signatures) + CASES)
        process = subprocess.run(
            ['g++', '-std=c++14', '-O0', '-g0', '-DEIGEN_DONT_VECTORIZE',
             '-I/usr/include/eigen3', str(cpp), '-o', str(cls.binary)],
            capture_output=True, text=True, timeout=90,
            env=dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1'))
        if process.returncode:
            raise AssertionError('Recovery harness compile failed:\n' + process.stderr)

    def test_transient_recovery_behaviour(self):
        for case in ('failed_more_than_five', 'complete_map_and_throttle',
                     'fresh_two_phase_with_new_maps', 'new_goal_between_phases',
                     'new_goal_during_validation', 'cancel_does_not_resume',
                     'collision_and_drift_still_reject', 'other_stop_owners'):
            with self.subTest(case=case):
                process = subprocess.run([str(self.binary), case], capture_output=True,
                                         text=True, timeout=5)
                self.assertEqual(process.returncode, 0, process.stdout + process.stderr)

    def test_raw_live_stop_enters_shared_recovery(self):
        safety = function(self.source, 'void REMANIReplanFSM::checkCollisionCallback(')
        raw = safety.index('dynamic_collision_path_distance <= dynamic_emergency_distance')
        stop = safety[raw:safety.index('if (!waitForRequiredESDF(', raw)]
        self.assertIn('beginTransientObstacleRecovery();', stop)
        goal = function(self.source, 'void REMANIReplanFSM::waypointCallback(')
        self.assertIn('++accepted_goal_revision_;', goal)
        self.assertIn('recovery_last_attempt_map_ = {0, 0};', goal)


if __name__ == '__main__':
    unittest.main(verbosity=2)
