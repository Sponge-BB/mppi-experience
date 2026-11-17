// Pure mechanism tests. No ROS master, planner, controller or hardware access.
#include "arm_box_refresh_policy.hpp"
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#ifdef ARM_BOX_CONTEXT_HEADER
#include ARM_BOX_CONTEXT_HEADER
#endif

using namespace arm_box_refresh;
#define CHECK(value) do{if(!(value))throw std::runtime_error(std::string(__func__)+":"+std::to_string(__LINE__)+": " #value);}while(false)

Packet event(double stamp=1014.856507012){
  Packet p;p.identity={1,1,42};p.episode=1;p.stamp=stamp;
  p.valid=p.active=p.checked=true;return p;
}
SourceTimes sources(double now){
  SourceTimes s;s.stamp=s.odom=s.joint=now-.02;s.cloud=s.dynamic=now-.10;return s;
}

void expired_after_map_wait_then_one_attempt(){
  Pending p;auto e=event();const auto id=e.identity;
  SourceTimes old;old.stamp=old.odom=old.joint=e.stamp;old.cloud=old.dynamic=1014.743207946;
  CHECK(freshSources(old,1014.931957767));
  CHECK(decide(p,e,id,1014.931957767,94,93,false,true,true)==Action::WaitMap);
  CHECK(p.active&&p.required_esdf==94);
  CHECK(!freshSources(old,1015.120331543));
  CHECK(decide(p,e,id,1015.120331543,95,94,false,true,false)==Action::WaitSeed);
  CHECK(p.active&&p.required_esdf==94);
  auto next=event(1015.243520302);
  CHECK(freshSources(sources(1015.263611218),1015.263611218));
  CHECK(decide(p,next,id,1015.263611218,98,96,false,true,true)==Action::Attempt);
  CHECK(p.required_esdf==94); // No chasing the newer raw frames 95 and 98.
  Packet captured;CHECK(beginAttempt(p,captured));CHECK(!p.active);
  CHECK(captured.stamp==e.stamp&&captured.episode==1); // Fixed event identity.
  CHECK(!beginAttempt(p,captured));
  // Caller consumes this episode before solving, including a failed solve.
  CHECK(decide(p,next,id,1015.28,99,96,false,false,true)==Action::Idle);
}

void required_repair_ignores_missing_seed_and_optional_cooldown(){
  Pending p;auto e=event();auto id=e.identity;
  CHECK(decide(p,e,id,e.stamp,20,19,false,true,false)==Action::WaitMap);
  Packet missing;
  CHECK(decide(p,missing,id,e.stamp+.1,21,19,true,false,false)==Action::Required);
  CHECK(!p.active);
  CHECK(decide(p,missing,id,e.stamp+.2,22,20,true,false,false)==Action::Required);
}

void changed_identity_cancels_without_moving_event_to_new_reference(){
  for(int field=0;field<3;++field){
    Pending p;auto e=event();auto id=e.identity;
    CHECK(decide(p,e,id,e.stamp,3,2,false,true,false)==Action::WaitMap);
    if(field==0)++id.generation;else if(field==1)++id.revision;else ++id.goal;
    CHECK(decide(p,e,id,e.stamp+.1,4,3,false,false,true)==Action::CancelIdentity);
    CHECK(!p.active);
    CHECK(decide(p,e,id,e.stamp+.12,4,4,false,false,true)==Action::Idle);
  }
}

void one_inactive_packet_is_not_episode_end(){
  Pending p;auto e=event();auto id=e.identity;
  CHECK(decide(p,e,id,e.stamp,3,2,false,true,false)==Action::WaitMap);
  auto clear=e;clear.active=false;clear.stamp+=.1;
  CHECK(decide(p,clear,id,clear.stamp,4,3,false,false,false)==Action::WaitSeed);
  CHECK(p.clear_packets==1&&p.active);
  for(int i=0;i<5;++i)
    CHECK(decide(p,clear,id,clear.stamp+.01*i,4,3,false,false,false)==Action::WaitSeed);
  CHECK(p.clear_packets==1); // Re-reading latest must not count as new evidence.
  auto active=event(clear.stamp+.1);
  CHECK(decide(p,active,id,active.stamp,5,4,false,true,false)==Action::WaitSeed);
  CHECK(p.clear_packets==0);
  for(int i=0;i<4;++i){
    clear.stamp=active.stamp+.1*(i+1);
    const auto action=decide(p,clear,id,clear.stamp,8+i,7+i,false,false,false);
    CHECK(action==(i==3?Action::CancelResolved:Action::WaitSeed));
  }
  CHECK(!p.active);
}

void pending_survives_stale_event_without_marking_an_attempt(){
  Pending p;auto e=event();auto id=e.identity;
  CHECK(decide(p,e,id,e.stamp,94,93,false,true,false)==Action::WaitMap);
  for(int i=1;i<=5;++i){
    CHECK(decide(p,e,id,e.stamp+i*.3,94+i,94,false,false,false)==Action::WaitSeed);
    CHECK(p.active&&p.event.stamp==e.stamp&&p.required_esdf==94);
  }
  auto next=event(e.stamp+1.6);next.episode=2;
  CHECK(decide(p,next,id,next.stamp,110,109,false,true,true)==Action::WaitMap);
  CHECK(p.event.episode==2&&p.required_esdf==110);
}

void every_source_stamp_has_independent_age_and_future_checks(){
  const double now=100.;const auto good=sources(now);CHECK(freshSources(good,now));
  double SourceTimes::* fields[]={&SourceTimes::stamp,&SourceTimes::odom,&SourceTimes::joint,
                                  &SourceTimes::cloud,&SourceTimes::dynamic};
  for(int i=0;i<5;++i){
    const double limit=i<3?.25:.35;
    auto s=good;s.*fields[i]=now-limit+.0001;CHECK(freshSources(s,now));
    s=good;s.*fields[i]=now-limit-.0001;CHECK(!freshSources(s,now));
    s=good;s.*fields[i]=now+.0499;CHECK(freshSources(s,now));
    s=good;s.*fields[i]=now+.0501;CHECK(!freshSources(s,now));
    s=good;s.*fields[i]=0.;CHECK(!freshSources(s,now));
    s=good;s.*fields[i]=std::numeric_limits<double>::quiet_NaN();CHECK(!freshSources(s,now));
  }
  CHECK(!freshSources(good,std::numeric_limits<double>::quiet_NaN()));
}

#ifdef ARM_BOX_CONTEXT_HEADER
collab::Seed full_seed(){
  collab::Seed s;s.stamp=100.;s.odom=100.;s.cloud=s.dynamic=99.9;
  s.generation=1;s.revision=1;s.goal=42;s.arm.joint_stamp=100.;
  for(int i=0;i<=20;++i){
    const double t=i*.1;
    std::array<double,8> row{{t,.2*t,0.,0.,.2,0.,.2,0.}};
    s.rows.push_back(row);
    active_arm_coupling::Row whole;whole.base=row;
    for(int j=0;j<6;++j){whole.arm.q[j]=.1*t;whole.arm.dq[j]=.1;}
    s.arm.rows.push_back(whole);
  }
  return s;
}

void frozen_seed_scope_keeps_epoch_and_exact_prefix_phase(){
  collab::seed()=full_seed();ros::test_now=100.05;
  const Identity id{1,1,42};const auto hash=collab::seedFingerprint(collab::seed());
  {
    collab::SeedScope scope(collab::seed(),ros::test_now,id,true);
    collab::seed().revision=99;collab::seed().stamp=100.8;
    ros::test_now=100.8; // Live clock and inbox advance during one request.
    CHECK(collab::seedEpoch()==100.05&&collab::strictPrefix());
    CHECK(collab::seedView().revision==1&&collab::seedFingerprint(collab::seedView())==hash);
    CHECK(collab::seedSourcesFresh(collab::seedView(),collab::seedEpoch()));
    std::vector<std::array<double,8>> rows;std::string reason;double origin=-1.;
    Eigen::VectorXd start{.01,0.};
    CHECK(collab::prefixRows(start,0.,1,rows,reason,&origin));
    CHECK(std::abs(origin-.05)<1e-10); // No ceil to the next .1s sample.
    CHECK(rows.front()[0]==0.&&std::abs(rows.front()[1]-.01)<1e-10);
    {
      collab::SeedScope inner(collab::seedView(),100.10,id,false);
      CHECK(collab::seedEpoch()==100.10&&!collab::strictPrefix());
    }
    CHECK(collab::seedEpoch()==100.05&&collab::strictPrefix());
  }
  CHECK(collab::seedView().revision==99&&collab::seedEpoch()==100.8&&!collab::strictPrefix());
}

void prefix_refuses_wrong_generation_revision_and_goal(){
  const Identity id{1,1,42};ros::test_now=100.05;
  for(int field=0;field<3;++field){
    auto s=full_seed();
    if(field==0)++s.generation;else if(field==1)++s.revision;else ++s.goal;
    collab::SeedScope scope(s,ros::test_now,id,true);
    std::vector<std::array<double,8>> rows;std::string reason;
    Eigen::VectorXd start{.01,0.};
    CHECK(!collab::prefixRows(start,0.,1,rows,reason));
    CHECK(reason=="reference_epoch_mismatch");
  }
}
#endif

int main(){
  try{
    expired_after_map_wait_then_one_attempt();
    required_repair_ignores_missing_seed_and_optional_cooldown();
    changed_identity_cancels_without_moving_event_to_new_reference();
    one_inactive_packet_is_not_episode_end();
    pending_survives_stale_event_without_marking_an_attempt();
    every_source_stamp_has_independent_age_and_future_checks();
    int cases=6;
#ifdef ARM_BOX_CONTEXT_HEADER
    frozen_seed_scope_keeps_epoch_and_exact_prefix_phase();
    prefix_refuses_wrong_generation_revision_and_goal();cases+=2;
#endif
    std::cout<<"{\"passed\":true,\"mechanism_cases\":"<<cases<<",\"ros_started\":false}"<<std::endl;
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<std::endl;return 1;}
}
