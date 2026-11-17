#pragma once
// Experiment-only optional-refresh bookkeeping. No ROS, control or map access.
#include <cmath>
#include <cstdint>

namespace arm_box_refresh {
struct Identity {
  std::uint64_t generation=0,revision=0,goal=0;
};
inline bool same(const Identity& a,const Identity& b){
  return a.generation==b.generation&&a.revision==b.revision&&a.goal==b.goal;
}
struct Packet {
  Identity identity;
  std::uint64_t episode=0;
  double stamp=0.;
  bool valid=false,active=false,checked=false;
};
template<class Event> inline Packet packet(const Event& event){
  Packet out;out.identity={event.generation,event.revision,event.goal_token};
  out.episode=event.episode;out.stamp=event.stamp;out.valid=event.valid;
  out.active=event.dynamic_avoid_active;out.checked=event.actual_command_validated;
  return out;
}
struct Pending {
  bool active=false;
  Packet event;
  std::uint64_t required_esdf=0;
  unsigned clear_packets=0;
  double last_clear_stamp=0.;
};
enum class Action {Idle,Required,CancelIdentity,CancelResolved,WaitMap,WaitSeed,Attempt};
inline bool freshAge(double stamp,double now,double maximum){
  return std::isfinite(stamp)&&std::isfinite(now)&&stamp>0.&&
         now-stamp>=-.05&&now-stamp<=maximum;
}
struct SourceTimes {double stamp=0.,odom=0.,joint=0.,cloud=0.,dynamic=0.;};
inline bool freshSources(const SourceTimes& s,double now){
  return freshAge(s.stamp,now,.25)&&freshAge(s.odom,now,.25)&&
         freshAge(s.joint,now,.25)&&freshAge(s.cloud,now,.35)&&freshAge(s.dynamic,now,.35);
}
inline Action decide(Pending& pending,const Packet& latest,const Identity& current,
                     double now,std::uint64_t raw,std::uint64_t esdf,
                     bool required,bool event_eligible,bool seed_ready){
  // Required guide repair/partial continuation never waits for an MPPI seed.
  if(required){pending=Pending();return Action::Required;}
  if(pending.active&&!same(pending.event.identity,current)){
    pending=Pending();return Action::CancelIdentity;
  }
  const bool current_packet=latest.valid&&latest.checked&&latest.episode>0&&
      same(latest.identity,current)&&freshAge(latest.stamp,now,.25);
  if(pending.active&&current_packet&&latest.stamp>=pending.event.stamp&&
     latest.episode>=pending.event.episode){
    if(latest.active){pending.clear_packets=0;pending.last_clear_stamp=0.;}
    else if(latest.stamp>pending.last_clear_stamp){
      // moving_away=false is a single-cycle observation, not an episode-end
      // declaration. Require four distinct clear observations, as the sender's
      // existing episode bookkeeping does; rereading one packet counts once.
      ++pending.clear_packets;pending.last_clear_stamp=latest.stamp;
      if(pending.clear_packets>=4){pending=Pending();return Action::CancelResolved;}
    }
  }
  if(current_packet&&event_eligible&&latest.active&&
     (!pending.active||latest.episode>pending.event.episode)){
    pending=Pending();pending.active=true;pending.event=latest;pending.required_esdf=raw;
  }
  if(!pending.active)return Action::Idle;
  // The raw boundary belongs to this pending event, and is never recaptured
  // just because a newer scan arrives while the seed is temporarily stale.
  if(esdf<pending.required_esdf)return Action::WaitMap;
  return seed_ready?Action::Attempt:Action::WaitSeed;
}
inline bool beginAttempt(Pending& pending,Packet& captured){
  if(!pending.active)return false;
  captured=pending.event;pending=Pending();return true;
}
} // namespace arm_box_refresh
