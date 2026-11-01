#include "fast_refresh_feedback.hpp"
#include <iostream>
#include <limits>

int main(){
  using namespace fast_refresh_feedback;
  State value;const Identity first{1,2,1234};
  const std::vector<double> good{1,100.,1,2,1234,1,1,1,.17};
  bool pass=parse(good,value.latest)&&eligible(value,100.1,first);
  auto rejects=[&](std::vector<double> packet,const Identity& identity,double now){
    State test;return parse(packet,test.latest)&&!eligible(test,now,identity);
  };
  const bool identity=rejects(good,{2,2,1234},100.1)&&rejects(good,{1,3,1234},100.1)&&
                      rejects(good,{1,2,5678},100.1);
  const bool freshness=rejects(good,first,100.251)&&rejects(good,first,99.949)&&
                       rejects(good,first,std::numeric_limits<double>::quiet_NaN());
  auto packet=good;packet[7]=0.;const bool unvalidated=rejects(packet,first,100.1);
  packet=good;packet[6]=0.;const bool inactive=rejects(packet,first,100.1);
  consume(value);const bool once=!eligible(value,100.1,first);
  packet=good;packet[3]=3.;parse(packet,value.latest);
  const bool same_episode_new_reference=!eligible(value,100.1,{1,3,1234});
  packet[5]=2.;parse(packet,value.latest);
  const bool next_episode=eligible(value,100.1,{1,3,1234});
  consume(value);packet[5]=1.;parse(packet,value.latest);
  const bool delayed_old_episode=!eligible(value,100.1,{1,3,1234});
  packet[2]=2.;packet[3]=4.;packet[4]=5678.;parse(packet,value.latest);
  const bool new_goal=eligible(value,100.1,{2,4,5678});
  bool malformed=true;
  for(int field:{0,2,3,4,5,6,7}){
    packet=good;packet[field]=-.5;Packet out;malformed=malformed&&!parse(packet,out)&&!out.valid;
  }
  packet=good;packet[5]=0.;Packet out;malformed=malformed&&!parse(packet,out);
  packet=good;packet[4]=4294967296.;malformed=malformed&&!parse(packet,out);
  packet=good;packet[8]=std::numeric_limits<double>::infinity();malformed=malformed&&!parse(packet,out);
  packet=good;packet.push_back(0.);malformed=malformed&&!parse(packet,out);
  unsetenv("FAST_REFRESH_FEEDBACK");const bool default_off=!enabled();
  setenv("FAST_REFRESH_FEEDBACK","1",1);const bool switched_on=enabled();
  unsetenv("FAST_REFRESH_FEEDBACK");
  pass=pass&&identity&&freshness&&unvalidated&&inactive&&once&&same_episode_new_reference&&
       next_episode&&delayed_old_episode&&new_goal&&malformed&&default_off&&switched_on;
  std::cout<<"{\"pass\":"<<(pass?"true":"false")
    <<",\"identity\":"<<(identity?"true":"false")
    <<",\"freshness\":"<<(freshness?"true":"false")
    <<",\"actual_command_required\":"<<(unvalidated?"true":"false")
    <<",\"once_per_episode_across_references\":"<<(once&&same_episode_new_reference?"true":"false")
    <<",\"next_episode\":"<<(next_episode?"true":"false")
    <<",\"delayed_old_episode_rejected\":"<<(delayed_old_episode?"true":"false")
    <<",\"new_goal\":"<<(new_goal?"true":"false")
    <<",\"malformed_rejected\":"<<(malformed?"true":"false")<<"}\n";
  return pass?0:1;
}
