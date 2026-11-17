#include "feasible_quality_polish.hpp"
#include <fstream>
#include <iostream>
#include <iterator>

// Synthetic exit routing only. No map/native predicate is mocked and reaching
// this counter is not a collision certificate or permission to publish.
enum class Stop { None, Error, Timeout };
bool syntheticExit(const delivery_candidate::Scope& accepted,
                   const feasible_quality_polish::Checkpoint& safe,
                   Stop stop,int& final_gate_entries){
  if(accepted.invalid)return false;
  if(stop==Stop::Timeout&&!safe.available&&!accepted.usable())return false;
  if(stop==Stop::Error)return false;
  ++final_gate_entries;return true;
}

int main(int argc,char** argv){
  using namespace feasible_quality_polish;
  Eigen::VectorXd x(2);x<<1.,2.;const Eigen::VectorXd g=Eigen::VectorXd::Ones(2);
  delivery_candidate::Scope accepted(true);accepted.completed(x,g,10.);accepted.accepted(x,10.);
  Snapshot snapshot;snapshot.valid=true;snapshot.total=10.;snapshot.x_hash=delivery_candidate::hash(x);
  Checkpoint safe;const bool checkpoint=safe.consider(x,10.,true,true,snapshot);
  accepted.fail();Stop stop=Stop::Error;stop=Stop::Timeout;
  // A later normal-looking callback must not clear the sticky error either.
  accepted.completed(x,g,9.);accepted.accepted(x,9.);
  int error_entries=0;
  const bool sticky=accepted.invalid&&!accepted.usable();
  const bool rejected=!syntheticExit(accepted,safe,stop,error_entries)&&error_entries==0;
  delivery_candidate::Scope normal(true);normal.completed(x,g,10.);normal.accepted(x,10.);
  int normal_entries=0;const bool normal_safe=syntheticExit(normal,safe,Stop::Timeout,normal_entries)&&normal_entries==1;
  bool source_guard=false;
  if(argc==2){
    std::ifstream in(argv[1]);const std::string source((std::istreambuf_iterator<char>(in)),std::istreambuf_iterator<char>());
    const std::string guard="    if(accepted_candidate.invalid){\n      diagnosis.reason=\"nonfinite\";";
    const auto a=source.find(guard),b=source.find("    if(force_stop_type_ == STOP_FOR_TIMEOUT){"),
               c=source.find("    // test collision",b);
    source_guard=a!=std::string::npos&&b!=std::string::npos&&c!=std::string::npos&&a<b&&b<c&&
      source.substr(a,b-a).find("return false;")!=std::string::npos;
  }
  const bool pass=checkpoint&&sticky&&rejected&&normal_safe&&source_guard;
  std::cout<<"{\"pass\":"<<(pass?"true":"false")
    <<",\"sticky_invalid_after_error_then_timeout\":"<<(sticky?"true":"false")
    <<",\"invalid_with_safe_stops_before_final_gate\":"<<(rejected?"true":"false")
    <<",\"normal_safe_timeout_can_continue\":"<<(normal_safe?"true":"false")
    <<",\"compiled_source_guard_precedes_timeout\":"<<(source_guard?"true":"false")
    <<",\"scope\":\"synthetic exit routing and actual source ordering; no native collision validation or motion authority tested\"}\n";
  return pass?0:1;
}
