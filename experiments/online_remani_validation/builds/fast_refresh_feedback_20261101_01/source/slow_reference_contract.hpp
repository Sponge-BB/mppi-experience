#pragma once
// Isolated SIL ablation of slow geometric-reference admission. Actual MPPI
// controls still require their unchanged latest-frame rollout/braking checks.
#include <cstdlib>
#include <string>

namespace slow_reference_contract {
inline bool nominalOnly(){
  const char* p=std::getenv("SLOW_REFERENCE_NOMINAL_ONLY");return p&&std::string(p)=="1";
}
inline bool includeHardBranch(std::size_t index){return index==0||!nominalOnly();}
} // namespace slow_reference_contract
