#pragma once
#include "frontend_rejoin_probe.hpp"
namespace frontend_rejoin {
// Only the paused diagnostic uses elapsed wall time. Live native search retains
// its existing ROS clock and exact configured search budget.
inline double probeNow(){return running()?failure_probe::wall():ros::Time::now().toSec();}
}
