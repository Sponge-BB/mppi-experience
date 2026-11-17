#!/usr/bin/env bash
set -euo pipefail

# Copy this script and ../config/distributed.rviz to hai; no robot libraries needed.
if [[ $# -gt 2 ]]; then
  echo "Usage: $0 [MASTER_URI [HAI_IP]]" >&2
  exit 2
fi
STANDBY_VIEW_MASTER="${1:-http://192.168.137.128:11462}"
STANDBY_VIEW_IP="${2:-192.168.137.107}"
STANDBY_VIEW_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
if [[ ! "$STANDBY_VIEW_MASTER" =~ ^http://[A-Za-z0-9_.-]+:[0-9]+/?$ ]]; then
  echo 'An explicit HTTP ROS master URI is required.' >&2
  exit 2
fi
if [[ "$STANDBY_VIEW_IP" == 192.168.137.128 ]] ||
   ! ip -o -4 address show | /usr/bin/awk -v wanted="$STANDBY_VIEW_IP" \
     '{split($4, address, "/"); if (address[1] == wanted) found=1} END {exit !found}'; then
  echo 'Run RViz on hai. HAI_IP must be an address of this computer, not the robot computer.' >&2
  exit 2
fi

unset ROS_NAMESPACE ROS_HOSTNAME ROS_IP ROS_MASTER_URI
unset ROS_PACKAGE_PATH CMAKE_PREFIX_PATH ROSLISP_PACKAGE_DIRECTORIES LD_LIBRARY_PATH PYTHONPATH
set +u
source /opt/ros/noetic/setup.bash
set -u
export ROS_MASTER_URI="$STANDBY_VIEW_MASTER"
export ROS_IP="$STANDBY_VIEW_IP"
unset ROS_NAMESPACE ROS_HOSTNAME
export DISPLAY="${DISPLAY:-:0}"
if [[ -z "${XAUTHORITY:-}" && -r "/run/user/$(id -u)/gdm/Xauthority" ]]; then
  export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
fi
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
# The informational Noetic EOL modal otherwise hides the actual RViz view.
export DISABLE_ROS1_EOL_WARNINGS=1
echo "RViz only: master=$ROS_MASTER_URI, hai=$ROS_IP"
echo 'world is the isolated trial local odometry frame, not the original global map.'
echo '2D Nav Goal publishes only /standby_trial_20261106_distributed_08/goal.'
exec /opt/ros/noetic/bin/rviz -d "$STANDBY_VIEW_ROOT/config/distributed.rviz" \
  __name:=standby_distributed_08_view __ns:=/standby_trial_20261106_distributed_08
