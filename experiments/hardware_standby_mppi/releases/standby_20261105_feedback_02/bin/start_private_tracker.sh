#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.sh"
if [[ $# -ne 2 && $# -ne 4 ]] || [[ "${1:-}" != --master-uri ]]; then
  echo 'Usage: start_private_tracker.sh --master-uri http://HOST:PORT [--advertise-host THIS_HOST]' >&2
  echo 'Read-only perception; existing real sensors and TF must already be running.' >&2
  exit 2
fi
STANDBY_TRACKER_URI="$2"
if [[ ! "$STANDBY_TRACKER_URI" =~ ^http://([A-Za-z0-9_.-]+):([0-9]+)/?$ ]]; then
  echo 'An explicit HTTP ROS master URI with host and port is required.' >&2
  exit 2
fi
STANDBY_TRACKER_MASTER_HOST="${BASH_REMATCH[1]}"
if [[ "$STANDBY_TRACKER_MASTER_HOST" == 127.0.0.1 || "$STANDBY_TRACKER_MASTER_HOST" == localhost ]]; then
  export ROS_IP=127.0.0.1 ROS_HOSTNAME=localhost
elif [[ $# -eq 4 && "$3" == --advertise-host && "$4" =~ ^[A-Za-z0-9_.-]+$ ]]; then
  export ROS_HOSTNAME="$4"
else
  echo 'A remote master requires --advertise-host THIS_HOST.' >&2
  exit 2
fi
export ROS_MASTER_URI="$STANDBY_TRACKER_URI"
export LD_LIBRARY_PATH="$STANDBY_RELEASE_DIR/lib:/opt/ros/noetic/lib"
export ROS_PACKAGE_PATH="$STANDBY_RELEASE_DIR:/opt/ros/noetic/share"
export ROS_NAMESPACE=/standby_trial_20261105_feedback_02/perception
# Never let roslaunch create a replacement master on an unintended port.
/usr/bin/python3 - "$ROS_MASTER_URI" <<'PY'
import sys,xmlrpc.client
class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self,host):
        c=super().make_connection(host);c.timeout=2.;return c
try:
    result=xmlrpc.client.ServerProxy(sys.argv[1],transport=TimeoutTransport()).getPid('/standby_private_tracker_preflight')
    assert result[0]==1
except Exception as exc:
    raise SystemExit('Explicit existing ROS master unavailable; no tracker started: '+str(exc))
PY
echo 'Starting perception only. Slow-target thresholds and model geometry are unvalidated.'
echo 'All perception outputs: /standby_trial_20261105_feedback_02/perception/ ; Ctrl-C stops this foreground tracker.'
exec /opt/ros/noetic/bin/roslaunch "$STANDBY_RELEASE_DIR/config/private_tracker.launch"
