#!/usr/bin/env bash
set -euo pipefail
STANDBY_SENSOR_STAGE="$1"
shift
source "$(dirname -- "${BASH_SOURCE[0]}")/../bin/environment.sh"
if [[ $# -ne 2 && $# -ne 4 ]] || [[ "${1:-}" != --master-uri ]]; then
  echo 'Usage: <sensor wrapper> --master-uri http://HOST:PORT [--advertise-host THIS_HOST]' >&2
  exit 2
fi
if [[ ! "$2" =~ ^http://([A-Za-z0-9_.-]+):([0-9]+)/?$ ]]; then
  echo 'Explicit HTTP ROS master URI required.' >&2; exit 2
fi
STANDBY_SENSOR_HOST="${BASH_REMATCH[1]}"
export ROS_MASTER_URI="$2"
if [[ "$STANDBY_SENSOR_HOST" == 127.0.0.1 || "$STANDBY_SENSOR_HOST" == localhost ]]; then
  export ROS_IP=127.0.0.1 ROS_HOSTNAME=localhost
elif [[ $# -eq 4 && "$3" == --advertise-host && "$4" =~ ^[A-Za-z0-9_.-]+$ ]]; then
  export ROS_HOSTNAME="$4"
else
  echo 'Remote master also requires --advertise-host THIS_HOST.' >&2; exit 2
fi
export ROS_PACKAGE_PATH="$STANDBY_RELEASE_DIR:/opt/ros/noetic/share"
export LD_LIBRARY_PATH="$STANDBY_RELEASE_DIR/lib:/opt/ros/noetic/lib"
export ROS_NAMESPACE=/standby_trial_20261105_feedback_02/sensors
/usr/bin/python3 - "$ROS_MASTER_URI" <<'PY'
import sys,xmlrpc.client
class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self,host):
        c=super().make_connection(host);c.timeout=2.;return c
try:
    result=xmlrpc.client.ServerProxy(sys.argv[1],transport=TimeoutTransport()).getPid('/feedback02_sensor_preflight')
    assert result[0]==1
except Exception as exc:
    raise SystemExit('Explicit ROS master unavailable; no sensor started: '+str(exc))
PY
case "$STANDBY_SENSOR_STAGE" in
  raw) STANDBY_SENSOR_LAUNCH=raw_sensors.launch ;;
  odometry)
    test -x "$STANDBY_RELEASE_DIR/sensors/bin/feedback02_fastlio_mapping" || { echo 'Isolated FASTLIO build not complete.' >&2; exit 2; }
    STANDBY_SENSOR_LAUNCH=sensor_odometry.launch ;;
  *) echo 'Unknown sensor stage.' >&2; exit 2 ;;
esac
echo "Starting perception-only stage: $STANDBY_SENSOR_STAGE. No base, arm, planner, ICP or enable nodes."
exec /opt/ros/noetic/bin/roslaunch "$STANDBY_RELEASE_DIR/sensors/$STANDBY_SENSOR_LAUNCH"
