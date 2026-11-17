#!/usr/bin/env bash
set -euo pipefail
exec "$(dirname -- "${BASH_SOURCE[0]}")/../sensors/start_sensor_stage.sh" raw "$@"
