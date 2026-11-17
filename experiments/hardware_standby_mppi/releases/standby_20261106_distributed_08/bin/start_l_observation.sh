#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.sh"
exec /usr/bin/python3 "$STANDBY_DISTRIBUTED_RELEASE/tools/run_distributed_observation.py" "$@"
