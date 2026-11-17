#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.sh"
exec /usr/bin/python3 "$STANDBY_RELEASE_DIR/bin/session.py" start-feedback "$@"
