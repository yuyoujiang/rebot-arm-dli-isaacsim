#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONUNBUFFERED=1
echo "[notice] The legacy autonomous trajectory is disabled; running the stationery teleoperation regression test." >&2
exec "${SCRIPT_DIR}/run.sh" --test "$@"
