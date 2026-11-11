#!/usr/bin/env bash
set -euo pipefail

container="${LITE3_SIM_CONTAINER:-lite3-sim}"
[[ "$(docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null || true)" == "true" ]] || {
  echo "Lite3 simulation is not running: $container" >&2
  exit 1
}

exec docker exec -it "$container" bash -lc '
  source /opt/ros/humble/setup.bash
  exec python3 /workspace/lite3-validation/teleop_twist.py
'
