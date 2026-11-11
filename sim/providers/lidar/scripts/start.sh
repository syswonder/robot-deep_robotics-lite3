#!/usr/bin/env bash
set -euo pipefail

container="${LITE3_SIM_CONTAINER:-lite3-sim}"
package_root="$(cd "$(dirname "$0")/.." && pwd)"
docker inspect "$container" >/dev/null 2>&1 || {
  echo "simulation container is not running: $container" >&2
  exit 1
}

exec docker exec -i \
  -e RBNX_PACKAGE_ROOT="$package_root" \
  -e RBNX_INSTANCE_NAME="${RBNX_INSTANCE_NAME:-lite3_sim_lidar}" \
  -e ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}" \
  -e ROBONIX_META_GRPC_ENDPOINT="${ROBONIX_META_GRPC_ENDPOINT:-127.0.0.1:50051}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  "$container" bash -lc '
    source /opt/ros/humble/setup.bash
    export PYTHONPATH="/workspace/robonix/pylib/robonix-api:$RBNX_PACKAGE_ROOT:${PYTHONPATH:-}"
    cd "$RBNX_PACKAGE_ROOT"
    exec python3 -m lite3_sim_lidar.main
  '
