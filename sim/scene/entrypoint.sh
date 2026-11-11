#!/usr/bin/env bash
set -eo pipefail

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
overlay=/scene/rbnx-build/codegen/ros2_idl/install/setup.bash
[[ ! -f "$overlay" ]] || source "$overlay"

cd /scene
proto=/scene/rbnx-build/codegen/scene_proto_gen
[[ -f "$proto/atlas_pb2.py" ]] || {
  echo "[scene-lite] missing runtime protobuf stubs; run ./build.sh --mode simulation" >&2
  exit 1
}
export PYTHONPATH="$proto:/scene/rbnx-build/codegen/robonix_mcp_types:/robonix-api:${PYTHONPATH:-}"
mkdir -p /scene/rbnx-build/data
exec python3 -m scene_service.service
