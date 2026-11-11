#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="${ROBONIX_DEPLOY_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"
WORKSPACE_DIR="$(cd "$DEPLOY_DIR/.." && pwd)"
SCENE_SRC="${ROBONIX_SOURCE_DIR:-$WORKSPACE_DIR/robonix}/system/scene"
IMAGE="${ROBONIX_SCENE_IMAGE:-robonix-scene-lite}"

rbnx codegen -p "$SCENE_SRC" --mcp --ros2
docker build --network=host --pull=false \
  -f "$DEPLOY_DIR/sim/scene/Dockerfile" \
  -t "$IMAGE" \
  "$DEPLOY_DIR/sim/scene"

idl="$SCENE_SRC/rbnx-build/codegen/ros2_idl"
if [[ -d "$idl/src/map" ]]; then
  docker run --rm --entrypoint bash --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$SCENE_SRC:/scene" "$IMAGE" -lc \
    'source /opt/ros/${ROS_DISTRO:-humble}/setup.bash && cd /scene/rbnx-build/codegen/ros2_idl && colcon build --packages-up-to map'
fi

echo "[scene-lite/build] image=$IMAGE ready"
