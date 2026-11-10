#!/usr/bin/env bash
set -euo pipefail

SIM_DIR="$(cd "$(dirname "$0")" && pwd)"
DEPLOY_DIR="$(cd "$SIM_DIR/.." && pwd)"
WORKSPACE_DIR="$(cd "$DEPLOY_DIR/.." && pwd)"
COMPOSE_FILE="$SIM_DIR/compose.yaml"
VENDOR_DIR="${LITE3_VENDOR_DIR:-$WORKSPACE_DIR/Lite3_rl_deploy}"
MODEL_DIR="$VENDOR_DIR/third_party/deep_robotics_model"
WEB3D_DIR="$SIM_DIR/web3d"
SIM_CONTAINER="${LITE3_SIM_CONTAINER:-lite3-sim}"
NAV_CONTAINER="${ROBONIX_NAV2_CONTAINER:-robonix_nav2}"

export ROBONIX_DEPLOY_DIR="${ROBONIX_DEPLOY_DIR:-$DEPLOY_DIR}"
export ROBONIX_SOURCE_DIR="${ROBONIX_SOURCE_DIR:-$WORKSPACE_DIR/robonix}"
export LITE3_VENDOR_DIR="$VENDOR_DIR"

[[ -f "$VENDOR_DIR/CMakeLists.txt" ]] || {
  echo "Lite3_rl_deploy must be a sibling of robot-deep_robotics-lite3" >&2
  exit 1
}
[[ -r /dev/dri/renderD128 ]] || {
  echo "/dev/dri/renderD128 is not readable" >&2
  exit 1
}

export SIM_UID="$(id -u)"
export SIM_GID="$(id -g)"
export RENDER_GID="$(stat -c '%g' /dev/dri/renderD128)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

compose() {
  docker compose -f "$COMPOSE_FILE" "$@"
}

apply_patch_idempotently() {
  local repo_dir="$1" patch_file="$2" label="$3"
  if git -C "$repo_dir" apply --ignore-space-change --check "$patch_file" 2>/dev/null; then
    git -C "$repo_dir" apply --ignore-space-change "$patch_file"
    echo "Applied $label patch: $(basename "$patch_file")"
  elif git -C "$repo_dir" apply --ignore-space-change --reverse --check "$patch_file" 2>/dev/null; then
    echo "$label patch already applied: $(basename "$patch_file")"
  else
    echo "$label does not match sim/vendor.lock or has conflicting edits" >&2
    exit 1
  fi
}

apply_pinned_patch() {
  local repo_dir="$1" lock_key="$2" patch_file="$3" label="$4"
  git -C "$repo_dir" rev-parse --git-dir >/dev/null 2>&1 || {
    echo "$label must be a sibling of robot-deep_robotics-lite3" >&2
    exit 1
  }
  local expected actual
  expected="$(awk -F= -v key="$lock_key" '$1 == key {print $2}' "$SIM_DIR/vendor.lock")"
  actual="$(git -C "$repo_dir" rev-parse HEAD)"
  [[ -n "$expected" && "$actual" == "$expected" ]] || {
    echo "$label revision $actual does not match sim/vendor.lock ($expected)" >&2
    exit 1
  }
  apply_patch_idempotently "$repo_dir" "$patch_file" "$label"
}

prepare_web3d() {
  [[ -f "$WEB3D_DIR/package-lock.json" ]] || {
    echo "missing Web3D package lock: $WEB3D_DIR/package-lock.json" >&2
    exit 1
  }
  if [[ ! -f "$WEB3D_DIR/node_modules/mujoco-js/dist/mujoco_wasm.js" ]]; then
    command -v node >/dev/null && command -v npm >/dev/null || {
      echo "Node.js 20+ and npm are required for the Web3D viewer" >&2
      exit 1
    }
    echo "[simulation] installing pinned Web3D runtime dependencies"
    npm --prefix "$WEB3D_DIR" ci --omit=dev --no-audit --no-fund \
      --registry=https://registry.npmmirror.com --replace-registry-host=always
  fi
}

prepare_sources() {
  apply_pinned_patch "$VENDOR_DIR" simulation_commit \
    "$SIM_DIR/patches/lite3-controller.patch" Lite3_rl_deploy
  apply_pinned_patch "$MODEL_DIR" model_commit \
    "$SIM_DIR/patches/simulated-sensors.patch" deep_robotics_model
}

build_controller() {
  compose run --rm lite3-sim bash -lc '
    source /opt/ros/humble/setup.bash
    cmake -S . -B build-simulation \
      -DBUILD_PLATFORM=x86 \
      -DBUILD_SIM=ON \
      -DSEND_REMOTE=OFF \
    && cmake --build build-simulation --parallel
  '
}

case "${1:-}" in
  build)
    prepare_web3d
    prepare_sources
    compose build
    build_controller
    ;;
  web3d-install)
    prepare_web3d
    ;;
  bridge-smoke)
    prepare_sources
    compose run --rm lite3-sim \
      bash /workspace/lite3-validation/bridge_smoke.sh
    ;;
  sensor-acceptance)
    [[ "$(docker inspect -f '{{.State.Running}}' "$SIM_CONTAINER" 2>/dev/null || true)" == "true" ]] || {
      echo "simulation is not running; start it with: ./start.sh --mode simulation" >&2
      exit 1
    }
    docker exec "$SIM_CONTAINER" bash -lc '
      source /opt/ros/humble/setup.bash
      python3 /workspace/lite3-validation/sensor_acceptance.py
    '
    ;;
  yaw-acceptance)
    [[ "$(docker inspect -f '{{.State.Running}}' "$SIM_CONTAINER" 2>/dev/null || true)" == "true" ]] || {
      echo "Lite3 simulation is not running" >&2
      exit 1
    }
    shift
    for argument in "$@"; do
      if [[ "$argument" == "--reset-between" && -f "$DEPLOY_DIR/rbnx-boot/mode" ]]; then
        echo "--reset-between requires the standalone simulator; stop the full stack first" >&2
        exit 1
      fi
    done
    docker exec "$SIM_CONTAINER" bash -lc '
      source /opt/ros/humble/setup.bash
      exec python3 /workspace/lite3-validation/yaw_acceptance.py "$@"
    ' yaw-acceptance "$@"
    ;;
  navigation-acceptance)
    [[ "$(docker inspect -f '{{.State.Running}}' "$SIM_CONTAINER" 2>/dev/null || true)" == "true" ]] || {
      echo "Lite3 simulation is not running" >&2
      exit 1
    }
    [[ "$(docker inspect -f '{{.State.Running}}' "$NAV_CONTAINER" 2>/dev/null || true)" == "true" ]] || {
      echo "Nav2 container is not running: $NAV_CONTAINER" >&2
      exit 1
    }
    goal_distance="${LITE3_NAV_GOAL_DISTANCE:-0.60}"
    goal_bearing_deg="${LITE3_NAV_GOAL_BEARING_DEG:-30.0}"
    docker exec \
      -e LITE3_NAV_GOAL_DISTANCE="$goal_distance" \
      -e LITE3_NAV_GOAL_BEARING_DEG="$goal_bearing_deg" \
      -i "$NAV_CONTAINER" bash -lc '
        source /opt/ros/humble/setup.bash
        python3 -
      ' < "$SIM_DIR/validation/navigation_acceptance.py"
    mapping_node="$(docker exec "$NAV_CONTAINER" bash -lc '
      source /opt/ros/humble/setup.bash
      ros2 node list | grep -Ex "/(slam_toolbox|rtabmap)" | head -n 1
    ')"
    [[ -n "$mapping_node" ]] || {
      echo "Neither /slam_toolbox nor /rtabmap is running" >&2
      exit 1
    }
    for node in \
      /planner_server /controller_server /bt_navigator /velocity_smoother \
      /local_costmap/local_costmap /global_costmap/global_costmap "$mapping_node"; do
      value="$(docker exec "$NAV_CONTAINER" bash -lc "
        source /opt/ros/humble/setup.bash
        timeout 8 ros2 param get '$node' use_sim_time
      ")"
      grep -Fq "Boolean value is: True" <<<"$value" || {
        echo "$node does not use simulation time: $value" >&2
        exit 1
      }
    done
    echo "SIMULATION_TIME_OK"
    ;;
  up)
    prepare_sources
    [[ -x "$VENDOR_DIR/build-simulation/rl_deploy" ]] || {
      echo "controller is not built; run: ./build.sh --mode simulation" >&2
      exit 1
    }
    docker rm -f "$SIM_CONTAINER" >/dev/null 2>&1 || true
    compose run -d --name "$SIM_CONTAINER" lite3-sim \
      bash /workspace/lite3-runtime/run_bridge.sh >/dev/null
    for _ in $(seq 1 24); do
      if ! docker inspect "$SIM_CONTAINER" >/dev/null 2>&1; then
        echo "simulation container exited during startup" >&2
        exit 1
      fi
      if [[ "$(docker inspect -f '{{.State.Running}}' "$SIM_CONTAINER")" != "true" ]]; then
        docker logs --tail 120 "$SIM_CONTAINER" >&2 || true
        docker rm -f "$SIM_CONTAINER" >/dev/null 2>&1 || true
        echo "simulation container exited during startup" >&2
        exit 1
      fi
      if docker exec "$SIM_CONTAINER" bash -lc '
        source /opt/ros/humble/setup.bash
        timeout 5 ros2 topic echo --once /clock rosgraph_msgs/msg/Clock >/dev/null 2>&1
      '; then
        echo "[simulation] simulator and ROS bridge are ready"
        exit 0
      fi
      sleep 1
    done
    docker logs --tail 120 "$SIM_CONTAINER" >&2 || true
    docker rm -f "$SIM_CONTAINER" >/dev/null 2>&1 || true
    echo "simulation bridge did not become ready" >&2
    exit 1
    ;;
  down)
    docker rm -f "$SIM_CONTAINER" >/dev/null 2>&1 || true
    echo "[simulation] simulator stopped"
    ;;
  status)
    docker ps --filter "name=^/${SIM_CONTAINER}$" --format '{{.Names}} {{.Status}}'
    ;;
  *)
    echo "usage: $0 {build|web3d-install|bridge-smoke|sensor-acceptance|yaw-acceptance|navigation-acceptance|up|down|status}" >&2
    exit 2
    ;;
esac
