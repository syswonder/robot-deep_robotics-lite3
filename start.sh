#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$DEPLOY_DIR/scripts/common.sh"

mode=""
viewer="headless"
declare -a rbnx_args=()
while (($#)); do
  case "$1" in
    --mode)
      (($# >= 2)) || { echo "--mode requires a value" >&2; exit 2; }
      mode="$2"; shift 2 ;;
    --mode=*) mode="${1#*=}"; shift ;;
    --viewer)
      (($# >= 2)) || { echo "--viewer requires a value" >&2; exit 2; }
      viewer="$2"; shift 2 ;;
    --viewer=*) viewer="${1#*=}"; shift ;;
    --headless) viewer="headless"; shift ;;
    -h|--help)
      echo "usage: $0 --mode {hardware|simulation} [--headless|--viewer web] [rbnx boot options]"
      exit 0 ;;
    *) rbnx_args+=("$1"); shift ;;
  esac
done

require_mode "$mode"
[[ "$viewer" == "headless" || "$viewer" == "web" ]] || {
  echo "--viewer supports only web" >&2; exit 2;
}
[[ "$mode" == "simulation" || "$viewer" == "headless" ]] || {
  echo "--viewer is available only in simulation mode" >&2; exit 2;
}

load_deployment_env
require_vlm_config
require_rbnx
mkdir -p "$RUN_DIR" "$DEPLOY_DIR/logs/$mode"
manifest="$(deployment_manifest "$mode")"
validate_deployment_manifest "$mode" "$manifest"

if [[ -f "$RUN_DIR/mode" ]]; then
  active="$(cat "$RUN_DIR/mode")"
  echo "another deployment is active (mode=$active); run ./stop.sh --mode $active" >&2
  exit 1
fi

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  "$DEPLOY_DIR/stop.sh" --mode "$mode" || true
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ "$mode" == "simulation" ]]; then
  primitive_cache="$RUN_DIR/cache/primitive-deep_robotics-lite3_quadruped"
  [[ -f "$primitive_cache/package_manifest.yaml" ]] || {
    echo "simulation packages are not built; run: ./build.sh --mode simulation" >&2
    exit 1
  }
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROBONIX_RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROBONIX_LITE3_MODE=simulation
  export LITE3_VIEWER="$viewer"
  export LITE3_ENVIRONMENT="${LITE3_ENVIRONMENT:-scenesmith_house_187}"
  [[ "$viewer" != "web" ]] || "$DEPLOY_DIR/sim/manage.sh" web3d-install
  "$DEPLOY_DIR/sim/manage.sh" up
  if [[ "$viewer" == "web" ]]; then
    echo "[start] web viewer: http://127.0.0.1:${LITE3_VIEWER_PORT:-8080}/"
  fi
else
  source_ros_humble
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
  export ROBONIX_RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION"
  if ss -H -lunp 2>/dev/null | awk '$5 ~ /:43897$/ { found=1 } END { exit !found }'; then
    echo "UDP 43897 is occupied; Lite3 hardware odometry cannot start" >&2
    exit 1
  fi
  if [[ "$RMW_IMPLEMENTATION" == "rmw_zenoh_cpp" ]]; then
    router_bin=/opt/ros/humble/lib/rmw_zenoh_cpp/rmw_zenohd
    [[ -x "$router_bin" ]] || {
      echo "missing $router_bin; install ros-humble-rmw-zenoh-cpp" >&2
      exit 1
    }
    if ! python3 - <<'PY'
import socket
try:
    with socket.create_connection(("127.0.0.1", 7447), timeout=0.25):
        pass
except OSError:
    raise SystemExit(1)
PY
    then
      "$router_bin" >"$DEPLOY_DIR/logs/hardware/rmw_zenohd.log" 2>&1 &
      router_pid=$!
      echo "$router_pid" >"$RUN_DIR/rmw_zenohd.hardware.pid"
      for _ in $(seq 1 40); do
        if python3 - <<'PY'
import socket
try:
    with socket.create_connection(("127.0.0.1", 7447), timeout=0.25):
        pass
except OSError:
    raise SystemExit(1)
PY
        then
          break
        fi
        kill -0 "$router_pid" 2>/dev/null || {
          tail -80 "$DEPLOY_DIR/logs/hardware/rmw_zenohd.log" >&2 || true
          exit 1
        }
        sleep 0.25
      done
    fi
  fi
fi

printf '%s\n' "$mode" >"$RUN_DIR/mode"
echo "[start] mode=$mode manifest=$manifest"
rbnx boot --no-update-check -f "$manifest" "${rbnx_args[@]}"
