#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
MANIFEST="${ROBONIX_MANIFEST:-$DEPLOY_DIR/robonix_manifest.yaml}"
RUN_DIR="$DEPLOY_DIR/.run"
LOG_DIR="$DEPLOY_DIR/logs"
mkdir -p "$RUN_DIR" "$LOG_DIR"
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
export ROBONIX_DEPLOY_DIR="$DEPLOY_DIR"

if [[ -f "$DEPLOY_DIR/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$DEPLOY_DIR/.env"
  set +a
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ROBONIX_RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION"

[[ -f "/opt/ros/humble/setup.bash" ]] || {
  echo "missing /opt/ros/humble/setup.bash; install ROS 2 Humble" >&2
  exit 1
}
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

command -v rbnx >/dev/null 2>&1 || {
  echo "rbnx not found; install Robonix and check PATH" >&2
  exit 1
}

# Return this deployment to a known clean state before boot. This also reaps
# package children orphaned by an earlier partial Soma bring-up.
"$DEPLOY_DIR/stop.sh"

if ss -H -lunp 2>/dev/null | awk '$5 ~ /:43897$/ { found=1 } END { exit !found }'; then
  echo "UDP 43897 is occupied; Lite3 odometry cannot start" >&2
  ss -H -lunp 2>/dev/null | awk '$5 ~ /:43897$/' >&2
  exit 1
fi

stack_started=0
router_pid=""

cleanup() {
  local status=$?
  trap - EXIT INT TERM

  if [[ "$stack_started" == "1" ]]; then
    rbnx shutdown -f "$MANIFEST" || true
  fi

  if [[ -n "$router_pid" ]]; then
    kill -TERM "$router_pid" 2>/dev/null || true
    wait "$router_pid" 2>/dev/null || true
  fi

  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ "$RMW_IMPLEMENTATION" == "rmw_zenoh_cpp" ]]; then
  router_bin="/opt/ros/humble/lib/rmw_zenoh_cpp/rmw_zenohd"
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
    "$router_bin" >"$LOG_DIR/rmw_zenohd.log" 2>&1 &
    router_pid=$!
    echo "$router_pid" >"$RUN_DIR/rmw_zenohd.pid"
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
      if ! kill -0 "$router_pid" 2>/dev/null; then
        tail -80 "$LOG_DIR/rmw_zenohd.log" >&2 || true
        exit 1
      fi
      sleep 0.25
    done
  fi
fi

stack_started=1
rbnx boot --no-update-check -f "$MANIFEST" "$@"
