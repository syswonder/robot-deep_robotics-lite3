#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
MANIFEST="${ROBONIX_MANIFEST:-$DEPLOY_DIR/robonix_manifest.yaml}"
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
export ROBONIX_DEPLOY_DIR="$DEPLOY_DIR"

if [[ -f "$DEPLOY_DIR/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$DEPLOY_DIR/.env"
  set +a
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROBONIX_RMW_IMPLEMENTATION="${ROBONIX_RMW_IMPLEMENTATION:-$RMW_IMPLEMENTATION}"

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

cleanup() {
  local status=$?
  trap - EXIT INT TERM

  if [[ "$stack_started" == "1" ]]; then
    rbnx shutdown -f "$MANIFEST" || true
  fi

  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

stack_started=1
rbnx boot --no-update-check -f "$MANIFEST" "$@"
