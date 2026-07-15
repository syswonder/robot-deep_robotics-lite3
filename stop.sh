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

CACHE_ROOT="$DEPLOY_DIR/rbnx-boot/cache"

collect_deployment_pids() {
  local pid cwd
  while read -r pid; do
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    [[ "$pid" != "$$" && "$pid" != "$PPID" ]] || continue
    cwd="$(readlink -f "/proc/$pid/cwd" 2>/dev/null || true)"
    if [[ "$cwd" == "$CACHE_ROOT" || "$cwd" == "$CACHE_ROOT/"* ]]; then
      printf '%s\n' "$pid"
    fi
  done < <(ps -u "$(id -u)" -o pid=)
}

terminate_deployment_orphans() {
  local -a pids=() alive=()
  local pid
  mapfile -t pids < <(collect_deployment_pids)
  ((${#pids[@]})) || return 0

  echo "[stop] terminating deployment-owned residual processes: ${pids[*]}"
  kill -TERM "${pids[@]}" 2>/dev/null || true
  for _ in {1..30}; do
    alive=()
    for pid in "${pids[@]}"; do
      kill -0 "$pid" 2>/dev/null && alive+=("$pid")
    done
    ((${#alive[@]} == 0)) && break
    sleep 0.2
  done

  if ((${#alive[@]})); then
    echo "[stop] force-killing deployment-owned residual processes: ${alive[*]}"
    kill -KILL "${alive[@]}" 2>/dev/null || true
  fi
}

# Normal teardown is authoritative. Do not match and kill generic ROS/Nav2
# process names because another deployment may be using the same host.
if [[ -f "$DEPLOY_DIR/rbnx-boot/state.json" ]]; then
  rbnx shutdown -f "$MANIFEST" "$@"
else
  echo "[stop] no Robonix boot state; checking deployment-owned residuals"
fi

# A package wrapper can fail before Robonix records or tears down all of its
# ROS children. Match residuals by cwd under this deployment's cache, not by
# generic ROS/Nav2 process names, so other deployments on the host are safe.
terminate_deployment_orphans

# Fast DDS' cleaner removes stale shared-memory artifacts while preserving
# segments that still belong to live participants.
if [[ "$RMW_IMPLEMENTATION" == "rmw_fastrtps_cpp" && -x /opt/ros/humble/bin/fastdds ]]; then
  /opt/ros/humble/bin/fastdds shm clean >/dev/null 2>&1 || true
fi

# jetson2motion must exclusively bind the Lite3 feedback port. Never kill an
# unknown owner here; report it so the operator can inspect it safely.
if ss -H -lunp 2>/dev/null | awk '$5 ~ /:43897$/ { found=1 } END { exit !found }'; then
  echo "[stop] ERROR: UDP 43897 is still occupied after scoped cleanup" >&2
  ss -H -lunp 2>/dev/null | awk '$5 ~ /:43897$/' >&2
  exit 1
fi

echo "[stop] deployment stopped cleanly"
