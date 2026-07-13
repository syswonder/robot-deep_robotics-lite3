#!/usr/bin/env bash
set -uo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
export ROBONIX_DEPLOY_DIR="$DEPLOY_DIR"

[[ ! -f "$DEPLOY_DIR/.env" ]] || { set -a; source "$DEPLOY_DIR/.env"; set +a; }

# Normal path. A failed boot removes state.json, so shutdown may legitimately
# fail even though Soma children are still alive; cleanup below is therefore
# always executed.
rbnx shutdown -f "$DEPLOY_DIR/robonix_manifest.yaml" "$@" || true

# Terminate only processes whose command line belongs to this deployment's
# local packages. Soma normally owns these; this is the recovery path for an
# interrupted/failed boot where Soma did not get a chance to reap children.
# Canonicalize this path: pgrep compares it with normalized paths in process
# command lines, so a literal "robot-deep_robotics-lite3/.." never matches.
LOCAL_ROOT="$(cd "$DEPLOY_DIR/.." && pwd)"
patterns=(
  "$LOCAL_ROOT/primitive-deep_robotics-lite3_quadruped"
  "$LOCAL_ROOT/lite3_rbnx_ws/primitives/mid360_lidar_rbnx"
  "$LOCAL_ROOT/lite3_rbnx_ws/services/rtabmap_rbnx"
  "$LOCAL_ROOT/robonix/system/scene"
  "$DEPLOY_DIR/rbnx-boot/cache/"
  "/opt/ros/humble/bin/ros2 launch transfer transfer_launch.py"
  "/opt/ros/humble/bin/ros2 launch livox_ros_driver2 msg_MID360_launch.py"
  "/opt/ros/humble/bin/ros2 launch orbbec_camera"
  # Python module entry points do not retain their checkout path in argv. If
  # their Soma parent is interrupted they are re-parented to PID 1, so match
  # the deployment-specific module names as well as the paths above.
  "python3 -m lite3_quadruped.main"
  "python3 -m mid360_driver.main"
  "python3 -m lite3_description.main"
  "python3 -m pcld2lscan_rbnx.atlas_bridge"
  "python3 -m rtabmap_rbnx.main"
  "python3 -m nav2_wrapper.atlas_bridge"
  "python3 -m nav2_wrapper.velocity_guard"
  "python3 -m orbbec_camera.main"
  "python3 -m scene_service.service"
  # Orphaned Nav2 launch children likewise contain no deployment path. This
  # robot runs a single Nav2 stack, and leaving a second lifecycle manager or
  # server alive causes duplicate node names and bond-heartbeat resets.
  "/opt/ros/humble/lib/nav2_"
)

collect_pids() {
  local pattern pid
  for pattern in "${patterns[@]}"; do
    while read -r pid; do
      [[ -n "$pid" && "$pid" != "$$" && "$pid" != "$PPID" ]] && echo "$pid"
    done < <(pgrep -f -- "$pattern" 2>/dev/null || true)
  done | sort -u
}

mapfile -t residual_pids < <(collect_pids)
if ((${#residual_pids[@]})); then
  echo "[stop] terminating residual deployment processes: ${residual_pids[*]}"
  kill "${residual_pids[@]}" 2>/dev/null || true
  for _ in {1..30}; do
    alive=()
    for pid in "${residual_pids[@]}"; do
      kill -0 "$pid" 2>/dev/null && alive+=("$pid")
    done
    ((${#alive[@]} == 0)) && break
    sleep 0.2
  done
  if ((${#alive[@]})); then
    echo "[stop] force-killing residual processes: ${alive[*]}"
    kill -KILL "${alive[@]}" 2>/dev/null || true
  fi
fi

# Fast DDS can leave zombie shared-memory lock files after a process is
# interrupted or SIGKILLed. They prevent fresh ROS 2 participants from
# discovering each other even when all package processes have been stopped.
# Use Fast DDS' own cleaner so live shared-memory segments are preserved.
FASTDDS_BIN="/opt/ros/humble/bin/fastdds"
if [[ -x "$FASTDDS_BIN" ]]; then
  "$FASTDDS_BIN" shm clean >/dev/null 2>&1 || true
fi

# Remove only this deployment's stale ProcessManager rows. Preserve records
# owned by any other deployment sharing ~/.robonix/processes.json.
PROCESS_STATE="$HOME/.robonix/processes.json"
if [[ -f "$PROCESS_STATE" ]] && command -v jq >/dev/null 2>&1; then
  tmp="${PROCESS_STATE}.tmp.$$"
  jq --arg prefix "$DEPLOY_DIR/rbnx-boot/logs/" \
    '[.[] | select(((.log_file // "") | startswith($prefix)) | not)]' \
    "$PROCESS_STATE" > "$tmp" && mv "$tmp" "$PROCESS_STATE"
fi

# A clean restart requires the Lite3 UDP feedback port to be free.
if ss -lunp 2>/dev/null | grep -qE ':43897\\b'; then
  echo "[stop] ERROR: UDP 43897 is still occupied; refusing to report success" >&2
  ss -lunp 2>/dev/null | grep -E ':43897\\b' >&2 || true
  exit 1
fi

# Scene's web server is part of this deployment too. A stale native Scene
# process can survive an interrupted boot without an rbnx state record; the
# next Scene then reports ACTIVE before aborting because port 50107 is taken.
if ss -ltnp 2>/dev/null | grep -qE ':50107\\b'; then
  echo "[stop] ERROR: Scene web port 50107 is still occupied; refusing to report success" >&2
  ss -ltnp 2>/dev/null | grep -E ':50107\\b' >&2 || true
  exit 1
fi

echo "[stop] deployment stopped cleanly"
