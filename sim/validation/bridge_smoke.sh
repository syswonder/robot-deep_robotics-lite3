#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u
REPO=/workspace/Lite3_rl_deploy
LOG_DIR="${TMPDIR:-/tmp}/lite3-simulation"
BRIDGE_LOG="$LOG_DIR/bridge.log"
CONTROLLER_LOG="$LOG_DIR/controller.log"
mkdir -p "$LOG_DIR"
: >"$BRIDGE_LOG"
: >"$CONTROLLER_LOG"

bridge_pid=""
controller_pid=""
cleanup() {
  [[ -z "$controller_pid" ]] || kill "$controller_pid" 2>/dev/null || true
  [[ -z "$bridge_pid" ]] || kill "$bridge_pid" 2>/dev/null || true
  [[ -z "$controller_pid" ]] || wait "$controller_pid" 2>/dev/null || true
  [[ -z "$bridge_pid" ]] || wait "$bridge_pid" 2>/dev/null || true
}
trap cleanup EXIT

python3 -u /workspace/lite3-runtime/lite3_bridge.py >"$BRIDGE_LOG" 2>&1 &
bridge_pid=$!

for _ in $(seq 1 60); do
  if ros2 topic list 2>/dev/null | grep -Fxq /clock; then break; fi
  if ! kill -0 "$bridge_pid" 2>/dev/null; then
    cat "$BRIDGE_LOG" >&2
    exit 1
  fi
  sleep 0.25
done
ros2 topic list | grep -Fxq /clock || {
  echo "Bridge did not publish /clock" >&2
  cat "$BRIDGE_LOG" >&2
  exit 1
}

(
  cd "$REPO/build-simulation"
  export LITE3_COMMAND_INTERFACE=udp
  export LD_LIBRARY_PATH=../third_party/onnxruntime/x86/lib
  exec ./rl_deploy
) >"$CONTROLLER_LOG" 2>&1 &
controller_pid=$!

python3 /workspace/lite3-validation/bridge_acceptance.py
for service in reset stand lie_down emergency_stop; do
  ros2 service list | grep -Fxq "/lite3_sim/$service" || {
    echo "Missing simulation service: /lite3_sim/$service" >&2
    exit 1
  }
done
sleep 3

required=(
  "LITE3_COMMAND_INTERFACE=udp"
  "[ONNX INIT] Model loaded successfully."
  "Robot interface started"
  "idle_state ------------> standup_state"
  "standup_state ------------> rl_control"
)
for expected in "${required[@]}"; do
  grep -Fq "$expected" "$CONTROLLER_LOG" || {
    echo "Missing controller milestone: $expected" >&2
    cat "$CONTROLLER_LOG" >&2
    exit 1
  }
done

stop_response="$(timeout 5s ros2 service call \
  /lite3_sim/emergency_stop std_srvs/srv/Trigger '{}')"
grep -Fq "success=True" <<<"$stop_response" || {
  echo "Emergency-stop service failed: $stop_response" >&2
  exit 1
}
sleep 0.5
grep -Fq "rl_control ------------> joint_damping" "$CONTROLLER_LOG" || {
  echo "Controller did not enter joint damping after emergency stop" >&2
  cat "$CONTROLLER_LOG" >&2
  exit 1
}

reset_response="$(timeout 5s ros2 service call \
  /lite3_sim/reset std_srvs/srv/Trigger '{}')"
grep -Fq "success=True" <<<"$reset_response" || {
  echo "Reset service failed: $reset_response" >&2
  exit 1
}
if grep -Eiq '(^|[^[:alpha:]])(nan|inf)([^[:alpha:]]|$)' "$BRIDGE_LOG"; then
  echo "Bridge produced a non-finite value" >&2
  tail -120 "$BRIDGE_LOG" >&2
  exit 1
fi

echo "Controller milestones:"
grep -F -e "LITE3_COMMAND_INTERFACE=udp" \
  -e "idle_state ------------> standup_state" \
  -e "standup_state ------------> rl_control" \
  -e "rl_control ------------> joint_damping" "$CONTROLLER_LOG"
echo "Services: emergency_stop=OK reset=OK"
echo "BRIDGE_SMOKE_OK"
