#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

REPO=/workspace/Lite3_rl_deploy
controller_pid=""
cleanup() {
  [[ -z "$controller_pid" ]] || kill "$controller_pid" 2>/dev/null || true
  [[ -z "$controller_pid" ]] || wait "$controller_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

(
  cd "$REPO/build-simulation"
  export LITE3_COMMAND_INTERFACE=udp
  export LD_LIBRARY_PATH=../third_party/onnxruntime/x86/lib
  exec ./rl_deploy
) &
controller_pid=$!

python3 -u /workspace/lite3-runtime/lite3_bridge.py \
  --viewer "${LITE3_VIEWER:-headless}" \
  --viewer-port "${LITE3_VIEWER_PORT:-8080}" \
  "$@"
