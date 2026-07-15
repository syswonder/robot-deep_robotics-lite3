#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
MANIFEST="${ROBONIX_MANIFEST:-$DEPLOY_DIR/robonix_manifest.yaml}"

export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
export ROBONIX_DEPLOY_DIR="$DEPLOY_DIR"
export UV_CACHE_DIR="${UV_CACHE_DIR:-$DEPLOY_DIR/.cache/uv}"
export PIP_CACHE_DIR="${PIP_CACHE_DIR:-$DEPLOY_DIR/.cache/pip}"

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

exec rbnx build -f "$MANIFEST" "$@"
