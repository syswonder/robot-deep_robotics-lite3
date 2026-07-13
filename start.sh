#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
export ROBONIX_DEPLOY_DIR="$DEPLOY_DIR"

[[ ! -f "$DEPLOY_DIR/.env" ]] || { set -a; source "$DEPLOY_DIR/.env"; set +a; }

# A failed/interrupted boot may leave package processes alive even though
# Robonix has already removed its deployment state. In particular, a stale
# jetson2motion keeps UDP 43897 bound and makes the next Soma stage-1 startup
# time out. stop.sh is intentionally idempotent, so every start first returns
# this deployment to a known clean state.
"$DEPLOY_DIR/stop.sh"

set +u
source /opt/ros/humble/setup.bash
set -u

exec rbnx boot -f "$DEPLOY_DIR/robonix_manifest.yaml" "$@"
