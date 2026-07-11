#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
export ROBONIX_DEPLOY_DIR="$DEPLOY_DIR"

[[ ! -f "$DEPLOY_DIR/.env" ]] || { set -a; source "$DEPLOY_DIR/.env"; set +a; }

exec rbnx shutdown -f "$DEPLOY_DIR/robonix_manifest.yaml" "$@"
