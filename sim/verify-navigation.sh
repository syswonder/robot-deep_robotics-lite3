#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")/.." && pwd)"
distance="${1:-0.60}"
bearing="${2:-30.0}"

[[ "$distance" =~ ^[0-9]+([.][0-9]+)?$ ]] || {
  echo "distance must be a non-negative number in metres" >&2
  exit 2
}
[[ "$bearing" =~ ^-?[0-9]+([.][0-9]+)?$ ]] || {
  echo "bearing must be a number in degrees" >&2
  exit 2
}

LITE3_NAV_GOAL_DISTANCE="$distance" \
LITE3_NAV_GOAL_BEARING_DEG="$bearing" \
  exec "$DEPLOY_DIR/sim/manage.sh" navigation-acceptance
