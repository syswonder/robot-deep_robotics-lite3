#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$DEPLOY_DIR/scripts/common.sh"

mode=""
declare -a rbnx_args=()
while (($#)); do
  case "$1" in
    --mode)
      (($# >= 2)) || { echo "--mode requires a value" >&2; exit 2; }
      mode="$2"; shift 2 ;;
    --mode=*) mode="${1#*=}"; shift ;;
    -h|--help)
      echo "usage: $0 --mode {hardware|simulation} [rbnx build options]"
      exit 0 ;;
    *) rbnx_args+=("$1"); shift ;;
  esac
done

require_mode "$mode"
load_deployment_env
require_rbnx
manifest="$(deployment_manifest "$mode")"
validate_deployment_manifest "$mode" "$manifest"

echo "[build] mode=$mode manifest=$manifest"
if [[ "$mode" == "simulation" ]]; then
  export RBNX_CODEGEN_PYTHON="$ROBONIX_PYTHON_ENV/bin/python"
  # rbnx must fetch URL packages before Docker Compose evaluates the primitive
  # cache bind mount; otherwise Compose creates an empty directory first.
  rbnx build --no-update-check -f "$manifest" "${rbnx_args[@]}"
  exec "$DEPLOY_DIR/sim/manage.sh" build
fi
exec rbnx build --no-update-check -f "$manifest" "${rbnx_args[@]}"
