#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$DEPLOY_DIR/scripts/common.sh"

mode=""
while (($#)); do
  case "$1" in
    --mode)
      (($# >= 2)) || { echo "--mode requires a value" >&2; exit 2; }
      mode="$2"; shift 2 ;;
    --mode=*) mode="${1#*=}"; shift ;;
    -h|--help)
      echo "usage: $0 --mode {hardware|simulation}"
      exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done
require_mode "$mode"
load_deployment_env
mkdir -p "$RUN_DIR"

if [[ -f "$RUN_DIR/mode" ]]; then
  active="$(cat "$RUN_DIR/mode")"
  if [[ "$active" != "$mode" ]]; then
    echo "refusing to stop mode=$mode while mode=$active is active" >&2
    exit 1
  fi
fi

manifest="$(deployment_manifest "$mode")"
if [[ -f "$RUN_DIR/state.json" && -f "$manifest" ]]; then
  if command -v rbnx >/dev/null 2>&1; then
    rbnx shutdown -f "$manifest" || true
  else
    echo "[stop] warning: rbnx unavailable; continuing scoped cleanup" >&2
  fi
fi

if [[ "$mode" == "simulation" ]]; then
  "$DEPLOY_DIR/sim/manage.sh" down
else
  cache_root="$RUN_DIR/cache"
  mapfile -t pids < <(
    for pid in $(ps -u "$(id -u)" -o pid=); do
      cwd="$(readlink -f "/proc/$pid/cwd" 2>/dev/null || true)"
      [[ "$cwd" == "$cache_root" || "$cwd" == "$cache_root/"* ]] && echo "$pid"
    done
  )
  if ((${#pids[@]})); then
    kill -TERM "${pids[@]}" 2>/dev/null || true
  fi
  router_pid_file="$RUN_DIR/rmw_zenohd.hardware.pid"
  if [[ -f "$router_pid_file" ]]; then
    router_pid="$(cat "$router_pid_file")"
    if [[ "$router_pid" =~ ^[0-9]+$ ]]; then
      kill -TERM "$router_pid" 2>/dev/null || true
      wait "$router_pid" 2>/dev/null || true
    fi
    rm -f "$router_pid_file"
  fi
  if ss -H -lunp 2>/dev/null | awk '$5 ~ /:43897$/ { found=1 } END { exit !found }'; then
    echo "[stop] ERROR: UDP 43897 is still occupied" >&2
    exit 1
  fi
fi

archive_robonix_logs "$mode"
rm -f "$RUN_DIR/mode"
echo "[stop] mode=$mode stopped cleanly"
