#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_DIR="$(cd "$DEPLOY_DIR/.." && pwd)"
RUN_DIR="$DEPLOY_DIR/rbnx-boot"
export ROBONIX_SOURCE_DIR="${ROBONIX_SOURCE_DIR:-$WORKSPACE_DIR/robonix}"

ROBONIX_PYTHON_ENV="${ROBONIX_PYTHON_ENV:-$HOME/.local/share/robonix-python}"
export PATH="$ROBONIX_PYTHON_ENV/bin:$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
export ROBONIX_DEPLOY_DIR="$DEPLOY_DIR"
export UV_CACHE_DIR="${UV_CACHE_DIR:-$DEPLOY_DIR/.cache/uv}"
export UV_DEFAULT_INDEX="${UV_DEFAULT_INDEX:-https://mirrors.pku.edu.cn/pypi/web/simple}"
export UV_INDEX_URL="${UV_INDEX_URL:-$UV_DEFAULT_INDEX}"
export PIP_CACHE_DIR="${PIP_CACHE_DIR:-$DEPLOY_DIR/.cache/pip}"

load_deployment_env() {
  if [[ -f "$DEPLOY_DIR/.env" ]]; then
    set -a
    # shellcheck disable=SC1091
    source "$DEPLOY_DIR/.env"
    set +a
  fi
}

require_vlm_config() {
  local variable
  local -a missing=()
  for variable in VLM_BASE_URL VLM_API_KEY VLM_MODEL; do
    [[ -n "${!variable:-}" ]] || missing+=("$variable")
  done
  if ((${#missing[@]})); then
    printf 'missing VLM configuration in %s/.env: %s\n' \
      "$DEPLOY_DIR" "${missing[*]}" >&2
    return 1
  fi
}

require_locked_checkout() {
  local repo_dir="$1" lock_key="$2" label="$3"
  git -C "$repo_dir" rev-parse --git-dir >/dev/null 2>&1 || {
    echo "missing $label Git checkout: $repo_dir" >&2
    return 1
  }
  local expected actual
  expected="$(awk -F= -v key="$lock_key" '$1 == key {print $2}' "$DEPLOY_DIR/sim/vendor.lock")"
  actual="$(git -C "$repo_dir" rev-parse HEAD)"
  [[ -n "$expected" && "$actual" == "$expected" ]] || {
    echo "$label revision $actual does not match sim/vendor.lock ($expected)" >&2
    return 1
  }
}

require_mode() {
  local mode="${1:-}"
  if [[ "$mode" != "hardware" && "$mode" != "simulation" ]]; then
    echo "--mode must be hardware or simulation" >&2
    return 2
  fi
}

deployment_manifest() {
  case "$1" in
    hardware) printf '%s\n' "$DEPLOY_DIR/robonix_manifest.yaml" ;;
    simulation) printf '%s\n' "$DEPLOY_DIR/robonix_manifest.simulation.yaml" ;;
    *) require_mode "$1" ;;
  esac
}

validate_deployment_manifest() {
  local mode="$1" manifest="$2"
  python3 "$DEPLOY_DIR/scripts/validate_deployment_manifest.py" \
    --mode "$mode" "$manifest"
}

require_rbnx() {
  command -v rbnx >/dev/null 2>&1 || {
    echo "rbnx not found; install the pinned Robonix checkout first" >&2
    return 1
  }
  [[ -d "$ROBONIX_SOURCE_DIR/.git" ]] || {
    echo "missing Robonix source: $ROBONIX_SOURCE_DIR" >&2
    return 1
  }
  require_locked_checkout "$ROBONIX_SOURCE_DIR" robonix_commit Robonix
  rbnx setup "$ROBONIX_SOURCE_DIR" >/dev/null
}

source_ros_humble() {
  [[ -f /opt/ros/humble/setup.bash ]] || {
    echo "hardware mode requires /opt/ros/humble/setup.bash" >&2
    return 1
  }
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
}

archive_robonix_logs() {
  local mode="$1"
  local source="$RUN_DIR/logs"
  local target="$DEPLOY_DIR/logs/$mode"
  [[ -d "$source" ]] || return 0
  mkdir -p "$target"
  cp -a "$source/." "$target/" 2>/dev/null || true
}
