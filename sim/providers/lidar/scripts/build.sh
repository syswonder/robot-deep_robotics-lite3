#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
FLAGS=(--mcp)
[[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)
rbnx codegen -p "$PKG" "${FLAGS[@]}"
touch "$PKG/rbnx-build/.rbnx-built"
