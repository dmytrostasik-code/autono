#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_NAME="ardurover-navigation"

BUILD_ARGS=()
if [[ "${1:-}" == "--no-cache" ]]; then
  BUILD_ARGS+=(--no-cache)
fi

docker build "${BUILD_ARGS[@]}" -t "${IMAGE_NAME}" -f "${ROOT}/Dockerfile" "${ROOT}"
