#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="ardurover-navigation"

exec docker exec -it \
  --user=developer \
  --workdir=/home/developer/ardurover_navigation \
  -e DISPLAY="${DISPLAY:-:0}" \
  "${CONTAINER_NAME}" \
  bash
