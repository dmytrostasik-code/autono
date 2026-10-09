#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-/opt/ardupilot}"
SITL_DIR="${ROOT}/.sitl"

mkdir -p "${SITL_DIR}"
cd "${SITL_DIR}"

export PATH="${HOME}/.local/bin:${PATH}"

# Match husky spawn in sim/worlds/baylands.sdf. Heading 157 deg from the live pose.
exec "${ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py" \
  -v Rover \
  --no-rebuild \
  --model JSON \
  --custom-location 37.412064043623495,-121.998765563356230,37.850,157 \
  --out 127.0.0.1:14551 \
  --out 127.0.0.1:14550 \
  --add-param-file "${ARDUPILOT_DIR}/Tools/autotest/default_params/rover-skid.parm" \
  --add-param-file "${ROOT}/sim/params/ardurover.parm" \
  "$@"
