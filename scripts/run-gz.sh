#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARDUPILOT_GAZEBO="${ARDUPILOT_GAZEBO_PATH:-/opt/ardupilot_gazebo}"
WORLD_SDF="${ROOT}/sim/worlds/baylands.sdf"

export GZ_SIM_RESOURCE_PATH="${ROOT}/sim/models:${ARDUPILOT_GAZEBO}/models:${ARDUPILOT_GAZEBO}/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ARDUPILOT_GAZEBO}/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
unset GZ_SIM_SERVER_CONFIG_PATH
# ROS Jazzy vendors gz-tools without sim; keep system Harmonic configs first.
export GZ_CONFIG_PATH="/usr/share/gz${GZ_CONFIG_PATH:+:$GZ_CONFIG_PATH}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
if command -v nvidia-smi >/dev/null 2>&1; then
  export __GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME:-nvidia}"
  export __NV_PRIME_RENDER_OFFLOAD="${__NV_PRIME_RENDER_OFFLOAD:-1}"
fi

if pgrep -f '(^|/)gz sim( |$)' >/dev/null; then
  echo "Gazebo is already running. Stop the other sim.launch.py first." >&2
  echo "A second instance usually shows a white 3D view." >&2
  pgrep -a -f '(^|/)gz sim( |$)' >&2 || true
  exit 1
fi

GZ_GUI="${GZ_GUI:-true}"
if [[ "${GZ_GUI}" == "false" || "${GZ_GUI}" == "0" ]]; then
  exec gz sim -s -r "${WORLD_SDF}"
fi
exec gz sim -r "${WORLD_SDF}"
