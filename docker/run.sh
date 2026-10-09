#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_NAME="ardurover-navigation"
CONTAINER_NAME="ardurover-navigation"
WORKDIR="/home/developer/ardurover_navigation"

xhost +local:docker >/dev/null 2>&1 || xhost +local: >/dev/null 2>&1 || true

RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
STABLE_AUTH="${RUNTIME_DIR}/.ardurover-nav.Xauthority"
HOST_DISPLAY="${DISPLAY:-:0}"

refresh_xauthority() {
  local src=""
  if [[ -n "${XAUTHORITY:-}" && -f "${XAUTHORITY}" ]]; then
    src="${XAUTHORITY}"
  elif [[ -f "${RUNTIME_DIR}/gdm/Xauthority" ]]; then
    src="${RUNTIME_DIR}/gdm/Xauthority"
  elif compgen -G "${RUNTIME_DIR}/.mutter-Xwaylandauth.*" >/dev/null; then
    src="$(ls -t "${RUNTIME_DIR}"/.mutter-Xwaylandauth.* | head -1)"
  elif [[ -f "${HOME}/.Xauthority" ]]; then
    src="${HOME}/.Xauthority"
  fi

  if [[ -d "${STABLE_AUTH}" ]]; then
    rmdir "${STABLE_AUTH}" 2>/dev/null || true
  fi
  if [[ ! -e "${STABLE_AUTH}" ]]; then
    : >"${STABLE_AUTH}"
  fi
  if [[ -n "${src}" ]]; then
    cat "${src}" >"${STABLE_AUTH}"
  fi
  chmod 600 "${STABLE_AUTH}" 2>/dev/null || true
}

refresh_xauthority

if docker container inspect "${CONTAINER_NAME}" >/dev/null 2>&1; then
  if [[ "$(docker container inspect -f '{{.State.Running}}' "${CONTAINER_NAME}")" != "true" ]]; then
    docker start "${CONTAINER_NAME}" >/dev/null
  fi
  exec bash "${ROOT}/docker/attach.sh"
fi

GPU_ARGS=()
if command -v nvidia-smi >/dev/null 2>&1 && docker info 2>/dev/null | grep -qi nvidia; then
  GPU_ARGS=(
    --gpus=all
    --env=NVIDIA_VISIBLE_DEVICES=all
    --env=NVIDIA_DRIVER_CAPABILITIES=all
  )
fi

docker run -it \
  --name="${CONTAINER_NAME}" \
  --init \
  --network=host \
  "${GPU_ARGS[@]}" \
  --env=DISPLAY="${HOST_DISPLAY}" \
  --env=XAUTHORITY=/home/developer/.Xauthority \
  --env=QT_QPA_PLATFORM=xcb \
  --env=QT_X11_NO_MITSHM=1 \
  --env=ARDUROVER_NAV_ROOT="${WORKDIR}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "${STABLE_AUTH}:/home/developer/.Xauthority:ro" \
  -v "${ROOT}:${WORKDIR}" \
  --workdir="${WORKDIR}" \
  --user=developer \
  "${IMAGE_NAME}" \
  bash
