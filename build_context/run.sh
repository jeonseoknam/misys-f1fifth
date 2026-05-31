#!/usr/bin/env bash
# Launch the rebuilt container with the SAME runtime options as the original
# container b520034a42ad (GPU + GUI/X11 + USB + serial + audio).
# Usage: ./run.sh [image-tag] [container-name]
set -uo pipefail
IMAGE="${1:-misys:3d_xvn_rebuilt}"
NAME="${2:-eyevy_3d}"

xhost +local:root >/dev/null 2>&1 || true   # let rviz/gazebo reach the host X server

EXTRA=()
[ -e /dev/ttyACM0 ] && EXTRA+=(--device /dev/ttyACM0)
[ -e /dev/ttyUSB0 ] && EXTRA+=(--device /dev/ttyUSB0)
[ -d "$HOME/shared_dir" ] || mkdir -p "$HOME/shared_dir"

docker run -it --rm \
  --name "$NAME" \
  --runtime nvidia \
  --privileged \
  --network host \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /dev/input:/dev/input \
  -v /dev/shm:/dev/shm \
  -v /media:/media \
  -v /usr/share/vulkan:/usr/share/vulkan \
  -v /run/udev:/run/udev \
  -v "$HOME/shared_dir:/home/misys/shared_dir" \
  --device /dev/snd \
  --device /dev/dri \
  "${EXTRA[@]}" \
  "$IMAGE"
