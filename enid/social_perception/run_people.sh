#!/usr/bin/env bash
# Simple Jetson HOST launcher for the Femto Bolt + YOLO + tracker + dashboard.
set -Eeuo pipefail

NAME=navigoat-social
IMAGE=navigoat:people-dev
HOST_DIR="$HOME/naviGoat/enid"
PIPELINE="$HOST_DIR/social_perception/jetson_pipeline.sh"
APP_DIR="$HOST_DIR/social_perception/femto_id_velocity_tracker"

# Use sudo because the Jetson's user currently manages Docker with sudo.
docker_cmd() { sudo docker "$@"; }

exists() { docker_cmd container inspect "$NAME" >/dev/null 2>&1; }
running() { [[ "$(docker_cmd inspect --format '{{.State.Running}}' "$NAME" 2>/dev/null)" == true ]]; }

start() {
  if running; then
    echo "${NAME} is already running. Dashboard: http://192.168.1.170:8765"
    return
  fi
  if exists; then
    docker_cmd start "$NAME"
  else
    for path in "$PIPELINE" "$APP_DIR/femto_track_people.py" "$APP_DIR/social_tracking_dashboard.py"; do
      if [[ ! -f "$path" ]]; then
        echo "Missing required file: $path" >&2
        exit 1
      fi
    done
    # The GUI dashboard has no authentication; it is exposed to the local
    # network for convenience. Set DASHBOARD_HOST=127.0.0.1 for SSH-only access.
    docker_cmd run -d \
      --name "$NAME" \
      --restart no \
      --init \
      --runtime=nvidia \
      --gpus all \
      -e NVIDIA_VISIBLE_DEVICES=all \
      -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
      --network=host \
      --ipc=host \
      --device-cgroup-rule='c 189:* rwm' \
      -v /dev/bus/usb:/dev/bus/usb \
      -v /run/udev:/run/udev:ro \
      -v "$HOST_DIR:/workspace/enid" \
      -e PYTHONUNBUFFERED=1 \
      -e "DASHBOARD_HOST=${DASHBOARD_HOST:-0.0.0.0}" \
      -e "DASHBOARD_PORT=${DASHBOARD_PORT:-8765}" \
      -e "YOLO_MODEL=${YOLO_MODEL:-yolov8s-worldv2.pt}" \
      -e "YOLO_IMGSZ=${YOLO_IMGSZ:-320}" \
      -e "YOLO_CONF=${YOLO_CONF:-0.40}" \
      -e "TRACK_MAX_AGE=${TRACK_MAX_AGE:-1.0}" \
      "$IMAGE" \
      bash -c 'source /opt/ros/jazzy/setup.bash && exec bash /workspace/enid/social_perception/jetson_pipeline.sh'
  fi
  echo "Started ${NAME}. Dashboard (trusted LAN): http://192.168.1.170:8765"
  echo "Logs: $0 logs"
}

case "${1:-start}" in
  start) start ;;
  stop)
    if exists; then docker_cmd stop "$NAME"; else echo "Container not yet created."; fi
    ;;
  restart)
    if exists; then docker_cmd restart "$NAME"; else start; fi
    ;;
  recreate)
    # Recreate when changing Docker image or environment variables.
    if exists; then docker_cmd stop "$NAME" >/dev/null || true; docker_cmd rm "$NAME" >/dev/null; fi
    start
    ;;
  status)
    docker_cmd ps -a --filter "name=^/${NAME}$" --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'
    ;;
  logs)
    docker_cmd logs --tail 100 -f "$NAME"
    ;;
  *)
    echo "Usage: $0 [start|stop|restart|recreate|status|logs]" >&2
    exit 2
    ;;
esac
