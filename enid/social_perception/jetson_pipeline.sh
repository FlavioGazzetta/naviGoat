#!/usr/bin/env bash
# Runs the already-tested ROS 2 perception pipeline inside navigoat:people-dev.
# This script is executed INSIDE the Docker container.
set -Eeuo pipefail

APP_DIR=/workspace/enid/social_perception/femto_id_velocity_tracker
cd "$APP_DIR"

if [[ ! -f femto_track_people.py || ! -f social_tracking_dashboard.py ]]; then
  echo "Missing tracker/dashboard in ${APP_DIR}" >&2
  exit 1
fi

# Keep all processes' output attached to the container's stdout/stderr,
# so `sudo docker logs -f navigoat-social` shows it without more terminals.
pids=()
cleanup() {
  trap - EXIT INT TERM
  echo "[pipeline] Stopping ROS processes..."
  for pid in "${pids[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  for pid in "${pids[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT
trap 'exit 143' TERM
trap 'exit 130' INT

echo "[pipeline] Starting Orbbec Femto Bolt driver..."
ros2 launch orbbec_camera femto_bolt.launch.py depth_registration:=false &
pids+=("$!")

# ROS 2 discovers publishers/subscribers asynchronously: it is safe to start
# the registration node before the first camera frames arrive.
echo "[pipeline] Starting external depth registration..."
ros2 run depth_image_proc register_node --ros-args \
  -r depth/image_rect:=/camera/depth/image_raw \
  -r depth/camera_info:=/camera/depth/camera_info \
  -r rgb/camera_info:=/camera/color/camera_info \
  -r depth_registered/image_rect:=/camera/depth_registered/image_rect &
pids+=("$!")

echo "[pipeline] Starting headless GPU person tracker..."
# Run from the bind-mounted project directory, so the model checkpoint is
# cached there between container launches rather than lost on container removal.
python3 -u femto_track_people.py \
  --model "${YOLO_MODEL:-yolov8s-worldv2.pt}" \
  --device cuda:0 \
  --imgsz "${YOLO_IMGSZ:-320}" \
  --conf "${YOLO_CONF:-0.45}" \
  --max-age "${TRACK_MAX_AGE:-1.0}" \
  --no-plot &
pids+=("$!")

echo "[pipeline] Starting browser dashboard on ${DASHBOARD_HOST:-0.0.0.0}:${DASHBOARD_PORT:-8765}..."
python3 -u social_tracking_dashboard.py \
  --host "${DASHBOARD_HOST:-0.0.0.0}" \
  --port "${DASHBOARD_PORT:-8765}" &
pids+=("$!")

echo "[pipeline] Started all four components. Waiting; Docker handles lifecycle."
# If any component exits, terminate the whole stack to avoid leaving
# a partially broken perception pipeline running. Restart manually.
set +e
wait -n "${pids[@]}"
exit_code=$?
set -e
echo "[pipeline] A component exited (code $exit_code); stopping pipeline." >&2
exit 1
