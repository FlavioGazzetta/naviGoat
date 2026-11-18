#!/usr/bin/env bash
# Subscriber + Nav2 goal preview. The team's shared perception/Nav2 stack must
# already be running. --execute-nav2 explicitly enables movement through Nav2.
set -Eeuo pipefail
APP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "$APP_DIR/ros_nav2_follow.py" ]]; then
  echo 'Expected ros_nav2_follow.py next to this launcher.' >&2
  exit 1
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
python3 -c 'import rclpy, tf2_ros, geometry_msgs.msg, std_msgs.msg, std_srvs.srv'
exec python3 -B -u "$APP_DIR/ros_nav2_follow.py" "$@"
