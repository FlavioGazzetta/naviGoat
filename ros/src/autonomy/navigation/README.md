# Navigation: how to run it

Copy-paste instructions for the real robot. What the topics are and how it works: [TOPICS.md](TOPICS.md). The old, long
README (Gazebo tests and details): [README_archive.md](README_archive.md). Gazebo test scenarios (no one, people
standing, people walking): [navigation_scenarios.md](navigation_scenarios.md).

## 1. Connect to the Jetson

### USB cable (most reliable)

Cable from the laptop to the Jetson's **USB-C** port. The Jetson is then always `192.168.55.1`.

```bash
ssh navigoat@192.168.55.1
```

### Wi-Fi / phone hotspot

Laptop and Jetson on the same network. Find the Jetson's address on the Jetson (🤖 `hostname -I`), e.g. `172.20.10.2`.

```bash
ssh -o ServerAliveInterval=5 -o ServerAliveCountMax=3 navigoat@172.20.10.2
```

On a phone hotspot: 5 GHz ("Maximise Compatibility" off), phone plugged in. Campus networks (Imperial-WPA, eduroam)
usually block devices from reaching each other.

## 2. Start the container

From the repository folder (it gets mounted as `/workspace`, so it **must** be run from `~/naviGoat`):
```bash
cd ~/naviGoat
docker run --rm -it --name navigoat-dev --network=host --runtime nvidia --gpus all --privileged -v /dev:/dev \
  -e DISPLAY -v "$PWD:/workspace" -v /tmp/.X11-unix:/tmp/.X11-unix:ro navigoat:dev bash
```
- More terminals in the same container: `docker exec -it navigoat-dev bash`
- The base and lidar ports are found automatically. For social nav the image needs Pete's extras:
`docker build -f docker/Dockerfile.dev --build-arg PETE=true -t navigoat:dev .`
- First time only, build: `cd /workspace && colcon build --packages-ignore unitree_lidar_ros unitree_lidar_sdk --symlink-install`

## 3. Launch

In every container terminal first:
```bash
source /workspace/install/setup.bash
NAV=/workspace/ros/src/autonomy/navigation/navigation_nodes/launch/navigation_robot.launch.py
MAP=/workspace/ros/src/bringup/maps/lab.db      # the map file: a new name = a new map
```

if using only the camera we add the flags:

```bash
use_depth_camera:=true use_lidar:=false
```

use_depth_camera:=true use_lidar:=false

Then **one** of these (only one launch at a time):

| What | Command |
| --- | --- |
| **Map + drive with Nav2 goals** | `ros2 launch $NAV database_path:=$MAP` |
| **Map + drive with the keyboard** (no Nav2) | `ros2 launch $NAV database_path:=$MAP teleop:=true` |
| **Navigate on a finished map** | `ros2 launch $NAV database_path:=$MAP mode:=localisation` |
| **Use lidar and the Femto together** | add `use_depth_camera:=true` |
| **Use the Femto with lidar off** | add `use_depth_camera:=true use_lidar:=false` |
| **Any of the above with social nav** (camera + people) | add `social:=true` |
| **Local costmap with people only** (no lidar/camera obstacles, social nav still works) | add `local_obstacles:=false` |
| **A clean map** (moving people left out of it) | add `filter:=object` |

- **A different map**: change `MAP`, e.g. `MAP=/workspace/ros/src/bringup/maps/corridor.db`. A new name starts a new map;
  an existing one is continued (mapping) or used (localisation). Each map keeps its own places (`corridor_places.yaml`).
- **The motors turn on by themselves** once the base connects (`Connected to Imperial_3097` in the output).
- **Keyboard** (with `teleop:=true`), second terminal: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
- **Map with the keyboard, then navigate**: run with `teleop:=true`, drive around, Ctrl+C (saves the map), then launch
  again with `mode:=localisation`.
- **In localisation**, the robot must start where the map was started, or get its pose from RViz's **2D Pose Estimate**.
- **Sensor defaults** are `use_lidar:=true use_depth_camera:=false`. At least one navigation sensor must be enabled.
- **`local_obstacles:=false`** keeps the lidar and camera out of the local costmap: it only has the people from the
  social layer (with `social:=true`) and their inflation. SLAM still uses the sensors, and the global costmap is
  unchanged (map + people). The robot then won't stop for anything that isn't on the map, so use it for tests only.
- **Femto navigation** adds RGB-D input to SLAM and uses the front-facing `/camera/depth/points` cloud for local obstacles. Start with a fresh database for RGB-D mapping and use that database in localisation. If the camera already runs with `enable_point_cloud:=true` and publishes registered depth and IMU, add `start_femto:=false`.
- **Everything incl. handle and wristband**: `ros2 launch bringup launch.py database_path:=$MAP` (same flags).

Other flags: `filter:=deskew|object|none` (default `deskew`), `enable_motors:=false`, `rviz:=true` (RViz on the Jetson,
heavy), `places_file:=...`, `pioneer_port:=...` / `lidar_port:=...` (only if the base or lidar don't connect).

### Just drive with the keyboard (no lidar, no map, no Nav2)

Only the base and its motors. Not together with a navigation launch (two base drivers on one port both fail):
```bash
source /workspace/install/setup.bash
ros2 launch bringup base.launch.py
```
Wait for `Connected to Imperial_3097`, then in a second terminal:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
The base's port is found automatically (add `port:=...` only if it doesn't connect).

## 4. RViz on the laptop

From the repository root on the laptop (built once with `colcon build`, for the robot's 3D model):
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```
then, **over the USB cable**:
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/ros/src/autonomy/navigation/config/fastdds_usb.xml   # ROS over the cable only, Wi-Fi stays for internet
unset ROS_STATIC_PEERS
rviz2 -d ros/src/autonomy/navigation/config/robot.rviz
```
or **over Wi-Fi** (replace with the Jetson's address):
```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_STATIC_PEERS=172.20.10.2
rviz2 -d ros/src/autonomy/navigation/config/robot.rviz
```
- First time on each network, let the Jetson through the laptop's firewall: `sudo ufw allow from 192.168.55.0/24`
  (cable) or `sudo ufw allow from <jetson address>` (Wi-Fi).
- On Wi-Fi, untick the lidar point cloud and `/cloud_map`: they're too heavy for a hotspot.
- RViz doesn't switch between cable and Wi-Fi by itself: restart it with the other settings.
- Toolbar: **2D Pose Estimate** (tell it where the robot is), **2D Goal Pose** (send a goal), **Publish Point** (add or
  remove a test person). The local costmap display is off by default; tick it to see obstacles around the robot.

## 5. Places

Drive the robot to the spot first: a place is saved at the robot's current position. Best in `localisation` mode
(in mapping mode the map can still shift).

```bash
ros2 topic pub --once /navigation/save_place   std_msgs/msg/String "{data: kitchen}"   # save here
ros2 topic pub --once /navigation/goto         std_msgs/msg/String "{data: kitchen}"   # drive there
ros2 topic pub --once /navigation/forget_place std_msgs/msg/String "{data: kitchen}"   # delete it
ros2 topic pub --once /navigation/cancel       std_msgs/msg/String "{data: ''}"        # stop driving
ros2 topic pub --once /navigation/goto_point geometry_msgs/msg/PointStamped "{header: {frame_id: map}, point: {x: 1.0, y: 0.0}}"
ros2 topic echo /navigation/status std_msgs/msg/String --field data   # saved:kitchen, going:kitchen, arrived:kitchen, failed:...
ros2 topic echo /navigation/places std_msgs/msg/String --once --qos-durability transient_local --qos-reliability reliable   # the saved places
```

By voice: "Pete, remember this place as the kitchen", "Pete, take me to the kitchen".

### Test going to an object (fake middleman + perception)

🐳 First switch to object mode (otherwise the object position is ignored), then send where the object is. The robot
stops 0.8 m before it. Watch `ros2 topic echo /navigation/status std_msgs/msg/String --field data`: `waiting:chair` → `going:chair` →
`arrived:chair`.

```bash
ros2 topic pub --once /middleman/mode std_msgs/msg/String "{data: '{\"mode\": \"object\", \"target\": \"chair\", \"stop\": false}'}"
```
then **one** of:
```bash
# relative to the camera: x = metres in front, y = metres to the left (negative = right). Just measure it
ros2 topic pub --once /object_mapping geometry_msgs/msg/PointStamped "{header: {frame_id: camera_link}, point: {x: 2.0, y: 0.0}}"
# a spot on the map: in RViz pick the Publish Point tool and only HOVER over the spot, the map x/y show in the status bar
ros2 topic pub --once /object_mapping geometry_msgs/msg/PointStamped "{header: {frame_id: map}, point: {x: -1.8, y: 0.3}}"
```
Don't click with Publish Point: a click adds a test person there (click again to remove it).

## 6. Check that things work


```bash
ros2 topic hz /odom                                     # base: ~10 Hz
ros2 topic hz /unilidar/cloud                           # lidar: ~10 Hz
ros2 topic echo /people navigoat_msgs/msg/People --no-daemon                     # people on the map (social:=true)
ros2 topic echo /social_perception/tracks std_msgs/msg/String --no-daemon --field data   # what the camera tracker sees
```
Always give the message type after the topic name, as above. Without it, `ros2 topic echo` first has to look the type
up, and when the Jetson is busy that can take too long: it then fails with `Could not determine the type for the
passed topic`, even though the topic is published. `--no-daemon` avoids it wrongly saying "not published yet" after
restarts. If `hz` shows nothing right after a restart, run `ros2 daemon stop` and try again.

**Vibrations** (what navigation sends to the devices), 🐳:
```bash
ros2 topic echo /handle/vibration std_msgs/msg/UInt8MultiArray --no-daemon --field data      # handle: the next turn on the path
ros2 topic echo /wristband/vibration std_msgs/msg/UInt8MultiArray --no-daemon --field data   # wristband: the closest person
```
Each message is `[side, strength]`: side `0` = left, `1` = right, strength `0`–`255` (`0` = off). It prints as
`array('B', [1, 140])`; `array('B', [0, 0])` means no vibration.
- **Handle**: only while driving a goal, when a turn on the path comes within 2 m. Sharper turn = stronger.
- **Wristband**: only while the robot moves, when a person is within 1.2 m (`social:=true`, or a test person clicked
  with Publish Point). Closer = stronger, 255 at 0.5 m.
- Both are sent 10 times a second; a `0` to the old side comes first when the side changes.
- To make the devices actually vibrate, their nodes have to run too (`ros2 launch bringup launch.py ...`, or see the
  [interaction README](../../interaction/README.md)).

## 7. Stop

- Normally: **one** Ctrl+C in the launch terminal, then wait 10–15 s (RTAB-Map saves the map). Not Ctrl+Z (that only pauses).
- If a terminal is lost, :
  ```bash
  docker exec navigoat-dev pkill -INT -f "ros2 launch"     # stop cleanly, map gets saved
  docker exec navigoat-dev pgrep -af "p2os_driver|rtabmap|controller_server"   # should print nothing
  ```
- Everything at once: `docker stop navigoat-dev` (then start the container again, step 2).

## When something's wrong

| Symptom | Likely cause |
| --- | --- |
| `SYNC failed` / `Setup() failed`, base never connects | base off, battery low (battery light red instead of green), or stuck: switch the base off and on |
| RViz empty | the base hasn't connected yet, the laptop's workspace isn't sourced, or wrong network settings (restart RViz) |
| Goals rejected, "Action server is inactive" | Nav2 switched itself off: restart the launch |
| No path drawn, robot spins on the spot | the planner found no path: wrong pose (2D Pose Estimate) or a noisy map (re-map with `filter:=object`) |
| `ros2 launch` says "not a valid package name" | the container was started from the wrong folder: restart it from `~/naviGoat` |
| A second launch is running | `pgrep -af "ros2 launch"`; stop the extra one (two base drivers on one port both fail) |
