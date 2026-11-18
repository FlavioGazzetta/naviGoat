# Navigation

This is the navigation part of Pete. It drives the robot with Nav2 to places we saved on the map, to objects that perception finds, or just to an x, y point. It also keeps the robot away from people, following a few social rules (like not walking between two people who are talking). And it tells the person being guided, through the handle or the wristband, how close people are and when a turn is coming.

## Topics

These are all the topics our nodes listen to and send. If you're connecting something to navigation, this is the part you need.

### What we subscribe to

| Topic | Type | Who sends it | What we do with it | Node |
| --- | --- | --- | --- | --- |
| `/middleman/mode` | `std_msgs/String` (JSON) | middleman | `location` → drive to that saved place. `object` → stop and wait for `/object_mapping`. A stop or any other mode (follow, manual, help...) → stop driving | places_node |
| `/object_mapping` | `geometry_msgs/PointStamped` | perception | where the object is. Only used in object mode: we drive to 0.8 m before it, facing it | places_node |
| `/navigation/save_place` | `std_msgs/String` | anyone | save where the robot is now under that name | places_node |
| `/navigation/forget_place` | `std_msgs/String` | anyone | delete that saved place | places_node |
| `/navigation/goto` | `std_msgs/String` | anyone | drive to that saved place (useful for testing without the middleman) | places_node |
| `/navigation/goto_point` | `geometry_msgs/PointStamped` | anyone | drive to that point (any frame) | places_node |
| `/navigation/cancel` | `std_msgs/String` | anyone | stop driving (the content doesn't matter) | places_node |
| `/social_perception/tracks` | `std_msgs/String` (JSON) | Enid's people tracker | the people the camera sees: `{"stamp_sec", "tracks": [{"id", "x", "y", "confirmed"}]}`, x forward and y left of the camera | people_node |
| `/clicked_point` | `geometry_msgs/PointStamped` | RViz "Publish Point" tool | click to add a fake standing person, click on one to remove them | people_node |
| `/people` | `navigoat_msgs/People` | people_node | everyone on the map (this one is also used inside our own stuff, see below) | social_layer, feedback_node |
| `/plan` | `nav_msgs/Path` | Nav2 | the path Nav2 is following, to know which turns are coming | feedback_node |

### What we publish

| Topic | Type | Who it's for | What's in it | Node |
| --- | --- | --- | --- | --- |
| `/middleman/done` | `std_msgs/String` | middleman | the name of what we drove to, once we got there (the middleman then says "Arrived.") | places_node |
| `/pete/response` | `std_msgs/String` | voice | things Pete says out loud, like "I don't know where the kitchen is yet." or "I couldn't get to the kitchen." | places_node |
| `/navigation/status` | `std_msgs/String` | us, for debugging | what's happening: `saved:kitchen`, `going:kitchen`, `arrived:kitchen`, `failed:...`, `cancelled:...`, `unknown:...`, `waiting:chair`, `error:...` | places_node |
| `/people` | `navigoat_msgs/People` | social_layer, feedback_node | every person on the map with their velocity, 10 times a second | people_node |
| `/navigation/proximity_feedback` | `std_msgs/UInt8MultiArray` | handle / wristband | `[side, magnitude]` for the closest person: side 0 = left, 1 = right, magnitude 0 to 255 (stronger = closer) | feedback_node |
| `/navigation/turn_feedback` | `std_msgs/UInt8MultiArray` | handle / wristband | `[side, magnitude]` for the next turn: stronger = sharper turn | feedback_node |
| `/navigation/places` | `std_msgs/String` | middleman | JSON list of the saved place names, e.g. `["balcony", "lab"]`, sent at start and after every save or forget (latched). The middleman understands them like its own places | places_node |
| `/navigation/places_markers` | `visualization_msgs/MarkerArray` | RViz | the saved places as orange arrows with their names | places_node |
| `/navigation/people_markers` | `visualization_msgs/MarkerArray` | RViz | the people as cylinders (blue = real, purple = fake), with an arrow in front of walkers | people_node |

Also, places_node doesn't send goals to Nav2 on a topic: it uses Nav2's `navigate_to_pose` **action** (you send a goal, Nav2 drives, and it tells you at the end if it worked).

The two feedback topics are in exactly the same `[side, magnitude]` format as `/handle/vibration` and `/wristband/vibration`, so they can be plugged straight into either device (with a remap). Which signal goes to which device is up to the interaction side.

## How it all fits together

There are three Python nodes, one C++ plugin that runs inside Nav2, and a message package:

1. **people_node** collects everyone we know about: the people Enid's tracker sees (it gives them relative to the camera, so we move them onto the map using TF), people you click in RViz, and fake people from a yaml file for testing. It works out how fast each person walks, and sends everyone on `/people` 10 times a second.
2. **social_layer** is a plugin that lives inside Nav2's costmaps (the grids Nav2 uses to plan, where each cell has a cost). It reads `/people` and draws costs around them: each person is an obstacle, they have a personal space, there's an expensive strip between two people who are together, and an expensive strip where walkers are about to go. So when Nav2 plans a path, it automatically goes around people.
3. **places_node** decides *where* to go. The middleman tells it the mode (go to a place, go to an object, stop...), it looks up the place in its saved places (or waits for perception to find the object), and sends the goal to Nav2. When Nav2 finishes, it tells the middleman we arrived, or Pete says it failed.
4. **Nav2** does the actual driving: it plans a path through the costmap (with our people in it) and the controller follows it.
5. **feedback_node** watches the path Nav2 is following and the people around, and turns that into vibrations for the person holding the handle: which side someone is close on, and which way the next turn goes.

### The files

| File | What it does |
| --- | --- |
| [navigation_nodes/places_node.py](navigation_nodes/places_node.py) | saves places, and sends Nav2 to places, objects and points |
| [navigation_nodes/people_node.py](navigation_nodes/people_node.py) | puts all the people on the map and publishes `/people` |
| [navigation_nodes/feedback_node.py](navigation_nodes/feedback_node.py) | the two vibration signals (people close by, turns coming) |
| [navigation_nodes/launch/navigation.launch.py](navigation_nodes/launch/navigation.launch.py) | starts the three nodes (Nav2 and localisation must already be running) |
| [navigation_nodes/launch/navigation_sim.launch.py](navigation_nodes/launch/navigation_sim.launch.py) | starts everything in Gazebo: the simulation (Gazebo, RTAB-Map, Nav2, RViz) plus our three nodes |
| [social_layer/src/social_layer.cpp](social_layer/src/social_layer.cpp) | the costmap plugin with the social rules |
| [navigoat_msgs/msg/](navigoat_msgs/msg) | our `People` and `Person` messages (a person = id, position, velocity) |
| [config/nav2_sim_params.yaml](config/nav2_sim_params.yaml) | Nav2 settings for Gazebo, including where the social layer is plugged in |
| [config/nav2_params.yaml](config/nav2_params.yaml) | the same for the real robot |
| [config/fake_people.yaml](config/fake_people.yaml) | test people: two talking 1.2 m apart and one walking up and down |
| [config/fake_people_gap.yaml](config/fake_people_gap.yaml) | test for the group rule: two people 2.0 m apart, with a gap the robot fits through |

Every Python file and the C++ file has comments explaining each part, so read those if you want the details.

The saved places go in a yaml file like this (in Gazebo it's `ros/src/simulation/maps/<world>_places.yaml`, next to the map):

```yaml
places:
  kitchen: {x: 1.25, y: -3.4, yaw: 1.571}
```

They only make sense on the map they were saved on, so if you rebuild the map, delete the places file too.

## Build

From the repository root (`/workspace` in the container):

```bash
colcon build --symlink-install --packages-up-to p2os_urdf navigation_nodes social_layer middleman
source install/setup.bash
```

If you change `social_layer.cpp` you need to build again and restart Nav2. The Python files don't need a rebuild, just restart the node.

## On the real robot

### 1. Start the container (on the Jetson)

From the repository root on the Jetson. The base and the lidar are given by their `/dev/serial/by-id/` names, because the
`/dev/ttyUSB0` / `/dev/ttyUSB1` numbers swap between boots (see the [SLAM README](../slam/README.md#setup-details)).

**Without social nav** (no camera needed):

```bash
xhost +si:localuser:root
docker run --rm -it --name navigoat-dev --network=host \
  --device=/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_02AF3CBD-if00-port0:/dev/unilidar \
  --device=/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_CWDZb2A4517-if00-port0:/dev/pioneer \
  -e DISPLAY -v "$PWD:/workspace" -v /tmp/.X11-unix:/tmp/.X11-unix:ro navigoat:dev bash
```

**With social nav**: the people tracker needs the camera, the GPU and Pete's extras, so build the image with
`--build-arg PETE=true` first (see the [perception README](../perception/README.md#setup)). The camera needs `--privileged
-v /dev:/dev`, which also shows the Jetson's own `/dev`, so `/dev/pioneer` and `/dev/unilidar` don't exist in this
container: the ports are passed to the launch instead (step 3).

```bash
xhost +si:localuser:root
docker run --rm -it --name navigoat-dev --network=host --runtime nvidia --gpus all --privileged -v /dev:/dev \
  -e DISPLAY -v "$PWD:/workspace" -v /tmp/.X11-unix:/tmp/.X11-unix:ro navigoat:dev bash
```

Open more terminals in the same container with `docker exec -it navigoat-dev bash`.

### 2. Build (once, inside the container)

```bash
cd /workspace
colcon build --packages-ignore unitree_lidar_ros unitree_lidar_sdk --symlink-install
source install/setup.bash
```

### 3. Launch

In every terminal:

```bash
source /workspace/install/setup.bash
NAV=/workspace/ros/src/autonomy/navigation/navigation_nodes/launch/navigation_robot.launch.py
MAP=/workspace/ros/src/bringup/maps/lab.db   # a new name makes a new map, an existing one is continued / reused
```

In the social nav container (`-v /dev:/dev`), also set the ports and add `$PORTS` to every `ros2 launch $NAV` below:

```bash
PORTS="pioneer_port:=/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_CWDZb2A4517-if00-port0 lidar_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_02AF3CBD-if00-port0"
```

[navigation_robot.launch.py](navigation_nodes/launch/navigation_robot.launch.py) starts the base (and turns the motors on
once it's connected), the lidar, RTAB-Map, Nav2 and our three nodes. Run only one at a time: two launches start two base
drivers on the same serial port, and neither works.

| | Without social nav | With social nav |
| --- | --- | --- |
| **Make a map** | `ros2 launch $NAV database_path:=$MAP` | `ros2 launch $NAV database_path:=$MAP social:=true` |
| **Navigate on a finished map** | `ros2 launch $NAV database_path:=$MAP mode:=localisation` | `ros2 launch $NAV database_path:=$MAP mode:=localisation social:=true` |

- **Making a map**: drive around with `ros2 run teleop_twist_keyboard teleop_twist_keyboard` (close it before Nav2
  drives), then Ctrl+C the launch to save the map. Don't save places yet, the map can still shift.
- **Map with teleop, then navigate**: `teleop:=true` leaves Nav2 out, so it can't fight the keyboard over `/cmd_vel`.
  Map the room with the keyboard, Ctrl+C (that saves the map), then launch again on the same map without `teleop` to
  navigate:

  ```bash
  ros2 launch $NAV database_path:=$MAP teleop:=true          # 1. map, no Nav2
  ros2 run teleop_twist_keyboard teleop_twist_keyboard        #    second terminal: drive around, then Ctrl+C both
  ros2 launch $NAV database_path:=$MAP mode:=localisation     # 2. navigate on that map, Nav2 on
  ```
- **Navigating**: save places (`/navigation/save_place`) and send goals as in [the Gazebo tests](#testing-in-gazebo). The
  places are kept next to the map: `lab.db` → `lab_places.yaml`.
- **Social nav**: `social:=true` also starts the camera, the YOLO server and Enid's people tracker (only those: no Qwen,
  scene description or middleman), and people_node puts the people on `/people`. It needs the social nav container (step
  1). Without it, the social layer stays in the costmaps but has no people to draw. Don't run it together with
  `pete.launch.py people:=true` (or the mapping launch's `start_camera:=true`): they start the same camera and YOLO
  server, and only one program can open the camera.
- **Everything, handle and wristband included**: `ros2 launch bringup launch.py database_path:=$MAP` (add
  `mode:=localisation` to navigate). The handle and wristband need their own devices in the container, see the
  [interaction README](../../interaction/README.md).

#### All the flags

Set a flag with `name:=value` after the launch file. Leave it out to get its default; to switch off a flag that is on by
default, set it to `false` (e.g. `enable_motors:=false`). They can be combined in any order, e.g.
`ros2 launch $NAV database_path:=$MAP mode:=localisation social:=true`. The same flags work on
`ros2 launch bringup launch.py`.

| Flag | Values | Default | What it does |
| --- | --- | --- | --- |
| `database_path` | a path to a `.db` file | **required**, no default | the map. A new file makes a new map; an existing one is continued (`mapping`) or navigated on (`localisation`) |
| `mode` | `mapping`, `localisation` | `mapping` | `mapping` builds the map while you drive. `localisation` navigates on a finished map without changing it |
| `teleop` | `true`, `false` | `false` | `true` leaves Nav2 out, so the keyboard has `/cmd_vel` to itself. Use it with `mapping` |
| `social` | `true`, `false` | `false` | `true` starts the camera, YOLO server and people tracker for social nav. Needs the social nav container |
| `enable_motors` | `true`, `false` | `true` | `false` leaves the motors off (the robot can't move until you send `/cmd_motor_state` `{state: 1}`) |
| `filter` | `object`, `deskew`, `none` | `object` | what RTAB-Map gets from the lidar. `object`: moving people left out of the map. `deskew`: scans corrected for the robot's motion, people included. `none`: raw scans. The costmaps always see everything |
| `rviz` | `true`, `false` | `false` | `true` opens RTAB-Map's RViz on the Jetson. Heavy: better run RViz on the laptop (step 4) |
| `pioneer_port` | a serial port | `/dev/pioneer` | the base. In the social nav container use `$PORTS`. If the base doesn't connect (`SYNC failed`, `Setup() failed`), the ports may be swapped |
| `lidar_port` | a serial port | `/dev/unilidar` | the lidar. Same as above |
| `places_file` | a path to a `.yaml` file | next to the map: `lab.db` → `lab_places.yaml` | where saved places are kept |

### 4. RViz on the laptop

The laptop and the Jetson need to be on the same network (the same Wi-Fi or hotspot, or the USB cable: the Jetson is then
`192.168.55.1`). Let the Jetson through the laptop's firewall once, then point ROS at the Jetson's address
(`hostname -I` on the Jetson):

```bash
sudo ufw allow from <jetson address>
export ROS_STATIC_PEERS=<jetson address>
rviz2 -d <rviz config>
```

## Testing in Gazebo

### 1. Make a map

```bash
ros2 launch /workspace/ros/src/autonomy/navigation/navigation_nodes/launch/navigation_sim.launch.py mode:=mapping
```

In another terminal, drive around the whole room with the keyboard, come back near where you started, then stop the launch with Ctrl+C. The map is saved to `ros/src/simulation/maps/indoor.db`.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/model/pioneer3at/cmd_vel -p speed:=0.3 -p turn:=0.5
```

Close the keyboard terminal before letting Nav2 drive, otherwise any key you press fights with Nav2.

### 2. Navigate, with people

```bash
ros2 launch /workspace/ros/src/autonomy/navigation/navigation_nodes/launch/navigation_sim.launch.py \
  fake_people:=/workspace/ros/src/autonomy/navigation/config/fake_people.yaml
```

Leave out `fake_people:=...` for an empty room. Wait until RViz shows the map, then watch what's going on in another terminal:

```bash
ros2 topic echo /navigation/status
```

Things to try:

- **Drive to a point.** The fake people show up in RViz as purple cylinders. The robot keeps its distance from them, and leaves more space in front of the walker than behind.
  ```bash
  ros2 topic pub --once /navigation/goto_point geometry_msgs/msg/PointStamped "{header: {frame_id: map}, point: {x: 4.0, y: -2.0}}"
  ```
- **Add people by clicking.** Pick the **Publish Point** tool in RViz and click on the map to add a standing person. Click on them again to remove them.
- **The group rule.** Launch with `fake_people:=.../config/fake_people_gap.yaml` instead. Send the robot to (-3, -2.5), then to (4, -2.5): it goes around the two people. Turn the rule off (the commands are at the top of that file) and do it again: now it goes through the gap.
- **Places.** Drive somewhere, save it, drive away and go back:
  ```bash
  ros2 topic pub --once /navigation/save_place std_msgs/msg/String "{data: kitchen}"
  ros2 topic pub --once /navigation/goto std_msgs/msg/String "{data: kitchen}"
  ros2 topic pub --once /navigation/cancel std_msgs/msg/String "{data: ''}"   # stop
  ```
- **Objects.** Pretend to be the middleman and perception: switch to object mode, then say "the chair is 2 m in front of the camera". The robot stops 0.8 m before it, facing it.
  ```bash
  ros2 topic pub --once /middleman/mode std_msgs/msg/String "{data: '{\"mode\": \"object\", \"target\": \"chair\", \"stop\": false}'}"
  ros2 topic pub --once /object_mapping geometry_msgs/msg/PointStamped "{header: {frame_id: camera_link}, point: {x: 2.0, y: 0.0}}"
  ```
- **Feedback.** While it drives, see what the handle or wristband would get:
  ```bash
  ros2 topic echo /navigation/turn_feedback --field data
  ros2 topic echo /navigation/proximity_feedback --field data
  ```
- **With the middleman.** Start it (`ros2 launch /workspace/ros/src/core/launch/middleman.launch.py`) and "speak" through the topic. The middleman only understands the names in `LOCATIONS` in [core/catalog.py](../../core/catalog.py), so save places under those names.
  ```bash
  ros2 topic pub --once /speech/text std_msgs/msg/String "{data: 'pete take me to the kitchen'}"
  ```

- **Real people on the simulated map.** Start Enid's tracker on the Jetson as usual (it runs the real camera too), then the simulation with `real_people:=true`. Whoever stands in front of the real camera shows up in front of the simulated robot, and the robot avoids them with the social layer only (the simulated lidar can't see them).
  ```bash
  ros2 launch .../navigation_sim.launch.py real_people:=true
  export ROS_DOMAIN_ID=42   # in every other terminal you use with this simulation
  ```
  Both cameras publish `/camera/color/image_raw` and friends, so the simulation goes on its own ROS domain (`sim_domain`, 42) to keep them apart, and people_node listens to the tracker on its domain (`tracker_domain`, 0) and passes `/social_perception/tracks` on. Point the real camera straight ahead, like the simulated one: the people are placed as if it were mounted on the robot.

If the robot draws a path but doesn't move, check `ros2 topic echo /model/pioneer3at/cmd_vel`. If it only shows zeros, the collision monitor is stopping it (it thinks something is inside the robot).

## The social rules

The social layer is in both of Nav2's costmaps, just before the inflation layer (so the robot's size is added around each person). Every time the costmap updates, it draws these:

| Rule | What it does | Parameters (defaults) |
| --- | --- | --- |
| The person | the person is an obstacle | `person_radius` 0.3 m |
| Personal space | a cost around them that fades to 0, and reaches further in front of someone walking | `personal_space` 1.0 m, `personal_space_cost` 150, `motion_stretch` 1.0 m per m/s |
| Groups | two people closer than `group_distance` count as together, and the strip between them is expensive, so the robot goes around them if it can | `group_distance` 2.2 m, `group_width` 0.6 m, `group_cost` 220 |
| Where walkers are going | an expensive strip along where each walker will be in the next `prediction_time` seconds, strongest right in front of them | `prediction_time` 2.0 s (0 = off), `prediction_width` 1.0 m, `prediction_cost` 200 |

Only the people themselves are lethal. All the other costs are below 253, which means they make a path more expensive but never block it. That's how "avoid it if possible" works in a costmap: if there's no other way (say a narrow corridor), the robot still goes through.

Two more details:

- People within 0.6 m of the robot (`ignore_radius`) are ignored. That's the person holding the handle, and a lethal cell on top of the robot would stop Nav2 from planning at all.
- If `/people` stops coming for 1 s (`people_timeout`), everyone is cleared, so a crashed people_node can't leave ghosts behind.

**Reading the costmap in RViz:** magenta is a person (lethal), the cyan ring around them is where the robot's centre can't go without touching them, and purple to blue is the cost fading out. If two people's cyan rings touch (like the 1.2 m pair in `fake_people.yaml`), there's physically no way between them, whatever the group rule says.

### What we tested

With the real Nav2 planner and `nav2_sim_params.yaml`, planning from (-3, -2.5) to (4, -2.5) past two people standing across that line:

| Distance apart | Group rule off | Group rule on |
| --- | --- | --- |
| 1.2 to 1.8 m | around | around |
| 2.0 m | between (7.0 m path) | around (8.2 m path) |
| 2.2 m | between | around |
| 2.3 m and more | between | between (they don't count as a group anymore) |

So below about 1.9 m, personal space alone already keeps the robot out of the gap. The group rule is what keeps it out of wider gaps it could easily fit through, up to `group_distance` (2.2 m, about the widest a two-person conversation usually gets).

For walkers: someone at (0.5, -1.2) walking towards the robot's path at 1 m/s. With the prediction on, the path crosses their way 2.0 m in front of them, past where they'll be in 2 s. With it off, it crosses 1.7 m in front, where they'll be in less than 2 s. The costmap is only a snapshot of one moment, so this is an approximation. The custom MPC, which can predict both the robot and the people over time, could do it properly.

### Changing the rules while it's running

Every parameter can be changed live, and it's used from the next costmap update. Change it in both costmaps. The costs are whole numbers, everything else needs a decimal point (`2.0`, not `2`):

```bash
ros2 param set /global_costmap/global_costmap social_layer.group_cost 0
ros2 param set /local_costmap/local_costmap social_layer.group_cost 0
```

`group_cost 0` turns off just the group rule, `prediction_cost 0` just the prediction, and `enabled false` all the people costs at once.

**To add a new rule:** in [social_layer.cpp](social_layer/src/social_layer.cpp), write a function like `groupCost()` that gives the cost at one point, call `paint()` with it in `updateCosts()`, and rebuild. Rules about speed instead of space (like "slow down near people") would go in the controller instead.

## The feedback signals

| Topic | What it means | Parameters (defaults) |
| --- | --- | --- |
| `/navigation/proximity_feedback` | the closest person: their side, and a magnitude from 0 at `warn_distance` up to 255 at `min_distance` or closer. Only while the robot is moving | `warn_distance` 1.2 m, `min_distance` 0.5 m, `only_when_moving` true |
| `/navigation/turn_feedback` | the next turn on Nav2's path, once it starts within `turn_warn_distance`: its side, and a magnitude that grows with the angle. Turning on the spot to set off counts too | `turn_warn_distance` 2.0 m, `min_turn_deg` 25, `full_turn_deg` 90 (that sharp or more = 255) |

Distances are measured from the middle of the robot. Both are sent 10 times a second. When the vibration switches side, we first send a 0 to the old side, because the devices keep a side vibrating until they get a 0 for it.

Because the social layer keeps the robot about 1 m from people, expect weak proximity values (around 70) in open space. You'll get stronger ones in narrow spots or when the walker comes close.

The turn signal reads Nav2's planned path. Once the custom MPC exists, its predicted trajectory would be more accurate and could feed the same topic.

## Still to do

- **The camera's position on the robot is a guess**: 0.2 m forward and 0.1 m above the top plate, level (`top_plate_camera_joint` in both `pioneer3at_lidar.xacro`). We need to measure it on the real robot, because every person and object position goes through it.
- **On the real robot, the lidar will see the camera** and the collision monitor will think something is inside the robot (this already happened in Gazebo). The robot's own points need to be cut out of the lidar cloud first.
- **Enid's tracker** is on the `enid/social-perception` branch, not on main yet. It gives positions relative to the camera, which people_node already handles. If it tracks in `odom` later, it could publish `navigoat_msgs/People` itself.
- **Failed drives**: Pete says so on `/pete/response`, but the middleman stays in location mode until the next request. `Middleman.fail()` exists, it just needs a topic.
- **The lidar layers are off in `nav2_sim_params.yaml`**, so in Gazebo Nav2 only avoids what's on the map and the people. Before turning them on, set `min_obstacle_height` above the floor, or the 3D lidar marks the floor as an obstacle.
- **Saving places by voice** is done: "Pete, remember this place as the balcony" makes the middleman say "Remembering this place as balcony." and publish the name on `/navigation/save_place`. places_node then says "Saved balcony.", or "I couldn't save balcony. I don't know where I am." when it has no position on the map, and sends the saved names to the middleman on `/navigation/places`.
