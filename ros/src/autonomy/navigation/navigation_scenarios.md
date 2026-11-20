# Navigation in Gazebo: test scenarios

Three runs of the full navigation stack (Gazebo, RTAB-Map, Nav2, our navigation nodes) in the indoor world
(`ros/src/simulation/worlds/indoor.sdf`), with the same RViz as the real robot ([config/robot.rviz](config/robot.rviz)):

| # | Scenario | People file |
| --- | --- | --- |
| 1 | No one | [people_none.yaml](../../simulation/config/people_none.yaml) |
| 2 | People standing still | [people_static.yaml](../../simulation/config/people_static.yaml) |
| 3 | People walking | [people_moving.yaml](../../simulation/config/people_moving.yaml) |

Each run does the same route, so the recordings can be compared. It covers **saving a place**, **going to a saved
place**, **going to a point** and **going to an object**:

```
start (0, 0) ──goto_point──> desk (3.5, -3) ──goto_point──> door (-3.5, 3) ──goto desk──> desk ──object──> chair (-3.5, -3.5) ──goto start──> start
```

The room is 10 × 10 m (walls at ±5 m), with a 1 × 1 m box at (2, 1). The robot spawns at (0, 0) facing +x, so map
coordinates are world coordinates.

> The people only exist in RViz (purple cylinders, with an arrow in front of walkers) and in Nav2's costmaps (the
> social layer). Gazebo's window and the lidar don't show them, so **record RViz**.

## 1. Setup (once)

Run the simulation **on the laptop**: Gazebo needs a screen, and everything stays on one machine. All the commands
below run in laptop terminals.

**First, keep the Jetson out of it.** Stop any launch running on the Jetson (Ctrl+C), and close an RViz started with
the real-robot settings: over the USB cable the laptop would otherwise see the Jetson's Nav2, `/tf` and `/clock` mixed
with the simulation's.

### On the laptop directly (ROS Jazzy and Gazebo installed)

From the repository root, build once:
```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to p2os_urdf navigation_nodes social_layer middleman
```

**In every terminal** first (from the repository root):
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
unset FASTRTPS_DEFAULT_PROFILES_FILE ROS_STATIC_PEERS
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST     # ROS stays on the laptop, even with the Jetson plugged in
SIM=$PWD/ros/src/autonomy/navigation/navigation_nodes/launch/navigation_sim.launch.py
PEOPLE=$PWD/ros/src/simulation/config
```

### Or in the container

From the repository root:
```bash
xhost +si:localuser:root
docker run --rm -it --name navigoat-dev --network=host -e DISPLAY -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -v "$PWD:/workspace" -v /tmp/.X11-unix:/tmp/.X11-unix:ro navigoat:dev bash
```
More terminals: `docker exec -it navigoat-dev bash`. Build once with the `colcon build` above (from `/workspace`), then
in every container terminal:
```bash
source /workspace/install/setup.bash
SIM=/workspace/ros/src/autonomy/navigation/navigation_nodes/launch/navigation_sim.launch.py
PEOPLE=/workspace/ros/src/simulation/config
```

### Running it on the Jetson instead (not recommended)

Over SSH the Jetson has no screen, so plain `gz sim` exits straight away and nothing runs (no `/clock`). Add
`headless:=true rviz:=false` to the launch commands below (Gazebo's server only, rendering on the GPU), and on the
laptop start RViz with the simulation's clock:
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/ros/src/autonomy/navigation/config/fastdds_usb.xml
rviz2 -d ros/src/autonomy/navigation/config/robot.rviz --ros-args -p use_sim_time:=true
```
Gazebo, RTAB-Map and Nav2 together are heavy for the Jetson, so expect it to run slower than real time.

### The map

The scenarios navigate on the saved map `ros/src/simulation/maps/indoor.db` (localisation mode). If it doesn't exist
yet, make it once:
```bash
ros2 launch $SIM mode:=mapping                                           # terminal 1
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/model/pioneer3at/cmd_vel -p speed:=0.3 -p turn:=0.5   # terminal 2
```
Drive around the whole room, come back near (0, 0), then Ctrl+C both and wait for RTAB-Map to save. Places saved on an
older map don't match a new one: delete `ros/src/simulation/maps/indoor_places.yaml` if you re-map.

## 2. Launch a scenario

Terminal 1, **one** of these (one launch at a time):

| Scenario | Command |
| --- | --- |
| 1. No one | `ros2 launch $SIM fake_people:=$PEOPLE/people_none.yaml` |
| 2. People static | `ros2 launch $SIM fake_people:=$PEOPLE/people_static.yaml` |
| 3. People moving | `ros2 launch $SIM fake_people:=$PEOPLE/people_moving.yaml` |

- RViz opens with the real robot's config (`robot.rviz` is now the default of `navigation_sim.launch.py`). Another
  config: add `rviz_config:=/path/to/file.rviz`.
- Wait until RViz shows the map and the robot, and the launch output prints `Managed nodes are active`.
- The launch output prints `People ready: N standing and M walking fake people`. Check it says 0/0, 4/0 or 0/3.
- Walkers start walking when the launch starts, so where they are on each leg depends on timing.

Terminal 2, watch what navigation does (leave it open for the whole run):
```bash
ros2 topic echo /navigation/status std_msgs/msg/String --no-daemon --field data
```

Terminal 3, check the robot is localised at the start (it should print about x 0, y 0):
```bash
ros2 topic echo /localization_pose geometry_msgs/msg/PoseWithCovarianceStamped --once --no-daemon --field pose.pose.position
```
If it doesn't, or the robot in RViz isn't on the map, use RViz's **2D Pose Estimate** at (0, 0) pointing along +x.

### Clean slate before recording (optional)

The places are kept in `ros/src/simulation/maps/indoor_places.yaml` between runs, so older places (the previous run's,
or ones like `kitchen` saved before) are still in RViz. To start each recording without any, forget every saved place:
```bash
for p in $(ros2 topic echo /navigation/places std_msgs/msg/String --once --qos-durability transient_local --qos-reliability reliable --no-daemon --field data | head -1 | tr -d "[]\"',"); do ros2 topic pub --once /navigation/forget_place std_msgs/msg/String "{data: $p}"; done
```
It prints `forgot:<name>` in the status terminal for each one. Names with a space in them (`living room`) have to be
forgotten by hand: `ros2 topic pub --once /navigation/forget_place std_msgs/msg/String "{data: living room}"`.
Or, with the simulation stopped: `rm ros/src/simulation/maps/indoor_places.yaml`.

## 3. The route (the same commands in all three scenarios)

Terminal 3. Send each step **only after terminal 2 shows `arrived:...`** for the previous one.

**Step 1: save a place.** The robot is at the start. Save it:
```bash
ros2 topic pub --once /navigation/save_place std_msgs/msg/String "{data: start}"
```
Status: `saved:start`. An orange arrow labelled `start` appears in RViz.

**Step 2: go to a point, then save it as a place.**
```bash
ros2 topic pub --once /navigation/goto_point geometry_msgs/msg/PointStamped "{header: {frame_id: map}, point: {x: 3.5, y: -3.0}}"
```
Status: `going:point 3.50,-3.00` → `arrived:point 3.50,-3.00`. Then:
```bash
ros2 topic pub --once /navigation/save_place std_msgs/msg/String "{data: desk}"
```

**Step 3: the same on the other side of the room.**
```bash
ros2 topic pub --once /navigation/goto_point geometry_msgs/msg/PointStamped "{header: {frame_id: map}, point: {x: -3.5, y: 3.0}}"
```
After `arrived:point -3.50,3.00`:
```bash
ros2 topic pub --once /navigation/save_place std_msgs/msg/String "{data: door}"
```

**Step 4: go to a saved place.** All the way across the room, back to the desk:
```bash
ros2 topic pub --once /navigation/goto std_msgs/msg/String "{data: desk}"
```
Status: `going:desk` → `arrived:desk`. It arrives facing the way it faced when the place was saved.

**Step 5: go to an object.** Act as the middleman and perception. First switch to object mode (otherwise the object's
position is ignored):
```bash
ros2 topic pub --once /middleman/mode std_msgs/msg/String "{data: '{\"mode\": \"object\", \"target\": \"chair\", \"stop\": false}'}"
```
Status: `waiting:chair`. Then send where the chair is, on the map:
```bash
ros2 topic pub --once /object_mapping geometry_msgs/msg/PointStamped "{header: {frame_id: map}, point: {x: -3.5, y: -3.5}}"
```
Status: `going:chair` → `arrived:chair`. The robot stops 0.8 m before the chair, facing it.

Or, like the real camera, relative to the robot's camera (x = metres in front, y = metres to the left):
```bash
ros2 topic pub --once /object_mapping geometry_msgs/msg/PointStamped "{header: {frame_id: camera_link}, point: {x: 2.0, y: 0.0}}"
```
For this one, send `/middleman/mode` again first: it waits for one object per request.

**Step 6: go back to the start.**
```bash
ros2 topic pub --once /navigation/goto std_msgs/msg/String "{data: start}"
```

### Other useful commands

```bash
ros2 topic pub --once /navigation/cancel std_msgs/msg/String "{data: ''}"                       # stop driving
ros2 topic echo /navigation/places std_msgs/msg/String --once --qos-durability transient_local --qos-reliability reliable --no-daemon   # saved places
ros2 topic echo /handle/vibration std_msgs/msg/UInt8MultiArray --no-daemon --field data      # what the handle would get: next turn
ros2 topic echo /wristband/vibration std_msgs/msg/UInt8MultiArray --no-daemon --field data   # what the wristband would get: closest person
```
RViz **Publish Point**: click on the map to add a standing person, click on them again to remove them.

## 4. What to expect

| Step | 1. No one | 2. People static | 3. People moving |
| --- | --- | --- | --- |
| 2: start → desk | straight diagonal | goes around A at (1.2, -2.1) | gives way to W1 crossing at x = 1.75 |
| 3: desk → door | straight diagonal | around A again, then about 1 m clear of B at (-1.75, 1.5) | W1, then W2 (crossing at y = 1.5) |
| 4: door → desk | straight diagonal | same as step 3, reversed | depends on where W1 and W2 are |
| 5: desk → chair | straight along y ≈ -3.3 | around pair C (2.0 m apart): the gap is wide enough, but the group rule keeps it out | meets W3 walking along y = -2.2 |
| 6: chair → start | straight | passes pair C | crosses W3's line |

The robot should pass behind walkers rather than cut in front (the social layer adds cost where each walker will be in
the next 2 s). The wristband only gets a value when someone is within 1.2 m, so expect weak or no values in the open.

**Group rule on and off (scenario 2, optional).** After step 5, turn the group rule off (section 5), then drive the same
leg again:
```bash
ros2 topic pub --once /navigation/goto std_msgs/msg/String "{data: desk}"
ros2 topic pub --once /middleman/mode std_msgs/msg/String "{data: '{\"mode\": \"object\", \"target\": \"chair\", \"stop\": false}'}"
ros2 topic pub --once /object_mapping geometry_msgs/msg/PointStamped "{header: {frame_id: map}, point: {x: -3.5, y: -3.5}}"
```
Now it goes through the gap between the two people of pair C.

## 5. Group rule on and off

The group rule keeps the robot from passing between two people who are together (closer than 2.2 m), even when the gap
is wide enough. It is the social layer's `group_cost` (see [TOPICS.md](TOPICS.md)): 0 is off, 220 (the default) is on.
It works while the simulation runs, from the next costmap update, and has to go to both costmaps.

**Off:**
```bash
ros2 param set /global_costmap/global_costmap social_layer.group_cost 0
ros2 param set /local_costmap/local_costmap social_layer.group_cost 0
```

**On:**
```bash
ros2 param set /global_costmap/global_costmap social_layer.group_cost 220
ros2 param set /local_costmap/local_costmap social_layer.group_cost 220
```

Check what is set now:
```bash
ros2 param get /global_costmap/global_costmap social_layer.group_cost
ros2 param get /local_costmap/local_costmap social_layer.group_cost
```

With it off, personal space and the walker prediction stay on. It shows best with pair C in scenario 2 (2.0 m apart,
across the desk -> chair leg): on, the robot goes around them; off, it goes through the gap. A relaunch always starts with
the rule on.

## 6. Recording the data too (optional)

Besides the screen recording, a bag of the run (terminal 4, start it before step 1, Ctrl+C after step 6):
```bash
mkdir -p bags
ros2 bag record -o bags/sim_static_$(date +%H%M) /clock /tf /tf_static /odom /map /plan \
  /global_costmap/costmap /local_costmap/costmap /people /navigation/status /navigation/people_markers \
  /navigation/places_markers /handle/vibration /wristband/vibration /robot_description
```
Change `sim_static` to `sim_none` / `sim_moving`. Replay it with `ros2 bag play <folder> --clock` and RViz with
`rviz2 -d ros/src/autonomy/navigation/config/robot.rviz --ros-args -p use_sim_time:=true`.

## 7. Between scenarios

One Ctrl+C in terminal 1, wait 10–15 s until everything has stopped, then launch the next scenario. Gazebo restarts,
so the robot is back at (0, 0). The saved places stay in `ros/src/simulation/maps/indoor_places.yaml`, and saving the same name again
overwrites it.

## When something's wrong

| Symptom | Likely cause |
| --- | --- |
| `error:no transform from ... to map` | RTAB-Map hasn't localised yet: wait, or use 2D Pose Estimate at (0, 0) |
| `error:Nav2 is not running` | Nav2 hasn't started yet (`Managed nodes are active`), or it switched itself off: restart the launch |
| Path drawn but the robot doesn't move | check `ros2 topic echo /model/pioneer3at/cmd_vel geometry_msgs/msg/Twist`. All zeros: the collision monitor is stopping it |
| `failed:...` next to a person | a walker was standing on the goal at that moment: send the step again |
| No purple cylinders | `fake_people:=` path wrong (the launch output says `0 standing and 0 walking`) |
| Old place arrows in the wrong spots | `indoor_places.yaml` is from an older map: forget them (section 2) or delete the file |
