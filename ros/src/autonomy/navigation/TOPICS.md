# Navigation: topics and how it works

The reference behind [README.md](README.md) (which is only about running things). The older, longer README with the
Gazebo tests is in [README_archive.md](README_archive.md).

## What we subscribe to

| Topic | Type | Who sends it | What we do with it | Node |
| --- | --- | --- | --- | --- |
| `/middleman/mode` | `std_msgs/String` (JSON) | middleman | `location` → drive to that saved place. `object` → stop and wait for `/object_mapping`. A stop or any other mode → stop driving | places_node |
| `/object_mapping` | `geometry_msgs/PointStamped` | perception | where the object is (object mode only): we drive to 0.8 m before it | places_node |
| `/navigation/save_place` | `std_msgs/String` | anyone | save where the robot is now under that name | places_node |
| `/navigation/forget_place` | `std_msgs/String` | anyone | delete that saved place | places_node |
| `/navigation/goto` | `std_msgs/String` | anyone | drive to that saved place | places_node |
| `/navigation/goto_point` | `geometry_msgs/PointStamped` | anyone | drive to that point (any frame) | places_node |
| `/navigation/cancel` | `std_msgs/String` | anyone | stop driving (the content doesn't matter) | places_node |
| `/social_perception/tracks` | `std_msgs/String` (JSON) | Enid's people tracker | the people the camera sees, x forward / y left of the camera | people_node |
| `/clicked_point` | `geometry_msgs/PointStamped` | RViz "Publish Point" | click to add a test person, click on one to remove them | people_node |
| `/people` | `navigoat_msgs/People` | people_node | everyone on the map | social_layer, feedback_node |
| `/plan` | `nav_msgs/Path` | Nav2 | the path being followed, to know which turns are coming | feedback_node |

## What we publish

| Topic | Type | For | What's in it | Node |
| --- | --- | --- | --- | --- |
| `/navigation/status` | `std_msgs/String` | debugging | `saved:kitchen`, `going:kitchen`, `arrived:kitchen`, `failed:...`, `cancelled:...`, `unknown:...`, `waiting:chair`, `error:...` | places_node |
| `/middleman/done` | `std_msgs/String` | middleman | the name of what we drove to, once there | places_node |
| `/pete/response` | `std_msgs/String` | voice | what Pete says, e.g. "I couldn't get to the kitchen." | places_node |
| `/navigation/places` | `std_msgs/String` | middleman | JSON list of the saved place names (latched) | places_node |
| `/navigation/places_markers` | `visualization_msgs/MarkerArray` | RViz | saved places as orange arrows with names (latched) | places_node |
| `/people` | `navigoat_msgs/People` | social_layer, feedback_node | every person on the map with their velocity, 10 times a second | people_node |
| `/navigation/people_markers` | `visualization_msgs/MarkerArray` | RViz | people as cylinders (blue = real, purple = test) | people_node |
| `/handle/vibration` | `std_msgs/UInt8MultiArray` | handle | `[side, magnitude]` for the next **turn** (called `/navigation/turn_feedback` in the code, remapped in navigation.launch.py) | feedback_node |
| `/wristband/vibration` | `std_msgs/UInt8MultiArray` | wristband | `[side, magnitude]` for the closest **person** (`/navigation/proximity_feedback` in the code) | feedback_node |

places_node sends goals to Nav2 through its `navigate_to_pose` **action**, not a topic. Goals from RViz's 2D Goal Pose go
straight to Nav2 and don't show on `/navigation/status`; Nav2 reports them on `/navigate_to_pose/_action/status`
(4 = succeeded, 5 = cancelled, 6 = aborted).

## How it fits together

1. **people_node** collects everyone: the people the tracker sees (moved from the camera onto the map with TF), people
   clicked in RViz, and test people from a yaml file. It works out how fast each one walks and publishes `/people`.
2. **social_layer** is a plugin inside both Nav2 costmaps. It draws costs around each person in `/people` (see below), so
   Nav2 plans around people.
3. **places_node** decides *where* to go (saved place, object, point) and sends the goal to Nav2.
4. **Nav2** plans (Smac 2D planner) and drives (regulated pure pursuit; turns on the spot first, never reverses).
5. **feedback_node** turns the path and the people into vibrations: turns → handle, people close by → wristband.

On the real robot, `navigation_robot.launch.py` also starts the base (motors on), the lidar and RTAB-Map
(`slam/launch/mapping.launch.py`), and with `social:=true` the camera, YOLO server and people tracker.

| File | What it does |
| --- | --- |
| [navigation_nodes/launch/navigation_robot.launch.py](navigation_nodes/launch/navigation_robot.launch.py) | everything on the real robot, with all the flags |
| [navigation_nodes/launch/navigation.launch.py](navigation_nodes/launch/navigation.launch.py) | only our three nodes |
| [navigation_nodes/launch/navigation_sim.launch.py](navigation_nodes/launch/navigation_sim.launch.py) | everything in Gazebo |
| [navigation_nodes/places_node.py](navigation_nodes/places_node.py), [people_node.py](navigation_nodes/people_node.py), [feedback_node.py](navigation_nodes/feedback_node.py) | the three nodes |
| [social_layer/src/social_layer.cpp](social_layer/src/social_layer.cpp) | the costmap plugin with the social rules |
| [config/nav2_params.yaml](config/nav2_params.yaml) / [config/nav2_sim_params.yaml](config/nav2_sim_params.yaml) | Nav2 settings, real robot / Gazebo |
| [config/robot.rviz](config/robot.rviz) | RViz on the laptop for the real robot |
| [config/fastdds_usb.xml](config/fastdds_usb.xml) | makes ROS on the laptop use only the USB cable |

Saved places go next to the map (`lab.db` → `lab_places.yaml`):

```yaml
places:
  kitchen: {x: 1.25, y: -3.4, yaw: 1.571}
```

They only fit the map they were saved on. Only `x` and `y` are used to drive there: the robot arrives in whatever
direction is convenient (`yaw` is only the arrow in RViz).

## The social rules

| Rule | What it does | Parameters (defaults) |
| --- | --- | --- |
| The person | an obstacle | `person_radius` 0.3 m |
| Personal space | a cost around them that fades to 0, further in front of someone walking | `personal_space` 1.0 m, `personal_space_cost` 150, `motion_stretch` 1.0 m per m/s |
| Groups | two people closer than `group_distance` count as together; the strip between them is expensive, so the robot goes around if it can | `group_distance` 2.2 m, `group_width` 0.6 m, `group_cost` 220 |
| Where walkers are going | an expensive strip along where each walker will be in the next `prediction_time` seconds | `prediction_time` 2.0 s (0 = off), `prediction_width` 1.0 m, `prediction_cost` 200 |

Only the people themselves block; the other costs make a path more expensive but never block it. People within 0.6 m of
the robot (`ignore_radius`, the person holding the handle) are ignored. If `/people` stops for 1 s, everyone is cleared.

Change a rule live (in both costmaps; costs are whole numbers, the rest needs a decimal point):

```bash
ros2 param set /global_costmap/global_costmap social_layer.group_cost 0
ros2 param set /local_costmap/local_costmap social_layer.group_cost 0
```

`group_cost 0` turns off the group rule, `prediction_cost 0` the prediction, `enabled false` all people costs.

**Costmap colours in RViz:** pink = obstacle / person, cyan = the robot's centre can't go here, red → purple = costly
but allowed, no colour = free.

## The feedback signals

| Goes to | What it means | Parameters (defaults) |
| --- | --- | --- |
| `/wristband/vibration` (people) | the closest person: side, and 0 at `warn_distance` up to 255 at `min_distance` or closer. Only while moving | `warn_distance` 1.2 m, `min_distance` 0.5 m, `only_when_moving` true |
| `/handle/vibration` (turns) | the next turn on the path within `turn_warn_distance`: side, stronger = sharper | `turn_warn_distance` 2.0 m, `min_turn_deg` 25, `full_turn_deg` 90 |

Both 10 times a second, side 0 = left, 1 = right. Before switching side, a 0 is sent to the old side.

## Known limits on the real robot

- **The lidar only sees from ~0.5 m above the floor upwards**: low obstacles are invisible to it. A tilted (~45°) mount is planned.
- **Localisation can't find the robot from scratch** (lidar only): start where the map was started, or set the pose with RViz's 2D Pose Estimate.
- **The global costmap has no live lidar** (only the map and people), because small localisation errors stamped real walls into free corridors. New obstacles are handled by the local costmap.
- **The collision monitor is off** (no `/scan` on this robot). To re-enable: a pointcloud source on the lidar, with the robot's own points filtered out.
- **The camera position** (`top_plate_camera_joint` in `bringup/description/pioneer3at_lidar.xacro`): height, pitch and roll
  were measured from the floor on 2026-11-20 (0.61 m up, 4.6° down, -1.7° roll). The forward offset is still a guess.
  People and the floor are placed through it, so re-measure it if the camera is moved.
- **Nav2's heartbeat check is off** (`bond_timeout` 0 in navigation_robot.launch.py): it kept switching Nav2 off at startup. A stuck controller isn't noticed, but the base stops by itself without commands.
