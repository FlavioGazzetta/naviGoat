# Person following

Select the nearest confirmed, observed person when enabled, retain that track
ID, and generate destinations about one metre behind them. Nav2 handles path
planning and velocities. This subsystem reuses the team's shared person tracks;
it does not load YOLO, run a camera driver or publish `cmd_vel`.

```text
Camera → shared YOLO server → person tracker → /social_perception/tracks
                                                       ↓
Lia's middleman → /middleman/mode → person following: select + monitor target
                                                       ↓ capture-time TF
                                             one-metre standoff goal
                                                       ↓
                                     /person_following/goal_preview
                                                       ↓ --execute-nav2 only
                                            NavigateToPose → Nav2 → base
```

Preview is the default. Actual navigation requires `--execute-nav2`. Run one
following runner at a time. Laptop tests pass; live ROS, Jetson and moving-robot
behaviour still need validation.

## Files

| File | Responsibility |
| --- | --- |
| `state_machine.py` | Enable/disable, nearest-person selection, target locking, loss and freshness checks |
| `perception_adapter.py` | Validate shared tracking JSON and preserve capture time and processing latency |
| `mode_bridge.py` | Interpret middleman commands and correlate status using request IDs |
| `goal_generator.py` | Transform planar target positions and calculate standoff goals, tolerance and update thresholds |
| `nav2_goal_client.py` | Asynchronous goal acceptance, preemption, cancellation and timeouts |
| `ros_nav2_follow.py` | ROS node connecting tracks, TF, mode/status, previews and optional Nav2 actions |
| `run_nav2_follow.sh` | Launch this node inside the existing team ROS environment |
| `goal_demo.py` | Laptop demonstration with invented observations and transforms |
| `tests/` | Offline checks for tracking, commands, geometry, action lifecycle and launcher |

The node is the ROS interface for person following. Its filename mentions Nav2
because that is how it asks the robot to move. The state machine calculates no
velocities; physical deceleration belongs to the navigation stack.

## Laptop checks

Python 3.9 or newer, standard library only. From the repository root:

```bash
python3 -B ros/src/autonomy/person_following/goal_demo.py
python3 -B ros/src/autonomy/person_following/ros_nav2_follow.py --help
```

Run the tests from the subsystem directory:

```bash
cd ros/src/autonomy/person_following
python3 -B -m unittest discover -s tests
```

The demo shows selection, target locking, goal updates, holding position,
temporary loss, resumption and eventual exit. It does not simulate robot physics.
ROS and hardware are not required for these checks.

## Start shared perception on the Jetson

Keep the camera stationary and level for the first test. Use the team's existing
image/container with `PETE=true` and `SOCIAL=true` support (`VOICE=true` for a
microphone). There is no separate following Docker image to build.

If Pete already runs the camera, shared detector, tracker and middleman with the
correct track lifetime, keep it running. Otherwise, from the repository root on
the Jetson, start the team's host launcher:

```bash
bash ros/src/core/start_pete.sh people:=true people_max_age:=2.0 use_qwen:=false
```

It creates `navigoat-dev`, mounts the checkout at `/workspace`, and starts Pete's
shared stack. `use_qwen:=false` disables the middleman's optional interpretation
for typed-command testing; scene description still starts its model. Wait for
the shared YOLO server and tracker to become ready. For microphone testing add
`voice:=true`; that launch already includes the middleman.

If starting within an existing compatible container instead, source ROS and run:

```bash
ros2 launch /workspace/ros/src/core/launch/pete.launch.py \
  people:=true people_max_age:=2.0 use_qwen:=false
```

Choose one launch path. Set `people_max_age` to match the following loss window
(default two seconds). Pete retains its one-second default for other users.

| Shared component | Interface |
| --- | --- |
| YOLO server | HTTP `127.0.0.1:8767`; the tracker uses `--yolo-url` |
| Person tracker | `/social_perception/tracks`, JSON in `std_msgs/String` |
| Dashboard | Port 8766 by default |
| Voice/middleman | `/middleman/mode` and `/pete/response` |

Object perception and the person tracker request detections from one server.
Following subscribes to the resulting tracks, without another model or HTTP
client. It uses the original tracks rather than `/people` so that actual
observations, predictions and capture-time freshness remain distinguishable.

## Preview with a camera only

In another Jetson terminal:

```bash
sudo docker exec -it navigoat-dev \
  bash /workspace/ros/src/autonomy/person_following/run_nav2_follow.sh \
  --global-frame camera_link --base-frame camera_link
```

This treats the camera as the hypothetical robot origin and requires no Nav2
action server. The runner starts idle; use the middleman commands below, or add
`--enable-on-start` for a standalone test. Preview cannot command movement.

With localisation and robot TF running, omit the frame overrides to preview in
`map` using the actual `base_link` position. Target transforms use the image's
capture timestamp; missing historical TF causes an exit. The latest robot pose
must also be fresh. Frames tilted more than about three degrees are rejected
because the tracker supplies no vertical target coordinate.

Processes must share a ROS domain and source-clock configuration. The live
adapter preserves capture latency and rejects stale/future timestamps; bag or
simulation clock setup is not supplied by this launcher.

## Lia's voice/middleman connection

In another terminal, enter the running container and source ROS:

```bash
sudo docker exec -it navigoat-dev bash
source /opt/ros/jazzy/setup.bash
```

Send each request separately:

```bash
ros2 topic pub --once /speech/text std_msgs/msg/String "{data: 'pete start following'}"
ros2 topic pub --once /speech/text std_msgs/msg/String "{data: 'pete stop following'}"
ros2 topic pub --once /speech/text std_msgs/msg/String "{data: 'pete manual mode'}"
```

Inspect the outputs from another ROS terminal:

```bash
ros2 topic echo /person_following/status
ros2 topic echo /person_following/goal_preview
ros2 topic echo /pete/response
ros2 topic hz /social_perception/tracks
```

| Interface | Type | Purpose |
| --- | --- | --- |
| `/middleman/mode` | `std_msgs/String` JSON | Receive `{mode, target, intent, stop, request_id}` |
| `/person_following/status` | `std_msgs/String` JSON | Publish `{request_id, state, event, target_id, navigation, navigation_pending}` |
| `/person_following/goal_preview` | `geometry_msgs/PoseStamped` | Show generated destinations; this topic does not move the robot |
| `/person_following/set_enabled` | `std_srvs/SetBool` service | Manual enable/disable for tests |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Optional navigation goals and cancellation |

An empty-target follow command enables acquisition. A named contact is rejected
until identity recognition exists. Stop or a different mode disables following.
Request IDs prevent old/duplicate failures from ending a newer session; both
middleman and following need the updated code. Following does not publish
`/middleman/done`: reaching an intermediate goal does not finish following or say
"Arrived". Manual sessions do not acquire ownership of a voice request.

## Behaviour and timing

- Select the nearest confirmed, observed ID by horizontal camera range once;
  a closer newcomer cannot replace it. Predicted tracks cannot refresh sightings.
- Generate a goal one metre from the target, on the robot's side and facing it.
  This is radial distance from `base_link`, not front-edge clearance. Within
  1.15 m, cancel navigation and hold instead of requesting retreat.
- Check state and freshness at 20 Hz. Send goal updates at most 2 Hz, with
  0.15 m position / 0.2 rad heading thresholds. Status is sent on events and
  approximately twice per second.
- Missing target: cancel navigation immediately and retain its ID briefly.
  The same ID can resume within two seconds of the last actual sighting.
  After two seconds, exit and report `target_lost` to the middleman. An explicit
  new follow request is then required.
- No fresh tracking input for more than 0.5 s, invalid input, unavailable TF or
  navigation failures: cancel and exit. Deadlines take effect on control ticks.
- Updated goals preempt old ones. Stop invalidates pending requests and cancels
  late acceptances. A new session waits for old cancellation to finish.
  Acknowledgement and terminal action results are checked separately; cancellation
  failure is reported rather than treated as confirmation that motion stopped.
- Ctrl+C/SIGTERM requests cancellation and keeps processing ROS responses for up
  to the action timeout. An abrupt process kill or network loss cannot guarantee
  stopping; the navigation/base watchdog remains necessary.

The old direct-velocity slowdown is not part of this path. Nav2 owns actual speed
limits, deceleration, reversing, recoveries and goal tolerances. Its current XY
goal tolerance is 0.25 m, larger than this following band; tune these together
with the navigation team before judging physical following accuracy.

## Navigation execution and remaining validation

Pete's perception launch does not start Nav2. Actual execution also requires the
team's Nav2/localisation bringup, a working action server and capture-time camera
to navigation-frame TF. Useful checks in that ROS environment:

```bash
ros2 action info /navigate_to_pose
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link camera_link
```

After controlled/simulated validation and addressing the tracking limitation
below, restart the one following runner with action execution enabled:

```bash
sudo docker exec -it navigoat-dev \
  bash /workspace/ros/src/autonomy/person_following/run_nav2_follow.sh --execute-nav2
```

This can move the robot. Goals default to `map`; frame names, action name,
distance, rates and timeouts are configurable with `--help`.

The upstream tracker associates camera-relative positions under a stationary,
level-camera assumption. Transforming its selected output into `map` does not
repair ID association when the robot moves. Stationary-camera preview is the
first milestone; ego-motion compensation, crossing/occlusion checks, live ROS
integration and physical stopping tests remain before dependable robot following.

Superseded velocity demos and standalone perception/Docker copies have been
removed from the active tree. A local pre-cleanup archive is ignored by Git;
previously committed versions also remain available in Git history.
