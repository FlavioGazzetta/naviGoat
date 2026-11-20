# Navigation tuning log: 2026-11-20

A record of what we looked at and tried for navigation, and why, so the report can go through the different attempts.

Status tags: **changed, not tested yet** / **tested offline only** / **idea, not tried** / **tried, rejected**

## Setup at this point

- **Real robot** (`config/nav2_params.yaml`): Smac 2D planner, and the Rotation Shim with Regulated Pure Pursuit (RPP)
  as the controller. The MPPI settings are still in the file as `FollowPathMPPI`, but they aren't loaded.
- **Sim** (`config/nav2_sim_params.yaml`): Smac Hybrid-A* planner (Dubins, 0.4 m turning radius) with MPPI (DiffDrive).
- **Footprint** is a polygon, not a circle: 0.52 x 0.52 m on the real robot, 0.60 x 0.52 m in sim. `robot_radius` is
  ignored when a footprint is set.
- **MPPI** uses `CostCritic`, not `ObstaclesCritic`, so there is no `collision_margin_distance`. The clearance comes
  from the inflation layer.
- **Collision monitor**: its polygon is `FootprintApproach`, the robot's footprint projected 1.2 s ahead along the
  current velocity. It's on in sim and off on the real robot.
- **Real global costmap**: it has no live obstacle layer (see the earlier attempts at the bottom). With RPP, that means
  an obstacle that isn't on the map makes the robot stop. It doesn't drive around it.

## Problems seen

1. When Nav2 can't plan or follow the path, the robot spins on the spot (and backs up).
2. The robot moves more than it needs to, keeps far from walls and furniture, and gets stuck.
3. On the depth-camera map, a table is seen from far away but forgotten up close, once the camera sees under the
   top. Some simple obstacles are never picked up at all, while walls come out clean.
4. People in the costmap lag behind the real person, the walking-direction (prediction) cost is rarely visible, and
   no group cost showed up in real tests.
5. At the end of a route the robot works hard to face one particular direction. For a saved place that's the way it
   faced when the place was saved, which a voice command can't even ask for.

## Attempts

### A. Stop instead of spinning when stuck: changed, not tested yet

- **Cause**: Nav2's default behaviour tree cycles through its recoveries (clear the costmaps, Spin 90°, Wait 5 s,
  BackUp 0.3 m), up to 6 times.
- **Change**: two trees of our own, `behavior_trees/navigate_to_pose_no_spin.xml` and
  `behavior_trees/navigate_through_poses_no_spin.xml`. They're the default trees with Spin and BackUp taken out.
  - When stuck, the robot clears the costmaps, waits 3 s and tries again.
  - After 3 tries the goal fails and the robot stays stopped.
  - `navigation_robot.launch.py` and `simulation.launch.py` load them, writing the absolute paths in with
    `RewrittenYaml`.
- **Why**: spinning or backing up while someone holds the handle is unsafe and confusing for them.
- **Side effect**: BackUp was the only way the robot could ever reverse (`vx_min` is 0 in sim). In a dead end it now
  stops and the goal fails, instead of backing out.
- **Not affected**: the Rotation Shim's on-the-spot turns (when the path points more than 45° away) are normal
  driving, not a recovery, so they still happen on the real robot.

### B. Smoother, cheaper inflation so the robot passes closer: changed, not tested yet

The inflation cost is `252 * exp(-factor * (d - 0.26))`, where d is the distance to the obstacle and 0.26 m is the
footprint's inscribed radius.

**Problem with the old values**: the radius cut the cost off while it was still high. The cost jumped from ~90 straight
to 0 at the edge of the inflation, so the path and speed jumped whenever the robot crossed that edge.

| costmap | before (factor / radius) | after | cost at the cutoff, before -> after |
|---|---|---|---|
| local, real | 5.0 / 0.45 m | 5.0 / 0.8 m | ~97 -> ~17 |
| global, real | 3.0 / 0.6 m | 5.0 / 0.9 m | ~91 -> ~10 |
| local, sim | 3.0 / 0.7 m | 5.0 / 0.8 m | ~67 -> ~17 |
| global, sim | 3.0 / 0.7 m | 5.0 / 0.9 m | ~67 -> ~10 |

- **Closer to walls**: with the steeper factor, the global cost 0.4 m from a wall goes from ~165 to ~125. The middle
  of a 1.5 m corridor (~22) is still a little cheaper than its sides.
- **RPP**: `cost_scaling_dist` 0.3 -> 0.2, so it only slows down when it's really close to something, e.g. in
  doorways.

### C. Global costmap updated at 3 Hz instead of 1 Hz (real and sim): changed, not tested yet

People were drawn on the global costmap up to 1 s late, and with RPP the planner is the only thing that steers around
them. This costs more CPU on the Jetson, so watch it.

### D. Less lag on people (`navigation_nodes/people_node.py`): tested offline only

- **Ghost tracks**: the tracker's Kalman filter works in the camera frame and assumes the camera doesn't move. When it
  misses a detection, it outputs a prediction (`observed: false`), and that prediction drifts as soon as the robot
  moves. We used to treat those predictions as measurements; now they're ignored, since `people_node` keeps its own
  memory of people.
- **Latency**: YOLO over HTTP plus the tracker take a few tenths of a second. Walkers are now moved forward by the age
  of the image they were seen in, up to 0.5 s (`max_extrapolation`).
- **Velocity**: it was a line fit over the last 1.0 s, available only after 0.5 s of data. So a person counted as
  standing for the first 0.5-1 s of walking, which is why the prediction cost barely showed. Now `velocity_window` is
  0.8 s and `min_velocity_span` 0.4 s. Shorter gets the speed sooner but makes it noisier.
- **Offline check** with fake tracker messages: a walker at 1 m/s was published 0.3 s ahead of the last image, its speed
  came out at 1.0 m/s, and the ghost track was dropped.
- **RViz**: part of the lag we saw is only on screen. The local costmap is published at 2 Hz and the global at 1 Hz.

### E. Goals are positions only, any heading: tested offline only

- **Problem**: every goal carried a heading the robot had to match on arrival (goal checker: within 0.25 rad).
  - Saved places used the heading the robot had when the place was saved.
  - Real robot: nothing turns it to that heading at the end (the Rotation Shim's `rotate_to_goal_heading` and RPP's
    `use_rotate_to_heading` are both off). So it reached the spot, fiddled there, and the progress checker gave up.
  - Sim: the Hybrid-A* planner plans to the exact heading, so the robot looped round to arrive facing the saved
    direction. MPPI's `GoalAngleCritic` also twisted it toward that heading over the last 0.5 m.
- **Change**:
  - Goal checker, real and sim: `yaw_goal_tolerance` 0.25 -> 3.15. A heading difference is at most pi, so any
    direction counts. "Arrived" now means within `xy_goal_tolerance` (0.25 m) of the goal's position.
  - `navigation_nodes/places_node.py`: a saved place's goal heading is now the direction from the robot to the place
    (the way it naturally arrives), not the saved yaw. Places are still saved with their yaw, but only the RViz arrow
    uses it. Points and objects already used the direction of travel.
  - Sim MPPI: `GoalAngleCritic` off.
  - Docs updated to match: `TOPICS.md`, `README.md`, `navigation_scenarios.md`, `design.md`.
- **Why**: voice commands can't ask for a direction, and a guide robot only needs to get the person to the place.
- **Limits**:
  - The sim's Hybrid-A* still needs a heading, and the installed Nav2 (1.3.13) has no "any heading" mode (newer Nav2
    has `goal_heading_mode`). It gets the straight-line direction, so if the route arrives from another side the robot
    may still curve at the end. Smac 2D in sim (like the real robot) would remove that, but would lose the start that
    curves from the robot's current heading.
  - The arrival radius is `xy_goal_tolerance`. RPP slows over the last 0.6 m, so 0.3-0.4 m would end routes sooner.
  - The 0.25 m is measured from the end of the planned path (Nav2's controller compares the robot to the path's last
    pose). That's the goal itself unless the goal is blocked, e.g. a person or a chair on the spot. Then the planner
    ends the path at the nearest free spot within its `tolerance` (0.5 m), so the robot can report "arrived" up to
    ~0.75 m from the requested point instead of failing.
- **Offline check**: with the robot at (0, 0) and a place at (0, 2) saved facing 3.14 rad, the goal went out with
  1.571 rad, straight at the place. Without a robot position it falls back to the saved yaw.

## Ideas not tried yet

### MPPI on the real robot again

- **For**:
  - It steers around obstacles in the local costmap at 20 Hz. RPP only stops and waits for the planner.
  - It reacts to people much faster.
  - The local costmap is in the odom frame, so the localisation offset that made us remove the global obstacle layer
    doesn't affect it.
  - We left MPPI because it wandered off the path. That may have come from the noisy costmaps and the inflation cliff
    (B).
- **Against**:
  - CPU on the Jetson. The real config runs 2000 samples x 56 steps at 20 Hz; it would need to come down to about
    1000 x 40.
  - The real MPPI config has `vx_min: -0.35`, so it can reverse into the person on the handle. RPP has
    `allow_reversing: false` for exactly that reason.
- **Test**: load both controllers and switch between them with `/controller_selector`, then drive the same route with
  each and compare stuck events and CPU.

### People tracker in the odom frame

Transform the detections into odom before the Kalman filter, so tracks don't lag or drift while the robot turns.
(This is Enid's tracker, `perception/social_perception/femto_id_velocity_tracker`.)

### Depth camera / RTAB-Map (`slam/config/rtabmap_hardware.yaml`)

- **Small obstacles missed**: `Grid/NormalsSegmentation` is on by default there (the sim turns it off). With it on,
  `Grid/MinClusterSize: 10` drops small objects after the 5 cm voxel filter. Try `Grid/MinClusterSize: 3`, or turn
  segmentation off.
- **Table forgotten**: RTAB-Map's 2D grid is a vote per cell. Close to the table the top is out of view, and the floor
  under it keeps voting "free" until it wins (`GridGlobal/OccupancyThr` 0.5, `ProbHit` 0.7, `ProbMiss` 0.4).
  - Stickier settings: `ProbMiss` 0.45 and `OccupancyThr` ~0.35. Moving people would then leave trails for longer.
  - Or map with the lidar instead, which looks upward and sees tabletops.
- **Camera pose** in `bringup/description/pioneer3at_lidar.xacro` is still a guess. A 2° pitch error is 10 cm at 3 m,
  which is all of `Grid/MaxGroundHeight`. Measure it.
- **Point cloud**: thin out `/camera/depth/points` before the voxel layer, because the full cloud is heavy for the
  Jetson. Black, shiny or thin objects are hard for a ToF camera whatever the settings.

### Groups never seen on the real robot

The layer's logic looks right: any two people 2.2 m apart or closer get a 0.6 m wide strip at cost 220 between them.
To test, have two people stand 1.5 m apart and run `ros2 topic echo /people`:
- **One entry**: it's a detection problem. YOLO at 320 px merges people standing side by side, and the tracker's
  1.0 m gate can merge two tracks into one.
- **Two entries but no strip**: it's a problem in the layer.

## Trials

**Rules for every attempt**
- Same route, same start pose and same speed, so only the params change.
- The route passes a table and a few small obstacles (a box, a bag, chair legs).
- If we can, record the route once as a rosbag and replay it for each attempt.
- Pictures go in `design_log/images/`. Take each one from the same RViz view.

### 1. RTAB-Map, camera only: params vs map

**How to run an attempt**
- Launch with `use_lidar:=false use_depth_camera:=true`.
- Use a **new database file every time** (e.g. `database_path:=.../cam_03.db`). RTAB-Map reloads the params saved in an
  existing database.
- The params are in `slam/config/rtabmap_hardware.yaml`, which the lidar mode uses too. Put it back after the trials,
  or give the camera its own file.

**Baseline (#0)**: the camera-only values right now.

| param | value | set in |
|---|---|---|
| `Grid/Sensor` / `Reg/Strategy` | 1 (depth) / 0 (visual) | `slam.launch.py`, camera only |
| `Grid/3D` | true | `rtabmap.yaml` |
| `Grid/NormalsSegmentation` | true | RTAB-Map default |
| `Grid/MinClusterSize` / `Grid/ClusterRadius` | 10 / 0.1 m | default |
| `Grid/MaxGroundAngle` / `Grid/NormalK` | 45° / 20 | `rtabmap_hardware.yaml` |
| `Grid/MinGroundHeight` / `Grid/MaxGroundHeight` | -0.3 / 0.1 m | `rtabmap_hardware.yaml` |
| `Grid/MaxObstacleHeight` | 1.5 m | `rtabmap_hardware.yaml` |
| `Grid/RayTracing` | true | `rtabmap_hardware.yaml` |
| `Grid/NoiseFilteringRadius` | 0 (off) | `rtabmap_hardware.yaml` |
| `Grid/RangeMax` | 5.0 m | default |
| `Grid/DepthDecimation` / `Grid/CellSize` | 4 / 0.05 m | default |
| `GridGlobal/OccupancyThr` / `ProbHit` / `ProbMiss` | 0.5 / 0.7 / 0.4 | default |

**Results**

| # | date | params changed (vs #0) | map | table kept? | small obstacles seen | walls / noise | score /5 | notes |
|---|---|---|---|---|---|---|---|---|
| 0 | | baseline | | | | | | |
| 1 | | `Grid/MinClusterSize: 3` | | | | | | planned |
| 2 | | `Grid/NormalsSegmentation: false` | | | | | | planned |
| 3 | | `GridGlobal/ProbMiss: 0.45`, `GridGlobal/OccupancyThr: 0.35` | | | | | | planned |
| 4 | | `Grid/RangeMax: 3.0` | | | | | | planned |
| 5 | | `Grid/MaxGroundHeight: 0.05` | | | | | | planned, only after measuring the camera pose |
| 6 | | best of 1-5 combined | | | | | | |

To fill the map cell: `<img src="images/cam_01.png" width="300">`

### 2. Controller: RPP vs MPPI

Run on the map we pick from table 1, with the same route and goals. To switch controller, swap the names `FollowPath`
and `FollowPathMPPI` in `config/nav2_params.yaml`.

| # | date | controller + key params | route | time to goal | stuck / stops | spins / reversals | CPU of `controller_server` | smooth? /5 | notes |
|---|---|---|---|---|---|---|---|---|---|
| 1 | | RPP + Rotation Shim (current, 0.3 m/s) | | | | | | | |
| 2 | | MPPI: sim critics, `batch_size` 1000, `time_steps` 40, `vx_min` 0 | | | | | | | planned |

To read the CPU: open `htop`, filter on `controller_server`, and note a rough average while the robot drives.

## Earlier attempts (from git history, config comments and sim tests)

| date | change | why / result |
|---|---|---|
| 2026-11-16 | Sim: MPPI with `vx_min` -0.4 | reversed whole routes -> set `vx_min` to 0 |
| 2026-11-16 | Sim: MPPI DiffDrive + Smac Hybrid-A* Dubins (r 0.4 m) | curves while driving, forward U-turns instead of spinning or reversing. Worked in sim |
| 2026-11-16 | Sim, rejected: NavFn | straight-line paths ignore the robot's heading, so it spun on the spot |
| 2026-11-16 | Sim, rejected: Reeds-Shepp planner | near the goal it reversed 1.5 m instead of turning around |
| 2026-11-16 | Sim, rejected: MPPI Ackermann | no on-the-spot turns: stuck on tight bends near walls, overshot the goal heading, looped |
| 2026-11-16 | Sim measurement | Gazebo's skid-steer turns at ~40% of the commanded rate on the spot, ~60-73% in arcs |
| 2026-11-18 | Real: local inflation 3.0 / 0.70 -> 5.0 / 0.45 m | narrow corridors were costly from wall to wall |
| 2026-11-18 | Real: collision monitor off | its source was `/scan`, which the robot doesn't have, so it zeroed every command |
| 2026-11-18 | Real: NavFn -> Smac 2D | NavFn treated unknown space as free floor, so paths cut through unmapped areas |
| 2026-11-18 | Real: MPPI -> Rotation Shim + RPP | MPPI wandered off the path; RPP drives the drawn line and is lighter on the CPU |
| 2026-11-19 | Real: global obstacle layer removed | localisation offsets stamped walls into free corridors, so the planner found no path |
| 2026-11-19 | Real: option to use the depth camera for navigation | the lidar only sees from ~0.54 m up, so low obstacles were invisible |

## Undo

- Default recoveries: remove `BT_REWRITES` / the `RewrittenYaml` from the two launch files.
- B and C: the old values are in the table above and in the comments next to each change.
- E: set `yaw_goal_tolerance` back to 0.25 in both files, `go_to_place` back to `place['yaw']`, and turn
  `GoalAngleCritic` back on in sim.
