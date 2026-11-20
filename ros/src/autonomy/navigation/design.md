# Navigation oral: design decisions breakdown

Here's the full breakdown, organised by the five oral questions, so you can map it straight onto slides. Two caveats: I've **assumed your role is navigation and social navigation** (that's what your commits are). And I've noted where a part belongs to a teammate (SLAM and hardware: Max; people tracker: Enid; handle and wristband: interaction), so you present it as integration, not as your own work.

---

## 1. Your role (< 1 min)

**Navigation and social navigation for Pete, a robot that guides a person** (who may not see well) holding a handle:
- **Where to go:** saved places, objects found by perception, points; integrated with the middleman (voice commands)
- **How to get there:** Nav2 on the Pioneer 3-AT, on RTAB-Map's map
- **Around people:** a social costmap layer (personal space, groups, walkers)
- **Telling the person:** turn cues to the handle, nearby-people cues to the wristband
- **Bringing it onto the real robot:** one launch for everything, operating instructions, tuning on the Jetson

---

## The navigation system, end to end (for the system-diagram slide)

The whole chain, in the order data flows through it: from the sensors, to a map and a position, to a goal, to a path,
to motor commands, and back to the person as vibrations. One command starts all of it on the robot
(`navigation_robot.launch.py`).

### 1. Sensing: what the robot knows about itself and its surroundings

- **Wheel odometry.** The Pioneer's driver (`p2os_driver`) counts wheel turns and publishes where the robot thinks it
  is, relative to where it started: the `/odom` topic and the transform `odom → base_link`, 10 times a second. It is
  smooth and never jumps, but it drifts, especially when turning, because a skid-steer robot slides its wheels sideways
  to turn.
- **The lidar.** The Unitree L1, 0.50 m above the robot's base, sends a 3D point cloud (`/unilidar/cloud`) about 10
  times a second, roughly 2,000 points per scan. Mounted pointing upward, it sees from about 0.54 m up to the ceiling:
  walls, desks, people's bodies, but not the floor or anything low.
- **The camera** (only with `social:=true`). The Femto Bolt colour and depth camera, used to find people.

### 2. Cleaning the lidar data

- **Deskewing** (`lidar_deskewing`). A scan takes about 0.1 s, and the robot moves during it, so a scan taken while
  turning comes out smeared. Deskewing uses the wheel odometry to work out where the robot was at the moment each point
  was measured, and moves every point to where it belongs. Output: `/unilidar/cloud/deskewed`.
- **Moving-object removal** (optional, `filter:=object`). Compares consecutive scans and removes what moves (people
  walking by), so they don't leave trails in the map. Output: `/unilidar/cloud_static`. It costs about a third of a CPU
  core, so the default is `filter:=deskew`, and `object` is used only when mapping a busy room.

### 3. Mapping and localisation (RTAB-Map)

- RTAB-Map takes the cleaned cloud and the wheel odometry. Every ~10 cm of travel it stores a **node**: the robot's
  pose plus what the lidar saw there. When it recognises a place it has already been (a **loop closure**), it corrects
  the whole chain of nodes, which removes the drift the wheels built up.
- From the nodes' 3D points it builds the **2D occupancy grid** `/map` that Nav2 uses: points between 0.1 m and 1.5 m
  high become obstacles, the cells a beam passes through on its way become free, and everything else stays unknown.
- It publishes the correction as the transform `map → odom`. Chained with the wheels' `odom → base_link`, that gives the
  robot's position on the map: smooth from moment to moment (wheels), and drift-free over time (lidar matching).
- Everything is saved in one **database file** (`lab.db`). In **mapping** mode it keeps adding to the map. In
  **localisation** mode the map is frozen and RTAB-Map only works out where the robot is on it. The robot then has to
  start where mapping started, or be told its position with RViz's 2D Pose Estimate.

### 4. People (social navigation)

- The camera images go to a **YOLO server** that detects people, and **Enid's tracker** follows each person over time.
  It publishes `/social_perception/tracks` about 9 times a second: each person's id and position, relative to the camera.
- **people_node** moves each person from the camera's frame onto the map, through the transforms (camera → robot →
  odom → map). It estimates how fast each person walks by fitting a line through their last second of positions, adds
  test people (clicked in RViz or from a file), and publishes everyone on `/people` 10 times a second. Someone the
  camera loses is kept for a moment, because the robot may only have turned away: a walker for 0.5 s, someone standing
  for 3 s. And if `/people` itself stops for 1 s, the social layer clears everyone, so a crashed node can't leave ghost
  people behind.

### 5. Deciding where to go (places_node)

- Goals arrive as a **voice command** through the middleman ("Pete, take me to the kitchen" →
  `/middleman/mode` `{"mode": "location", "target": "kitchen"}`), as an **object** found by perception (`object` mode,
  then the object's position on `/object_mapping`), or directly as a place name or a point (`/navigation/goto`,
  `/navigation/goto_point`).
- **Places** are robot positions saved by name (`/navigation/save_place`) in a file next to the map (`lab_places.yaml`),
  because they only make sense on that map.
- For an **object**, it computes a stopping point 0.8 m short of it, so the robot doesn't drive into the chair.
- Goals are **positions only**: Nav2's goal checker accepts any heading, so the robot arrives in whatever direction is
  convenient (a voice command can't ask for one).
- It sends the goal to Nav2's `navigate_to_pose` action, and reports back: `/navigation/status` (`going:`, `arrived:`,
  `failed:`), `/middleman/done` when it arrives (the middleman then says "Arrived"), and `/pete/response` for what Pete
  says out loud ("I couldn't get to the kitchen").

### 6. Planning and driving (Nav2)

- **Two costmaps**, grids where each cell has a cost from 0 (free) to 254 (obstacle):
  - the **global costmap**, on the map frame, updated once a second: RTAB-Map's `/map`, the **social layer**, and an
    inflation layer that makes cells near walls expensive, so paths keep to the middle of corridors;
  - the **local costmap**, a 3 × 3 m window that moves with the robot, 5 times a second: the **live lidar** (marks
    obstacles up to 2.0 m, the ceiling hits clear cells), the **social layer** and inflation. This is where obstacles
    that aren't on the map (a chair, a person not seen by the camera) appear.
- **The social layer** (my C++ plugin, in both costmaps) reads `/people` and paints, for each person: the person
  (lethal), a personal space that fades out over 1 m and reaches further in front of someone walking, an expensive strip
  between two people standing closer than 2.2 m, and an expensive strip along where each walker will be in the next 2 s.
  Only the person blocks; everything else is just costly, so the robot avoids it if it can and goes through if it must.
- **The planner** (Smac 2D) finds the cheapest path through the global costmap from the robot to the goal, and
  publishes it on `/plan`.
- **The controller** follows it 20 times a second: the rotation shim first turns the robot on the spot to face the path,
  then regulated pure pursuit drives along it at walking pace (0.3 m/s), slowing in tight turns, near obstacles and at
  the goal, and never reversing. If something blocks the way, it stops and the planner replans.
- The speed commands go through the velocity smoother (limits acceleration) and the collision monitor (in the chain, but
  disabled for now) to `/cmd_vel`, and the base driver turns them into wheel speeds. If no command arrives for 0.2 s,
  the base stops by itself.

### 7. Telling the person (feedback_node)

- **Turns → handle.** It reads the path on `/plan`. When a turn sharper than 25° comes within 2 m, it sends
  `[side, strength]` to `/handle/vibration`; the sharper the turn, the stronger (255 at 90° or more).
- **People → wristband.** It reads `/people`. While the robot moves, the closest person within 1.2 m gives
  `[side, strength]` on `/wristband/vibration`; the closer, the stronger (255 at 0.5 m).
- Both are sent 10 times a second. When the side changes, a 0 goes to the old side first, because the devices keep
  vibrating until they get a 0.

### One sentence version

The wheels and the lidar give RTAB-Map what it needs to build a map and say where the robot is on it; the camera gives
people_node the people; places_node turns a voice command or an object into a goal; Nav2 plans the cheapest path
through a map where people and their personal space have a cost, and drives it at walking pace without ever reversing;
and feedback_node turns the path and the people into vibrations for the person holding the handle.

---

## 2. Technical challenges (1–2 min)

1. **Social, not just collision-free.** Avoiding people isn't enough. Don't cut between two people talking, give walkers room in front, but still go through when there's no other way.
2. **The guided person is part of the system.** They're behind the robot, so no reversing; the path must be predictable (drive the drawn line); and they need **overhead clearance**, not only the robot.
3. **Perception in another frame.** People come from a camera tracker, relative to the camera, at about 9 Hz; they have to be placed on the map via TF, with velocities.
4. **Weak sensing on the real robot.** The Unitree L1 points upward: **0 of 21,000 points below 0.54 m** (measured). No floor, no low obstacles. Lidar-only localisation can't find the robot from scratch. The wheel odometry slips in turns (skid-steer).
5. **Compute.** The Jetson Orin Nano runs SLAM, Nav2, people detection and speech at once. A CPU at 100% caused late lidar data (1–4 s), dropped scans, and Nav2 shutting itself down.
6. **From simulation to reality.** Settings that worked in Gazebo (sim lidar, perfect odometry) didn't carry over: different frames, a different sensor, hardware glitches (USB ports swapping, the base not responding on low battery).

---

## 3. Alternatives evaluated (1–2 min)

| Decision | Chosen | Alternatives considered | Why |
|---|---|---|---|
| **Social navigation** | **a costmap layer plugin** (C++, inside Nav2) | social force model in the controller; MPC with predicted people; learning-based; existing social-costmap packages | works with **any** planner and controller; rules can be changed live; easy to test and explain. **Con:** a snapshot of one moment, so walker prediction is approximate (an MPC would do it properly, and that's the planned extension) |
| **Planner** | **Smac 2D** | NavFn (used first) | NavFn treated **unknown space as free**, so paths cut through unmapped areas. Smac counts unknown as expensive, keeps to corridor centres, and smooths |
| **Controller** | **Regulated Pure Pursuit + Rotation Shim** | MPPI (used first, in sim and on the robot) | MPPI treats the path as one cost among many, so it **wandered off the drawn path**, and it's **CPU-heavy** (2000 samples × 20 Hz). RPP **follows the line**, turns on the spot first, never reverses, slows near obstacles. **Con:** it doesn't swerve; it stops and the planner replans |
| **SLAM** (Max's part; you integrated it) | **RTAB-Map** | slam_toolbox, Cartographer | 3D lidar **and** RGB-D, a saved database, loop closure, localisation mode. **Con:** heavy; lidar-only can't relocalise globally |
| **Odometry** | wheels, read from TF (`odom_frame_id`) | lidar ICP odometry; wheels + IMU EKF | wheels are smooth and never lose track, which Nav2 needs. Planned next: an **EKF with the lidar's IMU gyro**, for rotation |
| **Lidar mount** | (upward; changing) | upside down; forward 90°; **45° tilt** | upside down: no overhangs, which a **guided person walks into**, and the body blocks the back. Forward 90°: weaker SLAM. **45°:** floor + low obstacles in front, overhangs, and good SLAM |
| **Global costmap** | **map + people only** | + live lidar (used first) | with small localisation errors, live walls got **stamped into free corridors**, so no path was found. Live obstacles go in the **local** costmap, relative to the robot |
| **People in the stack** | `social:=true` = camera + YOLO + tracker only | the whole Pete stack | saves Qwen and the middleman's CPU and RAM |
| **Haptics** | turns → handle, people → wristband | both to one device | two publishers on one device would switch each other's sides off |

### RTAB-Map settings I changed, and what was considered

RTAB-Map itself and its launch files are Max's. These are the settings I changed or added to get mapping working on
the real robot, from the moment it produced **no map at all** to clean maps Nav2 can plan on.

**The starting problem.** Mapping ran without errors, but the map stayed empty (`publishMaps: grid map is empty`). I
measured what the lidar actually sends: 10 scans, about 21,000 points, as heights above `base_link`. The lowest was
**0.54 m**, none below 0.5 m, and the median was the ceiling. The startup log showed
`Update RTAB-Map parameter "Grid/MaxObstacleHeight"="0.5" from database`: an old setting stored **inside the map
file**, which threw away every point above 0.5 m, so every point. The lidar-test database didn't have it, which is why
the lidar test showed a map and mapping didn't.

| Setting | Value | Why | Considered |
|---|---|---|---|
| **Grid settings in the YAML** (`rtabmap_hardware.yaml`) | all set explicitly | RTAB-Map applies settings saved in a database file when it opens it; settings given as ROS parameters override them. So an old file (`lab.db`, `temp_map.db`) can't bring back a bad value | always starting a fresh database file: works once, but any old map reopened later would break again |
| `Grid/MaxObstacleHeight` | **1.5 m** | points between the ground band and 1.5 m become obstacles: desks, people, walls. The ceiling (about 2.1–2.6 m above `base_link`) is dropped, otherwise the whole room would be one obstacle | **0.5** (the old value): drops every point of this lidar. **2.0**: covers a guided person's head height (overhangs they could walk into), and would be the value if the lidar is remounted to see forward; the ceiling would still be dropped |
| `Grid/MinGroundHeight` / `MaxGroundHeight` | −0.3 / **0.1 m** | points up to 10 cm above the floor count as floor, anything higher as obstacle | with the lidar pointing up, **these do nothing**: no floor point ever arrives. They start to matter after the remount (45° or upside down), when the floor is seen |
| `Grid/MaxGroundAngle`, `Grid/NormalK` | 45°, 20 | how steep a surface can be and still count as floor, and how many neighbours are used to work out each point's surface direction | defaults; same reason, they only matter once the floor is seen |
| `Grid/RayTracing` | **true** | marks every cell a beam passes through as **free**. This lidar never sees the floor, so ray tracing is the **only** way the map gets free space at all | off (the default): the map would be obstacles in a sea of unknown, and Nav2 couldn't plan |
| `Grid/NoiseFilteringRadius` | 0.1 m → **0 (off)** | the filter deletes points with fewer than 3 neighbours within 0.1 m. One L1 scan is only about 2,000 points, so far-away walls are sparse, and the filter **deleted most distant obstacles** | keeping it at 0.1 (it removes single noisy points, but here it removed real walls); a larger radius (keeps more, but also keeps more noise) |
| `odom_frame_id: odom` (in `navigation_robot.launch.py`) | read odometry **from TF** | without it, RTAB-Map subscribes to `/odom` and has to pair each wheel message (10 Hz) with a lidar scan (about 10 Hz) by timestamp. It kept failing (`Did not receive data`, `cannot get the corresponding TF`). With TF it asks "where was the robot at this scan's exact time?" and TF interpolates between wheel updates. Same wheel data, received differently; the first run that built a map used this | the topic (the default); lidar ICP odometry (`icp_odometry`), as in the lidar test |
| `wait_for_transform` of the deskewing (in `mapping.launch.py`) | 0.01 → **0.2 s** | deskewing needs the robot's pose at the start and end of each scan, but the wheels only publish at 10 Hz. Waiting only 10 ms, it failed on **every** scan (about 2,800 in one run), so scans taken while turning stayed smeared | no deskewing (`filter:=none`): smeared walls when turning |

**Related decisions:**
- **Moving-object filter (`filter:=object`).** Removes people walking past from what RTAB-Map maps, so they don't leave
  trails. It costs about a third of a CPU core, on a Jetson already at its limit. The default became `filter:=deskew`
  (Max changed it, after the CPU measurements); `object` is for mapping a busy room. While **navigating**
  (localisation) the map isn't changed, so it doesn't matter there. The costmaps always use the live cloud, so they see
  people either way.
- **Odometry source.** RTAB-Map gets wheel-only odometry, and a skid-steer robot's heading drifts in turns. Considered:
  (1) an **EKF** (`robot_localization`) fusing the wheels with the lidar's built-in IMU gyro, which measures rotation
  directly: cheap on the CPU and never loses track. That's the planned next step. (2) **Lidar scan matching**
  (`icp_odometry`) with that EKF as its starting guess. Postponed until the lidar is remounted, because it would be tuned
  on mostly ceiling points now.
- **CPU.** Considered turning off RTAB-Map's 3D map and OctoMap (`Grid/3D: false`), since Nav2 only needs the 2D `/map`.
  Not done yet; the bigger savings were the power mode and the filter default.
- **Localisation mode** (Max's settings): `Mem/IncrementalMemory: false` freezes the map, and `initial_pose 0 0 0`
  assumes the robot starts where mapping started. With an upward lidar, RTAB-Map can't reliably recognise where it is
  from scratch, so the robot must start on the same spot, or get a 2D Pose Estimate in RViz.
- **Where it ended up.** What fixed mapping, together: a database file without the old setting, the grid settings back
  in the YAML (so they override any database), odometry from TF, and the base's motors actually switched on (they start
  off, so the robot hadn't moved and RTAB-Map had nothing to add).

---

## 4. What's done; positives and negatives (2–3 min)

**Built:**
- **people_node:** tracker JSON → map (TF), velocities, test people from YAML or RViz clicks, a 1 s timeout against ghosts
- **social_layer (C++):** the person (lethal), personal space stretched by walking speed, the **group strip** (< 2.2 m apart), **walker prediction** (2 s), ignores the handler (< 0.6 m), live-tunable
- **places_node:** save, go to and forget places (stored next to each map), goals by point, **object mode** (stop 0.8 m before it), middleman and voice integration, status topics
- **feedback_node:** turn and proximity vibrations in the device format, sending a zero before switching side
- **Simulation:** Gazebo world, mapping then localisation, fake people, and a **real-people-on-simulated-map** mode (separate ROS domains)
- **Real robot:** `navigation_robot.launch.py`: **one command**, flags for `mode` / `teleop` / `social` / `filter`; ports auto-detected; motors switched on automatically
- **Tuning on hardware:** footprint (real P3-AT size), inflation, the ceiling-clearing source (stale obstacles), the Smac/RPP switch, no live lidar in the global costmap, the Nav2 heartbeat fix, MAXN_SUPER power mode, Wi-Fi power saving off
- **Documentation:** a copy-paste run README plus a topics reference

**Positives:**
- modular: the social rules work with any Nav2 planner and controller
- validated in simulation, with numbers
- the same code runs in simulation and on the robot
- operable by teammates (one launch, README)
- safety-minded: no reversing, the base stops by itself without commands

**Negatives (own them, examiners respect it):**
- the costmap is a **snapshot**, not a time-aware prediction
- the **upward lidar can't see low obstacles**, until the remount
- **localisation needs an initial pose**, with lidar only
- the **collision monitor is off** (no `/scan`; the robot's own points would trigger it)
- the **Nav2 heartbeat check is disabled** (a workaround for CPU stalls)
- the **CPU is close to its limit** with everything running
- maps get **noisy** when people are near the robot during mapping
- the vibration chain is **only validated at topic level**, not with the devices live

---

## 5. Demonstrating it works (2–3 min)

**Numbers from simulation** (already in `README_archive.md`):
| Two people apart | Group rule off | Group rule on |
|---|---|---|
| 2.0 m | goes **between** (7.0 m path) | goes **around** (8.2 m path) |
| 2.2 m | between | around |
| ≥ 2.3 m | between | between (not a group any more) |

Walker prediction: the path crosses **2.0 m** in front of a walker with prediction, against **1.7 m** without.

**On the real robot (verified this week):**
- **Social chain live:** the tracker at about 9 Hz → `/people` with **the same 2 people** in map coordinates → the group strip visible in the costmap
- **Mapping and localisation** work (clean maps; localisation with a pose estimate)
- **Nav2 goals succeed** after the fixes: the path is drawn and followed
- **Before / after evidence:**
  - load **8.9 → about 1.2** (MAXN + lighter settings)
  - Wi-Fi **20% loss → 0%** (power saving off / cable)
  - the global costmap **blocked → plannable** (no live lidar)
  - **0 lidar points below 0.54 m** (the reason for the remount)

**Demo plan** (videos are safer than live):
1. **Simulation:** two fake people 2.0 m apart, group rule on vs off (robot goes around vs through). Then a walker.
2. **Real robot:** a goal down a corridor (the path drawn and followed), with RViz screen-recorded alongside the robot.
3. **Real people:** a person in front of the camera → a blue cylinder in RViz → the path bends around them.
4. **Places:** save "kitchen", drive away, `goto kitchen`, `arrived:kitchen`.
5. **Haptics:** an echo of `/handle/vibration` while turning, if the devices work, the actual vibration.

**Suggested slides (about 10):** role → challenges → system diagram (tracker → people_node → social_layer → Nav2 → feedback) → the social rules (pictures) → sim results table → the alternatives table → sim-to-real problems and fixes (lidar heights, CPU, costmap stamping, controller swap) → demo videos → limits and next steps (lidar remount, EKF, collision monitor, MPC).

I can turn this into a slide deck, or a doc you can edit. Which would help more?
