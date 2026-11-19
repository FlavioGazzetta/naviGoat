# GPS navigation

This is the long-distance part of navigation: "GPS take me to the Science Museum". It plans a walking route on OpenStreetMap (preferring signalled crossings, never using steps), feeds Nav2 a stream of goals 20 to 30 m ahead along it, and stops before every road crossing until the user says go. It works out where the robot is by fusing the GPS with SLAM, copes with GPS dropouts (tunnels, tall buildings), and plans a new route when the robot leaves the old one.

It never steers and never broadcasts TF: Nav2, RTAB-Map and perception still handle obstacles, people and the last metres, exactly as indoors.

For the rest of the robot it is one node, **gps_bridge**, and the plain topics below. Everything else inside (the `guide/*` topics, services and actions) is private and can change.

## Topics

These are all the topics you need to connect anything to the GPS layer. Every name is a parameter in [guide_bringup/config/gps.yaml](guide_bringup/config/gps.yaml).

### What we subscribe to

| Topic | Type | Who sends it | What we do with it |
| --- | --- | --- | --- |
| `/middleman/mode` | `std_msgs/String` (JSON) | middleman | `gps` with a target → plan a route there and read it back. `stop`, or any other mode (location, object, follow, manual, help...) → stop our trip, if there is one. `idle` with nothing running does nothing |
| `/gps/go` | `std_msgs/Empty` | the handle button (to be wired) | the user's go: start the route, cross the road, carry on after a pause. Ignored when no GPS trip is running, so the button can stay wired in indoor modes |
| | | | With `auto_go: true` in [config/gps.yaml](guide_bringup/config/gps.yaml) (the setting now, as the robot has no go button) no go is needed: the trip sets off as soon as the route is planned and carries on at every road crossing without waiting, and "Press go ..." is not spoken. The robot then crosses roads on its own: set it to `false` once there is a button |
| `/gps/here` | `std_msgs/String` | middleman | "GPS, I'm at X": the name of the place the user says they are at. Used as the start of routes while there is no GPS position, and kept for later runs (see "Without a GPS position") |
| `/gps/fix` | `sensor_msgs/NavSatFix` | a GPS driver (none yet) | where the robot is. Sensor QoS (best effort), like any driver publishes |
| `/imu/data` | `sensor_msgs/Imu` | the IMU driver | the robot's heading (the orientation field must be filled in) |
| TF `map → base_link` | | RTAB-Map + the base driver | the robot's SLAM pose, fused with the GPS |
| `navigate_to_pose` | Nav2 action (we are the client) | Nav2 | we send it goals in the `map` frame and read how they went |

### What we publish

| Topic | Type | Who it's for | What's in it |
| --- | --- | --- | --- |
| `/pete/response` | `std_msgs/String` | voice | what Pete says, each line once: the planned route ("Science Museum. About 200 m walk, 2 road crossings. ... Press go to start."), "At crossing 1 of 2 (signalled, major road). Press go when it is safe to cross.", "Crossing now.", "GPS signal is weak. Continuing carefully.", a refused go ("Still planning, please wait.")... |
| `/middleman/done` | `std_msgs/String` | middleman | the target's name when we arrive. The middleman then says "Arrived." |
| `/middleman/failed` | `std_msgs/String` | middleman | the reason, ready to be spoken, when we give up ("I don't know a place called 'x', and OpenStreetMap has nothing by that name near here."). The middleman says it and goes back to idle |
| `/gps/crossing_wait` | `std_msgs/Bool` (latched) | handle / wristband (to be wired) | `true` while the robot waits at a crossing for the user's go |
| `/gps/status` | `std_msgs/String` | us, for debugging | `planning:x`, `ready:x`, `going:x`, `waiting:crossing`, `paused:x`, `gps_weak:x`, `position_lost:x`, `rerouting:x`, `arrived:x`, `cancelled:x`, `failed:<reason>` (same style as `/navigation/status`) |
| `/gps/waypoints` | `nav_msgs/Path` (latched) | navigation | the route as points every 5 m (`waypoint_spacing`), x and y in metres east and north of the origin in [config/gps.yaml](guide_bringup/config/gps.yaml) (frame `enu`), each with the direction of travel; the last point is the destination. Published when a trip sets off and again after a reroute; an empty path when the trip ends (stop, arrived, failed) or before one has started |
| `/gps/health` | `std_msgs/String` (JSON, latched) | anyone asking "why won't it go?" | what is running, see below |

We never repeat what the middleman already says: it says "GPS to science museum." when it starts us, "Arrived." after `/middleman/done`, and its own reply to "stop". A failure goes on `/middleman/failed` and the middleman speaks it, so it is said once.

### `/gps/health`

Published when something changes (latched, so `ros2 topic echo` shows the current state straight away):

```json
{"ready": false, "summary": "I can't navigate by GPS yet: there is no GPS signal.",
 "parts": {"mission":  {"ok": true,  "detail": "running"},
           "position": {"ok": false, "detail": "not localised yet"},
           "planner":  {"ok": true,  "detail": "running"},
           "nav2":     {"ok": false, "detail": "Nav2 is not running"},
           "gps":      {"ok": false, "detail": "no messages on /gps/fix"},
           "geocoder": {"ok": true,  "detail": "running"}}}
```

`ready` means a GPS trip could start now. While it is `false`, a GPS request is refused at once and `summary` is spoken (through `/middleman/failed`). The mission manager, the position, the route planner and Nav2 can block a start. The GPS alone does not, once there is a position (in a tunnel SLAM keeps it), and the geocoder never does (names in our list still work without it).

## How it all fits together

Seven nodes in `guide_nodes`, all started by one launch file. The algorithms live in `guide_core`, plain Python with no ROS, so they are unit-tested without starting anything.

1. **gps_bridge** is the only node you talk to. It turns `/middleman/mode` and `/gps/go` into our own action and service calls, and our mission's progress into the topics above. It also works out `/gps/health`.
2. **mission_manager** runs the trip: it finds the destination (our list first, then OpenStreetMap), asks for a route, reads it back, waits for the user's go, and reacts to everything that happens on the way (crossings, pauses, GPS loss, leaving the route).
3. **route_planner** plans on a cached OpenStreetMap walking graph of South Kensington (no internet needed). Signalled crossings are cheap, unsignalled ones of major roads very expensive, steps are forbidden.
4. **route_follower** keeps choosing the next goal 25 m ahead on the route, and holds the robot before every crossing until the user's go.
5. **goal_sender** sends those goals to Nav2 (`navigate_to_pose`), converted into RTAB-Map's `map` frame.
6. **localizer** fuses the GPS fix with the SLAM pose: smooth like SLAM, anchored to the world like GPS. It says how good the position is (GOOD, DEGRADED, LOST).
7. **geocoder** looks up names that are not in our list on OpenStreetMap's Nominatim, and keeps every answer in a cache on disk.

### The packages

| Package | What it is |
| --- | --- |
| [guide_core](guide_core) | the algorithms, no ROS. `pytest` runs its 935 tests |
| [guide_interfaces](guide_interfaces) | our private messages, services and the `NavigateTo` action |
| [guide_nodes](guide_nodes) | the seven nodes, plus two command-line tools (`fetch_osm_graph`, `plan_route`) |
| [guide_bringup](guide_bringup) | launch files, [config/gps.yaml](guide_bringup/config/gps.yaml), the destination list, the OSM graph, RViz config |
| [guide_sim](guide_sim) | development only: a simulated robot with GPS, IMU, odometry, fake RTAB-Map and fake Nav2, a fake Nominatim, and a run evaluator. Not built for the robot |

## Setup

The image needs the GPS layer's Python packages. Build it once with `GPS=true` (add the other build arguments you use, e.g. `PETE=true`):

```bash
docker build -f docker/Dockerfile.dev --build-arg GPS=true -t navigoat:dev .
```

Start the container as usual, with one more volume so the geocoder's cache survives the container (otherwise every new container asks Nominatim again, which its usage policy does not want):

```bash
mkdir -p ~/.local/share/navigoat
docker run --rm -it --name navigoat-dev --network=host -v "$PWD:/workspace" \
  -v ~/.local/share/navigoat:/root/.local/share/navigoat navigoat:dev bash
```

## Build

Inside the container, from `/workspace`:

```bash
colcon build --symlink-install --base-paths ros/src/autonomy/gps
source install/setup.bash
```

On the robot, leave the simulator out (it also drops the fake Nominatim):

```bash
colcon build --symlink-install --base-paths ros/src/autonomy/gps --packages-skip guide_sim
```

`colcon build` from the repository root builds us too. The Python files don't need a rebuild after a change (`--symlink-install`), the launch and config files do.

## Try it in the simulator

No robot needed: our simulator stands in for the robot, the GPS, RTAB-Map and Nav2, and a fake Nominatim answers a few names, so nothing goes online. Use the same container in every terminal (`docker exec -it navigoat-dev bash`, then `cd /workspace && source install/setup.bash`).

**Terminal 1**, start everything:

```bash
ros2 launch guide_bringup guide.launch.py sim:=true
```

Wait for `Health: Ready.` (about 5 s: the position has to settle).

**Terminal 2**, listen to Pete:

```bash
ros2 topic echo /pete/response --field data
```

**Terminal 3**, be the middleman and the handle:

```bash
ros2 topic pub -w 1 --once /middleman/mode std_msgs/msg/String "data: '{\"mode\": \"gps\", \"target\": \"science museum\"}'"
ros2 topic pub -w 1 --once /gps/go std_msgs/msg/Empty
```

The first line asks for the trip: terminal 2 says the route and "Press go to start.". The second is the go. The robot sets off, and stops before each of the two crossings ("At crossing 1 of 2 ... Press go when it is safe to cross."): send the same go again each time. After about 4 minutes it arrives, and `/middleman/done` gets `science museum` (`ros2 topic echo /middleman/done`). To see the state at any time: `ros2 topic echo /gps/status`.

Places that work in the simulator:

| Say | What happens |
| --- | --- |
| `south kensington station` (or `south ken`, `tube station`) | from our list: 650 m through the subway, 6 crossings, 2 with an island, about 11 minutes |
| `science museum`, `imperial college` | from the fake Nominatim: short trips |
| `museum` | the fake Nominatim knows five: the trip ends with the options read out ("Option 1: Natural History Museum ... Which one?"). Ask again with the full name |
| anything else | not found: the trip fails with the reason |

The simulated robot always starts at the beginning of the route to South Kensington station. Other things to try:

- **Stop.** While it drives, `ros2 topic pub -w 1 --once /middleman/mode std_msgs/msg/String "data: '{\"mode\": \"stop\", \"stop\": true}'"`. `/gps/status` says `cancelled:...` and the robot stops.
- **Change your mind.** Ask for another place while a trip runs: the old trip is cancelled and the new one planned.
- **Why won't it go?** `ros2 topic echo /gps/health --field data`. Start our launch without the simulator (`ros2 launch guide_bringup guide.launch.py`) and ask for a trip: it is refused with "there is no GPS signal".
- **Noisy GPS.** `ros2 launch guide_bringup mission.launch.py sensors:=urban_canyon` gives the GPS bias, jumps and dropouts of a city street, with tunnels where the route goes underground (our development launch, see below).

### Through the real middleman

Start the middleman too (`ros2 launch /workspace/ros/src/core/launch/middleman.launch.py`), and speak through the topic. The middleman only switches to GPS when you say "GPS":

```bash
ros2 topic pub -w 1 --once /speech/text std_msgs/msg/String "{data: 'gps take me to science museum'}"
ros2 topic pub -w 1 --once /speech/text std_msgs/msg/String "{data: 'stop'}"
```

Pete then says "GPS to science museum.", our read-back, and "Stopping." for the stop. A place that does not exist ends with the reason spoken and the middleman back to idle: that needs the `/middleman/failed` handling in `core/middleman_node.py`, which came with this branch.

## On the robot

```bash
ros2 launch guide_bringup guide.launch.py
```

(`sim:=false` is the default, so it can never start a second, fake Nav2 by mistake.) It expects:

- RTAB-Map: TF `map → odom` (and the base driver's `odom → base_link`).
- Nav2: the `navigate_to_pose` action. The goals are in the `map` frame.
- A GPS driver on `/gps/fix`, and an IMU with orientation on `/imu/data`.

If the names are different on the robot, change them in [config/gps.yaml](guide_bringup/config/gps.yaml), or pass your own copy with `params_file:=/path/to/gps.yaml`. Until a GPS driver exists, everything starts, `/gps/health` says "no GPS", and a GPS request is refused aloud.

### Settings

[config/gps.yaml](guide_bringup/config/gps.yaml):

| Parameter | Default | What it is |
| --- | --- | --- |
| `origin_lat`, `origin_lon` (all nodes) | 51.4988, -0.1749 | the centre of our metric frame. Routes and poses are metres east and north of it. It must be inside the OSM graph's area |
| `localizer.gps_topic`, `localizer.imu_topic` | `/gps/fix`, `/imu/data` | the drivers' topics |
| `localizer.map_frame`, `localizer.base_frame` | `map`, `base_link` | RTAB-Map's frame and the robot's |
| `goal_sender.action_name`, `goal_sender.nav_frame` | `navigate_to_pose`, `map` | Nav2's action and the frame it plans in |
| `gps_bridge.*_topic`, `gps_bridge.gps_mode` | the topics above, `gps` | the team's topic names and the middleman's mode name |
| `gps_bridge.gps_topic`, `gps_bridge.nav2_action` | `/gps/fix`, `navigate_to_pose` | for the health check only: keep them equal to the localizer's and goal sender's |

Arguments of `guide.launch.py` (`ros2 launch guide_bringup guide.launch.py --show-args`):

| Argument | Default | |
| --- | --- | --- |
| `sim` | `false` | `true` adds our simulator and the fake Nominatim |
| `params_file` | `config/gps.yaml` | the parameter file |
| `online_lookup` | `true` | `false`: never ask Nominatim (cached answers still work) |
| `graph_file` | `osm/south_ken_walk.graphml` | the cached OSM graph |
| `destinations_file` | `destinations/campus.yaml` | our list of named places |
| `geocode_cache_file` | `~/.local/share/navigoat/geocode_cache.json` (`..._sim.json` with `sim:=true`) | the saved Nominatim answers |
| `start_x`, `start_y`, `start_yaw_deg` | the start of the route to the station | `sim:=true` only: where the simulated robot starts |
| `rviz` | `false` | `true` also starts RViz with the graph and the route |

## Destinations and online lookup

A name is looked up in our list first, [destinations/campus.yaml](guide_bringup/destinations/campus.yaml). Each entry is the point to walk to (the step-free entrance, not the building's middle), with aliases. Names are matched without caring about case, accents, punctuation or a leading "the". The list is a demo: replace it with real entrances.

A name that is not in the list is looked up on OpenStreetMap's Nominatim, inside the graph's area only, by the **geocoder** node. Every answer is saved, so it also works offline afterwards. It follows Nominatim's [usage policy](https://operations.osmfoundation.org/policies/nominatim/): at most one request a second and 30 an hour, its own User-Agent, no retries, a 1-hour pause if the server refuses us, nothing sent but the name and the area. A place found online is always read back and waits for the user's go. Several robots, or many requests, would need our own Nominatim server or a paid one.

- Switches: `online_lookup:=false` at launch, or `ros2 param set /geocoder online_enabled false` while running, stops all requests (cached places still work). `use_geocoder: false` for `mission_manager` in your parameter file turns lookups off entirely.
- Cache: `~/.local/share/navigoat/geocode_cache.json` (in the container `/root/.local/share/navigoat`, hence the volume above). Places stay fresh 180 days, "no results" 7 days.
- Attribution: wherever results from Nominatim are shown, show "© OpenStreetMap contributors". The map data is under the ODbL.
- Speech: never look names up while the user is still speaking (no autocomplete: the policy forbids it).

## Still to wire up (team side)

- The handle button → `/gps/go`.
- `/gps/crossing_wait` → a vibration on `/handle/vibration` or `/wristband/vibration` (`[side, magnitude]`, `std_msgs/UInt8MultiArray`), so the user feels that the robot waits at a crossing.
- A GPS driver → `/gps/fix`.
- There is no spoken "go": the middleman has no phrase for it. Whether a spoken go at a road crossing is safe enough is a team decision.

The turn vibrations need nothing: feedback_node reads Nav2's `/plan`, which follows our goals. And places_node cancels its own goal when the mode becomes `gps`, so the two never drive Nav2 at the same time.

## Without a GPS position

The position routes start from is, in this order:

1. **The real position**: the GPS fused with SLAM. With a GPS fix but no SLAM pose, the fix itself.
2. **The last place we knew**, kept in `~/.local/share/navigoat/gps_start.json` (`assumed_file`): the last GPS fix, or the place the user stated since. "Pete, GPS, I'm at the Abdus Salam Library" states a place: the middleman tidies the name with Claude and publishes it on `/gps/here`, we look it up like a destination (our list, then OpenStreetMap), and Pete answers "Okay, starting from Abdus Salam Library." A stated place only counts while the GPS has nothing: while fixes arrive it is refused ("The GPS already knows where we are."), and the next fix replaces it.
3. **The Electrical Engineering building at Imperial College** (`assumed_lat`, `assumed_lon` in [config/gps.yaml](guide_bringup/config/gps.yaml)), when nothing was ever stored, 10 s after start.

For 2 and 3 the position is reported as weak ("GPS signal is weak. Continuing carefully."), and nothing can be sent to Nav2, because where the SLAM map lies in the world is unknown: the route only goes out on `/gps/waypoints`. The robot does not move along the route in this state: it stays "at" that place.

With `require_nav2: false` (the setting now) a trip also starts when Nav2 is not running; `/gps/health` still reports that Nav2 is missing.

## Known limits

- Tested with our simulator only. The real Nav2 and RTAB-Map are next (Nav2 failures, kerbs, islands).
- If RTAB-Map stops, its last transform stays in TF, so the localizer thinks the robot stands still and does not report the position as lost. To be fixed when we test with the real RTAB-Map.
- With GPS only (no SLAM correction yet), the robot stops 1.5 m or more before a kerb, as far as its position uncertainty demands. The last metres to the kerb need perception.
- Named places from Nominatim are the middle of a building, often 30 to 50 m from a path; the read-back says so ("is a large area: the route goes to its nearest path").

## Developing

- Unit tests: `cd ros/src/autonomy/gps/guide_core && python3 -m pytest -q test/` (935 tests, no ROS needed). All lint tests: `colcon test --base-paths ros/src/autonomy/gps && colcon test-result`.
- `mission.launch.py` is our development launch: the simulator with sensor presets (`sensors:=perfect|map_offset|open|urban_canyon`, `noise_seed:=N`), and a run evaluator that scores a trip against the simulator's ground truth (`evaluate:=true auto_go:=true` presses go by itself). It also accepts our own `guide/navigate_to` action directly.
- A new area: `ros2 run guide_nodes fetch_osm_graph --lat 51.4988 --lon -0.1749 --dist 1500 --out guide_bringup/osm/<name>.graphml` (needs internet; it asks Overpass, not Nominatim), then point `graph_file` and the origin at it.
- One route by hand, with the planner running: `ros2 run guide_nodes plan_route <start_lat> <start_lon> <goal_lat> <goal_lon>`.
