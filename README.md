# naviGoat


## Setup

Update submodules to the versions recorded by the repository:

```bash
git submodule update --init --recursive
```

### Build the image

From the repository root:

```bash
docker build -f docker/Dockerfile.dev -t navigoat:dev .
```

### Start the container from a terminal

From the repository root, allow the container to open GUI applications and start it. The lidar is mapped from `/dev/ttyUSB0` to `/dev/unilidar` and the Pioneer from `/dev/ttyUSB1` to `/dev/pioneer`. Replace the host USB paths as needed:

```bash
xhost +si:localuser:root
docker run --rm -it --name navigoat-dev --network=host --security-opt apparmor=unconfined -v /run/dbus:/run/dbus --device=/dev/ttyACM0:/dev/handle --device=/dev/ttyUSB0:/dev/unilidar --device=/dev/ttyUSB1:/dev/pioneer -e DISPLAY -v "$PWD:/workspace" -v /tmp/.X11-unix:/tmp/.X11-unix:ro navigoat:dev bash
```

For Pete (object finding and scene description), build with `--build-arg PETE=true` and pass the camera and GPU. `--gpus all` needs the NVIDIA Container Toolkit. See [ros/src/autonomy/perception/README.md](ros/src/autonomy/perception/README.md), including the Jetson build.

```bash
docker run --rm -it --name navigoat-dev --network=host --device=/dev/ttyUSB0:/dev/unilidar --device=/dev/ttyUSB1:/dev/pioneer --gpus all --device /dev/video0 -e DISPLAY -v "$PWD:/workspace" -v /tmp/.X11-unix:/tmp/.X11-unix:ro navigoat:dev bash
```

### Open in VS Code

1. Open the repository folder in VS Code.
2. Run Dev Containers: Reopen in Container from the Command Palette. VS Code builds the image automatically if needed.

### Open the running container in a terminal

Run on the host to open a shell in the running container:

```bash
docker exec -it navigoat-dev bash
```

### Build and run bringup

Inside the container, from `/workspace`:

```bash
colcon build --packages-ignore unitree_lidar_ros unitree_lidar_sdk --symlink-install
source install/setup.bash
ros2 launch bringup launch.py database_path:=/workspace/ros/src/bringup/maps/lab.db
```

This starts the whole robot: the base (motors turned on once it's connected), the lidar, RTAB-Map, Nav2, the
navigation nodes, the handle and the wristband. `database_path` is the map: a new file makes a new map, an existing one
continues it. On a finished map, add `mode:=localisation` to navigate on it without changing it. The places are saved
next to the map (`lab.db` → `lab_places.yaml`).

To run only the navigation part (no handle or wristband), with the same arguments:

```bash
ros2 launch /workspace/ros/src/autonomy/navigation/navigation_nodes/launch/navigation_robot.launch.py \
  database_path:=/workspace/ros/src/bringup/maps/lab.db
```

Useful arguments:

| Argument | Default | What it's for |
| --- | --- | --- |
| `mode` | `mapping` | `localisation` to navigate on a finished map |
| `pioneer_port`, `lidar_port` | `/dev/pioneer`, `/dev/unilidar` | swap them (`pioneer_port:=/dev/unilidar lidar_port:=/dev/pioneer`) if the base or lidar don't connect because the container's USB numbers are swapped (see the [SLAM README](ros/src/autonomy/slam/README.md)) |
| `filter` | `object` | `object` keeps moving people out of the map. The Nav2 costmaps still see them |
| `enable_motors` | `true` | `false` to leave the motors off |
| `rviz` | `false` | RTAB-Map's RViz on the Jetson. Heavy, so better run RViz on the laptop |

Only one of these at a time: two launches start two base drivers on the same serial port, and neither works.

## [Simulation](ros/src/simulation/README.md)

## [GPS navigation](ros/src/autonomy/gps/README.md)

Long walks outdoors: "GPS take me to ..." plans a route on OpenStreetMap and feeds Nav2 goals along it, stopping at every road crossing until the user's go. Build the image with `--build-arg GPS=true`.
