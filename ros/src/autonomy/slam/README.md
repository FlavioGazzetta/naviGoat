# Hardware SLAM

## Setup

From the repository root:

```bash
git submodule update --init --recursive
```

```bash
docker run --rm -it --name navigoat-dev --network=host --device=/dev/ttyUSB0:/dev/unilidar --device=/dev/ttyUSB1:/dev/pioneer -e DISPLAY -v "$PWD:/workspace" -v /tmp/.X11-unix:/tmp/.X11-unix:ro navigoat:dev bash
```

Inside the Jazzy container, build from `/workspace`:

```bash
colcon build --base-paths ros/src --packages-up-to bringup p2os_urdf unitree_lidar_ros2 navigation_resources --symlink-install
source install/setup.bash
```

### Setup Details

Follow the [container setup](../../../../README.md#setup). Identify serial ports:

```bash
ls -l /dev/ttyUSB*
``` 

Add these flags when creating the docker container, replacing USB paths as needed:

```bash
--device=/dev/ttyUSB0:/dev/unilidar --device=/dev/ttyUSB1:/dev/pioneer
```

## Run

```bash
ros2 launch /workspace/ros/src/autonomy/slam/launch/mapping.launch.py
```

The base launch enables the motors automatically after the driver subscriber is available. Pass `enable_motors:=false` to keep them disabled at startup. With `start_base:=false`, this launch neither starts the driver nor enables its motors.

With Femto Bolt camera:

```bash
ros2 launch /workspace/ros/src/autonomy/slam/launch/mapping.launch.py use_rgbd:=true start_camera:=true
```

For mapping without wheel odometry, pass the lidar device into Docker and run:

```bash
mkdir -p /workspace/maps
ros2 launch navigation_resources lidar_test.launch.py odometry:=icp database_path:=/workspace/maps/lidar-icp.db
```

This starts the lidar, lidar-based ICP odometry, RTAB-Map and RViz without the base driver. Stop other lidar and mapping launches first. The Pioneer driver must also be stopped because it publishes a competing `odom` to `base_link` transform. Move the lidar slowly to build the map, then stop the launch with Ctrl+C to save it. Without `odometry:=icp`, this test launch assumes a stationary lidar.
