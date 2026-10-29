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
colcon build --base-paths ros/src/hardware/base/p2os2 ros/src/hardware/lidar/unilidar_sdk/unitree_lidar_ros2 --packages-up-to p2os_bringup p2os_urdf unitree_lidar_ros2 --symlink-install
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

For a lidar-only test without the base, pass only the lidar device into Docker and run:
```bash
ros2 launch /workspace/ros/src/autonomy/slam/launch/lidar_test.launch.py
```
Movement and drift testing can be done with `odometry:=icp`.