# Hardware SLAM

## Setup

From the repository root:ros2 topic pub --once /cmd_motor_state p2os_msgs/msg/MotorState "{state: 1}"


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

With Femto Bolt camera:

```bash
ros2 launch /workspace/ros/src/autonomy/slam/launch/mapping.launch.py use_rgbd:=true start_camera:=true
```

For a lidar-only test without the base, pass only the lidar device into Docker and run:

```bash
ros2 launch /workspace/ros/src/autonomy/slam/launch/lidar_test.launch.py
```

Movement and drift testing can be done with `odometry:=icp`.



## garbage
```
docker exec -it navigoat-dev bash
```

to publish a goal position

```bash
ros2 topic pub --once /navigation/goto_point geometry_msgs/msg/PointStamped "{header: {frame_id: map}, point: {x: 0.5, y: 0.0}}"
```

to enable the motors 
```bash
ros2 topic pub --once /cmd_motor_state p2os_msgs/msg/MotorState "{state: 1}"
```


to run the mapping with the non sense parameters
```bash
ros2 launch /workspace/ros/src/autonomy/slam/launch/mapping.launch.py \
  database_path:=/workspace/ros/src/bringup/maps/<new_name>.db \
  odom_frame_id:=odom \
  rviz:=false
```



## TO move later (navigation stuff)
run basic nav2 with our parameters
```bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=/workspace/ros/src/autonomy/navigation/config/nav2_params.yaml \
  use_sim_time:=false use_composition:=False
```
run the custom nav node
```bash
ros2 launch /workspace/ros/src/autonomy/navigation/navigation_nodes/launch/navigation.launch.py \
  places_file:=/workspace/ros/src/bringup/maps/fresh2_places.yaml
```
