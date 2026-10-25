# Simulation
## Gazebo

Inside the Dev Container, build the Pioneer package once:

```bash
colcon build --packages-up-to p2os_urdf --symlink-install
source install/setup.bash
```

Start Gazebo and spawn the Pioneer 3-AT:

```bash
ros2 launch /workspace/src/simulation/launch/gazebo.launch.py
```

In another container terminal, start keyboard control:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/model/pioneer3at/cmd_vel
```

Keep the keyboard terminal focused. Press `i` to drive forwards, `,` to drive backwards, `j` or `l` to turn and `k` to stop.


## SLAM
### 3D mapping

To start Gazebo, RTAB-Map and RViz together instead of the Gazebo command above:

```bash
ros2 launch /workspace/src/simulation/launch/simulation.launch.py
```

Drive the Pioneer using the keyboard command above in another terminal, then return near the starting point to test loop closure. RTAB-Map saves the map to `src/simulation/maps/rtabmap.db`.

### Localisation

After stopping the mapping launch and saving the map, restart Gazebo, RTAB-Map and RViz in localisation mode:

```bash
ros2 launch /workspace/src/simulation/launch/simulation.launch.py mode:=localisation
```

Drive through the mapped area using the keyboard command above. RTAB-Map uses the saved map without adding new map points.

Check the estimated position in another container terminal:

```bash
ros2 topic echo /localization_pose --once --field pose.pose.position
```