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

### Open in VS Code

1. Open the repository folder in VS Code.
2. Run **Dev Containers: Reopen in Container** from the Command Palette. VS Code builds the image automatically if needed.

### Open the running container in a terminal

1. Run `docker ps` on the host and copy the container name.
2. Replace `CONTAINER_NAME` below to open a shell in the same container:

```bash
docker ps
docker exec -it <CONTAINER_NAME> bash
```


## Simulation
### Gazebo

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
