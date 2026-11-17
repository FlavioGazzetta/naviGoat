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
docker run --rm -it --name navigoat-dev --network=host --device=/dev/ttyUSB0:/dev/unilidar --device=/dev/ttyUSB1:/dev/pioneer -e DISPLAY -v "$PWD:/workspace" -v /tmp/.X11-unix:/tmp/.X11-unix:ro navigoat:dev bash
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
ros2 launch bringup launch.py
```

## [Simulation](ros/src/simulation/README.md)

For mapping and navigation with the Pioneer base on a saved map, see [hardware navigation](ros/src/autonomy/navigation/README.md#running-on-the-pioneer-base).
