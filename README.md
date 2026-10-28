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

From the repository root, allow the container to open GUI applications and start it:

```bash
xhost +si:localuser:root
docker run --rm -it --name navigoat-dev --network=host -e DISPLAY -v "$PWD:/workspace" -v /tmp/.X11-unix:/tmp/.X11-unix:ro navigoat:dev bash
```

For Pete (object finding and scene description), build with `--build-arg PETE=true` and pass the camera and GPU. `--gpus all` needs the NVIDIA Container Toolkit. See [ros/src/autonomy/perception/README.md](ros/src/autonomy/perception/README.md), including the Jetson build.

```bash
docker run --rm -it --name navigoat-dev --network=host --gpus all --device /dev/video0 -e DISPLAY -v "$PWD:/workspace" -v /tmp/.X11-unix:/tmp/.X11-unix:ro navigoat:dev bash
```

### Open in VS Code

1. Open the repository folder in VS Code.
2. Run Dev Containers: Reopen in Container from the Command Palette. VS Code builds the image automatically if needed.

### Open the running container in a terminal

1. Run `docker ps` on the host and copy the container name.
2. Replace `CONTAINER_NAME` below to open a shell in the same container:

```bash
docker ps
docker exec -it <CONTAINER_NAME> bash
```

## [Simulation](ros/src/simulation/README.md)
