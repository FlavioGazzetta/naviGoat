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
