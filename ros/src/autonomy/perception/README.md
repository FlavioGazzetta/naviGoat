# Perception: Pete

Pete is the object finding and scene description part of the robot. Everything is in [classification.py](classification.py).

Pete takes text requests. Speech to text is handled separately, so whatever it produces can be sent straight in. Pete replies with short spoken-style sentences.

| Request | What Pete does |
| --- | --- |
| `pete what's in front of me` | Describes the scene and says where the nearest objects are |
| `pete where is the remote` / `pete find cup` | Looks for the object and says where it is, e.g. "The cup is at 1 o'clock, slightly to your right, about 1 meter away." |
| `pete bring me to the chair` | Drives towards the chair (only when driving is enabled) |
| `pete stop` | Stops |
| anything else, e.g. `pete I'd like a coffee` | Works out what is wanted (here: find cup) or answers it as a question about the image |

## How it works

- **Object detection: YOLO-World** (`yolov8s-worldv2.pt`). It is open vocabulary, so it can search for any object name rather than a fixed list. Common names and aliases (sofa → couch, fridge → refrigerator, ...) are mapped to a shared vocabulary.
- **Scene understanding: Qwen3-VL 2B** (`qwen3-vl:2b-instruct`), run locally through Ollama. It is about 1.9 GB, so it fits comfortably on the 8 GB Jetson Orin Nano alongside YOLO. The `-instruct` tag matters: the plain `qwen3-vl:2b` is a thinking model and spends all its tokens reasoning before it answers.
- **Understanding requests.** Simple requests are parsed directly. Anything else goes to the model with a constrained JSON output, so it can only return a valid intent and an object from the vocabulary. That is how "I'd like a coffee" becomes "find cup".
- **Positions and distances come from the detector, not the language model.** The model was making distances up, so it now only describes the scene and Pete adds the positions itself:
  - direction from the box centre and the camera's horizontal field of view (default 60°, `--hfov`), given as a clock position and left/right
  - distance from the box height and a typical real height for the object (e.g. a chair is about 0.9 m)
  - if the box is cut off at the top or bottom of the frame, Pete says it can't tell how far away it is instead of guessing
- **Short answers.** Replies are capped at two sentences and only answer what was asked.
- **Guiding** turns the robot towards the target, turns on the spot to search if it is not in view, and stops once it is close. It publishes `geometry_msgs/Twist` on `/cmd_vel`, which `p2os_driver` listens to. The driver stops the wheels if commands stop arriving, and Pete also stops if the camera goes quiet.

> Guiding does **not** avoid obstacles yet. Keep driving disabled until a sonar/lidar stop is added.

## Running it

Build the container with Pete's extras. They are opt-in, so the normal image stays small for everyone else:

```bash
# laptop
docker build -f docker/Dockerfile.dev --build-arg PETE=true -t navigoat:dev .
# Jetson (the osrf desktop image has no ARM build)
docker build -f docker/Dockerfile.dev --build-arg BASE_IMAGE=ros:jazzy-perception-noble --build-arg PETE=true -t navigoat:dev .
```

This adds YOLO-World, PyTorch, Ollama and the Qwen model to the image (several GB). Ollama starts automatically when you open a shell in the container.

Start the container with the GPU and the camera:

```bash
# laptop (needs the NVIDIA Container Toolkit)
xhost +si:localuser:root
docker run --rm -it --name navigoat-dev --network=host --gpus all --device /dev/video0 -e DISPLAY -v "$PWD:/workspace" -v /tmp/.X11-unix:/tmp/.X11-unix:ro navigoat:dev bash
# Jetson over SSH (connect with ssh -X to see the camera window)
docker run --rm -it --name navigoat-dev --network=host --runtime nvidia --gpus all --device /dev/video0 -e DISPLAY -v $HOME/.Xauthority:/root/.Xauthority:ro -v "$PWD:/workspace" navigoat:dev bash
```

### Without ROS (testing)

```bash
python3 ros/src/autonomy/perception/classification.py --terminal --show
```

Type requests and Pete prints the replies. `--show` opens the camera with detection boxes. Quit with `q` in the window or Ctrl+D.

### With ROS

```bash
python3 ros/src/autonomy/perception/classification.py --ros-args -p use_webcam:=true -p enable_drive:=true
ros2 topic pub --once /pete/command std_msgs/msg/String "{data: 'where is the chair'}"
ros2 topic echo /pete/response
```

| Parameter | Default | |
| --- | --- | --- |
| `use_webcam` | `false` | read the camera directly instead of subscribing to `image_topic` |
| `image_topic` | `/camera/image_raw` | |
| `enable_drive` | `false` | allow Pete to publish drive commands |
| `cmd_vel_topic` | `/cmd_vel` | use `/model/pioneer3at/cmd_vel` in Gazebo |
| `hfov_deg` | `60.0` | camera horizontal field of view |
| `require_wake_word` | `false` | only respond to requests starting with "pete" |

## Jetson notes

- Tested on a Jetson Orin Nano (JetPack 7.2, Ubuntu 24.04) with a USB webcam on `/dev/video0`. The Pi Camera Module 3 isn't supported on this JetPack.
- The standard PyTorch ARM build prints a warning that it doesn't support the Orin (compute capability 8.7). It does run on the GPU, so the warning is hidden in the script. The NVIDIA Jetson wheels on `pypi.jetson-ai-lab.io/sbsa/cu130` are built for Thor and Spark only and don't work on the Orin.
- The first reply is slow while the Qwen model loads onto the GPU. After that it stays loaded.
- Building the image on the Jetson takes about an hour. Run it inside `tmux` so a dropped SSH connection doesn't kill it.

## Next steps

- Obstacle stop using the sonars (`sonar/range_N`) or the lidar before driving is enabled
- Real distances from sonar/lidar instead of estimating from box size
- Check that detection is fast enough on the Jetson for smooth guiding
