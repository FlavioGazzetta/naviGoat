# Perception: Pete

Pete's perception is two ROS 2 nodes. The middle man (not part of perception) tells them what to do through `/mode` and `/goal`.

| Node | Runs when | Does |
| --- | --- | --- |
| [object_detection.py](object_detection.py) | `/mode` is `perception` | Looks for the `/goal` object with YOLO-World and publishes where it is for navigation, or that it is not there |
| [scene_description.py](scene_description.py) | `/mode` switches to `description` | Describes the camera image with the Qwen3-VL vision model, for the speech interface to say |

## Topics

| Topic | Type | Direction | |
| --- | --- | --- | --- |
| `/mode` | `std_msgs/String` | in | `perception` starts object detection, `description` starts a scene description; anything else and YOLO stays idle |
| `/goal` | `std_msgs/String` | in | the object to look for, e.g. `chair`. A new goal, or the same one sent again, starts a fresh search |
| `/object_mapping` | `geometry_msgs/PointStamped` | out | the object's position in meters from the camera (x forward, y left, z up, frame `camera_link`), sent every frame it is seen |
| `/object_status` | `std_msgs/String` | out | `found:<object>` once it is found, or `not_found:<object>` if it is not seen within `search_timeout` seconds |
| `/scene_description` | `std_msgs/String` | out | one or two spoken sentences, e.g. "You are in an office. Ahead are desks and chairs." |
| `/camera/color/image_raw`, `/camera/depth_registered/image_rect`, `/camera/color/camera_info` | `sensor_msgs` | in | from [the camera node](../../hardware/camera) or the Orbbec camera driver; depth aligned to colour, as Enid's people tracker also expects |

## How it works

- **Object detection: YOLO-World** (`yolov8s-worldv2.pt`), open vocabulary, so it can look for any object name. It only runs in perception mode and only reports the goal object, but it is always given a list of about 30 common objects as well: with only the goal word, anything similar (a cup when looking for a bottle) gets labelled as the goal. Synonyms and plurals are mapped to the list's words (sofa to couch, chairs to chair), and the goal must be seen in 3 frames in a row before it counts as found. The position is the median depth over the middle of the object's box, turned into x and y with the camera's calibration (`camera_info`).
- **Scene description: Qwen3-VL 2B** (`qwen3-vl:2b-instruct`, about 1.9 GB), run locally through Ollama. It starts with the kind of place, then what matters for moving safely, and never says a road is safe to cross. The `-instruct` tag matters: the plain `qwen3-vl:2b` is a thinking model and spends all its tokens reasoning before it answers.
- **Start-up order.** The launch file starts object detection only after Qwen has loaded. On the Jetson the GPU shares memory with everything else, and if YOLO loads first, Ollama puts most of Qwen on the much slower CPU.

## Setup

Build the container with Pete's extras (YOLO-World, PyTorch, Ollama and the Qwen model, several GB). They are opt-in, so the normal image stays small for everyone else:

```bash
docker build -f docker/Dockerfile.dev --build-arg PETE=true -t navigoat:dev .
```

Start it with the GPU (Ollama starts automatically in the container's shells):

```bash
# laptop (needs the NVIDIA Container Toolkit)
docker run --rm -it --name navigoat-dev --network=host --gpus all -v "$PWD:/workspace" navigoat:dev bash
# Jetson
docker run --rm -it --name navigoat-dev --network=host --runtime nvidia --gpus all -v "$PWD:/workspace" navigoat:dev bash
```

The camera is run by the Orbbec ROS 2 driver with depth aligned to colour, e.g. `ros2 launch orbbec_camera femto_bolt.launch.py depth_registration:=true`. It can run in another container on the same machine: with `--network=host` the topics are shared.

Without the driver, [the camera node](../../hardware/camera) publishes the same topics from the Femto Bolt using the Orbbec SDK in our image: add `camera:=true` to the launch command, and start the container with `--privileged -v /dev:/dev` so it can reach the USB camera. Run only one of the two.

## Run

```bash
ros2 launch /workspace/ros/src/autonomy/perception/launch/perception.launch.py
```

Without the middle man, act as it by hand:

```bash
ros2 topic pub --once /goal std_msgs/msg/String "{data: chair}"
ros2 topic pub --once /mode std_msgs/msg/String "{data: perception}"
ros2 topic echo /object_mapping
ros2 topic echo /object_status
ros2 topic pub --once /mode std_msgs/msg/String "{data: description}"
ros2 topic echo /scene_description
```

To watch the camera from a laptop on the same network, with `camera:=true`, run `python3 ros/src/hardware/camera/view_camera.py` there (see [hardware/camera](../../hardware/camera)).

| Parameter | Node | Default | |
| --- | --- | --- | --- |
| `camera` | launch argument | `false` | `true` starts [the camera node](../../hardware/camera) for the Femto Bolt |
| `search_timeout` | object_detection | `10.0` | seconds before `not_found`; also a launch argument |
| `confidence` | object_detection | `0.25` | minimum YOLO confidence |
| `confirm_frames` | object_detection | `3` | frames in a row the goal must be seen before `found`, so one-frame flukes are ignored |
| `frame_id` | object_detection | `camera_link` | frame of `/object_mapping` |
| `mode_name` | both | `perception` / `description` | the `/mode` value each node reacts to |
| `image_topic`, `depth_topic`, `camera_info_topic`, `mode_topic`, `goal_topic`, ... | both | see the topics table | topic names |
| `vlm_model` | scene_description | `qwen3-vl:2b-instruct` | Ollama model |

## Jetson notes

- The standard PyTorch ARM build prints a warning that it doesn't support the Orin (compute capability 8.7). It does run on the GPU. The NVIDIA Jetson wheels on `pypi.jetson-ai-lab.io/sbsa/cu130` are built for Thor and Spark only and don't work on the Orin.
- The first start is slow: Ollama compiles its GPU code for the Orin (about 2 minutes) while loading Qwen. Keep the same container running between runs, or mount a folder at `/root/.nv` so the compiled code is kept.
- Building the image on the Jetson takes about an hour. Run it inside `tmux` so a dropped SSH connection doesn't kill it.
