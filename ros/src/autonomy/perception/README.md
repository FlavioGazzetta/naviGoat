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
| `/object_mapping` | `geometry_msgs/PointStamped` | out | the object's last location in meters from the camera (x forward, y left, z up, frame `camera_link`), sent once at the end of each search |
| `/object_status` | `std_msgs/String` | out | at the end of each search: `found:<object>` (with its location on `/object_mapping`), or `not_found:<object>` |
| `/scene_description` | `std_msgs/String` | out | one or two spoken sentences, e.g. "You are in an office. Ahead are desks and chairs." |
| `/camera/color/image_raw`, `/camera/depth_registered/image_rect`, `/camera/color/camera_info` | `sensor_msgs` | in | from [the camera node](../../hardware/camera) or the Orbbec camera driver; depth aligned to colour, as Enid's people tracker also expects |

## How it works

- **Object detection: YOLO-World** (`yolov8s-worldv2.pt`), open vocabulary, so it can look for any object name. Each time the mode switches to `perception` it looks for the goal for 5 seconds (`search_time`), following the closest one, then sends where it was last seen once and stops. It only reports the goal object, but it is always given a list of 88 clearly different objects as well (mostly the ones YOLO was trained on): with only the goal word, anything similar (a cup when looking for a bottle) gets labelled as the goal. Near-synonyms are kept out of the list, because they split one object's boxes between them; instead about 80 synonyms and plurals are mapped to the list's words (sofa to couch, football to sports ball, chairs to chair). The list is turned into YOLO's form once, at launch, so searching for a list word starts instantly. Any other word is added in the background, which takes a second or two on the laptop and a few seconds on the Jetson, and the search starts once it's done. YOLO is also warmed up at launch, so the first search runs at full speed, and the goal must be seen 3 times before it counts as found. To find small or far objects without the cost of full resolution, every other frame YOLO looks at one quarter of the frame in full detail, taking the corners in turn. Each quarter reaches a random point just past the middle, so the borders between them move. Depth is only read once YOLO has a box around the goal, and then only inside the box. The position is the median depth over the middle of the box, turned into x and y with the camera's calibration (`camera_info`).
- **Scene description: Qwen3-VL 2B** (`qwen3-vl:2b-instruct`, about 1.9 GB), run locally through Ollama. It gets the whole view plus close-ups of the two busiest parts of a 3x2 grid over the frame (most edges and colour variety, so not bare walls or floor), cut from the full-size frame so they show more detail, all in one request. It starts with the specific kind of room, then what is ahead and the details from the close-ups with where they are, and never says a road is safe to cross. On the Jetson a description takes about 11 seconds (6 with the whole view alone). The `-instruct` tag matters: the plain `qwen3-vl:2b` is a thinking model and spends all its tokens reasoning before it answers.
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
| `search_time` | object_detection | `5.0` | seconds YOLO looks for the goal each time the mode switches to `perception`, before sending its last location; also a launch argument |
| `confidence` | object_detection | `0.25` | minimum YOLO confidence |
| `image_size` | object_detection | `640` | width YOLO looks at. A whole 1280 px frame at 640 loses half its detail, which is why `tiles` is on |
| `tiles` | object_detection | `true` | every other frame, look at one quarter of the frame in full detail instead of the whole frame: finds small or far objects at the speed of 640 (about 37 ms a frame on the Jetson) |
| `tile_reach` | object_detection | `0.15` | each quarter runs from a corner to a random point past the middle, at most this share of the frame height from it, so the borders between quarters move and an object on one isn't always split |
| `confirm_frames` | object_detection | `3` | sightings of the same object before `found`, so one-off flukes are ignored. A far object seen only in its quarter takes about 0.8 s |
| `track_gate` | object_detection | `0.5` | meters: it follows the closest goal object, and each frame only accepts a detection this near to where it was, so it doesn't jump between two chairs |
| `lost_after` | object_detection | `1.0` | seconds without seeing the followed object before it picks the closest one again |
| `frame_id` | object_detection | `camera_link` | frame of `/object_mapping` |
| `mode_name` | both | `perception` / `description` | the `/mode` value each node reacts to |
| `image_topic`, `depth_topic`, `camera_info_topic`, `mode_topic`, `goal_topic`, ... | both | see the topics table | topic names |
| `vlm_model` | scene_description | `qwen3-vl:2b-instruct` | Ollama model |

## Jetson notes

- The standard PyTorch ARM build prints a warning that it doesn't support the Orin (compute capability 8.7). It does run on the GPU. The NVIDIA Jetson wheels on `pypi.jetson-ai-lab.io/sbsa/cu130` are built for Thor and Spark only and don't work on the Orin.
- The first start is slow: Ollama compiles its GPU code for the Orin (about 2 minutes) while loading Qwen. Keep the same container running between runs, or mount a folder at `/root/.nv` so the compiled code is kept.
- Building the image on the Jetson takes about an hour. Run it inside `tmux` so a dropped SSH connection doesn't kill it.
