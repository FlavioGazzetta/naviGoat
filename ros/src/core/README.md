# Middleman

Turns what the user said into one mode and one target, tells perception and navigation, and says the reply. The rules decide first, so a "stop" never waits for anything. Qwen (through Ollama) is only asked when the rules do not understand, to match a need ("I'm hungry") to a known place.

| File | What it does |
| --- | --- |
| [middleman.py](middleman.py) | The decisions: modes, rules, Qwen. No ROS. `python3 middleman.py` lets you type requests |
| [middleman_node.py](middleman_node.py) | The ROS 2 node around it |
| [catalog.py](catalog.py) | The places, the objects and their short names, and what each place is for |
| [launch/middleman.launch.py](launch/middleman.launch.py) | Starts the node, with a `use_qwen` argument |

## Topics

All are `std_msgs/String`. Every name is a ROS parameter (the first column), so a launch file can change it.

| Parameter | Default | Direction | |
| --- | --- | --- | --- |
| `speech_topic` | `/speech/text` | in | what the user said, from [speech to text](../interaction/voice) |
| `response_topic` | `/pete/response` | out | what Pete says: the middleman's replies, and the scene description. The voice node speaks it |
| `mode_topic` | `/mode` | out | what perception listens to: `perception` (search for the goal), `description` (describe the view), `idle`, or the mode's name |
| `goal_topic` | `/goal` | out | the object to look for. Sent before `/mode` = `perception` |
| `status_topic` | `/object_status` | in | perception's search result, `found:<object>` or `not_found:<object>`. A failed search is spoken and the mode goes back to idle |
| `description_topic` | `/scene_description` | in | perception's description of the view. It is passed on to the response topic |
| `done_topic` | `/middleman/done` | in | the robot reached the goal of a location, object or GPS request |
| `middleman_mode_topic` | `/middleman/mode` | out | JSON `{"mode", "target", "intent", "stop", "request_id"}` for the mode being started (follow, location, object, gps, manual, help, idle) |
| `following_status_topic` | `/person_following/status` | in | JSON `{"request_id", "state", "event", "target_id"}` from following; matching terminal failures return the mode to idle and are spoken |
| `save_place_topic` | `/navigation/save_place` | out | the name from "remember this place as the study". Navigation's places_node saves the robot's position under it, and "take me to the study" works afterwards |
| `places_topic` | `/navigation/places` | in | JSON list of the names navigation has saved (latched, so earlier runs' places arrive at start). They are understood like the catalog's places, so "take me to the balcony" works after a restart, and the language model is told about them too (by name only, they have no purpose line like the catalog's) |
| `heard_topic` | `/middleman/heard` | out | a request went to perception and there is no spoken reply: play a beep |

The other parameter is `use_qwen` (default `false`): let Qwen match a need to a known place. It needs Ollama with `qwen3-vl:2b-instruct` running on `localhost:11434`. The scene description uses the same model, so they share one loaded copy, and both ask with `num_ctx` 4096 (`OLLAMA_OPTIONS` in `middleman.py` must stay equal to the one in `scene_description.py`, or Ollama reloads the model every time the two take turns).

Following commands now include a unique `request_id`. The following runner
echoes it on `/person_following/status`. A matching terminal event in state
`IDLE` (`target_lost`, `input_stale`, `input_fault`, `unsupported_target`,
`command_invalid`, `disabled`, `transform_unavailable`, `navigation_unavailable`,
`navigation_failed`, or `navigation_cancel_failed`) ends that follow request and publishes its
explanation on `/pete/response`. Progress, old-session messages and duplicates
are ignored; following failures do not use the arrival handler. Both nodes need
the updated code. See [the following integration test](../autonomy/person_following/README.md#lias-voicemiddleman-connection)
for typed-speech testing without a robot.

## Running it

Inside the container, from `/workspace`:

```bash
ros2 launch /workspace/ros/src/core/launch/middleman.launch.py use_qwen:=true
```

By hand, acting as the voice and as perception:

```bash
ros2 topic pub --once /speech/text std_msgs/msg/String "{data: 'pete take me to the chair'}"
ros2 topic echo /pete/response
ros2 topic echo /mode
ros2 topic pub --once /object_status std_msgs/msg/String "{data: 'not_found:chair'}"
```

The same package can be built with `colcon build --packages-select middleman --symlink-install` (it has a `package.xml`), so `ros2 run middleman middleman_node` works too.

### The language model: Claude, with a local fallback

[llm.py](llm.py) is shared by the middleman and the scene description. With `ANTHROPIC_API_KEY` set and a connection, both ask Claude (`claude-haiku-5-5`, low effort, 4-second limit). Without a key, offline, or after a failure, both ask the same small local vision model. Offline, only the first request waits (at most 4 s, the address lookup included); after that the local model answers at once, and a background check every 10 s switches back to Claude when the connection returns. With the local model, the middleman only matches needs ("I'm hungry" to Pret): it does not swap a name it doesn't know for a place, because the local model is always sure of a wrong one. Both use the same small local vision model through Ollama, LFM2.5-VL 1.6B: about 0.9 GB loaded, where Qwen3-VL 2B took 2 GB and grew to nearly 4.

The key goes in `~/pete.env` on the Jetson, one line, never committed (template: [pete.env.example](pete.env.example)):

```bash
echo "ANTHROPIC_API_KEY=sk-ant-..." > ~/pete.env
```

The image needs the `anthropic` package and the local model: rebuild once with `docker build -f docker/Dockerfile.dev --build-arg PETE=true --build-arg VOICE=true -t navigoat:dev .` (only the new last layer is built).

### All of Pete at once

On the Jetson, [start_pete.sh](start_pete.sh) does everything: the headset and Whisper (with `voice:=true`), the container with the GPU, camera, sound and `~/pete.env`, Ollama, and the launch file below:

```bash
~/naviGoat/ros/src/core/start_pete.sh voice:=true
```


[launch/pete.launch.py](launch/pete.launch.py) starts the camera, perception (scene description, then object detection once Qwen has loaded) and the middleman with one command. The container needs the camera (`--privileged -v /dev:/dev`) and the GPU, as in [perception's README](../autonomy/perception/README.md):

```bash
ros2 launch /workspace/ros/src/core/launch/pete.launch.py
```

| Argument | Default | |
| --- | --- | --- |
| `camera` | `true` | `false` when the Orbbec ROS 2 driver runs the camera instead |
| `search_time` | `5.0` | seconds object detection looks for a goal |
| `use_qwen` | `true` | as above. Here it is on by default: Ollama runs in the same container for scene description, so the middleman shares that loaded Qwen |
| `people` | `false` | `true` also starts the people tracker (`femto_person_tracker`, publishes `/social_perception/tracks`) and its dashboard, from [social_perception](../autonomy/perception/social_perception). They use the camera Pete already runs, and the tracker uses perception's [YOLO server](../autonomy/perception/yolo_server.py) instead of loading its own model. Needs the `SOCIAL=true` image |
| `people_max_age` | `1.0` | seconds the tracker retains a missed track; use `2.0` with person following to match its two-second loss window |
| `people_port` | `8766` | the dashboard's port. Not 8765: that is Whisper's |
| `voice` | `false` | `true` also starts the voice node. It needs the `VOICE=true` image and the host's sound socket (see `start_voice_ros.sh`); the voice launch file starts the middleman itself, so it is never started twice |
| `whisper_url`, `listen`, `device` | as in the voice launch file | passed on to the voice node |

## What it does with a request

- **Stop and help** are decided by rules, first, whatever else is running.
- **"Follow me"**, **"take me to the bathroom"** (a stored place), **"take me to the chair"** (a stored object), **"GPS take me to ..."** (only when the user says GPS), **manual mode**.
- **"What's in front of me"** goes to perception as `/mode` = `description`. Perception only describes the whole view, so other camera questions get "Sorry, I can only describe what is in front of me."
- **A need** ("I'm hungry", "I need to study") is matched to a place by Qwen, only if it is at least 70% sure. Qwen can only answer with a name from the lists, so it cannot start follow or manual mode.
- **An unknown place** gets "I don't know where the X is." A single unfamiliar word is never guessed by meaning, because it is nearly always a mis-heard name.

## What is tested and what is not

Tested: the rules, with unit tests (they cover the rules, the matching of mis-heard names and the Qwen fallback with a stand-in); the node with real ROS 2 in the project container, driven by hand over the topics above, including a renamed topic; the Qwen understanding on the Jetson against the real model (about 14 of 15 test phrases right, 0.6 to 1.0 s each); and the scene description request against Flavio's real `scene_description.py` with a stand-in camera.

Not tested:
- The object search against Flavio's real `object_detection.py` (YOLO). It needs the Jetson's memory sorted out first.
- Person following consumes `/middleman/mode` and reports failures on
  `/person_following/status`. It has an offline goal demo and a ROS runner
  (preview by default; Nav2 actions require `--execute-nav2`). Other modes'
  arrival reporting uses `/middleman/done`.
- `found:<object>` is not spoken, and the object's position (`/object_mapping`) goes nowhere.
- The object and place lists in `catalog.py` are a copy of the ones in `autonomy/perception/object_detection.py` (88 words and 84 aliases). They have to be kept equal by hand until one imports the other.
