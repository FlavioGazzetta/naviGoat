#!/bin/bash
# Starts all of Pete on the Jetson with one command: the container (GPU, camera, sound), Ollama, and pete.launch.py.
#
#   ros/src/core/start_pete.sh                 camera, perception, middleman
#   ros/src/core/start_pete.sh voice:=true     also the headset, Whisper and the voice node
#
#   ros/src/core/start_pete.sh voice:=true show:=true     for a demo: Pete speaks through the laptop's speakers and
#                                              listens through the headset. Run voice/show_speaker.sh on the laptop
#                                              first. Another laptop address: SHOW_SPEAKER=<address> in front
#   PETE_IMAGE=navigoat:gps ros/src/core/start_pete.sh gps:=true     another image, here the one with the GPS layer
#
# Any arguments go to pete.launch.py. The Anthropic API key is read from ~/pete.env (see pete.env.example);
# without that file Pete uses the local model only. Ctrl+C stops everything.
REPO=$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)
VOICE_DIR=$REPO/ros/src/interaction/voice
RUNTIME_DIR=/run/user/$(id -u)

# show:=true: the laptop on the USB cable, unless SHOW_SPEAKER names another address
if [[ " $* " == *" show:=true "* ]]; then export SHOW_SPEAKER=${SHOW_SPEAKER:-192.168.55.100}; else unset SHOW_SPEAKER; fi
if [[ " $* " == *" voice:=true "* ]]; then
  "$VOICE_DIR/connect_airpods.sh" || exit 1
  "$VOICE_DIR/start_whisper_gpu.sh" || set -- "$@" whisper_url:=cpu
fi

ENV_FILE=()
if [ -f ~/pete.env ]; then ENV_FILE=(--env-file ~/pete.env); else echo "No ~/pete.env: Pete will use the local model only."; fi
mkdir -p ~/.nv-cache  # Ollama's compiled GPU code, kept between runs
mkdir -p ~/.local/share/navigoat  # the GPS layer's saved place look-ups, kept between runs (gps:=true)

docker rm -f navigoat-dev >/dev/null 2>&1
exec docker run --rm -it --name navigoat-dev --network=host --ipc=host --runtime nvidia --gpus all \
  --privileged -v /dev:/dev -v "$REPO:/workspace" -v ~/.nv-cache:/root/.nv -v ~/.local/share/navigoat:/root/.local/share/navigoat \
  -v "$RUNTIME_DIR/pulse:$RUNTIME_DIR/pulse" -e PULSE_SERVER=unix:$RUNTIME_DIR/pulse/native \
  "${ENV_FILE[@]}" "${PETE_IMAGE:-navigoat:dev}" bash -c "(ollama serve > /tmp/ollama.log 2>&1 &); source /opt/ros/jazzy/setup.bash && \
    ros2 launch /workspace/ros/src/core/launch/pete.launch.py $*"
