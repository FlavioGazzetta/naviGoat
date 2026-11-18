#!/bin/bash
# Starts all of Pete on the Jetson with one command: the container (GPU, camera, sound), Ollama, and pete.launch.py.
#
#   ros/src/core/start_pete.sh                 camera, perception, middleman
#   ros/src/core/start_pete.sh voice:=true     also the headset, Whisper and the voice node
#
# Any arguments go to pete.launch.py. The Anthropic API key is read from ~/pete.env (see pete.env.example);
# without that file Pete uses the local model only. Ctrl+C stops everything.
REPO=$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)
VOICE_DIR=$REPO/ros/src/interaction/voice
RUNTIME_DIR=/run/user/$(id -u)

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
  "${ENV_FILE[@]}" navigoat:dev bash -c "(ollama serve > /tmp/ollama.log 2>&1 &); source /opt/ros/jazzy/setup.bash && \
    ros2 launch /workspace/ros/src/core/launch/pete.launch.py $*"
