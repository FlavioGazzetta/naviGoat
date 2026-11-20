#!/bin/bash
# Starts all of Pete on the Jetson with one command: the container (GPU, camera, sound), Ollama, and pete.launch.py.
#
#   ros/src/core/start_pete.sh                 camera, perception, middleman
#   ros/src/core/start_pete.sh voice:=true     also the headset, Whisper and the voice node
#
#   ros/src/core/start_pete.sh voice:=true show:=true     for a demo: Pete speaks through the laptop's speakers and
#                                              listens through the headset. Run voice/show_speaker.sh on the laptop
#                                              first. Another laptop address: SHOW_SPEAKER=<address> in front
#   ros/src/core/start_pete.sh voice:=true local_llm:=false     without the local model: Ollama is not started, Claude
#                                              answers alone (about 0.9 GB less memory; nothing answers offline)
#   HEADSET=none ros/src/core/start_pete.sh voice:=true     on a laptop: its own microphone and speakers, no Bluetooth
#                                              headset, and Whisper on the CPU
#   PETE_LAUNCH=core/launch/everything.launch.py ros/src/core/start_pete.sh voice:=true database_path:=... mode:=localisation
#                                              another launch file (path from ros/src), here Pete with the base, SLAM and
#                                              Nav2. The built workspace (/workspace/install) is then sourced as well
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
  if [ "$HEADSET" = none ]; then
    # no Bluetooth headset: this computer's own microphone and speakers, and Whisper on the CPU inside the container
    set -- "$@" whisper_url:=cpu
  else
    "$VOICE_DIR/connect_airpods.sh" || exit 1
    "$VOICE_DIR/start_whisper_gpu.sh" || set -- "$@" whisper_url:=cpu
  fi
fi

# local_llm:=false: Ollama is not started at all
OLLAMA='(ollama serve > /tmp/ollama.log 2>&1 &);'
if [[ " $* " == *" local_llm:=false "* ]]; then OLLAMA=; fi

# PETE_LAUNCH: another launch file than Pete's own; those need the workspace's packages (the base driver, Nav2 ...)
LAUNCH=${PETE_LAUNCH:-core/launch/pete.launch.py}
WORKSPACE=
if [ -n "$PETE_LAUNCH" ]; then WORKSPACE='source /workspace/install/setup.bash &&'; fi

ENV_FILE=()
if [ -f ~/pete.env ]; then ENV_FILE=(--env-file ~/pete.env); else echo "No ~/pete.env: Pete will use the local model only."; [ -z "$OLLAMA" ] && echo "local_llm:=false as well: no language model at all."; fi
mkdir -p ~/.nv-cache  # Ollama's compiled GPU code, kept between runs
mkdir -p ~/.local/share/navigoat  # the GPS layer's saved place look-ups, kept between runs (gps:=true)

docker rm -f navigoat-dev >/dev/null 2>&1
exec docker run --rm -it --name navigoat-dev --network=host --ipc=host --runtime nvidia --gpus all \
  --privileged -v /dev:/dev -v "$REPO:/workspace" -v ~/.nv-cache:/root/.nv -v ~/.local/share/navigoat:/root/.local/share/navigoat \
  -v "$RUNTIME_DIR/pulse:$RUNTIME_DIR/pulse" -e PULSE_SERVER=unix:$RUNTIME_DIR/pulse/native \
  "${ENV_FILE[@]}" "${PETE_IMAGE:-navigoat:dev}" bash -c "$OLLAMA source /opt/ros/jazzy/setup.bash && $WORKSPACE \
    ros2 launch /workspace/ros/src/$LAUNCH $*"
