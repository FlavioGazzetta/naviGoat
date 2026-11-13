#!/bin/bash
# The ROS version of start_voice.sh: the voice nodes (voice.py and middleman_node.py) run in a container next to the
# other ROS nodes, and talk to them through topics. The AirPods stay connected on the host (connect_airpods.sh), and the
# container uses their speaker and microphone through the host's sound socket.
#
#   needs: ~/connect_airpods.sh, ~/start_whisper_gpu.sh, ~/start_ollama.sh (copy them from this folder) and the image:
#          docker build -f docker/Dockerfile.dev --build-arg PETE=true --build-arg VOICE=true -t navigoat:dev .   (VOICE adds the audio libraries)
#
# Ctrl+C stops the voice nodes. The Whisper and Ollama containers keep running. Extra arguments go to the launch file,
# e.g. ~/start_voice_ros.sh use_qwen:=false
RUNTIME_DIR=/run/user/$(id -u)

~/connect_airpods.sh || exit 1

WHISPER_URL=cpu
~/start_whisper_gpu.sh && WHISPER_URL=http://127.0.0.1:8765 || echo "GPU server did not start: Whisper will run on the CPU"
QWEN=false
~/start_ollama.sh && QWEN=true || echo "Ollama did not start: the rules alone will understand requests"

docker rm -f voice-nodes >/dev/null 2>&1
# The launch file runs the scripts from /workspace by path, like perception.launch.py, so there is nothing to build.
docker run -d --name voice-nodes --network host \
  -v $HOME/naviGoat:/workspace \
  -v $RUNTIME_DIR/pulse:$RUNTIME_DIR/pulse -e PULSE_SERVER=unix:$RUNTIME_DIR/pulse/native \
  ${VOICE_IMAGE:-navigoat:dev} bash -c "source /opt/ros/jazzy/setup.bash && \
    ros2 launch /workspace/ros/src/interaction/voice/launch/voice.launch.py use_qwen:=$QWEN whisper_url:=$WHISPER_URL $*" >/dev/null

trap 'docker rm -f voice-nodes >/dev/null 2>&1; ~/connect_airpods.sh --stop' EXIT
echo "Voice nodes starting. Say 'Pete, ...'. Ctrl+C to stop."
docker logs -f voice-nodes
