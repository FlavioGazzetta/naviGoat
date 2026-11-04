#!/bin/bash
# Start Whisper on the GPU (in the navigoat:people-dev container) and wait until it answers.
URL=http://127.0.0.1:8765/health
if curl -s -m 2 $URL | grep -q ok; then echo "Whisper GPU server already running"; exit 0; fi
docker rm -f whisper-gpu >/dev/null 2>&1
mkdir -p ~/.cache/whisper
docker run -d --rm --name whisper-gpu --runtime nvidia --gpus all --network host \
  -v $HOME/whisper-gpu:/pkgs -v $HOME/.cache/whisper:/root/.cache/whisper -v $HOME/naviGoat:/workspace:ro \
  -e PYTHONPATH=/pkgs/venv/lib/python3.12/site-packages \
  navigoat:people-dev python3 /workspace/ros/src/interaction/voice/whisper_server.py --model base.en >/dev/null
echo -n "Starting Whisper on the GPU"
for i in $(seq 1 60); do
  curl -s -m 2 $URL | grep -q ok && { echo " ready"; exit 0; }
  echo -n "."; sleep 2
done
echo " FAILED (see: docker logs whisper-gpu)"; exit 1