#!/bin/bash
# Start Whisper on the GPU (in the navigoat:dev container, which has PyTorch with CUDA) and wait until it answers.
# openai-whisper and the base.en weights are in the image (the VOICE option of docker/Dockerfile.dev): nothing is installed on the host.
# WHISPER_MODEL=tiny.en ~/start_whisper_gpu.sh uses another model; it is downloaded when the container first starts.
URL=http://127.0.0.1:8765/health
if curl -s -m 2 $URL | grep -q ok; then echo "Whisper GPU server already running"; exit 0; fi
docker rm -f whisper-gpu >/dev/null 2>&1
docker run -d --rm --name whisper-gpu --runtime nvidia --gpus all --network host \
  -v $HOME/naviGoat:/workspace:ro \
  ${WHISPER_IMAGE:-navigoat:dev} python3 /workspace/ros/src/interaction/voice/whisper_server.py --model ${WHISPER_MODEL:-base.en} >/dev/null
echo -n "Starting Whisper on the GPU"
for i in $(seq 1 90); do
  curl -s -m 2 $URL | grep -q ok && { echo " ready"; exit 0; }
  echo -n "."; sleep 2
done
echo " FAILED (see: docker logs whisper-gpu)"; exit 1
