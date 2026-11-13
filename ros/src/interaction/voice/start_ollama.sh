#!/bin/bash
# Start Ollama with Qwen on the GPU (in the navigoat:dev container, which already has the qwen3-vl:2b-instruct model
# in /opt/models/ollama) and wait until it answers. The middleman uses it to understand needs like "I'm hungry", and
# Flavio's scene description uses the same loaded model. Copy to ~/start_ollama.sh on the Jetson.
# The first start loads the model into memory, which takes a minute or two.
URL=http://127.0.0.1:11434/api/tags
if curl -s -m 2 $URL >/dev/null; then echo "Ollama already running"; exit 0; fi
docker rm -f ollama-qwen >/dev/null 2>&1
docker run -d --name ollama-qwen --runtime nvidia --gpus all --network host \
  -e OLLAMA_MODELS=/opt/models/ollama -e OLLAMA_KEEP_ALIVE=-1 \
  ${OLLAMA_IMAGE:-navigoat:dev} ollama serve >/dev/null
echo -n "Starting Ollama"
for i in $(seq 1 45); do
  curl -s -m 2 $URL >/dev/null && { echo " ready"; exit 0; }
  echo -n "."; sleep 2
done
echo " FAILED (see: docker logs ollama-qwen)"; exit 1
