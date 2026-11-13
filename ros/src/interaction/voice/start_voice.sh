#!/bin/bash
# Connect the AirPods, route audio to them, keep the voice link open and run the listener.
# Copy to ~/start_voice.sh on the Jetson. Extra arguments go to listen_controller.py, e.g. --no-speak or --qwen.
#
# The headset's address is set below, now the G21 Pro earphones (or AIRPODS=aa:bb:... ~/start_voice.sh, e.g. the AirPods
# at E4:76:84:52:6E:61). The variable keeps its old name. The headset must be out of its case, and not
# connected to a laptop or phone.
AIRPODS=${AIRPODS:-24:DF:17:09:22:83}
ID=${AIRPODS//:/_}
SINK=bluez_output.$ID.1
SRC=bluez_input.$ID.0
CARD=bluez_card.$ID

pkill -f listen_controller.py 2>/dev/null
pkill -f 'paplay.*zero' 2>/dev/null

# 1. Connect. They go to sleep in the case or when idle, so try a few times.
connected() { bluetoothctl info $AIRPODS 2>/dev/null | grep -q 'Connected: yes'; }
for try in 1 2 3; do
  connected && break
  echo "Connecting the AirPods (try $try)..."
  bluetoothctl connect $AIRPODS 2>&1 | tail -1
  sleep 4
done
connected || { echo "Could not connect. Take the AirPods out of the case, put one in your ear, and run this again."; exit 1; }

# 2. The login screen (gdm) runs its own audio system and takes Bluetooth audio from this user. Set up once, this is
#    switched off for the gdm user only (the login screen still works, it just has no sound):
#      for u in pipewire.service pipewire.socket wireplumber.service pipewire-pulse.service pipewire-pulse.socket; do
#        sudo ln -sf /dev/null /var/lib/gdm3/.config/systemd/user/$u; done      (after mkdir -p, chown -R gdm:gdm)
#    Also needed once: copy /usr/share/wireplumber/bluetooth.lua.d/50-bluez-config.lua to
#    ~/.config/wireplumber/bluetooth.lua.d/ and set ["with-logind"] = false in it. This step only acts if gdm's audio is back.
if pgrep -u gdm -x wireplumber >/dev/null; then
  echo "The login screen's audio is running and blocks Bluetooth audio. Stopping the login screen (needs sudo)..."
  sudo systemctl stop gdm && systemctl --user restart wireplumber
  sleep 4
  bluetoothctl disconnect $AIRPODS >/dev/null 2>&1; sleep 2; bluetoothctl connect $AIRPODS >/dev/null 2>&1
fi

# 3. Wait for the speaker and the microphone to appear.
for i in $(seq 1 20); do
  pactl list short sinks | grep -q "$SINK" && pactl list short sources | grep -q "$SRC" && break
  sleep 1
done
pactl list short sinks | grep -q "$SINK" || { echo "The AirPods are connected but have no audio device. See: pactl list cards short"; exit 1; }

# The wideband (mSBC, 16 kHz) profile must be chosen by name: plain "headset-head-unit" falls back to 8 kHz CVSD.
pactl set-card-profile $CARD headset-head-unit-msbc 2>/dev/null || pactl set-card-profile $CARD headset-head-unit-cvsd 2>/dev/null
pactl set-default-sink $SINK
pactl set-default-source $SRC
pactl set-sink-volume $SINK 60%
pactl set-source-volume $SRC 100%

# 4. The microphone only carries audio while something plays to the AirPods, so play silence.
nohup paplay --raw --rate=16000 --channels=1 --format=s16le --device=$SINK /dev/zero >/dev/null 2>&1 &
KEEP=$!
trap 'kill $KEEP 2>/dev/null' EXIT

# 5. Whisper on the GPU if its container is set up, otherwise on the CPU.
WHISPER=""
if [ -x ~/start_whisper_gpu.sh ] && docker run --rm navigoat:dev python3 -c 'import whisper' >/dev/null 2>&1; then
  ~/start_whisper_gpu.sh && WHISPER="--whisper-url http://127.0.0.1:8765" || echo "GPU server did not start: using the CPU instead"
fi

# 6. Qwen (through Ollama) to understand needs like "I'm hungry". The rules decide everything else, so a stop never waits for it.
QWEN=""
if [ -x ~/start_ollama.sh ] && docker image inspect navigoat:dev >/dev/null 2>&1; then
  ~/start_ollama.sh && QWEN="--qwen" || echo "Ollama did not start: the rules alone will understand requests"
fi

RUN=~/report/$(date +%Y%m%d_%H%M%S)
cd ~/naviGoat/ros/src/interaction/voice
echo "Saving recordings and log to $RUN"
~/voice-env/bin/python -u listen_controller.py --device pulse --min-rms 0.006 --silence 0.5 --save-dir "$RUN" $WHISPER $QWEN "$@"
