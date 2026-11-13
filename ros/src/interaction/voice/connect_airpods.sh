#!/bin/bash
# The host side of the voice system: connect the AirPods, route the sound to them and keep the microphone alive.
# Bluetooth and the sound system belong to the host, so this does not run in a container. The voice nodes in the
# container then use the AirPods through the mounted sound socket (start_voice_ros.sh).
# Copy to ~/connect_airpods.sh on the Jetson. "~/connect_airpods.sh --stop" ends the keep-alive.
#
# The headset's address is set below, now the G21 Pro earphones (or AIRPODS=aa:bb:... ~/connect_airpods.sh, e.g. the AirPods
# at E4:76:84:52:6E:61). The variable keeps its old name. The headset must be out of its case, and not
# connected to a laptop or phone.
AIRPODS=${AIRPODS:-24:DF:17:09:22:83}
ID=${AIRPODS//:/_}
SINK=bluez_output.$ID.1
SRC=bluez_input.$ID.0
CARD=bluez_card.$ID

pkill -f 'paplay.*zero' 2>/dev/null
[ "$1" = "--stop" ] && exit 0

# 1. Connect. They go to sleep in the case or when idle, so try a few times.
connected() { bluetoothctl info $AIRPODS 2>/dev/null | grep -q 'Connected: yes'; }
for try in 1 2 3; do
  connected && break
  echo "Connecting the AirPods (try $try)..."
  bluetoothctl connect $AIRPODS 2>&1 | tail -1
  sleep 4
done
connected || { echo "Could not connect. Take the AirPods out of the case, put one in your ear, and run this again."; exit 1; }

# 2. The login screen (gdm) runs its own audio system and takes Bluetooth audio from this user (see start_voice.sh).
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

# 4. The microphone only carries audio while something plays to the AirPods, so play silence. It keeps running after
#    this script ends; "--stop" ends it.
nohup paplay --raw --rate=16000 --channels=1 --format=s16le --device=$SINK /dev/zero >/dev/null 2>&1 &
echo "AirPods ready: speaker $SINK, microphone $SRC"
