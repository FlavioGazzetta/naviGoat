#!/bin/bash
# For a demo: lets Pete speak through THIS computer's speakers, so the room hears it. Run it on the laptop, not the Jetson.
#
#   ros/src/interaction/voice/show_speaker.sh           start accepting Pete's voice from the Jetson
#   ros/src/interaction/voice/show_speaker.sh --stop    stop
#
# Then start Pete on the Jetson with show:=true (see core/start_pete.sh). The microphone stays the headset.
# It opens the laptop's sound system to the network link to the Jetson only (the USB cable's, 192.168.55.100), without
# a password, so anything on that link could play sound here: stop it after the demo.
ADDRESS=${SHOW_ADDRESS:-192.168.55.100}  # this laptop's address on the link to the Jetson

for module in $(pactl list short modules | awk '/module-native-protocol-tcp/ {print $1}'); do pactl unload-module "$module"; done
[ "$1" = "--stop" ] && { echo "Stopped: the Jetson can no longer play sound here."; exit 0; }

if ! ip -4 addr | grep -q "inet $ADDRESS/"; then
  echo "This computer has no address $ADDRESS. Plug in the Jetson's USB cable, or give the address the Jetson can reach:"
  echo "  SHOW_ADDRESS=<address> $0"
  exit 1
fi
pactl load-module module-native-protocol-tcp listen="$ADDRESS" auth-anonymous=1 >/dev/null || { echo "Could not open the sound system to the network."; exit 1; }
echo "Ready: Pete can speak through this computer ($ADDRESS). On the Jetson, add show:=true to the start command."
