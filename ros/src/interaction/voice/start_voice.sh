#!/bin/bash
# Connect the AirPods, route audio to them, keep the voice link open, start Whisper on the GPU and run the listener.
A=E4:76:84:52:6E:61
SINK=bluez_output.E4_76_84_52_6E_61.1
SRC=bluez_input.E4_76_84_52_6E_61.0
pkill -f listen_controller.py 2>/dev/null
pkill -f 'paplay.*zero' 2>/dev/null
bluetoothctl info $A | grep -q 'Connected: yes' || { echo "Connecting AirPods..."; bluetoothctl connect $A; sleep 6; }
pactl set-card-profile bluez_card.E4_76_84_52_6E_61 headset-head-unit
sleep 2
pactl set-default-sink $SINK
pactl set-default-source $SRC
pactl set-sink-volume $SINK 40%
pactl set-source-volume $SRC 100%
# The mic only carries audio while something plays to the AirPods, so play silence.
nohup paplay --raw --rate=8000 --channels=1 --format=s16le --device=$SINK /dev/zero >/dev/null 2>&1 &
KEEP=$!
trap 'kill $KEEP 2>/dev/null' EXIT
WHISPER=""
~/start_whisper_gpu.sh && WHISPER="--whisper-url http://127.0.0.1:8765" || echo "GPU server did not start: using the CPU instead"
RUN=~/report/$(date +%Y%m%d_%H%M%S)
cd ~/naviGoat/ros/src/interaction/voice
echo "Saving recordings and log to $RUN"
~/voice-env/bin/python -u listen_controller.py --device pulse --min-rms 0.006 --save-dir "$RUN" $WHISPER