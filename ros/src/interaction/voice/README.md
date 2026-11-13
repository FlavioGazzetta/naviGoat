# Voice: talk to the robot through AirPods on the Jetson

Listens to a microphone, turns speech into text with Whisper, gives the text to the [middleman](../../core/middleman.py), which decides what to do (follow, go to a place, go to an object, stop, ...), and says the reply out loud. On the Jetson the microphone and speaker are AirPods over Bluetooth.

Say "Pete, follow me", "Pete, take me to the bathroom", "Pete, stop". Phrases without "Pete" are ignored on purpose.

## Every time you log in

1. **Power the Jetson and connect to it** (see [Reaching the Jetson](#reaching-the-jetson)).
2. **Wake the AirPods.** Take them out of the case and put one in your ear. They must not be connected to a laptop or phone: turn those Bluetooth connections off first.
3. **Log in:** `ssh navigoat@<jetson-address>`
4. **Get the latest code** (skip if nothing changed): `cd ~/naviGoat && git pull`. If git says local changes would be overwritten, some files were copied over with `scp` before they were committed: run `git stash`, then `git pull`.
5. **Start it:** `~/start_voice.sh`. Wait for `Listening. Say 'Pete, ...'`. Ctrl+C stops it. Extra options are passed on, for example `~/start_voice.sh --no-speak` (print only) or `--qwen`.
6. **Talk.** Each phrase prints what was heard, what the middleman decided, and how long transcription took. The reply is spoken in the AirPod.

```
heard: Hello Pete, follow me.
  FOLLOW  target=-  reply="Following."
  (3.26 s of speech, transcribed in 1.78 s)
```

Each run saves a `NNN.wav` per phrase and a `log.jsonl` in `~/report/<date_time>/`. Copy them to a laptop with `scp -r navigoat@<jetson-address>:report .`.

### How it decides you have started and finished talking

The listener uses the Silero voice-detection model (the file ships inside `faster-whisper`, and it runs on the CPU in under a millisecond per block). It hears quiet speech, ignores steady noise and taps on the microphone, and keeps a pause between words from ending a phrase. A phrase ends after `--silence` seconds of quiet (0.5 in `start_voice.sh`). If you have said less than `--short-phrase` seconds so far (a bare "Pete,"), it waits `--short-silence` (1.0 s) instead, so the request after the pause stays in the same phrase. The quiet at the end is trimmed before it goes to Whisper. `--vad energy` switches back to the old loudness rule (`--min-rms`). On 53 recorded phrases the old rule missed 4 and the new one heard all of them, and 40 s of noise with taps produced no false phrase.

After a bare "Pete." the listener beeps and accepts the next phrase without the wake word for `--follow-up` seconds (4).

### If something does not work

| What you see | What to do |
| --- | --- |
| `Could not connect. Take the AirPods out of the case...` | They are asleep or connected to another device. Wake them, turn Bluetooth off on the laptop and phone, run the script again. `page-timeout` in the error means the same thing. |
| `The AirPods are connected but have no audio device` | The one-time fixes in step 4 of [Setting up a Jetson from scratch](#setting-up-a-jetson-from-scratch) are missing, or the login screen's audio is back. |
| It prints `Listening` but never hears you | The microphone is silent unless something is playing to the AirPods. The script plays silence for that. Check `pgrep -f 'paplay.*zero'` and that the AirPods are still connected. |
| Words are wrong | See [Quality](#what-to-know-about-bluetooth-headphones-as-the-microphone). Say short, clear commands starting with "Pete". |
| It talks to itself | The microphone is ignored while the robot speaks and for half a second after, but a long echo can still get through. Lower the speaker volume. |

## Reaching the Jetson

- **USB-C cable, `192.168.55.1`.** The simplest, but the link has dropped and SSH sometimes times out ("banner exchange" or "connection timed out"). Use a USB 3 data cable in the Jetson's USB-C device port. Retrying, or keeping one session open instead of many short ones, helps.
- **Phone hotspot (more stable).** Put the laptop and the Jetson on the same hotspot. The Jetson remembers the network once it has joined it (`nmcli device wifi connect "<name>" password "<password>"`, with `sudo`). Find its address on the phone's list of connected devices, or run `hostname -I` on the Jetson. On an iPhone, turn on Maximize Compatibility and keep the hotspot screen open until both devices have joined.
- The campus Wi-Fi does not work: it normally stops devices from seeing each other.
- **After a re-flash** the Jetson has a new SSH key and your laptop warns "REMOTE HOST IDENTIFICATION HAS CHANGED". If you know it was re-flashed, run `ssh-keygen -R <address>` and connect again.

## Files

| File | What it does |
| --- | --- |
| [start_voice.sh](start_voice.sh) | Goes in the Jetson's home folder as `~/start_voice.sh`. Connects the AirPods (3 tries), waits for their speaker and microphone, sets them as the default sound devices, keeps the microphone alive, then runs `listen_controller.py` with recording on |
| [listen_controller.py](listen_controller.py) | Microphone, Whisper, the middleman, then speaks the reply. No ROS. Records every phrase and logs it with `--save-dir` |
| [voice.py](voice.py) | The pieces the others use (microphone capture, Whisper, speaking), and the ROS version: publishes the text on `/speech/text` and speaks `/pete/response` |
| [whisper_server.py](whisper_server.py), [start_whisper_gpu.sh](start_whisper_gpu.sh) | Whisper on the GPU, as a small local server in the `navigoat:people-dev` container. Not set up on the current Jetson, see [Whisper on the GPU](#whisper-on-the-gpu) |

`start_voice.sh` contains the AirPods' Bluetooth address (`E4:76:84:52:6E:61`). For other headphones, run `AIRPODS=aa:bb:cc:dd:ee:ff ~/start_voice.sh`.

## Setting up a Jetson from scratch

Done on the current Jetson (Ubuntu 24.04, BlueZ 5.72, PipeWire 1.0.5, WirePlumber 0.4.17), which was re-flashed. Do this on the Jetson itself or over SSH.

**1. Packages.**
```bash
sudo apt-get install -y pulseaudio-utils libportaudio2 libasound2-plugins espeak-ng python3-venv
python3 -m venv --system-site-packages ~/voice-env
~/voice-env/bin/pip install "numpy<2" sounddevice faster-whisper pyttsx3
```
`pulseaudio-utils` only provides `pactl` and `paplay`. The sound server is PipeWire. The first start downloads Whisper `base.en` (about 140 MB).

**2. Bluetooth classic mode.** In `/etc/bluetooth/main.conf`, change `#ControllerMode = dual` to `ControllerMode = bredr`, **with the `#` removed**, then `sudo systemctl restart bluetooth`. Check with `grep ControllerMode /etc/bluetooth/main.conf`. Without it AirPods Pro fail to pair (`AuthenticationRejected`). It is a system-wide Bluetooth setting.

**3. Pair the AirPods.** Put them in pairing mode (case lid open, hold the back button until the light flashes white). In `bluetoothctl`: `agent NoInputNoOutput`, `default-agent`, `scan on`, then `pair`, `trust`, `connect` with the address that shows as `AirPods Pro`. Pair the moment it appears: BlueZ forgets an unpaired device a few seconds after it stops advertising, and `pair` then answers "Device not available" until it advertises again. An entry ending in "Find My" is Apple's tracking broadcast and cannot be paired.

**4. Bluetooth audio and the login screen.** Two things stop PipeWire from creating the AirPods' speaker and microphone when you are only logged in over SSH. Both are one-time fixes:

- WirePlumber only serves Bluetooth audio to the user logged in on the Jetson's own screen (`with-logind = true`). Copy the default and turn it off for this user:
  ```bash
  mkdir -p ~/.config/wireplumber/bluetooth.lua.d
  cp /usr/share/wireplumber/bluetooth.lua.d/50-bluez-config.lua ~/.config/wireplumber/bluetooth.lua.d/
  sed -i 's/\["with-logind"\] = true,/["with-logind"] = false,/' ~/.config/wireplumber/bluetooth.lua.d/50-bluez-config.lua
  systemctl --user restart wireplumber
  ```
- The login screen (gdm) runs its own PipeWire and takes Bluetooth audio first. The registration then fails for your user with `RegisterProfile() failed: org.bluez.Error.NotPermitted`. Switch off PipeWire for the `gdm` user only. The login screen still works, it just has no sound, so a monitor can still be used to log in:
  ```bash
  D=/var/lib/gdm3/.config/systemd/user
  sudo mkdir -p $D
  for u in pipewire.service pipewire.socket wireplumber.service pipewire-pulse.service pipewire-pulse.socket; do sudo ln -sf /dev/null $D/$u; done
  sudo chown -R gdm:gdm /var/lib/gdm3/.config
  sudo systemctl restart gdm
  ```
  To undo, delete those five links. Check with `ps -eo user,comm | grep -E 'wireplumber|pipewire'`: only your user should be listed. `start_voice.sh` also handles this if the login screen's audio is back, by stopping gdm (it asks for the sudo password).

Restarting `bluetooth` makes PipeWire lose its registration, so run `systemctl --user restart wireplumber` and reconnect the AirPods afterwards.

**5. Copy `start_voice.sh`** to `~/start_voice.sh` and `chmod +x` it.

## What to know about Bluetooth headphones as the microphone

- **The microphone only works while something is playing to the headphones.** With the headset profile active and nothing playing, the source delivers pure silence. `start_voice.sh` plays silence (`paplay ... /dev/zero`) to keep it alive. This was found by testing, and the cause was not diagnosed.
- **Quality is phone-call level.** The headset profile is 8 kHz mono and the AirPods offer no better profile for the microphone. In testing, many phrases were mis-heard ("Pete, you probably forgot to do technically much"). Accuracy was not measured on a test set, so there is no error rate. A wired or USB microphone is much better.
- **Volumes.** The mic starts low, so the script sets it to 100% and the listener's threshold to `--min-rms 0.006`. The speaker is set to 40%.
- **The AirPods disconnect** when they go back in the case or sit idle. Wake them and run `start_voice.sh` again.
- **The voice is the system voice** (`espeak-ng`, played through PipeWire with `paplay`). Piper and Kokoro voices are used if installed and named with `--voice`, but they are not installed on the Jetson.
- BlueZ logs show warnings (`already registered`, `RFCOMM receive command but modem not available`). The setup works despite them.

## Whisper on the GPU

The code first ran Whisper on the CPU through `faster-whisper`. Its ARM build from pip has no CUDA, so on the current Jetson it runs on the CPU: about 1.6 to 2.8 s per phrase of 2 to 4 s with `base.en`, plus the 0.8 s of silence the listener waits for (`--silence`). On the previous install a GPU server was set up (`whisper_server.py`, run in the `navigoat:people-dev` container with OpenAI's `openai-whisper`), which measured 0.17 to 0.45 s with `base.en` and 0.15 to 0.24 s with `tiny.en`. The re-flashed Jetson does not have that container image, so `start_voice.sh` uses the CPU and only tries the GPU server if both `~/start_whisper_gpu.sh` and the image exist.

## Not done yet

- **A reboot has not been tested** with the login-screen fix. If `start_voice.sh` asks for a password after a reboot, the login screen's audio came back.
- **The ROS version is not running on the Jetson.** `voice.py` needs `rclpy`, and the Jetson host has no ROS (it is in the Docker image). The middleman's ROS node (`core/middleman_node.py`) was tested in the container with the real ROS 2 and with Flavio's real scene description node, but not with the real object search or the camera.
- **Nothing acts on the decisions yet.** "Following." is only said; no follow or navigation node reads `/middleman/mode`.
- **GPU Whisper together with Pete's models** (Qwen and YOLO) is untested. They share the 8 GB of the Orin Nano, so a smaller Whisper model may be needed.
