"""Voice: speech to text and text to speech for the robot. Everything is in this file.

Listens to the microphone, turns speech into text with Whisper and sends it to Pete,
then speaks Pete's replies. See README.md in this folder.

Without ROS (typed lines stand in for Pete's replies):
    python3 voice.py --terminal

With ROS:
    python3 voice.py
    # publishes std_msgs/String on /pete/command, speaks /pete/response
"""

import argparse
import json
import queue
import re
import shutil
import subprocess
import sys
import threading
import time
import urllib.parse
import urllib.request
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import sounddevice as sd

SAMPLE_RATE = 16000
BLOCK = 512  # 32 ms of audio per block

WAKE_WORD = re.compile(r'\b(pete|peet|peat)\b')
# Nudges Whisper towards the name and the kind of requests Pete understands.
PROMPT = "Pete, where is the chair? Pete, what's in front of me? Pete, bring me to the door. Pete, stop."
# What Whisper tends to invent when it is given noise.
HALLUCINATIONS = {'thank you', 'thanks', 'thanks for watching', 'you', 'bye', 'okay', 'so', 'the', 'uh', 'um', 'hmm'}


class Speaker:
    """Speaks text on its own thread, so a reply never blocks listening."""

    def __init__(self, voice=None, voice_dir=None, device=None, speed=1.0):
        self.voice, self.voice_dir, self.device, self.speed = voice, voice_dir, device, speed
        self.queue = queue.Queue()
        self.speaking = False
        self.until = 0.0  # when the last reply finished
        threading.Thread(target=self._run, daemon=True).start()

    def say(self, text):
        self.queue.put(text)

    def busy(self):
        return self.speaking or not self.queue.empty()

    def _piper(self):
        from piper import PiperVoice, SynthesisConfig
        path = Path(self.voice_dir).expanduser() / f'{self.voice}.onnx'
        voice = PiperVoice.load(path)
        config = SynthesisConfig(length_scale=1 / self.speed)

        def speak(text):
            # Piper yields one chunk per sentence: start speaking the first while the rest is made.
            ready = queue.Queue()

            def generate():
                for chunk in voice.synthesize(text, config):
                    ready.put(chunk.audio_float_array)
                ready.put(None)
            threading.Thread(target=generate, daemon=True).start()
            while (audio := ready.get()) is not None:
                sd.play(audio, voice.config.sample_rate, device=self.device)
                sd.wait()
        return speak

    def _kokoro(self):
        from kokoro_onnx import Kokoro
        folder = Path(self.voice_dir).expanduser().parent / 'kokoro'
        kokoro = Kokoro(str(folder / 'kokoro-v1.0.onnx'), str(folder / 'voices-v1.0.bin'))
        lang = 'en-gb' if self.voice.startswith('b') else 'en-us'

        make = lambda text: kokoro.create(text, voice=self.voice, speed=self.speed, lang=lang)
        make('Ready.')  # the first call is slow; do it now instead of during the first reply

        def speak(text):
            # Generation is slower than speech to start with, so say the first sentence while the rest is made.
            ready = queue.Queue()

            def generate():
                for sentence in re.split(r'(?<=[.!?])\s+', text.strip()):
                    ready.put(make(sentence) if sentence else None)
                ready.put(False)
            threading.Thread(target=generate, daemon=True).start()
            while (item := ready.get()) is not False:
                if item is not None:
                    sd.play(item[0], item[1], device=self.device)
                    sd.wait()
        return speak

    def _backend(self):
        if self.voice:
            try:
                # Kokoro voices are named like bm_george (British male); anything else is a Piper voice.
                return self._kokoro() if re.match(r'^[ab][mf]_', self.voice) else self._piper()
            except Exception as error:
                print(f'Piper voice {self.voice} not available ({error}); using the system voice.', file=sys.stderr)
        try:
            import pyttsx3
            pyttsx3.init()  # fail here, not on the first reply, if there is no voice

            def speak(text):
                # A new engine every time: on Windows a reused one speaks once, then goes silent.
                engine = pyttsx3.init()
                engine.setProperty('rate', 175)
                engine.say(text)
                engine.runAndWait()
            return speak
        except Exception:
            pass
        for exe in ('espeak-ng', 'espeak'):
            if shutil.which(exe):
                return lambda text: subprocess.run([exe, text], check=False)
        print('No text to speech available: pip install pyttsx3, or install espeak-ng.', file=sys.stderr)
        return lambda text: None

    def _run(self):
        speak = self._backend()  # pyttsx3 must be used on the thread that created it
        while True:
            text = self.queue.get()
            self.speaking = True
            try:
                speak(text)
            except Exception as error:
                print(f'Could not speak: {error}', file=sys.stderr)
            finally:
                self.speaking = False
                self.until = time.monotonic()


def utterances(device, muted, min_rms, silence_s, max_s, min_s=0.4):
    """Yield one float32 array per spoken phrase.

    A block counts as speech when it is clearly louder than the background level, which
    is tracked while nobody is talking. A phrase ends after `silence_s` of quiet.
    """
    blocks = queue.Queue()
    stream = sd.InputStream(samplerate=SAMPLE_RATE, blocksize=BLOCK, channels=1, dtype='float32', device=device,
                            callback=lambda indata, frames, time_info, status: blocks.put(indata[:, 0].copy()))
    preroll = deque(maxlen=8)  # keeps the start of a word that crossed the threshold late
    active, silent, floor = [], 0, 0.005
    seconds = lambda count: count * BLOCK / SAMPLE_RATE

    with stream:
        while True:
            block = blocks.get()
            if muted():  # the robot is talking: don't listen to ourselves
                active, silent = [], 0
                preroll.clear()
                continue
            rms = float(np.sqrt(np.mean(block ** 2)))
            loud = rms > max(min_rms, floor * 3)
            if not active:
                if loud:
                    active, silent = list(preroll) + [block], 0
                else:
                    floor = 0.95 * floor + 0.05 * rms
                    preroll.append(block)
                continue
            active.append(block)
            silent = 0 if loud else silent + 1
            if seconds(silent) >= silence_s or seconds(len(active)) >= max_s:
                if seconds(len(active) - silent) >= min_s:
                    yield np.concatenate(active)
                active, silent = [], 0


class RemoteWhisper:
    """Stands in for faster-whisper's WhisperModel, but sends the audio to whisper_server.py, which runs on the GPU."""

    def __init__(self, url):
        self.url = url.rstrip('/')

    def transcribe(self, audio, initial_prompt='', **_):
        query = urllib.parse.urlencode({'prompt': initial_prompt or ''})
        request = urllib.request.Request(f'{self.url}/transcribe?{query}', data=audio.astype('<f4').tobytes())
        with urllib.request.urlopen(request, timeout=30) as response:
            segments = json.load(response)['segments']
        return [SimpleNamespace(**segment) for segment in segments], None


def load_model(args):
    """Whisper on the GPU server if --whisper-url is given, otherwise faster-whisper on this machine's CPU."""
    if args.whisper_url:
        print(f'Using Whisper on the GPU server at {args.whisper_url}')
        return RemoteWhisper(args.whisper_url)
    from faster_whisper import WhisperModel
    print(f'Loading Whisper ({args.model})...')
    return WhisperModel(args.model, device='auto', compute_type='default')


def transcribe(model, audio):
    segments, _ = model.transcribe(audio, language='en', beam_size=1, vad_filter=True, initial_prompt=PROMPT,
                                   condition_on_previous_text=False)
    parts = [s.text.strip() for s in segments if s.no_speech_prob < 0.6 and s.avg_logprob > -1.0]
    text = ' '.join(parts).strip()
    if re.sub(r'[^a-z ]', '', text.lower()).strip() in HALLUCINATIONS:
        return ''
    return text


class Voice:
    def __init__(self, args, publish):
        self.args = args
        self.publish = publish
        self.speaker = Speaker(args.voice, args.voice_dir, args.output_device, args.speed)
        self.awaiting_follow_up = False  # set by a reply: the next phrase may skip the wake word

    def on_reply(self, text):
        print(f'robot: {text}')
        self.awaiting_follow_up = True
        self.speaker.say(text)

    def addressed(self, text):
        if not self.args.wake_word or WAKE_WORD.search(text.lower()):
            return True
        # After the robot has spoken, a short answer ("yes", "the red one") is meant for it.
        return self.awaiting_follow_up and time.monotonic() < self.speaker.until + self.args.follow_up

    def listen(self):
        model = load_model(self.args)

        def muted():
            return self.speaker.busy() or time.monotonic() < self.speaker.until + 0.5

        print("Listening. Say 'Pete, ...'" if self.args.wake_word else 'Listening.')
        for audio in utterances(self.args.device, muted, self.args.min_rms, self.args.silence, self.args.max_phrase):
            text = transcribe(model, audio)
            if not text:
                continue
            if self.addressed(text):
                print(f'heard: {text}')
                self.awaiting_follow_up = False
                self.publish(text)
            else:
                print(f'ignored (not for me): {text}')


def run_terminal(args):
    voice = Voice(args, lambda text: print(f'-> /pete/command: {text}'))
    if not args.no_listen:
        threading.Thread(target=voice.listen, daemon=True).start()
    print('Type a line to have it spoken, standing in for a Pete reply. Ctrl+C to quit.')
    try:
        for line in sys.stdin:
            if line.strip():
                voice.on_reply(line.strip())
    except KeyboardInterrupt:
        pass


def run_ros(args):
    import rclpy
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node('voice')
    command_pub = node.create_publisher(String, '/pete/command', 10)
    voice = Voice(args, lambda text: command_pub.publish(String(data=text)))
    node.create_subscription(String, '/pete/response', lambda msg: voice.on_reply(msg.data), 10)
    threading.Thread(target=voice.listen, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--terminal', action='store_true', help='run without ROS')
    parser.add_argument('--no-listen', action='store_true', help='terminal mode: only speak, skip the microphone and Whisper')
    parser.add_argument('--list-devices', action='store_true', help='list audio devices and exit')
    parser.add_argument('--device', default=None, help='input device index or name (default: system default)')
    parser.add_argument('--output-device', default=None, help='output device index or name (default: system default)')
    parser.add_argument('--voice', default='en_GB-northern_english_male-medium', help="Kokoro voice (bm_george, bm_fable, ...), a Piper voice name, or '' for the system voice")
    parser.add_argument('--speed', type=float, default=1.0, help='speaking speed: above 1 is faster, below 1 is slower')
    parser.add_argument('--voice-dir', default='~/.cache/piper', help='folder holding the Piper voice files')
    parser.add_argument('--model', default='base.en', help='Whisper model: tiny.en, base.en, small.en, ...')
    parser.add_argument('--whisper-url', default=None, help='use the Whisper GPU server (whisper_server.py), e.g. http://127.0.0.1:8765, instead of the CPU')
    parser.add_argument('--no-wake-word', dest='wake_word', action='store_false', help="act on everything heard, not only phrases containing 'Pete'")
    parser.add_argument('--follow-up', type=float, default=6.0, help='seconds after a reply in which no wake word is needed')
    parser.add_argument('--min-rms', type=float, default=0.01, help='minimum loudness that counts as speech; raise it in noisy rooms')
    parser.add_argument('--silence', type=float, default=0.8, help='seconds of quiet that end a phrase')
    parser.add_argument('--max-phrase', type=float, default=15.0, help='longest phrase in seconds')
    args, _ = parser.parse_known_args()  # leave --ros-args for ROS
    if args.list_devices:
        print(sd.query_devices())
        sys.exit()
    for name in ('device', 'output_device'):
        if isinstance(getattr(args, name), str) and getattr(args, name).isdigit():
            setattr(args, name, int(getattr(args, name)))
    run_terminal(args) if args.terminal else run_ros(args)
