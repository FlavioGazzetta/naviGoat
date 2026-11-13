"""Listen to the microphone, turn speech into text, give it to the middleman (core/middleman.py) and speak its reply.

No ROS: it beeps when a request is heard, prints what it heard and what the middleman decided, and says the reply
out loud ("Following.", "Going to the bathroom.", "I can't look for things."). The microphone is ignored while it speaks.

    python3 listen_controller.py
    python3 listen_controller.py --list-devices
    python3 listen_controller.py --no-wake-word    (act on everything heard)
    python3 listen_controller.py --save-dir run1   (keep each phrase as run1/NNN.wav and log it in run1/log.jsonl)
    python3 listen_controller.py --qwen            (Qwen reads each request after the stop and help rules; needs Ollama)
    python3 listen_controller.py --no-speak        (print only, say nothing)
    python3 listen_controller.py --output-device 3 (speak through another device, e.g. the headphones; see --list-devices)

Say "Pete, follow this person" or "Pete, take me to the bathroom". Ctrl+C to quit.
"""

import argparse
import json
import sys
import time
import wave
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'core'))

import sounddevice as sd

import numpy as np

from middleman import QWEN_MODEL, Middleman, format_decision, qwen_interpreter
from voice import SAMPLE_RATE, WAKE_WORD, Speaker, load_model, load_prompt, transcribe, utterances


def beep():
    """A short tone so the user knows the request was heard and is being processed. Never blocks, never fails."""
    try:
        t = np.arange(int(0.12 * 24000)) / 24000
        sd.play((0.25 * np.sin(2 * np.pi * 880 * t)).astype('float32'), 24000)
    except Exception:
        pass


def save_phrase(save_dir, number, audio, record):
    """Write the phrase as a wav file and add one line about it to log.jsonl."""
    save_dir.mkdir(parents=True, exist_ok=True)
    record['wav'] = f'{number:03d}.wav'
    with wave.open(str(save_dir / record['wav']), 'wb') as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(SAMPLE_RATE)
        wav.writeframes((audio.clip(-1, 1) * 32767).astype('<i2').tobytes())
    with open(save_dir / 'log.jsonl', 'a') as log:
        log.write(json.dumps(record) + '\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--list-devices', action='store_true', help='list audio devices and exit')
    parser.add_argument('--device', default=None, help='input device index or name (default: system default)')
    parser.add_argument('--output-device', default=None, help='output device index or name (default: system default)')
    parser.add_argument('--no-speak', action='store_true', help='do not say the replies out loud')
    parser.add_argument('--voice', default='en_GB-northern_english_male-medium', help="Kokoro voice (bm_george, ...), a Piper voice name, or '' for the system voice")
    parser.add_argument('--voice-dir', default='~/.cache/piper', help='folder holding the Piper voice files')
    parser.add_argument('--speed', type=float, default=1.0, help='speaking speed: above 1 is faster, below 1 is slower')
    parser.add_argument('--model', default='base.en', help='Whisper model: tiny.en, base.en, small.en, ...')
    parser.add_argument('--whisper-url', default=None, help='use the Whisper GPU server (whisper_server.py), e.g. http://127.0.0.1:8765, instead of the CPU')
    parser.add_argument('--no-wake-word', dest='wake_word', action='store_false', help="act on everything heard, not only phrases containing 'Pete'")
    parser.add_argument('--min-rms', type=float, default=0.01, help='minimum loudness that counts as speech; raise it in noisy rooms')
    parser.add_argument('--follow-up', type=float, default=4.0, help="seconds after a bare 'Pete' in which the next phrase needs no wake word")
    parser.add_argument('--silence', type=float, default=0.8, help='seconds of quiet that end a phrase')
    parser.add_argument('--vad', choices=('silero', 'energy'), default='silero', help='how speech is told from quiet: the Silero model, or the loudness rule (--min-rms)')
    parser.add_argument('--short-phrase', type=float, default=0.8, help='speech shorter than this (seconds) is probably a bare "Pete": wait --short-silence before ending it')
    parser.add_argument('--short-silence', type=float, default=1.0, help='seconds of quiet that end a very short phrase')
    parser.add_argument('--max-phrase', type=float, default=15.0, help='longest phrase in seconds')
    parser.add_argument('--save-dir', type=Path, default=None, help='save each phrase as a wav file and log what was heard and decided')
    parser.add_argument('--qwen', action='store_true', help='let Qwen (through Ollama) read each request after the stop and help rules')
    parser.add_argument('--qwen-model', default=QWEN_MODEL, help='Ollama model used with --qwen')
    args = parser.parse_args()
    if args.list_devices:
        print(sd.query_devices())
        return
    device = int(args.device) if args.device and args.device.isdigit() else args.device
    output_device = int(args.output_device) if args.output_device and args.output_device.isdigit() else args.output_device

    model = load_model(args)
    prompt = load_prompt()
    interpreter = qwen_interpreter(model=args.qwen_model) if args.qwen else None
    if interpreter:
        print(f'Loading {args.qwen_model} in Ollama...')
        if not interpreter.warm_up():
            print('Cannot reach Ollama, using the rules only.')
            interpreter = None
    middleman = Middleman(interpreter=interpreter, on_ack=beep)
    speaker = None if args.no_speak else Speaker(args.voice, args.voice_dir, output_device, args.speed)

    def muted():
        """True while the robot is talking and for a moment after, so it does not hear itself."""
        return speaker is not None and (speaker.busy() or time.monotonic() < speaker.until + 0.5)

    print("Listening. Say 'Pete, ...'" if args.wake_word else 'Listening.')
    number = 0
    follow_up_until = 0.0  # after a bare 'Pete' (and a beep), the next phrase is meant for the robot
    try:
        for audio in utterances(device, muted, args.min_rms, args.silence, args.max_phrase, vad=args.vad,
                               short_s=args.short_phrase, short_silence_s=args.short_silence):
            started = time.monotonic()
            text = transcribe(model, audio, prompt)
            took = time.monotonic() - started
            if not text:
                continue
            number += 1
            record = {'time': datetime.now().isoformat(timespec='seconds'), 'heard': text, 'audio_s': round(len(audio) / SAMPLE_RATE, 2), 'transcribe_s': round(took, 2)}
            if args.wake_word and not WAKE_WORD.search(text.lower()) and time.monotonic() > follow_up_until:
                print(f'ignored (no wake word): {text}')
                record['decision'] = 'ignored'
            else:
                print(f'heard: {text}')
                # Whisper often writes the name as peet or peat; the middleman only knows "pete".
                started = time.monotonic()
                decision = middleman.handle(WAKE_WORD.sub('pete', text.lower()))
                follow_up_until = 0.0
                if decision is None and WAKE_WORD.search(text.lower()):
                    follow_up_until = time.monotonic() + args.follow_up  # "Pete." on its own: the request is coming
                    beep()
                record['understand_s'] = round(time.monotonic() - started, 2)
                print(f'  {format_decision(decision)}' if decision else '  (nothing to do)')
                record['decision'] = format_decision(decision) if decision else 'nothing to do'
                if decision and decision.reply and speaker:
                    speaker.say(decision.reply)
                if decision:
                    record.update(mode='STOP' if decision.stop else decision.mode.value if decision.mode else 'none', intent=decision.intent, target=decision.target, reply=decision.reply, forward=decision.forward)
            timings = f'{record["audio_s"]} s of speech, transcribed in {record["transcribe_s"]} s'
            if 'understand_s' in record and interpreter:
                timings += f', understood in {record["understand_s"]} s'
            print(f'  ({timings})')
            if args.save_dir:
                save_phrase(args.save_dir, number, audio, record)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
