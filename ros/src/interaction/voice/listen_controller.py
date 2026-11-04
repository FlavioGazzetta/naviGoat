"""Listen to the microphone, turn speech into text and give it to the controller.

No speaking and no ROS: it only prints what it heard and what the controller decided.

    python3 listen_controller.py
    python3 listen_controller.py --list-devices
    python3 listen_controller.py --no-wake-word    (act on everything heard)
    python3 listen_controller.py --save-dir run1   (keep each phrase as run1/NNN.wav and log it in run1/log.jsonl)

Say "Pete, follow me" or "Pete, take me to the bathroom". Ctrl+C to quit.
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

from controller import Controller, format_decision
from voice import SAMPLE_RATE, WAKE_WORD, load_model, transcribe, utterances


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
    parser.add_argument('--model', default='base.en', help='Whisper model: tiny.en, base.en, small.en, ...')
    parser.add_argument('--whisper-url', default=None, help='use the Whisper GPU server (whisper_server.py), e.g. http://127.0.0.1:8765, instead of the CPU')
    parser.add_argument('--no-wake-word', dest='wake_word', action='store_false', help="act on everything heard, not only phrases containing 'Pete'")
    parser.add_argument('--min-rms', type=float, default=0.01, help='minimum loudness that counts as speech; raise it in noisy rooms')
    parser.add_argument('--silence', type=float, default=0.8, help='seconds of quiet that end a phrase')
    parser.add_argument('--max-phrase', type=float, default=15.0, help='longest phrase in seconds')
    parser.add_argument('--save-dir', type=Path, default=None, help='save each phrase as a wav file and log what was heard and decided')
    args = parser.parse_args()
    if args.list_devices:
        print(sd.query_devices())
        return
    device = int(args.device) if args.device and args.device.isdigit() else args.device

    model = load_model(args)
    controller = Controller()

    print("Listening. Say 'Pete, ...'" if args.wake_word else 'Listening.')
    number = 0
    try:
        for audio in utterances(device, lambda: False, args.min_rms, args.silence, args.max_phrase):
            started = time.monotonic()
            text = transcribe(model, audio)
            took = time.monotonic() - started
            if not text:
                continue
            number += 1
            record = {'time': datetime.now().isoformat(timespec='seconds'), 'heard': text, 'audio_s': round(len(audio) / SAMPLE_RATE, 2), 'transcribe_s': round(took, 2)}
            if args.wake_word and not WAKE_WORD.search(text.lower()):
                print(f'ignored (no wake word): {text}')
                record['decision'] = 'ignored'
            else:
                print(f'heard: {text}')
                # Whisper often writes the name as peet or peat; the controller only knows "pete".
                decision = controller.handle(WAKE_WORD.sub('pete', text.lower()))
                print(f'  {format_decision(decision)}' if decision else '  (nothing to do)')
                record['decision'] = format_decision(decision) if decision else 'nothing to do'
                if decision:
                    record.update(mode='STOP' if decision.stop else decision.mode.value, intent=decision.intent, target=decision.target)
            print(f'  ({record["audio_s"]} s of speech, transcribed in {record["transcribe_s"]} s)')
            if args.save_dir:
                save_phrase(args.save_dir, number, audio, record)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
