"""Listen to the microphone, turn speech into text and give it to the controller.

No speaking and no ROS: it only prints what it heard and what the controller decided.

    python3 listen_controller.py
    python3 listen_controller.py --list-devices
    python3 listen_controller.py --no-wake-word    (act on everything heard)

Say "Pete, follow me" or "Pete, take me to the bathroom". Ctrl+C to quit.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'core'))

import sounddevice as sd

from controller import Controller, format_decision
from voice import WAKE_WORD, transcribe, utterances


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--list-devices', action='store_true', help='list audio devices and exit')
    parser.add_argument('--device', default=None, help='input device index or name (default: system default)')
    parser.add_argument('--model', default='base.en', help='Whisper model: tiny.en, base.en, small.en, ...')
    parser.add_argument('--no-wake-word', dest='wake_word', action='store_false', help="act on everything heard, not only phrases containing 'Pete'")
    parser.add_argument('--min-rms', type=float, default=0.01, help='minimum loudness that counts as speech; raise it in noisy rooms')
    parser.add_argument('--silence', type=float, default=0.8, help='seconds of quiet that end a phrase')
    parser.add_argument('--max-phrase', type=float, default=15.0, help='longest phrase in seconds')
    args = parser.parse_args()
    if args.list_devices:
        print(sd.query_devices())
        return
    device = int(args.device) if args.device and args.device.isdigit() else args.device

    from faster_whisper import WhisperModel
    print(f'Loading Whisper ({args.model})...')
    model = WhisperModel(args.model, device='auto', compute_type='default')
    controller = Controller()

    print("Listening. Say 'Pete, ...'" if args.wake_word else 'Listening.')
    try:
        for audio in utterances(device, lambda: False, args.min_rms, args.silence, args.max_phrase):
            text = transcribe(model, audio)
            if not text:
                continue
            if args.wake_word and not WAKE_WORD.search(text.lower()):
                print(f'ignored (no wake word): {text}')
                continue
            print(f'heard: {text}')
            # Whisper often writes the name as peet or peat; the controller only knows "pete".
            decision = controller.handle(WAKE_WORD.sub('pete', text.lower()))
            print(f'  {format_decision(decision)}' if decision else '  (nothing to do)')
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
