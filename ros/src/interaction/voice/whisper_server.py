"""Whisper on the GPU, served over HTTP so voice.py and listen_controller.py can use it.

Runs in the navigoat:people-dev container, which has PyTorch with CUDA, with openai-whisper installed:

    python3 whisper_server.py --model base.en

Then start the listener with --whisper-url http://127.0.0.1:8765.

POST /transcribe?prompt=... takes 16 kHz mono float32 audio as the body and replies with the segments as JSON.
GET /health replies with the device the model is on.
"""

import argparse
import json
import warnings
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np
import torch
import whisper

warnings.filterwarnings('ignore')


class Handler(BaseHTTPRequestHandler):
    model = None

    def _reply(self, data):
        body = json.dumps(data).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        self._reply({'ok': True, 'device': str(next(self.model.parameters()).device)})

    def do_POST(self):
        prompt = parse_qs(urlparse(self.path).query).get('prompt', [''])[0]
        audio = np.frombuffer(self.rfile.read(int(self.headers['Content-Length'])), dtype='<f4').astype(np.float32)
        result = whisper.transcribe(self.model, audio, language='en', fp16=True, condition_on_previous_text=False,
                                    initial_prompt=prompt or None)
        self._reply({'segments': [{'text': s['text'], 'no_speech_prob': s['no_speech_prob'], 'avg_logprob': s['avg_logprob']}
                                  for s in result['segments']]})

    def log_message(self, *args):
        pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default='base.en', help='Whisper model: tiny.en, base.en, small.en, ...')
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()

    print(f'Loading Whisper ({args.model}) on {"the GPU" if torch.cuda.is_available() else "the CPU"}...', flush=True)
    Handler.model = whisper.load_model(args.model, device='cuda' if torch.cuda.is_available() else 'cpu')
    whisper.transcribe(Handler.model, np.zeros(16000, dtype=np.float32), language='en', fp16=True)  # the first call is slow
    print(f'Ready on http://127.0.0.1:{args.port}', flush=True)
    HTTPServer(('127.0.0.1', args.port), Handler).serve_forever()
