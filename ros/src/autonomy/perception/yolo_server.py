#!/usr/bin/env python3
"""YOLO-World on the GPU, loaded once and served over HTTP, the way Ollama serves the language model.

object_detection.py and the people tracker (social_perception/femto_track_people.py --yolo-url) both ask this server,
so PyTorch, CUDA and YOLO are in memory once and neither node depends on the other. No ROS in here.

    python3 yolo_server.py                 on http://127.0.0.1:8767, only reachable from this machine
    python3 yolo_server.py --host 0.0.0.0  reachable from other machines too (Pete on the laptop: yolo_url:=http://<this machine>:8767).
                                           There is no password: anyone on the same network can use it

    GET  /health    {"ok": true, "words": 80}
    POST /words     body: a JSON list of words. YOLO looks for these from now on. Words from core/catalog.py are
                    instant; any other word is worked out first, which takes seconds on the Jetson
    POST /detect?w=..&h=..&imgsz=..&conf=..   body: the picture as raw BGR bytes (h x w x 3)
                    {"boxes": [[x1, y1, x2, y2, confidence, word], ...], "ms": YOLO's time}
        &only=person    only that word's boxes
        &search=1       this request is part of an object search
        &yield=1        reply {"busy": true} while an object search is running, so the search has the whole GPU
"""

import argparse
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np

# The word lists are shared with the middleman (core/catalog.py), so everything knows the same objects
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'core'))
from catalog import VOCABULARY, WORN  # noqa: E402
KNOWN = VOCABULARY + WORN  # every word whose numbers are worked out at launch
SEARCH_GAP = 0.5  # seconds without a search request before the search counts as over


class Yolo:
    def __init__(self, path):
        from ultralytics import YOLOWorld
        self.lock = threading.Lock()
        self.model = YOLOWorld(path)
        self.words = list(KNOWN)
        self.model.set_classes(self.words)
        self.list_features = self.model.model.txt_feats.clone()  # for every word in KNOWN
        self.last_search = 0.0
        self.use_words(VOCABULARY)
        for h, w, size in ((720, 1280, 640), (396, 704, 640), (360, 640, 640), (180, 320, 320)):
            self.model.predict(np.zeros((h, w, 3), np.uint8), imgsz=size, verbose=False)

    def use_words(self, words):
        """Point YOLO at a word list: list words reuse their numbers from launch, any other word is worked out now."""
        import torch
        words = list(words)
        if words == self.words:
            return
        parts = [self.list_features[:, [KNOWN.index(w) for w in words if w in KNOWN]]]
        new = [w for w in words if w not in KNOWN]
        if new:
            parts.append(self.model.model.get_text_pe(new).to(self.list_features))
        with self.lock:
            world = self.model.model
            world.txt_feats = torch.cat(parts, dim=1)
            world.model[-1].nc = len(words)
            self.model.model.names = words
            self.model.predictor = None  # rebuilt on the next prediction, with the new number of words
            self.words = words

    def detect(self, image, image_size, confidence, only):
        with self.lock:
            words = self.words
            if only and only not in words:
                return [], 0.0
            result = self.model.predict(image, imgsz=image_size, conf=confidence, verbose=False,
                                        classes=[words.index(only)] if only else None)[0]
        boxes = [box + [conf, words[int(index)]] for box, conf, index in
                 zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist(), result.boxes.cls.tolist())]
        return boxes, float(result.speed.get('inference', 0.0))


class Handler(BaseHTTPRequestHandler):
    yolo = None

    def reply(self, data):
        body = json.dumps(data).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        self.reply({'ok': True, 'words': len(self.yolo.words)})

    def do_POST(self):
        url = urlparse(self.path)
        query = {key: values[0] for key, values in parse_qs(url.query).items()}
        data = self.rfile.read(int(self.headers['Content-Length']))
        try:  # a bad request must not stop the server
            if url.path == '/words':
                self.yolo.use_words(json.loads(data))
                return self.reply({'ok': True})
            if query.get('search'):
                self.yolo.last_search = time.monotonic()
            elif query.get('yield') and time.monotonic() - self.yolo.last_search < SEARCH_GAP:
                return self.reply({'busy': True})
            image = np.frombuffer(data, np.uint8).reshape(int(query['h']), int(query['w']), 3)
            boxes, ms = self.yolo.detect(image, int(query.get('imgsz', 640)), float(query.get('conf', 0.25)), query.get('only'))
            self.reply({'boxes': boxes, 'ms': ms})
        except Exception as error:
            self.reply({'error': f'{type(error).__name__}: {error}'})

    def log_message(self, *args):
        pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default=os.environ.get('PETE_YOLO_MODEL', 'yolov8s-worldv2.pt'))
    parser.add_argument('--port', type=int, default=8767)
    parser.add_argument('--host', default='127.0.0.1', help='0.0.0.0: also reachable from other machines')
    args = parser.parse_args()

    print(f'Loading YOLO ({args.model})...', flush=True)
    Handler.yolo = Yolo(args.model)
    # One thread answers every request in turn. A thread per request is slower here: PyTorch sets the GPU libraries up
    # again in each new thread (measured: 11 ms per picture instead of 3).
    server = HTTPServer((args.host, args.port), Handler)
    print(f'yolo_server ready on http://{args.host}:{args.port}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
