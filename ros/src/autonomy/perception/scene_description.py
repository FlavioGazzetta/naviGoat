#!/usr/bin/env python3
"""When /mode switches to 'description', describes the camera image with Qwen3-VL (through Ollama) on /scene_description."""

import base64
import json
import re
import threading
import time
import urllib.error
import urllib.request

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

OLLAMA_URL = 'http://localhost:11434/api/chat'
VLM_MODEL = 'qwen3-vl:2b-instruct'  # the plain tag is a thinking model, too slow here
OLLAMA_OPTIONS = {'num_ctx': 4096}  # keep identical on every request, or Ollama reloads the model
CAMERA_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

SYSTEM_PROMPT = """You are Pete, a guide robot for a blind or low-vision person. \
You see through your forward-facing camera, which points the way the person is heading.

You get one picture: the whole view on top, and close-ups of its busiest parts below, each with where it is. Use the \
close-ups to add details the whole view is too small to show: objects, their colours, what is on tables and shelves, text \
on signs. Describe the scene itself, not the picture's layout.

Your reply is read aloud to them, so:
- Answer in two or three short spoken sentences. No markdown, lists or emoji.
- Start with the specific kind of place (a bedroom, a kitchen, an office, a corridor, a bathroom, a living room, a street, \
a park), never just "a room". Then say what is ahead, then the useful details, saying where they are (on your left, ahead, \
on your right).
- Mention what matters for moving safely when you see it: people, obstacles, steps, doors and signs. Only mention a \
crossing light, and its colour, if one is actually in the picture; never mention one that isn't.
- Do not give distances, and do not guess how many people there are. Only mention people you can clearly see.
- Never say it is safe to cross a road. Whether to cross is the person's decision.
- Only describe what you can see. If the image is dark, blurry or blocked, say so plainly."""


def ollama_chat(body, timeout=60):
    headers = {'Content-Type': 'application/json'}
    request = urllib.request.Request(OLLAMA_URL, json.dumps(body).encode(), headers)
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.load(response)


def load_vlm(model):
    """Load the vision model onto the GPU and keep it there. True if it loaded."""
    for _ in range(30):  # Ollama may still be starting
        try:
            ollama_chat({'model': model, 'messages': [], 'keep_alive': -1, 'options': OLLAMA_OPTIONS}, timeout=300)
            return True
        except urllib.error.URLError:
            time.sleep(1)
        except TimeoutError:
            return False
    return False


def to_jpeg(image):
    return base64.standard_b64encode(cv2.imencode('.jpg', image)[1].tobytes()).decode()


def busiest_parts(frame, count=2):
    """The count most interesting parts of a 3x2 grid over the frame, as (crop, where), busiest first.
    Interesting = many edges and varied colour; bare walls, floor and ceiling score low. Crops come from the full-size
    frame with a margin, so they show more detail than the shrunk whole view."""
    h, w = frame.shape[:2]
    edges = cv2.Canny(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), 80, 160)
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    parts = []
    for row in range(2):
        for col in range(3):
            y1, y2, x1, x2 = row * h // 2, (row + 1) * h // 2, col * w // 3, (col + 1) * w // 3
            score = edges[y1:y2, x1:x2].mean() / 255 + lab[y1:y2, x1:x2, 1:].std() / 64  # edges + colour variety
            my, mx = (y2 - y1) // 6, (x2 - x1) // 6
            crop = frame[max(0, y1 - my):min(h, y2 + my), max(0, x1 - mx):min(w, x2 + mx)]
            where = ['on your left', 'straight ahead', 'on your right'][col] + (', higher up' if row == 0 else ', lower down')
            parts.append((score, crop, where))
    parts.sort(key=lambda p: -p[0])
    return [(crop, where) for _, crop, where in parts[:count]]


def collage(frame, parts):
    """One 1024 px picture: the whole view across the top, the close-ups side by side below.
    Ollama gives every image at least ~1024 tokens however small, so one collage is read about three times faster
    than three separate pictures (~3 s instead of ~8.5 s on the Jetson) and shows the same detail."""
    width, top_h, bottom_h = 1024, 576, 448
    canvas = np.full((top_h + bottom_h, width, 3), 255, np.uint8)
    canvas[:top_h] = cv2.resize(frame, (width, top_h))
    slot = width // len(parts)
    for i, (crop, _) in enumerate(parts):
        scale = min((slot - 8) / crop.shape[1], (bottom_h - 8) / crop.shape[0])
        small = cv2.resize(crop, (int(crop.shape[1] * scale), int(crop.shape[0] * scale)))
        x, y = i * slot + (slot - small.shape[1]) // 2, top_h + (bottom_h - small.shape[0]) // 2
        canvas[y:y + small.shape[0], x:x + small.shape[1]] = small
    return canvas


def describe(model, frame):
    """A spoken description of the image, from the whole view plus close-ups of its busiest parts, as one collage
    in one request. Always returns something sayable."""
    parts = busiest_parts(frame)
    parts.sort(key=lambda p: ['left', 'ahead', 'right'].index(next(s for s in ('left', 'ahead', 'right') if s in p[1])))  # left close-up on the left
    sides = ['bottom left', 'bottom right'] if len(parts) == 2 else ['bottom']
    where = ' '.join(f'The {side} part is a close-up of what is {spot}.' for side, (_, spot) in zip(sides, parts))
    prompt = f'Describe what is in front of me. The top part of the picture is the whole view. {where}'
    messages = [{'role': 'system', 'content': SYSTEM_PROMPT}, {'role': 'user', 'content': prompt, 'images': [to_jpeg(collage(frame, parts))]}]
    try:
        response = ollama_chat({'model': model, 'stream': False, 'keep_alive': -1, 'options': {**OLLAMA_OPTIONS, 'num_predict': 90},
                                'messages': messages}, timeout=90)
    except (urllib.error.URLError, TimeoutError):
        return 'I cannot reach the vision model.'
    answer = response.get('message', {}).get('content', '').replace('*', '').replace('#', '').strip()
    sentences = re.split(r'(?<=[.!?])\s+', answer)  # keep three sentences
    return ' '.join(sentences[:3]) or 'Sorry, I could not describe the scene.'


class SceneDescription(Node):
    def __init__(self):
        super().__init__('scene_description')
        param = lambda name, default: self.declare_parameter(name, default).value
        self.mode_name = param('mode_name', 'description')
        self.vlm_model = param('vlm_model', VLM_MODEL)
        self.bridge = CvBridge()
        self.mode = ''
        self.frame = None
        self.busy = threading.Event()

        self.description_pub = self.create_publisher(String, param('description_topic', '/scene_description'), 10)
        self.create_subscription(String, param('mode_topic', '/mode'), self.on_mode, 10)
        self.create_subscription(Image, param('image_topic', '/camera/color/image_raw'), self.on_image, CAMERA_QOS)

        if load_vlm(self.vlm_model):  # before YOLO starts, see the launch file
            self.get_logger().info(f'Vision model {self.vlm_model} loaded.')
        else:
            self.get_logger().warn('Could not load the vision model. Is Ollama running?')
        print('scene_description ready', flush=True)

    def on_image(self, msg):
        self.frame = msg

    def on_mode(self, msg):
        mode = msg.data.strip().lower()
        if mode == self.mode:
            return
        self.mode = mode
        if mode == self.mode_name:
            self.start_describe()

    def start_describe(self):
        if self.frame is None:
            return self.description_pub.publish(String(data='I cannot see anything yet.'))
        if self.busy.is_set():
            return
        self.busy.set()
        frame = self.bridge.imgmsg_to_cv2(self.frame, 'bgr8')

        def worker():
            try:
                text = describe(self.vlm_model, frame)
                self.get_logger().info(text)
                self.description_pub.publish(String(data=text))
            finally:
                self.busy.clear()
        threading.Thread(target=worker, daemon=True).start()


def main():
    rclpy.init()
    node = SceneDescription()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
