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

Your reply is read aloud to them, so:
- Answer in one or two short spoken sentences. No markdown, lists or emoji.
- Start with the kind of place (an office, a kitchen, a corridor, a street, a park), then what is ahead.
- Mention what matters for moving safely: people, obstacles, steps, doors, signs and the colour of crossing lights.
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


def describe(model, frame):
    """A short spoken description of the image. Always returns something sayable."""
    if frame.shape[1] > 640:  # fewer image tokens, much faster on a Jetson
        frame = cv2.resize(frame, (640, int(frame.shape[0] * 640 / frame.shape[1])))
    jpeg = base64.standard_b64encode(cv2.imencode('.jpg', frame)[1].tobytes()).decode()
    messages = [{'role': 'system', 'content': SYSTEM_PROMPT}, {'role': 'user', 'content': 'Describe what is in front of me.', 'images': [jpeg]}]
    try:
        response = ollama_chat({'model': model, 'stream': False, 'keep_alive': -1, 'options': {**OLLAMA_OPTIONS, 'num_predict': 80}, 'messages': messages})
    except (urllib.error.URLError, TimeoutError):
        return 'I cannot reach the vision model.'
    answer = response.get('message', {}).get('content', '').replace('*', '').replace('#', '').strip()
    sentences = re.split(r'(?<=[.!?])\s+', answer)  # keep two sentences
    return ' '.join(sentences[:2]) or 'Sorry, I could not describe the scene.'


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
