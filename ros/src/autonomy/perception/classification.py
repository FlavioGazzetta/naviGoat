"""Pete: finds objects, guides the user to them and describes what the camera sees.

Requests come in as text and replies go out as text.

    "what's in front of me"  -> the vision model describes the camera image
    "where is the remote"    -> "The remote is at 2 o'clock, to your right, about 1.5 meters away."
    "bring me to the chair"  -> drives to the chair (only with enable_drive:=true)
    "stop"                   -> stops
    anything else            -> the model works out what is wanted ("I'd like a coffee" -> find cup),
                                or answers it as a question about the image

The vision model runs locally through Ollama (https://ollama.com):
    curl -fsSL https://ollama.com/install.sh | sh
    ollama pull qwen3-vl:2b-instruct   # 1.9 GB; qwen3-vl:4b-instruct is better but tight on an 8 GB Orin Nano
Use the -instruct tags: the plain tags are thinking models that reason before answering, which is too slow here.

ROS 2, inside the repo's container built with PETE=true (Ollama and the model are in the image):
    python3 /workspace/ros/src/autonomy/perception/classification.py --ros-args -p use_webcam:=true -p enable_drive:=true

Driving publishes to /cmd_vel, which is what p2os_driver listens to on the real robot.
In Gazebo add -p cmd_vel_topic:=/model/pioneer3at/cmd_vel.
    ros2 topic pub --once /pete/command std_msgs/msg/String "{data: 'where is the chair'}"
    ros2 topic echo /pete/response

Without ROS (type requests, replies are printed):
    python3 classification.py --terminal --show   (q in the window or Ctrl+D quits)

Guidance only steers towards the target. It does not avoid obstacles, so keep
driving off until a sonar/lidar stop layer is in place.
"""

import argparse
import base64
import json
import math
import os
import queue
import re
import sys
import threading
import time
import urllib.error
import urllib.request
import warnings

import cv2

# Harmless on the Jetson: the sm_80 build runs fine on the Orin.
warnings.filterwarnings('ignore', message='Found GPU0 Orin')
warnings.filterwarnings('ignore', message='`torch.jit.load` is deprecated')

OLLAMA_URL = 'http://localhost:11434/api/chat'
VLM_MODEL = 'qwen3-vl:2b-instruct'
YOLO_MODEL = os.environ.get('PETE_YOLO_MODEL', 'yolov8s-worldv2.pt')

WAKE_WORDS = ('hey pete', 'ok pete', 'pete')

VOCABULARY = ['person', 'chair', 'couch', 'bed', 'table', 'door', 'tv', 'remote', 'laptop', 'cell phone', 'bottle', 'cup', 'book', 'backpack', 'keys', 'wallet', 'glasses', 'shoe', 'refrigerator', 'sink', 'toilet', 'stairs', 'dog', 'cat']
ALIASES = {'television': 'tv', 'remote control': 'remote', 'tv remote': 'remote', 'sofa': 'couch', 'phone': 'cell phone', 'fridge': 'refrigerator', 'desk': 'table', 'key': 'keys', 'steps': 'stairs'}

# Rough real heights (m), used to guess distance from box size.
HEIGHTS_M = {'person': 1.7, 'chair': 0.9, 'couch': 0.85, 'bed': 0.6, 'table': 0.75, 'door': 2.0, 'tv': 0.6, 'remote': 0.18, 'laptop': 0.25, 'cell phone': 0.15, 'bottle': 0.25, 'cup': 0.1, 'book': 0.22, 'backpack': 0.45, 'refrigerator': 1.7, 'sink': 0.3, 'toilet': 0.75}

SYSTEM_PROMPT = """You are Pete, a guide robot for a blind or low-vision person. \
You see through your forward-facing camera, which points the way the person is heading.

Your reply is read aloud to them, so:
- Answer in one or two short spoken sentences. Answer only what was asked. No markdown, lists or emoji.
- Mention what matters for moving safely first: people, obstacles, steps, doors and anything in the path.
- Do not give distances, directions or clock positions. Pete measures those itself and adds them after your answer.
- Only describe what you can see. If the image is dark, blurry or blocked, say so plainly.

You may be given a list of objects a detector found. It can be wrong, so trust the image when they disagree."""


# --- Understanding requests -------------------------------------------------

_STOP = re.compile(r"^(please )?(stop|halt|cancel|wait|freeze)\b")
_GUIDE = re.compile(r"\b(bring|take|lead|guide|walk|go)( me)? (to|towards)\b(?P<target>.*)")
_FIND = re.compile(r"\b(where's|where is|where are|find|locate|look for|is there|do you see)\b(?P<target>.*)")
_DESCRIBE = re.compile(r"\b(what's|what is|whats|describe)\b.*\b(in front|ahead|around|happening|room)\b|\bwhat (do|can) you see\b")
_FILLER = re.compile(r"^(the|a|an|my|any|nearest) | (please|in front of me|around me|here|in the room)$")


def parse_command(text, require_wake_word=False):
    """Return (intent, target, text) or None."""
    text = re.sub(r"[^a-z0-9' ]+", ' ', text.lower().replace('’', "'"))
    text = ' '.join(text.split())
    for word in WAKE_WORDS:  # longest first
        match = re.search(rf'\b{word}\b', text)
        if match:
            text = text[match.end():].strip()
            break
    else:
        if require_wake_word:
            return None
    if not text:
        return None

    if _STOP.search(text):
        return 'stop', None, text
    for intent, pattern in (('guide', _GUIDE), ('find', _FIND)):
        match = pattern.search(text)
        if match:
            target = match.group('target').strip()
            while (stripped := _FILLER.sub('', target).strip()) != target:
                target = stripped
            if target and target not in ('it', 'that', 'this'):
                return intent, target, text
    if _DESCRIBE.search(text):
        return 'describe', None, text
    return 'ask', None, text


# --- Seeing -----------------------------------------------------------------

class Detector:
    """YOLO-World, so any object name can be searched for."""

    def __init__(self, model_path=YOLO_MODEL, confidence=0.25):
        from ultralytics import YOLOWorld
        self.model = YOLOWorld(model_path)
        self.confidence = confidence
        self.vocabulary = list(VOCABULARY)
        self.model.set_classes(self.vocabulary)

    def add(self, name):
        name = ALIASES.get(name, name)
        if name.endswith('s') and name[:-1] in self.vocabulary:
            name = name[:-1]  # "chairs" -> "chair"
        if name not in self.vocabulary:
            self.vocabulary.append(name)
            self.model.set_classes(self.vocabulary)
        return name

    def detect(self, frame):
        """Return [(label, confidence, (x1, y1, x2, y2)), ...]."""
        result = self.model.predict(frame, conf=self.confidence, verbose=False)[0]
        boxes = result.boxes
        return [(result.names[int(c)], float(p), tuple(b)) for b, p, c in zip(boxes.xyxy.tolist(), boxes.conf.tolist(), boxes.cls.tolist())]


def ollama_chat(body, timeout=60):
    headers = {'Content-Type': 'application/json'}
    request = urllib.request.Request(OLLAMA_URL, json.dumps(body).encode(), headers)
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.load(response)


def describe_scene(model, frame, question, hint):
    """Ask the local vision model about the image. Always returns something sayable."""
    # Smaller images mean fewer image tokens, which matters a lot on a Jetson.
    if frame.shape[1] > 640:
        frame = cv2.resize(frame, (640, int(frame.shape[0] * 640 / frame.shape[1])))
    jpeg = base64.standard_b64encode(cv2.imencode('.jpg', frame)[1].tobytes()).decode()
    text = question or "What's in front of me?"
    if hint:
        text = f'Objects the detector found: {hint}\n\nQuestion: {text}'
    try:
        messages = [{'role': 'system', 'content': SYSTEM_PROMPT}, {'role': 'user', 'content': text, 'images': [jpeg]}]
        # keep_alive -1 keeps the model loaded so the next question is fast.
        response = ollama_chat({'model': model, 'stream': False, 'keep_alive': -1, 'options': {'num_ctx': 4096, 'num_predict': 80}, 'messages': messages})
    except urllib.error.URLError:
        return 'I cannot reach the vision model. Is Ollama running?'
    except TimeoutError:
        return 'The vision model took too long to answer.'
    answer = response.get('message', {}).get('content', '').replace('*', '').replace('#', '').strip()
    # Small models ramble; keep the first two sentences.
    sentences = re.split(r'(?<=[.!?])\s+', answer)
    return ' '.join(sentences[:2]) or 'Sorry, I did not get a description.'


INTERPRET_PROMPT = """You turn what a blind person says to Pete, their guide robot, into a command.

intent is one of:
- find: they want something or want to know where it is ("I'd like a coffee", "I'm looking for my keys")
- guide: they want to be taken to something ("take me somewhere to sit")
- describe: they want to know what is around them
- question: any other question about what is in view ("is the light on?")

object is the thing in the room that gives them what they want: coffee or a drink -> cup, somewhere to sit -> chair, the way out -> door, something to watch -> tv. Use "" for describe and question."""

# Only allow objects the detector knows, so "coffee" has to become "cup".
INTERPRET_SCHEMA = {'type': 'object', 'properties': {'intent': {'enum': ['find', 'guide', 'describe', 'question']}, 'object': {'enum': VOCABULARY + ['']}}, 'required': ['intent', 'object']}


def interpret(model, text):
    """Ask the model what an unrecognised request means. Returns (intent, object) or None."""
    messages = [{'role': 'system', 'content': INTERPRET_PROMPT}, {'role': 'user', 'content': text}]
    try:
        response = ollama_chat({'model': model, 'stream': False, 'keep_alive': -1, 'format': INTERPRET_SCHEMA, 'options': {'num_predict': 40}, 'messages': messages})
        result = json.loads(response['message']['content'])
    except (urllib.error.URLError, TimeoutError, KeyError, ValueError):
        return None
    intent, target = result.get('intent'), result.get('object', '').strip().lower()
    if intent in ('find', 'guide') and not target:
        intent = 'question'
    return intent, target or None


# --- Where things are -------------------------------------------------------

def bearing_deg(box, width, hfov):
    """Angle to the box centre, positive to the right."""
    focal = (width / 2) / math.tan(math.radians(hfov) / 2)
    return math.degrees(math.atan2((box[0] + box[2]) / 2 - width / 2, focal))


def cut_off(box, shape):
    """True if the box touches the top or bottom of the picture, so its height is not the object's full height."""
    return box[1] <= 3 or box[3] >= shape[0] - 3


def distance_m(label, box, shape, hfov):
    """Guess the distance from how tall the object looks. None if we can't."""
    focal = (shape[1] / 2) / math.tan(math.radians(hfov) / 2)
    height = box[3] - box[1]
    if label not in HEIGHTS_M or height <= 1 or cut_off(box, shape):
        return None
    return HEIGHTS_M[label] * focal / height


def where_is(label, box, shape, hfov):
    bearing = bearing_deg(box, shape[1], hfov)
    if abs(bearing) < 8:
        direction = 'straight ahead'
    else:
        hour = round(bearing / 30) % 12 or 12
        direction = f"at {hour} o'clock, to your {'right' if bearing > 0 else 'left'}"
    distance = distance_m(label, box, shape, hfov)
    if distance is not None:
        return f'The {label} is {direction}, {meters(distance)} away.'
    if cut_off(box, shape):
        return f'The {label} is {direction}, partly out of view, so I cannot tell how far.'
    return f'The {label} is {direction}.'


def meters(d):
    return 'less than a meter' if d < 1 else f'about {round(d * 2) / 2:g} meters'


# --- Pete -------------------------------------------------------------------

class Pete:
    """Feed frames to step() and text to submit(); replies go to reply(text)
    and, if driving is enabled, velocities to drive(linear m/s, angular rad/s)."""

    MAX_LINEAR = 0.25
    MAX_ANGULAR = 0.6
    ARRIVE_M = 0.8
    SEARCH_TIMEOUT_S = 30
    FIND_WINDOW_S = 1.5

    def __init__(self, detector, reply, drive=None, hfov=60.0, require_wake_word=False, vlm_model=VLM_MODEL):
        self.detector = detector
        self.vlm_model = vlm_model
        self.reply = reply
        self.drive = drive
        self.hfov = hfov
        self.require_wake_word = require_wake_word
        self.inbox = queue.Queue()
        self.task = None  # ('find' | 'guide', label, started)
        self.last_seen = None
        self.last_bearing = 0.0
        self.describing = threading.Event()
        # Load the vision model now rather than on the first question.
        threading.Thread(target=self.warm_up, daemon=True).start()

    def warm_up(self):
        try:
            ollama_chat({'model': self.vlm_model, 'messages': [], 'keep_alive': -1}, timeout=300)
        except (urllib.error.URLError, TimeoutError):
            pass  # describe_scene will tell the user if it is still unreachable

    def submit(self, text):
        command = parse_command(text, self.require_wake_word)
        if command:
            self.inbox.put(command)

    def stop(self):
        self.task = None
        if self.drive:
            self.drive(0.0, 0.0)

    def step(self, frame, now=None):
        now = time.monotonic() if now is None else now
        detections = self.detector.detect(frame)
        while not self.inbox.empty():
            self.handle(*self.inbox.get_nowait(), frame, detections, now)
        if self.task:
            kind, label, started = self.task
            boxes = [b for name, _, b in detections if name == label]
            if kind == 'find':
                self.update_find(label, boxes, frame.shape, now - started)
            else:
                self.update_guide(label, boxes, frame.shape, now, started)
        return detections

    def handle(self, intent, target, text, frame, detections, now):
        if intent == 'stop':
            self.stop()
            self.reply('Stopped.')
        elif intent == 'describe':
            self.start_describe(frame, None, detections)
        elif intent == 'question':
            self.start_describe(frame, text, detections)
        elif intent == 'ask':
            self.start_interpret(text)
        else:
            label = self.detector.add(target)
            if intent == 'guide' and self.drive:
                self.reply(f'Looking for the {label}.')
            else:
                intent = 'find'
            self.task = (intent, label, now)
            self.last_seen, self.last_bearing = None, 0.0

    def update_find(self, label, boxes, shape, elapsed):
        if boxes:
            self.reply(where_is(label, boxes[0], shape, self.hfov))
            self.task = None
        elif elapsed > self.FIND_WINDOW_S:
            self.reply(f'I do not see a {label} in front of you.')
            self.task = None

    def update_guide(self, label, boxes, shape, now, started):
        height, width = shape[:2]
        if not boxes:
            if self.last_seen and now - self.last_seen < 1.0:
                return self.drive(0.0, 0.0)  # probably a missed detection
            if now - (self.last_seen or started) > self.SEARCH_TIMEOUT_S:
                self.stop()
                return self.reply(f"I couldn't find a {label}. I have stopped.")
            # Turn on the spot towards where it was last seen.
            return self.drive(0.0, 0.35 if self.last_bearing < 0 else -0.35)

        box = min(boxes, key=lambda b: abs(bearing_deg(b, width, self.hfov) - self.last_bearing))
        bearing = bearing_deg(box, width, self.hfov)
        distance = distance_m(label, box, shape, self.hfov)
        if self.last_seen is None:
            self.reply(f'I see it. {where_is(label, box, shape, self.hfov)} Follow me.')
        self.last_seen, self.last_bearing = now, bearing

        close = (distance is not None and distance < self.ARRIVE_M) or (box[3] - box[1]) > 0.6 * height
        if close and abs(bearing) < 20:
            self.stop()
            return self.reply(f'We have reached the {label}. It is right in front of you.')

        angular = max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, -1.5 * math.radians(bearing)))
        # Turn in place if well off to the side, otherwise drive and steer.
        linear = 0.0 if abs(bearing) > 20 else self.MAX_LINEAR * math.cos(math.radians(bearing) * 4.5)
        self.drive(linear, angular)

    def start_interpret(self, text):
        def worker():  # the model takes a moment; the answer goes back through the inbox so step() acts on a fresh frame
            intent, target = interpret(self.vlm_model, text) or ('question', None)
            self.inbox.put((intent, target, text))
        threading.Thread(target=worker, daemon=True).start()

    def start_describe(self, frame, question, detections):
        if self.describing.is_set():
            return self.reply('I am still looking, one moment.')
        self.describing.set()
        hint = ', '.join(sorted({label for label, _, _ in detections}))
        # The vision model says what is there; positions come from the detector, nearest first.
        positions = ''
        if question is None:
            nearest = sorted(detections, key=lambda d: distance_m(d[0], d[2], frame.shape, self.hfov) or 99)[:2]
            positions = ' ' + ' '.join(where_is(label, box, frame.shape, self.hfov) for label, _, box in nearest)

        def worker():  # the vision model takes a few seconds; don't hold up guidance
            try:
                self.reply((describe_scene(self.vlm_model, frame.copy(), question, hint) + positions).strip())
            finally:
                self.describing.clear()
        threading.Thread(target=worker, daemon=True).start()


# --- Running ----------------------------------------------------------------

def draw(frame, detections, target):
    """Box every detection; the object Pete is looking for is red, the rest green."""
    for label, p, box in detections:
        x1, y1, x2, y2 = (int(v) for v in box)
        colour = (0, 0, 255) if label == target else (0, 200, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
        cv2.putText(frame, f'{label} {p:.0%}', (x1, max(15, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)
    return frame


def run_terminal(camera, hfov, vlm_model, show):
    capture = cv2.VideoCapture(int(camera) if camera.isdigit() else camera)
    if not capture.isOpened():
        sys.exit(f'Could not open camera {camera}. Is another program (or another copy of Pete) using it?')
    reply = lambda text: print(f'\n[pete] {text}\n> ', end='', flush=True)
    pete = Pete(Detector(), reply, hfov=hfov, vlm_model=vlm_model)

    def read_stdin():
        for line in sys.stdin:
            pete.submit(line)
        capture.release()
    threading.Thread(target=read_stdin, daemon=True).start()

    print('Type a request (Ctrl+D to quit).\n> ', end='', flush=True)
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        detections = pete.step(frame)
        if show:
            # Draw on a copy: the vision model must see the clean frame.
            cv2.imshow('pete', draw(frame.copy(), detections, pete.task[1] if pete.task else None))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    cv2.destroyAllWindows()


def run_ros():
    import rclpy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node('pete')
    param = lambda name, default: node.declare_parameter(name, default).value
    use_webcam = param('use_webcam', False)
    enable_drive = param('enable_drive', False)
    bridge = CvBridge()

    response_pub = node.create_publisher(String, '/pete/response', 10)
    cmd_pub = node.create_publisher(Twist, param('cmd_vel_topic', '/cmd_vel'), 10)

    def reply(text):
        node.get_logger().info(text)
        response_pub.publish(String(data=text))

    def drive(linear, angular):
        twist = Twist()
        twist.linear.x, twist.angular.z = float(linear), float(angular)
        cmd_pub.publish(twist)

    detector = Detector(param('model_path', YOLO_MODEL))
    pete = Pete(detector, reply, drive if enable_drive else None, param('hfov_deg', 60.0), param('require_wake_word', False), param('vlm_model', VLM_MODEL))
    node.create_subscription(String, '/pete/command', lambda msg: pete.submit(msg.data), 10)

    latest = {}
    if use_webcam:
        capture = cv2.VideoCapture(param('webcam_index', 0))
        if not capture.isOpened():
            sys.exit('Could not open the webcam. Is another program (or another copy of Pete) using it?')
    else:
        on_image = lambda msg: latest.update(frame=bridge.imgmsg_to_cv2(msg, 'bgr8'), time=time.monotonic())
        node.create_subscription(Image, param('image_topic', '/camera/image_raw'), on_image, 1)

    def tick():
        if use_webcam:
            ok, frame = capture.read()
            frame = frame if ok else None
        else:
            frame = latest.pop('frame', None)
            if frame is None and pete.task and time.monotonic() - latest.get('time', 0) > 0.5:
                pete.stop()  # camera went quiet: don't drive blind
        if frame is not None:
            pete.step(frame)

    node.create_timer(0.1, tick)
    node.get_logger().info(f"Pete is ready. Driving is {'ON' if enable_drive else 'off'}.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        drive(0.0, 0.0)
        rclpy.try_shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--terminal', action='store_true', help='run without ROS')
    parser.add_argument('--camera', default='0')
    parser.add_argument('--hfov', type=float, default=60.0, help='camera horizontal field of view')
    parser.add_argument('--vlm', default=VLM_MODEL, help='Ollama vision model')
    parser.add_argument('--show', action='store_true', help='show the camera with detection boxes (terminal mode)')
    args, _ = parser.parse_known_args()  # leave --ros-args for ROS
    run_terminal(args.camera, args.hfov, args.vlm, args.show) if args.terminal else run_ros()
