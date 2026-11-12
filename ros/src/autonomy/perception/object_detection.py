#!/usr/bin/env python3
"""When /mode switches to 'perception', looks for the /goal object with YOLO-World for search_time seconds, then publishes
its last location on /object_mapping and 'found:<object>' on /object_status, or 'not_found:<object>'."""

import math
import os
import random
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

YOLO_MODEL = os.environ.get('PETE_YOLO_MODEL', 'yolov8s-worldv2.pt')
CAMERA_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)  # newest frame only

# YOLO-World always gets this whole list, not just the goal: with only the goal word, anything similar
# (a cup when looking for a bottle) gets labelled as the goal. Only goal detections are used.
# The words are turned into YOLO's form once, at launch, so a long list costs almost nothing per frame (about 6% for 100
# words). Each word must be a clearly different object: near-synonyms (cup and mug) split one object's boxes between
# them, so synonyms go in ALIASES instead. Most words are YOLO's training classes (COCO), which it knows best.
VOCABULARY = [
    # people and animals
    'person', 'dog', 'cat', 'bird', 'horse',
    # furniture and the building
    'chair', 'couch', 'bed', 'table', 'bench', 'door', 'door handle', 'window', 'stairs', 'elevator', 'light switch',
    'toilet', 'sink', 'bathtub', 'shower', 'mirror', 'lamp', 'clock', 'shelf', 'cupboard', 'bin', 'plant', 'vase',
    # electronics
    'tv', 'laptop', 'keyboard', 'mouse', 'remote', 'cell phone', 'headphones', 'charger', 'printer', 'speaker',
    # kitchen
    'refrigerator', 'microwave', 'oven', 'toaster', 'kettle', 'bottle', 'can', 'cup', 'bowl', 'plate', 'fork',
    'knife', 'spoon',
    # food
    'banana', 'apple', 'orange', 'sandwich', 'pizza', 'cake',
    # personal things
    'backpack', 'handbag', 'suitcase', 'umbrella', 'keys', 'wallet', 'glasses', 'hat', 'jacket', 'shoe', 'book',
    'pen', 'scissors', 'toothbrush', 'towel', 'pillow', 'box', 'teddy bear', 'hair drier',
    # sport and play
    'sports ball', 'frisbee', 'tennis racket', 'skateboard',
    # street
    'traffic light', 'stop sign', 'fire hydrant', 'car', 'bus', 'truck', 'bicycle', 'motorcycle']
ALIASES = {
    'people': 'person', 'man': 'person', 'woman': 'person', 'child': 'person', 'human': 'person', 'puppy': 'dog',
    'kitten': 'cat',
    'seat': 'chair', 'armchair': 'chair', 'stool': 'chair', 'sofa': 'couch', 'settee': 'couch', 'desk': 'table',
    'dining table': 'table', 'doorway': 'door', 'handle': 'door handle', 'door knob': 'door handle',
    'doorknob': 'door handle', 'steps': 'stairs', 'staircase': 'stairs', 'lift': 'elevator', 'switch': 'light switch',
    'loo': 'toilet', 'bath': 'bathtub', 'washbasin': 'sink', 'basin': 'sink', 'light': 'lamp', 'bookshelf': 'shelf',
    'cabinet': 'cupboard', 'wardrobe': 'cupboard', 'trash can': 'bin', 'rubbish bin': 'bin',
    'potted plant': 'plant', 'flower pot': 'plant', 'flowers': 'vase',
    'television': 'tv', 'television set': 'tv', 'monitor': 'tv', 'screen': 'tv', 'computer screen': 'tv',
    'computer': 'laptop', 'remote control': 'remote', 'tv remote': 'remote', 'phone': 'cell phone',
    'mobile phone': 'cell phone', 'mobile': 'cell phone', 'smartphone': 'cell phone', 'earphones': 'headphones',
    'headset': 'headphones', 'cable': 'charger',
    'fridge': 'refrigerator', 'water bottle': 'bottle', 'mug': 'cup', 'glass': 'cup',
    'wine glass': 'cup', 'dish': 'plate',
    'rucksack': 'backpack', 'bag': 'handbag', 'purse': 'handbag', 'luggage': 'suitcase', 'key': 'keys',
    'sunglasses': 'glasses', 'spectacles': 'glasses', 'cap': 'hat', 'coat': 'jacket', 'shoes': 'shoe', 'trainers': 'shoe',
    'pencil': 'pen', 'cushion': 'pillow', 'package': 'box', 'parcel': 'box', 'hair dryer': 'hair drier',
    'ball': 'sports ball', 'football': 'sports ball', 'soccer ball': 'sports ball', 'basketball': 'sports ball',
    'tennis ball': 'sports ball', 'rugby ball': 'sports ball',
    'crossing light': 'traffic light', 'pedestrian light': 'traffic light', 'bike': 'bicycle', 'motorbike': 'motorcycle',
    'van': 'truck', 'lorry': 'truck'}
PLURAL = {'stairs', 'keys', 'glasses', 'headphones', 'scissors'}


def normalise(name):
    """The vocabulary's word for what was asked for: 'sofa' -> 'couch', 'chairs' -> 'chair'."""
    name = ALIASES.get(name, name)
    if name not in PLURAL and name not in VOCABULARY:
        for ending in ('es', 's'):  # 'buses' -> 'bus', 'chairs' -> 'chair'
            if name.endswith(ending) and ALIASES.get(name[:-len(ending)], name[:-len(ending)]) in VOCABULARY:
                return ALIASES.get(name[:-len(ending)], name[:-len(ending)])
    return name


def depth_at(box, depth, scale):
    """Median depth (m) over the middle half of the box, ignoring holes. None if there is too little.
    Only this patch of the depth image is read; scale turns its values into meters."""
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    patch = depth[int(y1 + h / 4):int(y2 - h / 4) + 1, int(x1 + w / 4):int(x2 - w / 4) + 1]
    valid = patch[patch > 0]
    return float(np.median(valid)) * scale if valid.size > 20 else None


class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')
        param = lambda name, default: self.declare_parameter(name, default).value
        self.mode_name = param('mode_name', 'perception')
        self.search_time = param('search_time', 5.0)  # YOLO runs this long per search, then the last location is sent
        self.confidence = param('confidence', 0.25)
        # YOLO's input width. At 640 a whole 1280 px frame loses half its detail, so with tiles on, every other frame
        # YOLO looks at one quarter of the frame instead, at full detail, for the same cost (~37 ms on the Jetson).
        # Each quarter gets a full-detail look about every 8 frames, which finds small or far objects.
        self.image_size = param('image_size', 640)
        self.tiles = param('tiles', True)
        self.tile_reach = param('tile_reach', 0.15)  # how far past the middle a quarter may reach, as a share of the frame height
        self.frame_id = param('frame_id', 'camera_link')
        self.confirm_frames = param('confirm_frames', 3)  # sightings of the same object before 'found', so one-off flukes are ignored
        # Once a goal object is picked (the closest), stick to it: each frame take the detection nearest to where it was,
        # within track_gate meters, so it doesn't jump between two chairs. Lost for lost_after seconds: pick again.
        self.track_gate = param('track_gate', 0.5)
        self.lost_after = param('lost_after', 1.0)
        self.target = None  # (x, y, z) of the object being followed
        self.target_seen = 0.0

        from ultralytics import YOLOWorld
        self.model = YOLOWorld(param('model_path', YOLO_MODEL))
        self.vocabulary = list(VOCABULARY)
        self.model.set_classes(self.vocabulary)
        # YOLO-World compares image regions with each word turned into numbers by a text model (CLIP). Keep the list's
        # numbers from launch: going back to them is instant, and a new word only needs its own numbers worked out.
        self.list_features = self.model.model.txt_feats.clone()
        self.wanted = list(VOCABULARY)  # the list the goal needs; differs from vocabulary while a new word is learnt
        self.learning = False
        # Warm up: the first prediction sets up the GPU code (seconds on the Jetson). Do it now, not on the first search.
        # Once for a whole frame and once for a quarter, as each shape gets its own setup.
        for h, w in ((720, 1280), (396, 704)):
            self.model.predict(np.zeros((h, w, 3), np.uint8), imgsz=self.image_size, verbose=False)
        self.hits = 0  # sightings of the followed object
        self.look = 0  # counts YOLO looks: even = whole frame, odd = the next quarter
        self.bridge = CvBridge()
        self.mode = ''
        self.goal = ''
        self.started = None  # None when not searching
        self.last_point = None  # the followed object's last confirmed location in this search
        self.depth_msg = None  # the newest depth image, only read once YOLO has a goal box
        self.intrinsics = None  # fx, fy, cx, cy

        self.point_pub = self.create_publisher(PointStamped, param('object_mapping_topic', '/object_mapping'), 10)
        self.status_pub = self.create_publisher(String, param('status_topic', '/object_status'), 10)
        # What YOLO sees, with its boxes drawn on: a small jpeg, only while searching
        self.preview_pub = self.create_publisher(CompressedImage, param('preview_topic', '/object_detection/image/compressed'), CAMERA_QOS)
        self.create_subscription(String, param('mode_topic', '/mode'), self.on_mode, 10)
        self.create_subscription(String, param('goal_topic', '/goal'), self.on_goal, 10)
        self.create_subscription(Image, param('image_topic', '/camera/color/image_raw'), self.on_image, CAMERA_QOS)
        self.create_subscription(Image, param('depth_topic', '/camera/depth_registered/image_rect'), self.on_depth, CAMERA_QOS)
        self.create_subscription(CameraInfo, param('camera_info_topic', '/camera/color/camera_info'), self.on_info, CAMERA_QOS)
        self.get_logger().info(f"Object detection ready. It searches while the mode is '{self.mode_name}'.")

    def on_mode(self, msg):
        mode = msg.data.strip().lower()
        if mode == self.mode:
            return
        self.mode = mode
        if mode == self.mode_name:
            self.start_search()
        else:
            self.started = None

    def on_goal(self, msg):
        self.goal = normalise(msg.data.strip().lower())
        if self.goal:
            # Any other word still works. Words inside it are left out, or 'chair' would win every 'armchair' box.
            self.wanted = VOCABULARY if self.goal in VOCABULARY else [w for w in VOCABULARY if w not in self.goal] + [self.goal]
            if self.goal in VOCABULARY and not self.learning:
                self.use_words(VOCABULARY)  # instant: the list's numbers are kept from launch
            elif self.wanted != self.vocabulary and not self.learning:
                self.learning = True
                threading.Thread(target=self.learn, daemon=True).start()
        if self.mode == self.mode_name:
            self.start_search()  # also when the same goal is sent again

    def learn(self):
        """Work out a new word's numbers. Loading the text model takes seconds on the Jetson, so this runs in the
        background and the node keeps answering; the search waits for it and its time starts once it's done."""
        while self.wanted != self.vocabulary:  # again if another goal came in meanwhile
            wanted = list(self.wanted)
            if wanted[-1] not in VOCABULARY:
                self.get_logger().info(f"'{wanted[-1]}' isn't in the word list; adding it.")
            self.use_words(wanted)
        self.learning = False
        if self.mode == self.mode_name:
            self.start_search()

    def use_words(self, words):
        """Point YOLO at a word list: list words reuse their numbers from launch, any other word is worked out now."""
        if words == self.vocabulary:
            return
        import torch
        parts = [self.list_features[:, [VOCABULARY.index(w) for w in words if w in VOCABULARY]]]
        new = [w for w in words if w not in VOCABULARY]
        if new:
            parts.append(self.model.model.get_text_pe(new).to(self.list_features))
        world = self.model.model
        world.txt_feats = torch.cat(parts, dim=1)
        world.model[-1].nc = len(words)
        self.model.model.names = list(words)
        self.model.predictor = None  # rebuilt on the next prediction, with the new number of words
        self.vocabulary = list(words)

    def start_search(self):
        if not self.goal:
            return
        self.started, self.last_point, self.hits, self.target, self.look = time.monotonic(), None, 0, None, 0
        if not self.learning:
            self.get_logger().info(f'Looking for a {self.goal}.')

    def on_info(self, msg):
        self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def on_depth(self, msg):
        self.depth_msg = msg  # kept as it is: read only once YOLO has a box, and then only inside it

    def on_image(self, msg):
        if self.started is None or self.learning:
            return  # YOLO stays idle, or the word list is being changed
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        point, boxes, chosen, region = self.locate(frame)
        self.publish_preview(frame, boxes, chosen, point, region)
        if point is not None and self.hits >= self.confirm_frames:
            point.header.stamp = msg.header.stamp
            self.last_point = point  # where it was last seen, sent when the search ends
        if time.monotonic() - self.started >= self.search_time:
            self.finish()

    def finish(self):
        """End of the search: send the object's last location once, or that it wasn't found. YOLO then stops."""
        if self.last_point is not None:
            p = self.last_point.point
            self.point_pub.publish(self.last_point)
            self.status_pub.publish(String(data=f'found:{self.goal}'))
            self.get_logger().info(f'Found the {self.goal}, last seen at x={p.x:.2f} m, y={p.y:.2f} m.')
        else:
            self.status_pub.publish(String(data=f'not_found:{self.goal}'))
            self.get_logger().info(f'No {self.goal} found in {self.search_time:g} s.')
        self.started = None

    def next_region(self, shape):
        """(x1, y1, x2, y2) of the frame YOLO looks at next: the whole frame, then a quarter, alternately.
        A quarter runs from one corner of the frame to a random point just past the middle (within tile_reach of it),
        so the four always cover the frame but their borders move, and an object on a border isn't always split."""
        h, w = shape[:2]
        self.look += 1
        if not self.tiles or self.look % 2:
            return 0, 0, w, h
        corner = (self.look // 2) % 4  # top left, top right, bottom left, bottom right
        reach = random.uniform(0.3, 1.0) * self.tile_reach * min(w, h)
        angle = random.uniform(0, math.pi / 2)
        dx, dy = reach * math.cos(angle), reach * math.sin(angle)  # how far past the middle, towards the far side
        x1, x2 = (0, w / 2 + dx) if corner in (0, 2) else (w / 2 - dx, w)
        y1, y2 = (0, h / 2 + dy) if corner in (0, 1) else (h / 2 - dy, h)
        return int(x1), int(y1), int(x2), int(y2)

    def locate(self, frame):
        """(PointStamped of the followed object or None, goal boxes as (xyxy, conf), the chosen box, the region looked at).
        YOLO runs first; depth is only read when it has found the goal, and then only inside the boxes."""
        region = self.next_region(frame.shape)
        x0, y0, x1, y1 = region
        result = self.model.predict(frame[y0:y1, x0:x1], imgsz=self.image_size, conf=self.confidence, verbose=False)[0]
        goal = self.vocabulary.index(self.goal)
        boxes = sorted((([b[0] + x0, b[1] + y0, b[2] + x0, b[3] + y0], p)  # back to whole-frame pixels
                        for b, p, c in zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist(), result.boxes.cls.tolist())
                        if int(c) == goal), key=lambda b: -b[1])  # the goal only; other words just compete

        candidates = []  # (position, box) of every goal detection with a depth reading
        if boxes:
            if self.depth_msg is None or self.intrinsics is None:
                self.get_logger().warn('Waiting for the depth image and camera info.', throttle_duration_sec=5.0)
                return None, boxes, None, region
            depth = self.bridge.imgmsg_to_cv2(self.depth_msg, 'passthrough')  # a view of the message, nothing converted yet
            if depth.shape[:2] != frame.shape[:2]:
                self.get_logger().warn('Depth is not aligned to colour; turn on depth registration in the camera driver.', throttle_duration_sec=5.0)
                return None, boxes, None, region
            scale = 0.001 if self.depth_msg.encoding == '16UC1' else 1.0  # 16UC1 is mm, 32FC1 m
            fx, fy, cx, cy = self.intrinsics
        for box, _ in boxes:
            z = depth_at(box, depth, scale)
            if z is not None:
                u, v = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
                candidates.append((np.array([z, -(u - cx) * z / fx, -(v - cy) * z / fy]), box))  # optical frame to x forward, y left, z up

        now = time.monotonic()
        if self.target is not None and now - self.target_seen > self.lost_after:
            self.target = None  # lost it: pick again
        if self.target is None:
            pick = min(candidates, key=lambda c: np.linalg.norm(c[0]), default=None)  # the closest one
            self.hits = 0
        else:
            pick = min(candidates, key=lambda c: np.linalg.norm(c[0] - self.target), default=None)  # the same one as before
            if pick is not None and np.linalg.norm(pick[0] - self.target) > self.track_gate:
                pick = None  # only other objects in view: don't swap to them
        if pick is None:
            return None, boxes, None, region  # a look that missed it (e.g. another quarter) doesn't undo the sightings

        self.target, self.target_seen, self.hits = pick[0], now, self.hits + 1
        point = PointStamped()
        point.header.frame_id = self.frame_id
        point.point.x, point.point.y, point.point.z = (float(c) for c in pick[0])
        return point, boxes, pick[1], region

    def publish_preview(self, frame, boxes, chosen, point, region):
        """Green box: the detection sent to navigation. Grey: other detections (no depth reading, or less confident).
        Blue outline: the quarter YOLO looked at in full detail this frame."""
        scale = 640 / frame.shape[1]
        image = cv2.resize(frame, (640, int(frame.shape[0] * scale)))
        if region is not None and region != (0, 0, frame.shape[1], frame.shape[0]):
            cv2.rectangle(image, *[tuple(int(c * scale) for c in corner) for corner in (region[:2], region[2:])], (255, 140, 0), 1)
        for box, conf in boxes:
            x1, y1, x2, y2 = (int(c * scale) for c in box)
            is_chosen = box is chosen
            label = f'{self.goal} {point.point.x:.1f} m' if is_chosen else f'{self.goal} {conf:.2f}'
            colour = (0, 200, 0) if is_chosen else (160, 160, 160)
            cv2.rectangle(image, (x1, y1), (x2, y2), colour, 2)
            cv2.putText(image, label, (x1, max(y1 - 6, 14)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)
        cv2.putText(image, f'YOLO: looking for {self.goal}', (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 2)
        self.preview_pub.publish(CompressedImage(format='jpeg', data=cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 60])[1].tobytes()))


def main():
    rclpy.init()
    node = ObjectDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
