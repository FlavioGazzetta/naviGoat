#!/usr/bin/env python3
"""When /mode switches to 'perception', looks for the /goal object with YOLO-World and publishes where it was last seen."""

import collections
import math
import os
import random
import sys
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

# The word lists are shared with the middleman (core/catalog.py), so both always know the same objects
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'core'))
from catalog import OBJECT_ALIASES as ALIASES, VOCABULARY, WORN  # noqa: E402
KNOWN = VOCABULARY + WORN  # every word whose numbers are worked out at launch
PLURAL = {'stairs', 'keys', 'glasses', 'headphones', 'scissors'}


def normalise(name):
    """The vocabulary's word for what was asked for: 'sofa' -> 'couch', 'chairs' -> 'chair'."""
    name = ALIASES.get(name, name)
    if name not in PLURAL and name not in KNOWN:
        for ending in ('es', 's'):  # 'buses' -> 'bus', 'chairs' -> 'chair'
            if name.endswith(ending) and ALIASES.get(name[:-len(ending)], name[:-len(ending)]) in KNOWN:
                return ALIASES.get(name[:-len(ending)], name[:-len(ending)])
    return name


def depth_at(box, depth, scale):
    """Median depth (m) over the middle half of the box, ignoring holes. None if there is too little."""
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
        # Not lower: tested on COCO128, 0.15 found 2 more of 18 bottles but called 5 more other things 'bottle'.
        self.confidence = param('confidence', 0.25)
        self.image_size = param('image_size', 640)
        self.tiles = param('tiles', True)
        self.tile_reach = param('tile_reach', 0.15)  # how far past the middle a quarter may reach, as a share of the frame height
        self.frame_id = param('frame_id', 'camera_link')
        self.confirm_frames = param('confirm_frames', 3)  # sightings of the same object before 'found', so one-off flukes are ignored
        self.track_gate = param('track_gate', 0.5)
        self.lost_after = param('lost_after', 1.0)
        self.lost_looks = param('lost_looks', 6)
        self.prev_target = None  # where the last followed object was lost, to recognise it when it is seen again
        self.close_hits = 0  # sightings of the followed object in the zoomed close-up around it
        self.misses = 0
        self.focused = False  # whether this look was a crop around the followed object
        self.target = None  # (x, y, z) of the object being followed
        self.target_box = None  # its last box in pixels: while following it, YOLO looks at a crop around it
        self.target_seen = 0.0

        from ultralytics import YOLOWorld
        self.model = YOLOWorld(param('model_path', YOLO_MODEL))
        self.vocabulary = list(KNOWN)
        self.model.set_classes(self.vocabulary)
        self.list_features = self.model.model.txt_feats.clone()  # for every word in KNOWN
        self.use_words(VOCABULARY)
        self.wanted = list(VOCABULARY)  # the list the goal needs; differs from vocabulary while a new word is learnt
        self.learning = False
        for h, w in ((720, 1280), (396, 704), (360, 640)):
            self.model.predict(np.zeros((h, w, 3), np.uint8), imgsz=self.image_size, verbose=False)
        self.hits = 0  # sightings of the followed object
        self.look = 0  # counts YOLO looks: even = whole frame, odd = the next quarter
        self.bridge = CvBridge()
        self.mode = ''
        self.goal = ''
        self.started = None  # None when not searching
        self.last_point = None  # the followed object's last confirmed location in this search
        self.depth_msgs = collections.deque(maxlen=5)  # recent depth images, only read once YOLO has a goal box
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
            if self.goal in VOCABULARY:
                self.wanted = VOCABULARY
            elif self.goal in WORN:
                self.wanted = VOCABULARY + [self.goal]  # worn things only join the list while they are the goal
            else:  # Any other word still works. Words inside it are left out, or 'chair' would win every 'armchair' box.
                self.wanted = [w for w in VOCABULARY if w not in self.goal] + [self.goal]
            if self.goal in KNOWN and not self.learning:
                self.use_words(self.wanted)  # instant: these words' numbers are kept from launch
            elif self.wanted != self.vocabulary and not self.learning:
                self.learning = True
                threading.Thread(target=self.learn, daemon=True).start()
        if self.mode == self.mode_name:
            self.start_search()  # also when the same goal is sent again

    def learn(self):
        """Work out a new word's numbers in the background, as it takes seconds on the Jetson."""
        while self.wanted != self.vocabulary:  # again if another goal came in meanwhile
            wanted = list(self.wanted)
            if wanted[-1] not in KNOWN:
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
        parts = [self.list_features[:, [KNOWN.index(w) for w in words if w in KNOWN]]]
        new = [w for w in words if w not in KNOWN]
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
        self.started, self.last_point, self.hits, self.target, self.target_box, self.look = time.monotonic(), None, 0, None, None, 0
        self.prev_target, self.misses, self.close_hits = None, 0, 0
        if not self.learning:
            self.get_logger().info(f'Looking for a {self.goal}.')

    def on_info(self, msg):
        self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def on_depth(self, msg):
        self.depth_msgs.append(msg)  # kept as they are: read only once YOLO has a box, and then only inside it

    def depth_for(self, stamp):
        """The depth image taken closest in time to the colour frame, so the box lands on the right pixels when moving."""
        t = stamp.sec + stamp.nanosec * 1e-9
        return min(self.depth_msgs, key=lambda m: abs(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 - t), default=None)

    def on_image(self, msg):
        if self.started is None or self.learning:
            return  # YOLO stays idle, or the word list is being changed
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        point, boxes, chosen, region = self.locate(frame, msg.header.stamp)
        self.misses = 0 if chosen is not None else self.misses + self.focused
        if self.misses >= self.lost_looks:  # gone from where it was: drop it and look at the whole frame next
            self.prev_target = self.target
            self.target, self.target_box, self.misses, self.look = None, None, 0, 0
        self.publish_preview(frame, boxes, chosen, point, region)
        # Also seen at least once in the zoomed close-up: things that only look like the goal in the wide view (seen from
        # far, at low detail) are not reported
        if point is not None and self.hits >= self.confirm_frames and self.close_hits >= 1:
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
        """(x1, y1, x2, y2) of the frame YOLO looks at next."""
        h, w = shape[:2]
        self.focused = self.target_box is not None and time.monotonic() - self.target_seen <= self.lost_after
        if self.focused:
            # About 4x the object: a small object is enlarged for YOLO (a 32 px bottle in 2.7x zoom was found in 6 of 6 busy
            # scenes, against 3 of 6 at the size of a quarter), a big one still gets a wide view
            x1, y1, x2, y2 = self.target_box
            ch = min(h, max(135, 4 * (y2 - y1), 4 * (x2 - x1) * 9 / 16))
            cw = min(w, ch * 16 / 9)
            ch = cw * 9 / 16
            cx = min(max((x1 + x2) / 2, cw / 2), w - cw / 2)
            cy = min(max((y1 + y2) / 2, ch / 2), h - ch / 2)
            return int(cx - cw / 2), int(cy - ch / 2), int(cx + cw / 2), int(cy + ch / 2)
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

    def locate(self, frame, stamp):
        """(PointStamped of the followed object or None, goal boxes as (xyxy, conf), the chosen box, the region looked at)."""
        region = self.next_region(frame.shape)
        x0, y0, x1, y1 = region
        result = self.model.predict(frame[y0:y1, x0:x1], imgsz=self.image_size, conf=self.confidence, verbose=False)[0]
        goal = self.vocabulary.index(self.goal)
        boxes = sorted((([b[0] + x0, b[1] + y0, b[2] + x0, b[3] + y0], p)  # back to whole-frame pixels
                        for b, p, c in zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist(), result.boxes.cls.tolist())
                        if int(c) == goal), key=lambda b: -b[1])  # the goal only; other words just compete
        if not boxes:
            return None, boxes, None, region  # a look that missed it (e.g. another quarter) doesn't undo the sightings
        if self.intrinsics is None:
            self.get_logger().warn('Waiting for the camera info.', throttle_duration_sec=5.0)
            return None, boxes, None, region

        depth_msg = self.depth_for(stamp)
        if depth_msg is None:
            self.get_logger().warn('Waiting for the depth image.', throttle_duration_sec=5.0)
            return None, boxes, None, region
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')  # a view of the message, nothing converted yet
        if depth.shape[:2] != frame.shape[:2]:
            self.get_logger().warn('Depth is not aligned to colour; turn on depth registration in the camera driver.', throttle_duration_sec=5.0)
            return None, boxes, None, region
        scale = 0.001 if depth_msg.encoding == '16UC1' else 1.0  # 16UC1 is mm, 32FC1 m
        fx, fy, cx, cy = self.intrinsics
        candidates = []  # (position, box) of every goal detection with a depth reading
        for box, _ in boxes:
            z = depth_at(box, depth, scale)
            if z is not None:
                u, v = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
                candidates.append((np.array([z, -(u - cx) * z / fx, -(v - cy) * z / fy]), box))  # optical frame to x forward, y left, z up

        now = time.monotonic()
        if self.target is not None and now - self.target_seen > self.lost_after:
            self.prev_target = self.target
            self.target, self.target_box = None, None  # lost it: pick again
        if self.target is None:
            pick = min(candidates, key=lambda c: np.linalg.norm(c[0]), default=None)  # the closest one
            # Found again where it was lost (a small object that YOLO misses now and then): keep counting its sightings
            if pick is None or self.prev_target is None or np.linalg.norm(pick[0] - self.prev_target) > self.track_gate:
                self.hits, self.close_hits = 0, 0
        else:
            pick = min(candidates, key=lambda c: np.linalg.norm(c[0] - self.target), default=None)  # the same one as before
            if pick is not None and np.linalg.norm(pick[0] - self.target) > self.track_gate:
                pick = None  # only other objects in view: don't swap to them
        if pick is None:
            return None, boxes, None, region

        self.target, self.target_box, self.target_seen, self.hits = pick[0], pick[1], now, self.hits + 1
        self.close_hits += self.focused
        point = PointStamped()
        point.header.frame_id = self.frame_id
        point.point.x, point.point.y, point.point.z = (float(c) for c in pick[0])
        return point, boxes, pick[1], region

    def publish_preview(self, frame, boxes, chosen, point, region):
        """Green box: the detection sent to navigation. Grey: other detections (no depth reading, or less confident)."""
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
