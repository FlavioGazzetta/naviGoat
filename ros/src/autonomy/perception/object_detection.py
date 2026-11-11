#!/usr/bin/env python3
"""While /mode is 'perception', finds the /goal object with YOLO-World and publishes its position on /object_mapping,
or 'not_found:<object>' on /object_status after search_timeout seconds."""

import os
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


def depth_at(box, depth):
    """Median depth (m) over the middle half of the box, ignoring holes. None if there is too little."""
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    patch = depth[int(y1 + h / 4):int(y2 - h / 4) + 1, int(x1 + w / 4):int(x2 - w / 4) + 1]
    valid = patch[patch > 0]
    return float(np.median(valid)) if valid.size > 20 else None


class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')
        param = lambda name, default: self.declare_parameter(name, default).value
        self.mode_name = param('mode_name', 'perception')
        self.timeout = param('search_timeout', 10.0)
        self.confidence = param('confidence', 0.25)
        self.frame_id = param('frame_id', 'camera_link')

        from ultralytics import YOLOWorld
        self.model = YOLOWorld(param('model_path', YOLO_MODEL))
        self.bridge = CvBridge()
        self.mode = ''
        self.goal = ''
        self.started = None  # None when not searching
        self.found = False
        self.depth = None
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
        self.goal = msg.data.strip().lower()
        if self.goal:
            self.model.set_classes([self.goal])
        if self.mode == self.mode_name:
            self.start_search()  # also when the same goal is sent again

    def start_search(self):
        if not self.goal:
            return
        self.started, self.found = time.monotonic(), False
        self.get_logger().info(f'Looking for a {self.goal}.')

    def on_info(self, msg):
        self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def on_depth(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        self.depth = depth / 1000.0 if msg.encoding == '16UC1' else depth  # 16UC1 is mm, 32FC1 m

    def on_image(self, msg):
        if self.started is None:
            return  # YOLO stays idle
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        point, boxes, chosen = self.locate(frame)
        self.publish_preview(frame, boxes, chosen, point)
        if point is not None:
            point.header.stamp = msg.header.stamp
            self.point_pub.publish(point)
            if not self.found:
                self.found = True
                self.status_pub.publish(String(data=f'found:{self.goal}'))
                self.get_logger().info(f'Found the {self.goal} at x={point.point.x:.2f} m, y={point.point.y:.2f} m.')
        elif not self.found and time.monotonic() - self.started > self.timeout:
            self.status_pub.publish(String(data=f'not_found:{self.goal}'))
            self.get_logger().info(f'No {self.goal} found after {self.timeout:g} s.')
            self.started = None

    def locate(self, frame):
        """(PointStamped of the most confident detection that has a depth reading or None, all boxes as (xyxy, conf), the chosen box)."""
        if self.depth is None or self.intrinsics is None:
            self.get_logger().warn('Waiting for the depth image and camera info.', throttle_duration_sec=5.0)
            return None, [], None
        if self.depth.shape[:2] != frame.shape[:2]:
            self.get_logger().warn('Depth is not aligned to colour; turn on depth registration in the camera driver.', throttle_duration_sec=5.0)
            return None, [], None
        result = self.model.predict(frame, conf=self.confidence, verbose=False)[0].boxes
        boxes = sorted(zip(result.xyxy.tolist(), result.conf.tolist()), key=lambda b: -b[1])
        for box, _ in boxes:
            z = depth_at(box, self.depth)
            if z is None:
                continue
            fx, fy, cx, cy = self.intrinsics
            u, v = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
            point = PointStamped()
            point.header.frame_id = self.frame_id
            point.point.x, point.point.y, point.point.z = z, -(u - cx) * z / fx, -(v - cy) * z / fy  # optical frame to x forward, y left, z up
            return point, boxes, box
        return None, boxes, None

    def publish_preview(self, frame, boxes, chosen, point):
        """Green box: the detection sent to navigation. Grey: other detections (no depth reading, or less confident)."""
        scale = 640 / frame.shape[1]
        image = cv2.resize(frame, (640, int(frame.shape[0] * scale)))
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
