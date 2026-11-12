#!/usr/bin/env python3
"""Shows Pete's camera preview in a window. While object detection is searching, it shows what YOLO sees, with its boxes."""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


class Viewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.camera = self.yolo = None
        self.yolo_time = 0.0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CompressedImage, '/camera/color/image_raw/compressed', self.on_camera, qos)
        self.create_subscription(CompressedImage, '/object_detection/image/compressed', self.on_yolo, qos)

    def on_camera(self, msg):
        self.camera = decode(msg)

    def on_yolo(self, msg):
        self.yolo, self.yolo_time = decode(msg), time.monotonic()

    def frame(self):
        """YOLO's view while it is searching, otherwise the plain camera."""
        return self.yolo if time.monotonic() - self.yolo_time < 1.0 else self.camera


def decode(msg):
    return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)


def main():
    rclpy.init()
    node = Viewer()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            if node.frame() is not None:
                cv2.imshow('Pete camera', node.frame())
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
