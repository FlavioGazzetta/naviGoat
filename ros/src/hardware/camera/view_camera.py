#!/usr/bin/env python3
"""Shows Pete's camera preview (/camera/color/image_raw/compressed) in a window. Run it on any computer on the robot's network.
Press q to quit."""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


class Viewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.frame = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CompressedImage, '/camera/color/image_raw/compressed', self.on_image, qos)

    def on_image(self, msg):
        self.frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)


def main():
    rclpy.init()
    node = Viewer()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            if node.frame is not None:
                cv2.imshow('Pete camera', node.frame)
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
