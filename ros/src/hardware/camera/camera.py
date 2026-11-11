#!/usr/bin/env python3
"""Publishes the Orbbec Femto Bolt's colour, depth (aligned to colour, 16UC1 mm) and camera info,
on the same topics as the Orbbec ROS 2 driver. Use one or the other, not both."""

import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

CAMERA_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class FemtoBolt:
    def __init__(self, width, height, fps):
        import pyorbbecsdk as ob
        ob.Context.set_logger_to_file(ob.OBLogLevel.NONE, '')  # otherwise the SDK writes a Log/ folder
        self.pipeline = ob.Pipeline()
        colour = self.pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR).get_video_stream_profile(width, height, ob.OBFormat.MJPG, fps)
        config = ob.Config()
        config.enable_stream(colour)
        config.enable_stream(self.pipeline.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR).get_default_video_stream_profile())
        self.pipeline.start(config)
        self.align = ob.AlignFilter(align_to_stream=ob.OBStreamType.COLOR_STREAM)
        self.intrinsic = colour.get_intrinsic()
        self.lock = threading.Lock()

    def read(self):
        """(bgr image, depth in mm as uint16) aligned to each other, or None."""
        with self.lock:
            if self.pipeline is None:
                return None
            frames = self.pipeline.wait_for_frames(1000)
            frames = None if frames is None else self.align.process(frames)
            frames = None if frames is None else frames.as_frame_set()
            if frames is None or frames.get_color_frame() is None or frames.get_depth_frame() is None:
                return None
            colour, depth = frames.get_color_frame(), frames.get_depth_frame()
            image = cv2.imdecode(np.frombuffer(colour.get_data(), np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                return None
            raw = np.frombuffer(depth.get_data(), np.uint16).reshape(depth.get_height(), depth.get_width())
            return image, (raw * depth.get_depth_scale()).astype(np.uint16)

    def release(self):
        with self.lock:  # stop explicitly: leaving it to Python's exit crashes on the Jetson
            if self.pipeline:
                self.pipeline.stop()
                self.pipeline = None


class Camera(Node):
    def __init__(self):
        super().__init__('camera')
        param = lambda name, default: self.declare_parameter(name, default).value
        self.frame_id = param('frame_id', 'camera_color_optical_frame')
        self.camera = FemtoBolt(param('width', 1280), param('height', 720), param('fps', 30))
        self.bridge = CvBridge()
        self.colour_pub = self.create_publisher(Image, '/camera/color/image_raw', CAMERA_QOS)
        self.depth_pub = self.create_publisher(Image, '/camera/depth_registered/image_rect', CAMERA_QOS)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', CAMERA_QOS)
        # A small jpeg preview to watch from another computer: full frames don't get through a USB or Wi-Fi link
        self.preview_pub = self.create_publisher(CompressedImage, '/camera/color/image_raw/compressed', CAMERA_QOS)
        i = self.camera.intrinsic
        self.info = CameraInfo(width=i.width, height=i.height, k=[i.fx, 0.0, i.cx, 0.0, i.fy, i.cy, 0.0, 0.0, 1.0])
        self.running = True
        threading.Thread(target=self.loop, daemon=True).start()
        self.get_logger().info(f'Femto Bolt streaming {i.width}x{i.height}, depth aligned to colour.')

    def loop(self):
        while self.running and rclpy.ok():
            frame = self.camera.read()
            if frame is None:
                continue
            image, depth = frame
            stamp = self.get_clock().now().to_msg()
            small = cv2.resize(image, (640, image.shape[0] * 640 // image.shape[1]))
            preview = CompressedImage(format='jpeg', data=cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 60])[1].tobytes())
            for msg, pub in ((self.bridge.cv2_to_imgmsg(image, 'bgr8'), self.colour_pub),
                             (self.bridge.cv2_to_imgmsg(depth, '16UC1'), self.depth_pub), (self.info, self.info_pub), (preview, self.preview_pub)):
                msg.header.stamp, msg.header.frame_id = stamp, self.frame_id
                pub.publish(msg)

    def close(self):
        self.running = False
        self.camera.release()


def main():
    rclpy.init()
    node = Camera()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
