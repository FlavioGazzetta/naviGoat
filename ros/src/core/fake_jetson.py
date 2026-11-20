#!/usr/bin/env python3
"""Stands in for the Jetson's side on a laptop, to try the split (Pete on the laptop, camera, YOLO and people tracker
on the Jetson) without the robot. It does what navigation's social:=true does, with the laptop's webcam as the camera:

  - the camera topics: /camera/color/image_raw (the webcam), /camera/depth_registered/image_rect (no real depth:
    every point is --distance metres away) and /camera/color/camera_info
  - yolo_server.py --host 0.0.0.0, as navigation starts it
  - the people tracker (/social_perception/tracks), unless --no-tracker

No base, lidar, SLAM or Nav2: "take me to ..." and follow_drive:=true have nothing to drive.

Terminal 1, the fake Jetson (the container needs the webcam: --device /dev/video0):
    python3 /workspace/ros/src/core/fake_jetson.py
Terminal 2, Pete as on the day, in another container:
    ros2 launch /workspace/ros/src/core/launch/start_pete.launch.py camera:=false follow:=true yolo_url:=http://localhost:8767
"""

import argparse
import os
import subprocess
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))  # ros/src
PERCEPTION_DIR = os.path.join(SRC_DIR, 'autonomy', 'perception')
PEOPLE_DIR = os.path.join(PERCEPTION_DIR, 'social_perception', 'femto_id_velocity_tracker')
FRAME = 'camera_color_optical_frame'


class FakeCamera(Node):
    def __init__(self, args):
        super().__init__('fake_jetson_camera')
        self.distance_mm = int(args.distance * 1000)
        self.webcam = cv2.VideoCapture(args.webcam)
        self.webcam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        if not self.webcam.isOpened():
            self.get_logger().warn(f'No webcam {args.webcam} (start the container with --device /dev/video0): sending a grey picture.')
        self.colour = self.create_publisher(Image, '/camera/color/image_raw', 1)
        self.depth = self.create_publisher(Image, '/camera/depth_registered/image_rect', 1)
        self.info = self.create_publisher(CameraInfo, '/camera/color/camera_info', 1)
        self.create_timer(1.0 / args.rate, self.send)
        self.get_logger().info(f'Fake camera: {args.rate:g} pictures a second, everything {args.distance:g} m away.')

    def image(self, array, encoding, stamp):
        msg = Image(height=array.shape[0], width=array.shape[1], encoding=encoding, step=array.strides[0], data=array.tobytes())
        msg.header.stamp, msg.header.frame_id = stamp, FRAME
        return msg

    def send(self):
        ok, picture = self.webcam.read() if self.webcam.isOpened() else (False, None)
        if not ok:
            picture = np.full((720, 1280, 3), 128, np.uint8)
        height, width = picture.shape[:2]
        stamp = self.get_clock().now().to_msg()
        self.colour.publish(self.image(picture, 'bgr8', stamp))
        self.depth.publish(self.image(np.full((height, width), self.distance_mm, np.uint16), '16UC1', stamp))
        focal = 0.8 * width  # about a 64 degree wide view, close to a webcam's and the Femto's colour camera
        info = CameraInfo(height=height, width=width, distortion_model='plumb_bob', d=[0.0] * 5,
                          k=[focal, 0.0, width / 2, 0.0, focal, height / 2, 0.0, 0.0, 1.0],
                          p=[focal, 0.0, width / 2, 0.0, 0.0, focal, height / 2, 0.0, 0.0, 0.0, 1.0, 0.0])
        info.header.stamp, info.header.frame_id = stamp, FRAME
        self.info.publish(info)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--webcam', type=int, default=0, help='the webcam number (/dev/video0 is 0)')
    parser.add_argument('--rate', type=float, default=10.0, help='pictures a second')
    parser.add_argument('--distance', type=float, default=2.0, help='the depth picture says everything is this far away, in metres')
    parser.add_argument('--no-tracker', action='store_true', help='leave the people tracker out')
    args = parser.parse_args()

    started = [subprocess.Popen([sys.executable, '-u', os.path.join(PERCEPTION_DIR, 'yolo_server.py'), '--host', '0.0.0.0'])]
    if not args.no_tracker:  # the same options as navigation_robot.launch.py
        started.append(subprocess.Popen(['/opt/people_env/bin/python3', '-u', 'femto_track_people.py', '--yolo-url', 'http://127.0.0.1:8767',
                                         '--imgsz', '320', '--conf', '0.40', '--max-age', '1.0', '--no-plot'], cwd=PEOPLE_DIR))
    rclpy.init()
    try:
        rclpy.spin(FakeCamera(args))
    except KeyboardInterrupt:
        pass
    finally:
        for process in started:
            process.terminate()


if __name__ == '__main__':
    main()
