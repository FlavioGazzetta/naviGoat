#!/usr/bin/env python3
"""Low-latency Femto Bolt person IDs + velocity estimation in ROS 2.

Camera/image topics match the user's working depth_image_proc setup:
  /camera/color/image_raw
  /camera/depth_registered/image_rect
  /camera/color/camera_info

Keeps only the latest unprocessed synced image pair; Kalman/Hungarian tracker
uses CAMERA timestamps. Demonstration positions are APPROXIMATE forward/left
coordinates relative to a stationary, level camera, NOT odometry-compensated.

Publishes /social_perception/tracks as JSON in std_msgs/String for prototyping.
For a moving robot, transform actual 3D detections into odom BEFORE tracking.
This is not a safety-critical obstacle avoidance system.
"""

import argparse
import json
import os
import threading
import time
from dataclasses import asdict

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from ultralytics import YOLO, YOLOWorld

from tracker2d import Tracker2D


def stamp_s(msg):
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


class FemtoPersonTracker(Node):
    def __init__(self, args):
        super().__init__("femto_person_tracker")
        self.args = args
        self.bridge = CvBridge()
        import torch
        if args.threads is not None:
            torch.set_num_threads(args.threads)
        self.device = (
            "cuda:0" if args.device == "auto" and torch.cuda.is_available()
            else "cpu" if args.device == "auto"
            else args.device
        )
        # Open-vocabulary .pt: specify person class explicitly. Other checkpoints
        # (including TensorRT exports) must already have person at class index 0.
        if "world" in os.path.basename(args.model).lower() and args.model.endswith(".pt"):
            self.model = YOLOWorld(args.model)
            self.model.set_classes(["person"])
        else:
            self.model = YOLO(args.model)
        self.tracker = Tracker2D(
            gate_m=args.gate, max_age_s=args.max_age,
            min_hits=args.min_hits,
        )

        self.camera_info = None
        self._pair_lock = threading.Lock()
        self._latest_pair = None
        self._new_pair = threading.Event()
        self._stop = threading.Event()
        self._result_lock = threading.Lock()
        self._latest_result = None
        self._result_seq = 0
        self._drop_count = 0
        self._last_log_time = 0.0
        self._warned_shape = False
        self._warned_depth = False

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self.on_camera_info, qos
        )
        color_sub = Subscriber(
            self, Image, "/camera/color/image_raw", qos_profile=qos
        )
        depth_sub = Subscriber(
            self, Image, "/camera/depth_registered/image_rect", qos_profile=qos
        )
        self.sync = ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=3, slop=args.slop
        )
        self.sync.registerCallback(self.on_images)
        self.pub = self.create_publisher(String, "/social_perception/tracks", 10)

        self._plt = None
        self._drawn_seq = -1
        self._plot_labels = []
        self._arrows = []
        if not args.no_plot:
            import matplotlib.pyplot as plt
            self._plt = plt
            plt.ion()
            self.fig, (self.image_ax, self.map_ax) = plt.subplots(
                1, 2, figsize=(12, 5), layout="constrained"
            )
            self.image_ax.set_title("RGB person detections + IDs")
            self.image_ax.axis("off")
            self.image_artist = self.image_ax.imshow(
                np.zeros((360, 640, 3), dtype=np.uint8)
            )
            self.map_ax.scatter([0], [0], s=130, marker="s",
                                c="tab:blue", label="Camera")
            self.map_scatter = self.map_ax.scatter(
                [], [], s=95, c="tab:orange", label="Tracked people"
            )
            self.map_ax.set_xlim(-0.5, 7.)
            self.map_ax.set_ylim(-4., 4.)
            self.map_ax.set_aspect("equal", adjustable="box")
            self.map_ax.set_xlabel("Forward from camera (m)")
            self.map_ax.set_ylabel("Left (+) / Right (-) (m)")
            self.map_ax.grid(alpha=0.3)
            self.map_ax.legend(loc="upper right")

        self._worker = threading.Thread(target=self.process_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f"Model={args.model}; device={self.device}; imgsz={args.imgsz}; "
            "latest-frame-only; tracking camera-relative positions."
        )
        self.get_logger().info(
            "Waiting for RGB, /camera/depth_registered/image_rect, and camera info..."
        )

    def on_camera_info(self, msg):
        self.camera_info = msg

    def on_images(self, rgb_msg, depth_msg):
        # ROS callback intentionally does NOT run YOLO / Kalman / matplotlib.
        with self._pair_lock:
            if self._latest_pair is not None:
                self._drop_count += 1
            self._latest_pair = (rgb_msg, depth_msg, time.monotonic())
            self._new_pair.set()

    def process_loop(self):
        while not self._stop.is_set():
            if not self._new_pair.wait(timeout=0.1):
                continue
            with self._pair_lock:
                pair = self._latest_pair
                self._latest_pair = None
                self._new_pair.clear()
            if pair is None:
                continue
            info = self.camera_info
            if info is None:
                continue
            rgb_msg, depth_msg, received_at = pair
            started = time.monotonic()
            try:
                bgr = self.bridge.imgmsg_to_cv2(
                    rgb_msg, desired_encoding="bgr8"
                )
                depth = self.bridge.imgmsg_to_cv2(
                    depth_msg, desired_encoding="passthrough"
                )
            except Exception as exc:
                self.get_logger().error(f"CvBridge conversion error: {exc}")
                continue

            if bgr.shape[:2] != depth.shape[:2]:
                if not self._warned_shape:
                    self.get_logger().error(
                        f"RGB shape {bgr.shape}, depth shape {depth.shape}. "
                        "Check that external depth_image_proc is publishing "
                        "/camera/depth_registered/image_rect."
                    )
                    self._warned_shape = True
                continue
            self._warned_shape = False

            if depth_msg.encoding == "16UC1":
                metres_per_unit = 0.001
            elif depth_msg.encoding == "32FC1":
                metres_per_unit = 1.0
            else:
                if not self._warned_depth:
                    self.get_logger().error(
                        f"Unsupported depth encoding {depth_msg.encoding}; "
                        "expected 16UC1 or 32FC1"
                    )
                    self._warned_depth = True
                continue
            self._warned_depth = False

            fx, cx = float(info.k[0]), float(info.k[2])
            if fx <= 0 or info.width != rgb_msg.width or info.height != rgb_msg.height:
                self.get_logger().warning(
                    "RGB CameraInfo dimensions/intrinsics do not match the RGB image; skipping"
                )
                continue

            try:
                result = self.model.predict(
                    bgr, classes=[0], conf=self.args.conf,
                    imgsz=self.args.imgsz, device=self.device,
                    verbose=False
                )[0]
            except Exception as exc:
                self.get_logger().error(f"YOLO prediction error: {exc}")
                continue

            positions = []
            boxes = []
            for box in result.boxes:
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                width, height = x2 - x1, y2 - y1
                if width < 10 or height < 10:
                    continue
                u1 = max(0, x1 + int(0.35 * width))
                u2 = min(depth.shape[1], x1 + int(0.65 * width))
                v1 = max(0, y1 + int(0.25 * height))
                v2 = min(depth.shape[0], y1 + int(0.55 * height))
                if u1 >= u2 or v1 >= v2:
                    continue
                roi = depth[v1:v2, u1:u2]
                values = roi.astype(np.float32) * metres_per_unit
                values = values[np.isfinite(values) &
                                (values >= self.args.min_depth) &
                                (values <= self.args.max_depth)]
                if values.size < 10:
                    continue
                z = float(np.median(values))
                u = 0.5 * (u1 + u2)
                # For a LEVEL camera: +x is forward, +y is LEFT.
                # Optical coordinates: +Z forward, +X right.
                positions.append((z, -(u - cx) * z / fx))
                boxes.append((x1, y1, x2, y2))

            # IMPORTANT: use the image CAPTURE time, not YOLO completion time.
            timestamp = stamp_s(rgb_msg)
            ids, all_tracks = self.tracker.update(positions, timestamp)
            confirmed = [t for t in all_tracks if t.confirmed]

            payload = {
                "stamp_sec": timestamp,
                "coordinate_convention": (
                    "stationary_level_camera: x_forward_m,y_left_m; "
                    "NOT odom-compensated"
                ),
                "tracks": [asdict(t) for t in confirmed],
            }
            msg = String()
            msg.data = json.dumps(payload, separators=(",", ":"))
            self.pub.publish(msg)

            image_rgb = None
            if self._plt is not None:
                annotated = result.plot()
                for bounds, identifier in zip(boxes, ids):
                    if identifier is None:
                        continue
                    cv2.putText(
                        annotated, f"ID {identifier}",
                        (bounds[0], max(18, bounds[1] - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.72,
                        (0, 255, 255), 2, cv2.LINE_AA
                    )
                annotated = cv2.resize(
                    annotated, (640, 360), interpolation=cv2.INTER_AREA
                )
                image_rgb = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)

            finished = time.monotonic()
            age_s = time.time() - timestamp
            camera_age_ms = 1000 * age_s if 0 <= age_s <= 60 else None
            diagnostics = {
                "yolo_ms": float(result.speed.get("inference", 0.)),
                "processing_ms": 1000 * (finished - started),
                "age_ms": camera_age_ms,
                "pair_wait_ms": 1000 * (started - received_at),
                "dropped_pairs": self._drop_count,
            }
            with self._result_lock:
                self._result_seq += 1
                self._latest_result = (
                    self._result_seq, image_rgb, confirmed, diagnostics, finished
                )
            if finished - self._last_log_time >= 2.0:
                tracks_text = ", ".join(
                    f"ID {t.id}: ({t.x:.2f},{t.y:.2f})m, "
                    f"v=({t.vx:.2f},{t.vy:.2f})m/s"
                    for t in confirmed if t.observed
                ) or "none confirmed"
                age_text = (
                    f"{camera_age_ms:.0f}ms" if camera_age_ms is not None
                    else "unknown"
                )
                self.get_logger().info(
                    f"YOLO={diagnostics['yolo_ms']:.0f}ms | "
                    f"process={diagnostics['processing_ms']:.0f}ms | "
                    f"camera-to-result={age_text} | "
                    f"pending-dropped={self._drop_count} | {tracks_text}"
                )
                self._last_log_time = finished

    def update_plot(self):
        if self._plt is None:
            return
        with self._result_lock:
            latest = self._latest_result
        if latest is None:
            return
        seq, image, tracks, diagnostic, finished = latest
        if seq == self._drawn_seq:
            if time.monotonic() - finished > 1.5 and not self._plot_stale:
                self.map_scatter.set_offsets(np.empty((0, 2)))
                for artist in self._plot_labels + self._arrows:
                    artist.remove()
                self._plot_labels.clear()
                self._arrows.clear()
                self.map_ax.set_title("STALE: no recent updates")
                self.fig.canvas.draw_idle()
                self._plot_stale = True
            return
        self._drawn_seq = seq
        self._plot_stale = False
        if image is not None:
            self.image_artist.set_data(image)
        self.map_scatter.set_offsets(
            np.asarray([(t.x, t.y) for t in tracks], dtype=float).reshape(-1, 2)
        )
        for artist in self._plot_labels + self._arrows:
            artist.remove()
        self._plot_labels.clear()
        self._arrows.clear()
        for t in tracks:
            speed = float(np.hypot(t.vx, t.vy))
            label = f"ID {t.id}: {speed:.2f} m/s"
            if not t.observed:
                label += " (predicted)"
            self._plot_labels.append(
                self.map_ax.annotate(
                    label, (t.x, t.y), xytext=(5, 7),
                    textcoords="offset points", fontsize=9
                )
            )
            if t.hits >= 3 and speed >= 0.10:
                self._arrows.append(
                    self.map_ax.arrow(
                        t.x, t.y, t.vx * 0.6, t.vy * 0.6,
                        color="tab:green", head_width=0.1,
                        width=0.015, length_includes_head=True
                    )
                )
        age = diagnostic["age_ms"]
        label_age = f" | image age {age:.0f}ms" if age is not None else ""
        self.map_ax.set_title(f"Confirmed tracks: {len(tracks)}{label_age}")
        self.fig.canvas.draw_idle()

    def stop_worker(self):
        self._stop.set()
        self._new_pair.set()
        self._worker.join(timeout=5.)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="yolov8s-worldv2.pt")
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument("--conf", type=float, default=0.45)
    parser.add_argument("--slop", type=float, default=0.08,
                        help="Maximum approximate RGB/depth timestamp difference")
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--device", default="auto",
                        help="auto, cpu, cuda:0; verify CUDA support on Jetson")
    parser.add_argument("--gate", type=float, default=1.0,
                        help="Max predicted-to-detected association distance (m)")
    parser.add_argument("--max-age", type=float, default=1.0,
                        help="Keep temporarily lost track for this many seconds")
    parser.add_argument("--min-hits", type=int, default=2,
                        help="Observations required before track publication")
    parser.add_argument("--min-depth", type=float, default=0.35)
    parser.add_argument("--max-depth", type=float, default=7.0)
    parser.add_argument("--no-plot", action="store_true",
                        help="Disable matplotlib (use for headless Jetson operation)")
    args = parser.parse_args()

    rclpy.init()
    node = FemtoPersonTracker(args)
    node._plot_stale = False
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        if args.no_plot:
            while rclpy.ok():
                time.sleep(0.2)
        else:
            plt = node._plt
            while rclpy.ok() and plt.fignum_exists(node.fig.number):
                node.update_plot()
                plt.pause(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop.set()
        node._new_pair.set()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.)
        node.stop_worker()
        node.destroy_node()
        if node._plt is not None:
            node._plt.close("all")


if __name__ == "__main__":
    main()
