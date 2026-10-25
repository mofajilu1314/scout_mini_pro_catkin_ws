#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Lightweight camera frame recorder (Idea 1 stub).

Saves sensor_msgs/Image frames as PNG sequence for GIF conversion.

  ros2 run scout_offroad_3d record_camera_stub --ros-args \\
    -p topic:=/scout_mini_pro/depth/rgb/image_raw \\
    -p out_dir:=/tmp/scout_cam_seq \\
    -p max_frames:=120 \\
    -p every_n:=2
"""

from __future__ import print_function

import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class RecordCameraStub(Node):
    def __init__(self):
        super().__init__("record_camera_stub")
        self.declare_parameter("topic", "/scout_mini_pro/depth/rgb/image_raw")
        self.declare_parameter("out_dir", "/tmp/scout_cam_seq")
        self.declare_parameter("max_frames", 120)
        self.declare_parameter("every_n", 1)
        self.topic = str(self.get_parameter("topic").value)
        self.out_dir = os.path.expanduser(str(self.get_parameter("out_dir").value))
        self.max_frames = int(self.get_parameter("max_frames").value)
        self.every_n = max(1, int(self.get_parameter("every_n").value))
        self._n = 0
        self._saved = 0
        if not os.path.isdir(self.out_dir):
            os.makedirs(self.out_dir)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, self.topic, self._cb, qos)
        self.get_logger().info(
            "recording %s → %s (max=%d every_n=%d)"
            % (self.topic, self.out_dir, self.max_frames, self.every_n)
        )

    def _cb(self, msg):
        self._n += 1
        if (self._n % self.every_n) != 0:
            return
        if self._saved >= self.max_frames:
            self.get_logger().info("done: saved %d frames" % self._saved)
            raise SystemExit(0)
        path = os.path.join(self.out_dir, "frame_%05d.png" % self._saved)
        try:
            self._save_png(msg, path)
            self._saved += 1
            if self._saved == 1 or (self._saved % 20) == 0:
                self.get_logger().info("saved %s", path)
        except Exception as e:
            self.get_logger().warning("save failed: %s" % e)

    def _save_png(self, msg, path):
        # Prefer cv_bridge; fallback raw RGB8/BGR8
        try:
            from cv_bridge import CvBridge
            import cv2

            bridge = CvBridge()
            img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imwrite(path, img)
            return
        except Exception:
            pass
        enc = (msg.encoding or "").lower()
        w, h = int(msg.width), int(msg.height)
        data = bytes(msg.data)
        try:
            from PIL import Image as PILImage
            import numpy as np

            if "rgb" in enc:
                arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
            elif "bgr" in enc:
                arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)[:, :, ::-1]
            elif "mono" in enc or "16uc1" in enc:
                arr = np.frombuffer(data, dtype=np.uint8)[: h * w].reshape(h, w)
            else:
                raise RuntimeError("unsupported encoding %s" % enc)
            PILImage.fromarray(arr).save(path)
        except Exception as e:
            raise RuntimeError("need cv_bridge or Pillow: %s" % e)


def main(args=None):
    rclpy.init(args=args)
    node = RecordCameraStub()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
