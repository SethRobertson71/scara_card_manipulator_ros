#!/usr/bin/env python3
"""Interactive image clicker for affine calibration point collection.

Usage example:
  python3 click_calibration_points.py --image-topic /camera/image_rect --out /tmp/pixel_points.json

Controls:
  Left click  - add a point
  u           - undo last point
  c           - clear all points
  s           - save points to JSON
  q / ESC     - quit
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import List, Tuple

import cv2
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.node import Node


class ClickCalibrationPoints(Node):
    def __init__(self, image_topic: str, output_path: Path, window_name: str) -> None:
        super().__init__("click_calibration_points")
        self._bridge = CvBridge()
        self._output_path = output_path
        self._window_name = window_name
        self._latest_frame = None
        self._points: List[Tuple[int, int]] = []

        self.create_subscription(Image, image_topic, self._image_callback, 10)
        cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self._window_name, self._mouse_callback)

        self.get_logger().info(f"Listening on {image_topic}")
        self.get_logger().info("Left click to add points, u=undo, c=clear, s=save, q/ESC=quit")
        self.get_logger().info(f"Saving points to {self._output_path}")

    def _image_callback(self, msg: Image) -> None:
        try:
            self._latest_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Failed to convert image: {exc}")

    def _mouse_callback(self, event: int, x: int, y: int, flags: int, param) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            self._points.append((x, y))
            self.get_logger().info(f"point[{len(self._points) - 1}] = ({x}, {y})")

    def _draw_overlay(self, frame):
        overlay = frame.copy()
        for idx, (x, y) in enumerate(self._points):
            cv2.circle(overlay, (x, y), 5, (0, 255, 0), -1)
            cv2.putText(
                overlay,
                str(idx),
                (x + 8, y - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
        cv2.putText(
            overlay,
            f"points: {len(self._points)}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return overlay

    def _save_points(self) -> None:
        payload = {
            "image_points": [{"u": int(x), "v": int(y)} for x, y in self._points]
        }
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        self._output_path.write_text(json.dumps(payload, indent=2))
        self.get_logger().info(f"Saved {len(self._points)} points to {self._output_path}")

    def spin_once(self) -> bool:
        if self._latest_frame is None:
            return True

        frame = self._draw_overlay(self._latest_frame)
        cv2.imshow(self._window_name, frame)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), 27):
            return False
        if key == ord("u") and self._points:
            removed = self._points.pop()
            self.get_logger().info(f"Removed point ({removed[0]}, {removed[1]})")
        if key == ord("c"):
            self._points.clear()
            self.get_logger().info("Cleared all points")
        if key == ord("s"):
            self._save_points()

        return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Click points on a live ROS image.")
    parser.add_argument("--image-topic", default="/camera/image_rect", help="Image topic to subscribe to")
    parser.add_argument("--out", default="/tmp/pixel_points.json", help="Where to save clicked points")
    parser.add_argument("--window-name", default="Calibration Points", help="OpenCV window name")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = ClickCalibrationPoints(args.image_topic, Path(args.out), args.window_name)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if not node.spin_once():
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
