#!/usr/bin/env python3
"""Interactive GUI tool to fit and save an affine pixel->world transform.

Workflow:
1. Subscribe to a camera image topic.
2. Click a pixel in the image.
3. Enter world X/Y (mm) for that clicked point.
4. Repeat for 6-20 points across the workspace.
5. Press 's' to fit and save affine_transform.npy.

Controls:
- Left click: add correspondence (pixel + world prompt)
- u: undo last point
- c: clear all points
- s: solve and save affine matrix
- p: save points JSON next to matrix file
- q / ESC: quit
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    import tkinter as tk
    from tkinter import simpledialog

    TK_AVAILABLE = True
except Exception:
    TK_AVAILABLE = False


class AffineCalibrationGUI(Node):
    def __init__(self, image_topic: str, output_path: Path, window_name: str, ransac_threshold: float) -> None:
        super().__init__("affine_calibration_gui")
        self._bridge = CvBridge()
        self._image_topic = image_topic
        self._output_path = output_path
        self._window_name = window_name
        self._ransac_threshold = ransac_threshold

        self._latest_frame: Optional[np.ndarray] = None
        self._pixel_points: List[Tuple[float, float]] = []
        self._world_points: List[Tuple[float, float]] = []
        self._last_matrix: Optional[np.ndarray] = None

        self.create_subscription(Image, self._image_topic, self._image_callback, 10)

        cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self._window_name, self._on_mouse)

        self.get_logger().info(f"Listening on {self._image_topic}")
        self.get_logger().info("Click points, enter world X/Y in mm, press 's' to fit and save")

    def _image_callback(self, msg: Image) -> None:
        try:
            self._latest_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")

    def _prompt_world_xy(self, px: float, py: float) -> Optional[Tuple[float, float]]:
        title = f"World coordinate for pixel ({px:.1f}, {py:.1f})"
        if TK_AVAILABLE:
            root = tk.Tk()
            root.withdraw()
            try:
                wx = simpledialog.askfloat(title, "World X (mm):", parent=root)
                if wx is None:
                    return None
                wy = simpledialog.askfloat(title, "World Y (mm):", parent=root)
                if wy is None:
                    return None
                return float(wx), float(wy)
            finally:
                root.destroy()

        # Fallback if tkinter is not available.
        self.get_logger().warn("Tkinter unavailable. Falling back to terminal input.")
        try:
            wx = float(input(f"{title} -> X (mm): ").strip())
            wy = float(input(f"{title} -> Y (mm): ").strip())
            return wx, wy
        except Exception:
            self.get_logger().warn("Invalid input. Point ignored.")
            return None

    def _on_mouse(self, event: int, x: int, y: int, flags: int, param) -> None:
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        world_xy = self._prompt_world_xy(float(x), float(y))
        if world_xy is None:
            self.get_logger().info("Point input canceled.")
            return

        self._pixel_points.append((float(x), float(y)))
        self._world_points.append(world_xy)
        idx = len(self._pixel_points) - 1
        self.get_logger().info(
            f"point[{idx}] pixel=({x:.1f}, {y:.1f}) world=({world_xy[0]:.2f}, {world_xy[1]:.2f})"
        )

    def _fit_affine(self) -> Optional[np.ndarray]:
        if len(self._pixel_points) < 3:
            self.get_logger().warn("Need at least 3 correspondences to fit affine transform")
            return None

        src = np.array(self._pixel_points, dtype=np.float32)
        dst = np.array(self._world_points, dtype=np.float32)

        matrix, inliers = cv2.estimateAffine2D(
            src,
            dst,
            method=cv2.RANSAC,
            ransacReprojThreshold=self._ransac_threshold,
            maxIters=5000,
            confidence=0.99,
            refineIters=20,
        )

        if matrix is None:
            self.get_logger().warn("RANSAC fit failed. Falling back to first 3 points.")
            matrix = cv2.getAffineTransform(src[:3], dst[:3])
            inlier_count = 3
        else:
            inlier_count = int(inliers.sum()) if inliers is not None else len(src)

        pred = (matrix[:, :2] @ src.T + matrix[:, 2:3]).T
        residual = np.linalg.norm(pred - dst, axis=1)
        self.get_logger().info(
            f"Affine fit complete: points={len(src)}, inliers={inlier_count}, "
            f"mean_err={residual.mean():.2f} mm, max_err={residual.max():.2f} mm"
        )
        return matrix

    def _save_matrix(self, matrix: np.ndarray) -> None:
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        np.save(self._output_path, matrix)
        self.get_logger().info(f"Saved affine matrix to {self._output_path}")

    def _save_points_json(self) -> None:
        json_path = self._output_path.with_suffix(".points.json")
        payload = {
            "image_topic": self._image_topic,
            "pixel_points": [{"u": p[0], "v": p[1]} for p in self._pixel_points],
            "world_points_mm": [{"x": p[0], "y": p[1]} for p in self._world_points],
        }
        json_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        self.get_logger().info(f"Saved point set to {json_path}")

    def _draw_overlay(self, frame: np.ndarray) -> np.ndarray:
        out = frame.copy()

        for idx, (pixel, world) in enumerate(zip(self._pixel_points, self._world_points)):
            x, y = int(round(pixel[0])), int(round(pixel[1]))
            cv2.circle(out, (x, y), 5, (0, 255, 0), -1)
            cv2.putText(
                out,
                f"{idx}:({world[0]:.0f},{world[1]:.0f})",
                (x + 8, y - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

        lines = [
            f"points: {len(self._pixel_points)}",
            "left-click add point  |  s fit+save  |  p save points  |  u undo  |  c clear  |  q quit",
            f"output: {self._output_path}",
        ]

        y = 22
        for text in lines:
            cv2.putText(out, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA)
            y += 24

        return out

    def spin_gui_once(self) -> bool:
        if self._latest_frame is not None:
            frame = self._draw_overlay(self._latest_frame)
            cv2.imshow(self._window_name, frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            return False
        if key == ord("u") and self._pixel_points:
            self._pixel_points.pop()
            self._world_points.pop()
            self.get_logger().info("Removed last point")
        if key == ord("c"):
            self._pixel_points.clear()
            self._world_points.clear()
            self.get_logger().info("Cleared all points")
        if key == ord("s"):
            matrix = self._fit_affine()
            if matrix is not None:
                self._last_matrix = matrix
                self._save_matrix(matrix)
        if key == ord("p"):
            self._save_points_json()

        return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Interactive affine calibration GUI")
    parser.add_argument("--image-topic", default="/camera/image_rect", help="ROS image topic")
    parser.add_argument("--out", default="/ros2_ws/src/affine_transform.npy", help="Output .npy path")
    parser.add_argument("--window-name", default="Affine Calibration", help="OpenCV window name")
    parser.add_argument("--ransac-threshold", type=float, default=4.0, help="RANSAC reprojection threshold in mm")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    rclpy.init()
    node = AffineCalibrationGUI(
        image_topic=args.image_topic,
        output_path=Path(args.out),
        window_name=args.window_name,
        ransac_threshold=float(args.ransac_threshold),
    )

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if not node.spin_gui_once():
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
