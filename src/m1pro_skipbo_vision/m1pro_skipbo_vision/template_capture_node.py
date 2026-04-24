#!/usr/bin/env python3
"""
Simple Skip-Bo template capture node.

Usage pattern:
- Place exactly one target card in view.
- Set target_number (1..12).
- Node captures number ROI templates and exits after required_samples.

This is intentionally simple and can be run repeatedly for each number.
"""

from __future__ import annotations

import os
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def order_points(pts: np.ndarray) -> np.ndarray:
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect


class TemplateCaptureNode(Node):

    def __init__(self) -> None:
        super().__init__("template_capture_node")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("output_dir", "/ros2_ws/models/skipbo_templates")
        self.declare_parameter("target_number", 1)
        self.declare_parameter("required_samples", 4)
        self.declare_parameter("capture_interval_sec", 0.8)

        self.declare_parameter("white_s_max", 45)
        self.declare_parameter("white_v_min", 165)
        self.declare_parameter("min_card_area", 5000)
        self.declare_parameter("card_aspect_min", 0.55)
        self.declare_parameter("card_aspect_max", 0.85)

        self.declare_parameter("rectified_width", 200)
        self.declare_parameter("rectified_height", 300)
        self.declare_parameter("number_roi_x", 0.54)
        self.declare_parameter("number_roi_y", 0.54)
        self.declare_parameter("number_roi_w", 0.40)
        self.declare_parameter("number_roi_h", 0.40)
        self.declare_parameter("roi_mode", "auto")
        self.declare_parameter("auto_roi_sat_min", 60)
        self.declare_parameter("auto_roi_val_min", 40)
        self.declare_parameter("auto_roi_min_area_frac", 0.003)
        self.declare_parameter("auto_roi_max_area_frac", 0.25)
        self.declare_parameter("auto_roi_padding", 8)
        self.declare_parameter("debug_dir", "")
        self.declare_parameter("overlay_topic", "/skipbo/template_overlay")
        self.declare_parameter("rectified_overlay_topic", "/skipbo/template_rectified")
        self.declare_parameter("show_fallback_roi", True)
        self.declare_parameter("fallback_roi_x", 0.35)
        self.declare_parameter("fallback_roi_y", 0.35)
        self.declare_parameter("fallback_roi_w", 0.30)
        self.declare_parameter("fallback_roi_h", 0.30)

        image_topic = self.get_parameter("image_topic").value
        self.output_dir = str(self.get_parameter("output_dir").value)
        self.target_number = int(self.get_parameter("target_number").value)
        self.required_samples = max(1, int(self.get_parameter("required_samples").value))
        self.capture_interval = max(0.1, float(self.get_parameter("capture_interval_sec").value))

        self.white_s_max = int(self.get_parameter("white_s_max").value)
        self.white_v_min = int(self.get_parameter("white_v_min").value)
        self.min_card_area = int(self.get_parameter("min_card_area").value)
        self.card_aspect_min = float(self.get_parameter("card_aspect_min").value)
        self.card_aspect_max = float(self.get_parameter("card_aspect_max").value)

        self.rect_w = int(self.get_parameter("rectified_width").value)
        self.rect_h = int(self.get_parameter("rectified_height").value)

        self.roi_x = float(self.get_parameter("number_roi_x").value)
        self.roi_y = float(self.get_parameter("number_roi_y").value)
        self.roi_w = float(self.get_parameter("number_roi_w").value)
        self.roi_h = float(self.get_parameter("number_roi_h").value)
        self.roi_mode = str(self.get_parameter("roi_mode").value).strip().lower()
        self.auto_roi_sat_min = int(self.get_parameter("auto_roi_sat_min").value)
        self.auto_roi_val_min = int(self.get_parameter("auto_roi_val_min").value)
        self.auto_roi_min_area_frac = float(self.get_parameter("auto_roi_min_area_frac").value)
        self.auto_roi_max_area_frac = float(self.get_parameter("auto_roi_max_area_frac").value)
        self.auto_roi_padding = int(self.get_parameter("auto_roi_padding").value)

        debug_dir = str(self.get_parameter("debug_dir").value)
        overlay_topic = str(self.get_parameter("overlay_topic").value)
        rectified_overlay_topic = str(self.get_parameter("rectified_overlay_topic").value)
        self.show_fallback_roi = bool(self.get_parameter("show_fallback_roi").value)
        self.fallback_roi_x = float(self.get_parameter("fallback_roi_x").value)
        self.fallback_roi_y = float(self.get_parameter("fallback_roi_y").value)
        self.fallback_roi_w = float(self.get_parameter("fallback_roi_w").value)
        self.fallback_roi_h = float(self.get_parameter("fallback_roi_h").value)

        os.makedirs(self.output_dir, exist_ok=True)
        self.debug_dir = debug_dir or os.path.join(self.output_dir, "debug")
        os.makedirs(self.debug_dir, exist_ok=True)

        if self.target_number < 1 or self.target_number > 12:
            self.get_logger().error("target_number must be in [1, 12]")
            raise ValueError("Invalid target_number")

        self.bridge = CvBridge()
        self.saved = 0
        self.last_capture_ts = 0.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, qos)
        self.overlay_pub = self.create_publisher(Image, overlay_topic, qos)
        self.rectified_overlay_pub = self.create_publisher(Image, rectified_overlay_topic, qos)

        self.get_logger().info(
            f"Template capture started: target={self.target_number}, "
            f"samples={self.required_samples}, output_dir={self.output_dir}"
        )

    def _choose_roi_rect(self, card_bgr: np.ndarray) -> tuple[int, int, int, int, str]:
        h, w = card_bgr.shape[:2]

        def fixed_rect() -> tuple[int, int, int, int]:
            x = int(self.roi_x * w)
            y = int(self.roi_y * h)
            rw = int(self.roi_w * w)
            rh = int(self.roi_h * h)
            x = min(max(0, x), w - 1)
            y = min(max(0, y), h - 1)
            rw = min(max(1, rw), w - x)
            rh = min(max(1, rh), h - y)
            return x, y, rw, rh

        if self.roi_mode not in ("auto", "fixed"):
            self.roi_mode = "auto"

        if self.roi_mode == "fixed":
            x, y, rw, rh = fixed_rect()
            return x, y, rw, rh, "fixed"

        hsv = cv2.cvtColor(card_bgr, cv2.COLOR_BGR2HSV)
        sat = hsv[:, :, 1]
        val = hsv[:, :, 2]

        # Colored print tends to have meaningful saturation even on white cards.
        color_mask = cv2.inRange(sat, self.auto_roi_sat_min, 255)
        value_mask = cv2.inRange(val, self.auto_roi_val_min, 255)
        mask = cv2.bitwise_and(color_mask, value_mask)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        card_area = float(w * h)
        min_area = self.auto_roi_min_area_frac * card_area
        max_area = self.auto_roi_max_area_frac * card_area

        best = None
        best_area = 0.0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area or area > max_area:
                continue
            x, y, rw, rh = cv2.boundingRect(contour)
            # Prefer blobs not glued to the border.
            border_touch = (x <= 1 or y <= 1 or (x + rw) >= (w - 1) or (y + rh) >= (h - 1))
            if border_touch:
                continue
            if area > best_area:
                best = (x, y, rw, rh)
                best_area = area

        if best is None:
            x, y, rw, rh = fixed_rect()
            return x, y, rw, rh, "fixed_fallback"

        x, y, rw, rh = best
        pad = max(0, self.auto_roi_padding)
        x = max(0, x - pad)
        y = max(0, y - pad)
        rw = min(w - x, rw + 2 * pad)
        rh = min(h - y, rh + 2 * pad)
        return x, y, rw, rh, "auto"

    def _find_card_and_roi(self, frame_bgr: np.ndarray) -> Optional[tuple[np.ndarray, np.ndarray, np.ndarray, tuple[int, int, int, int], np.ndarray, str]]:
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsv, (0, 0, self.white_v_min), (180, self.white_s_max, 255))

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        best = None
        best_area = 0.0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_card_area:
                continue

            rect = cv2.minAreaRect(contour)
            (_, _), (rw, rh), _ = rect
            if rw <= 1.0 or rh <= 1.0:
                continue

            short_side = min(rw, rh)
            long_side = max(rw, rh)
            aspect = short_side / long_side
            if aspect < self.card_aspect_min or aspect > self.card_aspect_max:
                continue

            if area > best_area:
                best = rect
                best_area = area

        if best is None:
            return None

        box = cv2.boxPoints(best)
        src = order_points(box.astype(np.float32))
        dst = np.array(
            [
                [0, 0],
                [self.rect_w - 1, 0],
                [self.rect_w - 1, self.rect_h - 1],
                [0, self.rect_h - 1],
            ],
            dtype=np.float32,
        )

        transform = cv2.getPerspectiveTransform(src, dst)
        card = cv2.warpPerspective(frame_bgr, transform, (self.rect_w, self.rect_h))

        x, y, rw, rh, roi_mode_used = self._choose_roi_rect(card)

        roi = card[y : y + rh, x : x + rw]
        roi_corners = np.array(
            [
                [x, y],
                [x + rw, y],
                [x + rw, y + rh],
                [x, y + rh],
            ],
            dtype=np.float32,
        )
        roi_corners = cv2.perspectiveTransform(roi_corners[None, :, :], np.linalg.inv(transform))[0]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        digit_bin = cv2.adaptiveThreshold(
            blur,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            21,
            7,
        )
        return digit_bin, card, box.astype(np.float32), (x, y, rw, rh), roi_corners, roi_mode_used

    def _publish_overlay(self, frame_bgr: np.ndarray, card_box: np.ndarray, roi_corners: np.ndarray, card_rectified: np.ndarray, roi_rect: tuple[int, int, int, int], roi_mode_used: str) -> None:
        raw_overlay = frame_bgr.copy()
        cv2.polylines(raw_overlay, [card_box.astype(np.int32)], True, (0, 255, 0), 2)
        cv2.polylines(raw_overlay, [roi_corners.astype(np.int32)], True, (0, 255, 255), 2)
        cv2.putText(
            raw_overlay,
            f"target={self.target_number} roi={roi_mode_used}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        rectified_overlay = card_rectified.copy()
        x, y, rw, rh = roi_rect
        cv2.rectangle(rectified_overlay, (x, y), (x + rw, y + rh), (0, 255, 255), 2)
        cv2.putText(
            rectified_overlay,
            f"ROI {x},{y} {rw}x{rh} ({roi_mode_used})",
            (5, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )

        raw_msg = self.bridge.cv2_to_imgmsg(raw_overlay, encoding="bgr8")
        raw_msg.header.stamp = self.get_clock().now().to_msg()
        self.overlay_pub.publish(raw_msg)

        rectified_msg = self.bridge.cv2_to_imgmsg(rectified_overlay, encoding="bgr8")
        rectified_msg.header.stamp = raw_msg.header.stamp
        self.rectified_overlay_pub.publish(rectified_msg)

    def _publish_no_card_overlay(self, frame_bgr: np.ndarray) -> None:
        overlay = frame_bgr.copy()
        fh, fw = overlay.shape[:2]

        if self.show_fallback_roi:
            x = int(self.fallback_roi_x * fw)
            y = int(self.fallback_roi_y * fh)
            rw = int(self.fallback_roi_w * fw)
            rh = int(self.fallback_roi_h * fh)

            x = min(max(0, x), fw - 1)
            y = min(max(0, y), fh - 1)
            rw = min(max(1, rw), fw - x)
            rh = min(max(1, rh), fh - y)

            cv2.rectangle(overlay, (x, y), (x + rw, y + rh), (0, 165, 255), 2)
            cv2.putText(
                overlay,
                "Fallback ROI",
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 165, 255),
                2,
                cv2.LINE_AA,
            )

        cv2.putText(
            overlay,
            "NO CARD DETECTED",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

        raw_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        raw_msg.header.stamp = self.get_clock().now().to_msg()
        self.overlay_pub.publish(raw_msg)

    def image_callback(self, msg: Image) -> None:
        now = time.time()
        if now - self.last_capture_ts < self.capture_interval:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return

        extracted = self._find_card_and_roi(frame)
        if extracted is None:
            self._publish_no_card_overlay(frame)
            return

        digit, card, card_box, roi_rect, roi_corners, roi_mode_used = extracted

        self._publish_overlay(frame, card_box, roi_corners, card, roi_rect, roi_mode_used)

        self.last_capture_ts = now

        ts = int(now * 1000)
        sample_name = f"{self.target_number}_{ts}_{self.saved + 1}.png"
        sample_path = os.path.join(self.output_dir, sample_name)
        cv2.imwrite(sample_path, digit)

        canonical_path = os.path.join(self.output_dir, f"{self.target_number}.png")
        cv2.imwrite(canonical_path, digit)

        debug_card = card.copy()
        x, y, rw, rh = roi_rect
        cv2.rectangle(debug_card, (x, y), (x + rw, y + rh), (0, 255, 255), 2)
        cv2.putText(
            debug_card,
            f"ROI {x},{y} {rw}x{rh} ({roi_mode_used})",
            (5, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )

        debug_card_path = os.path.join(self.debug_dir, f"{self.target_number}_{ts}_{self.saved + 1}_card.png")
        debug_roi_path = os.path.join(self.debug_dir, f"{self.target_number}_{ts}_{self.saved + 1}_roi.png")
        cv2.imwrite(debug_card_path, debug_card)
        cv2.imwrite(debug_roi_path, digit)

        self.saved += 1
        self.get_logger().info(
            f"Saved sample {self.saved}/{self.required_samples}: {sample_path}"
        )
        self.get_logger().info(
            f"Debug outputs: {debug_card_path}, {debug_roi_path}"
        )

        if self.saved >= self.required_samples:
            self.get_logger().info("Template capture complete, shutting down.")
            self.destroy_node()
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TemplateCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
