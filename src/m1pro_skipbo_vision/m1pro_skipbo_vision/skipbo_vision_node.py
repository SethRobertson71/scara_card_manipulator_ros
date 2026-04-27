#!/usr/bin/env python3
"""
Skip-Bo vision node based on classical computer vision.

Pipeline:
1) Segment white card rectangles on a dark table.
2) Reject contours that are not card-sized / card-shaped.
3) Rectify each card to a fixed portrait view.
4) Classify card color group.
5) Match a digit ROI against templates from models/card-templates/templates.

Published topics:
  /skipbo/detections         (vision_msgs/Detection2DArray)
  /skipbo/pick_targets       (geometry_msgs/PoseArray) - camera-frame card poses
  /skipbo/pick_target_labels (std_msgs/StringMultiArray) - labels per target
    /camera/skipbo_annotated   (sensor_msgs/Image)
"""

from __future__ import annotations

import glob
import json
import os
import re
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
    Pose2D,
)
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import TransformStamped


def order_points(pts: np.ndarray) -> np.ndarray:
    rect = np.zeros((4, 2), dtype=np.float32)
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect


class SkipboVisionNode(Node):

    def __init__(self) -> None:
        super().__init__("skipbo_vision_node")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("detections_topic", "/skipbo/detections")
        self.declare_parameter("pick_targets_topic", "/skipbo/pick_targets")
        self.declare_parameter("pick_labels_topic", "/skipbo/pick_target_labels")
        self.declare_parameter("annotated_topic", "/camera/skipbo_annotated")
        self.declare_parameter("annotated_publish_scale", 0.5)
        self.declare_parameter("template_roi_topic", "/camera/skipbo_template_roi")

        self.declare_parameter("white_s_max", 45)
        self.declare_parameter("white_v_min", 165)
        self.declare_parameter("min_card_area", 5000)
        self.declare_parameter("min_card_area_frac", 0.012)
        self.declare_parameter("max_card_area_frac", 0.085)
        self.declare_parameter("card_aspect_min", 0.55)
        self.declare_parameter("card_aspect_max", 0.85)
        self.declare_parameter("min_white_fill", 0.72)

        self.declare_parameter("rectified_width", 200)
        self.declare_parameter("rectified_height", 300)
        self.declare_parameter("min_color_pixels", 120)

        self.declare_parameter("template_dir", "/ros2_ws/models/card-templates/templates")
        self.declare_parameter("template_roi_x", 0.10)
        self.declare_parameter("template_roi_y", 0.14)
        self.declare_parameter("template_roi_w", 0.80)
        self.declare_parameter("template_roi_h", 0.72)
        self.declare_parameter("template_roi_scales", [0.9, 1.0, 1.1])
        self.declare_parameter("min_template_score", 0.45)
        self.declare_parameter("match_upside_down_cards", True)
        self.declare_parameter("use_glare_robust_preprocess", True)
        self.declare_parameter("use_color_number_prior", True)
        self.declare_parameter("color_number_prior_weight", 0.35)
        self.declare_parameter("color_number_consistency_bonus", 1.12)
        self.declare_parameter("color_number_mismatch_penalty", 0.90)
        self.declare_parameter("enable_overlap_split", True)
        self.declare_parameter("overlap_cluster_min_area_frac", 0.04)
        self.declare_parameter("overlap_cluster_max_area_frac", 0.40)
        self.declare_parameter("watershed_dist_peak_ratio", 0.48)
        self.declare_parameter("watershed_min_peak_area_px", 120)

        self.declare_parameter("workspace_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("workspace_poses_topic", "/skipbo/workspace_poses")

        image_topic = self.get_parameter("image_topic").value
        publish_rate = float(self.get_parameter("publish_rate").value)
        detections_topic = self.get_parameter("detections_topic").value
        pick_targets_topic = self.get_parameter("pick_targets_topic").value
        pick_labels_topic = self.get_parameter("pick_labels_topic").value
        annotated_topic = self.get_parameter("annotated_topic").value
        self.annotated_publish_scale = float(
            self.get_parameter("annotated_publish_scale").value
        )
        template_roi_topic = self.get_parameter("template_roi_topic").value

        self.white_s_max = int(self.get_parameter("white_s_max").value)
        self.white_v_min = int(self.get_parameter("white_v_min").value)
        self.min_card_area = int(self.get_parameter("min_card_area").value)
        self.min_card_area_frac = float(self.get_parameter("min_card_area_frac").value)
        self.max_card_area_frac = float(self.get_parameter("max_card_area_frac").value)
        self.card_aspect_min = float(self.get_parameter("card_aspect_min").value)
        self.card_aspect_max = float(self.get_parameter("card_aspect_max").value)
        self.min_white_fill = float(self.get_parameter("min_white_fill").value)

        self.rect_w = int(self.get_parameter("rectified_width").value)
        self.rect_h = int(self.get_parameter("rectified_height").value)
        self.min_color_pixels = int(self.get_parameter("min_color_pixels").value)

        self.template_dir = str(self.get_parameter("template_dir").value)
        self.template_roi_x = float(self.get_parameter("template_roi_x").value)
        self.template_roi_y = float(self.get_parameter("template_roi_y").value)
        self.template_roi_w = float(self.get_parameter("template_roi_w").value)
        self.template_roi_h = float(self.get_parameter("template_roi_h").value)
        self.template_roi_scales = [
            float(v) for v in self.get_parameter("template_roi_scales").value
        ]
        if 1.0 not in self.template_roi_scales:
            self.template_roi_scales.append(1.0)
        self.template_roi_scales = sorted(
            s for s in self.template_roi_scales if s >= 0.7 and s <= 1.3
        )
        if not self.template_roi_scales:
            self.template_roi_scales = [1.0]
        self.min_template_score = float(self.get_parameter("min_template_score").value)
        self.match_upside_down_cards = bool(self.get_parameter("match_upside_down_cards").value)
        self.use_glare_robust_preprocess = bool(
            self.get_parameter("use_glare_robust_preprocess").value
        )
        self.use_color_number_prior = bool(self.get_parameter("use_color_number_prior").value)
        self.color_number_prior_weight = float(
            self.get_parameter("color_number_prior_weight").value
        )
        self.color_number_consistency_bonus = float(
            self.get_parameter("color_number_consistency_bonus").value
        )
        self.color_number_mismatch_penalty = float(
            self.get_parameter("color_number_mismatch_penalty").value
        )
        self.enable_overlap_split = bool(self.get_parameter("enable_overlap_split").value)
        self.overlap_cluster_min_area_frac = float(
            self.get_parameter("overlap_cluster_min_area_frac").value
        )
        self.overlap_cluster_max_area_frac = float(
            self.get_parameter("overlap_cluster_max_area_frac").value
        )
        self.watershed_dist_peak_ratio = float(
            self.get_parameter("watershed_dist_peak_ratio").value
        )
        self.watershed_min_peak_area_px = int(
            self.get_parameter("watershed_min_peak_area_px").value
        )

        self.workspace_frame = self.get_parameter("workspace_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        workspace_poses_topic = self.get_parameter("workspace_poses_topic").value
        self._last_tf_warn_ns = 0

        self.publish_period = 1.0 / max(publish_rate, 0.1)
        self.last_inference_time = None

        self.bridge = CvBridge()
        self.templates = self._load_templates(self.template_dir)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, qos)
        self.detections_pub = self.create_publisher(Detection2DArray, detections_topic, 10)
        self.pick_targets_pub = self.create_publisher(PoseArray, pick_targets_topic, 10)
        self.pick_labels_pub = self.create_publisher(String, pick_labels_topic, 10)
        self.workspace_poses_pub = self.create_publisher(PoseArray, workspace_poses_topic, 10)
        self.annotated_pub = self.create_publisher(Image, annotated_topic, qos)
        self.template_roi_pub = self.create_publisher(Image, template_roi_topic, qos)

        self.get_logger().info(
            f"Skip-Bo node ready - image_topic={image_topic}, mode=color_plus_template, "
            f"templates={len(self.templates)}, rate={publish_rate}Hz, "
            f"annotated_scale={self.annotated_publish_scale:.2f}"
        )

    def _load_templates(self, template_dir: str) -> Dict[int, np.ndarray]:
        templates: Dict[int, np.ndarray] = {}
        if not template_dir or not os.path.isdir(template_dir):
            self.get_logger().warn(f"Template dir missing: {template_dir}")
            return templates

        for path in glob.glob(os.path.join(template_dir, "*.png")):
            name = os.path.splitext(os.path.basename(path))[0]
            if not re.fullmatch(r"\d+", name):
                continue
            number = int(name)
            if number < 1 or number > 12:
                continue
            img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue
            templates[number] = self._preprocess_digit(img)

        loaded = ", ".join(str(k) for k in sorted(templates.keys())) if templates else "none"
        self.get_logger().info(f"Loaded templates: {loaded}")
        return templates

    def _preprocess_digit(self, gray: np.ndarray) -> np.ndarray:
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        return cv2.adaptiveThreshold(
            blur,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            21,
            7,
        )

    def _preprocess_digit_variants(self, gray: np.ndarray) -> list[np.ndarray]:
        variants = [self._preprocess_digit(gray)]
        if not self.use_glare_robust_preprocess:
            return variants

        norm = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(norm)
        blur = cv2.GaussianBlur(clahe, (5, 5), 0)
        otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
        variants.append(otsu)
        return variants

    def _classify_color(self, card_bgr: np.ndarray) -> Tuple[str, float, Dict[str, float]]:
        h, w = card_bgr.shape[:2]
        x0 = int(0.2 * w)
        y0 = int(0.2 * h)
        x1 = int(0.8 * w)
        y1 = int(0.8 * h)

        center = card_bgr[y0:y1, x0:x1]
        hsv = cv2.cvtColor(center, cv2.COLOR_BGR2HSV)

        # Ignore the white card field by requiring saturation.
        color_mask = cv2.inRange(hsv[:, :, 1], 45, 255)

        green_mask = cv2.inRange(hsv, (35, 50, 40), (90, 255, 255))
        red_mask_1 = cv2.inRange(hsv, (0, 50, 40), (15, 255, 255))
        red_mask_2 = cv2.inRange(hsv, (170, 60, 40), (180, 255, 255))
        purple_mask = cv2.inRange(hsv, (125, 50, 40), (165, 255, 255))

        green_pixels = int(cv2.countNonZero(cv2.bitwise_and(green_mask, color_mask)))
        red_pixels = int(cv2.countNonZero(cv2.bitwise_and(red_mask_1, color_mask)))
        red_pixels += int(cv2.countNonZero(cv2.bitwise_and(red_mask_2, color_mask)))
        purple_pixels = int(cv2.countNonZero(cv2.bitwise_and(purple_mask, color_mask)))

        counts = {
            "green": green_pixels,
            "red": red_pixels,
            "purple": purple_pixels,
        }

        color = max(counts, key=counts.get)
        max_pixels = counts[color]
        total = max(red_pixels + green_pixels + purple_pixels, 1)
        probs = {
            "green": float(green_pixels) / float(total),
            "red": float(red_pixels) / float(total),
            "purple": float(purple_pixels) / float(total),
        }

        if max_pixels < self.min_color_pixels:
            return "unknown", 0.0, {"green": 0.0, "red": 0.0, "purple": 0.0}

        return color, probs[color], probs

    def _number_group(self, number: int) -> str:
        if 1 <= number <= 4:
            return "green"
        if 5 <= number <= 8:
            return "red"
        return "purple"

    def _extract_template_roi(self, card_bgr: np.ndarray) -> np.ndarray:
        h, w = card_bgr.shape[:2]
        x = int(self.template_roi_x * w)
        y = int(self.template_roi_y * h)
        rw = int(self.template_roi_w * w)
        rh = int(self.template_roi_h * h)

        x = min(max(0, x), w - 1)
        y = min(max(0, y), h - 1)
        rw = min(max(1, rw), w - x)
        rh = min(max(1, rh), h - y)

        return card_bgr[y : y + rh, x : x + rw]

    def _extract_template_roi_variants(self, card_bgr: np.ndarray) -> list[np.ndarray]:
        h, w = card_bgr.shape[:2]
        cx = self.template_roi_x + 0.5 * self.template_roi_w
        cy = self.template_roi_y + 0.5 * self.template_roi_h

        rois: list[np.ndarray] = []
        seen_shapes: set[tuple[int, int]] = set()

        for scale in self.template_roi_scales:
            rw_frac = self.template_roi_w * scale
            rh_frac = self.template_roi_h * scale
            x = int((cx - 0.5 * rw_frac) * w)
            y = int((cy - 0.5 * rh_frac) * h)
            rw = int(rw_frac * w)
            rh = int(rh_frac * h)

            x = min(max(0, x), w - 1)
            y = min(max(0, y), h - 1)
            rw = min(max(1, rw), w - x)
            rh = min(max(1, rh), h - y)

            key = (rw, rh)
            if key in seen_shapes:
                continue
            seen_shapes.add(key)
            rois.append(card_bgr[y : y + rh, x : x + rw])

        if not rois:
            rois.append(self._extract_template_roi(card_bgr))
        return rois

    def _publish_template_roi(self, card_bgr: np.ndarray, msg: Image) -> None:
        roi = self._extract_template_roi(card_bgr)
        roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        roi_bin = self._preprocess_digit_variants(roi_gray)[0]
        roi_msg = self.bridge.cv2_to_imgmsg(roi_bin, encoding="mono8")
        roi_msg.header = msg.header
        self.template_roi_pub.publish(roi_msg)

    def _classify_number(
        self,
        card_bgr: np.ndarray,
        color: str,
        color_probs: Dict[str, float],
    ) -> Tuple[Optional[int], float]:
        if not self.templates:
            return None, 0.0

        candidate_map = {
            "green": [1, 2, 3, 4],
            "red": [5, 6, 7, 8],
            "purple": [9, 10, 11, 12],
        }
        if self.use_color_number_prior:
            candidates = list(sorted(self.templates.keys()))
        else:
            candidates = candidate_map.get(color, list(sorted(self.templates.keys())))

        roi_variants: list[np.ndarray] = []
        for roi in self._extract_template_roi_variants(card_bgr):
            roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            roi_bins = self._preprocess_digit_variants(roi_gray)
            for roi_bin in roi_bins:
                roi_variants.append(roi_bin)
                if self.match_upside_down_cards:
                    roi_variants.append(cv2.rotate(roi_bin, cv2.ROTATE_180))

        best_num: Optional[int] = None
        best_score = -1.0

        for num in candidates:
            templ = self.templates.get(num)
            if templ is None:
                continue
            for roi_variant in roi_variants:
                resized = cv2.resize(roi_variant, (templ.shape[1], templ.shape[0]))
                score = float(cv2.matchTemplate(resized, templ, cv2.TM_CCOEFF_NORMED)[0, 0])
                if self.use_color_number_prior:
                    group = self._number_group(num)
                    prior = max(0.0, min(1.0, color_probs.get(group, 0.0)))
                    score *= 1.0 + self.color_number_prior_weight * prior
                    if color != "unknown":
                        if group == color:
                            score *= self.color_number_consistency_bonus
                        else:
                            score *= self.color_number_mismatch_penalty
                if score > best_score:
                    best_score = score
                    best_num = num

        if best_num is None or best_score < self.min_template_score:
            return None, max(best_score, 0.0)

        return best_num, best_score

    def _process_card_contour(
        self,
        frame: np.ndarray,
        contour: np.ndarray,
        frame_area: float,
        msg: Image,
        detections_msg: Detection2DArray,
        annotated: np.ndarray,
    ) -> Optional[Tuple[float, float, float, str, float]]:
        """Process a card contour. Returns (pixel_x, pixel_y, theta, label, confidence) or None."""
        area = cv2.contourArea(contour)
        if area < self.min_card_area:
            return None

        rect = cv2.minAreaRect(contour)
        (_, _), (rw, rh), _theta = rect
        if rw <= 1.0 or rh <= 1.0:
            return None

        short_side = min(rw, rh)
        long_side = max(rw, rh)
        aspect = short_side / long_side
        if aspect < self.card_aspect_min or aspect > self.card_aspect_max:
            return None

        area_frac = area / frame_area
        if area_frac < self.min_card_area_frac or area_frac > self.max_card_area_frac:
            return None

        rect_area = float(rw * rh)
        if rect_area <= 1.0:
            return None
        fill_ratio = area / rect_area
        if fill_ratio < self.min_white_fill:
            return None

        box = cv2.boxPoints(rect)
        src = order_points(box.astype(np.float32))
        center_xy = np.mean(src, axis=0)
        center_x = float(center_xy[0])
        center_y = float(center_xy[1])
        dst = np.array(
            [
                [0, 0],
                [self.rect_w - 1, 0],
                [self.rect_w - 1, self.rect_h - 1],
                [0, self.rect_h - 1],
            ],
            dtype=np.float32,
        )
        src_w = np.linalg.norm(src[1] - src[0])
        src_h = np.linalg.norm(src[3] - src[0])
        if src_w > src_h:
            # Landscape quad: rotate mapping to keep the rectified output portrait.
            dst = np.array(
                [
                    [0, self.rect_h - 1],
                    [0, 0],
                    [self.rect_w - 1, 0],
                    [self.rect_w - 1, self.rect_h - 1],
                ],
                dtype=np.float32,
            )

        transform = cv2.getPerspectiveTransform(src, dst)
        card = cv2.warpPerspective(frame, transform, (self.rect_w, self.rect_h))

        self._publish_template_roi(card, msg)

        color, color_score, color_probs = self._classify_color(card)
        number, number_score = self._classify_number(card, color, color_probs)

        if number is None:
            label = f"{color}_unknown"
            score = color_score * 0.5
        else:
            label = f"{color}_{number}"
            score = color_score * number_score

        det = self._build_detection(
            msg,
            center_x,
            center_y,
            float(short_side),
            float(long_side),
            label,
            score,
        )
        detections_msg.detections.append(det)

        box_i = box.astype(int)
        cv2.polylines(annotated, [box_i], True, (0, 0, 255), 2)
        cv2.putText(
            annotated,
            f"{label}", #:{score:.2f}
            (box_i[0][0], box_i[0][1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.5,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.circle(annotated, (int(center_x), int(center_y)), 4, (0, 255, 0), -1)
        # Use the long card edge as orientation, apply -90 degree offset,
        # and normalize to [0, pi) i.e. [0, 180) degrees.
        long_vec = (src[3] - src[0]) if src_h >= src_w else (src[1] - src[0])
        theta_raw = np.arctan2(long_vec[1], long_vec[0]) - (0.5 * np.pi)
        theta_rad = float((-theta_raw) % np.pi)
        return (center_x, center_y, theta_rad, label, score)

    def _split_overlap_cluster(
        self,
        frame: np.ndarray,
        white_mask: np.ndarray,
        contour: np.ndarray,
    ) -> list[np.ndarray]:
        x, y, w, h = cv2.boundingRect(contour)
        if w < 20 or h < 20:
            return []

        roi_mask = np.zeros((h, w), dtype=np.uint8)
        shifted = contour.copy()
        shifted[:, :, 0] -= x
        shifted[:, :, 1] -= y
        cv2.drawContours(roi_mask, [shifted], -1, 255, thickness=cv2.FILLED)

        white_roi = white_mask[y : y + h, x : x + w]
        cluster = cv2.bitwise_and(white_roi, roi_mask)
        if cv2.countNonZero(cluster) < self.min_card_area:
            return []

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        cluster = cv2.morphologyEx(cluster, cv2.MORPH_OPEN, kernel)

        dist = cv2.distanceTransform(cluster, cv2.DIST_L2, 5)
        max_dist = float(dist.max())
        if max_dist <= 0.0:
            return []

        _, sure_fg = cv2.threshold(
            dist,
            self.watershed_dist_peak_ratio * max_dist,
            255,
            cv2.THRESH_BINARY,
        )
        sure_fg = sure_fg.astype(np.uint8)

        num_cc, cc_labels, stats, _ = cv2.connectedComponentsWithStats(sure_fg)
        for cc_id in range(1, num_cc):
            if stats[cc_id, cv2.CC_STAT_AREA] < self.watershed_min_peak_area_px:
                sure_fg[cc_labels == cc_id] = 0

        num_cc, cc_labels, _stats, _centroids = cv2.connectedComponentsWithStats(sure_fg)
        if num_cc <= 2:
            return []

        sure_bg = cv2.dilate(cluster, kernel, iterations=2)
        unknown = cv2.subtract(sure_bg, sure_fg)

        markers = cc_labels + 1
        markers[unknown == 255] = 0

        ws_img = frame[y : y + h, x : x + w].copy()
        markers = cv2.watershed(ws_img, markers)

        sub_contours: list[np.ndarray] = []
        for label_id in np.unique(markers):
            if label_id <= 1:
                continue
            region = np.zeros_like(cluster, dtype=np.uint8)
            region[markers == label_id] = 255
            cnts, _ = cv2.findContours(region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                continue

            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) < self.min_card_area:
                continue

            c = c.astype(np.int32)
            c[:, :, 0] += x
            c[:, :, 1] += y
            sub_contours.append(c)

        return sub_contours

    def _build_detection(
        self,
        msg: Image,
        x: float,
        y: float,
        w: float,
        h: float,
        label: str,
        score: float,
    ) -> Detection2D:
        det = Detection2D()
        det.header = msg.header
        det.bbox = BoundingBox2D()
        det.bbox.center = Pose2D()
        det.bbox.center.position.x = x
        det.bbox.center.position.y = y
        det.bbox.center.theta = 0.0
        det.bbox.size_x = w
        det.bbox.size_y = h

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = label
        hyp.hypothesis.score = float(max(0.0, min(score, 1.0)))
        det.results.append(hyp)
        return det

    def image_callback(self, msg: Image) -> None:
        now = self.get_clock().now()
        if self.last_inference_time is not None:
            elapsed = (now - self.last_inference_time).nanoseconds / 1e9
            if elapsed < self.publish_period:
                return
        self.last_inference_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsv, (0, 0, self.white_v_min), (180, self.white_s_max, 255))

        frame_area = float(frame.shape[0] * frame.shape[1])

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections_msg = Detection2DArray()
        detections_msg.header = msg.header
        pick_poses = []  # Collect (x, y, theta, label, confidence) tuples
        annotated = frame.copy()

        for contour in contours:
            target_data = self._process_card_contour(
                frame, contour, frame_area, msg, detections_msg, annotated
            )
            if target_data is not None:
                pick_poses.append(target_data)
                continue

            if not self.enable_overlap_split:
                continue

            area_frac = cv2.contourArea(contour) / frame_area
            if (
                area_frac < self.overlap_cluster_min_area_frac
                or area_frac > self.overlap_cluster_max_area_frac
            ):
                continue

            split_contours = self._split_overlap_cluster(frame, white_mask, contour)
            for sub_contour in split_contours:
                target_data = self._process_card_contour(
                    frame, sub_contour, frame_area, msg, detections_msg, annotated
                )
                if target_data is not None:
                    pick_poses.append(target_data)

        self.detections_pub.publish(detections_msg)

        # Publish camera-frame pick targets
        if pick_poses:
            pose_array = PoseArray()
            pose_array.header = msg.header
            labels_list = []

            for pixel_x, pixel_y, theta, label, confidence in pick_poses:
                # Create pose: position = (pixel_x, pixel_y, 0), ori = quaternion from theta
                pose = Pose()
                pose.position.x = pixel_x
                pose.position.y = pixel_y
                pose.position.z = 0.0  # 2D image space
                # Represent theta as rotation around Z axis
                # theta is in radians, convert to quaternion
                cos_half = np.cos(theta / 2.0)
                sin_half = np.sin(theta / 2.0)
                pose.orientation = Quaternion(
                    x=0.0, y=0.0, z=sin_half, w=cos_half
                )
                pose_array.poses.append(pose)
                labels_list.append(label)

            self.pick_targets_pub.publish(pose_array)
            # Publish labels as JSON array, indexed to match poses
            labels_msg = String(data=json.dumps(labels_list))
            self.pick_labels_pub.publish(labels_msg)

        if pick_poses:
            source_frame = self.camera_frame if self.camera_frame else msg.header.frame_id
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.workspace_frame, source_frame, rclpy.time.Time()
                )
                workspace_poses = PoseArray()
                workspace_poses.header.stamp = self.get_clock().now().to_msg()
                workspace_poses.header.frame_id = self.workspace_frame
                for pose in pose_array.poses:
                    transformed_pose = do_transform_pose(pose, transform)
                    workspace_poses.poses.append(transformed_pose)
                self.workspace_poses_pub.publish(workspace_poses)
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as e:
                # Throttle repetitive TF warnings to once every 5 seconds.
                now_ns = self.get_clock().now().nanoseconds
                if now_ns - self._last_tf_warn_ns > 5_000_000_000:
                    self.get_logger().warn(
                        f"Could not transform poses from '{source_frame}' to "
                        f"'{self.workspace_frame}': {e}"
                    )
                    self._last_tf_warn_ns = now_ns

        if 0.0 < self.annotated_publish_scale < 1.0:
            annotated_h, annotated_w = annotated.shape[:2]
            scaled_w = max(1, int(round(annotated_w * self.annotated_publish_scale)))
            scaled_h = max(1, int(round(annotated_h * self.annotated_publish_scale)))
            annotated = cv2.resize(annotated, (scaled_w, scaled_h), interpolation=cv2.INTER_AREA)

        annotated_img = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
        annotated_img.header = msg.header
        self.annotated_pub.publish(annotated_img)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SkipboVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
