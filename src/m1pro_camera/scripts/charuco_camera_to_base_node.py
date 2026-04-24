#!/usr/bin/env python3
"""Estimate camera-to-base transform from live ChArUco detections.

The board frame is treated as the base/workspace frame. The node estimates
the board pose in the camera frame, inverts it to camera->base, and publishes
base->camera in TF (parent=base_frame, child=camera_frame).
"""

from __future__ import annotations

import math
from collections import deque
from pathlib import Path
from typing import Deque, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def load_dictionary(dictionary_name: str):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("cv2.aruco is not available in this OpenCV build")
    try:
        dictionary_id = getattr(cv2.aruco, dictionary_name)
    except AttributeError as exc:
        raise ValueError(f"Unknown predefined dictionary: {dictionary_name}") from exc
    return cv2.aruco.getPredefinedDictionary(dictionary_id)


def create_charuco_board(squares_x: int, squares_y: int, square_length: float, marker_length: float, dictionary):
    board = None
    if hasattr(cv2.aruco, "CharucoBoard"):
        try:
            board = cv2.aruco.CharucoBoard((squares_x, squares_y), square_length, marker_length, dictionary)
        except TypeError:
            board = None
    if board is None and hasattr(cv2.aruco, "CharucoBoard_create"):
        board = cv2.aruco.CharucoBoard_create(squares_x, squares_y, square_length, marker_length, dictionary)
    if board is None:
        raise RuntimeError("Could not create a ChArUco board with this OpenCV build")
    return board


def board_corner_object_points(board) -> np.ndarray:
    if hasattr(board, "getChessboardCorners"):
        corners = board.getChessboardCorners()
    elif hasattr(board, "chessboardCorners"):
        corners = board.chessboardCorners
    else:
        raise RuntimeError("Unable to access ChArUco board corner coordinates")
    return np.asarray(corners, dtype=np.float64)


def rotation_matrix_to_quaternion(r: np.ndarray) -> np.ndarray:
    tr = float(r[0, 0] + r[1, 1] + r[2, 2])
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2, 1] - r[1, 2]) / s
        qy = (r[0, 2] - r[2, 0]) / s
        qz = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        qw = (r[2, 1] - r[1, 2]) / s
        qx = 0.25 * s
        qy = (r[0, 1] + r[1, 0]) / s
        qz = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        qw = (r[0, 2] - r[2, 0]) / s
        qx = (r[0, 1] + r[1, 0]) / s
        qy = 0.25 * s
        qz = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        qw = (r[1, 0] - r[0, 1]) / s
        qx = (r[0, 2] + r[2, 0]) / s
        qy = (r[1, 2] + r[2, 1]) / s
        qz = 0.25 * s
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    return q / np.linalg.norm(q)


def solve_transform(
    gray: np.ndarray,
    board,
    dictionary,
    camera_matrix: np.ndarray,
    distortion_coeffs: np.ndarray,
    min_corners: int,
) -> Tuple[np.ndarray, int, float]:
    detector_params = cv2.aruco.DetectorParameters()
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, detector_params)
        marker_corners, marker_ids, _ = detector.detectMarkers(gray)
    else:
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=detector_params)

    if marker_ids is None or len(marker_ids) == 0:
        raise RuntimeError("No ArUco markers detected")

    # OpenCV ChArUco API changed across versions.
    if hasattr(cv2.aruco, "interpolateCornersCharuco"):
        interpolation = cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids, gray, board)
        if len(interpolation) == 3:
            _, charuco_corners, charuco_ids = interpolation
        else:
            _, charuco_corners, charuco_ids, _ = interpolation
    elif hasattr(cv2.aruco, "CharucoDetector"):
        charuco_detector = cv2.aruco.CharucoDetector(board)
        detection = charuco_detector.detectBoard(gray)

        if len(detection) >= 2:
            charuco_corners, charuco_ids = detection[0], detection[1]
        else:
            raise RuntimeError("CharucoDetector.detectBoard returned unexpected output")
    else:
        raise RuntimeError("No supported ChArUco interpolation API found in cv2.aruco")

    if charuco_ids is None or len(charuco_ids) < min_corners:
        detected = 0 if charuco_ids is None else len(charuco_ids)
        raise RuntimeError(f"Detected {detected} ChArUco corners, need at least {min_corners}")

    object_points_all = board_corner_object_points(board)
    ids = charuco_ids.flatten().astype(int)
    object_points = object_points_all[ids]
    image_points = charuco_corners.reshape(-1, 2).astype(np.float64)

    success, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        distortion_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success:
        raise RuntimeError("solvePnP failed")

    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, distortion_coeffs)
    reproj_error = np.linalg.norm(projected.reshape(-1, 2) - image_points, axis=1).mean()

    rotation, _ = cv2.Rodrigues(rvec)
    t_camera_board = np.eye(4, dtype=np.float64)
    t_camera_board[:3, :3] = rotation
    t_camera_board[:3, 3] = tvec.reshape(3)

    t_board_camera = np.linalg.inv(t_camera_board)
    return t_board_camera, len(charuco_ids), float(reproj_error)


class CharucoCameraToBaseNode(Node):
    def __init__(self) -> None:
        super().__init__("charuco_camera_to_base")

        self.declare_parameter("image_topic", "/camera/image_rect")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("base_frame", "workspace_base")
        self.declare_parameter("camera_frame", "")
        self.declare_parameter("squares_x", 7)
        self.declare_parameter("squares_y", 5)
        self.declare_parameter("square_length", 0.035)
        self.declare_parameter("marker_length", 0.026)
        self.declare_parameter("dictionary", "DICT_5X5_50")
        self.declare_parameter("min_corners", 8)
        self.declare_parameter("max_reprojection_error", 3.0)
        self.declare_parameter("stable_samples", 10)
        self.declare_parameter("output_path", "/ros2_ws/src/m1pro_camera/config/camera_to_base.npy")
        self.declare_parameter("use_saved_transform", True)

        image_topic = str(self.get_parameter("image_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        squares_x = int(self.get_parameter("squares_x").value)
        squares_y = int(self.get_parameter("squares_y").value)
        square_length = float(self.get_parameter("square_length").value)
        marker_length = float(self.get_parameter("marker_length").value)
        dictionary_name = str(self.get_parameter("dictionary").value)
        self.min_corners = int(self.get_parameter("min_corners").value)
        self.max_reprojection_error = float(self.get_parameter("max_reprojection_error").value)
        self.stable_samples = int(self.get_parameter("stable_samples").value)
        self.output_path = Path(str(self.get_parameter("output_path").value))
        self.use_saved_transform = bool(self.get_parameter("use_saved_transform").value)

        self.bridge = CvBridge()
        self.camera_matrix: Optional[np.ndarray] = None
        self.distortion_coeffs: Optional[np.ndarray] = None
        self.solved = False
        self.have_camera_info = False
        self.frames_seen = 0
        self.last_error: Optional[str] = None

        self.translation_buffer: Deque[np.ndarray] = deque(maxlen=self.stable_samples)
        self.quaternion_buffer: Deque[np.ndarray] = deque(maxlen=self.stable_samples)

        self.dictionary = load_dictionary(dictionary_name)
        self.board = create_charuco_board(squares_x, squares_y, square_length, marker_length, self.dictionary)

        self.tf_broadcaster = StaticTransformBroadcaster(self)

        self.camera_info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.on_camera_info, 10)
        self.image_sub = self.create_subscription(Image, image_topic, self.on_image, 10)
        self.status_timer = self.create_timer(5.0, self.on_status_timer)

        self.get_logger().info(f"Listening for images on {image_topic}")
        self.get_logger().info(f"Listening for camera info on {camera_info_topic}")

        if self.use_saved_transform and self.output_path.exists():
            try:
                t_saved = np.load(self.output_path)
                if t_saved.shape != (4, 4):
                    raise ValueError(f"Expected 4x4 matrix, got shape {t_saved.shape}")
                self.publish_static_tf(t_saved)
                self.get_logger().info(f"Loaded and published saved transform from {self.output_path}")
                self.solved = True
            except Exception as exc:
                self.get_logger().warn(f"Failed to load saved transform at {self.output_path}: {exc}")

    def on_camera_info(self, msg: CameraInfo) -> None:
        if len(msg.k) == 9:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if len(msg.d) > 0:
            self.distortion_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        else:
            self.distortion_coeffs = np.zeros((5, 1), dtype=np.float64)
        self.have_camera_info = True

        if not self.camera_frame:
            self.camera_frame = msg.header.frame_id if msg.header.frame_id else "camera_link"

    def on_image(self, msg: Image) -> None:
        self.frames_seen += 1
        if self.solved:
            return
        if self.camera_matrix is None or self.distortion_coeffs is None:
            if not self.camera_frame and msg.header.frame_id:
                self.camera_frame = msg.header.frame_id
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            t_board_camera, corners, reproj_error = solve_transform(
                gray,
                self.board,
                self.dictionary,
                self.camera_matrix,
                self.distortion_coeffs,
                self.min_corners,
            )
        except Exception as exc:
            self.last_error = str(exc)
            self.get_logger().debug(self.last_error)
            return

        if reproj_error > self.max_reprojection_error:
            self.get_logger().warn(
                f"Rejected pose with reprojection error {reproj_error:.2f}px > {self.max_reprojection_error:.2f}px"
            )
            return

        rot = t_board_camera[:3, :3]
        trans = t_board_camera[:3, 3]
        quat = rotation_matrix_to_quaternion(rot)

        if self.quaternion_buffer and np.dot(self.quaternion_buffer[-1], quat) < 0.0:
            quat = -quat

        self.translation_buffer.append(trans)
        self.quaternion_buffer.append(quat)
        self.last_error = None

        self.get_logger().info(
            f"Accepted sample {len(self.translation_buffer)}/{self.stable_samples} "
            f"(corners={corners}, reproj={reproj_error:.2f}px)"
        )

        if len(self.translation_buffer) < self.stable_samples:
            return

        t_avg = np.mean(np.array(self.translation_buffer), axis=0)
        q_avg = np.mean(np.array(self.quaternion_buffer), axis=0)
        q_avg = q_avg / np.linalg.norm(q_avg)

        t_matrix = np.eye(4, dtype=np.float64)
        x, y, z, w = q_avg
        t_matrix[:3, :3] = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )
        t_matrix[:3, 3] = t_avg

        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        np.save(self.output_path, t_matrix)
        self.publish_static_tf(t_matrix)

        self.get_logger().info("Camera-to-base solved and published as static TF.")
        self.get_logger().info(f"Saved transform matrix to {self.output_path}")
        self.solved = True

    def publish_static_tf(self, t_matrix: np.ndarray) -> None:
        rot = t_matrix[:3, :3]
        trans = t_matrix[:3, 3]
        quat = rotation_matrix_to_quaternion(rot)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.base_frame
        tf_msg.child_frame_id = self.camera_frame if self.camera_frame else "camera_link"
        tf_msg.transform.translation.x = float(trans[0])
        tf_msg.transform.translation.y = float(trans[1])
        tf_msg.transform.translation.z = float(trans[2])
        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(tf_msg)

    def on_status_timer(self) -> None:
        if self.solved:
            return
        if not self.have_camera_info:
            self.get_logger().warn("Waiting for camera_info. Check camera_info_topic parameter and camera node state.")
            return
        if self.frames_seen == 0:
            self.get_logger().warn("Waiting for image frames. Check image_topic parameter and camera node state.")
            return
        if self.last_error:
            self.get_logger().warn(f"Waiting for valid ChArUco pose. Last issue: {self.last_error}")
            return
        self.get_logger().warn("Waiting for stable ChArUco samples before publishing TF.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CharucoCameraToBaseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()