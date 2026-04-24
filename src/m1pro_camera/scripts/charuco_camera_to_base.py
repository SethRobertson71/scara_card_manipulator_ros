#!/usr/bin/env python3
"""Estimate camera-to-base transform from a ChArUco board image.

This script assumes the ChArUco board defines the workspace base frame.
It estimates the board pose in the camera frame, then inverts it to get
the transform that maps camera-frame points into base-frame points.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Tuple

import cv2
import numpy as np
import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Estimate camera-to-base transform from a ChArUco image.")
    parser.add_argument("--image", required=True, help="Path to the image containing the ChArUco board")
    parser.add_argument("--camera-info", required=True, help="ROS-style camera calibration YAML file")
    parser.add_argument("--squares-x", type=int, default=7, help="Number of ChArUco squares in X")
    parser.add_argument("--squares-y", type=int, default=5, help="Number of ChArUco squares in Y")
    parser.add_argument("--square-length", type=float, default=0.035, help="Square size in meters")
    parser.add_argument("--marker-length", type=float, default=0.026, help="Marker size in meters")
    parser.add_argument(
        "--dictionary",
        default="DICT_5X5_50",
        help="OpenCV predefined dictionary name, for example DICT_5X5_50",
    )
    parser.add_argument("--min-corners", type=int, default=4, help="Minimum interpolated ChArUco corners required")
    parser.add_argument("--output", default="camera_to_base.npy", help="Where to save the 4x4 transform")
    parser.add_argument("--show", action="store_true", help="Display the detection overlay")
    return parser.parse_args()


def load_camera_model(yaml_path: Path) -> Tuple[np.ndarray, np.ndarray]:
    with yaml_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    camera_matrix_data = data.get("camera_matrix", {}).get("data")
    if camera_matrix_data is None or len(camera_matrix_data) != 9:
        raise ValueError(f"camera_matrix.data missing or invalid in {yaml_path}")

    distortion_data = data.get("distortion_coefficients", {}).get("data", [])

    camera_matrix = np.array(camera_matrix_data, dtype=np.float64).reshape(3, 3)
    distortion_coeffs = np.array(distortion_data, dtype=np.float64).reshape(-1, 1)
    if distortion_coeffs.size == 0:
        distortion_coeffs = np.zeros((5, 1), dtype=np.float64)

    return camera_matrix, distortion_coeffs


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


def detect_charuco_corners(gray: np.ndarray, board, dictionary):
    detector_params = cv2.aruco.DetectorParameters()
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, detector_params)
        marker_corners, marker_ids, rejected = detector.detectMarkers(gray)
    else:
        marker_corners, marker_ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=detector_params)

    if marker_ids is None or len(marker_ids) == 0:
        return marker_corners, marker_ids, rejected, None, None

    interpolation = cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids, gray, board)
    if len(interpolation) == 3:
        _, charuco_corners, charuco_ids = interpolation
    else:
        _, charuco_corners, charuco_ids, _ = interpolation

    return marker_corners, marker_ids, rejected, charuco_corners, charuco_ids


def board_corner_object_points(board) -> np.ndarray:
    if hasattr(board, "getChessboardCorners"):
        corners = board.getChessboardCorners()
    elif hasattr(board, "chessboardCorners"):
        corners = board.chessboardCorners
    else:
        raise RuntimeError("Unable to access ChArUco board corner coordinates")
    return np.asarray(corners, dtype=np.float64)


def estimate_pose(
    board,
    charuco_corners: np.ndarray,
    charuco_ids: np.ndarray,
    camera_matrix: np.ndarray,
    distortion_coeffs: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    object_points_all = board_corner_object_points(board)
    ids = charuco_ids.flatten().astype(int)
    object_points = object_points_all[ids]
    image_points = charuco_corners.reshape(-1, 2).astype(np.float64)

    if len(object_points) < 4:
        raise RuntimeError("Need at least 4 ChArUco corners to estimate a pose")

    success, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        distortion_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success:
        raise RuntimeError("cv2.solvePnP failed to estimate the board pose")

    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, distortion_coeffs)
    projected = projected.reshape(-1, 2)
    error = np.linalg.norm(projected - image_points, axis=1).mean()
    return rvec, tvec, object_points, float(error)


def transform_from_pose(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    rotation, _ = cv2.Rodrigues(rvec)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = tvec.reshape(3)
    return transform


def main() -> int:
    args = parse_args()

    image_path = Path(args.image)
    camera_info_path = Path(args.camera_info)
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"Failed to read image: {image_path}")

    camera_matrix, distortion_coeffs = load_camera_model(camera_info_path)
    dictionary = load_dictionary(args.dictionary)
    board = create_charuco_board(args.squares_x, args.squares_y, args.square_length, args.marker_length, dictionary)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    marker_corners, marker_ids, rejected, charuco_corners, charuco_ids = detect_charuco_corners(gray, board, dictionary)

    if charuco_corners is None or charuco_ids is None or len(charuco_ids) < args.min_corners:
        raise RuntimeError(
            f"Detected only {0 if charuco_ids is None else len(charuco_ids)} ChArUco corners; "
            f"need at least {args.min_corners}"
        )

    rvec, tvec, object_points, reprojection_error = estimate_pose(
        board,
        charuco_corners,
        charuco_ids,
        camera_matrix,
        distortion_coeffs,
    )

    t_camera_board = transform_from_pose(rvec, tvec)
    t_board_camera = np.linalg.inv(t_camera_board)

    np.save(args.output, t_board_camera)

    print("ChArUco pose estimated successfully")
    print(f"Image: {image_path}")
    print(f"Detected ChArUco corners: {len(charuco_ids)}")
    print(f"Mean reprojection error: {reprojection_error:.3f} px")
    print("\nT_camera_board (board/base -> camera):")
    print(t_camera_board)
    print("\nT_board_camera (camera -> board/base):")
    print(t_board_camera)
    print(f"\nSaved camera-to-base transform to {args.output}")

    if args.show:
        overlay = image.copy()
        cv2.aruco.drawDetectedMarkers(overlay, marker_corners, marker_ids)
        cv2.aruco.drawDetectedCornersCharuco(overlay, charuco_corners, charuco_ids)
        cv2.drawFrameAxes(overlay, camera_matrix, distortion_coeffs, rvec, tvec, args.square_length * 2.0)
        cv2.imshow("charuco pose", overlay)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())