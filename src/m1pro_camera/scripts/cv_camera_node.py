#!/usr/bin/env python3

import copy
from pathlib import Path
from urllib.parse import urlparse

import cv2
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from sensor_msgs.srv import SetCameraInfo


def _device_from_param(device_param: str):
    if device_param.startswith('/dev/video'):
        suffix = device_param.replace('/dev/video', '')
        if suffix.isdigit():
            return int(suffix)
    return device_param


def _camera_info_from_yaml(yaml_path: Path, frame_id: str) -> CameraInfo:
    info = CameraInfo()
    info.header.frame_id = frame_id

    if not yaml_path.exists():
        return info

    with yaml_path.open('r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}

    info.width = int(data.get('image_width', 0))
    info.height = int(data.get('image_height', 0))
    info.distortion_model = str(data.get('distortion_model', 'plumb_bob'))

    distortion = data.get('distortion_coefficients', {}).get('data', [])
    info.d = [float(value) for value in distortion]

    camera_matrix = data.get('camera_matrix', {}).get('data', [])
    if len(camera_matrix) == 9:
        info.k = [float(value) for value in camera_matrix]

    rectification = data.get('rectification_matrix', {}).get('data', [])
    if len(rectification) == 9:
        info.r = [float(value) for value in rectification]

    projection = data.get('projection_matrix', {}).get('data', [])
    if len(projection) == 12:
        info.p = [float(value) for value in projection]

    return info


def _camera_info_to_yaml_dict(camera_info: CameraInfo, camera_name: str) -> dict:
    return {
        'image_width': int(camera_info.width),
        'image_height': int(camera_info.height),
        'camera_name': camera_name,
        'camera_matrix': {'rows': 3, 'cols': 3, 'data': [float(v) for v in camera_info.k]},
        'distortion_model': camera_info.distortion_model,
        'distortion_coefficients': {
            'rows': 1,
            'cols': len(camera_info.d),
            'data': [float(v) for v in camera_info.d],
        },
        'rectification_matrix': {'rows': 3, 'cols': 3, 'data': [float(v) for v in camera_info.r]},
        'projection_matrix': {'rows': 3, 'cols': 4, 'data': [float(v) for v in camera_info.p]},
    }


class OpenCVCameraNode(Node):
    def __init__(self):
        super().__init__('usb_cam', automatically_declare_parameters_from_overrides=True)

        self.bridge = CvBridge()

        self.camera_name = str(self.get_parameter('camera_name').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.device_name = str(self.get_parameter('video_device').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.framerate = float(self.get_parameter('framerate').value)
        self.camera_info_url = str(self.get_parameter('camera_info_url').value)

        self.camera_info_path = None
        if self.camera_info_url.startswith('file://'):
            parsed = urlparse(self.camera_info_url)
            self.camera_info_path = Path(parsed.path)

        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = self.frame_id

        if self.camera_info_path is not None:
            self.camera_info_msg = _camera_info_from_yaml(self.camera_info_path, self.frame_id)
            if self.camera_info_msg.width == 0:
                self.camera_info_msg.width = self.image_width
            if self.camera_info_msg.height == 0:
                self.camera_info_msg.height = self.image_height

        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)
        self.set_camera_info_srv = self.create_service(
            SetCameraInfo,
            '/camera/set_camera_info',
            self._handle_set_camera_info,
        )

        self.capture = cv2.VideoCapture(_device_from_param(self.device_name), cv2.CAP_V4L2)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.image_width))
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.image_height))
        if self.framerate > 0.0:
            self.capture.set(cv2.CAP_PROP_FPS, float(self.framerate))

        if not self.capture.isOpened():
            raise RuntimeError(f'Unable to open camera device: {self.device_name}')

        period_s = 1.0 / self.framerate if self.framerate > 0.0 else 1.0 / 30.0
        self.timer = self.create_timer(period_s, self._publish_frame)

    def _handle_set_camera_info(self, request: SetCameraInfo.Request, response: SetCameraInfo.Response):
        new_info = copy.deepcopy(request.camera_info)
        new_info.header.frame_id = self.frame_id
        self.camera_info_msg = new_info

        if self.camera_info_path is not None:
            self.camera_info_path.parent.mkdir(parents=True, exist_ok=True)
            with self.camera_info_path.open('w', encoding='utf-8') as handle:
                yaml.safe_dump(
                    _camera_info_to_yaml_dict(self.camera_info_msg, self.camera_name),
                    handle,
                    sort_keys=False,
                )

        response.success = True
        response.status_message = 'camera info updated'
        return response

    def _publish_frame(self):
        success, frame = self.capture.read()
        if not success:
            self.get_logger().warning('Failed to capture frame from camera device.')
            return

        stamp = self.get_clock().now().to_msg()
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self.frame_id

        cam_info = copy.deepcopy(self.camera_info_msg)
        cam_info.header.stamp = stamp
        cam_info.header.frame_id = self.frame_id

        if cam_info.width == 0:
            cam_info.width = image_msg.width
        if cam_info.height == 0:
            cam_info.height = image_msg.height

        self.image_pub.publish(image_msg)
        self.camera_info_pub.publish(cam_info)

    def destroy_node(self):
        if hasattr(self, 'capture') and self.capture is not None:
            self.capture.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = OpenCVCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
