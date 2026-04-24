#!/usr/bin/env python3

from pathlib import Path
from urllib.parse import urlparse

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo


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


class CameraInfoServiceProxy(Node):
    def __init__(self):
        super().__init__('camera_info_proxy', automatically_declare_parameters_from_overrides=True)

        self.camera_name = str(self.get_parameter('camera_name').value)
        self.camera_info_url = str(self.get_parameter('camera_info_url').value)
        self.camera_info_path = None

        if self.camera_info_url.startswith('file://'):
            parsed = urlparse(self.camera_info_url)
            self.camera_info_path = Path(parsed.path)

        self.create_service(SetCameraInfo, 'set_camera_info', self._handle_set_camera_info)

    def _handle_set_camera_info(self, request: SetCameraInfo.Request, response: SetCameraInfo.Response):
        if self.camera_info_path is None:
            response.success = False
            response.status_message = 'camera_info_url is not configured as a file:// path'
            return response

        self.camera_info_path.parent.mkdir(parents=True, exist_ok=True)
        with self.camera_info_path.open('w', encoding='utf-8') as handle:
            yaml.safe_dump(
                _camera_info_to_yaml_dict(request.camera_info, self.camera_name),
                handle,
                sort_keys=False,
            )

        response.success = True
        response.status_message = 'camera info updated'
        return response


def main():
    rclpy.init()
    node = CameraInfoServiceProxy()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()