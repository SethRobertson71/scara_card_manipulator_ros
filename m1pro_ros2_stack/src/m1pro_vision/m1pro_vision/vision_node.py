#!/usr/bin/env python3
"""
M1Pro Vision Node
-----------------
Subscribes to /camera/image_raw, runs YOLO inference via OpenVINO,
publishes Detection2DArray and annotated image.

Topics published:
  /detections                  (vision_msgs/Detection2DArray)
  /camera/image_annotated      (sensor_msgs/Image)

Parameters:
  model_path    : path to YOLO model (.pt or OpenVINO .xml)
  device        : OpenVINO device (AUTO, CPU, GPU)
  confidence    : detection confidence threshold (default 0.5)
  image_topic   : input image topic (default /camera/image_raw)
  publish_rate  : max inference rate Hz (default 10)
  classes       : list of class names to filter (empty = all)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2DArray, Detection2D, ObjectHypothesisWithPose,
    BoundingBox2D, Pose2D,
)


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('device', 'AUTO')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('classes', [''])

        model_path    = self.get_parameter('model_path').value
        device        = self.get_parameter('device').value
        self.conf     = self.get_parameter('confidence').value
        image_topic   = self.get_parameter('image_topic').value
        publish_rate  = self.get_parameter('publish_rate').value
        classes_param = self.get_parameter('classes').value
        self.filter_classes = [c for c in classes_param if c]

        self.model = None
        self._load_model(model_path, device)

        self.bridge = CvBridge()
        self.publish_period = 1.0 / publish_rate
        self.last_inference_time = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, qos)
        self.detections_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self.annotated_pub  = self.create_publisher(Image, '/camera/image_annotated', qos)

        self.get_logger().info(
            f'Vision node ready — model: {model_path}, device: {device}, '
            f'conf: {self.conf}, rate: {publish_rate}Hz'
        )

    def _load_model(self, model_path: str, device: str):
        try:
            from ultralytics import YOLO
            self.get_logger().info(f'Loading YOLO model: {model_path}')
            self.model = YOLO(model_path)
            self.ov_device = device
            self.get_logger().info('YOLO model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            self.model = None

    def image_callback(self, msg: Image):
        if self.model is None:
            return

        now = self.get_clock().now()
        if self.last_inference_time is not None:
            elapsed = (now - self.last_inference_time).nanoseconds / 1e9
            if elapsed < self.publish_period:
                return
        self.last_inference_time = now

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model.predict(
                source=cv_image, conf=self.conf, device=self.ov_device, verbose=False
            )

            detections_msg = Detection2DArray()
            detections_msg.header = msg.header

            if results and len(results) > 0:
                result = results[0]
                for box in result.boxes:
                    class_id   = int(box.cls[0])
                    class_name = result.names[class_id]
                    confidence = float(box.conf[0])

                    if self.filter_classes and class_name not in self.filter_classes:
                        continue

                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0
                    w  = x2 - x1
                    h  = y2 - y1

                    det = Detection2D()
                    det.header = msg.header
                    det.bbox = BoundingBox2D()
                    det.bbox.center = Pose2D()
                    det.bbox.center.position.x = cx
                    det.bbox.center.position.y = cy
                    det.bbox.center.theta = 0.0
                    det.bbox.size_x = w
                    det.bbox.size_y = h

                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = class_name
                    hyp.hypothesis.score    = confidence
                    det.results.append(hyp)
                    detections_msg.detections.append(det)

            self.detections_pub.publish(detections_msg)

            if self.annotated_pub.get_subscription_count() > 0:
                annotated = results[0].plot() if results else cv_image
                ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                ann_msg.header = msg.header
                self.annotated_pub.publish(ann_msg)

            if detections_msg.detections:
                self.get_logger().info(
                    f'Detected {len(detections_msg.detections)} objects: '
                    + ', '.join([
                        f'{d.results[0].hypothesis.class_id} ({d.results[0].hypothesis.score:.2f})'
                        for d in detections_msg.detections
                    ])
                )

        except Exception as e:
            self.get_logger().error(f'Inference error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
