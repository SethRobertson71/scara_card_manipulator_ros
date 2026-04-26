#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseArray, Point
import os
from ament_index_python.packages import get_package_share_directory
from copy import deepcopy
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class CardWorldPublisher(Node):
    def __init__(self):
        super().__init__('card_world_publisher_node')

        self.declare_parameter('transform_mode', 'affine')
        self.declare_parameter('world_frame', 'world')

        self.transform_mode = str(self.get_parameter('transform_mode').value).strip().lower()
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.affine_matrix = None
        self.camera_to_base_matrix = None

        ws_root = os.path.abspath(os.path.join(get_package_share_directory('m1pro_camera'), '..', '..', '..', '..'))

        # Keep affine support available, but default operation is ChArUco transform.
        if self.transform_mode == 'affine':
            try:
                transform_path = os.path.join(ws_root, 'src', 'affine_transform.npy')
                self.affine_matrix = np.load(transform_path)
                self.get_logger().info(f"Using affine transform from {transform_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to load affine_transform.npy: {e}")
                self.get_logger().error("Set transform_mode:=charuco or ensure affine_transform.npy exists.")
                return
        else:
            try:
                transform_path = os.path.join(ws_root, 'src', 'm1pro_camera', 'config', 'camera_to_base.npy')
                self.camera_to_base_matrix = np.load(transform_path)
                if self.camera_to_base_matrix.shape != (4, 4):
                    raise ValueError(f"Expected 4x4 matrix, got {self.camera_to_base_matrix.shape}")
                self.get_logger().info(f"Using ChArUco camera-to-base transform from {transform_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to load camera_to_base.npy: {e}")
                self.get_logger().error("Run charuco_camera_to_base_node first, or set transform_mode:=affine.")
                return

        # Subscriber for card poses in camera frame
        self.create_subscription(
            PoseArray,
            '/skipbo/pick_targets',
            self.camera_poses_callback,
            10)
        
        # Publisher for card poses in world frame
        self.world_poses_pub = self.create_publisher(PoseArray, '/world/card_poses', 10)

        self.get_logger().info("Card World Publisher node started.")
        self.get_logger().info("Listening for camera-frame card poses on /skipbo/pick_targets.")
        self.get_logger().info("Publishing world-frame card poses on /world/card_poses.")

    def camera_poses_callback(self, msg: PoseArray):
        if self.transform_mode == 'affine' and self.affine_matrix is None:
            self.get_logger().warn("Affine matrix not loaded. Cannot perform conversion.")
            return
        if self.transform_mode != 'affine' and self.camera_to_base_matrix is None:
            self.get_logger().warn("ChArUco camera_to_base matrix not loaded. Cannot perform conversion.")
            return

        world_poses = PoseArray()
        world_poses.header.stamp = self.get_clock().now().to_msg()
        world_poses.header.frame_id = self.world_frame

        for idx, camera_pose in enumerate(msg.poses):
            cam_x = camera_pose.position.x
            cam_y = camera_pose.position.y
            cam_z = camera_pose.position.z

            if self.transform_mode == 'affine':
                point_in = np.array([cam_x, cam_y, 1.0])
                world_coord = self.affine_matrix.dot(point_in)
                world_x, world_y, world_z = float(world_coord[0]), float(world_coord[1]), 0.0
            else:
                point_in = np.array([cam_x, cam_y, cam_z, 1.0])
                world_coord = self.camera_to_base_matrix.dot(point_in)
                world_x = float(world_coord[0])
                world_y = float(world_coord[1])
                world_z = float(world_coord[2])

            # Create a new pose in the world frame
            world_pose = deepcopy(camera_pose)
            world_pose.position.x = world_x
            world_pose.position.y = world_y
            world_pose.position.z = world_z

            # Extract yaw from camera pose orientation
            q = camera_pose.orientation
            roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            
            # Set orientation to only the z-axis rotation (yaw)
            qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
            world_pose.orientation.x = qx
            world_pose.orientation.y = qy
            world_pose.orientation.z = qz
            world_pose.orientation.w = qw

            world_poses.poses.append(world_pose)
            self.get_logger().info(
                f"card[{idx}] cam=({cam_x:.1f}, {cam_y:.1f}) "
                f"world=({world_pose.position.x:.3f}, {world_pose.position.y:.3f}, {world_pose.position.z:.3f}) "
                f"yaw={np.degrees(yaw):.3f}"
            )
        
        if world_poses.poses:
            self.world_poses_pub.publish(world_poses)
            #self.get_logger().info(f"Published {len(world_poses.poses)} card poses to /world/card_poses.")


def main(args=None):
    rclpy.init(args=args)
    node = CardWorldPublisher()
    if (node.transform_mode == 'affine' and node.affine_matrix is not None) or (
        node.transform_mode != 'affine' and node.camera_to_base_matrix is not None
    ):
        rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
