#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point
import os
from ament_index_python.packages import get_package_share_directory

class PixelToWorldAffine(Node):
    def __init__(self):
        super().__init__('pixel_to_world_affine_node')

        # Construct the full path to the affine_transform.npy file
        # This assumes the script is run from the workspace root, and the file is in the 'src' directory
        # A more robust way is to place it in a package's share directory if it's a permanent file.
        try:
            # Assumes the npy file is in the 'src' folder of the workspace
            ws_root = os.path.abspath(os.path.join(get_package_share_directory('m1pro_camera'), '..', '..', '..', '..'))
            transform_path = os.path.join(ws_root, 'src', 'affine_transform.npy')
            
            self.affine_matrix = np.load(transform_path)
            self.get_logger().info(f"Successfully loaded affine transformation from {transform_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load affine_transform.npy: {e}")
            self.get_logger().error("Please ensure 'affine_transform.npy' is in the 'src/' directory of your workspace.")
            self.affine_matrix = None
            return

        self.pixel_sub = self.create_subscription(
            Point,
            '/pixel_coordinates',
            self.pixel_callback,
            10)
        
        self.world_point_pub = self.create_publisher(Point, '/world_coordinates', 10)

        self.get_logger().info("Pixel to World (Affine) node started.")
        self.get_logger().info("Listening for pixel coordinates on /pixel_coordinates.")
        self.get_logger().info("Publishing world coordinates on /world_coordinates.")


    def pixel_callback(self, msg):
        if self.affine_matrix is None:
            self.get_logger().warn("Affine matrix not loaded. Cannot perform conversion.")
            return

        # Pixel coordinates from the message
        u, v = msg.x, msg.y

        # Create a homogeneous coordinate for the pixel: (u, v, 1)
        pixel_coord = np.array([u, v, 1])

        # Apply the affine transformation
        # result = M * [u, v, 1]^T
        world_coord = self.affine_matrix.dot(pixel_coord)

        # The result is a 2D point (x, y)
        x, y = world_coord[0], world_coord[1]
        
        self.get_logger().info(f"Pixel ({u:.2f}, {v:.2f}) -> World ({x:.4f}, {y:.4f})")

        # Publish the world coordinate
        world_point_msg = Point()
        world_point_msg.x = x
        world_point_msg.y = y
        world_point_msg.z = 0.0  # Assuming a 2D plane, so z is 0
        self.world_point_pub.publish(world_point_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PixelToWorldAffine()
    if node.affine_matrix is not None:
        rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
