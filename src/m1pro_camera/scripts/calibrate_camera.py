import cv2
import numpy as np

def calculate_affine_transform():
    """
    Calculates the affine transformation matrix from pixel to robot coordinates.
    """
    # --- USER INPUT: Enter your point correspondences here ---

    # Add at least 3 corresponding points.
    # More points will likely give a more accurate result.
    # Make sure the points are spread out over the workspace.

    # (u, v) pixel coordinates from the camera image
    pixel_points = np.float32([
        [440, 162],  # Example Point 1
        [440, 420],  # Example Point 2
        [645, 419],  # Example Point 3
        # [u4, v4],  # Add more points if you have them
    ])

    # (x, y) coordinates from the robot's base frame (in millimeters)
    robot_points = np.float32([
        [225, 0],   # Example Point 1
        [350, 0],  # Example Point 2
        [350, 100],  # Example Point 3
        # [x4, y4],   # Add more points if you have them
    ])

    # --- Calculation ---

    if len(pixel_points) < 3 or len(robot_points) < 3:
        print("Error: You need at least 3 points to calculate the transform.")
        return None

    if len(pixel_points) != len(robot_points):
        print("Error: The number of pixel points and robot points must be the same.")
        return None

    # Calculate the affine transformation matrix
    affine_matrix = cv2.getAffineTransform(pixel_points, robot_points)

    print("Calculated Affine Transformation Matrix:")
    print(affine_matrix)
    print("\nThis matrix can be used to transform pixel coordinates to robot coordinates.")

    return affine_matrix

def transform_pixel_to_robot(pixel_coord, matrix):
    """
    Transforms a single pixel coordinate to a robot coordinate using the affine matrix.
    """
    if matrix is None:
        return None

    pixel_np = np.float32([pixel_coord[0], pixel_coord[1], 1])
    robot_coord = np.dot(matrix, pixel_np)
    
    return robot_coord

if __name__ == '__main__':
    # 1. Calculate the transformation matrix
    transform_matrix = calculate_affine_transform()

    if transform_matrix is not None:
        # 2. Example: Transform a new pixel coordinate to a robot coordinate
        # Replace this with a pixel coordinate you want to test
        test_pixel = (400, 300) 

        robot_xy = transform_pixel_to_robot(test_pixel, transform_matrix)

        print(f"\n--- Test Transformation ---")
        print(f"Pixel coordinate {test_pixel} corresponds to Robot coordinate:")
        print(f"X: {robot_xy[0]:.2f} mm")
        print(f"Y: {robot_xy[1]:.2f} mm")

        # You can save the 'transform_matrix' to a file (e.g., using numpy.save)
        # to use it in your main robot application without recalculating it every time.
        np.save("affine_transform.npy", transform_matrix)
        print("\nTransformation matrix saved to 'affine_transform.npy'")
