#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control_msgs.action import FollowJointTrajectory
from m1pro_bringup.srv import JointMovJ
import math

class M1ProTrajectoryController(Node):
    def __init__(self):
        super().__init__('m1pro_arm_controller')
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'm1pro_arm_controller/follow_joint_trajectory',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self._joint_mov_j = self.create_client(JointMovJ, '/bringup/srv/JointMovJ')
        self.get_logger().info('M1Pro Trajectory Controller ready')

    def goal_callback(self, goal):
        self.get_logger().info('Received trajectory goal')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal):
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing trajectory...')
        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names

        # Only send final waypoint — avoids jitter from rapid sequential JointMovJ calls
        for point in [trajectory.points[-1]]:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            positions = dict(zip(joint_names, point.positions))

            # ROS joints -> firmware degrees
            # joint1=zslide(m->mm), joint2=base, joint3=elbow, joint4=eef
            j1_deg = math.degrees(positions.get('joint2', 0.0))
            j2_deg = math.degrees(positions.get('joint3', 0.0))
            j3_mm  = positions.get('joint1', 0.0) * 1000.0
            j4_deg = math.degrees(positions.get('joint4', 0.0))

            self.get_logger().info(
                f'Sending JointMovJ: J1={j1_deg:.2f} J2={j2_deg:.2f} J3={j3_mm:.2f} J4={j4_deg:.2f}'
            )

            req = JointMovJ.Request()
            req.j1 = j1_deg
            req.j2 = j2_deg
            req.j3 = j3_mm
            req.j4 = j4_deg

            if not self._joint_mov_j.wait_for_service(timeout_sec=3.0):
                self.get_logger().error('JointMovJ service not available')
                goal_handle.abort()
                return FollowJointTrajectory.Result()

            future = self._joint_mov_j.call_async(req)
            await future
            if future.result().res != 0:
                self.get_logger().error(f'JointMovJ failed: {future.result().res}')
                goal_handle.abort()
                return FollowJointTrajectory.Result()

        goal_handle.succeed()
        self.get_logger().info('Trajectory execution complete')
        return FollowJointTrajectory.Result()

def main():
    rclpy.init()
    node = M1ProTrajectoryController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
