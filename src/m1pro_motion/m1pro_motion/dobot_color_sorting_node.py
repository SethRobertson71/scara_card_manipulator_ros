#!/usr/bin/env python3
"""
Color Sorting ROS2 Node with State Machine Workflow

The node implements a three-state workflow to ensure vision data is always
from the home position (400, 0, 240, 0°):

  SCANNING --[detect cards]--> SORTING --[finished]--> RETURNING_HOME
     ↑                             |                           |
     |_________________________[at home]________________________|

- SCANNING: Robot at home, vision publishes pick targets, node accepts them
- SORTING: Robot processing cards from vision snapshot, ignores new vision data
- RETURNING_HOME: Robot moving back to home, ignores new vision data

Uses ROS2 services from bringup node for robot control (no direct sockets).
"""
import json
import math
import time
from enum import Enum

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from m1pro_bringup.srv import MovJ, ToolDO, EnableRobot, ClearError, SpeedFactor, DisableRobot, Sync


class SortingState(Enum):
    """State machine for the color sorting workflow."""
    SCANNING = "scanning"           # At home, ready to accept picks
    SORTING = "sorting"             # Processing cards from current scan
    RETURNING_HOME = "returning_home"  # Moving back to home position


class DobotColorSortingNode(Node):
    def __init__(self) -> None:
        super().__init__('dobot_color_sorting_node')

        self.declare_parameter('speed_factor', 40)
        self.declare_parameter('pick_targets_topic', '/skipbo/pick_targets')
        self.declare_parameter('pick_labels_topic', '/skipbo/pick_target_labels')
        self.declare_parameter('default_color', 'red')
        self.declare_parameter('default_pick_z', 240.0)
        self.declare_parameter('default_pick_r', 0.0)
        self.declare_parameter('bin_red_pos', "-18, -303, 130, -94")
        self.declare_parameter('bin_blue_pos', "108, -311, 130, -92")
        self.declare_parameter('bin_green_pos', "234, -319, 130, -91")
        self.declare_parameter('home_x', 400.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_z', 240.0)
        self.declare_parameter('home_rotation_deg', 0.0)

        pick_targets_topic = self.get_parameter('pick_targets_topic').value
        pick_labels_topic = self.get_parameter('pick_labels_topic').value
        self.default_color = str(self.get_parameter('default_color').value)
        self.default_pick_z = float(self.get_parameter('default_pick_z').value)
        self.default_pick_r = float(self.get_parameter('default_pick_r').value)
        self.speed_factor = int(self.get_parameter('speed_factor').value)

        self.home_pos = {
            'x': float(self.get_parameter('home_x').value),
            'y': float(self.get_parameter('home_y').value),
            'z': float(self.get_parameter('home_z').value),
            'r': float(self.get_parameter('home_rotation_deg').value),
        }

        self.bins = {
            'red': str(self.get_parameter('bin_red_pos').value),
            'blue': str(self.get_parameter('bin_blue_pos').value),
            'green': str(self.get_parameter('bin_green_pos').value),
        }

        # Create ROS service clients for robot control
        self.movj_client = self.create_client(MovJ, '/bringup/srv/MovJ')
        self.tool_do_client = self.create_client(ToolDO, '/bringup/srv/ToolDO')
        self.enable_robot_client = self.create_client(EnableRobot, '/bringup/srv/EnableRobot')
        self.clear_error_client = self.create_client(ClearError, '/bringup/srv/ClearError')
        self.speed_factor_client = self.create_client(SpeedFactor, '/bringup/srv/SpeedFactor')
        self.sync_client = self.create_client(Sync, '/bringup/srv/Sync')
        self.disable_robot_client = self.create_client(DisableRobot, '/bringup/srv/DisableRobot')

        # State machine
        self.state = SortingState.SCANNING
        self.latest_targets = None
        self.latest_labels = []
        self.pending_cards = []
        self.last_stamp_key = None
        self.busy = False
        self.initialization_complete = False

        self.create_subscription(PoseArray, pick_targets_topic, self._targets_cb, 10)
        self.create_subscription(String, pick_labels_topic, self._labels_cb, 10)
        self._init_timer = self.create_timer(0.1, self._initialization_step)
        self.create_timer(0.1, self._process_queue)

        self.get_logger().info('Dobot color sorting node started (state=SCANNING).')

    def _dispatch_request(self, client, request, action_name: str):
        future = client.call_async(request)
        return future

    def _wait_for_future(self, future, action_name: str, timeout_sec: float = 10.0) -> bool:
        start_ns = self.get_clock().now().nanoseconds
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed_sec = (self.get_clock().now().nanoseconds - start_ns) / 1e9
            if elapsed_sec > timeout_sec:
                self.get_logger().error(f'{action_name} timed out after {timeout_sec:.1f}s')
                return False

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'{action_name} failed: {exc}')
            return False

        if response is None:
            self.get_logger().error(f'{action_name} returned no response')
            return False
        if hasattr(response, 'res') and response.res != 0:
            self.get_logger().error(f'{action_name} failed with res={response.res}')
            return False

        self.get_logger().info(f'{action_name} completed successfully')
        return True

    def _initialization_step(self) -> None:
        if self.initialization_complete:
            self._init_timer.cancel()
            return

        if not self.clear_error_client.service_is_ready() or not self.enable_robot_client.service_is_ready() or not self.speed_factor_client.service_is_ready():
            self.get_logger().info('Waiting for bringup services...')
            return

        self.get_logger().info('Services available, initializing...')

        req = ClearError.Request()
        if not self._wait_for_future(self._dispatch_request(self.clear_error_client, req, 'ClearError'), 'ClearError'):
            return

        req = EnableRobot.Request()
        req.args = []
        if not self._wait_for_future(self._dispatch_request(self.enable_robot_client, req, 'EnableRobot'), 'EnableRobot'):
            return

        req = SpeedFactor.Request()
        req.ratio = self.speed_factor
        if not self._wait_for_future(self._dispatch_request(self.speed_factor_client, req, 'SpeedFactor'), 'SpeedFactor'):
            return

        self.initialization_complete = True
        self.get_logger().info('Initialization commands dispatched.')
        self._init_timer.cancel()

    def _targets_cb(self, msg: PoseArray) -> None:
        # Only queue new picks when in SCANNING state (at home)
        if self.state != SortingState.SCANNING:
            self.get_logger().debug(
                f'Received {len(msg.poses)} pick targets but ignoring (state={self.state.value})'
            )
            return
        
        self.latest_targets = msg
        self.get_logger().debug(f'Received {len(msg.poses)} pick targets in SCANNING state')
        self._queue_latest_cards_if_ready()

    def _labels_cb(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
            if isinstance(parsed, list):
                self.latest_labels = [str(item) for item in parsed]
            else:
                self.latest_labels = []
        except Exception:
            self.latest_labels = []
        # Only queue if in SCANNING state
        if self.state == SortingState.SCANNING:
            self._queue_latest_cards_if_ready()

    def _extract_color(self, label: str) -> str:
        text = str(label).lower()
        if 'red' in text:
            return 'red'
        if 'blue' in text:
            return 'blue'
        if 'green' in text:
            return 'green'
        return self.default_color

    @staticmethod
    def _yaw_deg_from_quaternion_zw(z: float, w: float) -> float:
        return math.degrees(2.0 * math.atan2(z, w))

    def _pose_to_pos_string(self, pose) -> str:
        x = float(pose.position.x)
        y = float(pose.position.y)
        z = float(pose.position.z)
        if abs(z) < 1e-6:
            z = self.default_pick_z
        yaw_deg = self._yaw_deg_from_quaternion_zw(
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        if abs(yaw_deg) < 1e-6:
            yaw_deg = self.default_pick_r
        return f'{x:.3f}, {y:.3f}, {z:.3f}, {yaw_deg:.3f}, 1'

    def _queue_latest_cards_if_ready(self) -> None:
        if self.latest_targets is None:
            return

        poses = self.latest_targets.poses
        if not poses:
            return

        stamp = self.latest_targets.header.stamp
        stamp_key = (int(stamp.sec), int(stamp.nanosec), len(poses))
        if stamp_key == self.last_stamp_key:
            return

        labels = self.latest_labels
        for idx, pose in enumerate(poses):
            label = labels[idx] if idx < len(labels) else self.default_color
            self.pending_cards.append(
                {
                    'color': self._extract_color(label),
                    'pos': self._pose_to_pos_string(pose),
                }
            )

        self.last_stamp_key = stamp_key
        num_queued = len(poses)
        self.get_logger().info(f'Queued {num_queued} card targets - transitioning to SORTING')
        self.state = SortingState.SORTING

    def _initialize_robot(self) -> None:
        """Initialize robot via ROS services."""
        self.get_logger().warn('_initialize_robot() is deprecated; startup now uses _initialization_step().')

    def _move_to_position(self, x: float, y: float, z: float, r: float) -> bool:
        """Move robot to position via MovJ service."""
        try:
            # Wait for service if not already done
            if not self.movj_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error('MovJ service not available')
                return False
            
            req = MovJ.Request()
            req.x = x
            req.y = y
            req.z = z
            req.r = r
            req.param_value = []
            
            move_future = self._dispatch_request(self.movj_client, req, f'MovJ({x}, {y}, {z}, {r})')
            if not self._wait_for_future(move_future, f'MovJ({x}, {y}, {z}, {r})'):
                return False

            sync_req = Sync.Request()
            sync_future = self._dispatch_request(self.sync_client, sync_req, 'Sync')
            if not self._wait_for_future(sync_future, 'Sync', timeout_sec=30.0):
                return False

            return True
        except Exception as e:
            self.get_logger().error(f'MovJ error: {e}')
            return False

    def _parse_position_string(self, pos_str: str) -> tuple:
        """Parse position string 'x, y, z, r' into floats."""
        try:
            parts = [float(p.strip()) for p in pos_str.split(',')]
            return tuple(parts[:4])  # x, y, z, r
        except Exception as e:
            self.get_logger().error(f'Failed to parse position string "{pos_str}": {e}')
            return (0, 0, 0, 0)

    def _set_tool_do(self, pin: int, status: int) -> bool:
        """Set tool digital output (pump/solenoid control)."""
        try:
            # Wait for service if not already done
            if not self.tool_do_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error('ToolDO service not available')
                return False
            
            req = ToolDO.Request()
            req.index = pin
            req.status = status
            
            future = self._dispatch_request(self.tool_do_client, req, f'ToolDO({pin}, {status})')
            return self._wait_for_future(future, f'ToolDO({pin}, {status})')
        except Exception as e:
            self.get_logger().error(f'ToolDO error: {e}')
            return False

    def _sort_card(self, card: dict) -> None:
        """Sort a single card."""
        color = card['color']
        pos_str = card['pos']
        x, y, z, r = self._parse_position_string(pos_str)
        
        self.get_logger().info(f'Sorting {color.upper()} card at ({x}, {y}, {z}, {r})')
        
        # Move to pick position
        if not self._move_to_position(x, y, z, r):
            self.get_logger().error(f'Failed to move to pick position for {color}')
            return
        
        # Pump on (pickup)
        self.get_logger().info('Pump ON')
        self._set_tool_do(3, 1)  # Pump pin
        time.sleep(0.5)
        
        # Get bin position
        bin_pos = self.bins.get(color)
        if not bin_pos:
            self.get_logger().error(f'No bin configured for color {color}')
            self._set_tool_do(3, 0)  # Turn off pump
            return
        
        x_bin, y_bin, z_bin, r_bin = self._parse_position_string(bin_pos)
        
        # Move to bin
        if not self._move_to_position(x_bin, y_bin, z_bin, r_bin):
            self.get_logger().error(f'Failed to move to bin for {color}')
            self._set_tool_do(3, 0)
            return
        
        # Release (pump off, solenoid pulse)
        self.get_logger().info('Pump OFF, Solenoid PULSE')
        self._set_tool_do(3, 0)  # Pump off
        time.sleep(0.2)
        self._set_tool_do(4, 1)  # Solenoid on
        time.sleep(0.5)
        self._set_tool_do(4, 0)  # Solenoid off
        
        self.get_logger().info(f'Card {color} sorted successfully.')

    def _process_queue(self) -> None:
        if self.state == SortingState.SCANNING:
            # At home, waiting for new picks
            pass

        elif self.state == SortingState.SORTING:
            # Process cards one at a time
            if self.busy:
                return
            
            if not self.pending_cards:
                # All cards processed - transition to returning home
                self.get_logger().info('All cards sorted - transitioning to RETURNING_HOME')
                self.state = SortingState.RETURNING_HOME
                return
            
            # Process next card
            self.busy = True
            card = self.pending_cards.pop(0)
            try:
                self._sort_card(card)
                remaining = len(self.pending_cards)
                if remaining > 0:
                    self.get_logger().info(f'Card sorted. {remaining} remaining.')
            finally:
                self.busy = False

        elif self.state == SortingState.RETURNING_HOME:
            # Move back to home position and transition to SCANNING
            if self.busy:
                return
            
            self.busy = True
            try:
                self.get_logger().info('Moving back to home position...')
                if self._move_to_position(self.home_pos['x'], self.home_pos['y'], 
                                         self.home_pos['z'], self.home_pos['r']):
                    self.get_logger().info('Reached home - transitioning to SCANNING')
                    self.state = SortingState.SCANNING
                else:
                    self.get_logger().error('Failed to return to home position')
            finally:
                self.busy = False

    def destroy_node(self):
        try:
            # Disable robot gracefully
            req = DisableRobot.Request()
            self.disable_robot_client.call_async(req)
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DobotColorSortingNode()
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
