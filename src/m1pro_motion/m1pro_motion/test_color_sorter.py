#!/usr/bin/env python3
"""
Test Color Sorter with ROS2 Vision Integration
Adapted from Dobot_Color_SortingV2.py to work with ROS pick_targets and labels
"""

import json
import time
import threading
from typing import List, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String

from m1pro_bringup.srv import MovJ, ToolDO, ClearError, EnableRobot, GetErrorID


# Hardware Pin Mapping (DO17 = Pin 2, DO18 = Pin 3)
PUMP_PIN = 3
SOLENOID_PIN = 4

# Bin Coordinates
BINS = {
    "red": {"x": -18.0, "y": -303.0, "z": 130.0, "r": 90.0},
    "purple": {"x": 108.0, "y": -311.0, "z": 130.0, "r": 90.0},
    "green": {"x": 234.0, "y": -319.0, "z": 130.0, "r": 90.0},
}




class TestColorSorter(Node):
    def __init__(self):
        super().__init__('test_color_sorter')

        self.declare_parameter('pick_targets_topic', '/world/card_poses')
        self.declare_parameter('pick_labels_topic', '/skipbo/pick_target_labels')
        self.declare_parameter('speed_factor', 40)
        self.declare_parameter('default_pick_z', 140.0)
        self.declare_parameter('default_pick_r', 0.0)
        self.declare_parameter('home_x', 400.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_z', 240.0)
        self.declare_parameter('home_r', 0.0)
        self.declare_parameter('home_settle_sec', 1.0)
        self.declare_parameter('vision_capture_sec', 1.0)
        self.declare_parameter('max_abs_workspace_x', 600.0)
        self.declare_parameter('max_abs_workspace_y', 600.0)
        self.declare_parameter('pump_on_level', 1)
        self.declare_parameter('pump_off_level', 0)
        self.declare_parameter('solenoid_on_level', 1)
        self.declare_parameter('solenoid_off_level', 0)
        self.declare_parameter('release_pulse_sec', 1.0)

        pick_targets_topic = self.get_parameter('pick_targets_topic').value
        pick_labels_topic = self.get_parameter('pick_labels_topic').value
        self.speed_factor = int(self.get_parameter('speed_factor').value)
        self.default_pick_z = float(self.get_parameter('default_pick_z').value)
        self.default_pick_r = float(self.get_parameter('default_pick_r').value)
        self.home_x = float(self.get_parameter('home_x').value)
        self.home_y = float(self.get_parameter('home_y').value)
        self.home_z = float(self.get_parameter('home_z').value)
        self.home_r = float(self.get_parameter('home_r').value)
        self.home_settle_sec = float(self.get_parameter('home_settle_sec').value)
        self.vision_capture_sec = float(self.get_parameter('vision_capture_sec').value)
        self.max_abs_workspace_x = float(self.get_parameter('max_abs_workspace_x').value)
        self.max_abs_workspace_y = float(self.get_parameter('max_abs_workspace_y').value)
        self.pump_on_level = int(self.get_parameter('pump_on_level').value)
        self.pump_off_level = int(self.get_parameter('pump_off_level').value)
        self.solenoid_on_level = int(self.get_parameter('solenoid_on_level').value)
        self.solenoid_off_level = int(self.get_parameter('solenoid_off_level').value)
        self.release_pulse_sec = float(self.get_parameter('release_pulse_sec').value)

        # ROS service clients
        self.movj_client = self.create_client(MovJ, '/bringup/srv/MovJ')
        self.tool_do_client = self.create_client(ToolDO, '/bringup/srv/ToolDO')
        self.clear_error_client = self.create_client(ClearError, '/bringup/srv/ClearError')
        self.enable_robot_client = self.create_client(EnableRobot, '/bringup/srv/EnableRobot')
        self.get_error_id_client = self.create_client(GetErrorID, '/bringup/srv/GetErrorID')

        # State
        self.detected_cards: List[Dict] = []
        self.latest_targets: PoseArray = None
        self.latest_labels: List[str] = []
        self.busy = False
        self.initialized = False
        self.halted_on_error = False
        self.accept_vision_updates = False
        self.vision_lock = threading.Lock()

        # Subscriptions
        self.create_subscription(PoseArray, pick_targets_topic, self._targets_cb, 10)
        self.create_subscription(String, pick_labels_topic, self._labels_cb, 10)

        # Start background processing thread
        self.process_thread = threading.Thread(target=self._process_loop, daemon=True)
        self.process_thread.start()

        self.get_logger().info('Test color sorter initialized')

    def _targets_cb(self, msg: PoseArray):
        """Receive pick targets from vision"""
        if not self.accept_vision_updates:
            return
        with self.vision_lock:
            self.latest_targets = msg
        self.get_logger().info(
            f"Received {len(msg.poses)} pick targets (frame='{msg.header.frame_id}')"
        )

    def _labels_cb(self, msg: String):
        """Receive labels from vision"""
        if not self.accept_vision_updates:
            return
        try:
            labels = json.loads(msg.data)
            with self.vision_lock:
                self.latest_labels = labels
            self.get_logger().info(f'Received labels: {self.latest_labels}')
        except json.JSONDecodeError:
            # If not JSON, treat as single string
            with self.vision_lock:
                self.latest_labels = [msg.data]

    def _convert_to_detected_cards(self, targets_msg: Optional[PoseArray], labels: List[str]) -> List[Dict]:
        """Convert ROS messages to detected_cards format"""
        if targets_msg is None:
            return []

        cards = []
        poses = targets_msg.poses
        labels = labels if isinstance(labels, list) else []

        for i, pose in enumerate(poses):
            label = labels[i] if i < len(labels) else f"card_{i}"

            # Extract color from label
            color = self._extract_color(label)

            cards.append({
                'id': i,
                'color': color,
                'pos': {
                    'x': float(pose.position.x),
                    'y': float(pose.position.y),
                    'z': self.default_pick_z,
                    'r': self.default_pick_r
                }
            })

        return cards

    def _extract_color(self, label: str) -> str:
        """Extract color name from label"""
        label_lower = label.lower()
        for color in ['green', 'red', 'purple', 'blue']:
            if color in label_lower:
                return color
        return 'red'  # default color

    def _is_reasonable_workspace_pose(self, pose_dict: Dict) -> bool:
        """Reject obvious pixel-space values before sending robot motion commands."""
        x = abs(float(pose_dict['x']))
        y = abs(float(pose_dict['y']))
        return x <= self.max_abs_workspace_x and y <= self.max_abs_workspace_y

    def _wait_for_future(self, future, timeout: float = 2.0) -> Tuple[bool, Optional[object]]:
        """Wait for a ROS service future and return (ok, result)."""
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.01)

        if not future.done():
            return False, None

        try:
            return True, future.result()
        except Exception:
            return False, None

    def _read_robot_error_id(self) -> Optional[int]:
        """Return current robot error id (0 means no error), or None if unavailable."""
        if not self.get_error_id_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service /bringup/srv/GetErrorID not available')
            return None

        req = GetErrorID.Request()
        future = self.get_error_id_client.call_async(req)
        ok, result = self._wait_for_future(future, timeout=2.0)
        if not ok or result is None:
            self.get_logger().error('GetErrorID call failed or timed out')
            return None

        raw_res = getattr(result, 'res', None)

        def extract_first_int(value) -> Optional[int]:
            if isinstance(value, bool):
                return int(value)
            if isinstance(value, int):
                return value
            if isinstance(value, float):
                return int(value)
            if isinstance(value, str):
                try:
                    return int(value)
                except ValueError:
                    return None
            if isinstance(value, (list, tuple)):
                for item in value:
                    extracted = extract_first_int(item)
                    if extracted is not None:
                        return extracted
                return None
            return None

        err = extract_first_int(raw_res)
        self.get_logger().info(f'GetErrorID raw={raw_res!r} parsed={err}')
        return err

    def _post_command_error_check(self, command_name: str) -> bool:
        """Check robot error state after each command and halt processing if error is active."""
        err = self._read_robot_error_id()
        if err is None:
            self.get_logger().error(
                f'Unable to verify error state after {command_name}; halting for safety.'
            )
            self.halted_on_error = True
            return False

        if err != 0:
            self.get_logger().error(
                f'Robot error active after {command_name}: error_id={err}. Halting sorter.'
            )
            self.halted_on_error = True
            return False

        return True

    def _call_service_sync(self, client, request, *, command_name: str = 'command', check_error: bool = True) -> bool:
        """Call ROS service synchronously with timeout and optional post-command error check."""
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Service {client.srv_name} not available')
            return False

        future = client.call_async(request)
        ok, result = self._wait_for_future(future, timeout=2.0)
        if not ok:
            self.get_logger().error(f'Service call timeout/failure: {client.srv_name}')
            return False

        if result is None:
            self.get_logger().error(f'Service call returned no result: {client.srv_name}')
            return False

        if check_error:
            return self._post_command_error_check(command_name)

        return True

    def pickup_sequence(self):
        """Pump on and hold - waits for completion"""
        self.get_logger().info('>>> PUMP ON')
        req = ToolDO.Request()
        req.index = PUMP_PIN
        req.status = self.pump_on_level
        if not self._call_service_sync(self.tool_do_client, req, command_name='ToolDO pump on'):
            return False
        time.sleep(1.0)
        return True

    def release_sequence(self):
        """Pump off and pulse solenoid - waits for completion"""
        self.get_logger().info('>>> PUMP OFF | RELEASING PRESSURE')

        # Pump off
        req = ToolDO.Request()
        req.index = PUMP_PIN
        req.status = self.pump_off_level
        if not self._call_service_sync(self.tool_do_client, req, command_name='ToolDO pump off'):
            return False

        # Solenoid on
        req.index = SOLENOID_PIN
        req.status = self.solenoid_on_level
        if not self._call_service_sync(self.tool_do_client, req, command_name='ToolDO solenoid on'):
            return False
        time.sleep(self.release_pulse_sec)

        # Solenoid off
        req.status = self.solenoid_off_level
        if not self._call_service_sync(self.tool_do_client, req, command_name='ToolDO solenoid off'):
            return False

        return True

    def movj(self, pose_dict: Dict) -> bool:
        """Execute MovJ command - waits for robot to complete before returning"""
        req = MovJ.Request()
        req.x = pose_dict['x']
        req.y = pose_dict['y']
        req.z = pose_dict['z']
        req.r = pose_dict['r']
        req.param_value = []
        
        return self._call_service_sync(self.movj_client, req, command_name='MovJ')

    def get_priority_list(self, cards: List[Dict]) -> List[Dict]:
        """Sort cards by Green(1-4), Red(5-8), Purple(9-12)"""
        color_order = {"green": 1, "red": 2, "purple": 3}
        return sorted(cards, key=lambda x: (color_order.get(x['color'], 99), x['id']))

    def _home_pose(self) -> Dict:
        return {
            'x': self.home_x,
            'y': self.home_y,
            'z': self.home_z,
            'r': self.home_r,
        }

    def _capture_targets_at_home(self) -> List[Dict]:
        """Move to home, briefly accept vision updates, and freeze a snapshot for sorting."""
        self.accept_vision_updates = False
        self.get_logger().info(
            f"Moving home for scan: x={self.home_x:.1f}, y={self.home_y:.1f}, z={self.home_z:.1f}, r={self.home_r:.1f}"
        )
        if not self.movj(self._home_pose()):
            self.get_logger().error('Failed to move to home position before scan')
            return []

        time.sleep(self.home_settle_sec)
        with self.vision_lock:
            self.latest_targets = None
            self.latest_labels = []

        self.accept_vision_updates = True
        time.sleep(self.vision_capture_sec)
        self.accept_vision_updates = False

        with self.vision_lock:
            snapshot_targets = self.latest_targets
            snapshot_labels = list(self.latest_labels)

        cards = self._convert_to_detected_cards(snapshot_targets, snapshot_labels)
        self.get_logger().info(f'Captured {len(cards)} target(s) at home scan window')
        return cards

    def _initialize_robot(self):
        """Initialize robot - clear errors and enable"""
        if self.initialized:
            return
        
        self.get_logger().info('Initializing robot...')
        
        # Clear errors
        self.get_logger().info('Clearing robot errors')
        req = ClearError.Request()
        self._call_service_sync(self.clear_error_client, req, command_name='ClearError', check_error=False)
        time.sleep(1.0)
        
        # Enable robot
        self.get_logger().info('Enabling robot')
        req = EnableRobot.Request()
        if not self._call_service_sync(self.enable_robot_client, req, command_name='EnableRobot'):
            self.get_logger().error('EnableRobot failed or robot still in error state')
            self.halted_on_error = True
            return
        time.sleep(1.0)

        if not self.movj(self._home_pose()):
            self.get_logger().error('Failed initial move to home position')
            self.halted_on_error = True
            return
        time.sleep(self.home_settle_sec)
        
        self.get_logger().info('Robot initialization complete')
        self.initialized = True

    def _process_loop(self):
        """Background thread processing loop - synchronous, waits for robot"""
        # Initialize robot first
        self._initialize_robot()
        
        while True:
            try:
                if self.halted_on_error:
                    if self.busy:
                        self.busy = False
                    self.get_logger().error(
                        'Sorter halted due to robot error state. Clear fault and restart node.'
                    )
                    time.sleep(1.0)
                    continue

                # Wait for current cycle completion
                if self.busy:
                    time.sleep(1.0)
                    continue

                # Capture a valid vision snapshot only while at home.
                self.detected_cards = self._capture_targets_at_home()

                if not self.detected_cards:
                    time.sleep(1.0)
                    continue

                self.busy = True

                # Get priority list
                queue = self.get_priority_list(self.detected_cards)
                cycle_failed = False

                # Process each card - synchronously waits for robot to finish each step
                while queue:
                    target = queue.pop(0)
                    self.get_logger().info(f"Processing Card {target['id']} ({target['color']})")

                    if not self._is_reasonable_workspace_pose(target['pos']):
                        self.get_logger().error(
                            f"Skipping target with out-of-range workspace pose: {target['pos']} "
                            f"(check pick_targets_topic and TF)"
                        )
                        continue

                    # Move to pick location
                    self.get_logger().info(f"Moving to pick: {target['pos']}")
                    if not self.movj(target['pos']):
                        self.get_logger().error(f"Failed to move to pick location")
                        cycle_failed = True
                        break
                    time.sleep(1.0)

                    # Pickup
                    if not self.pickup_sequence():
                        self.get_logger().error('Pickup sequence failed')
                        cycle_failed = True
                        break

                    # Move to bin
                    bin_pos = BINS.get(target['color'], BINS['red'])
                    self.get_logger().info(f"Moving to bin ({target['color']}): {bin_pos}")
                    if not self.movj(bin_pos):
                        self.get_logger().error(f"Failed to move to bin")
                        cycle_failed = True
                        break
                    time.sleep(1.0)

                    # Release
                    if not self.release_sequence():
                        self.get_logger().error('Release sequence failed')
                        cycle_failed = True
                        break
                    time.sleep(1.0)

                if cycle_failed:
                    self.get_logger().error('Sort cycle aborted due to command failure or robot error.')
                else:
                    self.get_logger().info("✅ All cards sorted!")
                self.busy = False

            except Exception as e:
                self.get_logger().error(f'Error in processing loop: {e}', exc_info=True)
                self.busy = False
                time.sleep(1.0)


def main():
    rclpy.init()
    node = TestColorSorter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
