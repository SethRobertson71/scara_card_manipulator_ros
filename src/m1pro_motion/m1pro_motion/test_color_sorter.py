#!/usr/bin/env python3
"""
Test Color Sorter with ROS2 Vision Integration
Adapted from Dobot_Color_SortingV2.py to work with ROS pick_targets and labels
"""

import json
import math
import re
import time
import threading
from typing import List, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String

from m1pro_bringup.srv import MovJ, ToolDO, Sync, ClearError, EnableRobot, GetErrorID, SpeedFactor, SetArmOrientation 
from tf_transformations import euler_from_quaternion


# Hardware Pin Mapping (DO17 = Pin 2, DO18 = Pin 3)
PUMP_PIN = 4
SOLENOID_PIN = 3

# Bin Coordinates
BINS = {
    "purple": {"x": -20.0, "y": -306.0, "z": 140.0, "r": 90.0},
    "red": {"x": 105.0, "y": -315.0, "z": 140.0, "r": 90.0},
    "green": {"x": 233.0, "y": -318.0, "z": 140.0, "r": 90.0},
}

COLOR_FACE_VALUES = {
    'green': [1, 2, 3, 4],
    'red': [5, 6, 7, 8],
    'purple': [9, 10, 11, 12],
}

class TestColorSorter(Node):
    def __init__(self):
        super().__init__('test_color_sorter')

        self.declare_parameter('pick_targets_topic', '/world/card_poses')
        self.declare_parameter('pick_labels_topic', '/skipbo/pick_target_labels')
        self.declare_parameter('speed_factor', 80)
        self.declare_parameter('lift_speed_factor', 30)
        self.declare_parameter('default_pick_z', 140.0)
        self.declare_parameter('default_pick_r', 0.0)
        self.declare_parameter('home_x', 400.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_z', 240.0)
        self.declare_parameter('home_r', 0.0)
        self.declare_parameter('home_settle_sec', 0.5)
        self.declare_parameter('vision_capture_sec', 1.0)
        self.declare_parameter('max_abs_workspace_x', 600.0)
        self.declare_parameter('max_abs_workspace_y', 600.0)
        self.declare_parameter('max_workspace_xy_radius_mm', 400.0)
        self.declare_parameter('pump_on_level', 1)
        self.declare_parameter('pump_off_level', 0)
        self.declare_parameter('solenoid_on_level', 1)
        self.declare_parameter('solenoid_off_level', 0)
        self.declare_parameter('release_pulse_sec', 0.1)

        pick_targets_topic = self.get_parameter('pick_targets_topic').value
        pick_labels_topic = self.get_parameter('pick_labels_topic').value
        self.speed_factor = int(self.get_parameter('speed_factor').value)
        self.lift_speed_factor = int(self.get_parameter('lift_speed_factor').value)
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
        self.max_workspace_xy_radius_mm = float(self.get_parameter('max_workspace_xy_radius_mm').value)
        self.command_timeout_sec = 2.0
        self.sync_timeout_sec = 30.0
        self.pump_on_level = int(self.get_parameter('pump_on_level').value)
        self.pump_off_level = int(self.get_parameter('pump_off_level').value)
        self.solenoid_on_level = int(self.get_parameter('solenoid_on_level').value)
        self.solenoid_off_level = int(self.get_parameter('solenoid_off_level').value)
        self.release_pulse_sec = float(self.get_parameter('release_pulse_sec').value)

        # ROS service clients
        self.movj_client = self.create_client(MovJ, '/bringup/srv/MovJ')
        self.sync_client = self.create_client(Sync, '/bringup/srv/Sync')
        self.tool_do_client = self.create_client(ToolDO, '/bringup/srv/ToolDO')
        self.clear_error_client = self.create_client(ClearError, '/bringup/srv/ClearError')
        self.enable_robot_client = self.create_client(EnableRobot, '/bringup/srv/EnableRobot')
        self.get_error_id_client = self.create_client(GetErrorID, '/bringup/srv/GetErrorID')
        self.speed_factor_client = self.create_client(SpeedFactor, '/bringup/srv/SpeedFactor')
        self.set_arm_orientation_client = self.create_client(SetArmOrientation, '/bringup/srv/SetArmOrientation')

        # State
        self.detected_cards: List[Dict] = []
        self.latest_targets: PoseArray = None
        self.latest_labels: List[str] = []
        self.busy = False
        self.initialized = False
        self.halted_on_error = False
        self.accept_vision_updates = False
        self.vision_lock = threading.Lock()
        self.max_empty_scan_cycles = 5
        self.empty_scan_cycles = 0

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
            face_value = self._extract_face_value(label)

            # Extract yaw from orientation
            q = pose.orientation
            roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            r_value = math.degrees(yaw)

            cards.append({
                'id': i,
                'label': label,
                'color': color,
                'face_value': face_value,
                'pos': {
                    'x': float(pose.position.x),
                    'y': float(pose.position.y),
                    'z': self.default_pick_z,
                    'r': r_value
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

    def _extract_face_value(self, label: str) -> Optional[int]:
        """Extract card face value from labels like 'green_7'; returns None when unknown."""
        match = re.search(r'(\d+)(?!.*\d)', str(label))
        if not match:
            return None
        try:
            return int(match.group(1))
        except ValueError:
            return None

    def _infer_missing_face_values(self, cards: List[Dict]) -> List[Dict]:
        """Fill one missing face value when 4 cards of one color leave exactly one unknown."""
        cards_by_color: Dict[str, List[Dict]] = {}
        for card in cards:
            cards_by_color.setdefault(card.get('color', ''), []).append(card)

        for color, color_cards in cards_by_color.items():
            expected_values = COLOR_FACE_VALUES.get(color)
            if not expected_values or len(color_cards) != 4:
                continue

            known_values = []
            unknown_cards = []
            for card in color_cards:
                face_value = card.get('face_value')
                if isinstance(face_value, int):
                    known_values.append(face_value)
                else:
                    unknown_cards.append(card)

            if len(unknown_cards) != 1 or len(known_values) != 3:
                continue

            missing_values = [value for value in expected_values if value not in known_values]
            if len(missing_values) != 1:
                continue

            missing_value = missing_values[0]
            unknown_card = unknown_cards[0]
            unknown_card['face_value'] = missing_value
            self.get_logger().info(
                f"Inferred missing {color} card face value {missing_value} from known values {sorted(known_values)}"
            )

        return cards

    def _is_reasonable_workspace_pose(self, pose_dict: Dict) -> bool:
        """Reject obvious pixel-space values before sending robot motion commands."""
        x = float(pose_dict['x'])
        y = float(pose_dict['y'])
        radial_xy = math.hypot(x, y)
        return (
            abs(x) <= self.max_abs_workspace_x
            and abs(y) <= self.max_abs_workspace_y
            and radial_xy <= self.max_workspace_xy_radius_mm
        )

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

    def _rearm_robot_for_recovery(self) -> bool:
        """Clear faults and re-enable robot before each recovery motion attempt."""
        req = ClearError.Request()
        if not self._call_service_sync(
            self.clear_error_client,
            req,
            command_name='ClearError(recovery)',
            check_error=False,
            wait_for_sync=False,
        ):
            self.get_logger().error('Recovery failed: ClearError command failed')
            return False

        req = EnableRobot.Request()
        if not self._call_service_sync(
            self.enable_robot_client,
            req,
            command_name='EnableRobot(recovery)',
            check_error=False,
            wait_for_sync=False,
        ):
            self.get_logger().error('Recovery failed: EnableRobot command failed')
            return False

        speed_req = SpeedFactor.Request()
        speed_req.ratio = max(1, min(100, int(self.speed_factor)))
        if not self._call_service_sync(
            self.speed_factor_client,
            speed_req,
            command_name=f'SpeedFactor(recovery={speed_req.ratio})',
            check_error=False,
            wait_for_sync=False,
        ):
            self.get_logger().error('Recovery failed: could not restore default speed factor')
            return False

        return True

    def _set_arm_orientation_l_or_r(self, l_or_r: int) -> bool:
        """Set left/right arm orientation while keeping other orientation fields fixed."""
        req = SetArmOrientation.Request()
        req.l_or_r = int(l_or_r)
        req.u_or_d = 0
        req.f_or_n = 0
        req.config6 = 0

        max_attempts = 2
        for attempt in range(1, max_attempts + 1):
            if not self.set_arm_orientation_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('Service /bringup/srv/SetArmOrientation not available')
                return False

            future = self.set_arm_orientation_client.call_async(req)
            ok, result = self._wait_for_future(future, timeout=self.command_timeout_sec)
            if not ok or result is None:
                self.get_logger().error('SetArmOrientation call failed or timed out')
                return False

            res_code = getattr(result, 'res', None)
            if res_code == -10000 and attempt < max_attempts:
                self.get_logger().warn(
                    'SetArmOrientation returned -10000, retrying once'
                )
                continue

            if res_code == -10000:
                self.get_logger().error('SetArmOrientation returned -10000 after retry')
                return False

            err = self._read_robot_error_id()
            if err is None:
                self.get_logger().error('Unable to read GetErrorID after SetArmOrientation')
                return False
            if err != 0:
                self.get_logger().warn(
                    f'GetErrorID after SetArmOrientation is {err}; continuing recovery flow'
                )

            return True

        return False

    def _movj_raw(self, pose_dict: Dict) -> bool:
        """Send one MovJ command and wait for completion without implicit error policy."""
        req = MovJ.Request()
        req.x = pose_dict['x']
        req.y = pose_dict['y']
        req.z = pose_dict['z']
        req.r = pose_dict['r']
        req.param_value = []
        return self._call_service_sync(
            self.movj_client,
            req,
            command_name='MovJ',
            check_error=False,
            wait_for_sync=False,
        )

    def _call_service_sync(
        self,
        client,
        request,
        *,
        command_name: str = 'command',
        check_error: bool = True,
        wait_for_sync: bool = True,
    ) -> bool:
        """Call ROS service synchronously, then wait for robot sync when requested."""
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Service {client.srv_name} not available')
            return False

        future = client.call_async(request)
        timeout_sec = self.sync_timeout_sec if client == self.sync_client else self.command_timeout_sec
        ok, result = self._wait_for_future(future, timeout=timeout_sec)
        if not ok:
            self.get_logger().error(f'Service call timeout/failure: {client.srv_name}')
            return False

        if result is None:
            self.get_logger().error(f'Service call returned no result: {client.srv_name}')
            return False

        if wait_for_sync and client != self.sync_client:
            sync_req = Sync.Request()
            if not self._call_service_sync(
                self.sync_client,
                sync_req,
                command_name='Sync',
                check_error=False,
                wait_for_sync=False,
            ):
                self.get_logger().error(f'Robot sync failed after {command_name}')
                return False

        if check_error:
            return self._post_command_error_check(command_name)

        return True

    def pickup_sequence(self):
        """Descend to pickup height, activate suction, then lift back up."""
        self.get_logger().info('>>> MOVE DOWN TO PICKUP HEIGHT')
        pick_pose = {
            'x': self.current_pick_pose['x'],
            'y': self.current_pick_pose['y'],
            'z': 111.0,
            'r': self.current_pick_pose['r'],
        }
        if not self.movj(pick_pose):
            self.get_logger().error('Failed to move down to pickup height')
            return False

        self.get_logger().info('>>> PUMP ON')
        req = ToolDO.Request()
        req.index = PUMP_PIN
        req.status = self.pump_on_level
        if not self._call_service_sync(
            self.tool_do_client,
            req,
            command_name='ToolDO pump on',
        ):
            return False

        self.get_logger().info(f'>>> SET LIFT SPEED FACTOR {self.lift_speed_factor}%')
        if not self.set_speed_factor(self.lift_speed_factor):
            self.get_logger().error('Failed to set lift speed factor')
            return False

        self.get_logger().info('>>> LIFT BACK TO PICK HEIGHT')
        if not self.movj(self.current_pick_pose):
            self.get_logger().error('Failed to lift back to pickup height')
            self.set_speed_factor(self.speed_factor)
            return False

        self.get_logger().info(f'>>> RESTORE SPEED FACTOR {self.speed_factor}%')
        if not self.set_speed_factor(self.speed_factor):
            self.get_logger().error('Failed to restore default speed factor')
            return False

        return True

    def release_sequence(self):
        """Pump off and pulse solenoid - waits for completion"""
        self.get_logger().info('>>> PUMP OFF | RELEASING PRESSURE')

        # Pump off
        req = ToolDO.Request()
        req.index = PUMP_PIN
        req.status = self.pump_off_level
        if not self._call_service_sync(
            self.tool_do_client,
            req,
            command_name='ToolDO pump off',
        ):
            return False

        # Solenoid on
        req.index = SOLENOID_PIN
        req.status = self.solenoid_on_level
        if not self._call_service_sync(
            self.tool_do_client,
            req,
            command_name='ToolDO solenoid on',
        ):
            return False
        time.sleep(self.release_pulse_sec)

        # Solenoid off
        req.status = self.solenoid_off_level
        if not self._call_service_sync(
            self.tool_do_client,
            req,
            command_name='ToolDO solenoid off',
        ):
            return False

        return True

    def _ensure_correct_orientation(self, target_y: float) -> bool:
        """Check target y and set arm orientation: y > 0 → l_or_r=1, y < 0 → l_or_r=0"""
        if target_y == 0:
            # Skip check for y=0
            return True
        
        desired_orientation = 1 if target_y > 0 else 0
        self.get_logger().info(
            f'Pre-move orientation check: target_y={target_y:.1f}, setting l_or_r={desired_orientation}'
        )
        return self._set_arm_orientation_l_or_r(desired_orientation)

    def movj(self, pose_dict: Dict) -> bool:
        """Execute MovJ with orientation check based on target y, then error-18 recovery via home return and orientation toggling."""
        # Check and set arm orientation based on target y coordinate
        if not self._ensure_correct_orientation(pose_dict['y']):
            self.get_logger().error('Failed to set correct arm orientation before move')
            self.halted_on_error = True
            return False
        
        if not self._movj_raw(pose_dict):
            self.get_logger().error('MovJ command failed before error-state check')
            self.halted_on_error = True
            return False

        err = self._read_robot_error_id()
        if err is None:
            self.get_logger().error('Unable to verify error state after MovJ; halting for safety.')
            self.halted_on_error = True
            return False

        if err == 0:
            return True

        if err != 18:
            self.get_logger().error(f'Robot error active after MovJ: error_id={err}. Halting sorter.')
            self.halted_on_error = True
            return False

        # Error 18 recovery: try toggling arm orientation back and forth
        self.get_logger().warn('Error-18 detected. Attempting recovery via orientation toggle.')
        
        retry_orientations = [0, 1]
        for idx, orientation in enumerate(retry_orientations, start=1):
            self.get_logger().warn(
                f'Error-18 retry attempt {idx}/{len(retry_orientations)}: '
                f'set l_or_r={orientation} and retry target move'
            )
            
            if not self._set_arm_orientation_l_or_r(orientation):
                self.get_logger().error(
                    f'Failed to set arm orientation l_or_r={orientation} during error-18 retry'
                )
                continue
            
            if self._movj_raw(pose_dict):
                retry_err = self._read_robot_error_id()
                if retry_err is None:
                    self.get_logger().error(
                        'Unable to verify error state after MovJ retry; halting for safety.'
                    )
                    self.halted_on_error = True
                    return False
                if retry_err == 0:
                    self.get_logger().info('Error-18 recovery successful with orientation toggle')
                    return True
                if retry_err != 18:
                    self.get_logger().error(
                        f'Robot error active after MovJ retry: error_id={retry_err}. Halting sorter.'
                    )
                    self.halted_on_error = True
                    return False
            else:
                self.get_logger().error('MovJ retry command failed before error-state check')

            if idx < len(retry_orientations):
                self.get_logger().warn(
                    'MovJ retry still hit error_id=18. Trying next orientation.'
                )

        self.get_logger().error(
            'MovJ retry failed after orientation toggles. Halting sorter.'
        )
        self.halted_on_error = True
        return False

    def set_speed_factor(self, ratio: int) -> bool:
        """Set robot global speed factor (1-100)."""
        ratio_clamped = max(1, min(100, int(ratio)))
        if ratio_clamped != ratio:
            self.get_logger().warn(f'Clamped speed factor from {ratio} to {ratio_clamped}')

        req = SpeedFactor.Request()
        req.ratio = ratio_clamped
        return self._call_service_sync(
            self.speed_factor_client,
            req,
            command_name=f'SpeedFactor({ratio_clamped})',
            wait_for_sync=False,
        )

    def get_priority_list(self, cards: List[Dict]) -> List[Dict]:
        """Sort cards by color group, then by ascending card face value."""
        color_order = {"green": 1, "red": 2, "purple": 3}
        return sorted(
            cards,
            key=lambda x: (
                color_order.get(x['color'], 99),
                x['face_value'] if x['face_value'] is not None else 999,
                x['id'],
            ),
        )

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
        cards = self._infer_missing_face_values(cards)
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
        
        # Enable robot
        self.get_logger().info('Enabling robot')
        req = EnableRobot.Request()
        if not self._call_service_sync(self.enable_robot_client, req, command_name='EnableRobot'):
            self.get_logger().error('EnableRobot failed or robot still in error state')
            self.halted_on_error = True
            return

        # Set default runtime speed factor
        self.get_logger().info(f'Setting default speed factor to {self.speed_factor}%')
        if not self.set_speed_factor(self.speed_factor):
            self.get_logger().error('Failed to set default speed factor')
            self.halted_on_error = True
            return

        if not self.movj(self._home_pose()):
            self.get_logger().error('Failed initial move to home position')
            self.halted_on_error = True
            return
        
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
                    self.empty_scan_cycles += 1
                    self.get_logger().warn(
                        f'No cards detected this cycle ({self.empty_scan_cycles}/{self.max_empty_scan_cycles})'
                    )
                    if self.empty_scan_cycles >= self.max_empty_scan_cycles:
                        self.halted_on_error = True
                        self.get_logger().error(
                            'No cards detected for 5 consecutive cycles. Halting sorter. '
                            'Confirm vision stream/poses and restart node.'
                        )
                    time.sleep(1.0)
                    continue

                self.empty_scan_cycles = 0

                self.busy = True

                # Get priority list
                queue = self.get_priority_list(self.detected_cards)
                cycle_failed = False

                # Process each card - synchronously waits for robot to finish each step
                while queue:
                    target = queue.pop(0)
                    face_value = target.get('face_value')
                    value_text = str(face_value) if face_value is not None else 'unknown'
                    self.get_logger().info(
                        f"Processing Card {target['id']} ({target['color']} {value_text})"
                    )

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

                    # Pickup
                    self.current_pick_pose = dict(target['pos'])
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

                    # Release
                    if not self.release_sequence():
                        self.get_logger().error('Release sequence failed')
                        cycle_failed = True
                        break

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
