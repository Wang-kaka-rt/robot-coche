"""
Ball Follower Node for ROS2.

Autonomous red card following with obstacle avoidance and multimodal feedback.

Implements a state machine with four states:
- IDLE: No card detected, robot stopped
- SEARCHING: Card recently lost, rotating to re-acquire
- TRACKING: Card visible, following with proportional control
- OBSTACLE: Obstacle too close, backing up

Sensor integration:
- Ultrasonic: obstacle avoidance (requirement 4.3)
- Infrared: ground/edge detection (requirement 4.3)
- Battery: low-voltage LED warning (requirement 4.4)

Multimodal communication (requirement 4.4):
- LEDs: mode 2 (chase) = tracking, mode 3 (blink) = danger/obstacle,
        mode 4 (breathing) = searching, mode 5 (rainbow) = low battery
- Buzzer: double beep = card found, triple beep = obstacle, long beep = card lost
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import Range, BatteryState
from std_msgs.msg import Bool, Int32, Float32MultiArray, UInt8
import threading
import time
from enum import IntEnum


class RobotState(IntEnum):
    IDLE = 0
    SEARCHING = 1
    TRACKING = 2
    OBSTACLE = 3


# LED mode mapping (matches car_base_node _tick_led_effect modes)
class LedMode(IntEnum):
    OFF = 0            # colorBlink(0) -> All LEDs off (idle)
    CHASE = 2          # following() -> Rainbow chase (tracking)
    COLOR_BLINK = 3    # colorBlink(1) -> R/G/B blink (warning/obstacle)
    BREATHING = 4      # rainbowbreathing() -> Breathing (searching)
    RAINBOW = 5        # rainbowCycle() -> Rainbow cycle (low battery)


class BallFollowerNode(Node):
    """ROS2 node for autonomous red card following with obstacle avoidance."""

    def __init__(self):
        super().__init__('ball_follower_node')

        # --- Parameters ---
        self.declare_parameter('frame_width', 400)
        self.declare_parameter('frame_height', 300)
        self.declare_parameter('linear_speed', 0.35)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('min_card_size', 10)
        self.declare_parameter('max_card_size', 80)
        self.declare_parameter('target_card_size', 35)
        self.declare_parameter('center_tolerance', 40)
        self.declare_parameter('lost_timeout_s', 1.5)
        self.declare_parameter('search_timeout_s', 8.0)
        self.declare_parameter('obstacle_distance_m', 0.25)
        self.declare_parameter('obstacle_critical_m', 0.12)
        self.declare_parameter('low_battery_v', 6.5)
        self.declare_parameter('search_angular_speed', 0.30)

        self.frame_w = self.get_parameter('frame_width').value
        self.frame_h = self.get_parameter('frame_height').value
        self.linear_spd = self.get_parameter('linear_speed').value
        self.angular_spd = self.get_parameter('angular_speed').value
        self.min_card_size = self.get_parameter('min_card_size').value
        self.max_card_size = self.get_parameter('max_card_size').value
        self.target_card_size = self.get_parameter('target_card_size').value
        self.center_tol = self.get_parameter('center_tolerance').value
        self.lost_timeout = self.get_parameter('lost_timeout_s').value
        self.search_timeout = self.get_parameter('search_timeout_s').value
        self.obstacle_dist = self.get_parameter('obstacle_distance_m').value
        self.obstacle_critical = self.get_parameter('obstacle_critical_m').value
        self.low_battery_v = self.get_parameter('low_battery_v').value
        self.search_angular = self.get_parameter('search_angular_speed').value

        # --- State ---
        self.state = RobotState.IDLE
        self.ball_detected = False
        self.last_detection_time = 0.0
        self.ball_position = Point()
        self.card_size = 0.0
        self.last_error_x = 0.0  # Remember which side card was last seen

        self.obstacle_range = float('inf')
        self.battery_voltage = 0.0
        self.battery_low = False
        self.line_sensor_value = 0

        # Buzzer pattern state
        self.buzzer_on = False
        self.buzzer_pattern = []  # List of (on_seconds, off_seconds)
        self.buzzer_pattern_idx = 0
        self.buzzer_step_time = 0.0

        # Lock for shared state
        self.state_lock = threading.Lock()

        # --- Publishers ---
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_buzzer = self.create_publisher(Bool, '/buzzer', 10)
        self.pub_led_mode = self.create_publisher(Int32, '/led_mode', 10)

        # --- Subscribers ---
        # Vision detection topics
        self.create_subscription(
            Bool, '/ball_detection/status', self._on_detection_status, 10)
        self.create_subscription(
            Point, '/ball_detection/position', self._on_card_position, 10)
        self.create_subscription(
            Float32MultiArray, '/ball_detection/radius', self._on_card_size, 10)
        # Sensor topics from car_base_node
        self.create_subscription(
            Range, '/ultrasonic/range', self._on_ultrasonic, 10)
        self.create_subscription(
            BatteryState, '/battery', self._on_battery, 10)
        self.create_subscription(
            UInt8, '/line_sensor', self._on_line_sensor, 10)

        # --- Control loop at 20 Hz ---
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info('Ball Follower Node initialized')
        self.get_logger().info(
            f'  frame={self.frame_w}x{self.frame_h}, '
            f'speeds: lin={self.linear_spd} ang={self.angular_spd}')
        self.get_logger().info(
            f'  obstacle: warn={self.obstacle_dist}m crit={self.obstacle_critical}m, '
            f'battery_low={self.low_battery_v}V')

    # ------------------------------------------------------------------ #
    #  Callbacks                                                          #
    # ------------------------------------------------------------------ #

    def _on_detection_status(self, msg: Bool) -> None:
        with self.state_lock:
            self.ball_detected = msg.data
            if msg.data:
                self.last_detection_time = time.time()

    def _on_card_position(self, msg: Point) -> None:
        with self.state_lock:
            self.ball_position = msg
            # Track direction for search behavior
            self.last_error_x = (self.frame_w / 2.0) - msg.x

    def _on_card_size(self, msg: Float32MultiArray) -> None:
        with self.state_lock:
            if len(msg.data) > 0:
                self.card_size = msg.data[0]

    def _on_ultrasonic(self, msg: Range) -> None:
        with self.state_lock:
            self.obstacle_range = msg.range

    def _on_battery(self, msg: BatteryState) -> None:
        with self.state_lock:
            self.battery_voltage = msg.voltage
            was_low = self.battery_low
            # Only flag low if we actually get a reading (>0.5V means sensor connected)
            self.battery_low = 0.5 < msg.voltage < self.low_battery_v
            if self.battery_low and not was_low:
                self.get_logger().warn(
                    f'LOW BATTERY: {msg.voltage:.1f}V < {self.low_battery_v}V')

    def _on_line_sensor(self, msg: UInt8) -> None:
        with self.state_lock:
            self.line_sensor_value = msg.data

    # ------------------------------------------------------------------ #
    #  Main control loop                                                  #
    # ------------------------------------------------------------------ #

    def _control_loop(self) -> None:
        now = time.time()

        # Snapshot shared state
        with self.state_lock:
            detected = self.ball_detected
            position = Point(x=self.ball_position.x, y=self.ball_position.y)
            card_size = self.card_size
            last_det_time = self.last_detection_time
            obstacle = self.obstacle_range
            battery_low = self.battery_low
            last_err_x = self.last_error_x
            line_val = self.line_sensor_value

        time_since_det = (now - last_det_time) if last_det_time > 0 else float('inf')

        # Process buzzer timing
        self._update_buzzer(now)

        # Determine new state
        new_state = self._determine_state(
            detected, card_size, time_since_det, obstacle, now)

        # Handle state transitions
        if new_state != self.state:
            self._on_state_change(self.state, new_state, now, battery_low)
            self.state = new_state

        # Execute behavior
        if self.state == RobotState.IDLE:
            self._behavior_idle(battery_low)
        elif self.state == RobotState.SEARCHING:
            self._behavior_searching(last_err_x)
        elif self.state == RobotState.TRACKING:
            self._behavior_tracking(position, card_size, obstacle, line_val)
        elif self.state == RobotState.OBSTACLE:
            self._behavior_obstacle(obstacle)

    # ------------------------------------------------------------------ #
    #  State machine                                                      #
    # ------------------------------------------------------------------ #

    def _determine_state(self, detected, card_size, time_since_det, obstacle, now):
        # Critical obstacle override (except when idle)
        if self.state != RobotState.IDLE and obstacle < self.obstacle_critical:
            return RobotState.OBSTACLE

        # Card visible and large enough to track
        # (obstacle warning zone is handled by proportional slow-down in tracking behavior;
        #  only critical distance triggers OBSTACLE state above)
        if detected and card_size >= self.min_card_size:
            return RobotState.TRACKING

        # Grace period: maintain tracking briefly after losing card
        if time_since_det < self.lost_timeout:
            if self.state == RobotState.TRACKING:
                return RobotState.TRACKING
            return RobotState.SEARCHING

        # Search phase: rotate to re-acquire
        if time_since_det < self.lost_timeout + self.search_timeout:
            return RobotState.SEARCHING

        return RobotState.IDLE

    def _on_state_change(self, old_state, new_state, now, battery_low):
        self.get_logger().info(f'State: {old_state.name} -> {new_state.name}')

        if new_state == RobotState.TRACKING and old_state != RobotState.TRACKING:
            self._play_pattern_success()
            self._set_led_mode(LedMode.CHASE)

        elif new_state == RobotState.SEARCHING:
            self._set_led_mode(LedMode.BREATHING)
            if old_state == RobotState.TRACKING:
                self._play_pattern_lost()

        elif new_state == RobotState.OBSTACLE:
            self._play_pattern_danger()
            self._set_led_mode(LedMode.COLOR_BLINK)

        elif new_state == RobotState.IDLE:
            self._send_command(0.0, 0.0, 0.0)
            if battery_low:
                self._set_led_mode(LedMode.RAINBOW)
            else:
                self._set_led_mode(LedMode.OFF)

    # ------------------------------------------------------------------ #
    #  Behaviors                                                          #
    # ------------------------------------------------------------------ #

    def _behavior_idle(self, battery_low):
        self._send_command(0.0, 0.0, 0.0)
        # Keep battery LED updated
        if battery_low:
            self._set_led_mode(LedMode.RAINBOW)

    def _behavior_searching(self, last_error_x):
        # Rotate toward last known direction of card
        direction = 1.0 if last_error_x >= 0 else -1.0
        self._send_command(0.0, 0.0, direction * self.search_angular)

    def _behavior_tracking(self, position, card_size, obstacle, line_val):
        frame_center_x = self.frame_w / 2.0

        # Card too large -> too close, back up slowly
        if card_size > self.max_card_size:
            self._send_command(-self.linear_spd * 0.4, 0.0, 0.0)
            return

        # --- Angular: proportional control to center card in frame ---
        # error_x is negative when ball is right of center → negative angular.z → turn right.
        error_x = frame_center_x - position.x
        angular_vel = (error_x / frame_center_x) * self.angular_spd

        # --- Linear: distance-based speed control ---
        size_ratio = card_size / self.target_card_size

        # turn_factor reduces speed only at full cruise speed; do NOT stack it on top of
        # the size-ratio slow-down (0.5× or 0.2×), which would compound to nearly zero.
        turn_factor = 1.0 - min(abs(error_x) / frame_center_x, 0.7)

        if size_ratio > 1.3:
            linear_vel = self.linear_spd * 0.2   # Too close: back-off speed, no extra damping
        elif size_ratio > 1.0:
            linear_vel = self.linear_spd * 0.5   # Slightly close: moderate speed, no extra damping
        else:
            linear_vel = self.linear_spd * turn_factor  # Normal cruise: dampen when turning hard

        # Slow down near obstacles
        if obstacle < self.obstacle_dist:
            safe_range = self.obstacle_dist - self.obstacle_critical
            if safe_range > 0:
                obstacle_factor = max(0.2,
                    (obstacle - self.obstacle_critical) / safe_range)
                linear_vel *= obstacle_factor

        # Line sensor edge detection: if all 3 sensors triggered, stop
        # (bits: IR1=4, IR2=2, IR3=1; value 7 = all triggered)
        if line_val == 7:
            linear_vel = 0.0
            self.get_logger().debug('All infrared sensors triggered - edge detected')

        self._send_command(linear_vel, 0.0, angular_vel)

    def _behavior_obstacle(self, obstacle):
        if obstacle < self.obstacle_critical:
            self._send_command(-self.linear_spd * 0.5, 0.0, 0.0)
        else:
            self._send_command(0.0, 0.0, 0.0)

    # ------------------------------------------------------------------ #
    #  Actuator helpers                                                   #
    # ------------------------------------------------------------------ #

    def _send_command(self, linear_x, linear_y, angular_z):
        twist = Twist()
        # Use standard ROS convention: positive linear.x = forward.
        # car_base_node applies its own hardware-level negation internally.
        twist.linear.x = float(linear_x)
        twist.linear.y = float(linear_y)
        # positive angular_z = turn left (standard ROS convention)
        twist.angular.z = float(angular_z)
        self.pub_cmd_vel.publish(twist)

    def _set_led_mode(self, mode):
        msg = Int32()
        msg.data = int(mode)
        self.pub_led_mode.publish(msg)

    # ------------------------------------------------------------------ #
    #  Buzzer pattern system                                              #
    #  Patterns are lists of (on_seconds, off_seconds) tuples.            #
    #  The control loop advances through the pattern over time.           #
    # ------------------------------------------------------------------ #

    def _set_buzzer(self, on):
        msg = Bool()
        msg.data = bool(on)
        self.pub_buzzer.publish(msg)
        self.buzzer_on = on

    def _play_pattern_success(self):
        """Double short beep: card detected (success)."""
        self.buzzer_pattern = [(0.08, 0.06), (0.08, 0.0)]
        self.buzzer_pattern_idx = 0
        self.buzzer_step_time = time.time()
        self._set_buzzer(True)

    def _play_pattern_danger(self):
        """Triple quick beep: obstacle detected (danger)."""
        self.buzzer_pattern = [(0.06, 0.06), (0.06, 0.06), (0.06, 0.0)]
        self.buzzer_pattern_idx = 0
        self.buzzer_step_time = time.time()
        self._set_buzzer(True)

    def _play_pattern_lost(self):
        """Single long beep: card lost (failure)."""
        self.buzzer_pattern = [(0.3, 0.0)]
        self.buzzer_pattern_idx = 0
        self.buzzer_step_time = time.time()
        self._set_buzzer(True)

    def _update_buzzer(self, now):
        """Advance through the active buzzer pattern based on elapsed time."""
        if not self.buzzer_pattern:
            return

        idx = self.buzzer_pattern_idx
        if idx >= len(self.buzzer_pattern):
            if self.buzzer_on:
                self._set_buzzer(False)
            self.buzzer_pattern = []
            return

        on_dur, off_dur = self.buzzer_pattern[idx]
        elapsed = now - self.buzzer_step_time

        if self.buzzer_on:
            # Currently in "on" phase
            if elapsed >= on_dur:
                self._set_buzzer(False)
                self.buzzer_step_time = now
        else:
            # Currently in "off" phase
            if elapsed >= off_dur:
                self.buzzer_pattern_idx += 1
                if self.buzzer_pattern_idx < len(self.buzzer_pattern):
                    self._set_buzzer(True)
                    self.buzzer_step_time = now
                else:
                    self.buzzer_pattern = []


def main(args=None):
    rclpy.init(args=args)
    node = BallFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown: stop motors, buzzer, LEDs
        twist = Twist()
        node.pub_cmd_vel.publish(twist)

        buzzer_msg = Bool()
        buzzer_msg.data = False
        node.pub_buzzer.publish(buzzer_msg)

        led_msg = Int32()
        led_msg.data = int(LedMode.OFF)
        node.pub_led_mode.publish(led_msg)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
