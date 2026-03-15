"""
Object Tracker Node for ROS2.

Advanced object tracking with PID control for smooth autonomous following.
Implements proportional-integral-derivative control for precise tracking.

Features:
- PID-based smooth motion control
- Adaptive speed based on object distance
- Multi-state tracking (LOST, SEARCHING, TRACKING, FOLLOWING)
- Obstacle avoidance integration
- Battery monitoring and low-power behavior
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import Range, BatteryState
from std_msgs.msg import Bool, Float32MultiArray, Int32
import threading
import time
from enum import IntEnum
from typing import Optional, Tuple


class TrackingState(IntEnum):
    """Tracking state machine states."""
    LOST = 0          # No object detected, searching
    SEARCHING = 1     # Recently lost, active search
    TRACKING = 2      # Object detected, adjusting position
    FOLLOWING = 3     # Object tracked, moving towards it


class PIDController:
    """
    PID controller for smooth motion control.
    
    Implements discrete-time PID control with:
    - Proportional term for immediate response
    - Integral term for eliminating steady-state error
    - Derivative term for damping oscillations
    """
    
    def __init__(self, kp: float, ki: float, kd: float, 
                 integral_limit: float = 1.0,
                 output_limit: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.time()
    
    def reset(self):
        """Reset PID state."""
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.time()
    
    def compute(self, error: float) -> float:
        """
        Compute PID output for given error.
        
        Args:
            error: Current error value
            
        Returns:
            Control output (clamped to output_limit)
        """
        now = time.time()
        dt = max(0.01, now - self.last_time)  # Prevent division by zero
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.previous_error) / dt
        d_term = self.kd * derivative
        
        # Compute total output
        output = p_term + i_term + d_term
        
        # Clamp output
        output = max(-self.output_limit, min(self.output_limit, output))
        
        # Update state
        self.previous_error = error
        self.last_time = now
        
        return output


class ObjectTrackerNode(Node):
    """
    ROS2 node for autonomous object tracking with PID control.
    
    Subscriptions:
        - /ball_detection/status: Object detection status
        - /ball_detection/position: Object position in image
        - /ball_detection/radius: Object size/distance estimate
        - /ultrasonic/range: Obstacle distance
        - /battery: Battery voltage
        
    Publications:
        - /cmd_vel: Velocity commands for robot base
        - /tracking/state: Current tracking state
    """
    
    def __init__(self):
        super().__init__('object_tracker_node')
        
        # === Parameters ===
        self.declare_parameter('frame_width', 400)
        self.declare_parameter('frame_height', 300)
        self.declare_parameter('target_object_size', 60.0)
        self.declare_parameter('min_object_size', 10.0)
        self.declare_parameter('max_object_size', 150.0)
        self.declare_parameter('center_tolerance', 30.0)
        self.declare_parameter('lost_timeout_s', 2.0)
        self.declare_parameter('search_timeout_s', 10.0)
        self.declare_parameter('obstacle_distance_m', 0.3)
        self.declare_parameter('obstacle_critical_m', 0.15)
        self.declare_parameter('low_battery_v', 6.5)
        
        # PID parameters for angular control (left/right turning)
        self.declare_parameter('angular_kp', 0.015)
        self.declare_parameter('angular_ki', 0.0005)
        self.declare_parameter('angular_kd', 0.0001)
        
        # PID parameters for linear control (forward/backward)
        self.declare_parameter('linear_kp', 0.008)
        self.declare_parameter('linear_ki', 0.0002)
        self.declare_parameter('linear_kd', 0.00005)
        
        # Speed limits
        self.declare_parameter('max_linear_speed', 0.25)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('min_linear_speed', 0.08)
        self.declare_parameter('min_angular_speed', 0.15)
        
        # Get parameters
        self.frame_w = self.get_parameter('frame_width').value
        self.frame_h = self.get_parameter('frame_height').value
        self.target_size = self.get_parameter('target_object_size').value
        self.min_size = self.get_parameter('min_object_size').value
        self.max_size = self.get_parameter('max_object_size').value
        self.center_tol = self.get_parameter('center_tolerance').value
        self.lost_timeout = self.get_parameter('lost_timeout_s').value
        self.search_timeout = self.get_parameter('search_timeout_s').value
        self.obstacle_dist = self.get_parameter('obstacle_distance_m').value
        self.obstacle_critical = self.get_parameter('obstacle_critical_m').value
        self.low_battery_v = self.get_parameter('low_battery_v').value
        
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.min_linear = self.get_parameter('min_linear_speed').value
        self.min_angular = self.get_parameter('min_angular_speed').value
        
        # Initialize PID controllers
        self.angular_pid = PIDController(
            kp=self.get_parameter('angular_kp').value,
            ki=self.get_parameter('angular_ki').value,
            kd=self.get_parameter('angular_kd').value,
            output_limit=self.max_angular
        )
        
        self.linear_pid = PIDController(
            kp=self.get_parameter('linear_kp').value,
            ki=self.get_parameter('linear_ki').value,
            kd=self.get_parameter('linear_kd').value,
            output_limit=self.max_linear
        )
        
        # === State ===
        self.state = TrackingState.LOST
        self.object_detected = False
        self.last_detection_time = 0.0
        self.object_position = Point()
        self.object_size = 0.0
        self.last_error_x = 0.0  # For search direction
        self.follow_enabled = False
        
        self.obstacle_range = float('inf')
        self.battery_voltage = 0.0
        self.battery_low = False
        
        self.state_lock = threading.Lock()
        
        # === Publishers ===
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_state = self.create_publisher(Int32, '/tracking/state', 10)
        
        # === Subscribers ===
        self.create_subscription(Bool, '/ball_detection/status', self._on_status, 10)
        self.create_subscription(Point, '/ball_detection/position', self._on_position, 10)
        self.create_subscription(Float32MultiArray, '/ball_detection/radius', self._on_size, 10)
        self.create_subscription(Range, '/ultrasonic/range', self._on_obstacle, 10)
        self.create_subscription(BatteryState, '/battery', self._on_battery, 10)
        self.create_subscription(Bool, '/follow_enable', self._on_follow_enable, 10)
        
        # === Control loop at 20 Hz ===
        self.create_timer(0.05, self._control_loop)
        
        self.get_logger().info('Object Tracker Node initialized with PID control')
        self.get_logger().info(f'  PID angular: kp={self.angular_pid.kp}, ki={self.angular_pid.ki}, kd={self.angular_pid.kd}')
        self.get_logger().info(f'  PID linear: kp={self.linear_pid.kp}, ki={self.linear_pid.ki}, kd={self.linear_pid.kd}')
    
    # === Callbacks ===
    
    def _on_status(self, msg: Bool) -> None:
        with self.state_lock:
            self.object_detected = msg.data
            if msg.data:
                self.last_detection_time = time.time()
    
    def _on_position(self, msg: Point) -> None:
        with self.state_lock:
            self.object_position = msg
            self.last_error_x = (self.frame_w / 2.0) - msg.x
    
    def _on_size(self, msg: Float32MultiArray) -> None:
        with self.state_lock:
            if len(msg.data) > 0:
                self.object_size = msg.data[0]
    
    def _on_obstacle(self, msg: Range) -> None:
        with self.state_lock:
            self.obstacle_range = msg.range
    
    def _on_battery(self, msg: BatteryState) -> None:
        with self.state_lock:
            self.battery_voltage = msg.voltage
            was_low = self.battery_low
            self.battery_low = 0.5 < msg.voltage < self.low_battery_v
            if self.battery_low and not was_low:
                self.get_logger().warn(f'LOW BATTERY: {msg.voltage:.1f}V')

    def _on_follow_enable(self, msg: Bool) -> None:
        with self.state_lock:
            self.follow_enabled = bool(msg.data)
    
    # === Main Control Loop ===
    
    def _control_loop(self) -> None:
        now = time.time()
        
        # Snapshot state
        with self.state_lock:
            detected = self.object_detected
            position = Point(x=self.object_position.x, y=self.object_position.y)
            size = self.object_size
            last_det_time = self.last_detection_time
            obstacle = self.obstacle_range
            last_err_x = self.last_error_x
            follow_enabled = self.follow_enabled
        
        time_since_det = (now - last_det_time) if last_det_time > 0 else float('inf')

        if not follow_enabled:
            if self.state != TrackingState.LOST:
                self._on_state_change(self.state, TrackingState.LOST)
                self.state = TrackingState.LOST
            state_msg = Int32()
            state_msg.data = int(self.state)
            self.pub_state.publish(state_msg)
            self._send_command(0.0, 0.0, 0.0)
            return

        # Determine state
        new_state = self._determine_state(detected, position, size, time_since_det, obstacle)
        
        # Handle state transition
        if new_state != self.state:
            self._on_state_change(self.state, new_state)
            self.state = new_state
        
        # Publish state
        state_msg = Int32()
        state_msg.data = int(self.state)
        self.pub_state.publish(state_msg)
        
        # Execute behavior
        if self.state == TrackingState.LOST:
            self._behavior_lost()
        elif self.state == TrackingState.SEARCHING:
            self._behavior_search(last_err_x)
        elif self.state == TrackingState.TRACKING:
            self._behavior_tracking(position, size)
        elif self.state == TrackingState.FOLLOWING:
            self._behavior_following(position, size, obstacle)
    
    # === State Machine ===
    
    def _determine_state(self, detected: bool, position: Point, size: float,
                         time_since_det: float, obstacle: float) -> TrackingState:
        # Critical obstacle override
        if obstacle < self.obstacle_critical and self.state != TrackingState.LOST:
            return TrackingState.LOST
        
        # Object detected and within size range
        if detected and self.min_size <= size <= self.max_size:
            # Check if well-centered for following
            error_x = abs((self.frame_w / 2.0) - position.x)
            if error_x < self.center_tol:
                return TrackingState.FOLLOWING
            return TrackingState.TRACKING
        
        # Grace period after losing object
        if time_since_det < self.lost_timeout:
            return TrackingState.TRACKING
        
        # Active search phase
        if time_since_det < self.lost_timeout + self.search_timeout:
            return TrackingState.SEARCHING
        
        return TrackingState.LOST
    
    def _on_state_change(self, old_state: TrackingState, new_state: TrackingState):
        self.get_logger().info(f'State: {old_state.name} -> {new_state.name}')
        
        # Reset PID on state change to prevent windup
        if new_state != old_state:
            self.angular_pid.reset()
            self.linear_pid.reset()
    
    # === Behaviors ===
    
    def _behavior_lost(self):
        """Stop all motion when object is lost."""
        self._send_command(0.0, 0.0, 0.0)
    
    def _behavior_search(self, last_error_x: float):
        """Rotate to search for lost object."""
        direction = 1.0 if last_error_x >= 0 else -1.0
        search_speed = max(self.min_angular, self.max_angular * 0.5)
        self._send_command(0.0, 0.0, direction * search_speed)
    
    def _behavior_tracking(self, position: Point, size: float):
        """
        Adjust position to center object in camera view.
        Uses PID control for smooth turning.
        """
        frame_center_x = self.frame_w / 2.0
        error_x = frame_center_x - position.x
        
        # Normalize error to [-1, 1]
        error_normalized = error_x / (self.frame_w / 2.0)
        
        # PID control for angular velocity
        angular_vel = self.angular_pid.compute(error_normalized)
        
        # Ensure minimum turning speed for responsiveness
        if abs(error_x) > self.center_tol and abs(angular_vel) < self.min_angular:
            angular_vel = self.min_angular * (1.0 if angular_vel >= 0 else -1.0)
        
        # Move forward slowly while tracking
        linear_vel = self.min_linear
        
        # Adjust speed based on object size (distance)
        size_ratio = size / self.target_size
        if size_ratio > 1.5:
            linear_vel = 0.0  # Too close, stop
        
        self._send_command(linear_vel, 0.0, angular_vel)
    
    def _behavior_following(self, position: Point, size: float, obstacle: float):
        """
        Follow object while maintaining safe distance.
        Uses PID control for both linear and angular motion.
        """
        frame_center_x = self.frame_w / 2.0
        
        # Angular control: keep object centered
        error_x = frame_center_x - position.x
        error_x_normalized = error_x / (self.frame_w / 2.0)
        angular_vel = self.angular_pid.compute(error_x_normalized)
        
        # Linear control: maintain target distance (based on object size)
        size_error = self.target_size - size
        size_error_normalized = size_error / self.target_size
        linear_vel = self.linear_pid.compute(size_error_normalized)
        
        # Ensure minimum speeds
        if abs(linear_vel) < self.min_linear and abs(linear_vel) > 0.01:
            linear_vel = self.min_linear * (1.0 if linear_vel >= 0 else -1.0)
        
        if abs(angular_vel) < self.min_angular and abs(angular_vel) > 0.01:
            angular_vel = self.min_angular * (1.0 if angular_vel >= 0 else -1.0)
        
        # Obstacle avoidance: reduce speed near obstacles
        if obstacle < self.obstacle_dist:
            safe_range = self.obstacle_dist - self.obstacle_critical
            if safe_range > 0:
                obstacle_factor = max(0.2, (obstacle - self.obstacle_critical) / safe_range)
                linear_vel *= obstacle_factor
                self.get_logger().debug(f'Obstacle detected: {obstacle:.2f}m, slowing down')
        
        # Safety limits
        if size > self.max_size:
            linear_vel = -self.min_linear  # Back up if too close
        
        self._send_command(linear_vel, 0.0, angular_vel)
    
    # === Helper Methods ===
    
    def _send_command(self, linear_x: float, linear_y: float, angular_z: float):
        """Publish velocity command to robot base."""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.linear.y = float(linear_y)
        twist.angular.z = float(angular_z)
        self.pub_cmd_vel.publish(twist)


def main(args=None):
    """Entry point for object tracker node."""
    rclpy.init(args=args)
    node = ObjectTrackerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown: stop robot
        twist = Twist()
        node.pub_cmd_vel.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
