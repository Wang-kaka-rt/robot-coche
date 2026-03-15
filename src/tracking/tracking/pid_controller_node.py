"""
Standalone PID Controller Node for ROS2.

Provides a reusable PID controller service that can be used
by other nodes for precise motion control.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float32, Float32MultiArray
import time
from typing import Optional


class PIDControllerNode(Node):
    """
    Generic PID controller node for ROS2.
    
    This node provides PID-based control for tracking setpoints.
    It can be configured via parameters for different control tasks.
    
    Subscriptions:
        - /pid/setpoint: Target value (Float32)
        - /pid/process_variable: Current measured value (Float32)
        
    Publications:
        - /pid/output: PID controller output (Float32)
        - /pid/error: Current error (Float32)
    """
    
    def __init__(self):
        super().__init__('pid_controller_node')
        
        # Parameters
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.05)
        self.declare_parameter('integral_limit', 10.0)
        self.declare_parameter('output_min', -1.0)
        self.declare_parameter('output_max', 1.0)
        self.declare_parameter('setpoint', 0.0)
        
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.integral_limit = self.get_parameter('integral_limit').value
        self.output_min = self.get_parameter('output_min').value
        self.output_max = self.get_parameter('output_max').value
        self.setpoint = self.get_parameter('setpoint').value
        
        # PID state
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.time()
        
        # Current values
        self.current_value = 0.0
        self.current_setpoint = self.setpoint
        
        # Publishers
        self.pub_output = self.create_publisher(Float32, '/pid/output', 10)
        self.pub_error = self.create_publisher(Float32, '/pid/error', 10)
        
        # Subscribers
        self.create_subscription(Float32, '/pid/setpoint', self._on_setpoint, 10)
        self.create_subscription(Float32, '/pid/process_variable', self._on_pv, 10)
        
        # Control loop at 50 Hz
        self.create_timer(0.02, self._control_loop)
        
        self.get_logger().info('PID Controller initialized')
        self.get_logger().info(f'  Kp={self.kp}, Ki={self.ki}, Kd={self.kd}')
    
    def _on_setpoint(self, msg: Float32) -> None:
        self.current_setpoint = msg.data
    
    def _on_pv(self, msg: Float32) -> None:
        self.current_value = msg.data
    
    def _control_loop(self):
        now = time.time()
        dt = max(0.001, now - self.last_time)
        
        # Compute error
        error = self.current_setpoint - self.current_value
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.previous_error) / dt
        d_term = self.kd * derivative
        
        # Total output
        output = p_term + i_term + d_term
        
        # Clamp output
        output = max(self.output_min, min(self.output_max, output))
        
        # Publish
        output_msg = Float32()
        output_msg.data = output
        self.pub_output.publish(output_msg)
        
        error_msg = Float32()
        error_msg.data = error
        self.pub_error.publish(error_msg)
        
        # Update state
        self.previous_error = error
        self.last_time = now


def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
