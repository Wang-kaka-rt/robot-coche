"""
Tracking utilities for ROS2 object following.

Provides helper functions and classes for:
- Distance estimation from object size
- Smooth motion profiling
- State estimation and filtering
"""

import numpy as np
from typing import Tuple, Optional
from collections import deque


def estimate_distance(object_size_px: float, 
                      known_real_size_m: float,
                      focal_length_px: float) -> float:
    """
    Estimate distance to object based on its apparent size in image.
    
    Uses pinhole camera model: distance = (known_size * focal_length) / image_size
    
    Args:
        object_size_px: Object size in pixels (width or height)
        known_real_size_m: Known real-world size of object in meters
        focal_length_px: Camera focal length in pixels
        
    Returns:
        Estimated distance in meters
    """
    if object_size_px <= 0:
        return float('inf')
    
    distance = (known_real_size_m * focal_length_px) / object_size_px
    return distance


def calculate_focal_length(known_distance_m: float,
                           known_real_size_m: float,
                           measured_size_px: float) -> float:
    """
    Calculate camera focal length from known distance and object size.
    
    Useful for camera calibration.
    
    Args:
        known_distance_m: Known distance to object in meters
        known_real_size_m: Known real-world size of object in meters
        measured_size_px: Measured size of object in pixels
        
    Returns:
        Focal length in pixels
    """
    if measured_size_px <= 0:
        raise ValueError("Object size must be positive")
    
    focal_length = (measured_size_px * known_distance_m) / known_real_size_m
    return focal_length


class ExponentialMovingAverage:
    """
    Exponential moving average filter for smoothing sensor readings.
    
    Provides low-pass filtering to reduce noise in measurements.
    """
    
    def __init__(self, alpha: float = 0.3):
        """
        Initialize EMA filter.
        
        Args:
            alpha: Smoothing factor (0.0-1.0). Higher = less smoothing.
        """
        self.alpha = alpha
        self.value = None
    
    def update(self, new_value: float) -> float:
        """
        Update filter with new measurement.
        
        Args:
            new_value: New measurement value
            
        Returns:
            Filtered value
        """
        if self.value is None:
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1.0 - self.alpha) * self.value
        return self.value
    
    def reset(self):
        """Reset filter state."""
        self.value = None


class KalmanFilter1D:
    """
    Simple 1D Kalman filter for state estimation.
    
    Provides optimal estimation of a single variable with Gaussian noise.
    """
    
    def __init__(self, process_variance: float = 1e-5,
                 measurement_variance: float = 1e-2):
        """
        Initialize 1D Kalman filter.
        
        Args:
            process_variance: Expected variance in process dynamics
            measurement_variance: Expected variance in measurements
        """
        self.Q = process_variance  # Process noise covariance
        self.R = measurement_variance  # Measurement noise covariance
        
        self.x = 0.0  # State estimate
        self.P = 1.0  # Error covariance
    
    def predict(self, control_input: float = 0.0) -> float:
        """
        Predict next state.
        
        Args:
            control_input: Optional control input (e.g., commanded velocity)
            
        Returns:
            Predicted state
        """
        # Prediction step
        self.x = self.x + control_input
        self.P = self.P + self.Q
        return self.x
    
    def update(self, measurement: float) -> float:
        """
        Update state estimate with new measurement.
        
        Args:
            measurement: New measurement value
            
        Returns:
            Updated state estimate
        """
        # Update step
        K = self.P / (self.P + self.R)  # Kalman gain
        self.x = self.x + K * (measurement - self.x)
        self.P = (1.0 - K) * self.P
        return self.x
    
    def filter(self, measurement: float, control_input: float = 0.0) -> float:
        """
        Perform one complete filter cycle (predict + update).
        
        Args:
            measurement: New measurement
            control_input: Optional control input
            
        Returns:
            Filtered state estimate
        """
        self.predict(control_input)
        return self.update(measurement)
    
    def reset(self, initial_value: float = 0.0):
        """Reset filter state."""
        self.x = initial_value
        self.P = 1.0


class MotionProfiler:
    """
    Motion profiler for smooth velocity transitions.
    
    Generates smooth velocity profiles with acceleration limits
    to prevent jerky robot motion.
    """
    
    def __init__(self, max_accel: float = 0.5,
                 max_decel: float = 1.0,
                 dt: float = 0.05):
        """
        Initialize motion profiler.
        
        Args:
            max_accel: Maximum acceleration (m/s² or rad/s²)
            max_decel: Maximum deceleration (m/s² or rad/s²)
            dt: Time step (seconds)
        """
        self.max_accel = max_accel
        self.max_decel = max_decel
        self.dt = dt
        
        self.current_velocity = 0.0
        self.target_velocity = 0.0
    
    def set_target(self, target: float):
        """Set target velocity."""
        self.target_velocity = target
    
    def update(self) -> float:
        """
        Update velocity profile.
        
        Returns:
            Current velocity (after applying acceleration limit)
        """
        error = self.target_velocity - self.current_velocity
        
        if abs(error) < 0.001:
            self.current_velocity = self.target_velocity
            return self.current_velocity
        
        # Determine max allowed change
        if error > 0:
            max_change = self.max_accel * self.dt
        else:
            max_change = self.max_decel * self.dt
        
        # Apply acceleration limit
        if abs(error) > max_change:
            self.current_velocity += np.sign(error) * max_change
        else:
            self.current_velocity = self.target_velocity
        
        return self.current_velocity
    
    def reset(self):
        """Reset profiler state."""
        self.current_velocity = 0.0
        self.target_velocity = 0.0


def normalize_angle(angle_rad: float) -> float:
    """
    Normalize angle to [-π, π] range.
    
    Args:
        angle_rad: Angle in radians
        
    Returns:
        Normalized angle in radians
    """
    while angle_rad > np.pi:
        angle_rad -= 2.0 * np.pi
    while angle_rad < -np.pi:
        angle_rad += 2.0 * np.pi
    return angle_rad


def calculate_steering_angle(error_x: float, 
                             frame_width: int,
                             max_angle: float = 0.5) -> float:
    """
    Calculate steering angle from object position error.
    
    Args:
        error_x: Horizontal error (pixels) from image center
        frame_width: Image width in pixels
        max_angle: Maximum steering angle in radians
        
    Returns:
        Steering angle in radians
    """
    # Normalize error to [-1, 1]
    normalized_error = error_x / (frame_width / 2.0)
    normalized_error = max(-1.0, min(1.0, normalized_error))
    
    # Map to steering angle
    steering_angle = normalized_error * max_angle
    
    return steering_angle
