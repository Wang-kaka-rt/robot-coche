"""
Launch configuration for ROS2 object tracking.

Launches the object tracker node with PID control for autonomous following.

Usage:
  ros2 launch tracking object_tracking.launch.py
  
With custom parameters:
  ros2 launch tracking object_tracking.launch.py target_object_size:=80.0
  ros2 launch tracking object_tracking.launch.py angular_kp:=0.02
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Arguments for customization
    target_size_arg = DeclareLaunchArgument(
        'target_object_size',
        default_value='60.0',
        description='Target object size in pixels for distance maintenance'
    )
    
    angular_kp_arg = DeclareLaunchArgument(
        'angular_kp',
        default_value='0.015',
        description='Proportional gain for angular PID controller'
    )
    
    linear_kp_arg = DeclareLaunchArgument(
        'linear_kp',
        default_value='0.008',
        description='Proportional gain for linear PID controller'
    )
    
    max_linear_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='0.25',
        description='Maximum linear speed (m/s)'
    )
    
    obstacle_dist_arg = DeclareLaunchArgument(
        'obstacle_distance_m',
        default_value='0.3',
        description='Obstacle avoidance distance (meters)'
    )
    
    # Object Tracker Node
    object_tracker = Node(
        package='tracking',
        executable='object_tracker',
        name='object_tracker',
        output='screen',
        parameters=[
            {'frame_width': 400},
            {'frame_height': 300},
            {'target_object_size': LaunchConfiguration('target_object_size')},
            {'min_object_size': 10.0},
            {'max_object_size': 150.0},
            {'center_tolerance': 30.0},
            {'lost_timeout_s': 2.0},
            {'search_timeout_s': 10.0},
            {'obstacle_distance_m': LaunchConfiguration('obstacle_distance_m')},
            {'obstacle_critical_m': 0.15},
            {'low_battery_v': 6.5},
            {'angular_kp': LaunchConfiguration('angular_kp')},
            {'angular_ki': 0.0005},
            {'angular_kd': 0.0001},
            {'linear_kp': LaunchConfiguration('linear_kp')},
            {'linear_ki': 0.0002},
            {'linear_kd': 0.00005},
            {'max_linear_speed': LaunchConfiguration('max_linear_speed')},
            {'max_angular_speed': 0.8},
            {'min_linear_speed': 0.08},
            {'min_angular_speed': 0.15},
        ],
    )
    
    return LaunchDescription([
        target_size_arg,
        angular_kp_arg,
        linear_kp_arg,
        max_linear_arg,
        obstacle_dist_arg,
        object_tracker,
    ])
