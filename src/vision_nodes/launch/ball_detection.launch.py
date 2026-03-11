"""
Launch configuration for red card detection and autonomous following.

Launches two nodes:
  1. ball_detector - Detects red cards in camera images (HSV segmentation)
  2. ball_follower - Follows detected cards with obstacle avoidance

Usage:
  ros2 launch vision_nodes ball_detection.launch.py
  ros2 launch vision_nodes ball_detection.launch.py camera_topic:=/image_raw/compressed
  ros2 launch vision_nodes ball_detection.launch.py publish_debug_images:=True
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Arguments
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/image_raw/compressed',
        description='Camera topic to subscribe to'
    )

    publish_debug_arg = DeclareLaunchArgument(
        'publish_debug_images',
        default_value='False',
        description='Whether to publish debug visualization images'
    )

    # Nodes
    ball_detector = Node(
        package='vision_nodes',
        executable='ball_detector',
        name='ball_detector',
        output='screen',
        parameters=[
            {'camera_topic': LaunchConfiguration('camera_topic')},
            {'publish_debug_images': LaunchConfiguration('publish_debug_images')},
            {'min_ball_area': 100},
            {'max_ball_area': 100000},
        ],
    )

    ball_follower = Node(
        package='vision_nodes',
        executable='ball_follower',
        name='ball_follower',
        output='screen',
        parameters=[
            {'frame_width': 400},
            {'frame_height': 300},
            {'linear_speed': 0.15},
            {'angular_speed': 0.4},
            {'min_card_size': 15},
            {'max_card_size': 120},
            {'target_card_size': 55},
            {'center_tolerance': 40},
            {'lost_timeout_s': 1.5},
            {'search_timeout_s': 8.0},
            {'obstacle_distance_m': 0.25},
            {'obstacle_critical_m': 0.12},
            {'low_battery_v': 6.5},
            {'search_angular_speed': 0.25},
        ],
    )

    return LaunchDescription([
        camera_topic_arg,
        publish_debug_arg,
        ball_detector,
        ball_follower,
    ])
