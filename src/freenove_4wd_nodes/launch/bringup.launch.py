from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    car = Node(
        package="freenove_4wd_nodes",
        executable="car_base_node",
        name="car_base",
        output="screen",
    )

    camera = Node(
        package="freenove_4wd_nodes",
        executable="camera_node",
        name="camera",
        output="screen",
    )

    return LaunchDescription([car, camera])

