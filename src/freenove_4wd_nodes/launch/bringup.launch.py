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
        executable="libcamera_camera_node",
        name="camera",
        output="screen",
    )

    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
    )

    return LaunchDescription([car, camera, rosbridge])
