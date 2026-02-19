from setuptools import find_packages, setup

package_name = "freenove_4wd_nodes"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/bringup.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="ROS2 (rclpy) nodes for Freenove 4WD Smart Car.",
    license="CC-BY-NC-SA-3.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "car_base_node = freenove_4wd_nodes.car_base_node:main",
            "camera_node = freenove_4wd_nodes.camera_node:main",
        ],
    },
)

