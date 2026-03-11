from setuptools import find_packages, setup

package_name = 'vision_nodes'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ball_detection.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot-coche',
    maintainer_email='robot@localhost',
    description='Vision nodes for object detection and tracking',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_detector = vision_nodes.ball_detector_node:main',
            'ball_follower = vision_nodes.ball_follower_node:main',
        ],
    },
)
