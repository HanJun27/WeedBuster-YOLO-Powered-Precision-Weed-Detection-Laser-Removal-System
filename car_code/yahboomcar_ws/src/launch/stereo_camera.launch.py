"""
stereo_camera.launch.py
运行：
    ros2 launch laser_calibration stereo_camera.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='laser_calibration',
            executable='stereo_camera',
            name='stereo_camera',
            output='screen',
            emulate_tty=True,
        ),
    ])
