from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # See: https://github.com/ros2/rosidl_python/issues/79
        SetEnvironmentVariable('PYTHONOPTIMIZE', '1'),

        Node(
            package='reachy_controllers',
            executable='joint_state_controller',
        ),
        Node(
            package='reachy_controllers',
            executable='camera_publisher',
        ),
        Node(
            package='reachy_controllers',
            executable='camera_zoom_service',
        ),
        Node(
            package='reachy_controllers',
            executable='gripper_controller',
        ),
    ])
