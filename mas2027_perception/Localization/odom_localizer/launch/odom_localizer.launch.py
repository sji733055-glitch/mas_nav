import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_yaml = os.path.join(
        get_package_share_directory("odom_localizer"), "config", "params.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="odom_localizer",
                executable="odom_localizer_node",
                name="odom_localizer",
                output="screen",
                emulate_tty=True,
                parameters=[config_yaml],
            )
        ]
    )
