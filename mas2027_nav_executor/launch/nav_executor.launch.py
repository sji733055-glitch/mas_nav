import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory("mas2027_nav_executor"), "config"
    )
    configs = sorted(glob.glob(os.path.join(config_dir, "*.yaml")))
    system_library_path = os.pathsep.join(
        ["/lib/x86_64-linux-gnu", os.environ.get("LD_LIBRARY_PATH", "")]
    )
    return LaunchDescription([
        Node(
            package="mas2027_nav_executor",
            executable="mas2027_nav_executor_node",
            output="screen",
            emulate_tty=True,
            parameters=configs,
            additional_env={
                "LD_LIBRARY_PATH": system_library_path,
                "OMP_NUM_THREADS": "2",
                "OMP_WAIT_POLICY": "PASSIVE",
            },
        )
    ])
