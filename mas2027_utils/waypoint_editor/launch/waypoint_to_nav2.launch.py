from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=(
            "Do not use waypoint_to_nav2 (Nav2 FollowWaypoints has no per-point "
            "timeout on Humble). Drive the CSV with:\n"
            "  ros2 launch mas2027_nav_bringup waypoint_navigator.launch.py "
            "waypoint_file:=/path/to/lab3_patrol.csv"
        )),
    ])
