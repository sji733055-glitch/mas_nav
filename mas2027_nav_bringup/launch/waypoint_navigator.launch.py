from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    waypoint_file = LaunchConfiguration("waypoint_file")
    frame_id = LaunchConfiguration("frame_id")
    loop = LaunchConfiguration("loop")
    start_delay_sec = LaunchConfiguration("start_delay_sec")
    goal_timeout_sec = LaunchConfiguration("goal_timeout_sec")

    return LaunchDescription([
        DeclareLaunchArgument(
            "waypoint_file",
            default_value="",
            description="CSV from waypoint_editor (id,pose_x,pose_y,...,rot_z,rot_w).",
        ),
        DeclareLaunchArgument("frame_id", default_value="map"),
        DeclareLaunchArgument("loop", default_value="true"),
        DeclareLaunchArgument("start_delay_sec", default_value="8.0"),
        DeclareLaunchArgument("goal_timeout_sec", default_value="60.0"),
        Node(
            package="mas2027_nav_bringup",
            executable="waypoint_navigator.py",
            name="waypoint_navigator",
            output="screen",
            parameters=[{
                "waypoint_file": waypoint_file,
                "frame_id": frame_id,
                "loop": ParameterValue(loop, value_type=bool),
                "start_delay_sec": ParameterValue(start_delay_sec, value_type=float),
                "goal_timeout_sec": ParameterValue(goal_timeout_sec, value_type=float),
                "attempts_per_waypoint": 2,
                "stop_on_failure": False,
            }],
        ),
    ])
