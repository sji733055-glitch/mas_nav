from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def _default_map_yaml():
    """lab3 prior map. Do not depend on mas2027_nav_bringup at parse time."""
    try:
        bringup = get_package_share_directory("mas2027_nav_bringup")
        candidate = os.path.join(bringup, "map", "lab3.yaml")
        if os.path.isfile(candidate):
            return candidate
    except PackageNotFoundError:
        pass
    return "/home/ros2_ws/src/mas2027_nav_bringup/map/lab3.yaml"


def generate_launch_description():
    pkg = get_package_share_directory("waypoint_editor")
    rviz_config = os.path.join(pkg, "rviz", "rviz_waypoint_editor.rviz")

    map_yaml_file = LaunchConfiguration("map_yaml")
    use_map_server = LaunchConfiguration("use_map_server")

    declare_map_yaml = DeclareLaunchArgument(
        "map_yaml",
        default_value=_default_map_yaml(),
        description="Prior occupancy map yaml in the map frame. Default is lab3.",
    )
    declare_use_map_server = DeclareLaunchArgument(
        "use_map_server",
        default_value="true",
        description=(
            "Start a dedicated map_server. Set false when Nav2 is already running "
            "(its map_server already publishes /map). Do not run two map_servers."
        ),
    )

    map_server = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace="",
        output="screen",
        parameters=[{"yaml_filename": map_yaml_file}],
        condition=IfCondition(use_map_server),
    )

    lifecycle_mgr = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        namespace="",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": ["map_server"],
        }],
        condition=IfCondition(use_map_server),
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        declare_map_yaml,
        declare_use_map_server,
        map_server,
        lifecycle_mgr,
        rviz2,
    ])
