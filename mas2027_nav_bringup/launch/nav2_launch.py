# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory("mas2027_nav_bringup")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    perception_params_file = LaunchConfiguration("perception_params_file")
    map_yaml_file = LaunchConfiguration("map")
    use_terrain_analysis = LaunchConfiguration("use_terrain_analysis")
    use_terrain_analysis_near = LaunchConfiguration("use_terrain_analysis_near")
    use_fake_vel_transform = LaunchConfiguration("use_fake_vel_transform")
    use_ros2_comm = LaunchConfiguration("use_ros2_comm")

    configured_nav2_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params_file,
            root_key=namespace,
            param_rewrites={
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "yaml_filename": map_yaml_file,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )
    configured_perception_params = ParameterFile(
        RewrittenYaml(
            source_file=perception_params_file,
            root_key=namespace,
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation clock",
    )
    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="Automatically activate Nav2 lifecycle nodes",
    )
    declare_use_respawn = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Respawn Nav2 server processes after a crash",
    )
    declare_log_level = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Nav2 log level",
    )
    declare_nav2_params_file = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=os.path.join(bringup_dir, "config", "nav2_params.yaml"),
        description="Navigation2 parameter file",
    )
    declare_map_yaml_file = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(bringup_dir, "map", "lab3.yaml"),
        description="Full path to the prior occupancy map yaml for map_server / StaticLayer",
    )
    declare_perception_params_file = DeclareLaunchArgument(
        "perception_params_file",
        default_value=os.path.join(
            bringup_dir, "config", "small_point_lio_params.yaml"
        ),
        description="Small Point-LIO and terrain parameter file",
    )
    # 默认 False：local_costmap 和 global_costmap 的观测源都已改为
    # rog_map ProjectionLayer 经 layer_value_to_cloud 发布的 /rog_map/terrain_map，
    # /terrain_map 和 /terrain_map_ext 都没有消费者了。
    # 节点和参数块都保留，置 True 即可回到原链路做 A/B（同时要把
    # nav2_params.yaml 里两个 costmap 的 topic 改回去）。
    declare_use_terrain_analysis = DeclareLaunchArgument(
        "use_terrain_analysis",
        default_value="False",
        description="Start terrain_analysis and terrain_analysis_ext (unused since ROG-Map took over both costmaps)",
    )
    # 近区 terrain_analysis 的单独开关，与 use_terrain_analysis 取逻辑与，
    # 便于只跑 terrain_analysis_ext 做对比。
    declare_use_terrain_analysis_near = DeclareLaunchArgument(
        "use_terrain_analysis_near",
        default_value="False",
        description="Start the near-field terrain_analysis node that publishes /terrain_map",
    )
    declare_use_fake_vel_transform = DeclareLaunchArgument(
        "use_fake_vel_transform",
        default_value="True",
        description="Start base_link_fake TF and velocity conversion",
    )
    declare_use_ros2_comm = DeclareLaunchArgument(
        "use_ros2_comm",
        default_value="True",
        description="Send /cmd_vel to the lower controller through ros2_comm",
    )

    terrain_analysis = Node(
        package="terrain_analysis",
        executable="terrainAnalysis",
        name="terrainAnalysis",
        namespace=namespace,
        output="screen",
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    use_terrain_analysis,
                    "'.lower() in ('true', '1') and '",
                    use_terrain_analysis_near,
                    "'.lower() in ('true', '1')",
                ]
            )
        ),
        parameters=[configured_perception_params],
        remappings=[
            ("lidar_odometry", "/Odometry"),
            ("registered_scan", "/cloud_registered"),
        ],
    )
    terrain_analysis_ext = Node(
        package="terrain_analysis_ext",
        executable="terrainAnalysisExt",
        name="terrainAnalysisExt",
        namespace=namespace,
        output="screen",
        condition=IfCondition(use_terrain_analysis),
        parameters=[configured_perception_params],
        remappings=[
            ("lidar_odometry", "/Odometry"),
            ("registered_scan", "/cloud_registered"),
        ],
    )
    fake_vel_transform = Node(
        package="fake_vel_transform",
        executable="fake_vel_transform_node",
        name="fake_vel_transform",
        namespace=namespace,
        output="screen",
        condition=IfCondition(use_fake_vel_transform),
        parameters=[configured_perception_params],
    )

    ros2_comm = Node(
        package="ros2_comm",
        executable="ros2_comm_node",
        name="ros2_comm_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(use_ros2_comm),
    )

    tf_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    nav_arguments = ["--ros-args", "--log-level", log_level]

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        namespace=namespace,
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_nav2_params],
        arguments=nav_arguments,
        remappings=tf_remappings + [("cmd_vel", "/cmd_vel_nav")],
    )
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace=namespace,
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_nav2_params],
        arguments=nav_arguments,
        remappings=tf_remappings,
    )
    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        namespace=namespace,
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_nav2_params],
        arguments=nav_arguments,
        remappings=tf_remappings,
    )
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        namespace=namespace,
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_nav2_params],
        arguments=nav_arguments,
        remappings=tf_remappings,
    )
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        namespace=namespace,
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_nav2_params],
        arguments=nav_arguments,
        remappings=tf_remappings + [("cmd_vel", "/cmd_vel_nav")],
    )
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        namespace=namespace,
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_nav2_params],
        arguments=nav_arguments,
        remappings=tf_remappings,
    )
    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        namespace=namespace,
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_nav2_params],
        arguments=nav_arguments,
        remappings=tf_remappings,
    )
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        namespace=namespace,
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_nav2_params],
        arguments=nav_arguments,
        remappings=tf_remappings
        + [
            ("cmd_vel", "/cmd_vel_nav"),
            ("cmd_vel_smoothed", "/cmd_vel_nav_smoothed"),
        ],
    )

    lifecycle_nodes = [
        "map_server",
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        namespace=namespace,
        output="screen",
        arguments=nav_arguments,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
        ],
    )

    return LaunchDescription(
        [
            declare_namespace,
            declare_use_sim_time,
            declare_autostart,
            declare_use_respawn,
            declare_log_level,
            declare_nav2_params_file,
            declare_map_yaml_file,
            declare_perception_params_file,
            declare_use_terrain_analysis,
            declare_use_terrain_analysis_near,
            declare_use_fake_vel_transform,
            declare_use_ros2_comm,
            terrain_analysis,
            terrain_analysis_ext,
            fake_vel_transform,
            ros2_comm,
            map_server,
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            lifecycle_manager,
        ]
    )
