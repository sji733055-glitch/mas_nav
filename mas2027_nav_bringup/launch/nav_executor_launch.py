"""Standalone ROGMap + MINCO + MPC navigation, without Nav2 servers."""

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("mas2027_nav_bringup")
    executor_dir = get_package_share_directory("mas2027_nav_executor")
    launch_dir = os.path.join(bringup_dir, "launch")
    system_first_library_path = os.pathsep.join(
        ["/lib/x86_64-linux-gnu", os.environ.get("LD_LIBRARY_PATH", "")]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_ros2_comm = LaunchConfiguration("use_ros2_comm")
    use_odom_localizer = LaunchConfiguration("use_odom_localizer")
    output_topic = LaunchConfiguration("output_topic")
    map_pcd = LaunchConfiguration("map_pcd")

    lio_params = os.path.join(bringup_dir, "config", "small_point_lio_params.yaml")
    executor_params = sorted(glob.glob(os.path.join(executor_dir, "config", "*.yaml")))
    localizer_params = os.path.join(
        get_package_share_directory("odom_localizer"), "config", "params.yaml"
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "robot_state_publisher_launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    mid360_driver = Node(
        package="mid360_driver",
        executable="mid360_driver_node",
        name="mid360_driver",
        output="screen",
        parameters=[lio_params, {"use_sim_time": use_sim_time}],
    )
    small_point_lio = Node(
        package="small_point_lio",
        executable="small_point_lio_node",
        name="small_point_lio",
        output="screen",
        parameters=[lio_params, {"use_sim_time": use_sim_time, "publish_odom_tf": False}],
    )
    odom_localizer = Node(
        package="odom_localizer",
        executable="odom_localizer_node",
        name="odom_localizer",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(use_odom_localizer),
        additional_env={"LD_LIBRARY_PATH": system_first_library_path},
        parameters=[
            localizer_params,
            {
                "map.prior_pcd_file": map_pcd,
                "use_sim_time": use_sim_time,
                "tf.publish_direct": False,
            },
        ],
    )
    tf_maintainer = Node(
        package="tf_maintainer",
        executable="tf_maintainer_node",
        name="tf_maintainer",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "publish_odom_to_base": True,
                "publish_map_to_odom": True,
            }
        ],
    )
    nav_executor = Node(
        package="mas2027_nav_executor",
        executable="mas2027_nav_executor_node",
        output="screen",
        emulate_tty=True,
        additional_env={
            "LD_LIBRARY_PATH": system_first_library_path,
            "OMP_NUM_THREADS": "2",
            "OMP_WAIT_POLICY": "PASSIVE",
        },
        parameters=executor_params
        + [{"use_sim_time": use_sim_time, "node.topics.cmd_vel_pub": output_topic}],
    )
    terrain_map_server = Node(
        package="map_server",
        executable="map_server_node",
        name="terrain_map_server",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "terrain_map_path": os.path.join(bringup_dir, "map", "lab3_terrain.msgpack"),
            "frame_id": "map",
            "origin_x": -4.6,
            "origin_y": -7.94,
            "bypass_dynamic_obstacle": True,
        }],
        remappings=[
            ("cost_map", "/cost_map"),
            ("direction_map", "/direction_map"),
            ("cost_maps", "/cost_maps"),
        ],
    )
    ros2_comm = Node(
        package="ros2_comm",
        executable="ros2_comm_node",
        name="ros2_comm_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(use_ros2_comm),
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "rviz_config": os.path.join(bringup_dir, "rviz", "nav_executor_view.rviz"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="False"),
        DeclareLaunchArgument("use_rviz", default_value="True"),
        # Keep hardware output opt-in for the first on-robot test.
        DeclareLaunchArgument("use_ros2_comm", default_value="False"),
        DeclareLaunchArgument("use_odom_localizer", default_value="True"),
        DeclareLaunchArgument("output_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument(
            "map_pcd",
            default_value=os.path.join(bringup_dir, "pcd", "lab3.pcd"),
        ),
        robot_state_publisher,
        mid360_driver,
        small_point_lio,
        odom_localizer,
        tf_maintainer,
        terrain_map_server,
        nav_executor,
        ros2_comm,
        rviz,
    ])
