"""Standalone map_server + MINCO + MPC navigation bringup."""

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
        additional_env={"LD_LIBRARY_PATH": system_first_library_path},
        parameters=[{
            "use_sim_time": use_sim_time,
            "terrain_map_path": os.path.join(bringup_dir, "map", "lab3_terrain.msgpack"),
            "global_cloud_path": map_pcd,
            "frame_id": "map",
            "origin_x": -4.6,
            "origin_y": -7.94,
            # True = 不做动态障碍检测（map_server_node.cpp:67 起连点云都不订阅），
            # /dynamic_cost_map 保持全空，/cost_map 与 /direction_map 照常发布。
            # 理由：旧工程 /home/mas/mas_nav_2027 的 mas2027_perception 下根本没有 map_server，
            # 动态物体靠 ROGMap 的时间衰减（keep_time 0.8 s / clear_time 1.2 s）处理；本工程多出的
            # 这一层会按 full_cost 0.2 m / cutoff 0.4 m 膨胀，RViz 里明显比实物厚，并触发
            # 「Braking: current dynamic obstacle intersects the MPC reference horizon」，
            # 是实车「不丝滑」的一大来源。需要动态避障时改回 False。
            "bypass_dynamic_obstacle": True,
        }],
        remappings=[
            ("cost_map", "/cost_map"),
            ("direction_map", "/direction_map"),
            ("dynamic_cost_map", "/dynamic_cost_map"),
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
        # 底盘转发桥默认开启：ros2_comm 是 /cmd_vel 的唯一消费者，不启动它就会出现
        # 「cmd_vel 一直有值但车不动」。协议只发 vx/vy/nav_state，不含角速度。
        # 上机前确认底盘上电与急停状态；只想看导航不发车时用 use_ros2_comm:=False 关掉。
        DeclareLaunchArgument("use_ros2_comm", default_value="True"),
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
