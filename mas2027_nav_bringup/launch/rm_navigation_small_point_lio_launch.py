# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("mas2027_nav_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Create the launch configuration variables、
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    small_point_lio_params_file = LaunchConfiguration(
        "small_point_lio_params_file"
    )
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    map_yaml_file = LaunchConfiguration("map")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_fake_vel_transform = LaunchConfiguration("use_fake_vel_transform")
    use_ros2_comm = LaunchConfiguration("use_ros2_comm")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_rviz = LaunchConfiguration("use_rviz")
    use_rog_map = LaunchConfiguration("use_rog_map")
    use_odom_localizer = LaunchConfiguration("use_odom_localizer")

    # Declare the launch arguments

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )
 
    declare_small_point_lio_params_file_cmd = DeclareLaunchArgument(
        "small_point_lio_params_file",
        default_value=os.path.join(
            bringup_dir, "config", "small_point_lio_params.yaml"
        ),
        description="Full path to the MID360 and Small Point-LIO parameter file",
    )

    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=os.path.join(bringup_dir, "config", "nav2_params.yaml"),
        description="Full path to the Navigation2 parameter file",
    )

    declare_map_yaml_file_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(bringup_dir, "map", "lab3.yaml"),
        description="Full path to the prior occupancy map yaml for map_server / StaticLayer",
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="True",
        description="Whether to start the robot state publisher",
    )

    declare_use_fake_vel_transform_cmd = DeclareLaunchArgument(
        "use_fake_vel_transform",
        default_value="True",
        description="Publish base_link_fake and transform Nav2 velocity into base_link",
    )
    declare_use_ros2_comm_cmd = DeclareLaunchArgument(
        "use_ros2_comm",
        default_value="True",
        description="Send /cmd_vel to the lower controller through ros2_comm",
    )

    declare_use_nav2_cmd = DeclareLaunchArgument(
        "use_nav2",
        default_value="True",
        description="Start fake velocity transform, ros2_comm, and Nav2",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    # NOTE: 必须为 True。nav2_params.yaml 里 local_costmap 和 global_costmap 的
    # 观测源都指向 /rog_map/terrain_map，由 layer_value_to_cloud 桥接节点发布。
    # 关掉之后两个 costmap 都收不到任何障碍观测，导航会直接撞障碍。
    declare_use_rog_map_cmd = DeclareLaunchArgument(
        "use_rog_map",
        default_value="True",
        description="Start the layer_value_to_cloud bridge that feeds both costmaps",
    )

    declare_use_odom_localizer_cmd = DeclareLaunchArgument(
        "use_odom_localizer",
        default_value="True",
        description=(
            "Publish dynamic map->odom via GICP against mas2027_nav_bringup/pcd/*.pcd. "
            "Set False to keep the identity static transform (mapping / map-free)."
        ),
    )

    # ROG-Map 实例的归属：整条链路只有一份 ROG-Map，由 MincoPlanner 插件在
    # planner_server 进程里建，配置在 nav2_params.yaml 的
    # planner_server.ros__parameters.MincoPlanner.rog_map 段——那是唯一一份
    # ROG-Map 参数，改参数只改那里。插件通过 rog_map::MapRegistry 内部查询 ESDF；
    # 它发布的 /rog_map/layer_value 是绝对话题名，下面的桥接节点照旧能订阅。
    #
    # 本启动文件曾提供 use_rog_map_standalone 开关来另起一个独立 rog_map 节点
    # （配 config/rog_map_params.yaml），用于 use_nav2:=False 时单看建图效果。
    # 该开关与那份参数文件已删除：两份参数要靠测试盯着才不漂移，而独立节点跑起来
    # 必须保证不与插件内那份同时存在，否则两份 rog_map 同发一个话题、滑动原点不同，
    # 桥接输出会在两套栅格之间跳变。要单独调建图，直接跑 rog_map 包自带的
    # rog_map_node 并自备参数文件（节点名 rog_map，参数前缀同为 rog_map）。

    # Create our own temporary YAML files that include substitutions

    configured_small_point_lio_params = ParameterFile(
        RewrittenYaml(
            source_file=small_point_lio_params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "robot_state_publisher_launch.py")
        ),
        # NOTE: This startup file is only used when the navigation module is standalone
        condition=IfCondition(use_robot_state_pub),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
        }.items(),
    )
    tf_map2odom = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen',
            condition=UnlessCondition(use_odom_localizer))

    start_odom_localizer_node = Node(
        package="odom_localizer",
        executable="odom_localizer_node",
        name="odom_localizer",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(use_odom_localizer),
        parameters=[
            ParameterFile(
                RewrittenYaml(
                    source_file=os.path.join(
                        get_package_share_directory("odom_localizer"),
                        "config",
                        "params.yaml",
                    ),
                    param_rewrites={
                        "prior_pcd_file": os.path.join(
                            bringup_dir, "pcd", "lab3.pcd"
                        ),
                    },
                    convert_types=True,
                ),
                allow_substs=True,
            )
        ],
    )

    # mid360启动
    start_mid360_driver_node = Node(
        package="mid360_driver",
        executable="mid360_driver_node",
        name="mid360_driver",
        output="screen",
        parameters=[configured_small_point_lio_params],
    )

    start_small_point_lio_node = Node(
        package="small_point_lio",
        executable="small_point_lio_node",
        name="small_point_lio",
        output="screen",
        parameters=[configured_small_point_lio_params],
    )

    # /rog_map/layer_value (OccupancyGrid) -> /rog_map/terrain_map (PointXYZI)
    # 供 local_costmap 的 pb_nav2_costmap_2d::IntensityVoxelLayer 使用。
    # NOTE: layer_value 是在 rog_map 的可视化定时器里发布的，所以 nav2_params.yaml 的
    # MincoPlanner.rog_map.visualization.enable 必须为 True，
    # 且 visualization.rate 就是 local_costmap 实际拿到观测的频率。
    start_rog_map_costmap_bridge_node = Node(
        package="rog_map",
        executable="layer_value_to_cloud",
        name="layer_value_to_cloud",
        output="screen",
        condition=IfCondition(use_rog_map),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/rog_map/layer_value",
                "output_topic": "/rog_map/terrain_map",
                # 只有 OCCUPIED 参与标记；PASSABLE/FREE/UNKNOWN 在 layer_value 里都是 0
                "obstacle_threshold": 100,
                # 落在 nav2_params.yaml 的 [min_obstacle_intensity 0.1, max 2.0] 内
                "point_intensity": 1.0,
                # 落在体素带 origin_z 0.0 + 0.05 x 16 = [0.0, 0.8] 内
                "point_z": 0.2,
            }
        ],
    )

    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "nav2_launch.py")),
        condition=IfCondition(use_nav2),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "nav2_params_file": nav2_params_file,
            "map": map_yaml_file,
            "perception_params_file": small_point_lio_params_file,
            "use_fake_vel_transform": use_fake_vel_transform,
            "use_ros2_comm": use_ros2_comm,
        }.items(),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file,
        }.items(),
    )


    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_nav2_params_file_cmd)
    ld.add_action(declare_map_yaml_file_cmd)
    ld.add_action(declare_small_point_lio_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_nav2_cmd)
    ld.add_action(declare_use_fake_vel_transform_cmd)
    ld.add_action(declare_use_ros2_comm_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_rog_map_cmd)
    ld.add_action(declare_use_odom_localizer_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(tf_map2odom)
    ld.add_action(start_odom_localizer_node)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_mid360_driver_node)
    ld.add_action(start_small_point_lio_node)
    ld.add_action(start_rog_map_costmap_bridge_node)
    ld.add_action(start_nav2_cmd)
    ld.add_action(rviz_cmd)

    return ld
