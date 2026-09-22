#!/usr/bin/env python3
"""Check map_server -> goal -> trajectory in an isolated ROS_DOMAIN_ID."""
import argparse
import math
import os
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from tf2_ros import StaticTransformBroadcaster


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("terrain_map")
    parser.add_argument("executor_config_dir")
    parser.add_argument(
        "--map-yaml",
        help="matching Nav2 map YAML (defaults to <name>.yaml beside <name>_terrain.msgpack)",
    )
    parser.add_argument("--goal", default="2.56226,0.437201", help="map-frame goal x,y")
    parser.add_argument("--preempt", action="store_true",
                        help="Send a newer goal and verify that its path replaces the first")
    args = parser.parse_args()
    map_yaml = args.map_yaml
    if map_yaml is None:
        suffix = "_terrain.msgpack"
        if not args.terrain_map.endswith(suffix):
            raise SystemExit("--map-yaml is required when terrain map lacks _terrain.msgpack suffix")
        map_yaml = args.terrain_map[:-len(suffix)] + ".yaml"
    goal_x, goal_y = (float(value) for value in args.goal.split(","))
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = "/lib/x86_64-linux-gnu:" + env.get("LD_LIBRARY_PATH", "")
    # 先把配置复制一份再改掉 ROGMap 的性能 CSV 路径：PerformanceMonitor 以 trunc 模式打开
    # planner_params.yaml 里的 CSV，直接跑冒烟会把实车那一次的数据覆盖成一行冒烟样本。
    config_dir = os.path.abspath(args.executor_config_dir)
    smoke_run_dir = os.path.join(os.getcwd(), ".scratch", "smoke_run")
    os.makedirs(smoke_run_dir, exist_ok=True)
    resolved_config_dir = os.path.join(smoke_run_dir, "config")
    os.makedirs(resolved_config_dir, exist_ok=True)
    for name in ("mpc_params.yaml", "node_params.yaml", "planner_params.yaml"):
        with open(os.path.join(config_dir, name)) as src:
            text = src.read()
        if name == "planner_params.yaml":
            csv_names = {"summary_csv_path": "rog_map_summary.csv",
                         "detailed_csv_path": "rog_map_detailed.csv"}
            lines = text.splitlines()
            for key, filename in csv_names.items():
                target = f'{key}: "{os.path.join(smoke_run_dir, filename)}"'
                existing = [l for l in lines if l.strip().startswith(key + ":")]
                if existing:
                    for line in existing:
                        indent = line[:len(line) - len(line.lstrip())]
                        text = text.replace(line, indent + target)
                    continue
                # 配置没显式写 detailed_csv_path 时默认值指向 /tmp、冒烟侧读不到，就地补一行并对齐缩进。
                anchor = next((l for l in lines
                               if l.strip().startswith("summary_rate:")), None)
                if anchor is None:
                    raise RuntimeError(
                        f"cannot locate summary_rate in {name} to inject {key}")
                indent = anchor[:len(anchor) - len(anchor.lstrip())]
                text = text.replace(anchor, indent + target + "\n" + anchor)
                lines = text.splitlines()
        with open(os.path.join(resolved_config_dir, name), "w") as dst:
            dst.write(text)
    map_exe = os.path.join(get_package_prefix("map_server"), "lib/map_server/map_server_node")
    nav_exe = os.path.join(get_package_prefix("mas2027_nav_executor"),
                           "lib/mas2027_nav_executor/mas2027_nav_executor_node")
    server = subprocess.Popen([
        map_exe, "--ros-args", "-p", f"terrain_map_path:={args.terrain_map}",
        "-p", f"map_yaml_path:={map_yaml}",
    ], env=env)
    nav = subprocess.Popen([
        nav_exe, "--ros-args",
        *[item for name in ("mpc_params.yaml", "node_params.yaml", "planner_params.yaml")
          for item in ("--params-file", os.path.join(resolved_config_dir, name))],
    ], env=env)
    rclpy.init()
    node = rclpy.create_node("goal_smoke")
    broadcaster = StaticTransformBroadcaster(node)
    tf = TransformStamped()
    tf.header.stamp = node.get_clock().now().to_msg()
    tf.header.frame_id = "map"
    tf.child_frame_id = "odom"
    tf.transform.rotation.w = 1.0
    base_tf = TransformStamped()
    base_tf.header.stamp = tf.header.stamp
    base_tf.header.frame_id = "odom"
    base_tf.child_frame_id = "base_link"
    base_tf.transform.rotation.w = 1.0
    broadcaster.sendTransform([tf, base_tf])
    odom_pub = node.create_publisher(Odometry, "/Odometry", 10)
    cloud_pub = node.create_publisher(point_cloud2.PointCloud2, "/cloud_registered", 10)
    goal_pub = node.create_publisher(PoseStamped, "/goal_pose", 10)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    # 折线与终点球是同一 tick 背靠背发的两条 Marker；depth=1 的订阅队列会把先到的
    # 折线覆盖掉，只剩终点球，所以这里必须用更大的队列。
    marker_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
    terrain_maps = []
    paths = []
    global_plans = []
    global_markers = []
    node.create_subscription(OccupancyGrid, "/cost_map", terrain_maps.append, qos)
    node.create_subscription(Path, "/nav_executor/minco_path", paths.append, qos)
    node.create_subscription(Path, "/nav_executor/global_plan", global_plans.append, qos)
    node.create_subscription(Marker, "/nav_executor/debug/global_plan",
                             global_markers.append, marker_qos)
    goal_sent = False
    second_sent = False
    first_end_x = None
    first_map_at = None
    try:
        deadline = time.monotonic() + 25.0
        while time.monotonic() < deadline:
            odom = Odometry()
            odom.header.stamp = node.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                Header(stamp=odom.header.stamp, frame_id="map"),
                [(1.0 + 0.07 * x, 1.0 + 0.07 * y, -0.25)
                 for x in range(11) for y in range(11)]))
            rclpy.spin_once(node, timeout_sec=0.1)
            if server.poll() is not None or nav.poll() is not None:
                raise RuntimeError(f"node exited: map_server={server.poll()} nav_executor={nav.poll()}")
            if terrain_maps and first_map_at is None:
                first_map_at = time.monotonic()
            if first_map_at is not None and time.monotonic() - first_map_at >= 2.0 and not goal_sent:
                goal = PoseStamped()
                goal.header.stamp = node.get_clock().now().to_msg()
                goal.header.frame_id = "map"
                goal.pose.position.x = goal_x
                goal.pose.position.y = goal_y
                goal.pose.orientation.w = 1.0
                goal_pub.publish(goal)
                goal_sent = True
            if paths and goal_sent:
                if not args.preempt:
                    # 全局折线走 5 Hz 定时器，比轨迹晚一拍；而且 spin_once 一次只处理一个
                    # 回调，所以必须等到指定的那条 LINE_STRIP 到齐，不能见到任意 Marker 就走。
                    if global_plans and any(
                            m.ns == "global_plan" and m.type == Marker.LINE_STRIP
                            for m in global_markers):
                        break
                elif not second_sent:
                    first_end_x = paths[-1].poses[-1].pose.position.x
                    goal = PoseStamped()
                    goal.header.stamp = node.get_clock().now().to_msg()
                    goal.header.frame_id = "map"
                    goal.pose.position.x = goal_x + 0.43774
                    goal.pose.position.y = goal_y
                    goal.pose.orientation.w = 1.0
                    goal_pub.publish(goal)
                    second_sent = True
                elif (paths[-1].poses[-1].pose.position.x > first_end_x + 0.25
                      and any(len(p.poses) >= 2
                              and abs(p.poses[-1].pose.position.x
                                      - (goal_x + 0.43774)) < 1e-6
                              for p in global_plans)
                      and any(m.ns == "global_plan" and m.type == Marker.LINE_STRIP
                              and m.points
                              and abs(m.points[-1].x - (goal_x + 0.43774)) < 1e-6
                              for m in global_markers)):
                    # Path 与 Marker 同一 tick 背靠背发出，回调逐个处理；两条都要等到，
                    # 否则断言会在 Marker 回调之前就跑完（表现为 global_markers 为空）。
                    break
        assert goal_sent, "static terrain map was not ready"
        assert paths and len(paths[-1].poses) >= 2, "goal produced no path"
        if args.preempt:
            assert second_sent and paths[-1].poses[-1].pose.position.x > first_end_x + 0.25, \
                "newer goal did not replace the active path"

        # 全局搜索折线必须真的发出来，否则 RViz 里那条 "Global Plan" 是空的。
        # 换目标时 invalidateGlobalPath 会先发一条空的 Path 去清 RViz，所以取最后一条非空的。
        assert global_plans, "no message on /nav_executor/global_plan"
        plan = next((p for p in reversed(global_plans) if len(p.poses) >= 2), None)
        assert plan is not None, \
            f"no non-empty global plan; got sizes {[len(p.poses) for p in global_plans]}"
        expected_end_x = goal_x + 0.43774 if args.preempt else goal_x
        assert len(plan.poses) >= 2, f"global plan has {len(plan.poses)} poses"
        assert plan.header.frame_id == "odom", f"global plan frame is {plan.header.frame_id!r}"
        for pose in plan.poses:
            for value in (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z):
                assert math.isfinite(value), "global plan contains a non-finite coordinate"
        # 折线末端被 makePlanOnQuery 贴到精确目标上，所以应当与目标点重合。
        end = plan.poses[-1].pose.position
        assert abs(end.x - expected_end_x) < 1e-6 and abs(end.y - goal_y) < 1e-6, \
            f"global plan does not end at the goal: ({end.x}, {end.y})"
        # 判别性检查：必须是搜索输出的格点路径，而非被接回 MINCO 的轨迹（后者按 dt 采样、间距小一量级）；
        # SMAC 在 0.05 m 格上扩展，步长仅 0.05（直走）或 0.0707 m（斜走 √2）；末段为贴到精确目标的跳变段，排除。
        steps = [math.dist((a.pose.position.x, a.pose.position.y),
                           (b.pose.position.x, b.pose.position.y))
                 for a, b in zip(plan.poses, plan.poses[1:])]
        lattice_steps = steps[:-1]
        assert lattice_steps, "global plan has no consecutive lattice pairs"
        assert max(lattice_steps) <= 0.08, \
            f"global plan looks resampled, not a lattice path (max step {max(lattice_steps):.3f} m)"
        assert min(lattice_steps) >= 0.04, \
            f"global plan points are too dense for a lattice path (min step {min(lattice_steps):.3f} m)"
        assert steps[-1] <= 0.60, \
            f"snap-to-goal segment is unexpectedly long ({steps[-1]:.3f} m)"

        # 醒目样式：线宽来自 node.visualization.global_plan_line_width，必须明显粗于 MINCO 轨迹(0.07)。
        lines = [m for m in global_markers
                 if m.ns == "global_plan" and m.type == Marker.LINE_STRIP and m.action == Marker.ADD]
        assert lines, ("no LINE_STRIP marker on /nav_executor/debug/global_plan; got "
                       + repr([(m.ns, m.type, m.action, len(m.points)) for m in global_markers]))
        line = lines[-1]
        assert len(line.points) == len(plan.poses), \
            f"marker has {len(line.points)} points but the path has {len(plan.poses)}"
        assert line.scale.x >= 0.10, f"global plan line width {line.scale.x} is not prominent"
        assert line.color.a >= 0.99, "global plan line is not fully opaque"
        # 起点应贴着机器人（本烟测里机器人在 odom 原点）。
        assert line.points[0].x < 0.5 and line.points[0].y < 0.5, \
            f"global plan starts far from the robot: ({line.points[0].x}, {line.points[0].y})"

        print(f"goal smoke passed: {len(paths[-1].poses)} trajectory poses, "
              f"{len(plan.poses)} global plan poses, marker width {line.scale.x:.3f} m")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for process in (server, nav):
            process.terminate()
        for process in (server, nav):
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


if __name__ == "__main__":
    main()
