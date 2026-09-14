#!/usr/bin/env python3
"""Check map_server -> goal -> trajectory in an isolated ROS_DOMAIN_ID."""
import argparse
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
from tf2_ros import StaticTransformBroadcaster


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("terrain_map")
    parser.add_argument("executor_config_dir")
    parser.add_argument("static_cloud")
    parser.add_argument("--preempt", action="store_true",
                        help="Send a newer goal and verify that its path replaces the first")
    args = parser.parse_args()
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = "/lib/x86_64-linux-gnu:" + env.get("LD_LIBRARY_PATH", "")
    map_exe = os.path.join(get_package_prefix("map_server"), "lib/map_server/map_server_node")
    nav_exe = os.path.join(get_package_prefix("mas2027_nav_executor"),
                           "lib/mas2027_nav_executor/mas2027_nav_executor_node")
    server = subprocess.Popen([
        map_exe, "--ros-args", "-p", f"terrain_map_path:={args.terrain_map}",
        "-p", "origin_x:=-4.6", "-p", "origin_y:=-7.94",
        "-p", f"global_cloud_path:={args.static_cloud}",
    ], env=env)
    nav = subprocess.Popen([
        nav_exe, "--ros-args",
        *[item for name in ("mpc_params.yaml", "node_params.yaml", "planner_params.yaml")
          for item in ("--params-file", os.path.join(args.executor_config_dir, name))],
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
    maps = []
    paths = []
    node.create_subscription(OccupancyGrid, "/dynamic_cost_map", maps.append, qos)
    node.create_subscription(Path, "/nav_executor/global_path", paths.append, qos)
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
            if maps and first_map_at is None:
                first_map_at = time.monotonic()
            if first_map_at is not None and time.monotonic() - first_map_at >= 2.0 and not goal_sent:
                goal = PoseStamped()
                goal.header.stamp = node.get_clock().now().to_msg()
                goal.header.frame_id = "map"
                # Reproduce the free-space goal recorded in the user's RViz log.
                goal.pose.position.x = 2.56226
                goal.pose.position.y = 0.437201
                goal.pose.orientation.w = 1.0
                goal_pub.publish(goal)
                goal_sent = True
            if paths and goal_sent:
                if not args.preempt:
                    break
                if not second_sent:
                    first_end_x = paths[-1].poses[-1].pose.position.x
                    goal = PoseStamped()
                    goal.header.stamp = node.get_clock().now().to_msg()
                    goal.header.frame_id = "map"
                    goal.pose.position.x = 3.0
                    goal.pose.position.y = 0.437201
                    goal.pose.orientation.w = 1.0
                    goal_pub.publish(goal)
                    second_sent = True
                elif paths[-1].poses[-1].pose.position.x > first_end_x + 0.25:
                    break
        assert goal_sent, "dynamic map was not ready"
        assert paths and len(paths[-1].poses) >= 2, "goal produced no path"
        if args.preempt:
            assert second_sent and paths[-1].poses[-1].pose.position.x > first_end_x + 0.25, \
                "newer goal did not replace the active path"
        print(f"goal smoke passed: {len(paths[-1].poses)} trajectory poses")
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
