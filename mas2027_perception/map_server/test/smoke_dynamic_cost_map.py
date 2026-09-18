#!/usr/bin/env python3
"""Smoke-check the current-frame dynamic cost map without robot hardware."""
import argparse
import os
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("terrain_map")
    parser.add_argument("static_cloud")
    args = parser.parse_args()
    executable = os.path.join(get_package_prefix("map_server"), "lib", "map_server", "map_server_node")
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = "/lib/x86_64-linux-gnu:" + env.get("LD_LIBRARY_PATH", "")
    process = subprocess.Popen([
        executable, "--ros-args",
        "-p", f"terrain_map_path:={args.terrain_map}",
        "-p", f"global_cloud_path:={args.static_cloud}",
        "-p", "origin_x:=-4.6", "-p", "origin_y:=-7.94",
        "-p", "local_map.min_points_per_cell:=1",
        "-p", "local_map.min_cluster_cells:=1",
        "-p", "local_map.dropout_hold_seconds:=0.8",
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)
    rclpy.init()
    node = rclpy.create_node("dynamic_cost_map_smoke")
    broadcaster = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.header.frame_id = "map"
    transform.child_frame_id = "base_link"
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)
    publisher = node.create_publisher(point_cloud2.PointCloud2, "/cloud_registered", 10)
    received = []
    node.create_subscription(OccupancyGrid, "/dynamic_cost_map", received.append, 10)
    try:
        ground = [(1.0 + 0.07 * x, 1.0 + 0.07 * y, -0.25)
                  for x in range(11) for y in range(11)]
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline and not received:
            header = Header(stamp=node.get_clock().now().to_msg(), frame_id="map")
            publisher.publish(point_cloud2.create_cloud_xyz32(header, ground))
            rclpy.spin_once(node, timeout_sec=0.1)
            if process.poll() is not None:
                raise RuntimeError(f"map_server exited with code {process.returncode}")
        assert received, "no dynamic cost map received"
        assert all(value == 0 for value in received[-1].data), "ground became an obstacle"
        received.clear()
        # 11 cm above the floor: the old 0.2 m PCD match erased these points.
        obstacle = [(1.0 + 0.05 * x, 1.0 + 0.05 * y, -0.14)
                    for x in range(6) for y in range(6)]
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline and not any(
                any(value > 0 for value in grid.data) for grid in received):
            header = Header(stamp=node.get_clock().now().to_msg(), frame_id="map")
            publisher.publish(point_cloud2.create_cloud_xyz32(header, ground + obstacle))
            rclpy.spin_once(node, timeout_sec=0.1)
            if process.poll() is not None:
                raise RuntimeError(f"map_server exited with code {process.returncode}")
        assert received, "no obstacle cost map received"
        grid = next(grid for grid in reversed(received) if any(value > 0 for value in grid.data))
        assert len(grid.data) == 770 * 347
        def send_and_wait(points):
            stamp = node.get_clock().now().to_msg()
            header = Header(stamp=stamp, frame_id="map")
            received.clear()
            publisher.publish(point_cloud2.create_cloud_xyz32(header, points))
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                for candidate in received:
                    if candidate.header.stamp == stamp:
                        return candidate
            raise AssertionError("no dynamic map for test cloud")

        # The three-frame cloud queue is empty of obstacles after four ground frames.
        for _ in range(4):
            held = send_and_wait(ground)
        assert any(value > 0 for value in held.data), "one-frame dropout removed the obstacle"
        time.sleep(0.9)
        cleared = send_and_wait(ground)
        assert all(value == 0 for value in cleared.data), "expired obstacle was not cleared"
        print(f"dynamic_cost_map smoke passed: occupied={sum(value > 0 for value in grid.data)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


if __name__ == "__main__":
    main()
