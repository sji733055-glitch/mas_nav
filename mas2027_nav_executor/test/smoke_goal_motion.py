#!/usr/bin/env python3
"""Check that a goal actually produces a sustained motion command (not just a published trajectory).

Regression for the field symptom "发目标点后车不动，优化出的轨迹闪烁，cmd_vel 只有一段时间有数值":
`/opt_path` briefly carried a trajectory but the runtime safety monitor rejected it a few ms later
(publish gate 0.30 m vs monitor 0.50 m), so the executor braked and replanned at ~10 Hz and
/cmd_vel stayed 0 apart from a handful of ticks. A second failure mode was the robot being parked
closer to a wall than `collision_dist`: every trajectory then failed the clearance check at the
robot's own start pose and nothing was ever published.

The scene is replayed from the prior PCD (lab3.pcd) fed as the live registered cloud in odom, so the
geometry matches the real bench setup. `--near-wall` additionally pushes the wall next to the robot
closer, which is the "parked against the wall" case.

Usage:
  ROS_DOMAIN_ID=229 python3 test/smoke_goal_motion.py \
      <lab3_terrain.msgpack> <executor_config_dir> <lab3.pcd> \
      [--map-yaml lab3.yaml] [--near-wall 0.16] [--goal -0.15,-0.70]
"""
import argparse
import math
import os
import struct
import subprocess
import sys
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from interfaces.msg import MpcPositionCommand
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster

# map->odom recorded by odom_localizer in the 2026-09-15 15:52 bench run. Override with
# --map-to-odom when replaying another session.
DEFAULT_MAP_TO_ODOM = "-0.106,0.635,0.256,0.0171,-0.0026,0.0257,0.9995"


def lzf_decompress(data, expected):
    """Minimal liblzf decoder for PCL `DATA binary_compressed` clouds."""
    out = bytearray()
    ip = 0
    while ip < len(data):
        ctrl = data[ip]
        ip += 1
        if ctrl < 32:
            length = ctrl + 1
            out += data[ip:ip + length]
            ip += length
        else:
            length = ctrl >> 5
            if length == 7:
                length += data[ip]
                ip += 1
            ref = len(out) - ((ctrl & 0x1F) << 8) - data[ip] - 1
            ip += 1
            for i in range(length + 2):
                out.append(out[ref + i])
    if len(out) != expected:
        raise RuntimeError(f"lzf size mismatch {len(out)} != {expected}")
    return bytes(out)


def load_pcd_xyz(path):
    raw = open(path, "rb").read()
    end = raw.index(b"\n", raw.index(b"DATA binary_compressed")) + 1
    header = raw[:end].decode("ascii", "replace")
    fields, sizes, types, points = [], [], [], 0
    for line in header.splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "FIELDS":
            fields = parts[1:]
        elif parts[0] == "SIZE":
            sizes = [int(v) for v in parts[1:]]
        elif parts[0] == "TYPE":
            types = parts[1:]
        elif parts[0] == "POINTS":
            points = int(parts[1])
    body = raw[end:]
    compressed, uncompressed = struct.unpack("<II", body[:8])
    data = lzf_decompress(body[8:8 + compressed], uncompressed)
    columns, offset = {}, 0
    for name, size, ptype in zip(fields, sizes, types):
        length = size * points
        chunk = data[offset:offset + length]
        offset += length
        if ptype == "F" and size == 4:
            columns[name] = struct.unpack(f"<{points}f", chunk)
    return points, columns


def quat_to_matrix(q):
    x, y, z, w = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def planar_speed(twist):
    return math.hypot(twist.linear.x, twist.linear.y)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("terrain_map")
    parser.add_argument("executor_config_dir")
    parser.add_argument("prior_pcd")
    parser.add_argument(
        "--map-yaml",
        help="matching Nav2 map YAML (defaults to <name>.yaml beside <name>_terrain.msgpack)",
    )
    parser.add_argument("--goal", default="-0.15,-0.70", help="goal in odom, x,y")
    parser.add_argument("--map-to-odom", default=DEFAULT_MAP_TO_ODOM,
                        help="tx,ty,tz,qx,qy,qz,qw")
    parser.add_argument("--near-wall", type=float, default=None,
                        help="push the wall beside the robot (|x|<0.6) to this odom y")
    parser.add_argument("--seconds", type=float, default=15.0, help="observation window")
    parser.add_argument("--cloud-hz", type=float, default=5.0)
    args = parser.parse_args()
    map_yaml = args.map_yaml
    if map_yaml is None:
        suffix = "_terrain.msgpack"
        if not args.terrain_map.endswith(suffix):
            raise SystemExit("--map-yaml is required when terrain map lacks _terrain.msgpack suffix")
        map_yaml = args.terrain_map[:-len(suffix)] + ".yaml"
    gx, gy = (float(v) for v in args.goal.split(","))
    map_to_odom = [float(v) for v in args.map_to_odom.split(",")]
    if len(map_to_odom) != 7:
        raise SystemExit("--map-to-odom needs 7 comma separated values")

    point_count, columns = load_pcd_xyz(args.prior_pcd)
    rot = quat_to_matrix(map_to_odom[3:])
    cloud = []
    for i in range(0, point_count, 2):
        px = columns["x"][i] - map_to_odom[0]
        py = columns["y"][i] - map_to_odom[1]
        pz = columns["z"][i] - map_to_odom[2]
        ox = rot[0][0] * px + rot[1][0] * py + rot[2][0] * pz
        oy = rot[0][1] * px + rot[1][1] * py + rot[2][1] * pz
        oz = rot[0][2] * px + rot[1][2] * py + rot[2][2] * pz
        if args.near_wall is not None and abs(ox) < 0.6 and 0.05 < oy < 0.95:
            oy = args.near_wall
        cloud.append((ox, oy, oz))
    print(f"replay cloud: {len(cloud)} points, near wall: {args.near_wall}")

    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = "/lib/x86_64-linux-gnu:" + env.get("LD_LIBRARY_PATH", "")
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
          for item in ("--params-file", os.path.join(args.executor_config_dir, name))],
    ], env=env)

    rclpy.init()
    node = rclpy.create_node("goal_motion_smoke")
    broadcaster = StaticTransformBroadcaster(node)
    tf = TransformStamped()
    tf.header.stamp = node.get_clock().now().to_msg()
    tf.header.frame_id = "map"
    tf.child_frame_id = "odom"
    tf.transform.translation.x = map_to_odom[0]
    tf.transform.translation.y = map_to_odom[1]
    tf.transform.translation.z = map_to_odom[2]
    tf.transform.rotation.x = map_to_odom[3]
    tf.transform.rotation.y = map_to_odom[4]
    tf.transform.rotation.z = map_to_odom[5]
    tf.transform.rotation.w = map_to_odom[6]
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
    cmds, trajs, terrain_maps = [], [], []
    node.create_subscription(Twist, "/cmd_vel", cmds.append, 10)
    node.create_subscription(MpcPositionCommand, "/opt_path", trajs.append, 10)
    node.create_subscription(OccupancyGrid, "/cost_map", terrain_maps.append, qos)

    goal_sent = False
    goal_sent_at = None
    map_ready_at = None
    last_cloud = 0.0
    try:
        deadline = time.monotonic() + 90.0
        while time.monotonic() < deadline:
            stamp = node.get_clock().now().to_msg()
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)
            if time.monotonic() - last_cloud >= 1.0 / args.cloud_hz:
                last_cloud = time.monotonic()
                cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                    Header(stamp=stamp, frame_id="odom"), cloud))
            rclpy.spin_once(node, timeout_sec=0.02)
            if server.poll() is not None or nav.poll() is not None:
                raise RuntimeError(
                    f"node exited: map_server={server.poll()} nav_executor={nav.poll()}")
            if terrain_maps and map_ready_at is None:
                map_ready_at = time.monotonic()
            if not goal_sent and map_ready_at and time.monotonic() - map_ready_at > 6.0:
                goal = PoseStamped()
                goal.header.stamp = stamp
                goal.header.frame_id = "odom"
                goal.pose.position.x = gx
                goal.pose.position.y = gy
                goal.pose.orientation.w = 1.0
                goal_pub.publish(goal)
                goal_sent = True
                goal_sent_at = time.monotonic()
                cmds.clear()
                trajs.clear()
            if goal_sent and time.monotonic() - goal_sent_at >= args.seconds:
                break

        moving = [c for c in cmds if planar_speed(c) > 0.01]
        max_speed = max((planar_speed(c) for c in cmds), default=0.0)
        stamps = {t.header.stamp.sec * 10**9 + t.header.stamp.nanosec for t in trajs}
        print(f"cmd_vel: {len(cmds)} msgs, moving {len(moving)}, max speed {max_speed:.3f} m/s")
        print(f"/opt_path: {len(trajs)} msgs, {len(stamps)} distinct trajectories")
        assert goal_sent, "static terrain map never became ready"
        assert stamps, "goal produced no trajectory on /opt_path"
        assert len(moving) > 0.5 * len(cmds), (
            f"only {len(moving)}/{len(cmds)} cmd_vel messages command motion: the planner is "
            "publishing a trajectory but the executor keeps braking")
        assert max_speed > 0.1, f"max planar speed {max_speed:.3f} m/s, robot would not move"
        print("goal motion smoke passed")
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
    sys.exit(main())
