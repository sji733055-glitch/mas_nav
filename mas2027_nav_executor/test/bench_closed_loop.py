#!/usr/bin/env python3
"""闭环台架：真实 map_server + 真实 nav_executor + 理想一阶底盘模型。

【为什么固化进仓库】2026-09-17 排查"车慢且卡顿"时，规划侧的三道否决（种子门 / 轨迹校验 /
指令门）与 MPC 输出只有放在同一条时间轴上才能分清是谁在刹车。本台架用真实 map_server +
真实 nav_executor + 真实 lab3 地图跑闭环，把"规划速度 / MPC 输出 / 车实际速度"三路对齐记录，
并把节点 stdout（COLD_START / 停车前缀 / MINCO 失败 / 急停）落到文件，可离线反复复现。

⚠️ 台架结论是**双峰**的：起点净空 0.158 m 时，同一份配置有时 70% 的拍在动、有时只有 11%
（取决于头 1 秒能不能从贴墙状态里挤出来）。判读必须同时看"移动拍占比 / 覆盖位移 / MINCO
失败次数"，同一配置至少跑 2~3 次；2026-09-17 的 rho 100/500 对照就是各跑两次、结论互相矛盾，
因此**没有**据此改配置（见 docs/executor_velocity_triage_2026-09-17.md）。

目的：把"规划速度 → MPC 输出 → 车实际速度"三路在**当前代码**下量出来，
判断"慢"断在规划、MPC、还是底盘。

  * 底盘：v ← v + (cmd - v)·dt/τ（τ 可调，默认 0.10 s），yaw 同理用 ω 积分。
  * 云：lab3.pcd 按当前位姿搬到车体周围（模拟真实在线点云），只保留 8 m 内。
  * TF：map→odom 用实车记录值（可用 --map-to-odom 覆盖），odom→base_link 恒定。
  * 记录：/opt_path（最近点速度 + 峰值）、/cmd_vel（含到达间隔）、/Odometry，
    并把 nav_executor 的 stdout（MincoPlanner 的 COLD_START / 急停等）落到文件。

用法（必须先 source 工作区，否则 interfaces.msg 导入失败）：
  source install/setup.bash
  ROS_LOG_DIR=$PWD/.scratch/roslog python3 src/mas2027_nav_executor/test/bench_closed_loop.py \
      src/mas2027_nav_bringup/map/lab3_terrain.msgpack \
      src/mas2027_nav_executor/config \
      /home/mas/mas_nav_2027/mas2027_nav_bringup/pcd/lab3.pcd \
      --map-yaml src/mas2027_nav_bringup/map/lab3.yaml \
      --goal 5.55,3.52 --seconds 40
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
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster

try:
    from interfaces.msg import MpcPositionCommand
except Exception as exc:  # noqa: BLE001
    raise SystemExit(f"需要先 source install/setup.bash：{exc}")

DEFAULT_MAP_TO_ODOM = "-0.106,0.635,0.256,0.0171,-0.0026,0.0257,0.9995"


def lzf_decompress(data, expected):
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


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("terrain_map")
    parser.add_argument("executor_config_dir")
    parser.add_argument("prior_pcd")
    parser.add_argument(
        "--map-yaml",
        help="matching Nav2 map YAML (defaults to <name>.yaml beside <name>_terrain.msgpack)",
    )
    parser.add_argument("--goal", default="5.55,3.52", help="goal in odom: x,y")
    parser.add_argument("--goals", default="",
                        help="多航段：\"x1,y1;x2,y2;...\"，到达当前目标（<0.6 m）后自动发下一个")
    parser.add_argument("--start", default="0.0,0.0", help="start pose in odom: x,y")
    parser.add_argument("--map-to-odom", default=DEFAULT_MAP_TO_ODOM)
    parser.add_argument("--seconds", type=float, default=40.0)
    parser.add_argument("--tau", type=float, default=0.10, help="chassis first-order time constant")
    parser.add_argument("--cloud-hz", type=float, default=8.0)
    parser.add_argument("--cloud-radius", type=float, default=8.0)
    parser.add_argument("--cloud-z-min", type=float, default=-10.0,
                        help="只发布 odom z 大于该值的点（用于排除地面点）")
    parser.add_argument("--cloud-z-max", type=float, default=10.0)
    parser.add_argument("--cloud-blind", type=float, default=0.35,
                        help="模拟 ray_range 下限：车体水平距离以内的点不发布")
    parser.add_argument("--out", default=".scratch/closed_loop_trace.csv")
    parser.add_argument("--stdout", default=".scratch/closed_loop_node_stdout.log")
    parser.add_argument("--domain", default="241")
    args = parser.parse_args()
    map_yaml = args.map_yaml
    if map_yaml is None:
        suffix = "_terrain.msgpack"
        if not args.terrain_map.endswith(suffix):
            raise SystemExit("--map-yaml is required when terrain map lacks _terrain.msgpack suffix")
        map_yaml = args.terrain_map[:-len(suffix)] + ".yaml"

    gx, gy = (float(v) for v in args.goal.split(","))
    mission = []
    if args.goals:
        for item in args.goals.split(";"):
            if not item.strip():
                continue
            mx, my = (float(v) for v in item.split(","))
            mission.append((mx, my))
        if mission:
            gx, gy = mission[0]
    mission_index = 0
    mission_reached = []
    sx, sy = (float(v) for v in args.start.split(","))
    map_to_odom = [float(v) for v in args.map_to_odom.split(",")]
    os.environ["ROS_DOMAIN_ID"] = args.domain

    point_count, columns = load_pcd_xyz(args.prior_pcd)
    rot = quat_to_matrix(map_to_odom[3:])
    world = []
    for i in range(0, point_count, 2):
        px = columns["x"][i] - map_to_odom[0]
        py = columns["y"][i] - map_to_odom[1]
        pz = columns["z"][i] - map_to_odom[2]
        world.append((rot[0][0] * px + rot[1][0] * py + rot[2][0] * pz,
                      rot[0][1] * px + rot[1][1] * py + rot[2][1] * pz,
                      rot[0][2] * px + rot[1][2] * py + rot[2][2] * pz))
    print(f"[probe] replay cloud: {len(world)} points (half of {point_count})")

    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = "/lib/x86_64-linux-gnu:" + env.get("LD_LIBRARY_PATH", "")
    map_exe = os.path.join(get_package_prefix("map_server"), "lib/map_server/map_server_node")
    nav_exe = os.path.join(get_package_prefix("mas2027_nav_executor"),
                           "lib/mas2027_nav_executor/mas2027_nav_executor_node")
    config_dir = os.path.abspath(args.executor_config_dir)
    server = subprocess.Popen([
        map_exe, "--ros-args", "-p", f"terrain_map_path:={args.terrain_map}",
        "-p", f"map_yaml_path:={map_yaml}",
    ], env=env)
    stdout_file = open(args.stdout, "w")
    nav = subprocess.Popen([
        nav_exe, "--ros-args",
        *[item for name in ("mpc_params.yaml", "node_params.yaml", "planner_params.yaml")
          for item in ("--params-file", os.path.join(config_dir, name))],
    ], env=env, stdout=stdout_file, stderr=subprocess.STDOUT)

    rclpy.init()
    node = rclpy.create_node("closed_loop_probe")
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

    state = {
        "cmd": [0.0, 0.0, 0.0],      # vx, vy, omega (body)
        "cmd_stamps": [],
        "traj": None,
        "last_traj_at": 0.0,
        "terrain_maps": 0,
    }
    rows = []

    def on_cmd(msg):
        state["cmd"] = [msg.linear.x, msg.linear.y, msg.angular.z]
        state["cmd_stamps"].append(time.monotonic())

    def on_traj(msg):
        state["traj"] = msg
        state["last_traj_at"] = time.monotonic()

    node.create_subscription(Twist, "/cmd_vel", on_cmd, 10)
    node.create_subscription(MpcPositionCommand, "/opt_path", on_traj,
                             QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                        durability=DurabilityPolicy.VOLATILE))
    from nav_msgs.msg import OccupancyGrid
    node.create_subscription(OccupancyGrid, "/cost_map",
                             lambda m: state.__setitem__(
                                 "terrain_maps", state["terrain_maps"] + 1),
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                        durability=DurabilityPolicy.TRANSIENT_LOCAL))

    pose = [sx, sy, 0.0]
    vel = [0.0, 0.0, 0.0]
    goal_sent_at = None
    map_ready_at = None
    last_cloud = 0.0
    t0 = time.monotonic()
    dt = 0.02
    obs = []
    try:
        while time.monotonic() - t0 < args.seconds:
            now = time.monotonic()
            tau = max(1e-3, args.tau)
            for i in range(3):
                vel[i] += (state["cmd"][i] - vel[i]) * (dt / tau)
            yaw = pose[2]
            c, s = math.cos(yaw), math.sin(yaw)
            pose[0] += (c * vel[0] - s * vel[1]) * dt
            pose[1] += (s * vel[0] + c * vel[1]) * dt
            pose[2] += vel[2] * dt

            stamp = node.get_clock().now().to_msg()
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = pose[0]
            odom.pose.pose.position.y = pose[1]
            q = yaw_to_quat(pose[2])
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]
            odom.twist.twist.linear.x = vel[0]
            odom.twist.twist.linear.y = vel[1]
            odom.twist.twist.angular.z = vel[2]
            odom_pub.publish(odom)

            if state["terrain_maps"]:
                if map_ready_at is None:
                    map_ready_at = now
                if (goal_sent_at is not None and mission_index + 1 < len(mission) and
                        math.hypot(gx - pose[0], gy - pose[1]) < 0.6):
                    mission_reached.append((mission_index, now - t0))
                    mission_index += 1
                    gx, gy = mission[mission_index]
                    goal_sent_at = None
                    state["traj"] = None
                if goal_sent_at is None and now - map_ready_at > 1.5:
                    goal = PoseStamped()
                    goal.header.stamp = stamp
                    goal.header.frame_id = "odom"
                    goal.pose.position.x = gx
                    goal.pose.position.y = gy
                    goal.pose.orientation.w = 1.0
                    goal_pub.publish(goal)
                    goal_sent_at = now
                    print(f"[probe] goal sent at t={now - t0:.2f}s")

            if now - last_cloud >= 1.0 / args.cloud_hz:
                last_cloud = now
                pts = []
                for wx, wy, wz in world:
                    dx = wx - pose[0]
                    dy = wy - pose[1]
                    r2 = dx * dx + dy * dy
                    if r2 > args.cloud_radius ** 2:
                        continue
                    # 传感器自身盲区：真实链路 ray_range 下限 0.3 m 会丢掉车体附近的点，不模拟则
                    # 机器人脚下/头顶的点被投影成本格障碍（ROGMap is not clear at the robot pose），目标被拒。
                    if r2 < args.cloud_blind ** 2:
                        continue
                    if wz < args.cloud_z_min or wz > args.cloud_z_max:
                        continue
                    # world→body(world axes shifted; robot yaw applied)
                    pts.append((c * dx + s * dy, -s * dx + c * dy, wz))
                cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                    Header(stamp=stamp, frame_id="odom"), pts))
                if len(rows) == 0 or (now - rows[-1][0]) > 0.2:
                    pass

            # 0.05 s 采样记录
            if not obs or now - obs[-1]["t"] >= 0.05:
                traj = state["traj"]
                plan_peak = plan_near = 0.0
                if traj is not None and traj.cmds:
                    best = None
                    for cmd in traj.cmds:
                        v = math.hypot(cmd.velocity.x, cmd.velocity.y)
                        plan_peak = max(plan_peak, v)
                        d = (cmd.position.x - pose[0]) ** 2 + (cmd.position.y - pose[1]) ** 2
                        if best is None or d < best[0]:
                            best = (d, v)
                    plan_near = best[1]
                obs.append({
                    "t": now - t0, "plan_peak": plan_peak, "plan_near": plan_near,
                    "cmd": math.hypot(state["cmd"][0], state["cmd"][1]),
                    "cmd_w": state["cmd"][2],
                    "v": math.hypot(vel[0], vel[1]), "x": pose[0], "y": pose[1],
                    "goal_dist": math.hypot(gx - pose[0], gy - pose[1]),
                })

            rclpy.spin_once(node, timeout_sec=0.0)
            if server.poll() is not None or nav.poll() is not None:
                print(f"[probe] node exited: map_server={server.poll()} nav={nav.poll()}")
                break
            time.sleep(max(0.0, dt - (time.monotonic() - now)))
    finally:
        cmd_stamps = state["cmd_stamps"]
        try:
            server.terminate()
            nav.terminate()
            server.wait(timeout=5)
            nav.wait(timeout=5)
        except Exception:  # noqa: BLE001
            pass
        stdout_file.close()
        node.destroy_node()
        rclpy.shutdown()

    with open(args.out, "w") as fh:
        fh.write("t,plan_peak,plan_near,cmd,cmd_w,odom,x,y,goal_dist\n")
        for o in obs:
            fh.write(f"{o['t']:.2f},{o['plan_peak']:.3f},{o['plan_near']:.3f},"
                     f"{o['cmd']:.3f},{o['cmd_w']:.3f},{o['v']:.3f},"
                     f"{o['x']:.3f},{o['y']:.3f},{o['goal_dist']:.3f}\n")

    if not obs:
        print("[probe] no samples")
        return
    moved = obs[-1]["x"] - obs[0]["x"], obs[-1]["y"] - obs[0]["y"]
    dist = math.hypot(*moved)
    t_end = obs[-1]["t"]
    peaks = max(o["plan_peak"] for o in obs)
    vmax = max(o["v"] for o in obs)
    cmds = [o["cmd"] for o in obs]
    print(f"\n[probe] 采样 {len(obs)} 点 / {t_end:.1f}s")
    print(f"[probe] 轨迹峰值速度 {peaks:.2f} m/s | 实际峰值 {vmax:.2f} m/s "
          f"| 平均实际 {dist / max(t_end, 1e-3):.2f} m/s | 直线位移 {dist:.2f} m")
    print(f"[probe] cmd: 均值 {sum(cmds) / len(cmds):.2f}, 中位 {sorted(cmds)[len(cmds) // 2]:.2f}, "
          f"max {max(cmds):.2f}, 占比>0.05: "
          f"{sum(1 for c in cmds if c > 0.05) / len(cmds):.0%}")
    if len(cmd_stamps) > 2:
        gaps = [b - a for a, b in zip(cmd_stamps, cmd_stamps[1:])]
        gaps_sorted = sorted(gaps)
        print(f"[probe] /cmd_vel {len(cmd_stamps)} 条, 间隔 中位 "
              f"{gaps_sorted[len(gaps_sorted) // 2] * 1000:.1f} ms, p90 "
              f"{gaps_sorted[int(len(gaps_sorted) * 0.9)] * 1000:.1f} ms, max "
              f"{max(gaps) * 1000:.1f} ms")
    print(f"[probe] 轨迹消息 {sum(1 for o in obs if o['plan_peak'] > 0)}/{len(obs)} 拍有规划")
    if mission_reached:
        print("[probe] 各航段到达时刻: " +
              ", ".join(f"#{i + 1}@{t:.1f}s" for i, t in mission_reached))
    print(f"[probe] 记录: {args.out} / 节点 stdout: {args.stdout}")


if __name__ == "__main__":
    main()
