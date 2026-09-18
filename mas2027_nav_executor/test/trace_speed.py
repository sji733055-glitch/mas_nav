#!/usr/bin/env python3
"""速度三路对照记录器（实车排障用）。

同一时刻记录三种速度 + 里程计位置，判断"跟踪不上"断在哪一环：
  plan_peak    /opt_path 全轨迹的峰值速度（规划器打算跑多快）
  plan@robot   /opt_path 里离车最近点的速度（与 PathExecutor::buildReference 同口径）
  cmd          /cmd_vel 线速度（执行器实际发出去多少）
  odom         /Odometry 的 twist（车实际跑多快；规划器的 vel_error 就用它）
  x,y          /Odometry 的位置（用来算真实走过的路程，区分"慢"与"路绕远了"）

QoS 必须与发布端一致，否则静默收不到（上一版就栽在这）：
  /Odometry 由 small_point_lio 用 best_effort 发布
  /opt_path 用 KeepLast(1)（reliable）发布
  /cmd_vel  用 QoS(1)（reliable）发布

【必须先 source 工作区】否则 `interfaces.msg` 导入失败，plan 两列恒为 0 ——
2026-09-17 14:02 实车那次就这么白跑了一趟，只剩 cmd/odom 两列可用。

用法：
  source /home/mas/mas_nav_2027_native/install/setup.bash
  python3 src/mas2027_nav_executor/test/trace_speed.py .scratch/speed_trace.csv 120

判读：plan@robot 是规划器认为"车所在处"的速度，cmd 是 MPC 实际发出的，odom 是车真正跑的。
2026-09-17 14:02 实车那次：cmd 1.14~1.73 m/s 而 odom 0.55~0.91 m/s（约一半），且 odom 与
位姿位移一致 ⇒ 那一轮的"慢"不在规划/MPC，而在底盘跟随或 /cmd_vel 时序，见
docs/executor_velocity_triage_2026-09-17.md §5 的继续排查步骤。
"""
import csv
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

try:
    from interfaces.msg import MpcPositionCommand
    HAVE_TRAJ_MSG = True
except Exception as exc:  # noqa: BLE001
    HAVE_TRAJ_MSG = False
    print(f"!! 无法 import interfaces.msg.MpcPositionCommand：{exc}")
    print("!! 请先 source /home/mas/mas_nav_2027_native/install/setup.bash —— plan 两列会是空的")

out = sys.argv[1] if len(sys.argv) > 1 else ".scratch/speed_trace.csv"
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0

BEST_EFFORT = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
RELIABLE = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)

rclpy.init()
node = rclpy.create_node("speed_trace")
s = {"plan_peak": 0.0, "plan_near": 0.0, "cmd": 0.0, "odom": 0.0, "x": 0.0, "y": 0.0,
     "n_traj": 0, "n_cmd": 0, "n_odom": 0}


def on_traj(msg):
    s["n_traj"] += 1
    if not msg.cmds:
        return
    best, best_d, peak = 0.0, float("inf"), 0.0
    for c in msg.cmds:
        v = math.hypot(c.velocity.x, c.velocity.y)
        peak = max(peak, v)
        d = (c.position.x - s["x"]) ** 2 + (c.position.y - s["y"]) ** 2
        if d < best_d:
            best_d, best = d, v
    s["plan_peak"], s["plan_near"] = peak, best


def on_cmd(msg):
    s["n_cmd"] += 1
    s["cmd"] = math.hypot(msg.linear.x, msg.linear.y)


def on_odom(msg):
    s["n_odom"] += 1
    s["odom"] = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
    s["x"] = msg.pose.pose.position.x
    s["y"] = msg.pose.pose.position.y


if HAVE_TRAJ_MSG:
    node.create_subscription(MpcPositionCommand, "/opt_path", on_traj, RELIABLE)
node.create_subscription(Twist, "/cmd_vel", on_cmd, RELIABLE)
node.create_subscription(Odometry, "/Odometry", on_odom, BEST_EFFORT)

with open(out, "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t", "plan_peak", "plan_at_robot", "cmd", "odom", "x", "y"])
    t0 = time.monotonic()
    print(f"记录到 {out}，{seconds:.0f} s；Ctrl-C 提前结束")
    nxt = t0
    while time.monotonic() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.monotonic() >= nxt:
            nxt += 0.5
            t = time.monotonic() - t0
            w.writerow([f"{t:.2f}", f"{s['plan_peak']:.3f}", f"{s['plan_near']:.3f}",
                        f"{s['cmd']:.3f}", f"{s['odom']:.3f}", f"{s['x']:.3f}", f"{s['y']:.3f}"])
            fh.flush()
            print(f"t={t:5.1f} plan={s['plan_peak']:.2f}(车处 {s['plan_near']:.2f}) "
                  f"cmd={s['cmd']:.2f} odom={s['odom']:.2f} "
                  f"pos=({s['x']:.2f},{s['y']:.2f}) [收到 traj/cmd/odom = "
                  f"{s['n_traj']}/{s['n_cmd']}/{s['n_odom']}]")
node.destroy_node()
rclpy.shutdown()
