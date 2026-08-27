#!/usr/bin/env python3
# Copyright 2026 mas2027
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

"""
量雷达的安装倾角和 gravity 参数，顺手把要改的那几行直接算好打印出来。

原理：静止时加速度计读的是"向上"的比力，取反就是重力向量，也就是 small_point_lio
的 gravity 参数。URDF 里 lidar_joint 的旋转是 Rz(90°)·Rx(r)，所以重力在雷达系里
只有 [0, -n·sin r, -n·cos r] 两个分量，倾角 r = atan2(a_y, a_z)。

用法：车停在水平地面上、完全静止，起好驱动（跑整个 bringup 或者只跑 mid360_driver），
然后

    python3 src/mas2027_nav_bringup/scripts/measure_lidar_mount.py

只想看某个角度对应的参数、不连雷达：

    python3 src/mas2027_nav_bringup/scripts/measure_lidar_mount.py --tilt 30

两个东西重力量不出来：
- 车体自身的俯仰会整个叠加到 r 上，重力分不开这两者。地面不平就先量车体俯仰，
  用 --chassis-pitch 减掉。
- rpy 里的 yaw(1.5708) 绕重力方向转，重力完全测不到，只能靠机械装配保证，
  或者对着已知墙面用点云标。
"""

import argparse
import math
import os
import re
import time

# URDF lidar_joint 的平移，base_link -> lidar_link，单位米
JOINT_XYZ = (0.09, 0.03, 0.11)
# 雷达原点离地高度：base_link 离地 0.10（base_link_to_base_footprint）+ 雷达高出 base_link 0.11
LIDAR_HEIGHT = 0.21
# Mid360 垂直视场，厂家标称值，仓库里没有东西能验证它
FOV_MIN_DEG, FOV_MAX_DEG = -7.0, 52.0
# 现在配置里 gravity 的模长（单位 g，IMU 的标度误差就体现在这 0.4% 上）
CURRENT_NORM = 0.995636
URDF_REL_PATH = "mas2027_robot_description/urdf/mas2027_sentry.urdf"


def blind_center(tilt_rad):
    """base_link 原点在 lidar_link 中的坐标 = -Rᵀ·t，R = Rz(90°)·Rx(r)。"""
    tx, ty, tz = JOINT_XYZ
    c, s = math.cos(tilt_rad), math.sin(tilt_rad)
    return (-ty, tx * c - tz * s, -(tx * s + tz * c))


def lidar_elevation(tilt_rad, azimuth_deg, rho, dz):
    """水平距离 rho、相对雷达高度 dz、方位角 azimuth_deg 的点，在雷达自身系里的仰角（度）。"""
    x = rho * math.cos(math.radians(azimuth_deg))
    y = rho * math.sin(math.radians(azimuth_deg))
    norm = math.sqrt(x * x + y * y + dz * dz)
    if norm < 1e-9:
        return 0.0
    # 雷达光轴 z_lidar 在 base_link 里是 (sin r, 0, cos r)
    axis_component = math.sin(tilt_rad) * x + math.cos(tilt_rad) * dz
    return math.degrees(math.asin(max(-1.0, min(1.0, axis_component / norm))))


def first_visible(tilt_rad, azimuth_deg, dz, limit=10.0):
    """沿某方位角，高度 dz 的水平面上第一个落进视场的水平距离；整条线都看不到返回 None。"""
    rho = 0.01
    while rho <= limit:
        if FOV_MIN_DEG <= lidar_elevation(tilt_rad, azimuth_deg, rho, dz) <= FOV_MAX_DEG:
            return rho
        rho += 0.005
    return None


def urdf_tilt_deg():
    """从 URDF 里读 lidar_joint 的 roll（度），读不到返回 None。"""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", URDF_REL_PATH)
    try:
        with open(path, encoding="utf-8") as handle:
            text = handle.read()
    except OSError:
        return None
    match = re.search(r'lidar_joint.*?rpy="\s*([-\d.eE]+)', text, re.S)
    return math.degrees(float(match.group(1))) if match else None


def report(tilt_deg, gravity):
    """打印安装姿态、要改的几行、以及这个角度对应的视场覆盖。"""
    tilt = math.radians(tilt_deg)
    bx, by, bz = blind_center(tilt)
    blind_norm = math.sqrt(bx * bx + by * by + bz * bz)
    urdf_deg = urdf_tilt_deg()

    print("\n安装姿态")
    print("  倾角 r           %.2f°  (%.4f rad)" % (tilt_deg, tilt))
    if urdf_deg is not None:
        mark = "一致" if abs(urdf_deg - tilt_deg) < 1.0 else "不一致，URDF 和实物有一个没跟上"
        print("  URDF 现在        %.2f°  差 %.2f° → %s" % (urdf_deg, tilt_deg - urdf_deg, mark))

    print("\n要改的三处")
    print("  %s" % URDF_REL_PATH)
    print('    <origin xyz="%.2f %.2f %.2f" rpy="%.4f 0 1.5708"/>' % (*JOINT_XYZ, tilt))
    print("  mas2027_nav_bringup/config/small_point_lio_params.yaml")
    print("    blind_center: [ %.3f, %.3f, %.4f ]" % (bx, by, bz))
    print("    gravity: [ %.6f, %.6f, %.6f ]" % gravity)
    check = "对" if abs(blind_norm - 0.1453) < 5e-4 else "不对，JOINT_XYZ 和 URDF 不一致了"
    print("    # |blind_center| = %.4f，跟倾角无关，恒等于 0.1453 → %s" % (blind_norm, check))

    print("\n这个角度的覆盖（雷达离地 %.2f m，视场 %+.0f°…%+.0f°）" % (LIDAR_HEIGHT, FOV_MIN_DEG, FOV_MAX_DEG))
    for label, azimuth in (("正前", 0.0), ("侧前 45°", 45.0), ("正侧 90°", 90.0), ("正后", 180.0)):
        ground = first_visible(tilt, azimuth, -LIDAR_HEIGHT)
        blind = first_visible(tilt, azimuth, 1.0)
        ground_txt = "%.2f m 起" % ground if ground else "看不到"
        blind_txt = "%.2f m 内是盲区" % blind if blind else "全是盲区"
        print("  %-9s 地面 %-10s 头顶 1 m 高处 %s" % (label, ground_txt, blind_txt))
    print("  正前方仰角上限 %.1f°（%.0f° 视场上界减掉倾角），高于它的东西正面看不见"
          % (FOV_MAX_DEG - tilt_deg, FOV_MAX_DEG))


def collect(topic, samples, timeout):
    """收 samples 帧 IMU 的 linear_acceleration，单位跟驱动一致（g）。"""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu

    rclpy.init()
    node = rclpy.create_node("measure_lidar_mount")
    accel = []
    # best_effort 订阅能同时匹配 reliable 和 best_effort 的发布者
    node.create_subscription(
        Imu, topic,
        lambda msg: accel.append((msg.linear_acceleration.x,
                                  msg.linear_acceleration.y,
                                  msg.linear_acceleration.z)),
        qos_profile_sensor_data)
    node.get_logger().info("等 %s 的 %d 帧，保持静止……" % (topic, samples))
    deadline = time.time() + timeout
    while rclpy.ok() and len(accel) < samples and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    return accel


def stats(accel):
    """返回三轴均值和三轴单帧标准差。"""
    n = len(accel)
    mean = tuple(sum(sample[i] for sample in accel) / n for i in range(3))
    std = tuple(math.sqrt(sum((sample[i] - mean[i]) ** 2 for sample in accel) / n) for i in range(3))
    return mean, std


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", default="/mid360_driver/imu", help="IMU 话题")
    parser.add_argument("--samples", type=int, default=400, help="采样帧数，200 Hz 下 400 帧约 2 秒")
    parser.add_argument("--timeout", type=float, default=30.0, help="等不到数据就放弃的秒数")
    parser.add_argument("--tilt", type=float, help="跳过测量，直接算这个倾角（度）对应的参数")
    parser.add_argument("--chassis-pitch", type=float, default=0.0,
                        help="车体自身的俯仰（度），车头朝上为正，会从实测倾角里减掉")
    args = parser.parse_args()

    if args.tilt is not None:
        tilt_deg = args.tilt
        tilt = math.radians(tilt_deg)
        report(tilt_deg, (0.0, -CURRENT_NORM * math.sin(tilt), -CURRENT_NORM * math.cos(tilt)))
        print("\n（--tilt 模式：gravity 按理想值给出，模长沿用现在配置的 %.4f g；"
          "实车还是要静止测一遍，把装配残差量进去。）" % CURRENT_NORM)
        return 0

    accel = collect(args.topic, args.samples, args.timeout)
    if len(accel) < 20:
        print("只收到 %d 帧 %s，驱动没起或者话题名不对" % (len(accel), args.topic))
        return 1
    mean, std = stats(accel)
    norm = math.sqrt(sum(value * value for value in mean))
    # 静止时加速度计读向上的比力，取反就是重力向量，也就是 gravity 参数
    gravity = tuple(-value for value in mean)
    measured_deg = math.degrees(math.atan2(mean[1], mean[2]))
    # a_x 只跟车体侧倾有关：a_x(雷达系) 恒等于 a_y(base_link)
    side_deg = math.degrees(math.asin(max(-1.0, min(1.0, mean[0] / norm))))

    print("\n实测（%d 帧，单位 g）" % len(accel))
    print("  加速度均值       x=%+.6f  y=%+.6f  z=%+.6f" % mean)
    print("  单帧标准差       x=%.5f   y=%.5f   z=%.5f" % std)
    if max(std) > 0.02:
        print("                   ↑ 偏大，车没停稳或者有振动，重测")
    print("  模长             %.6f g" % norm)
    if abs(norm - CURRENT_NORM) > 0.01:
        print("                   ↑ 和配置里的 %.4f 差超过 1%%，IMU 标度不该变，先查是不是没停稳"
              % CURRENT_NORM)
    print("  车体侧倾残差     %.2f°%s" % (side_deg, "" if abs(side_deg) < 1.5 else "   ← 车歪了或者装配带了 roll"))
    if args.chassis_pitch:
        print("  实测倾角 %.2f° 减掉车体俯仰 %.2f°" % (measured_deg, args.chassis_pitch))
    report(measured_deg - args.chassis_pitch, gravity)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
