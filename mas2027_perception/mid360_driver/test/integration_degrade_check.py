#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双雷达掉线降级自测：本机假雷达 → 真实 mid360_driver 节点 → 真实话题。

不接真雷达也能验证"一台掉线后进入单雷达模式、导航输入不断流"：
脚本用回环地址 127.0.0.2 / 127.0.0.3 冒充前/后雷达，按 Livox 私有协议发点云与 IMU
UDP 包（time_type=0，即现场的无 PTP 模式），再订阅 /mid360_driver/lidar 与
/mid360_driver/imu 逐条核对。

覆盖的场景（时间轴从两路雷达都开始发数据算起）：
  0.0–1.5  s  双雷达：一条消息里同时含前后雷达的点（强度 10 / 20）
  1.5  s      停后雷达 → 前雷达单独发帧，点云不断流
  3.0  s      后雷达恢复（静默 1.5 s > packet_resync_silence）→ 恢复双雷达配对
  4.5  s      停前雷达 → 后雷达单独发帧，且 IMU 切到后雷达（旋转到前雷达系）
  6.5  s      前雷达恢复 → 恢复双雷达配对，IMU 切回前雷达
  8.0  s      停后雷达
  9.5  s      后雷达重启（设备时钟归零）→ 仍能重新配对（验证时间戳重锚定）
 11.5  s      结束

用法：
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    python3 src/mas2027_perception/mid360_driver/test/integration_degrade_check.py

退出码 0 = 全部通过。
"""

import argparse
import math
import os
import shutil
import socket
import struct
import subprocess
import sys
import time

# 必须在 import/初始化 rclpy 之前设好，父进程与子进程才会落在同一个域。
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
os.environ["ROS_DOMAIN_ID"] = os.environ.get("MID360_TEST_DOMAIN_ID", "231")
# 沙箱/只读 HOME 下 ~/.ros/log 可能不可写，日志统一落到工作目录的 log/ 下。
LOG_DIR = os.environ.get("ROS_LOG_DIR") or os.path.join(os.getcwd(), "log", "mid360_degrade_check")
os.makedirs(LOG_DIR, exist_ok=True)
os.environ["ROS_LOG_DIR"] = LOG_DIR

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Imu, PointCloud2  # noqa: E402

# Livox 私有协议：见 src/mid360_driver.cpp 的 DataHeader / CartesianLowPoint / Imu。
# DataHeader = version(u8) length(u16) time_interval(u16) dot_num(u16) udp_cnt(u16)
#              frame_cnt(u8) data_type(u8) time_type(u8) reserved[12] crc32(u32) timestamp(u64)
DATA_HEADER = struct.Struct("<BHHHHBBB12sIQ")
CARTESIAN_LOW_POINT = struct.Struct("<hhhBB")
IMU_PAYLOAD = struct.Struct("<ffffff")

DATA_TYPE_IMU = 0x00
DATA_TYPE_CARTESIAN_LOW = 0x02
TIME_TYPE_NO_SYNC = 0x00

FRONT_IP = "127.0.0.1"
FRONT_SRC_IP = "127.0.0.2"
BACK_SRC_IP = "127.0.0.3"
CLOUD_PORT = 56301
IMU_PORT = 56401
CLOUD_SRC_PORT = 56300
IMU_SRC_PORT = 56400

FRONT_INTENSITY = 10.0
BACK_INTENSITY = 20.0
# 后雷达→前雷达外参，与 config/params.yaml 的 merge_extrinsic_back_to_front 一致。
BACK_TO_FRONT_EXTRINSIC = [0.0, 0.200484459, -0.223172538, -1.677800, 0.0, 0.0]

CLOUD_PERIOD_S = 0.05
IMU_PERIOD_S = 0.01
# 点云消息之间允许的最大空档：降级判定 0.4 s + 一个发布周期 0.05 s + 余量。
MAX_CLOUD_GAP_S = 0.60


def quaternion_rotation(extrinsic):
    """只取外参的旋转部分（与驱动里的 make_merge_transform 同一套公式）。"""
    roll, pitch, yaw = extrinsic[3], extrinsic[4], extrinsic[5]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
        [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
        [-sp, sr * cp, cr * cp],
    ]


def rotate(rotation, vector):
    return [sum(rotation[row][col] * vector[col] for col in range(3)) for row in range(3)]


class FakeRadar:
    """一台假 MID360：两台 UDP socket（点云 / IMU），源地址与源端口都要像真雷达。"""

    def __init__(self, source_ip, intensity, point_mm, acc, gyro):
        self.source_ip = source_ip
        self.intensity = intensity
        self.point_mm = point_mm
        self.acc = acc
        self.gyro = gyro
        self.cloud_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cloud_socket.bind((source_ip, CLOUD_SRC_PORT))
        self.imu_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.imu_socket.bind((source_ip, IMU_SRC_PORT))
        self.active = False
        self.device_clock_origin = None

    def start(self, reboot=False):
        """开始发包；reboot=True 模拟雷达重启（内部时钟从 0 重新计时）。"""
        self.active = True
        if self.device_clock_origin is None or reboot:
            self.device_clock_origin = time.monotonic()

    def stop(self):
        self.active = False

    def _timestamp_ns(self):
        return int((time.monotonic() - self.device_clock_origin) * 1e9)

    def send_cloud(self, counter):
        x, y, z = self.point_mm
        payload = CARTESIAN_LOW_POINT.pack(x, y, z, int(self.intensity), 0)
        header = DATA_HEADER.pack(
            5,
            DATA_HEADER.size + len(payload),
            5000,  # time_interval：0.1 µs 单位，5000 → 包内 0.5 ms
            1,
            counter & 0xFFFF,
            0,
            DATA_TYPE_CARTESIAN_LOW,
            TIME_TYPE_NO_SYNC,
            b"\x00" * 12,
            0,  # crc32 = 0 且节点 validate_crc=false
            self._timestamp_ns(),
        )
        self.cloud_socket.sendto(header + payload, (FRONT_IP, CLOUD_PORT))

    def send_imu(self):
        payload = IMU_PAYLOAD.pack(
            self.gyro[0], self.gyro[1], self.gyro[2], self.acc[0], self.acc[1], self.acc[2]
        )
        header = DATA_HEADER.pack(
            5,
            DATA_HEADER.size + len(payload),
            0,
            0,
            0,
            0,
            DATA_TYPE_IMU,
            TIME_TYPE_NO_SYNC,
            b"\x00" * 12,
            0,
            self._timestamp_ns(),
        )
        self.imu_socket.sendto(header + payload, (FRONT_IP, IMU_PORT))


class Recorder(Node):
    def __init__(self):
        super().__init__("mid360_degrade_check")
        self.started_at = None
        self.clouds = []  # (wall, header_stamp, [(x, y, z, intensity), ...])
        self.imus = []    # (wall, header_stamp, (ax, ay, az))
        self.create_subscription(PointCloud2, "/mid360_driver/lidar", self.on_cloud, 200)
        self.create_subscription(Imu, "/mid360_driver/imu", self.on_imu, 400)

    def on_cloud(self, msg):
        points = []
        for index in range(msg.width):
            offset = index * msg.point_step
            x, y, z, intensity = struct.unpack_from("<ffff", msg.data, offset)
            points.append((x, y, z, intensity))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.started_at is None:
            self.started_at = time.monotonic()
        self.clouds.append((time.monotonic(), stamp, points))

    def on_imu(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.imus.append(
            (
                time.monotonic(),
                stamp,
                (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z),
            )
        )


def intensities_of(points):
    return {round(point[3]) for point in points}


def find_point(points, intensity):
    for x, y, z, value in points:
        if abs(value - intensity) < 0.5:
            return (x, y, z)
    return None


def check(condition, message, failures):
    status = "PASS" if condition else "FAIL"
    print(f"  [{status}] {message}")
    if not condition:
        failures.append(message)


def start_node(executable, ros_args):
    if executable:
        command = [executable, "--ros-args"] + ros_args
    else:
        ros2 = shutil.which("ros2")
        if ros2 is None:
            raise RuntimeError("找不到 ros2 命令，请先 source /opt/ros/jazzy/setup.bash 与 install/setup.bash")
        command = [ros2, "run", "mid360_driver", "mid360_driver_node", "--ros-args"] + ros_args
    print("启动节点：" + " ".join(command))
    log_path = os.path.join(LOG_DIR, "node_console.log")
    log_file = open(log_path, "w", encoding="utf-8")
    print(f"节点日志：{log_path}")
    return subprocess.Popen(command, env=os.environ.copy(), stdout=log_file, stderr=subprocess.STDOUT), log_path


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--domain-id", default=os.environ["ROS_DOMAIN_ID"], help="隔离用的 ROS_DOMAIN_ID（默认 231）")
    parser.add_argument("--node", default=None, help="直接指定节点可执行文件（默认用 ros2 run）")
    args = parser.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)

    ros_args = [
        "-p", "host_ip:=127.0.0.1",
        "-p", "lidar_topic:=/mid360_driver/lidar",
        "-p", "imu_topic:=/mid360_driver/imu",
        "-p", "lidar_frame:=lidar_link",
        "-p", "imu_frame:=lidar_imu",
        "-p", "lidar_publish_time_interval:=0.05",
        "-p", "is_topic_name_with_lidar_ip:=false",
        "-p", "enable_lidar_merge:=true",
        "-p", "merge_front_ip:=127.0.0.2",
        "-p", "merge_back_ip:=127.0.0.3",
        "-p", "merge_extrinsic_back_to_front:=[0.0,0.200484459,-0.223172538,-1.677800,0.0,0.0]",
        "-p", "merge_max_interval_ms:=100.0",
        "-p", "merge_stale_timeout_s:=0.4",
        "-p", "merge_recover_hold_s:=0.3",
        "-p", "merge_imu_stale_timeout_s:=0.1",
        "-p", "packet_resync_silence:=1.0",
        "-p", "validate_crc:=false",
        "-p", "max_packet_time_jump:=0.5",
        "-p", "max_packet_time_span:=0.1",
        "-p", "max_point_range:=300.0",
        "-p", "max_imu_acc:=100.0",
        "-p", "max_imu_gyro:=100.0",
        "-p", "min_drop_log_interval:=0.5",
    ]
    node, node_log_path = start_node(args.node, ros_args)
    failures = []
    recorder = None
    try:
        rclpy.init()
        recorder = Recorder()

        # 前雷达：参考雷达，原点处的点原样发出；IMU 单位加速度 (0,0,1)。
        # 后雷达：点在前方 1 m（后系 +z = 车体前向），IMU 同样是 (0,0,1)，经旋转后应变成前系的 (0,0.9943,−0.1068)。
        front = FakeRadar(FRONT_SRC_IP, FRONT_INTENSITY, (1000, 0, 0), (0.0, 0.0, 1.0), (0.0, 0.0, 0.0))
        back = FakeRadar(BACK_SRC_IP, BACK_INTENSITY, (0, 0, 1000), (0.0, 0.0, 1.0), (0.0, 0.0, 0.0))

        # 等节点起来（订阅到话题为止）。
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline and not recorder.clouds:
            rclpy.spin_once(recorder, timeout_sec=0.05)
            front.start()
            back.start()
            front.send_cloud(0)
            back.send_cloud(0)
            front.send_imu()
            back.send_imu()
            time.sleep(0.05)
        if node.poll() is not None:
            raise RuntimeError(f"节点启动即退出，returncode={node.returncode}")

        # 时间轴：t 从"两路雷达都开始发数据"算起。
        timeline = [
            (0.0, "both"),
            (1.5, "stop_back"),
            (3.0, "start_back"),
            (4.5, "stop_front"),
            (6.5, "start_front"),
            (8.0, "stop_back"),
            (9.5, "reboot_back"),
            (11.5, "end"),
        ]
        print("\n开始时间轴（共 11.5 s）…")
        t0 = time.monotonic()
        next_action = 0
        last_cloud_sent = 0.0
        last_imu_sent = 0.0
        cloud_counter = 0
        while True:
            now = time.monotonic()
            elapsed = now - t0
            if next_action < len(timeline) and elapsed >= timeline[next_action][0]:
                action = timeline[next_action][1]
                print(f"  t={elapsed:5.2f}s  动作：{action}")
                if action == "both":
                    front.start()
                    back.start()
                elif action == "stop_back":
                    back.stop()
                elif action == "start_back":
                    back.start()
                elif action == "stop_front":
                    front.stop()
                elif action == "start_front":
                    front.start()
                elif action == "reboot_back":
                    back.start(reboot=True)  # 设备时钟归零，模拟雷达重启
                next_action += 1
                if action == "end":
                    break
            if elapsed - last_cloud_sent >= CLOUD_PERIOD_S:
                last_cloud_sent = elapsed
                cloud_counter += 1
                for radar in (front, back):
                    if radar.active:
                        radar.send_cloud(cloud_counter)
            if elapsed - last_imu_sent >= IMU_PERIOD_S:
                last_imu_sent = elapsed
                for radar in (front, back):
                    if radar.active:
                        radar.send_imu()
            rclpy.spin_once(recorder, timeout_sec=0.002)

        # 收尾：再 spin 一小会儿，把在途消息收全。
        settle_deadline = time.monotonic() + 0.3
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(recorder, timeout_sec=0.02)

        clouds = recorder.clouds
        imus = recorder.imus
        print(f"\n收到点云 {len(clouds)} 条、IMU {len(imus)} 条")
        if not clouds:
            print("FAIL: 一条点云都没收到")
            return 1
        origin = clouds[0][0]

        def window(start, end):
            return [entry for entry in clouds if start <= entry[0] - origin < end]

        def merged_windows(entries):
            return [entry for entry in entries if intensities_of(entry[2]) >= {FRONT_INTENSITY, BACK_INTENSITY}]

        print("\n场景 1：双雷达融合（0.5–1.5 s）")
        seg = window(0.5, 1.5)
        check(bool(seg), "有点云输出", failures)
        check(bool(merged_windows(seg)), "同一条消息里同时含前后雷达的点（强度 10 与 20）", failures)

        print("\n场景 2：后雷达掉线 → 单雷达模式（2.0–3.0 s）")
        seg = window(2.0, 3.0)
        check(bool(seg), "点云仍在输出（没有整段静默）", failures)
        check(all(intensities_of(entry[2]) == {FRONT_INTENSITY} for entry in seg), "只剩前雷达的点", failures)

        print("\n场景 3：后雷达恢复 → 重新配对（3.8–4.5 s）")
        seg = window(3.8, 4.5)
        check(bool(merged_windows(seg)), "恢复成双雷达融合帧（时间戳重锚定生效）", failures)

        print("\n场景 4：前雷达掉线 → 单雷达模式 + IMU 换源（5.2–6.5 s）")
        seg = window(5.2, 6.5)
        check(bool(seg), "点云仍在输出", failures)
        check(all(intensities_of(entry[2]) == {BACK_INTENSITY} for entry in seg), "只剩后雷达的点", failures)
        rotation = quaternion_rotation(BACK_TO_FRONT_EXTRINSIC)
        expected_point = rotate(rotation, [0.0, 0.0, 1.0])
        expected_point = [expected_point[0] + BACK_TO_FRONT_EXTRINSIC[0],
                          expected_point[1] + BACK_TO_FRONT_EXTRINSIC[1],
                          expected_point[2] + BACK_TO_FRONT_EXTRINSIC[2]]
        if seg:
            point = find_point(seg[0][2], BACK_INTENSITY)
            check(point is not None, "后雷达的点仍在帧里", failures)
            if point is not None:
                check(
                    all(abs(point[i] - expected_point[i]) < 2e-3 for i in range(3)),
                    f"后雷达点已换算到前雷达系：{tuple(round(v, 4) for v in point)} ≈ {tuple(round(v, 4) for v in expected_point)}",
                    failures,
                )
        imu_seg = [entry for entry in imus if 5.2 <= entry[0] - origin < 6.5]
        check(bool(imu_seg), "IMU 仍在输出（换到后雷达）", failures)
        expected_acc = rotate(rotation, [0.0, 0.0, 1.0])
        if imu_seg:
            sample = imu_seg[len(imu_seg) // 2][2]
            check(
                all(abs(sample[i] - expected_acc[i]) < 2e-3 for i in range(3)),
                f"IMU 已按外参旋转到前雷达系：{tuple(round(v, 4) for v in sample)} ≈ {tuple(round(v, 4) for v in expected_acc)}",
                failures,
            )

        print("\n场景 5：前雷达恢复 → 重新配对、IMU 切回（7.0–8.0 s）")
        seg = window(7.0, 8.0)
        check(bool(merged_windows(seg)), "恢复成双雷达融合帧", failures)
        imu_seg = [entry for entry in imus if 7.0 <= entry[0] - origin < 8.0]
        if imu_seg:
            sample = imu_seg[len(imu_seg) // 2][2]
            check(
                all(abs(sample[i] - (0.0, 0.0, 1.0)[i]) < 2e-3 for i in range(3)),
                f"IMU 切回前雷达（未经旋转）：{tuple(round(v, 4) for v in sample)}",
                failures,
            )

        print("\n场景 6：后雷达重启（设备时钟归零）→ 仍能重新配对（10.4–11.5 s）")
        seg = window(10.4, 11.5)
        check(bool(merged_windows(seg)), "重启后的后雷达重新参与融合", failures)

        print("\n全局不变量：点云时间戳连续、无长空档")
        stamps = [entry[1] for entry in clouds]
        check(all(stamps[i] <= stamps[i + 1] for i in range(len(stamps) - 1)), "点云 header.stamp 单调不减", failures)
        # 空档要把"整段停发"也算进去：端点补上时间轴的首尾，否则一条消息都不发反而看起来没有空档。
        timeline_end = timeline[-1][0]
        marks = [origin] + [entry[0] for entry in clouds] + [origin + timeline_end]
        gaps = [marks[i + 1] - marks[i] for i in range(len(marks) - 1)]
        worst = max(gaps) if gaps else timeline_end
        check(worst <= MAX_CLOUD_GAP_S, f"点云最大空档 {worst * 1e3:.0f} ms ≤ {MAX_CLOUD_GAP_S * 1e3:.0f} ms", failures)
        # 每 0.5 s 都要有点云（从 0.8 s 起，前面留出启动与首次配对的时间）。
        empty_windows = [
            (start, start + 0.5)
            for start in [0.8 + 0.5 * index for index in range(int((timeline_end - 0.8) / 0.5))]
            if not window(start, start + 0.5)
        ]
        check(not empty_windows, f"每个 0.5 s 窗口都有点云（空窗口：{empty_windows}）", failures)
        check(
            clouds[-1][0] - origin >= timeline_end - 0.5,
            f"点云一直发到时间轴结束（最后一条在 {clouds[-1][0] - origin:.2f} s）",
            failures,
        )

        check(node.poll() is None, "节点全程存活", failures)
    finally:
        if recorder is not None:
            recorder.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001 - 清理阶段不再抛错
            pass
        node.terminate()
        try:
            node.wait(timeout=5)
        except subprocess.TimeoutExpired:
            node.kill()

    if failures:
        print(f"\n结果：FAIL（{len(failures)} 项）")
        for item in failures:
            print(f"  - {item}")
        print(f"\n节点日志尾部（{node_log_path}）：")
        with open(node_log_path, encoding="utf-8", errors="replace") as handle:
            lines = handle.readlines()
        print("".join(lines[-40:]))
        return 1
    print("\n结果：PASS（全部场景通过）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
