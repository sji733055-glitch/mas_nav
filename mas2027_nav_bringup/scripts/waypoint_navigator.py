#!/usr/bin/env python3
# ... (保持原有版权和文档注释)

import csv
import math
import os
import threading
from typing import List, Tuple, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

# 状态机取值
_WAIT_SERVER = "WAIT_SERVER"
_DELAY = "DELAY"
_SEND = "SEND"
_AWAIT_ACCEPT = "AWAIT_ACCEPT"
_AWAIT_RESULT = "AWAIT_RESULT"
_PAUSE = "PAUSE"
_STOPPED = "STOPPED"

_TICK_PERIOD_SEC = 0.2


class WaypointNavigator(Node):
    """按顺序把航点作为 NavigateToPose 目标发出去, 跑完可循环."""

    def __init__(self):
        super().__init__("waypoint_navigator")

        # 参数声明（保持不变）
        self.declare_parameter("waypoints", [])
        self.declare_parameter("waypoint_file", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("loop", True)
        self.declare_parameter("start_delay_sec", 5.0)
        self.declare_parameter("goal_timeout_sec", 60.0)
        self.declare_parameter("pause_at_waypoint_sec", 0.2)
        self.declare_parameter("attempts_per_waypoint", 2)
        self.declare_parameter("stop_on_failure", False)

        # 参数读取（保持不变）
        self._frame_id = self.get_parameter("frame_id").value
        self._loop = bool(self.get_parameter("loop").value)
        self._start_delay = float(self.get_parameter("start_delay_sec").value)
        self._goal_timeout = float(self.get_parameter("goal_timeout_sec").value)
        self._pause = float(self.get_parameter("pause_at_waypoint_sec").value)
        self._attempts = max(1, int(self.get_parameter("attempts_per_waypoint").value))
        self._stop_on_failure = bool(self.get_parameter("stop_on_failure").value)
        
        # 航点解析
        csv_path = str(self.get_parameter("waypoint_file").value or "").strip()
        if csv_path:
            self._waypoints = self._parse_csv(csv_path)
        else:
            self._waypoints = self._parse_waypoints(
                self.get_parameter("waypoints").value
            )

        # Action客户端和状态变量
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        
        # 添加线程锁保护共享状态
        self._lock = threading.Lock()
        
        self._state = _WAIT_SERVER
        self._index = 0
        self._attempt = 0
        self._lap = 0
        self._goal_handle = None
        self._send_future = None
        self._result_future = None
        self._mark = self.get_clock().now()
        self._deadline = None
        
        # 添加取消完成标志
        self._cancel_future = None

        if not self._waypoints:
            self.get_logger().error(
                "没有航点，节点空转。给 waypoint_file:=某个.csv（waypoint_editor 保存），"
                "或 waypoints:=[x, y, yaw_deg, ...] 扁平数组。"
            )
            self._state = _STOPPED
        else:
            self.get_logger().info(
                f"航点自动行驶：{len(self._waypoints)} 个航点，frame_id={self._frame_id}，"
                f"loop={self._loop}，单点超时 {self._goal_timeout:.1f}s，"
                f"每点最多 {self._attempts} 次尝试。等 navigate_to_pose 服务端..."
            )

        self._timer = self.create_timer(_TICK_PERIOD_SEC, self._tick)

    def _parse_csv(self, path: str) -> List[Tuple[float, float, float]]:
        """Load waypoint_editor CSV: id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,..."""
        if not os.path.isfile(path):
            self.get_logger().error(f"waypoint_file 不存在: {path}")
            return []
        
        waypoints = []
        try:
            with open(path, newline="") as handle:
                reader = csv.reader(handle)
                header = next(reader, None)
                if header is None:
                    return []
                
                for row_num, row in enumerate(reader, start=2):  # 从第2行开始（跳过header）
                    if len(row) < 8:
                        self.get_logger().warn(f"CSV第{row_num}行列数不足，跳过")
                        continue
                    
                    try:
                        x = float(row[1])
                        y = float(row[2])
                        qz = float(row[6])
                        qw = float(row[7])
                        
                        # 添加四元数合法性检查
                        quat_norm = qz*qz + qw*qw
                        if abs(quat_norm - 1.0) > 0.1:  # 允许10%的误差
                            self.get_logger().warn(
                                f"CSV第{row_num}行四元数不规范 (|q|={quat_norm:.3f})，跳过"
                            )
                            continue
                        
                        # 归一化四元数
                        norm = math.sqrt(quat_norm)
                        qz /= norm
                        qw /= norm
                        
                        yaw_deg = math.degrees(math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz))
                        waypoints.append((x, y, yaw_deg))
                        
                    except (ValueError, ZeroDivisionError) as exc:
                        self.get_logger().warn(f"CSV第{row_num}行解析失败: {exc}，跳过")
                        continue
                        
        except OSError as exc:
            self.get_logger().error(f"读 waypoint_file 失败: {exc}")
            return []
        
        self.get_logger().info(f"从 CSV 读到 {len(waypoints)} 个航点: {path}")
        return waypoints

    def _parse_waypoints(self, raw) -> List[Tuple[float, float, float]]:
        """Turn the flat [x, y, yaw_deg, ...] array into a list of triples."""
        if raw is None:
            return []
        
        try:
            values = [float(v) for v in raw]
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f"waypoints 参数解析失败: {exc}")
            return []
        
        if len(values) % 3 != 0:
            self.get_logger().error(
                f"waypoints 长度 {len(values)} 不是 3 的倍数，按 [x, y, yaw_deg] 三元组解析失败。"
            )
            return []
        
        return [tuple(values[i : i + 3]) for i in range(0, len(values), 3)]

    def _elapsed(self) -> float:
        """计算从当前状态开始经过的时间（秒）"""
        return (self.get_clock().now() - self._mark).nanoseconds * 1e-9

    def _goto(self, state: str):
        """切换到新状态并重置计时器"""
        self._state = state
        self._mark = self.get_clock().now()
        self.get_logger().debug(f"状态切换: {state}")

    def _make_goal(self, waypoint: Tuple[float, float, float]) -> NavigateToPose.Goal:
        """根据航点创建导航目标"""
        x, y, yaw_deg = waypoint
        yaw = math.radians(yaw_deg)
        
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        goal.pose = pose
        
        return goal

    def _safe_cancel_goal(self):
        """安全地取消当前目标"""
        with self._lock:
            if self._goal_handle is not None:
                self._cancel_future = self._goal_handle.cancel_goal_async()
                self._goal_handle = None

    def _tick(self):
        """State machine driven by a timer, so nothing here may block."""
        with self._lock:
            # 处理取消完成
            if self._cancel_future is not None and self._cancel_future.done():
                self._cancel_future = None
            
            if self._state == _STOPPED:
                return

            if self._state == _WAIT_SERVER:
                if self._client.server_is_ready():
                    self.get_logger().info(
                        f"navigate_to_pose 就绪，{self._start_delay:.1f}s 后开始跑航线。"
                    )
                    self._goto(_DELAY)
                else:
                    self.get_logger().info(
                        "等 navigate_to_pose 服务端...", throttle_duration_sec=5.0
                    )
                return

            if self._state == _DELAY:
                if self._elapsed() >= self._start_delay:
                    self._goto(_SEND)
                return

            if self._state == _SEND:
                # bt_navigator 掉了就退回等待
                if not self._client.server_is_ready():
                    self.get_logger().warn("navigate_to_pose 服务端消失，退回等待。")
                    self._goto(_WAIT_SERVER)
                    return
                
                waypoint = self._waypoints[self._index]
                self._attempt += 1
                self.get_logger().info(
                    f"[第 {self._lap + 1} 圈] 航点 {self._index + 1}/{len(self._waypoints)} "
                    f"-> x={waypoint[0]:.2f} y={waypoint[1]:.2f} yaw={waypoint[2]:.1f}° "
                    f"(第 {self._attempt}/{self._attempts} 次尝试)"
                )
                
                self._send_future = self._client.send_goal_async(
                    self._make_goal(waypoint)
                )
                self._goto(_AWAIT_ACCEPT)
                return

            if self._state == _AWAIT_ACCEPT:
                self._tick_await_accept()
                return

            if self._state == _AWAIT_RESULT:
                self._tick_await_result()
                return

            if self._state == _PAUSE and self._elapsed() >= self._pause:
                self._advance()

    def _tick_await_accept(self):
        """处理等待目标被接受的逻辑"""
        if not self._send_future.done():
            return
        
        goal_handle = self._send_future.result()
        self._send_future = None
        
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn(f"航点 {self._index + 1} 的目标被 bt_navigator 拒绝。")
            self._on_failure()
            return
        
        self._goal_handle = goal_handle
        self._result_future = goal_handle.get_result_async()
        self._deadline = self.get_clock().now() + rclpy.duration.Duration(
            seconds=self._goal_timeout
        )
        self._goto(_AWAIT_RESULT)

    def _tick_await_result(self):
        """处理等待导航结果的逻辑"""
        if self._result_future.done():
            status = self._result_future.result().status
            self._result_future = None
            self._goal_handle = None
            
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f"航点 {self._index + 1} 到达。")
                self._attempt = 0
                self._goto(_PAUSE)
            else:
                self.get_logger().warn(f"航点 {self._index + 1} 失败，GoalStatus={status}。")
                self._on_failure()
            return

        # 检查超时
        if self.get_clock().now() >= self._deadline:
            self.get_logger().warn(
                f"航点 {self._index + 1} 超过 {self._goal_timeout:.1f}s 未完成，撤销该目标。"
            )
            self._safe_cancel_goal()
            self._result_future = None
            self._on_failure()

    def _on_failure(self):
        """处理航点失败的情况"""
        if self._attempt < self._attempts:
            self._goto(_SEND)
            return
        
        self._attempt = 0
        
        if self._stop_on_failure:
            self.get_logger().error(
                f"航点 {self._index + 1} 用尽 {self._attempts} 次尝试，"
                "stop_on_failure=true，航线停止。"
            )
            self._goto(_STOPPED)
            return
        
        self.get_logger().warn(f"跳过航点 {self._index + 1}，继续下一个。")
        self._goto(_PAUSE)

    def _advance(self):
        """前进到下一个航点"""
        self._index += 1
        
        if self._index < len(self._waypoints):
            self._goto(_SEND)
            return
        
        self._index = 0
        self._lap += 1
        
        if self._loop:
            self.get_logger().info(f"第 {self._lap} 圈跑完，从头开始。")
            self._goto(_SEND)
        else:
            self.get_logger().info("航线跑完，loop=false，停止。")
            self._goto(_STOPPED)

    def destroy_node(self):
        """清理资源"""
        self._safe_cancel_goal()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.get_logger().error(f"未预期的异常: {exc}")
    finally:
        # 清理资源
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()