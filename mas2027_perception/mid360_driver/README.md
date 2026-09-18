# Mid-360 Driver

This is an implementation of the Mid-360 driver, intended to serve as a replacement for [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2).

<img src="./img/ACE.jpg" width="200px">

## Install dependencies

1. Please make sure you have install ROS2.
2. Install Asio. If you are using ubuntu, you can install by following command: `sudo apt install libasio-dev`

## Param

here are some parameters you can set in config file:

```yaml
mid360_driver:
    ros__parameters:
        lidar_topic: /livox/lidar
        lidar_frame: livox_frame
        imu_topic: /livox/imu
        imu_frame: imu_frame
        lidar_publish_time_interval: 0.1
        is_topic_name_with_lidar_ip: false # 是否在话题名后面加雷达ip，可以用于区分多个雷达
```

## 双雷达融合与单雷达降级

`enable_lidar_merge: true` 时驱动把两台 MID360 融合成一路发布（本工程默认，见
`config/params.yaml` 与 `mas2027_nav_bringup/config/small_point_lio_params.yaml`）：
后雷达的点按 `merge_extrinsic_back_to_front` 变到前（参考）雷达系，两路各按
`lidar_publish_time_interval` 切帧、按 `merge_max_interval_ms` 配对后合成一帧，
绝不发"半帧"。IMU 默认只转发参考雷达的。

任一雷达掉线时进入**单雷达模式**，导航输入不断流：

| 参数 | 默认 | 作用 |
| --- | --- | --- |
| `merge_stale_timeout_s` | `0.5` | 某台雷达超过这么久没有点云即判为掉线，改用另一台单独发帧；`0` = 关闭降级 |
| `merge_recover_hold_s` | `0.5` | 掉线雷达回来后需连续在线这么久才重新参与配对（防边界抖动）；首次收到数据不受此限 |
| `merge_imu_stale_timeout_s` | `0.1` | 参考雷达 IMU 超过这么久没有数据，IMU 切到另一台 |
| `packet_resync_silence` | `1.0` | 某台雷达静默超过这么久后允许重新锚定时间戳；`0` = 关闭 |

要点：

- 降级期间仍发布**同一个话题、同一个坐标系**，LIO 侧无需任何改动；后雷达单独工作时，
  它的点已经在前雷达系里。
- IMU 换源时，后雷达的陀螺与加速度会按外参旋转到前雷达 IMU 系，并补上两安装点之间的
  离心项 `ω×(ω×d)`（角加速度项不补偿：需要微分陀螺，噪声大于收益）。两端 IMU 的零偏
  不同，切换瞬间 LIO 会看到一次偏置台阶，属已知限制。
- `packet_resync_silence` 修的是一个更基础的坑：时间戳水位线只在"接受"时前进，所以
  一次超过 `max_packet_time_jump` 的断线（或雷达重启后内部时钟归零）会让这台雷达
  **永久**被判为 `implausible timestamp` 而再也收不到数据。重锚定会同时重算
  `NO_SYNC` 模式下的设备时钟→主机墙钟 delta。
- 双雷达都静默时不发任何帧（没有可用数据）；这时应查网络/供电，而不是驱动参数。

## 测试

```bash
# 纯逻辑单测（外参变换 / IMU 换源换算 / 在线判据），已挂 CTest
colcon test --packages-select mid360_driver --event-handlers console_direct+

# 端到端：本机假雷达（127.0.0.2/127.0.0.3）发 Livox UDP 包，真实节点 + 真实话题，
# 覆盖掉线降级、恢复重配对、IMU 换源、雷达重启后重锚定
source install/setup.bash
python3 src/mas2027_perception/mid360_driver/test/integration_degrade_check.py
```

## Contact

QQ group: 1070252119

Email: 1709185482@qq.com

## License

Copyright (C) 2025 Yingjie Huang

Licensed under the MIT License. See License.txt in the project root for license information.
