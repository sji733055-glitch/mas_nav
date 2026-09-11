# 双 MID360 融合方案

本文是本仓库把单雷达改成前后双雷达的设计与落地说明。对照了 `navi_minco_bit` 的驱动内融合，以及 `HERO_2026_Sentry_NAV` 的独立 `lidar_merge` 节点；**本栈不换 livox 官方驱动、不搬 HERO 的 Odin1 去畸变链路**。融合发生在现有 `mid360_driver` 里，下游仍只看见一份 `/mid360_driver/lidar`。

驱动内融合已经合入 `mid360_driver`。`enable_lidar_merge` **默认 false**，现车单雷达行为不变。接上两台雷达、填好 IP / 外参 / PTP 后再打开。

---

## 1. 目标与不改什么

### 要变成什么样

```text
前 MID360 ──┐
            ├─ mid360_driver（按源 IP 分桶 → 5 ms 窗口配对 → T_back→front）
后 MID360 ──┘
                │  /mid360_driver/lidar    PointCloud2，frame = lidar_link（前雷达）
                │  /mid360_driver/imu      只用前雷达 IMU
                v
           small_point_lio                 lidar_type: custom_mid360_driver
                │  /Odometry
                │  /cloud_registered
                │  TF odom → base_link
                v
     odom_localizer / ROG-Map / MINCO / Nav2   （话题名与帧名都不改）
```

LIO、ROG-Map、costmap、MINCO、行为树、航点巡逻都继续订现在的话题。双雷达对它们只表现为：同一 `lidar_link` 下点更多、FOV 接近 360°。

### 明确不做

| 做法 | 原因 |
|---|---|
| 换成 `livox_ros_driver2` | 本栈适配器读的是 PointCloud2 字段 `x,y,z,intensity,timestamp`（**绝对秒**）。navi 的 CustomMsg `offset_time`、以及它内部融合写的「相对 timebase 的 timestamp」都会把去畸变算坏 |
| 复制 HERO 的 `lidar_merge` + `hero_lidar_scan` + `dog_map` | HERO 提交里 `TWO_LIDARS` 未打开；去畸变依赖 `/odin1/odometry_highfreq`，本仓库没有 Odin1 |
| 两台雷达各开一个 `small_point_lio` | 两套 odom 无法合成 |
| 把两路 IMU 混进 `/mid360_driver/imu` | 紧耦合滤波器会把两套加速度当成同一刚体 |
| 第二块网卡专接后雷达 | NUC 一个口够；靠交换机 + 源 IP 区分 |

### 现在插第二台会怎样（务必先看）

`is_topic_name_with_lidar_ip: false` 时，驱动已经按 UDP **源 IP** 收两台雷达，但会把两路点、两路 IMU **无外参地拼进同一个 publisher**。后雷达点仍在后雷达系，时间戳也未经配对。这不是融合，LIO 会漂或直接把跨度过大的帧丢掉（见 §5）。

---

## 2. 硬件连接与数据传输

### 2.1 拓扑：一网口 + 交换机

NUC 只有一个以太网口是常态，不是缺陷。navi 也是一块 `enp86s0` 接两台 MID360。

```text
NUC 唯一网口
  静态 IP  192.168.1.50/24     ← 与 small_point_lio_params.yaml 的 host_ip 一致
        │
     [千兆二层交换机]
        ├── 前雷达  192.168.1.xxx    目的 IP = 192.168.1.50
        └── 后雷达  192.168.1.yyy    目的 IP = 192.168.1.50
```

- 没有交换机、一根网线直连 NUC，第二台雷达物理上接不进去。
- USB 转网口再分两段能凑合，PTP 硬件打时戳和时延都差，不当量产方案。
- 容器 `docker-compose.yml` 已是 `network_mode: host`，和主机共用这个口，驱动绑的就是宿主机地址。

### 2.2 UDP 端口（本驱动写死，不用改）

| 方向 | 地址 | 作用 |
|---|---|---|
| 驱动 bind | `host_ip:56301` | 收点云 |
| 驱动 bind | `host_ip:56401` | 收 IMU |
| 雷达发出 | 源端口 `56300` | 点云；其它源端口直接丢 |
| 雷达发出 | 源端口 `56400` | IMU；其它源端口直接丢 |

两台雷达 **目的 IP / 目的端口相同**。驱动用 `sender_endpoint.address()` 认是哪一台，不靠第几块网卡。

Livox Viewer / 设备网页里：

1. 两台雷达 IP 互不冲突，都在 `192.168.1.0/24`。
2. Host IP 都填 `192.168.1.50`。
3. 点云/IMU 端口保持官方默认（56300/56400 → 主机 56301/56401）。

本机没有 `192.168.1.x` 地址时 bind 失败，整条导航链从驱动开始静音。见 `docs/troubleshooting.md`。

### 2.3 带宽

MID360 约 10 Hz、每帧数万点。两路点云 + 两路 IMU + PTP 组播，普通千兆交换机余量足够。不要用会乱丢组播的「IGMP 窥探」廉价交换机，否则 PTP 锁不住。

### 2.4 这个口还要 SSH / 上网

雷达网段是静态 `192.168.1.0/24`。若 NUC 现在靠同一网口 DHCP 上实验室网，会和 `192.168.1.50` 抢。

推荐顺序：

1. **网口专给雷达，SSH 走 Wi‑Fi**（车上最常见）。
2. 实验室交换机划 VLAN：雷达 VLAN 静态 `192.168.1.50`，管理 VLAN 另配。
3. 一根线上行再进实验室：上车载交换机，注意 DHCP 不要改掉 `192.168.1.50`。

### 2.5 容器注意

`pid: host`。不要在 `mas_nav` 里 `pkill -f`，会误杀宿主机进程。停节点用记下的 PID 或 `pkill -x` 精确命令名。

---

## 3. 时间同步（PTP）

### 3.1 PTP 是什么

PTP（IEEE 1588 / gPTP）让两台雷达的**硬件时钟**对齐到同一时间基准。它不是融合算法，只是让「前雷达这一帧」和「后雷达这一帧」的时间戳可以相减。Livox 包头 `time_type`：

| 值 | 含义 | 本驱动行为 |
|---|---|---|
| 0 `NO_SYNC` | 雷达内部钟，和主机无关 | 每个源 IP **第一次**算 `delta = host_now − lidar_internal`，之后一直加这个偏移 |
| 1 `GPTP_OR_PTP` | 已跟 PTP/gPTP | 直接用包内纳秒时间戳 |
| 2 `GPS` | GPS 授时 | 同上，当绝对时间用 |

单雷达 `NO_SYNC` 够用：只有一个钟，偏移是常数。双雷达各锁一次 `delta`，两台雷达内部钟互相漂，几分钟后帧间隔就会超过 5 ms 窗口，融合开始大量丢帧。

### 3.2 为什么窗口是 5 ms

MID360 一圈约 100 ms。`|t_front − t_back| ≤ 5 ms` 表示两帧扫到的是几乎同一时刻的场景，才能当一帧刚体点云喂给 LIO 去畸变。窗口再大，车在转时后雷达点会被用错的 IMU 姿态去畸变。

5 ms 依赖两台雷达时间戳可比。没有 PTP 时不要照抄这个数。

### 3.3 怎么跑（同一块网卡）

PTP master 也跑在 NUC **那唯一的口**上，不需要第二网卡。两台雷达当 slave，从交换机上听同一套 announce。

先确认网卡支持硬件打时戳（Intel i210 / i225 一类常见；很多 USB 网卡没有）：

```bash
ethtool -T <iface> | grep -E 'hardware-transmit|hardware-receive'
```

有 HW timestamping 时（在**宿主机**，不要在容器里乱杀进程），参考 navi 的 `scripts/ptp_sync.bash` 思路：

```bash
# 停掉会抢时钟的 NTP/chrony，再：
sudo ptp4l -i <iface> -2 -m
sudo phc2sys -c <iface> -s CLOCK_REALTIME -O 0
```

`-2` 是 IEEE 802.3 / 以太网封装，Livox 常用。具体 iface 名用 `ip -br a` 看，不要照抄 navi 的 `enp86s0`。

雷达侧在 Livox Viewer 把时间同步设成 PTP/gPTP。驱动日志里对应包的 `time_type` 应为 1，而不再走 `delta_time_map`。

### 3.4 网卡没有硬件打时戳

不要硬上 5 ms。退路：

- 把 `merge_max_interval_ms` 放到 20–50，接受高速自旋时后雷达拖影；或
- 先 `is_topic_name_with_lidar_ip: true` 分话题，用 `message_filters` ApproximateTime 做宽松配对（调试用，正式导航仍建议驱动内融合）。

---

## 4. 坐标系与外参

### 4.1 融合帧 = 前雷达 `lidar_link`

和现在单雷达一样：LIO 的 `lidar_frame` 仍是 `lidar_link`，URDF `lidar_joint` 仍是 **前雷达** 相对 `base_link` 的安装。后雷达点在进 `/mid360_driver/lidar` 之前乘刚体变换到前雷达系。

当前单雷达 URDF（`mas2027_sentry.urdf`）：

```text
base_link → lidar_link
  xyz = 0.00  -0.15  0.11
  rpy = 0.8389  0  0          # roll ≈ 48°，scripts/measure_lidar_mount.py 测的
```

`small_point_lio_params.yaml` 的 `blind_center` 已按当前 URDF 重算（`[0.000, 0.0184, -0.1851]`）。改安装后跑一键脚本，不要手抄旧值：

```bash
bash mas2027_nav_bringup/scripts/measure_lidar_mount.sh
```

在本机 ROS 2 Jazzy 环境下跑。车停水平地面；脚本提示转云台后再动手，底盘刹住、只转云台匀速 1–2 圈。它会分话题起驱动、各雷达各开一个 LIO，先静止测倾角/`gravity`，再自旋测水平偏移，打印 URDF 两个关节、`merge_extrinsic_back_to_front`、`blind_center`。导航栈必须先停（独占雷达 UDP）。只算不改文件。

### 4.2 后 → 前：只有一份外参

```text
merge_extrinsic_back_to_front: [x, y, z, roll, pitch, yaw]
# 米 + 弧度，T 把后雷达光学中心的点变到前雷达光学中心
# p_front = R_back_to_front * p_back + t_back_to_front
```

**不要抄 navi 的 `[0.0, 0.4, 0.0, -0.35453, 0.0, 0.0]`。** 本车是左右双雷达，**手性对称**（关于 `base_link` 的 XZ 平面镜像）：右 `xyz="0.00 -0.15 0.11" rpy="0.8389 0 0"`，左 `xyz="0.00 0.15 0.11" rpy="-0.8389 0 0"`。Y 取反时 roll 也取反。不要两侧写同一套 rpy（那是旋转相同，不是镜面），也不要给左侧加 `yaw=π`（会迭出 `Rz(π)Rx(2α)`）。相对旋转是 `R = Rx(-2α)`（`α=0.8389`，roll=`-1.6778`），平移 `t = Rx(-α)*[0, 0.30, 0]`，即 `merge_extrinsic_back_to_front: [0.0, 0.200484459, -0.223172538, -1.677800, 0.0, 0.0]`。真机点云已按这份外参对齐。不要再复制到盲区 / ROG 五处。本仓库约定：

| 量 | 来源 |
|---|---|
| 前雷达相对车体 | URDF `lidar_joint`（现有） |
| 后雷达相对前雷达 | **唯一**参数 `merge_extrinsic_back_to_front` |
| 后雷达可视化 | URDF `lidar_back_link` = 前关节 ∘ T_back_to_front 的逆（或直接写后雷达相对 base） |
| LIO `blind_center` | 仍是 **base_link 原点在前雷达 lidar_link 中的坐标**；车体变长/雷达改位后跑 `measure_lidar_mount.sh` |
| `min_distance` | 盲区半径要罩住车身在前雷达系里的投影，双雷达后车体更「长」，大概率要加大 |
| IMU 外参 `extrinsic_T/R` | 仍是 **前雷达光学中心 → 前雷达内置 IMU**，一般不用因加后雷达而改 |

### 4.3 IMU 只用前雷达

- 点云：两路融合后发 `/mid360_driver/lidar`，`frame_id = lidar_link`。
- IMU：只转发前雷达 IP 的包到 `/mid360_driver/imu`，`frame_id = lidar_imu`。
- 左雷达 IMU 直接丢弃（回调里只有 `merge_front_ip` 会进 `on_receive_imu`）。要看它就临时关融合、开 `is_topic_name_with_lidar_ip`。

重力向量 `gravity`、`fix_gravity_direction` 仍按前雷达 IMU 静止标定。加装后雷达若改了前雷达倾角，要重跑 `measure_lidar_mount.sh`。

### 4.4 URDF 后雷达 link（已合入）

```xml
<!-- 前雷达：现有 lidar_joint，不要改名 -->
<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.00 -0.15 0.11" rpy="0.8389 0 0"/>
</joint>

<!-- 左侧：仅可视化 / 标定辅助，LIO 不 lookup；与右侧手性对称 -->
<link name="lidar_back_link"/>
<joint name="lidar_back_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_back_link"/>
  <origin xyz="0.00 0.15 0.11" rpy="-0.8389 0 0"/>
</joint>
```

`robot_state_publisher` 会多一条静态 TF。不要再 launch 一份 `base_link → livox_frame` 的恒等 TF（`small_point_lio.launch.py` 里那条是给官方驱动用的，本栈总 launch 不能带它）。

---

## 5. 点云处理（驱动内融合）

### 5.1 为何放在 mid360_driver 里

驱动回调里已经有 `asio::ip::address`。按 IP 入队、配对、变换、一次发布，省掉：

- 外部 merge 节点的订阅/拷贝/再发布；
- HERO ApproximateTime 在 ROS 层排队；
- 把未变换的点送进 LIO。

`LidarDataQueue` 那套是 livox SDK 的，本驱动没有。用每 IP 一个 `std::deque<MergeFrame>` 即可。

### 5.2 配对规则（已实现，配对抄 navi，归并抄 HERO）

代码在 `Mid360DriverNode::stage_merge_points` / `enqueue_merge_frame` / `collect_merged_frames`。

1. 只认 `merge_front_ip` / `merge_back_ip`，其它源 IP 直接丢。
2. UDP 回调只**暂存**：前雷达点原样进 `merge_front_pending_`；后雷达点先 `p' = R p + t` 再进 `merge_back_pending_`。变换放在入队前，配对时不再动坐标。
3. `lidar_publish_time_interval`（0.05 s）的定时器把两路 pending 各切成一帧压进各自队列，`base_timestamp` = 该帧点时间戳的 min（与 `header.stamp` 取法一致）。
4. 两队都非空才配对；**禁止发半帧**。`|Δbase| ≤ merge_max_interval_ms` → 合并，两队各 pop 一帧。
5. 超限 → pop 更旧的队头（`front_base <= back_base` 就丢 front），节流 warn `lidar merge dropped stale ...`。
6. 一台掉线：另一队涨到 `MAX_MERGE_QUEUE_FRAMES`(8) 就丢最旧的，节流 warn `lidar merge is waiting for the ... lidar`，`/mid360_driver/lidar` 静音。宁可停也不喂半个 FOV 给 LIO；要「降级单雷达继续跑」得另外加超时兜底。
7. 用 `std::merge` 做双指针归并（两路各自已按时间递增），输出整帧按**绝对秒**有序，避免 LIO `Preprocess` 的 `last_timestamp_lidar` 水位线把后半帧整段砍掉。
8. 每点 `timestamp` 仍是**绝对秒**，不是 offset。不要学 navi 的 PointCloud2 分支写相对 timebase，也不要学 HERO rebase 到较晚 timebase。
9. `header.stamp` = 合并帧点时间戳的 min，`frame_id = lidar_link`。
10. 发布仍走 `publish_points`，`span > 2 * max_packet_time_span`(0.2 s) 的整帧检查保留。配对已保证两帧同期，正常跨度 ≈ 0.05 s + 窗口，不会误杀。
11. 发布在锁外做：`collect_merged_frames()` 只动队列并返回帧，定时器拿到后再 publish。

### 5.3 点格式（必须保持）

```text
PointCloud2 fields:
  x,y,z          FLOAT32
  intensity      FLOAT32
  timestamp      FLOAT64   绝对秒，不是 offset_time、也不是相对 timebase
point_step = 24
```

`lidar_type: custom_mid360_driver` 不要改成 `livox_custom_msg`，除非整条驱动都换成 CustomMsg。

### 5.4 参数（已合入 YAML）

写在 `mas2027_nav_bringup/config/small_point_lio_params.yaml` 的 `mid360_driver` 段（包内 `config/params.yaml` 只是默认，bringup 会覆盖）：

```yaml
mid360_driver:
  ros__parameters:
    lidar_topic: /mid360_driver/lidar
    lidar_frame: lidar_link
    imu_topic: /mid360_driver/imu
    imu_frame: lidar_imu
    lidar_publish_time_interval: 0.05
    host_ip: 192.168.1.50
    is_topic_name_with_lidar_ip: false   # 融合开启后保持 false；调试分话题时才 true
    validate_crc: false
    max_packet_time_jump: 0.5
    max_packet_time_span: 0.1
    max_point_range: 300.0

    enable_lidar_merge: true             # 已开
    merge_front_ip: "192.168.1.136"      # 右侧原雷达，点坐标不变，IMU 也只用它
    merge_back_ip: "192.168.1.193"       # 左侧雷达，点先乘 T 再拼
    merge_max_interval_ms: 50.0          # 配对窗口；上 PTP 后收到 5.0
    merge_extrinsic_back_to_front: [0.0, 0.200484459, -0.223172538, -1.677800, 0.0, 0.0]  # 手性对称 R=Rx(-2α)
```

调试两路原始点走 `enable_lidar_merge: false` + `is_topic_name_with_lidar_ip: true`，此时 `front_lidar_frame` / `back_lidar_frame` 让两个话题各自带自己的 frame。

互斥：

- `enable_lidar_merge: true` 且 `is_topic_name_with_lidar_ip: true` → 启动直接失败，避免「融合了又分话题」的假象。
- 调试两路原始点：`enable_lidar_merge: false` + `is_topic_name_with_lidar_ip: true`。此时 **不要** 把 LIO 订到未加 IP 后缀的 `/mid360_driver/lidar`（那条不会发）。

### 5.5 LIO / ROG 侧要一起改的量

点云大约 ×2，去畸变和 ROG 射线更重。

- `space_downsample_leaf_size`（现 0.5）可先不动，点多时优先靠体素而不是把 `point_filter_num` 加大到丢结构。
- ROG OpenMP：双雷达后更不要开宽 team，`[ROG WARN] Unfinished frame` 仍是调度问题。CSV 在容器 `/tmp/rog_map_perf_*.csv`。
- `blind_center` / `min_distance`：后雷达会看到车尾、天线；盲区仍以前雷达系的车体球为准，半径不够会把车体当障碍。
- 先验 PCD：FOV 变了必须 **重新绕场** `/map_save` → `save_pcd_and_make_map.sh`。旧 `lab3.pcd` 是单雷达几何，GICP 重叠会变差。不要对旧图乘一个「后雷达外参」凑合。

下游 **不用改话题** 的：

- `odom_localizer` `registered_cloud: /cloud_registered`
- `MincoPlanner.rog_map` `cloud_topic: /cloud_registered`
- costmap、`fake_vel_transform`、BT、航点 CSV

---

## 6. 和两个开源方案的差异（避免搬错）

| | 本仓库目标 | navi_minco InternalLidarMerger | HERO lidar_merge |
|---|---|---|---|
| 融合位置 | `mid360_driver` 内 | `livox_ros_driver2` 内 | 独立节点（主 CMake 未定义 `TWO_LIDARS`，提交态是单雷达） |
| 输出类型 | PointCloud2 绝对 timestamp | 默认 CustomMsg `offset_time`；PointCloud2 写成相对 offset | Livox CustomMsg，rebase 到较晚 timebase |
| 时间门限 | 5 ms（有 PTP） | 5 ms + PTP 脚本 | ApproximateTime queue 5，无硬 5 ms |
| 半帧 | 不发 | 不发 | 同步器行为，可能不等 |
| IMU | 仅前雷达 | 多话题，Point-LIO 订前雷达 IMU | 不融合 IMU |
| 去畸变 | small_point_lio | Point-LIO | `hero_lidar_scan` + Odin1 高频里程计 |
| 外参 | 一份 `merge_extrinsic_back_to_front` | 0.4 m 基线复制了五处 | `cfg.yaml` lidar1→lidar2 或都→odin1 |

---

## 7. 落地顺序

代码已合入，默认关融合。按这个顺序打开，不要在 `enable_lidar_merge: false` 且 `is_topic_name_with_lidar_ip: false` 时直接插第二台。

### 阶段 A — 线与 IP

1. 千兆交换机：NUC + 两台 MID360。
2. NUC 该口静态 `192.168.1.50`。
3. 两台雷达设不同 IP，Host 都指向 `192.168.1.50`。
4. 暂不改驱动。`is_topic_name_with_lidar_ip: true` 启动，确认：

```bash
ros2 topic list | grep mid360_driver
# 期望类似：
# /mid360_driver/lidar_192_168_1_xxx
# /mid360_driver/lidar_192_168_1_yyy
# /mid360_driver/imu_192_168_1_xxx
# /mid360_driver/imu_192_168_1_yyy
ros2 topic hz /mid360_driver/lidar_192_168_1_xxx /mid360_driver/lidar_192_168_1_yyy
```

两路都接近 10–20 Hz（由 `lidar_publish_time_interval` 和设备决定）再往下。这一阶段 **LIO 不要订融合话题**。

### 阶段 B — PTP

1. `ethtool -T` 确认 HW timestamping。
2. 宿主机起 `ptp4l` / `phc2sys`；雷达切到 PTP。
3. 看驱动是否还在给这两个 IP 写 `delta_time_map`（`NO_SYNC`）。应为 PTP 路径。
4. 分话题下对比两帧 header stamp，稳态应在数毫秒内。

没有 HW TS：记下，融合窗口改宽，不要假装 5 ms。

### 阶段 C — 外参

1. 量前后雷达相对车体：停导航，车放水平地面，跑 `bash mas2027_nav_bringup/scripts/measure_lidar_mount.sh`（先静止测倾角，提示后再转云台测半径）。把打印出来的两个关节、`merge_extrinsic_back_to_front`、`blind_center`、`gravity` 写回 URDF / YAML。
2. 外参已按前后对称写入 YAML 与 URDF。RViz 里看两路墙是否重合；有偏差再微调 `merge_extrinsic_back_to_front`，并保持与 `lidar_back_joint` 一致。
3. 调试话题下用 RViz 看两路；把后雷达点按 T 变到 `lidar_link`，墙应对齐，车体点应能被 `blind_center` 球罩住。

### 阶段 D — 驱动融合（代码改动，见 §8）

1. 实现 §5 的配对与变换。
2. `enable_lidar_merge: true`，LIO 仍订 `/mid360_driver/lidar` + 前雷达 IMU。
3. `ros2 topic hz /mid360_driver/lidar` 接近单雷达频率；`width` 约为两路之和。
4. 静止 LIO；再低速转一圈看 `/cloud_registered` 有没有「重影墙」。

### 阶段 E — 导航与地图

1. 重标 `blind_center` / `min_distance` / 如有必要 `gravity`。
2. 重新 `/map_save` + `save_pcd_and_make_map.sh`，三处路径一起换（`odom_localizer.prior_pcd_file`、ROG `prior_map`、launch `map:=`）。
3. 再跑 Nav2 / MINCO。点云 ×2 若 ROG 掉帧，先减 OpenMP 而不是关融合。

---

## 8. 代码改了哪里

| 文件 | 改什么 |
|---|---|
| `mas2027_perception/mid360_driver/src/mid360_driver_node.cpp` | 按 IP 分队列；merge 定时器；后雷达变换；只发前雷达 IMU |
| `mas2027_perception/mid360_driver/include/mid360_driver/mid360_driver_node.hpp` | 队列、外参矩阵、参数 |
| `mas2027_nav_bringup/config/small_point_lio_params.yaml` | §5.4 参数；`blind_center` / `min_distance`（近处障碍后续见 §11） |
| `mas2027_perception/mid360_driver/config/params.yaml` | 同样的默认值，避免有人直接 launch 包内文件 |
| `mas2027_robot_description/urdf/mas2027_sentry.urdf` | `lidar_back_link` |
| `docs/data-flow.md` / `docs/tf.md` | 总图改成双雷达；TF 树加后雷达 link |
| （可选）`scripts/ptp_sync.bash` | 从 navi 改编，iface 可配，**不要** `killall -9` |

不改：`custom_mid360_driver.h` 的绝对时间语义、`MincoPlanner.rog_map.cloud_topic`、行为树 `planner_id`。`odom_localizer` 后来加了 `lock_z`，见 §11。

`publish_pointcloud` 现有的 `frame_time_span` 门槛对融合帧过严时，把「跨度检查」留在单雷达入队处，融合出口只检查「配对成功 + 点坐标有限」。

---

## 9. 验收与排查

### 9.1 硬件 / 时间

```bash
ip -br a | grep 192.168.1
ros2 topic hz /mid360_driver/lidar /mid360_driver/imu /Odometry /cloud_registered
```

| 期望 | 失败时 |
|---|---|
| 网卡有 `192.168.1.50` | bind 失败，所有话题 0 |
| 融合点云 hz ≈ 单雷达 | 窗口太紧或 PTP 没锁：日志 `dropped stale` |
| 点数 ≈ 两路之和 | 后雷达 IP 填错；或半帧逻辑被破坏 |
| IMU hz 仍是一台的量级 | 若翻倍，后雷达 IMU 漏进了同一话题 |
| `/cloud_registered` 无双边重影 | 外参错或时间差过大 |

### 9.2 LIO 特有

`Preprocess` 会丢 `timestamp < last_timestamp_lidar` 的点。融合后若后雷达钟慢一截，RViz 里像「只有前雷达」。先看分话题 stamp，再看 PTP。

两路 IMU 混进一条：`last_timestamp_imu` 来回跳，滤波炸掉。融合路径必须按 `merge_front_ip` 过滤 IMU。

### 9.3 导航

先验重叠变差、GICP 锁不上：先换双雷达重录的 PCD，不要先调 GICP 阈值。车体当障碍：加大 `min_distance` 或重算 `blind_center`。近处同身高点不是障碍、车被抬高、ESDF 闪：见 §11。ROG 掉帧：OpenMP team 与 `/tmp/rog_map_perf_*.csv`。

---

## 10. 和现有文档的关系

| 文档 | 双雷达后怎么读 |
|---|---|
| [数据链路](data-flow.md) | 总图第一段从「一台 MID360」变成「两台 → 驱动内融合 → 同一对话题」 |
| [TF 链路](tf.md) | 多 `lidar_back_link`；LIO 仍只认 `lidar_link`；`map→odom` 的 z 可被 `lock_z` 钉住 |
| [组件原理](components.md) | `mid360_driver` 增加配对/外参职责，仍不去畸变 |
| [排查](troubleshooting.md) | 无点云时先分话题确认两路 UDP，再查 PTP 与 merge 窗口；近处漏障 / 车被抬高见 §3 和本文 §11 |
| 根 README 离线地图 | 换雷达几何后必须重录 PCD/PGM |

---

## 11. 近处同身高障碍与 z — 修改清单（2026-09-10）

单雷达时近处纸箱能进占用。双雷达开起来后：近处和车体同高的点进不了障碍、ESDF 断断续续、RViz 里车模明显高于 `lab3.pcd`。雷达 mesh 本身比地面高 0.21 m 是安装，不是定位错了。真正要修的是投影窗口、薄柱分类、盲区球、以及 GICP 在 z 上对旧单雷达先验的偏置。

### 11.1 原因（不要混成一件事）

| 现象 | 根因 | 不在这条链上 |
|---|---|---|
| 近处同身高点不是障碍 | ProjectionLayer：`height_delta ≤ 0.1` 的薄占据带判 `THIN_SURFACE` / PASSABLE；双雷达 ±48° roll 正好在 IMU 高度打出这种薄带 | 雷达 mesh 高过 PGM（那是 URDF `z=0.11`） |
| odom z 漂高后近处点整段消失 | `scan_z_*` 以前是 odom **绝对**坐标；默认 `scan_z_max_abs: 0.75`，车被抬高后窗口切掉车体高度带 | ROG `frame_id` 是 `odom`，不读 `map→odom` |
| 车相对 `lab3.pcd` 往上漂 | 双雷达 live 云对单雷达 prior，GICP 在 z 上有偏；`map→odom` 把整车抬起来 | 重定位 xy 对上了也不代表 z 可信 |
| 左雷达把车体点送进 LIO | `blind_center` 还是旧 URDF `xyz="0.09 0.03 0.11" rpy="… 1.5708"`，球心偏了约 7 cm | `min_distance: 0.38` 半径本身够 |
| `[ROG WARN] Unfinished frame cnt > 1` 更明显 | 点大约 ×2，20 ms 预算更紧；CSV / 宽 OpenMP 仍是调度问题 | 不是这次投影分类的回归 |

### 11.2 已改文件

| 文件 | 改什么 | 要不要重编 |
|---|---|---|
| `mas2027_perception/rog_map/include/rog_map/rog_map_core/config.hpp` | 新参数 `projection.scan_z_relative_to_robot`、`projection.surface_max_height_below_robot`；校验文案 | 是，`rog_map` |
| `mas2027_perception/rog_map/include/rog_map/projection_layer.hpp` | `ProjectionLayerConfig` 增加 `surface_max_height_below_robot`、`robot_z` | 是 |
| `mas2027_perception/rog_map/src/rog_map/projection_layer.cpp` | 薄柱：`occupied_z_max` 高于 `robot_z - surface_max_height_below_robot` → `OCCUPIED` / `AMBIGUOUS_OCCUPIED`，不再 PASSABLE | 是 |
| `mas2027_perception/rog_map/src/rog_map/rog_map.cpp` | `refreshLayers()`：相对 `/Odometry` z 算 `scan_z`；把 `robot_z` 传进投影层 | 是 |
| `mas2027_nav_bringup/config/nav2_params.yaml` | 见 §11.3 | 否 |
| `mas2027_nav_bringup/config/small_point_lio_params.yaml` | `blind_center` 换成当前 URDF | 否 |
| `mas2027_perception/Localization/odom_localizer/include/odom_localizer/odom_localizer_node.hpp` | `maybe_lock_z()`、`lock_z_` | 是，`odom_localizer` |
| `mas2027_perception/Localization/odom_localizer/src/odom_localizer_node.cpp` | `update.lock_z`；初始化与每次 GICP 更新都钉 z | 是 |
| `mas2027_perception/Localization/odom_localizer/config/params.yaml` | `update.lock_z: true` | 否 |
| `mas2027_nav_bringup/scripts/measure_lidar_mount.py` | `JOINT_XYZ`、`blind_center`、光轴改成 **只有 Rx**（旧脚本还按 `yaw=π/2` 算视场） | 否 |

容器里编这两个包（YAML 不走编译，source 即生效）：

```bash
cd /home/ros2_ws && colcon build --packages-select rog_map odom_localizer --symlink-install
```

`minco_planner` 会因 `rog_map` 头文件跟着编。只编 `rog_map` 不编 `odom_localizer` 时，`lock_z` 不会进二进制，GICP 仍会把车抬起来。

### 11.3 生效参数

`nav2_params.yaml` → `planner_server.MincoPlanner.rog_map`：

```yaml
virtual_ground_height: -1.5   # 默认 -0.80；odom z 漂高时 1.8 的 ceil 会整帧跳过
virtual_ceil_height: 3.0
projection:
  scan_z_relative_to_robot: true   # false 时 scan_z_* 仍是 odom 绝对坐标
  scan_z_min_abs: -1.2             # 相对 /Odometry z 的偏移
  scan_z_max_abs: 0.75
  surface_height_delta_max: 0.1
  surface_max_height_below_robot: 0.15  # <0 关闭；0.15 = 底盘附近薄带当墙
```

`small_point_lio_params.yaml`：

```yaml
blind_center: [ 0.000, 0.0184, -0.1851 ]  # -Rx(-α)ᵀ·t，α=0.8389，t=(0,-0.15,0.11)
```

`odom_localizer/config/params.yaml`：

```yaml
update:
  lock_z: true   # map→odom 的 z 钉在 startup.initial_transform（现为 0）
```

地面车可以一直开着 `lock_z`。重录双雷达 prior 之后也不必关：GICP 仍不该用 z 去抬整车。

### 11.4 没改、也还没做

| 项 | 为什么先不动 |
|---|---|
| LIO `Preprocess` 按雷达分水位线 | 两路自由钟会让落后那一路跨帧被砍；要改 `last_timestamp_dense_point`，不是 YAML |
| 双射线原点 | ROG 仍从 `base_link` 单点 raycast；近处漏障这次是分类/窗口，不是射线原点 |
| 关 perf CSV | 能减 `[ROG WARN] Unfinished frame`，和近处漏障不是同一条 |
| 重录 `lab3.pcd` | **还要在真车上做**：`/map_save` → `save_pcd_and_make_map.sh`。现在的 prior 仍是单雷达几何 |
| merge PTP / `merge_max_interval_ms: 5` | 现车无 PTP，窗口仍是 50 ms |

### 11.5 上机核对

1. 车前同身高纸箱进 `/rog_map/layer_value`（OCCUPIED）和 ESDF，不是一闪而过的薄面。
2. `ros2 run tf2_ros tf2_echo map odom`：translation z 钉在 0 附近；日志有 `(z locked)`。
3. RViz 车模相对 PGM 的高度只剩 URDF 那 0.11/0.21 m，不再整车腾空。
4. `Unfinished frame cnt > 1` 会少一些；双雷达点数大约翻倍，这条不必清零。
