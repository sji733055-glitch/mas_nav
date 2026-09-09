# 双 MID360 融合方案

本文是本仓库把单雷达改成前后双雷达的设计与落地说明。对照了 `navi_minco_bit` 的驱动内融合，以及 `HERO_2026_Sentry_NAV` 的独立 `lidar_merge` 节点；**本栈不换 livox 官方驱动、不搬 HERO 的 Odin1 去畸变链路**。融合发生在现有 `mid360_driver` 里，下游仍只看见一份 `/mid360_driver/lidar`。

当前仓库仍是单雷达。文中带「拟改」的 YAML / URDF 是目标态，合入代码前不要当已经生效。

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

`small_point_lio_params.yaml` 里注释还写着旧的 `0.09 0.03 0.11`，以 URDF 为准。

### 4.2 后 → 前：只有一份外参

```text
merge_extrinsic_back_to_front: [x, y, z, roll, pitch, yaw]
# 米 + 弧度，T 把后雷达光学中心的点变到前雷达光学中心
# p_front = R_back_to_front * p_back + t_back_to_front
```

**不要抄 navi 的 `[0.0, 0.4, 0.0, -0.35453, 0.0, 0.0]`。** 那是他们车 0.4 m 横向基线。本车前后雷达的基线、倾角要实测。navi 把同一 0.4 m 复制进了 merge 外参、盲区、ROG `center_offset`、`lidar_offset_y` 五处，改一处漏四处。本仓库约定：

| 量 | 来源 |
|---|---|
| 前雷达相对车体 | URDF `lidar_joint`（现有） |
| 后雷达相对前雷达 | **唯一**参数 `merge_extrinsic_back_to_front` |
| 后雷达可视化 | URDF `lidar_back_link` = 前关节 ∘ T_back_to_front 的逆（或直接写后雷达相对 base） |
| LIO `blind_center` | 仍是 **base_link 原点在前雷达 lidar_link 中的坐标**；车体变长/雷达改位后跑 `measure_lidar_mount.py` |
| `min_distance` | 盲区半径要罩住车身在前雷达系里的投影，双雷达后车体更「长」，大概率要加大 |
| IMU 外参 `extrinsic_T/R` | 仍是 **前雷达光学中心 → 前雷达内置 IMU**，一般不用因加后雷达而改 |

### 4.3 IMU 只用前雷达

- 点云：两路融合后发 `/mid360_driver/lidar`，`frame_id = lidar_link`。
- IMU：只转发前雷达 IP 的包到 `/mid360_driver/imu`，`frame_id = lidar_imu`。
- 后雷达 IMU 可丢弃；需要排查时用 `merge_keep_debug_topics: true` 另发，不进 LIO。

重力向量 `gravity`、`fix_gravity_direction` 仍按前雷达 IMU 静止标定。加装后雷达若改了前雷达倾角，要重跑 `measure_lidar_mount.py`。

### 4.4 拟改 URDF（示意）

```xml
<!-- 前雷达：现有 lidar_joint，不要改名 -->
<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.00 -0.15 0.11" rpy="0.8389 0 0"/>
</joint>

<!-- 后雷达：仅可视化 / 标定辅助，LIO 不 lookup 这个 link -->
<link name="lidar_back_link"/>
<joint name="lidar_back_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_back_link"/>
  <!-- 填本车实测，不是 navi 的 0.4 m -->
  <origin xyz="..." rpy="..."/>
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

`LidarDataQueue` 那套是 livox SDK 的，本驱动没有。用每 IP 一个 `std::deque` 的扫描帧即可。

### 5.2 配对规则（对齐 navi `InternalLidarMerger`）

1. 只认 `merge_front_ip` / `merge_back_ip`。其它源 IP 丢弃并节流打日志。
2. 两队列都非空才考虑发布；**禁止发半帧**（只有前或只有后）。
3. 队头帧时间 `t` 取该帧点时间戳的 min（与现在 `publish_pointcloud` 的 header stamp 一致）。
4. `|t_front − t_back| ≤ merge_max_interval_ms` → 合并发布，两队各 pop 一帧。
5. 超限 → 丢掉更旧的队头，节流 warning `Internal lidar merge dropped stale`。
6. 合并后 `header.stamp = min(t_front, t_back)`，`frame_id = lidar_link`。
7. 前雷达点原样写入；后雷达点先 `p' = R p + t`。
8. 每点 `timestamp` 仍写 **绝对秒**（`custom_mid360_driver.h` 直接 `new_point.timestamp = *out_timestamp`）。不要写成相对帧头的 offset。
9. 合并后按绝对时间排序再发布。LIO `Preprocess` 会在本批内再 sort，但水位线 `last_timestamp_lidar` 按上一帧最大值砍点：两路时钟若差出一帧，后雷达整帧会被静默丢掉。
10. `max_packet_time_span`（现 0.1 s）在 `publish_pointcloud` 里用 `span > 2 * max_packet_time_span` 丢整帧。未配对就把两路拼进去，时钟一偏整帧消失。融合路径应在配对成功后再组包，或把该检查改成「单雷达包」而不是「融合帧」。

### 5.3 点格式（必须保持）

```text
PointCloud2 fields:
  x,y,z          FLOAT32
  intensity      FLOAT32
  timestamp      FLOAT64   绝对秒，不是 offset_time、也不是相对 timebase
point_step = 24
```

`lidar_type: custom_mid360_driver` 不要改成 `livox_custom_msg`，除非整条驱动都换成 CustomMsg。

### 5.4 拟增参数

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

    # ---- 以下拟增，尚未合入驱动 ----
    enable_lidar_merge: true
    merge_front_ip: 192.168.1.135        # 换成车上前雷达实际 IP
    merge_back_ip: 192.168.1.122         # 换成车上后雷达实际 IP
    merge_max_interval_ms: 5.0           # 无 PTP 时不要用 5
    merge_extrinsic_back_to_front: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 本车实测
    merge_keep_debug_topics: false       # true 时额外发 /mid360_driver/lidar_<ip>
```

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

软件尚未合入时，按这个顺序做，不要跳步把两台雷达直接喂现有 `is_topic_name_with_lidar_ip: false`。

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

1. 量前后雷达相对车体（尺 + 倾角，或离线点云配准）。
2. 写出 `T_back_to_front`，填拟增参数和 URDF `lidar_back_joint`。
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

## 8. 代码改哪里（合入时对照）

| 文件 | 改什么 |
|---|---|
| `mas2027_perception/mid360_driver/src/mid360_driver_node.cpp` | 按 IP 分队列；merge 定时器；后雷达变换；只发前雷达 IMU |
| `mas2027_perception/mid360_driver/include/mid360_driver/mid360_driver_node.hpp` | 队列、外参矩阵、参数 |
| `mas2027_nav_bringup/config/small_point_lio_params.yaml` | §5.4 参数；`blind_center` / `min_distance` |
| `mas2027_perception/mid360_driver/config/params.yaml` | 同样的默认值，避免有人直接 launch 包内文件 |
| `mas2027_robot_description/urdf/mas2027_sentry.urdf` | `lidar_back_link` |
| `docs/data-flow.md` / `docs/tf.md` | 总图改成双雷达；TF 树加后雷达 link |
| （可选）`scripts/ptp_sync.bash` | 从 navi 改编，iface 可配，**不要** `killall -9` |

不改：`custom_mid360_driver.h` 的绝对时间语义、`odom_localizer`、`MincoPlanner.rog_map.cloud_topic`、行为树 `planner_id`。

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

先验重叠变差、GICP 锁不上：先换双雷达重录的 PCD，不要先调 GICP 阈值。车体当障碍：加大 `min_distance` 或重算 `blind_center`。ROG 掉帧：OpenMP team 与 `/tmp/rog_map_perf_*.csv`。

---

## 10. 和现有文档的关系

| 文档 | 双雷达后怎么读 |
|---|---|
| [数据链路](data-flow.md) | 总图第一段从「一台 MID360」变成「两台 → 驱动内融合 → 同一对话题」 |
| [TF 链路](tf.md) | 多 `lidar_back_link`；LIO 仍只认 `lidar_link` |
| [组件原理](components.md) | `mid360_driver` 增加配对/外参职责，仍不去畸变 |
| [排查](troubleshooting.md) | 无点云时先分话题确认两路 UDP，再查 PTP 与 merge 窗口 |
| 根 README 离线地图 | 换雷达几何后必须重录 PCD/PGM |
