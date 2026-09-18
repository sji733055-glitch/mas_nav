# Foxglove 远程可视化与调试

在另一台机器上通过 SSH 远程使用 Foxglove 观察/调试本仓库 nav 栈的操作手册。

现场环境（2026-09-17 实测）：

- 机器人：Ubuntu 24.04 + ROS 2 Jazzy，主机名 `mas-intel-1`，用户 `mas`
  - `wlo1`：`192.168.77.79`（SSH 从这里进来）
  - `enp2s0`：`192.168.1.50`
  - 机器上装了 `FlClash`（TUN，`198.18.0.0/30`）
- 上位机/笔记本：`192.168.77.15`（与机器人同一网段）
- 机器人已装 `ros-jazzy-foxglove-bridge 3.5.0` 与 Foxglove Studio 桌面版 `3.1.1`

## 1. 链路

```text
笔记本 Foxglove ──ws://127.0.0.1:8765──▶ SSH 隧道 ──▶ 机器人 127.0.0.1:8765
                                                        foxglove_bridge
                                                              │ DDS
                                                              ▼
                                        nav_executor / ROGMap / map_server / ...
```

`foxglove_bridge` 与 nav 栈是**两条独立的 launch**，互不依赖：bridge 只负责把 DDS 图
暴露成 WebSocket，nav 栈没起时 Foxglove 里能看到连接但没数据。

## 2. 机器人端

### 起 bridge

```bash
# 方式 A：只绑回环 + SSH 隧道（推荐：端口不外露，不用动防火墙）
ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=127.0.0.1 port:=8765

# 方式 B：直连（笔记本装桌面版 Foxglove，直接连机器人 IP）
ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=0.0.0.0 port:=8765
```

两种方式**不能同时起**，会 `Failed to bind port: Address already in use (os error 98)`。

确认活着：

```bash
ss -tln | grep 8765        # 看到 127.0.0.1:8765 或 0.0.0.0:8765
```

正常启动日志：

```text
[foxglove_bridge-1] [INFO] [foxglove_bridge]: Server listening on port 8765
[foxglove_bridge-1] [INFO] [foxglove_bridge]: Advertising new channel 46 for topic "/cmd_spin"
```

停止：

```bash
pkill -f foxglove_bridge
```

### 起 nav 栈

```bash
source install/setup.bash
ros2 launch mas2027_nav_bringup nav_executor_launch.py
```

### 限流白名单（点云太多、走 WiFi 卡时用）

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=127.0.0.1 port:=8765 \
  topic_whitelist:="['^/cloud_registered$','^/opt_path_vis$','^/Odometry$','^/tf.*']"
```

## 3. 笔记本端

### 方式 A：SSH 隧道（配合 `address:=127.0.0.1`）

```bash
ssh -N -L 8765:127.0.0.1:8765 mas@192.168.77.79
```

这条命令要一直挂着。建议写进 `~/.ssh/config` 免得每次敲：

```sshconfig
Host mas-robot
    HostName 192.168.77.79
    User mas
    LocalForward 8765 127.0.0.1:8765
    ServerAliveInterval 30
    ServerAliveCountMax 3
```

之后 `ssh -N mas-robot` 即可。

### 方式 B：直连（配合 `address:=0.0.0.0`）

Foxglove 桌面版直接填 `ws://192.168.77.79:8765`。

### 连接

Foxglove → `Open connection` → 选 **Foxglove WebSocket** → 填 URL → `Open`。

- 方式 A 填 `ws://127.0.0.1:8765`
- 方式 B 填 `ws://192.168.77.79:8765`

## 4. 面板配置

3D 面板的 **Fixed frame 填 `map`**（TF 链是 `map → odom → base_link → lidar_link`，
由 `tf_maintainer` 统一发布）。若 `map` 帧不存在，先查 TF，否则 3D 里基本没东西可看。

| 面板类型 | Topic | 看什么 |
|---|---|---|
| TF | — | `map→odom→base_link` 是否通 |
| PointCloud2 | `/cloud_registered` | 实时配准点云（首选） |
| PointCloud2 | `/rog_map/occupied` | ROGMap 动态占据 |
| PointCloud2 | `/rog_map/field` | 2D 距离场 |
| OccupancyGrid | `/cost_map`、`/dynamic_cost_map` | 代价图 |
| Path | `/opt_path_vis` | 实际执行的轨迹 |
| Path | `/nav_executor/global_plan` | 全局路径 |
| Path | `/backup_path_vis`、`/minco_candidate_path_vis` | 备份/候选轨迹 |
| MarkerArray | `/nav_executor/debug/safe_corridor` | MINCO 安全走廊 |
| Marker | `/nav_executor/debug/dynamic_obstacles` | 当前帧动态障碍 |
| Odometry | `/Odometry` | 位姿 |
| Image | `/direction_map` | 地形方向图 |

Plot 面板排查"慢/卡顿"最直观：`/cmd_vel` 的 `twist.linear.x` / `twist.angular.z`，
`/Odometry` 的 `twist.twist.linear.x`。

`Layout → Save` 可把布局导出成 json，下次直接导入，不用重配。

## 5. 已知坑

### 5.1 握手子协议改了（本次卡最久的一条）

`foxglove_bridge 3.5.0` 基于新版 Rust `foxglove-sdk`，WebSocket 握手**只接受子协议
`foxglove.sdk.v1`**，不再接受老名字 `foxglove.websocket.v1`。旧客户端只发
`foxglove.websocket.v1` 时会被 400 掉，服务端日志是：

```text
ERROR foxglove::websocket::server] Dropping client 127.0.0.1:58700: handshake failed
```

HTTP 响应体是 `Missing expected sec-websocket-protocol header`（该字符串在
`/opt/ros/jazzy/lib/libfoxglove.so` 里）。

实测（向 8765 发原始握手请求）：

| 客户端声明的子协议 | 结果 |
|---|---|
| `foxglove.websocket.v1` | `400 Bad Request` |
| `foxglove.sdk.v1` | `101 Switching Protocols` |
| 两个都声明 | `101`，服务端选 `foxglove.sdk.v1` |

**所以客户端必须够新。** Foxglove Studio 3.1.1 的客户端代码是
`SUPPORTED_SUBPROTOCOLS=["foxglove.websocket.v1","foxglove.sdk.v1"]`，两个都认；
版本旧就升级，或改用网页版 `https://studio.foxglove.dev`。

### 5.2 绑定地址

`address:=127.0.0.1` 时只有本机能连，远端必然连不上（也不会有明显报错）。远端要连
就必须 `address:=0.0.0.0`，或者老实用 SSH 隧道。

### 5.3 网页版的 mixed content

`https://studio.foxglove.dev` 是 https 页面，浏览器只允许它连 **localhost** 的
`ws://`。所以网页版**必须走 SSH 隧道**（隧道把服务映射到笔记本本机），不能填
`ws://192.168.77.79:8765`。

### 5.4 自定义消息类型在 3D 面板里加不了

`/opt_path`、`/backup_path` 是 `interfaces/msg/MpcPositionCommand`，3D 面板不认。
要画线用 `_vis` 后缀的 `nav_msgs/Path`（`/opt_path_vis`、`/backup_path_vis`），
要看数值用 Raw Messages 面板。

### 5.5 带宽

点云很吃带宽。优先 `/cloud_registered`，别一上来开 `/cloud_registered_full`；
`/rog_map/*` 那六七个层全开，走 WiFi 会明显卡。必要时用 `topic_whitelist` 限流。

### 5.6 代理

机器人上有 `FlClash`（TUN 模式）。如果笔记本也开代理，记得把 `localhost` / `127.0.0.1`
加进 bypass，否则隧道到本机的那段可能被代理截走。

### 5.7 防火墙

机器人 `/etc/ufw/ufw.conf` 里是 `ENABLED=no`，所以方式 B 直连不需要额外放行。
若哪天启用了 ufw，直连需要：

```bash
sudo ufw allow from 192.168.77.0/24 to any port 8765 proto tcp
```

### 5.8 SSH 转发

`sshd_config` 没有显式设 `AllowTcpForwarding`（默认 `yes`），`X11Forwarding yes`。
隧道方案开箱可用。

## 6. 排障速查

| 现象 | 查什么 |
|---|---|
| Foxglove 连不上、无报错 | `ss -tln \| grep 8765`；是不是绑了 `127.0.0.1` 又没开隧道 |
| 连上立刻断、日志 `handshake failed` | 客户端太旧，不认 `foxglove.sdk.v1`（见 5.1） |
| 启动报 `Address already in use` | 已有一个 bridge 在跑，`pkill -f foxglove_bridge` 后再起 |
| 连上但没数据 | nav 栈没起，或 topic 被白名单滤掉 |
| 3D 里一片空白 | Fixed frame 不是 `map`；或 TF 链缺 `map`/`odom` |
| 卡成幻灯片 | 点云太多，收窄白名单（见 5.5） |

## 7. 远程调试常用动作

- **发目标点**：3D 面板里点 `Publish` → 选 `/goal_pose` → 在地图上点一下即发布导航目标。
  （bridge 默认 capabilities 含 `clientPublish`。）
- **手动重定位**：同理可发 `/initialpose`（`geometry_msgs/PoseWithCovarianceStamped`）。
- **录包留证据**：

  ```bash
  ros2 bag record -o /tmp/debug_bag /cloud_registered /opt_path /cmd_vel /Odometry /tf
  ```
