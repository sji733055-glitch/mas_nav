# mas_nav_2027

在线导航的学习文档（数据链路、TF、组件原理、读代码顺序、排查）见 [`docs/README.md`](docs/README.md)。

# 开发环境配置

本仓库在 **Ubuntu 24.04 + ROS 2 Jazzy** 上原生编译运行，不再走 Docker。

## 环境配置

### 1. 安装 ROS 2 Jazzy 基础环境

如果系统还没有 ROS 2 Jazzy，先按照官方文档安装 `ros-jazzy-desktop`，然后确认以下文件存在：

```bash
test -f /opt/ros/jazzy/setup.bash
```

每个新终端先加载 ROS 环境：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 2. 安装工程依赖

需要本机 sudo 密码：

```bash
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-behaviortree-cpp \
  ros-jazzy-rmw-cyclonedds-cpp \
  libdw-dev \
  libomp-dev \
  python3-pip
```

本机已有 `ros-jazzy-desktop` 时，上面只补 Nav2 / BT.CPP / CycloneDDS / 编译库。

### 3. 初始化 rosdep

新机器只需要执行一次。若 `rosdep init` 提示已经初始化，直接执行 `rosdep update` 即可：

```bash
sudo rosdep init
rosdep update
```

### 4. 编译工作区

仓库根目录就是 colcon 工作区，源码位于 `src/`，构建产物位于根目录的 `build/`、`install/` 和 `log/`：

```bash
cd /home/mas/mas_nav_2027_native
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
rosdep install -r --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers 4
source install/setup.bash
```

离线建图工具在 `mas2027_utils/`（`pcd2pgm`、`pcd2ele`、`pcd2esdf`、`pcd_trans`），随 colcon 一起编。行为树编辑器是 `mas2027_utils/bt_editor/bt_editor.html`（浏览器打开，导入 `mas2027_nav_bringup/behavior_trees/*.xml`）。跟踪对照是 `mas2027_utils/data_analyzer/`，不进主 launch。

### 5. 运行

```bash
source /opt/ros/jazzy/setup.bash
source /home/mas/mas_nav_2027_native/install/setup.bash
ros2 launch mas2027_nav_bringup rm_navigation_small_point_lio_launch.py
```

## 离线静态地图：点云 → 2D 栅格 + 3D 先验

运行时只需两份事先，放在 `mas2027_nav_bringup/` 里：

- **3D** `pcd/<name>.pcd` → `odom_localizer` GICP（`map→odom`）
- **2D** `map/<name>.pgm` + `.yaml` → `map_server` / StaticLayer / ROG `prior_map`

当前实验室图是 `pcd/lab3.pcd` + `map/lab3.{pgm,yaml}`。PCD 和 PGM 同系（建图时 map == 那次 odom），不要再乘 `T_base_lidar`。

```text
LIO 绕场 + /map_save
  → T_odom_from_internal 转到 odom（一键脚本默认做）
  → 写出 mas2027_nav_bringup/pcd/<name>.pcd
  → 可选 pcd_trans（只平移/旋转场地原点）
  → pcd2pgm（Z 带切片 → OccupancyGrid）
  → map_edit（RViz 擦墙、补洞）
  → 保存 pgm/yaml；把同一份 PCD 指给 odom_localizer
```

可选：`pcd2ele` 出高程灰度（**不能**当 Nav2 占用图）；`pcd2esdf` 从 PGM 烘焙 2D 距离场（运行时 MINCO 仍用 ROG 在线 ESDF）。

下面命令都在 **仓库根目录**，先 source overlay。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 一键：保存点云并切二维图

绕场之前把 `mas2027_nav_bringup/config/small_point_lio_params.yaml` 的 `save_pcd` 设成 `true`，只开定位：

```bash
ros2 launch mas2027_nav_bringup rm_navigation_small_point_lio_launch.py \
  use_nav2:=False use_rviz:=True use_odom_localizer:=False
```

绕完两圈，**另开终端**（LIO 继续跑）：

```bash
bash mas2027_nav_bringup/scripts/save_pcd_and_make_map.sh lab3
```

会调用 `/map_save`、用它写出的 `scan_T_odom_from_internal.txt` 把内部世界系转到 `/cloud_registered`（odom）再写入 `pcd/lab3.pcd`，然后 `pcd2pgm` + `map_saver_cli` 写出 `map/lab3.{pgm,yaml}`。已有文件会先备份成 `.bak.时间戳`。切片走独立话题 `/pcd2pgm_map`，不和 Nav2 的 `/map` 抢。只要内部世界、不要扶正：加 `--no-align`。

已经有 PCD、只想重切：

```bash
bash mas2027_nav_bringup/scripts/save_pcd_and_make_map.sh lab3 --from-pcd
# 挡墙留不住 / 地面太厚：
#   --z-min 0.05 --z-max 1.5 --resolution 0.05
```

这一步**会**做扶正（与直播 `/cloud_registered` 同系），**不**开 `map_edit`、也不改 `odom_localizer` / ROG `prior_map` 路径。墙要擦、场地原点要平移、换场地改三处路径，仍用下面分步。用完把 `save_pcd` 改回 `false`。

### 1. 录 LIO 点云并保存 PCD

终端 1，只开定位和点云，不要 Nav2。建图时把 `mas2027_nav_bringup/config/small_point_lio_params.yaml` 的 `save_pcd` 设成 `true`（用完改回 `false`，很占内存）：

```bash
ros2 launch mas2027_nav_bringup rm_navigation_small_point_lio_launch.py \
  use_nav2:=False use_rviz:=True use_odom_localizer:=False
```

绕场两圈后：

```bash
ros2 service call /map_save std_srvs/srv/Trigger
```

写出 `mas2027_perception/Odometry/small_point_lio/pcd/scan.pcd`。拷到 bringup 并改名（换场地就把 `lab3` 换成场地名）：

```bash
cp mas2027_perception/Odometry/small_point_lio/pcd/scan.pcd \
  mas2027_nav_bringup/pcd/lab3.pcd
```

现成的 `lab3.pcd` 已经和 `/cloud_registered` 同系。一键脚本写出的 `pcd/<name>.pcd` 也会乘上 `/map_save` 的 `T_odom_from_internal`，不要再乘 `T_base_lidar`。只拷内部世界、不扶正时才加 `--no-align`。

可选录 bag（QoS 必须覆盖，否则点云 0 条）：

```bash
bash mas2027_nav_bringup/scripts/record_lio_bag.sh \
  /tmp/lio_lab
ros2 bag info /tmp/lio_lab
```

### 2. 点云变换（pcd_trans，可选）

一键脚本已经用 LIO 的扶正矩阵把先验转到 odom。这里只在**场地原点**要对齐（平移/绕 z 转）时用：

```bash
python3 mas2027_utils/pcd_trans/pcd_tool.py --info \
  --in mas2027_nav_bringup/pcd/lab3.pcd

python3 mas2027_utils/pcd_trans/pcd_tool.py \
  --in mas2027_nav_bringup/pcd/lab3.pcd \
  --out mas2027_nav_bringup/pcd/lab3.pcd \
  --tx 0 --ty 0 --tz 0 --yaw 0
```

浏览器里对齐点云和 PGM：打开 `mas2027_utils/pcd_trans/visual_point_cloud_lab.html`。先选 `map/lab3.yaml` 再选 `lab3.pgm`，点云选 `pcd/lab3.pcd`（压缩 PCD 现在能读）。雷达外参矩阵：`python3 mas2027_utils/rotmat_cal/cal_rotmat.py`。

### 3. 点云转二维栅格（pcd2pgm）

`pcd2pgm` 走 colcon。改 `mas2027_utils/pcd2pgm/config/pcd.yaml`，或启动时覆盖参数。`file_directory` 必须以 `/` 结尾；`file_name` 不含 `.pcd`。

```yaml
/pcd2pgm:
  ros__parameters:
    file_directory: mas2027_nav_bringup/pcd/
    file_name: lab3
    flag_pass_through: false   # false = 保留 [thre_z_min, thre_z_max] 之间的点
    map_resolution: 0.05       # 与现有 Nav2 地图一致
    map_topic_name: map
    thre_radius: 0.5
    thre_z_max: 1.5            # 雷达原点 z-up；挡墙留不住就提高，地面太厚就提高 min
    thre_z_min: 0.05
    thres_point_count: 10
    use_sim_time: false
```

启动（launch 与 yaml 都是 `use_sim_time:=false`）：

```bash
ros2 launch pcd2pgm pcd2pgm.launch.py
```

或直接覆盖路径：

```bash
ros2 run pcd2pgm pcd2pgm_node --ros-args \
  -p use_sim_time:=false \
  -p file_directory:=mas2027_nav_bringup/pcd/ \
  -p file_name:=lab3 \
  -p map_topic_name:=map \
  -p map_resolution:=0.05 \
  -p thre_z_min:=0.05 \
  -p thre_z_max:=1.5 \
  -p flag_pass_through:=false
```

节点每秒发布 `/map`（transient_local）。RViz 加 Map 显示 `/map` 看切片。高度带不对就改 `thre_z_*` 再启动。满意后另开终端保存：

```bash
ros2 run nav2_map_server map_saver_cli \
  -f mas2027_nav_bringup/map/lab3
```

会生成 `lab3.pgm` + `lab3.yaml`。

### 4. Map Editor 修图

`map_edit` 也是 colcon 包。启动自带面板和橡皮擦的 RViz：

```bash
ros2 launch map_edit map_edit.launch.py
```

**打开地图（二选一）**

- 本地文件：左侧 `MapEditPanel` →「选择本地地图文件」→ 选上一步的 `lab3.yaml`。
- 边转边修：先开着 `pcd2pgm`（它在发 `/map`），插件会订阅 `/map`，改完的图画在 `/map_edit`，**不会**改 Nav2 正在用的图。

**橡皮擦（工具栏选 MapEraserTool）**

- 左键画黑（障碍），右键画白（可通行），按住拖是连续画
- ↑ / ↓ 调笔刷大小
- Ctrl+Z 撤销
- Shift + 左/右键：两点连成直线

**保存**

点「保存到本地」，选目录。会写出 `pgm` + `yaml`。改完的图要 Nav2 重新加载 yaml 才生效。

建议保存回：

```text
mas2027_nav_bringup/map/lab3.pgm
mas2027_nav_bringup/map/lab3.yaml
```

Nav2 用地图时确认 yaml 里 `image:`、`resolution:`（0.05）、`origin:` 和实际 pgm 一致。

建图时场上有行人，切片会把拖影写成永久墙。空场可直接切；走过通道就在 `map_edit` 里擦掉，不要靠提高 `thre_z_min` 躲人（矮挡墙会一起没）。

### 4b. 在先验图上点航点

`map_edit` 只修墙。航点用 `mas2027_utils/waypoint_editor`（RViz 插件）。导航不要同时开它自带的 `map_server`：

```bash
ros2 launch waypoint_editor waypoint_editor.launch.py
```

工具栏 **Add Waypoint**，Save WPs 存 CSV（建议 `mas2027_nav_bringup/config/lab3_patrol.csv`）。跑线：

```bash
ros2 launch mas2027_nav_bringup waypoint_navigator.launch.py \
  waypoint_file:=mas2027_nav_bringup/config/lab3_patrol.csv
```

逐点 `navigate_to_pose`；重复的 `waypoint_to_nav2` / `waypoint_through_nav2` 已删除。主导航 RViz 不加载编辑器，在线编辑需另启 `waypoint_editor.launch.py use_map_server:=false`。详情见 `mas2027_utils/waypoint_editor/README.md`。

### 5. 接到导航（三处一起改）

换图时 `map:=` **只动** `map_server`。另外两处要手改成同一套名字：

| 消费者 | 文件 | 当前 lab3 |
|---|---|---|
| `odom_localizer` | `mas2027_perception/Localization/odom_localizer/config/params.yaml` → `map.prior_pcd_file` | `.../pcd/lab3.pcd` |
| ROG `prior_map` | `mas2027_nav_bringup/config/nav2_params.yaml` → `projection.prior_map.yaml_path` / `pgm_path` | `.../map/lab3.yaml` |
| Nav2 `map_server` | launch 参数 `map:=` | 同上 yaml |

`prior_map.frame_id` 必须是 `map`。

### 6. 可选：高程图 / 离线 ESDF

高程（格子取 max-z，**不要**喂给 `map_server`）：

```bash
ros2 launch pcd2ele pcd2ele.launch.py
```

launch 默认 `use_sim_time:=false`。

配置在 `mas2027_utils/pcd2ele/config/pcd2ele.yaml`，默认读 `pcd/lab3.pcd`，写出 `map/lab3_elevation.{pgm,yaml}`。

从占用 PGM 烘焙 2D 有符号距离场（调试用，MINCO 仍查 ROG）：

```bash
ros2 launch pcd2esdf pcd2esdf.launch.py
```

默认读 `map/lab3.yaml`，写出 `pcd/lab3_esdf.pcd`，并往 `/esdf_result` 发一份。
