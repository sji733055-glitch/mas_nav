# mas_nav_2027

在线导航的学习文档（数据链路、TF、组件原理、读代码顺序、排查）见 [`docs/README.md`](docs/README.md)。

# 开发环境配置
1. docker 安装
```bash
// docker 安装脚本
export DOWNLOAD_URL="https://mirrors.tuna.tsinghua.edu.cn/docker-ce"
wget -O- https://raw.githubusercontent.com/docker/docker-install/master/install.sh | sh
// 更换为国内docker镜像源
bash <(wget -qO- https://xuanyuan.cloud/docker.sh)
// 权限处理
sudo usermod -aG docker mas
```
2. 构建容器
```bash
sudo docker compose up -d --build
```
3. 编译
```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers 4
```

`mas2027_utils/ERASOR2` 带 `COLCON_IGNORE`，不会进 colcon。干净点云那一步单独用 CMake 编，见下文。

4. 在宿主机内使用解决X11授权
```bash
xhost +si:localuser:root
```

## SLAM Toolbox 二维建图

建图链路使用 Small Point-LIO 的 `/Odometry` 和 `/cloud_registered`：

```text
/cloud_registered -> pointcloud_to_laserscan -> /scan -> slam_toolbox
/Odometry + TF: map -> odom -> base_link
```

启动建图：

```bash
ros2 launch mas2027_nav_bringup mapping_launch.py
```

建图期间不要启动其他 `map -> odom` 发布器。检查数据：

```bash
ros2 topic hz /scan
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map odom
```

保存 Nav2 使用的二维地图：

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /home/ros2_ws/src/mas2027_nav_bringup/map/field
```

同时保存 SLAM Toolbox 位姿图，供后续 localization 模式使用：

```bash
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/ros2_ws/src/mas2027_nav_bringup/map/field'}"
```

点云切片高度、量程、闭环与分辨率参数位于
`mas2027_nav_bringup/config/slam_toolbox_mapping.yaml`。高度参数以
`base_link` 为参考，平地出现在扫描中时提高 `min_height`，低矮障碍消失时降低。

## 离线静态地图：干净点云 → 二维栅格 → Map Editor

线上 `slam_toolbox` 之外，还可以用 Small Point-LIO 的 bag 做一条**离线**链路：

```text
录 /cloud_registered + /Odometry
  → export_lio_bag.py（逐帧 .bin + T_map_lidar）
  → kitti_clustering.py（地面 + 实例标签）
  → ERASOR2 mapgen / run_erasor2（去掉动态物体，得到全局 PCD）
  → pcd2pgm（高度切片 → OccupancyGrid）
  → map_edit（RViz 里擦墙、补洞）
  → 保存 pgm + yaml 给 Nav2
```

不要用 `/map_save` 吐出来的 `scan.pcd`：那是 LIO 内部合成云，没有逐帧位姿，ERASOR2 吃不了。

下面命令都在 **mas_nav 容器**里、工作目录 `/home/ros2_ws`。仓库挂在 `/home/ros2_ws/src`，数据请放仓库内（compose 没有挂 `/data`）：

```bash
mkdir -p /home/ros2_ws/src/data
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash   # colcon 编过之后
```

### 1. 录 LIO bag

终端 1，只开定位和点云，不要 Nav2：

```bash
ros2 launch mas2027_nav_bringup rm_navigation_small_point_lio_launch.py \
  use_nav2:=False use_rviz:=True
```

终端 2。`/cloud_registered` 是 best_effort，必须走脚本里的 QoS 覆盖，否则 bag 里点云是 0 条：

```bash
bash /home/ros2_ws/src/mas2027_utils/ERASOR2/scripts/record_lio_bag.sh \
  /home/ros2_ws/src/data/lio_rmuc
```

绕场两圈后 Ctrl-C。检查：

```bash
ros2 bag info /home/ros2_ws/src/data/lio_rmuc
```

`/cloud_registered` 和 `/Odometry` 条数都应该大于 0。

### 2. 导出关键帧

```bash
python3 /home/ros2_ws/src/mas2027_utils/ERASOR2/scripts/export_lio_bag.py \
  --bag /home/ros2_ws/src/data/lio_rmuc \
  --out /home/ros2_ws/src/data/sequences/rmuc \
  --preview-pcd
```

会写出 `velodyne/NNNNNN.bin`、`poses_suma_optim.txt`（`T_map_lidar`，雷达原点、z-up、只保留 yaw；本趟 map == odom）、`metadata.json`、`erasor2_seq.yaml`。

用 CloudCompare / pcl_viewer 看 `accumulated_preview.pcd`，应和 RViz 里 `/cloud_registered` 对得上。整图拧了 90° 说明位姿约定错了，先停，不要进聚类。

```bash
python3 - <<'PY'
import open3d as o3d
p = o3d.io.read_point_cloud("/home/ros2_ws/src/data/sequences/lab/accumulated_preview.pcd")
print("points:", len(p.points))
if p.is_empty():
    raise SystemExit("empty cloud — check the path")
o3d.visualization.draw_geometries(
    [p], window_name="accumulated_preview", width=1280, height=800
)
PY
```

无 bag 自检：

```bash
python3 /home/ros2_ws/src/mas2027_utils/ERASOR2/scripts/export_lio_bag.py --self-test
```

记下日志里的 `N keyframes`：后面 `--end_stamp` 和 yaml 的 `end_frame` 都是 `N - 1`。

更细的坐标系说明见 `mas2027_utils/ERASOR2/OFFLINE_LIO.md`。

### 3. 聚类（地面 + 动态实例）

需要镜像里的 `open3d` / `hdbscan` / `pypatchworkpp`（Dockerfile 已 pip 安装；改过 Dockerfile 要 `docker compose up -d --build`）。Humble 是 Ubuntu 22.04，**不要** `LD_PRELOAD`。脚本会开 Open3D 窗口，DISPLAY 和 RViz 一样即可。

```bash
# <last> = 上一步的 N - 1
python3 /home/ros2_ws/src/mas2027_utils/ERASOR2/scripts/kitti_clustering.py \
  --sequence-dir /home/ros2_ws/src/data/sequences/rmuc \
  --init_stamp 0 --end_stamp <last> \
  --save-instance-labels --save-ground-labels
```

标签写在序列目录的 `hdbscan/` 和 `patchwork/`。先目视确认：行人/车是独立颜色、没有和地面糊成一团，再往下走。

### 4. ERASOR2 去动态，得到干净全局 PCD

ERASOR2 是纯 CMake，不要 `colcon build` 它：

```bash
cd /home/ros2_ws/src/mas2027_utils/ERASOR2
sudo apt-get install -y libyaml-cpp-dev libopencv-dev libomp-dev libpcl-dev
cmake -B build -S . -DERASOR2_ENABLE_RERUN=OFF
cmake --build build -j$(nproc)

cp /home/ros2_ws/src/data/sequences/lab/erasor2_seq.yaml config/erasor2/lab.yaml
# 如需改输出目录，编辑 dataloader.abs_save_dir（默认 .../data/erasor2_out）
# mapgen 会读到 end_frame + accum_interval，序列里至少多留 1 帧
./build/mapgen      config/erasor2/lab.yaml
./build/run_erasor2 config/erasor2/lab.yaml
```

干净地图在 `abs_save_dir`，文件名类似：

```text
<data>/erasor2_out/rmuc_0_frame_0_to_<end>_estimated.pcd
```

这就是后面投影二维用的全局点云。容器和宿主机共享 PID namespace，里面不要 `pkill -f`。

### 5. 点云转二维栅格（pcd2pgm）

`pcd2pgm` 走 colcon。改 `mas2027_utils/pcd2pgm/config/pcd.yaml`，或启动时覆盖参数。`file_directory` 必须以 `/` 结尾；`file_name` 不含 `.pcd`。

```yaml
/pcd2pgm:
  ros__parameters:
    file_directory: /home/ros2_ws/src/data/erasor2_out/
    file_name: rmuc_0_frame_0_to_<end>_estimated   # 换成上一步真实文件名
    flag_pass_through: false   # false = 保留 [thre_z_min, thre_z_max] 之间的点
    map_resolution: 0.05       # 与现有 Nav2 地图一致
    map_topic_name: map
    thre_radius: 0.5
    thre_z_max: 1.5            # 雷达原点 z-up；挡墙留不住就提高，地面太厚就提高 min
    thre_z_min: 0.05
    thres_point_count: 10
    use_sim_time: false
```

启动（launch 默认 `use_sim_time:=true`，离线务必关掉）：

```bash
ros2 launch pcd2pgm pcd2pgm.launch.py use_sim_time:=false
```

或直接覆盖路径：

```bash
ros2 run pcd2pgm pcd2pgm_node --ros-args \
  -p use_sim_time:=false \
  -p file_directory:=/home/ros2_ws/src/data/erasor2_out/ \
  -p file_name:=rmuc_0_frame_0_to_<end>_estimated \
  -p map_topic_name:=map \
  -p map_resolution:=0.05 \
  -p thre_z_min:=0.05 \
  -p thre_z_max:=1.5 \
  -p flag_pass_through:=false
```

节点每秒发布 `/map`（transient_local）。RViz 加 Map 显示 `/map` 看切片。高度带不对就改 `thre_z_*` 再启动。满意后另开终端保存：

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /home/ros2_ws/src/mas2027_nav_bringup/map/rmuc
```

会生成 `rmuc.pgm` + `rmuc.yaml`。

### 6. Map Editor 修图

`map_edit` 也是 colcon 包。启动自带面板和橡皮擦的 RViz：

```bash
ros2 launch map_edit map_edit.launch.py
```

**打开地图（二选一）**

- 本地文件：左侧 `MapEditPanel` →「选择本地地图文件」→ 选上一步的 `rmuc.yaml`。
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
/home/ros2_ws/src/mas2027_nav_bringup/map/rmuc.pgm
/home/ros2_ws/src/mas2027_nav_bringup/map/rmuc.yaml
```

Nav2 用地图时确认 yaml 里 `image:`、`resolution:`（0.05）、`origin:` 和实际 pgm 一致。


