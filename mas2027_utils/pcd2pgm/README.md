# PCD2PGM

把 `.pcd` 切成 Nav2 占用图（OccupancyGrid）。接在 `mas2027_nav_bringup/pcd/` 先验点云之后、Map Editor 之前。完整流程见仓库根 [`README.md`](../../README.md) **离线静态地图**。

|pcd|pgm|
|:-:|:-:|
|![pcd](.docs/pcd.png)|![pgm](.docs/pgm.png)|

## 功能

- 读取指定的 `.pcd`
- Pass Through 按 Z 带切片
- Radius Outlier 去飞点
- XY 投影成占据栅格（有点=100，其余=0，没有 unknown=205）
- 每秒发 `/map`（transient_local）

## 使用

包在 `mas2027_utils/pcd2pgm/`，随仓库 colcon 一起编。改 `config/pcd.yaml`：`file_directory` 必须以 `/` 结尾，`file_name` 不含 `.pcd`。当前默认读 `lab3.pcd`。

```bash
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash
ros2 launch pcd2pgm pcd2pgm.launch.py
```

或直接覆盖路径：

```bash
ros2 run pcd2pgm pcd2pgm_node --ros-args \
  -p file_directory:=/home/ros2_ws/src/mas2027_nav_bringup/pcd/ \
  -p file_name:=lab3 \
  -p map_resolution:=0.05 \
  -p thre_z_min:=0.05 \
  -p thre_z_max:=1.5 \
  -p flag_pass_through:=false
```

RViz 加 Map 订 `/map` 看切片。满意后另开终端保存：

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /home/ros2_ws/src/mas2027_nav_bringup/map/lab3
```

会写出 `lab3.pgm` + `lab3.yaml`。修墙用 `map_edit`。

## 参数

```yaml
/pcd2pgm:
  ros__parameters:
    file_directory: /home/ros2_ws/src/mas2027_nav_bringup/pcd/
    file_name: lab3
    flag_pass_through: false   # false = 保留 [thre_z_min, thre_z_max] 之间的点
    map_resolution: 0.05
    map_topic_name: map
    thre_radius: 0.5
    thre_z_max: 1.5
    thre_z_min: 0.05
    thres_point_count: 10
    use_sim_time: false
```

上游来源：https://gitee.com/LihanChen2004/pcd2pgm
