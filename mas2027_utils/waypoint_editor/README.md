# waypoint_editor

在先验 2D 图（`map` 系）上点、拖、转航点，保存为 CSV。来源：[kzm784/waypoint_editor](https://github.com/kzm784/waypoint_editor)（Apache-2.0），本仓库默认图改成 `lab3.yaml`。

重复实现 Nav2 动作客户端的 `waypoint_to_nav2` / `waypoint_through_nav2` 已删除。本仓库巡逻统一使用主导航的 `waypoint_navigator.py`，逐点发送 `navigate_to_pose` → MincoPlanner。

## 离线点航点（导航不要同时开）

会自己起一个 `map_server`，和正在跑的 Nav2 抢 `/map`：

```bash
ros2 launch waypoint_editor waypoint_editor.launch.py
# 换图：
# ros2 launch waypoint_editor waypoint_editor.launch.py map_yaml:=/home/ros2_ws/src/mas2027_nav_bringup/map/lab3.yaml
```

RViz Fixed Frame 是 `map`。工具栏选 **Add Waypoint**，在图上点住拖出朝向。绿球可平移，箭头可转 yaw。右键菜单可删点或改序号。

面板：

- **Load Map**：调 `map_server/load_map`。导航在跑时不要点，会把正在用的先验图换掉。
- **Save WPs** / **Load WPs**：CSV。

CSV 列：`id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command,...`
建议存到 `mas2027_nav_bringup/config/`，例如 `lab3_patrol.csv`。

## 导航已经在跑

主导航 RViz 不加载航点编辑器。确需在线编辑时，另开编辑器并复用 Nav2 的 `/map`：

```bash
ros2 launch waypoint_editor waypoint_editor.launch.py use_map_server:=false
```

选 Interact 才能拖已放的点。

点航点前把 Fixed Frame 改成 **map**（默认视图是 `odom`）。插件把点击坐标直接标成 `map`，Fixed Frame 留在 `odom` 上存下去会偏。

## 跑航线

```bash
python3 /home/ros2_ws/src/mas2027_nav_bringup/scripts/waypoint_navigator.py --ros-args \
  -p frame_id:=map \
  -p waypoint_file:=/home/ros2_ws/src/mas2027_nav_bringup/config/lab3_patrol.csv \
  -p loop:=true \
  -p start_delay_sec:=8.0
```

编过 bringup 之后也可以：

```bash
ros2 launch mas2027_nav_bringup waypoint_navigator.launch.py \
  waypoint_file:=/home/ros2_ws/src/mas2027_nav_bringup/config/lab3_patrol.csv
```

`frame_id` 必须是 `map`。`command` 列给编辑器备注用，导航脚本不读。
