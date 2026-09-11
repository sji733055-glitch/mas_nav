# 和本仓库导航一起用

点航点用 `ros2 launch waypoint_editor waypoint_editor.launch.py`（默认 `lab3.yaml`）。

重复的 `waypoint_to_nav2` / `waypoint_through_nav2` 动作客户端已删除。把 CSV 统一交给主导航的 `waypoint_navigator.py`：

```bash
ros2 launch mas2027_nav_bringup waypoint_navigator.launch.py \
  waypoint_file:=/path/to/lab3_patrol.csv
```

导航已经在跑时，单独启动编辑器并传入 `use_map_server:=false`；主导航 RViz 不再加载编辑器插件。完整说明见 [README.md](README.md)。
