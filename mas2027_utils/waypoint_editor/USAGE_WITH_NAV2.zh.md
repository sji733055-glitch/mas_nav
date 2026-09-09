# 和本仓库导航一起用

点航点用 `ros2 launch waypoint_editor waypoint_editor.launch.py`（默认 `lab3.yaml`）。

**不要**用 `waypoint_to_nav2`（`FollowWaypoints`，Humble 无单点超时）或 `waypoint_through_nav2`（`NavigateThroughPoses`）。把 CSV 交给 `waypoint_navigator.py`：

```bash
ros2 launch mas2027_nav_bringup waypoint_navigator.launch.py \
  waypoint_file:=/path/to/lab3_patrol.csv
```

导航已经在跑时，用默认 RViz 里的 Add Waypoint，不要再起第二份 `map_server`。完整说明见 [README.md](README.md)。
