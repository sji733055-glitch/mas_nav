# Use with this stack

Place waypoints with `ros2 launch waypoint_editor waypoint_editor.launch.py` (defaults to `lab3.yaml`).

Do **not** use `waypoint_to_nav2` (`FollowWaypoints`) or `waypoint_through_nav2` (`NavigateThroughPoses`). Feed the CSV to `waypoint_navigator.py`:

```bash
ros2 launch mas2027_nav_bringup waypoint_navigator.launch.py \
  waypoint_file:=/path/to/lab3_patrol.csv
```

If Nav2 is already running, use Add Waypoint in the default RViz config. Do not start a second `map_server`. See [README.md](README.md).
