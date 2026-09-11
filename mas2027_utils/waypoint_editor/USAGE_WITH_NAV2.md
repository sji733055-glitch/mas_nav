# Use with this stack

Place waypoints with `ros2 launch waypoint_editor waypoint_editor.launch.py` (defaults to `lab3.yaml`).

The redundant `waypoint_to_nav2` and `waypoint_through_nav2` clients have been removed. Feed the CSV to the main stack's `waypoint_navigator.py`:

```bash
ros2 launch mas2027_nav_bringup waypoint_navigator.launch.py \
  waypoint_file:=/path/to/lab3_patrol.csv
```

If Nav2 is already running, start the standalone editor with `use_map_server:=false`; the main RViz intentionally does not load editor plugins. See [README.md](README.md).
