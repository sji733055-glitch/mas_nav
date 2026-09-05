# pcd2ele

把 PCD 栅格化成 max-Z 高程 PGM/YAML。**不是** Nav2 占用图：灰度是高度，空格子写成 0，套 `occupied_thresh` 会把低处/空洞当成墙。

本仓库默认读 `mas2027_nav_bringup/pcd/lab3.pcd`，写出 `map/lab3_elevation.{pgm,yaml}`。占用图请用 `pcd2pgm`。完整流程见仓库根 README **离线静态地图**。

```bash
ros2 launch pcd2ele pcd2ele.launch.py
```

离线用，launch 默认 `use_sim_time:=false`。
