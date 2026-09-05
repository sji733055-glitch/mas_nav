# pcd2esdf

名字是 pcd，实际输入是 Nav2 YAML/PGM。对占用图做双向距离变换，把有符号距离写进 PCD 的 intensity，并发布 `/esdf_result`（`frame_id: map`）。

运行时 MINCO 查的是 ROG 在线 ESDF，这份离线场只给可视化/对照。默认读 `mas2027_nav_bringup/map/lab3.yaml`。完整流程见仓库根 README **离线静态地图**。

```bash
ros2 launch pcd2esdf pcd2esdf.launch.py
```
