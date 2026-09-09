# pcd_trans

离线 PCD 刚体变换 + 格式转换。

一键建图脚本 `save_pcd_and_make_map.sh` 会用 `--matrix` 乘上 LIO 的 `T_odom_from_internal`（扶正）。这里的 `--tx/--yaw` 只留给场地原点对齐。

```bash
python3 pcd_tool.py --info --in /home/ros2_ws/src/mas2027_nav_bringup/pcd/lab3.pcd
python3 pcd_tool.py --in scan.pcd --out lab3.pcd --matrix scan_T_odom_from_internal.txt
python3 pcd_tool.py --in lab3.pcd --out lab3.pcd --tx 0 --ty 0 --yaw 0
```

浏览器里叠点云和 PGM：用系统浏览器打开 `visual_point_cloud_lab.html`（需要能访问 cdnjs/unpkg；不要指望 file:// 被禁网时还能跑）。

1. 地图页：先选 `map/lab3.yaml`，再选 `lab3.pgm`。墙是橙色。只选 PGM 时 origin 默认 0，要手填 yaml 里的 `[-4.6, -7.94]`。
2. 文件页：选 `pcd/lab3.pcd`。这个文件是 Open3D/PCL 的 `binary_compressed`，页面已经能解压。

完整流程见仓库根 README **离线静态地图**。
