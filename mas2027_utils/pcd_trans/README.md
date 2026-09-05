# pcd_trans

离线 PCD 刚体变换 + 格式转换。

```bash
python3 pcd_tool.py --info --in /home/ros2_ws/src/mas2027_nav_bringup/pcd/lab3.pcd
python3 pcd_tool.py --in lab3.pcd --out lab3.pcd --tx 0 --ty 0 --yaw 0
```

`visual_point_cloud_lab.html` 可在浏览器里把点云叠在 PGM 上旋转导出。完整流程见仓库根 README **离线静态地图**。
