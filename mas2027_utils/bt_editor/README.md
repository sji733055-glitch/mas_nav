# bt_editor

从上游 `navi_minco_bit/src/utils/bt_editor/bt_editor.html` 搬来的浏览器行为树编辑器。不进主 launch，也不依赖 Groot。

本仓库的树在 `mas2027_nav_bringup/behavior_trees/`。编辑器已经认识 Nav2 / MINCO 节点（`RecoveryNode`、`ComputePathToPose`、`FollowPath`、`ZeroGoalStamp`、`GoalUpdatedController` 等），导出带 `BTCPP_format="3"`。

## 用法

宿主机或容器里用浏览器打开：

```bash
# 容器内若有浏览器
xdg-open /home/ros2_ws/src/mas2027_utils/bt_editor/bt_editor.html
```

更常见是在宿主机打开同一份挂载文件：

```text
<repo>/mas2027_utils/bt_editor/bt_editor.html
```

然后「导入 XML」选：

```text
mas2027_nav_bringup/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml
```

改完导出，覆盖回 `behavior_trees/`。`planner_id` 必须仍是 `MincoPlanner`，`controller_id` 是 `FollowPath`。

可选：连 rosbridge（`ws://localhost:9090`）看话题列表。本栈默认不启 rosbridge，离线改 XML 就够。
