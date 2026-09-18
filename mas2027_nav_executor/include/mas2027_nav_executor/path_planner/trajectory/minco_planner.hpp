#ifndef MINCO_PLANNER__MINCO_PLANNER_HPP_
#define MINCO_PLANNER__MINCO_PLANNER_HPP_

#include "minco_core/header.hpp"
#include "minco_core/performance/planner_performance_monitor.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <limits>
#include <map>
#include <string>

namespace minco_planner {

class Visualizer;

class MincoPlanner
{
public:
  using Ptr = std::shared_ptr<MincoPlanner>;

  // === Constructor & Lifecycle ===
  MincoPlanner();
  ~MincoPlanner();

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf);
  void cleanup();

  // === Core Planning Interfaces ===
  bool PlanGlobalPath(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal);

  void setMap(const std::shared_ptr<rog_map::MapQueryInterface> & map);
  void setTerrainGrid(std::shared_ptr<mas2027_nav_executor::TerrainGrid> terrain);

  bool ReplanLocal(const geometry_msgs::msg::PoseStamped & current_pose);
  bool makePlan(const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal,
    double tolerance,
    std::function<bool()> cancel_checker,
    nav_msgs::msg::Path & plan);

  // === Callbacks ===
  void safetyTimerCallback();

  // === Utility & Helper Functions ===
  bool checkCollision();
  bool checkCollision(const geometry_utils::Trajectory & traj);

  /// 净空要求（发布前校验与运行时监视共用同一个公式）。
  /// 只要两处用不同阈值，就会出现「发布前 0.30 通过、监视器 0.50 立刻否决」的死循环：
  /// cmd_vel 只在发布的瞬间有值，RViz 轨迹在规划轨迹与急停轨迹之间来回闪。
  /// required = collision_dist + max(v * replan_react_time, monitor_margin)
  double requiredClearance(double speed) const;

  // Accessors for FSM
  bool isTrajSafe() const { return is_traj_safe_.load(); }
  RecoverServer::Ptr recoveryServer() const { return recovery_server_; }
  double getForceReplanPeriod() const { return force_replan_period_sec_; }
  double nowSeconds() const;
  double getTrajectoryRemainTime() const;
  bool isTrajectoryTimeExpired(double now_s) const;
  double getLookaheadDist() const { return lookahead_dist_; }
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose) const;
  bool checkGoalReached(const geometry_msgs::msg::PoseStamped & current_pose);
  bool checkGoalReached(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & goal) const;
  bool hasGlobalPath() const;
  void invalidateGlobalPath();
  // 拷贝一份当前全局搜索（SMAC 2D / Astar）输出的折线，供 RViz 显示。
  // 返回 false 表示目前没有可用的全局路径（尚未搜索或已被 invalidateGlobalPath 清掉）。
  // 与 hasGlobalPath() 的区别：这里连数据一起取出，避免调用方分两次加锁看到不同的快照。
  bool copyLatestGlobalPath(std::vector<geometry_msgs::msg::PoseStamped> & out) const;
  Eigen::Vector3d getCurrentSpeed() const;
  double getCurrentYawFromOdom() const;

  // Query ESDF distance at the given position.
  double getEsdfDistance(const Eigen::Vector3d & pos) const;

  void publishEscapeCommand(
    const geometry_msgs::msg::PoseStamped & current_pose, const Eigen::Vector2d & escape_vel);

  void clearRecoveryDebugVisualization();

  void publishEmergencyStop(const geometry_msgs::msg::PoseStamped & current_pose);

  traj_opt::Trajectory generateBackupTraj(const Eigen::Matrix3d & start_state);
  std::vector<Eigen::Vector3d> extractLocalPath(const Eigen::Vector3d & cur_pos);

private:
  // === Internal Types ===
  enum class PlanningState
  {
    COLD_START,     // Full replanning with zero initial velocity/acceleration.
    HOT_START,      // Replanning with inherited velocity/acceleration.
    EMERGENCY_STOP  // Immediate backup braking with safety priority.
  };

  // === Utility & Helper Functions ===
  PlanningState determinePlanningState(
    const geometry_msgs::msg::Pose & start_pose, const std::vector<Eigen::Vector3d> & new_path);

  void prepareColdStart(const geometry_msgs::msg::Pose & start_pose,
    Eigen::Matrix3d & start_state,
    const std::vector<Eigen::Vector3d> & sparse_path);

  void prepareHotStart(
    const geometry_msgs::msg::Pose & start_pose, double t_dur, Eigen::Matrix3d & start_state);

  void PTAllocation(const std::vector<Eigen::Vector3d> & sparse_path,
    const Eigen::Matrix3d & start_state,
    bool stop_at_local_end,
    PlanningState state,
    bool has_shifted_seed,
    const vec_Vec3f & shifted_waypoints,
    const VecDf & shifted_durations,
    vec_Vec3f & init_ps,
    VecDf & init_ts,
    VecDf & local_vmaxs) const;

  bool validateTrajectory(const traj_opt::Trajectory & traj, const Eigen::Vector3d & expected_end_pos);

  /// 把备份（急停）优化器实际使用的安全盒（SFC）发布成 RViz 调试 marker。
  /// 纯可视化，不参与任何约束构造或规划决策。
  void publishSafeCorridorBox(const PolyhedronH & poly);

  bool optimizeYaw(const Eigen::Matrix3d & start_state,
    const traj_opt::Trajectory & pos_traj,
    traj_opt::Trajectory & out_yaw_traj,
    PlanningState state,
    const geometry_msgs::msg::Pose & current_pose,
    double goal_yaw);

  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters);

  bool ensureMapAvailable();
  void rebuildModeDependentQueries();
  void configureMincoPerfLogging(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & prefix);

  void initPlannerMode(
    const std::string & planner_mode_param, const std::string & map_frame, const std::string & rog_frame);
  // === ROS 2 Interfaces (Publishers, Subscribers, Timers) ===
  rclcpp::Publisher<interfaces::msg::MpcPositionCommand>::SharedPtr opt_path_pub_;
  rclcpp::Publisher<interfaces::msg::MpcPositionCommand>::SharedPtr backup_path_pub_;
  /// 备份安全盒调试可视化，由 generateBackupTraj() 在每次重规划时刷新。
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr safe_corridor_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr safety_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  // === TF & Costmap & Frames ===
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<rog_map::MapQueryInterface> map_;
  std::shared_ptr<rog_map::MapQueryInterface> rog_query_raw_;
  std::string global_frame_, planning_frame_, output_frame_, map_frame_, rog_frame_, name_;
  PlannerModeParams mode_params_;

  // === Configurations & Parameters ===
  double tolerance_;
  bool allow_unknown_;
  // 全局主搜索：true 走 SMAC 2D（对齐 mas_nav_2027），false 退回 Astar。
  bool use_smac_{true};
  // SMAC 的 ESDF 势场软代价，默认值对齐 mas_nav_2027 的 smac_2d 段。
  bool smac_use_esdf_cost_{true};
  double smac_esdf_weight_{1.0};
  double smac_esdf_decay_{0.8};
  double smac_esdf_max_cost_{0.5};
  bool use_yaw_opt_{true};
  bool exploration_unknown_as_occupied_{true};
  bool exploration_prefer_goal_direction_{true};
  double exploration_boundary_margin_{0.8};
  double exploration_boundary_sample_step_{0.1};
  double lidar_offset_x_{0.0};
  double lidar_offset_y_{-0.2};
  double opt_freq_;
  double lookahead_dist_;
  double traj_goal_tolerance_{0.15};
  double collision_dist_{0.30};
  double force_replan_period_sec_{0.2};
  double replan_react_time_{0.35};
  /// 速度感知净空开关：开启后优化器的位置罚项改用 required(v) 口径（见 MincoOptimizer::Config）。
  bool speed_aware_clearance_{false};
  /// 优化器软目标相对硬判据的余量（优化器只能渐近逼近，避免毫米级擦边被否）。
  double clearance_optimizer_margin_{0.05};
  double safety_lookahead_time_{1.2};
  double monitor_margin_{0.20};
  double last_estop_s_{-1.0};
  MincoOptimizer::Config minco_config;
  RecoverServer::Config recovery_server_config_{};
  PlannerPerformanceMonitor planner_perf_monitor_;

  // === Core Modules (Pointers to FSM, Optimizers, etc.) ===
  std::unique_ptr<Astar> astar_planner_;
  // SMAC 2D 主搜索（对齐 mas_nav_2027 的 PRIORMAP）。use_smac 为真时 GlobalPathSearcher
  // 走它，astar_planner_ 退为 use_smac:=false 时的备用分支。
  std::unique_ptr<mas2027_nav_executor::smac::SmacPlanner2DSimple> smac_planner_;
  std::unique_ptr<MincoOptimizer> minco_optimizer_;
  std::unique_ptr<traj_opt::BackupTrajOpt> backup_opt_;
  std::unique_ptr<traj_opt::YawTrajOpt> yaw_opt_;
  std::unique_ptr<PlannerModeContext> mode_context_;
  std::unique_ptr<GlobalPathSearcher> global_path_searcher_;
  std::shared_ptr<mas2027_nav_executor::TerrainGrid> terrain_;
  std::unique_ptr<LocalPathProcessor> local_path_processor_;
  std::unique_ptr<TrajectorySafetyChecker> safety_checker_;
  SimpleCorridorGenerator::Ptr corridor_gen_;
  RecoverServer::Ptr recovery_server_;

  // === State Variables & Caches ===
  uint32_t opt_trajectory_id_{0};
  uint32_t backup_trajectory_id_{0};
  geometry_utils::Trajectory last_traj_;
  geometry_utils::Trajectory last_yaw_traj_;
  std::vector<geometry_msgs::msg::PoseStamped> latest_global_path_;
  nav_msgs::msg::Odometry latest_odom_;

  bool has_last_traj_ = false;
  bool has_last_yaw_traj_ = false;
  bool has_latest_odom_{false};
  std::atomic_bool is_traj_safe_{true};

  // === 失败原因统计（排障用） ===
  // 失败原因日志原来整条 2 s 节流：实车一次 10 分钟运行 350 次失败只留下 139 条原因，
  // 其中"修复后 0.3 ms 立即失败"的 76 次现场里有 60 次连原因都没打出来，"死在哪一步"
  // 无法从日志判断。现在按原因计数：每种原因前 failure_log_first_n_ 次逐条打出（带累计计数），
  // 之后每 failure_log_every_n_ 次采样一条；每 failure_summary_every_ 次失败再打一条汇总。
  uint64_t replan_failure_total_{0};
  std::map<std::string, uint64_t> replan_failure_counts_;
  int64_t failure_log_first_n_{10};
  // 第 failure_log_first_n_ 次之后的采样间隔。**不要退回按时间节流**：2 s 节流在
  // "失败比节流窗口更密"时会丢现场——2026-09-16 14:55 那次运行 158 次失败只留下 54 条原因，
  // COLLISION 109 次被压成 31 条，正是这个原因。按计数采样则无论失败多密都保证留下样本。
  int64_t failure_log_every_n_{25};
  int64_t failure_summary_every_{50};

  // === 地形门否决点插桩（只加日志，不改任何判据/阈值） ===
  // 起因：2026-09-16 20:13 那次运行 50 次失败里 TERRAIN_COLLISION_OR_DIRECTION 占 26 次
  // （最长一次连续卡顿 21 s），但这条原因**从不打印否决位置**，因此无法判断它是在真实的
  // 先验图墙体上否决（说明现场挤压是真的）还是在别处（说明判据或坐标系有问题）。
  // 净空门早就有 "Trajectory clearance ... at (x,y)"，地形门缺同等待遇。
  // 前 kTerrainRejectLogFirstN 次逐条打印，之后每 kTerrainRejectLogEveryN 次采样一条。
  uint64_t terrain_reject_log_count_{0};

  // 第四层兜底（短距离脱困前缀）参数，见 minco_planner.cpp 的 configure 段。
  bool escape_enable_{true};
  double escape_min_length_{0.08};
  double escape_buffer_{0.05};

  // === 软种子门的退路：连续失败后切回"净空硬否决"（绕行 / 停车前缀 / 脱困前缀） ===
  // 背景（2026-09-17 实车 20:09 那次）：种子门撤掉净空硬否决后，遇到"折线本身就是唯一通路、
  // 但比 required 窄几毫米"的现场时，MINCO 每次都拿到同一个不可能成功的种子，于是
  // `MINCO path generation failed; retrying` 以 2 Hz 刷屏、`/opt_path` 一直为空、车原地不动
  // （那一次连续 600 次失败，目标 (7.16, 2.32) 永远到不了）。硬否决版本则会在同一处走三层
  // 兜底把车带出去（旧行为，实车能走）。因此：连续失败达到 strict_seed_after_failures_ 次
  // 就把种子门切回硬否决，成功一次即复位。0 表示关闭（永远用软种子门 = 回到本次改动之前）。
  int64_t strict_seed_after_failures_{3};
  uint64_t consecutive_local_failures_{0};
  bool strict_seed_active_{false};

  mutable std::mutex path_mutex_;
  std::mutex perf_mutex_;
  mutable std::mutex odom_mutex_;
  mutable std::mutex mutex_;

  std::unique_ptr<Visualizer> visualizer_;
  rclcpp::Logger logger_{rclcpp::get_logger("MincoPlanner")};
  double last_global_search_time_ms_{std::numeric_limits<double>::quiet_NaN()};
  bool has_fresh_global_search_time_{false};
  long long last_minco_perf_stamp_ns_{0};
  std::string last_validation_failure_reason_{"KINEMATIC_VIOLATION"};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__MINCO_PLANNER_HPP_
