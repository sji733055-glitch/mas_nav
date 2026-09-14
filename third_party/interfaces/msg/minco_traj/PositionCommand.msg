# MINCO 轨迹上单个采样点的期望状态，由 nav_executor 生成并跟踪。
# 来源：navi_minco_bit 的 src/ros_interfaces/msg/minco_traj/PositionCommand.msg，逐行搬运。
std_msgs/Header header
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 jerk
geometry_msgs/Vector3 angular_velocity
float64 yaw
float64 yaw_dot
float64 vel_norm
float64 acc_norm


float64[3] kx
float64[3] kv
uint32 trajectory_id
