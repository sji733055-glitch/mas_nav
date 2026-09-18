#ifndef MINCO_PLANNER__UTILS__HEADER_HPP_
#define MINCO_PLANNER__UTILS__HEADER_HPP_

// C++ standard library
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

// Third-party
#include <Eigen/Core>

// ROS 2
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker.hpp"

// Project messages
#include "interfaces/msg/mpc_position_command.hpp"
#include "interfaces/msg/position_command.hpp"
#include "std_msgs/msg/header.hpp"

// Project dependencies
#include "data_structure/base/trajectory.h"
#include "minco_core/map_query_interface.hpp"
#include "traj_opt/backup_traj_optimizer_s4.h"
#include "mas2027_nav_executor/path_planner/trajectory/minco_optimizer.hpp"
#include "traj_opt/yaw_traj_opt.h"
#include "utils/header/color_text.hpp"

// Minco core headers
#include "mas2027_nav_executor/path_planner/search/astar.hpp"
#include "mas2027_nav_executor/path_planner/search/global_path_searcher.hpp"
#include "mas2027_nav_executor/path_planner/trajectory/local_path_processor.hpp"
#include "minco_core/components/planner_mode_context.hpp"
#include "mas2027_nav_executor/task_manager/recovery/recovery_behaivor.hpp"
#include "mas2027_nav_executor/path_planner/trajectory/trajectory_safety_checker.hpp"
#include "mas2027_nav_executor/path_planner/trajectory/corridor_generator.hpp"
#include "minco_core/minco_utils.hpp"
#include "minco_core/visualizer.hpp"

namespace minco_planner {
inline constexpr unsigned char kFreeCost = 0;
inline constexpr unsigned char kInscribedCost = 253;
inline constexpr unsigned char kLethalCost = 254;
inline constexpr unsigned char kUnknownCost = 255;
}  // namespace minco_planner

#endif  // MINCO_PLANNER__UTILS__HEADER_HPP_
