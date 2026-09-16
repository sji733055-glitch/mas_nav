// Copyright (c) 2020, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.
//
// 移植自 mas_nav_2027 的 mas2027_planner/minco_planner/include/smac_search/types.hpp。
// 改动：命名空间改为 mas2027_nav_executor::smac；删除只服务 Nav2 平滑器的
// SmootherParams（含 rclcpp_lifecycle 依赖）与 GoalHeadingMode，本工程不使用；其余结构体保持原样。

#ifndef MAS2027_NAV_EXECUTOR__PATH_PLANNER__SEARCH__SMAC__TYPES_HPP_
#define MAS2027_NAV_EXECUTOR__PATH_PLANNER__SEARCH__SMAC__TYPES_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace mas2027_nav_executor {
namespace smac {

typedef std::pair<float, uint64_t> NodeHeuristicPair;

/**
 * @struct mas2027_nav_executor::smac::SearchInfo
 * @brief Search properties and penalties
 */
struct SearchInfo
{
  float minimum_turning_radius{8.0};
  float non_straight_penalty{1.05};
  float change_penalty{0.0};
  float reverse_penalty{2.0};
  float cost_penalty{2.0};
  float retrospective_penalty{0.015};
  float rotation_penalty{5.0};
  float analytic_expansion_ratio{3.5};
  float analytic_expansion_max_length{60.0};
  float analytic_expansion_max_cost{200.0};
  bool analytic_expansion_max_cost_override{false};
  std::string lattice_filepath;
  bool cache_obstacle_heuristic{false};
  bool allow_reverse_expansion{false};
  bool allow_primitive_interpolation{false};
  bool downsample_obstacle_heuristic{true};
  bool use_quadratic_cost_penalty{false};
};

/**
 * @struct mas2027_nav_executor::smac::TurnDirection
 * @brief A struct with the motion primitive's direction embedded
 */
enum class TurnDirection
{
  UNKNOWN = 0,
  FORWARD = 1,
  LEFT = 2,
  RIGHT = 3,
  REVERSE = 4,
  REV_LEFT = 5,
  REV_RIGHT = 6
};

/**
 * @struct mas2027_nav_executor::smac::MotionPose
 * @brief A struct for poses in motion primitives
 */
struct MotionPose
{
  MotionPose() {}

  MotionPose(const float & x, const float & y, const float & theta, const TurnDirection & turn_dir)
  : _x(x), _y(y), _theta(theta), _turn_dir(turn_dir)
  {
  }

  MotionPose operator-(const MotionPose & p2)
  {
    return MotionPose(this->_x - p2._x, this->_y - p2._y, this->_theta - p2._theta,
             TurnDirection::UNKNOWN);
  }

  float _x;
  float _y;
  float _theta;
  TurnDirection _turn_dir;
};

typedef std::vector<MotionPose> MotionPoses;

/**
 * @struct mas2027_nav_executor::smac::LatticeMetadata
 * @brief A struct of all lattice metadata
 */
struct LatticeMetadata
{
  float min_turning_radius;
  float grid_resolution;
  unsigned int number_of_headings;
  std::vector<float> heading_angles;
  unsigned int number_of_trajectories;
  std::string motion_model;
};

/**
 * @struct mas2027_nav_executor::smac::MotionPrimitive
 * @brief A struct of all motion primitive data
 */
struct MotionPrimitive
{
  unsigned int trajectory_id;
  float start_angle;
  float end_angle;
  float turning_radius;
  float trajectory_length;
  float arc_length;
  float straight_length;
  bool left_turn;
  MotionPoses poses;
};

/**
 * @struct mas2027_nav_executor::smac::GoalState
 * @brief A struct to store the goal state
 */
template <typename NodeT> struct GoalState
{
  NodeT * goal = nullptr;
  bool is_valid = true;
};

typedef std::vector<MotionPrimitive> MotionPrimitives;
typedef std::vector<MotionPrimitive *> MotionPrimitivePtrs;

}  // namespace smac
}  // namespace mas2027_nav_executor

#endif  // MAS2027_NAV_EXECUTOR__PATH_PLANNER__SEARCH__SMAC__TYPES_HPP_
