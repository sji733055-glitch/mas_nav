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
// 移植自 mas_nav_2027 的 mas2027_planner/minco_planner/include/smac_search/constants.hpp。
// 唯一改动：命名空间改为 mas2027_nav_executor::smac，代价常量由 float 改为 uint8_t，
// 使 2027 里对 nav2_costmap_2d::NO_INFORMATION / INSCRIBED_INFLATED_OBSTACLE 的比较
// 在没有 Nav2 的本工程里保持同一套取值（255/254/253/252）。

#ifndef MAS2027_NAV_EXECUTOR__PATH_PLANNER__SEARCH__SMAC__CONSTANTS_HPP_
#define MAS2027_NAV_EXECUTOR__PATH_PLANNER__SEARCH__SMAC__CONSTANTS_HPP_

#include <cstdint>
#include <string>

namespace mas2027_nav_executor {
namespace smac {

enum class MotionModel
{
  UNKNOWN = 0,
  TWOD = 1,
  DUBIN = 2,
  REEDS_SHEPP = 3,
  STATE_LATTICE = 4,
};

inline std::string toString(const MotionModel & n)
{
  switch (n) {
  case MotionModel::TWOD:
    return "2D";
  case MotionModel::DUBIN:
    return "Dubin";
  case MotionModel::REEDS_SHEPP:
    return "Reeds-Shepp";
  case MotionModel::STATE_LATTICE:
    return "State Lattice";
  default:
    return "Unknown";
  }
}

inline MotionModel fromString(const std::string & n)
{
  if (n == "2D") {
    return MotionModel::TWOD;
  } else if (n == "DUBIN") {
    return MotionModel::DUBIN;
  } else if (n == "REEDS_SHEPP") {
    return MotionModel::REEDS_SHEPP;
  } else if (n == "STATE_LATTICE") {
    return MotionModel::STATE_LATTICE;
  } else {
    return MotionModel::UNKNOWN;
  }
}

// 与 Nav2 costmap 取值一致，见 nav2_costmap_2d/cost_values.hpp。
constexpr uint8_t UNKNOWN_COST = 255U;         // NO_INFORMATION
constexpr uint8_t OCCUPIED_COST = 254U;        // LETHAL_OBSTACLE
constexpr uint8_t INSCRIBED_COST = 253U;       // INSCRIBED_INFLATED_OBSTACLE
constexpr uint8_t MAX_NON_OBSTACLE_COST = 252U;
constexpr uint8_t FREE_COST = 0U;

}  // namespace smac
}  // namespace mas2027_nav_executor

#endif  // MAS2027_NAV_EXECUTOR__PATH_PLANNER__SEARCH__SMAC__CONSTANTS_HPP_
