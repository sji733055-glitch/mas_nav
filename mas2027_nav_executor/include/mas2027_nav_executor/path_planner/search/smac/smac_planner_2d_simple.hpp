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
// 移植自 mas_nav_2027 的 mas2027_planner/minco_planner/{include,src}/smac_search/
// smac_planner_2d_simple.{hpp,cpp}。算法本体逐行保留（8 邻域 A*、octile 启发式、
// tolerance 到点判定、膨胀代价因子、ESDF 势场软代价、SoA 搜索缓冲）。
// 相对 2027 的改动只有三处，都是为了在没有 Nav2 的本工程里编译与运行：
//   1. 去掉 nav2_costmap_2d::Costmap2DROS 成员与两个 configure 重载，改为
//      configure(rclcpp::Logger)：地图几何全部从 MapQueryInterface 取，
//      createPath() 本来每次都会刷新 origin/resolution/size。
//   2. 代价常量改从本目录 constants.hpp 取（取值与 Nav2 相同：255/254/253/252）。
//   3. ESDF 参数由 MincoPlanner 读 ROS 参数后经 setESDFParameters() 传入，
//      避免本类依赖 rclcpp::Node 的参数接口。

#ifndef MAS2027_NAV_EXECUTOR__PATH_PLANNER__SEARCH__SMAC__SMAC_PLANNER_2D_SIMPLE_HPP_
#define MAS2027_NAV_EXECUTOR__PATH_PLANNER__SEARCH__SMAC__SMAC_PLANNER_2D_SIMPLE_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "mas2027_nav_executor/path_planner/search/smac/constants.hpp"
#include "mas2027_nav_executor/path_planner/search/smac/types.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rog_map/map_query_interface.hpp"

namespace mas2027_nav_executor {
namespace smac {

/**
 * @class mas2027_nav_executor::smac::SmacPlanner2DSimple
 * @brief A simplified SMAC 2D planner (8-connected grid A*) with optional ESDF biasing
 */
class SmacPlanner2DSimple
{
public:
  struct Coordinates
  {
    Coordinates() = default;
    Coordinates(float x_in, float y_in) : x(x_in), y(y_in) {}

    float x{0.0f};
    float y{0.0f};
  };

  using CoordinateVector = std::vector<Coordinates>;

  struct NodeMin
  {
    uint64_t index{0u};
    float f_score{0.0f};
    bool operator>(const NodeMin & other) const { return f_score > other.f_score; }
  };

  SmacPlanner2DSimple();
  ~SmacPlanner2DSimple();

  /**
   * @brief Configure the planner (logger only; map geometry comes from the query)
   * @param logger Logger used for the [SMAC 2D] failure diagnostics
   */
  void configure(rclcpp::Logger logger);

  /**
   * @brief Optional distance-field biasing, queried through MapQueryInterface
   * @param enable Use ESDF potential cost in the traversal cost
   * @param weight Potential weight (mas_nav_2027 smac_2d.esdf_weight)
   * @param decay Exponential decay length in metres (smac_2d.esdf_decay)
   * @param max_cost Upper clamp of the potential, <=0 disables the clamp (smac_2d.esdf_max_cost)
   */
  void setESDFParameters(bool enable, double weight, double decay, double max_cost);

  void setMap(const std::shared_ptr<rog_map::MapQueryInterface> & map);
  void setESDFQuery(const std::shared_ptr<rog_map::MapQueryInterface> & query);

  /**
   * @brief Create a path from start to goal
   * @param start_x Start X in map coordinates
   * @param start_y Start Y in map coordinates
   * @param goal_x Goal X in map coordinates
   * @param goal_y Goal Y in map coordinates
   * @param path Output path coordinates, goal -> start (callers reverse it)
   * @param cancel_checker Function to check if planning should be canceled
   * @return true if path found
   */
  bool createPath(const unsigned int & start_x,
    const unsigned int & start_y,
    const unsigned int & goal_x,
    const unsigned int & goal_y,
    CoordinateVector & path,
    std::function<bool()> cancel_checker = nullptr);

  /**
   * @brief Set parameters
   * @param allow_unknown If we allow traversing unknown space
   * @param max_iterations Maximum iterations
   * @param tolerance Tolerance for goal reaching (metres)
   */
  void setParameters(bool allow_unknown, int max_iterations, float tolerance);

private:
  void ensureSearchBuffers();
  void invalidateESDFCache();
  void logFailure(const std::string & reason,
    unsigned int start_x,
    unsigned int start_y,
    unsigned int goal_x,
    unsigned int goal_y,
    int iterations = -1) const;

  /**
   * @brief Compute ESDF-based potential cost for a grid cell index
   */
  float getESDFPotentialCost(unsigned int mx, unsigned int my);

  /**
   * @brief Evaluate inflation traversal factor from costmap cell cost
   */
  float evaluateInflationCost(unsigned char cell_cost);

  // Parameters
  bool allow_unknown_;
  int max_iterations_;
  float tolerance_;

  // Costmap
  std::shared_ptr<rog_map::MapQueryInterface> map_;
  std::shared_ptr<rog_map::MapQueryInterface> esdf_query_;
  unsigned int size_x_;
  unsigned int size_y_;

  // Cached costmap metadata (updated in setMap()/createPath() to avoid per-cell getters).
  double costmap_origin_x_{0.0};
  double costmap_origin_y_{0.0};
  double costmap_resolution_{0.0};

  // SoA search buffers (lazy reset via planning_id_)
  std::vector<float> g_score_;
  std::vector<int> parent_;
  std::vector<uint32_t> visited_;
  std::vector<uint32_t> closed_;
  uint32_t planning_id_{0u};
  uint64_t grid_size_{0u};

  // Optional distance-field biasing queried through MapQueryInterface.
  bool use_esdf_cost_{false};
  float esdf_weight_{1.0f};
  float esdf_decay_{0.5f};
  float esdf_max_cost_{5.0f};

  // Per-planning-iteration cache to avoid allocations in the search loop.
  std::vector<float> esdf_cost_cache_;
  std::vector<uint32_t> esdf_cost_cache_id_;

  // Search info
  SearchInfo search_info_;
  MotionModel motion_model_;

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("SmacPlanner2DSimple")};
};

}  // namespace smac
}  // namespace mas2027_nav_executor

#endif  // MAS2027_NAV_EXECUTOR__PATH_PLANNER__SEARCH__SMAC__SMAC_PLANNER_2D_SIMPLE_HPP_
