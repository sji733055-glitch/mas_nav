#pragma once

#include <memory>

#include <Eigen/Core>

#include "rog_map/map_query_interface.hpp"

namespace mas2027_nav_executor {

class RogMapQueryAdapter final
{
public:
  explicit RogMapQueryAdapter(std::shared_ptr<rog_map::MapQueryInterface> query)
  : query_(std::move(query)) {}

  bool isFree(const Eigen::Vector2d & point) const
  {
    if (!query_) return false;
    unsigned int x = 0;
    unsigned int y = 0;
    return query_->worldToMap(point.x(), point.y(), x, y) && query_->isValid(x, y) &&
           query_->isFree(x, y);
  }

  bool evaluate(const Eigen::Vector3d & point, double & distance, Eigen::Vector3d & gradient) const
  {
    return query_ && query_->evaluate(point, distance, gradient);
  }

private:
  std::shared_ptr<rog_map::MapQueryInterface> query_;
};

}  // namespace mas2027_nav_executor
