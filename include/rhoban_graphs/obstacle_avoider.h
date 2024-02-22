#pragma once

#include <functional>

#include "rhoban_geometry/circle.h"
#include "rhoban_geometry/point.h"

namespace rhoban_graphs
{
class ObstacleAvoider
{
public:
  void addObstacle(Eigen::Vector2d center, double radius);

  std::vector<Eigen::Vector2d> findPath(
      Eigen::Vector2d start, Eigen::Vector2d goal, double accuracy = 20, double* score = NULL,
      std::function<bool(Eigen::Vector2d)> filter = [](rhoban_geometry::Point) { return true; });

protected:
  std::vector<rhoban_geometry::Circle> obstacles;
};

}  // namespace rhoban_graphs
