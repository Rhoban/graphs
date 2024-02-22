#pragma once

#include <functional>

#include "rhoban_geometry/circle.h"
#include "rhoban_geometry/point.h"

namespace rhoban_graphs
{
class ObstacleAvoider
{
public:
  struct Result
  {
    std::vector<Eigen::Vector2d> path;
    double score;
    std::vector<int> obstacles;
  };

  int addObstacle(Eigen::Vector2d center, double radius);

  Result findPath(
      Eigen::Vector2d start, Eigen::Vector2d goal, double accuracy = 0.25, double margin = 0.1,
      std::function<bool(Eigen::Vector2d)> filter = [](rhoban_geometry::Point) { return true; });

  Result findPathClipped(Eigen::Vector2d start, Eigen::Vector2d goal, double accuracy = 0.25, double margin = 0.1,
                         double x_min = -10, double x_max = 10, double y_min = -10, double y_max = 10);

protected:
  std::map<int, rhoban_geometry::Circle> obstacles;
};

}  // namespace rhoban_graphs
