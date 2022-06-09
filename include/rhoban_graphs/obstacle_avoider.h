#pragma once

#include <functional>

#include "rhoban_geometry/circle.h"
#include "rhoban_geometry/point.h"

namespace rhoban_graphs
{
class ObstacleAvoider
{
public:
  void addObstacle(rhoban_geometry::Point center, double radius);

  std::vector<rhoban_geometry::Point> findPath(
      rhoban_geometry::Point start, rhoban_geometry::Point goal, double accuracy = 20, double* score = NULL,
      std::function<bool(rhoban_geometry::Point)> filter = [](rhoban_geometry::Point) { return true; });

protected:
  std::vector<rhoban_geometry::Circle> obstacles;
};

}  // namespace rhoban_graphs
