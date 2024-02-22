#include <set>
#include <map>
#include <cmath>

#include "rhoban_geometry/segment.h"
// #include "rhoban_graphs/dijkstra.h"
#include "rhoban_graphs/path_finder.h"

#include "rhoban_graphs/graph.h"
#include "rhoban_graphs/obstacle_avoider.h"

// #define DEBUG

using namespace rhoban_geometry;

namespace rhoban_graphs
{
int ObstacleAvoider::addObstacle(Eigen::Vector2d center, double radius)
{
  int obstacle_id = obstacles.size();
  obstacles[obstacle_id] = Circle(center, radius);

  return obstacle_id;
}

typedef std::pair<Graph::Node, Graph::Node> NodePair;

ObstacleAvoider::Result ObstacleAvoider::findPath(Eigen::Vector2d start, Eigen::Vector2d goal, double accuracy,
                                                  double margin, std::function<bool(Eigen::Vector2d)> filter)
{
  ObstacleAvoider::Result result;

  Graph graph;

  // Position of the nodes from the graph
  std::map<Graph::Node, Eigen::Vector2d> nodePositions;

  // Avoid intersection checks between nodes from a circle
  std::map<NodePair, size_t> ignoreCollisions;
  std::map<Graph::Node, size_t> nodeObstacle;

  std::map<Graph::Node, double> heuristic;  // heuristic distance from node to target (Euclidian distance)

  // Start and goal nodes
  nodePositions[0] = start;
  nodePositions[1] = goal;
  nodeObstacle[0] = -1;
  nodeObstacle[1] = -1;

  // Adding circle nodes
  size_t count = 2;
  for (auto& entry : obstacles)
  {
    int obstacle_id = entry.first;
    rhoban_geometry::Circle obstacle = entry.second;

    Graph::Node first;

    double perimeter = 2 * M_PI * obstacle.getRadius();
    size_t steps = round(perimeter / accuracy);
    if (steps < 8)
      steps = 8;

    for (size_t k = 0; k < steps; k++)
    {
      // XXX: Parametrize the margin
      double x = obstacle.getCenter().x + cos(k * 2 * M_PI / steps) * (obstacle.getRadius() + margin);
      double y = obstacle.getCenter().y + sin(k * 2 * M_PI / steps) * (obstacle.getRadius() + margin);
      nodePositions[count] = Eigen::Vector2d(x, y);
      nodeObstacle[count] = obstacle_id;

      // Connecting sequential points
      if (k > 0)
      {
        ignoreCollisions[NodePair(count - 1, count)] = obstacle_id;
      }
      else
      {
        first = count;
      }
      count++;
    }
    // Closing the circle
    ignoreCollisions[NodePair(first, count - 1)] = obstacle_id;
  }

  // Adding nodes to the graph
  for (auto& entry : nodePositions)
  {
    if (entry.first == 0 || entry.first == 1 || filter(entry.second))
    {
      graph.add(entry.first);
    }
  }

  // Connecting the nodes, if no intersection
  for (auto& node1 : graph.nodes)
  {
    for (auto& node2 : graph.nodes)
    {
      // Nodes are different and not connected
      if (node1 < node2)
      {
        // Creating the segment
        Segment segment(nodePositions[node1], nodePositions[node2]);
        BoundingBox segment_bbox = segment.getBoundingBox();
        bool ok = true;
        double score = segment.getLength();

        for (auto& entry : obstacles)
        {
          int obstacle_id = entry.first;
          rhoban_geometry::Circle obstacle = entry.second;

          auto p = NodePair(node1, node2);
          bool startOrGoal = (node1 == 0 || node1 == 1);

          if (!ignoreCollisions.count(p) || ignoreCollisions[p] != obstacle_id)
          {
            if ((nodeObstacle[node1] == obstacle_id && nodeObstacle[node2] == obstacle_id) ||
                (segment_bbox.intersects(obstacle.getBoundingBox()) && segment.intersects(obstacle)))
            {
              if (startOrGoal)
              {
                score *= 15;
              }
              else
              {
                ok = false;
              }
            }
          }
        }

        if (ok)
        {
          graph.connect(node1, node2, score);
        }
      }
    }

    // Point targetNodePosition = nodePositions[1];
    Point currentNodePosition = nodePositions[node1];

    heuristic[node1] = currentNodePosition.getDist(goal);
  }
  // for (auto& node : graph.nodes)
  // {
  //   std::cout << node << " : " << heuristic[node] << std::endl;
  // }
  // Running A*
  auto path_finder_result = PathFinder::findPath(graph, 0, 1, &result.score, heuristic);
  // auto result = PathFinder::findPath(graph, 0, 1, score);

#ifdef DEBUG
  // Draw the graph -DEBUG-
  for (auto& nodeEdges : graph.edges)
  {
    for (auto& edge : nodeEdges.second)
    {
      auto node1 = nodePositions[edge.node1];
      auto node2 = nodePositions[edge.node2];

      std::cout << node1.x << " " << node1.y << " " << edge.weight << std::endl;
      std::cout << node2.x << " " << node2.y << " " << edge.weight << std::endl;
      std::cout << std::endl << std::endl;
    }
  }
#endif

  for (auto& node : path_finder_result)
  {
    result.path.push_back(nodePositions[node]);
    result.obstacles.push_back(nodeObstacle[node]);
  }

  if (!result.path.size())
  {
    throw std::runtime_error("No path found");
  }

#ifdef DEBUG
  // Draw the path -DEBUG-
  for (auto& pt : path)
  {
    std::cerr << pt.x << " " << pt.y << std::endl;
  }
#endif

  return result;
}

ObstacleAvoider::Result ObstacleAvoider::findPathClipped(Eigen::Vector2d start, Eigen::Vector2d goal, double accuracy,
                                                         double margin, double x_min, double x_max, double y_min,
                                                         double y_max)
{
  return findPath(start, goal, accuracy, margin, [x_min, x_max, y_min, y_max](Eigen::Vector2d pt) {
    return pt.x() >= x_min && pt.x() <= x_max && pt.y() >= y_min && pt.y() <= y_max;
  });
}

}  // namespace rhoban_graphs
