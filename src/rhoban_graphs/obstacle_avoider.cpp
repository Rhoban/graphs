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
void ObstacleAvoider::addObstacle(Eigen::Vector2d center, double radius)
{
  obstacles.push_back(Circle(center, radius));
}

typedef std::pair<Graph::Node, Graph::Node> NodePair;

std::vector<Eigen::Vector2d> ObstacleAvoider::findPath(Eigen::Vector2d start, Eigen::Vector2d goal, double accuracy,
                                                       double* score, std::function<bool(Eigen::Vector2d)> filter)
{
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
  nodeObstacle[0] = 0;
  nodeObstacle[1] = 0;

  // Adding circle nodes
  size_t count = 2;
  size_t oId = 1;
  for (auto& obstacle : obstacles)
  {
    Graph::Node first;

    double perimeter = 2 * M_PI * obstacle.getRadius();
    size_t steps = round(perimeter / accuracy);
    if (steps < 8)
      steps = 8;

    for (size_t k = 0; k < steps; k++)
    {
      // XXX: Parametrize the margin
      double x = obstacle.getCenter().x + cos(k * 2 * M_PI / steps) * (obstacle.getRadius() * 1.01);
      double y = obstacle.getCenter().y + sin(k * 2 * M_PI / steps) * (obstacle.getRadius() * 1.01);
      nodePositions[count] = Eigen::Vector2d(x, y);
      nodeObstacle[count] = oId;

      // Connecting sequential points
      if (k > 0)
      {
        ignoreCollisions[NodePair(count - 1, count)] = oId;
      }
      else
      {
        first = count;
      }
      count++;
    }
    // Closing the circle
    ignoreCollisions[NodePair(first, count - 1)] = oId;
    oId++;
  }

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

        size_t oId = 1;
        for (auto& obstacle : obstacles)
        {
          auto p = NodePair(node1, node2);
          bool startOrGoal = (node1 == 0 || node1 == 1);

          if (!ignoreCollisions.count(p) || ignoreCollisions[p] != oId)
          {
            if ((nodeObstacle[node1] == oId && nodeObstacle[node2] == oId) ||
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
          oId++;
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
  auto result = PathFinder::findPath(graph, 0, 1, score, heuristic);
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

  std::vector<Eigen::Vector2d> path;
  for (auto& node : result)
  {
    path.push_back(nodePositions[node]);
  }

  // Pruning
  if (path.size())
  {
    std::vector<Eigen::Vector2d> prunedPath;
    Eigen::Vector2d last = path[0];
    prunedPath.push_back(last);
    for (size_t k = 1; k < path.size(); k++)
    {
      if (k == path.size() - 1 || (path[k] - last).norm() > accuracy * 0.75)
      {
        last = path[k];
        prunedPath.push_back(last);
      }
    }
    path = prunedPath;
  }
  else
  {
    std::cout << "PATH FIND ERROR!" << std::endl;
    std::cout << start.x() << " " << start.y() << std::endl;
    std::cout << goal.x() << " " << goal.y() << std::endl;
    std::cout << obstacles[0].getCenter().x << " " << obstacles[0].getCenter().y << " " << obstacles[0].getRadius()
              << std::endl;
  }

#ifdef DEBUG
  // Draw the path -DEBUG-
  for (auto& pt : path)
  {
    std::cerr << pt.x << " " << pt.y << std::endl;
  }
#endif

  return path;
}

}  // namespace rhoban_graphs
