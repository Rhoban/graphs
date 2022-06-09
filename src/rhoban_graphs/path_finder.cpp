#include <iostream>
#include <algorithm>
#include <set>
#include "rhoban_graphs/path_finder.h"

namespace rhoban_graphs
{
std::vector<Graph::Node> PathFinder::findPath(Graph& graph, Graph::Node start, Graph::Node goal, double* score,
                                              std::map<Graph::Node, double> heuristic)
{
  // Maps a node to the next node that should be taken
  std::map<Graph::Node, Graph::Node> path;

  // Distance from the starting node to the goal
  std::map<Graph::Node, double> distances;

  // Explored nodes
  std::set<PathFinder::NodeToExplore> toExplore;
  std::map<Graph::Node, PathFinder::NodeToExplore> knownNodes;

  // Initalizing with start edge
  distances[start] = 0;
  knownNodes[0] = NodeToExplore(0, 0);
  toExplore.insert(knownNodes[0]);

  while (toExplore.size())
  {
    // Select the next node
    auto entry = *toExplore.begin();
    toExplore.erase(entry);
    Graph::Node node = entry.node;

    // Removing the node
    double weight = distances[node];

    // Updating
    for (auto& edge : graph.edges[node])
    {
      auto target = ((edge.node1 == node) ? edge.node2 : edge.node1);
      double cost = weight + edge.weight;

      if (!distances.count(target) || cost < distances[target])
      {
        distances[target] = cost;
        path[target] = node;

        double cost_heuristic = cost;

        if (heuristic.count(target))
        {
          cost_heuristic += heuristic[target];
        }
        if (!knownNodes.count(target))
        {
          knownNodes[target] = NodeToExplore(target, cost_heuristic);
        }
        else
        {
          toExplore.erase(knownNodes[target]);
          knownNodes[target].score = cost_heuristic;
        }

        // Updating heuristic score
        toExplore.insert(knownNodes[target]);
      }
    }

    if (node == goal)
    {
      break;
    }
  }

  std::vector<Graph::Node> nodes;
  if (path.count(goal))
  {
    Graph::Node tmp = goal;
    nodes.push_back(tmp);
    while (tmp != start)
    {
      tmp = path[tmp];
      nodes.push_back(tmp);
    }
    std::reverse(nodes.begin(), nodes.end());
  }

  if (score != NULL)
  {
    *score = distances[goal];
  }

  return nodes;
}
}  // namespace rhoban_graphs