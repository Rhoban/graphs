#pragma once

#include "rhoban_graphs/graph.h"
namespace rhoban_graphs
{
// Implements Dijkstra and A*
// if heuristic is set, computes A*
// if not, computes Dijkstra
class PathFinder
{
public:
  static std::vector<Graph::Node> findPath(Graph& graph, Graph::Node start, Graph::Node goal, double* score = NULL,
                                           std::map<Graph::Node, double> heuristic = {});
};

}  // namespace rhoban_graphs
