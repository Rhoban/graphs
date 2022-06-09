#pragma once

#include "rhoban_graphs/graph.h"

namespace rhoban_graphs
{
class Dijkstra
{
public:
  static std::vector<Graph::Node> findPath(Graph& graph, Graph::Node start, Graph::Node goal, double* score = NULL);
};

}  // namespace rhoban_graphs
