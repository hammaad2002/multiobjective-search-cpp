// Single-objective Dijkstra and the per-objective "perfect" heuristic used by
// the multi-objective searches.
//
// For a goal t, h_i(u) is the exact cost of a cheapest u->t path under
// objective i, computed by Dijkstra from t on the reversed graph. Each h_i is
// admissible and consistent (h_i(u) <= c_i(u,v) + h_i(v)); h(u) = kInfCost
// marks nodes that cannot reach the goal.
#pragma once

#include <cstddef>
#include <utility>
#include <vector>

#include "mosp/graph.hpp"
#include "mosp/types.hpp"

namespace mosp {

class Dijkstra {
 public:
  explicit Dijkstra(const Graph& graph);

  // Costs of cheapest paths from source under one objective (kInfCost if
  // unreachable). The returned reference is overwritten by the next run().
  const std::vector<Cost>& run(NodeId source, unsigned objective);

 private:
  const Graph* graph_;
  std::vector<Cost> dist_;
  std::vector<std::pair<Cost, NodeId>> heap_;  // reused binary heap storage
};

// Fills h[u][i] = cheapest u->goal cost under objective i, for i < K.
// `backward` must have been constructed on the reversed graph.
template <std::size_t K>
void compute_heuristic(Dijkstra& backward, NodeId goal, unsigned num_nodes,
                       std::vector<CostVec<K>>& h) {
  h.resize(num_nodes);
  for (unsigned i = 0; i < K; ++i) {
    const std::vector<Cost>& d = backward.run(goal, i);
    for (unsigned u = 0; u < num_nodes; ++u) h[u][i] = d[u];
  }
}

}  // namespace mosp
