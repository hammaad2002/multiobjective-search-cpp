// Single-objective Dijkstra and the per-objective "perfect" heuristic used by
// the multi-objective searches.
//
// For a goal t, h_i(u) is the exact cost of a cheapest u->t path under
// objective i, computed by Dijkstra from t on the reversed graph. Each h_i is
// admissible and consistent (h_i(u) <= c_i(u,v) + h_i(v)); h(u) = kInfCost
// marks nodes that cannot reach the goal.
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
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

// Per-objective sums of all arc costs (an upper bound on any simple path).
[[nodiscard]] std::vector<std::uint64_t> total_costs(const Graph& graph);

// f = g + h must not overflow Cost (or reach the kInfCost sentinel). With g
// bounded by the total arc cost, it suffices that total_i + max finite h_i <
// kInfCost for each objective i. Throws std::overflow_error otherwise.
template <std::size_t K>
void check_heuristic_range(const std::vector<std::uint64_t>& totals,
                           const std::vector<CostVec<K>>& h) {
  CostVec<K> max_h{};
  for (const auto& hu : h) {
    if (hu[0] == kInfCost) continue;  // unreachable states are never expanded
    for (std::size_t i = 0; i < K; ++i) max_h[i] = std::max(max_h[i], hu[i]);
  }
  for (std::size_t i = 0; i < K; ++i) {
    if (totals[i] + max_h[i] >= kInfCost) {
      throw std::overflow_error("objective " + std::to_string(i) +
                                ": path cost plus heuristic may overflow 32-bit costs");
    }
  }
}

}  // namespace mosp
