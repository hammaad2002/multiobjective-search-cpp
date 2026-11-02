#include "mosp/heuristic.hpp"

#include <algorithm>
#include <functional>
#include <stdexcept>

namespace mosp {

Dijkstra::Dijkstra(const Graph& graph) : graph_(&graph), dist_(graph.num_nodes(), kInfCost) {
  heap_.reserve(graph.num_nodes());
}

const std::vector<Cost>& Dijkstra::run(NodeId source, unsigned objective) {
  if (source >= graph_->num_nodes()) throw std::out_of_range("dijkstra: source out of range");
  if (objective >= graph_->num_objectives()) {
    throw std::out_of_range("dijkstra: objective out of range");
  }
  std::fill(dist_.begin(), dist_.end(), kInfCost);
  heap_.clear();
  const auto cmp = std::greater<std::pair<Cost, NodeId>>{};

  dist_[source] = 0;
  heap_.emplace_back(0, source);
  while (!heap_.empty()) {
    std::pop_heap(heap_.begin(), heap_.end(), cmp);
    const auto [d, u] = heap_.back();
    heap_.pop_back();
    if (d != dist_[u]) continue;  // stale entry (lazy deletion)
    for (EdgeId e = graph_->first_arc(u); e < graph_->end_arc(u); ++e) {
      const NodeId v = graph_->head(e);
      const Cost nd = d + graph_->cost(e, objective);
      if (nd < dist_[v]) {
        dist_[v] = nd;
        heap_.emplace_back(nd, v);
        std::push_heap(heap_.begin(), heap_.end(), cmp);
      }
    }
  }
  return dist_;
}

std::vector<std::uint64_t> total_costs(const Graph& graph) {
  std::vector<std::uint64_t> totals(graph.num_objectives());
  for (unsigned i = 0; i < graph.num_objectives(); ++i) totals[i] = graph.total_cost(i);
  return totals;
}

}  // namespace mosp
