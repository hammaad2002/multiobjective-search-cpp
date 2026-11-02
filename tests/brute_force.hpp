// Exhaustive reference: enumerate every simple s-t path (every choice of
// parallel arc counts as a different path) and Pareto-filter their costs.
// With non-negative costs, a cycle never improves any objective, so the Pareto
// front over all walks equals the front over simple paths. Exponential; only
// for graphs with a handful of nodes.
#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

#include "mosp/graph.hpp"
#include "mosp/types.hpp"

namespace mosp::test {

template <std::size_t K>
std::vector<CostVec<K>> brute_force_front(const Graph& g, NodeId s, NodeId t) {
  std::vector<CostVec<K>> costs;
  std::vector<bool> on_path(g.num_nodes(), false);

  auto dfs = [&](auto&& self, NodeId u, const CostVec<K>& acc) -> void {
    if (u == t) {
      costs.push_back(acc);
      return;  // a path continuing past t and returning would revisit t
    }
    on_path[u] = true;
    for (EdgeId e = g.first_arc(u); e < g.end_arc(u); ++e) {
      const NodeId v = g.head(e);
      if (on_path[v]) continue;
      self(self, v, add(acc, g.cost_vec<K>(e)));
    }
    on_path[u] = false;
  };
  dfs(dfs, s, CostVec<K>{});
  return pareto_filter(std::move(costs));
}

// True if some s-t walk along `path` (choosing any parallel arc between
// consecutive nodes) has exactly cost `target`.
template <std::size_t K>
bool path_has_cost(const Graph& g, const std::vector<NodeId>& path, const CostVec<K>& target) {
  if (path.empty()) return false;
  std::vector<CostVec<K>> reachable{CostVec<K>{}};
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    std::vector<CostVec<K>> next;
    for (EdgeId e = g.first_arc(path[i]); e < g.end_arc(path[i]); ++e) {
      if (g.head(e) != path[i + 1]) continue;
      for (const auto& c : reachable) {
        const CostVec<K> n = add(c, g.cost_vec<K>(e));
        if (weakly_dominates(n, target)) next.push_back(n);  // prune overshoots
      }
    }
    std::sort(next.begin(), next.end());
    next.erase(std::unique(next.begin(), next.end()), next.end());
    reachable = std::move(next);
  }
  for (const auto& c : reachable) {
    if (c == target) return true;
  }
  return false;
}

}  // namespace mosp::test
