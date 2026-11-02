#include "mosp/boa_star.hpp"

#include <algorithm>
#include <stdexcept>

#include "mosp/dimacs.hpp"
#include "mosp/heuristic.hpp"

namespace mosp {

BoaStar::BoaStar(const Graph& graph) : graph_(&graph), g2_min_(graph.num_nodes(), kInfCost) {
  if (graph.num_objectives() != 2) throw std::invalid_argument("BOA* needs exactly 2 objectives");
  check_cost_range(graph);
  totals_ = total_costs(graph);
}

SearchResult<2> BoaStar::search(NodeId start, NodeId goal, const std::vector<CostVec<2>>& h,
                                const SearchOptions& options) {
  const Graph& graph = *graph_;
  if (start >= graph.num_nodes() || goal >= graph.num_nodes()) {
    throw std::out_of_range("BOA*: node out of range");
  }
  if (h.size() != graph.num_nodes()) throw std::invalid_argument("BOA*: heuristic size mismatch");
  check_heuristic_range<2>(totals_, h);

  SearchResult<2> result;
  const Deadline deadline(options.timeout_seconds);
  std::fill(g2_min_.begin(), g2_min_.end(), kInfCost);
  nodes_.clear();
  open_.clear();
  std::vector<std::uint32_t> solution_ids;

  if (h[start][0] != kInfCost) {
    nodes_.push_back(Node{{0, 0}, start, kNoParent});
    open_.push(h[start], 0);
    result.stats.generated = 1;
  }

  std::uint64_t pops = 0;
  while (!open_.empty()) {
    if (++pops % kDeadlinePollInterval == 0 && deadline.expired()) {
      result.stats.status = SearchStatus::kTimeout;
      break;
    }
    const auto [f, id] = open_.pop();
    const Node x = nodes_[id];
    // Lines 11-12: lazy dominance check at extraction.
    if (x.g[1] >= g2_min_[x.state] || f[1] >= g2_min_[goal]) continue;
    // Line 13.
    g2_min_[x.state] = x.g[1];
    ++result.stats.expansions;
    // Lines 14-16.
    if (x.state == goal) {
      solution_ids.push_back(id);
      continue;
    }
    // Lines 17-24.
    for (EdgeId e = graph.first_arc(x.state); e < graph.end_arc(x.state); ++e) {
      const NodeId t = graph.head(e);
      const CostVec<2>& ht = h[t];
      if (ht[0] == kInfCost) continue;  // t cannot reach the goal
      const Cost* c = graph.costs_of(e);
      const CostVec<2> g{x.g[0] + c[0], x.g[1] + c[1]};
      const Cost f2 = g[1] + ht[1];
      if (g[1] >= g2_min_[t] || f2 >= g2_min_[goal]) continue;
      if (options.max_labels != 0 && nodes_.size() >= options.max_labels) {
        result.stats.status = SearchStatus::kMemout;
        break;
      }
      nodes_.push_back(Node{g, t, id});
      open_.push(CostVec<2>{g[0] + ht[0], f2}, static_cast<std::uint32_t>(nodes_.size() - 1));
      ++result.stats.generated;
    }
    if (result.stats.status != SearchStatus::kOk) break;
  }
  result.stats.runtime_ms = deadline.elapsed_seconds() * 1000.0;
  result.stats.labels_stored = nodes_.size();

  // Solutions are found in increasing g1 and strictly decreasing g2 order.
  result.solutions.reserve(solution_ids.size());
  for (const std::uint32_t id : solution_ids) {
    Solution<2> s{nodes_[id].g, {}};
    if (options.extract_paths) s.path = backtrack(nodes_, id);
    result.solutions.push_back(std::move(s));
  }
  return result;
}

SearchResult<2> boa_star(const Graph& graph, NodeId start, NodeId goal,
                         const SearchOptions& options) {
  const Graph reverse = graph.reversed();
  Dijkstra backward(reverse);
  std::vector<CostVec<2>> h;
  compute_heuristic<2>(backward, goal, graph.num_nodes(), h);
  BoaStar search(graph);
  return search.search(start, goal, h, options);
}

}  // namespace mosp
