#include "mosp/namoa_dr.hpp"

#include <algorithm>
#include <cassert>
#include <stdexcept>

#include "mosp/dimacs.hpp"
#include "mosp/heuristic.hpp"

namespace mosp {

template <std::size_t K>
NamoaDr<K>::NamoaDr(const Graph& graph)
    : graph_(&graph), op_head_(graph.num_nodes(), kNil) {
  if (graph.num_objectives() != K) {
    throw std::invalid_argument("NAMOA*dr: graph objective count does not match K");
  }
  check_cost_range(graph);
  totals_ = total_costs(graph);
  if constexpr (K == 2) {
    g2_min_.assign(graph.num_nodes(), kInfCost);
  } else {
    cl_head_.assign(graph.num_nodes(), kNil);
  }
}

template <std::size_t K>
bool NamoaDr<K>::closed_dominates(NodeId s, const CostVec<K>& v) const {
  if constexpr (K == 2) {
    return g2_min_[s] <= v[1];
  } else {
    for (std::uint32_t c = cl_head_[s]; c != kNil; c = labels_[c].next) {
      if (weakly_dominates_truncated(labels_[c].g, v)) return true;
    }
    return false;
  }
}

template <std::size_t K>
void NamoaDr<K>::add_closed(NodeId s, std::uint32_t id) {
  labels_[id].status = Status::kClosed;
  if constexpr (K == 2) {
    g2_min_[s] = labels_[id].g[1];
  } else {
    // Drop truncated vectors the new one weakly dominates; any later vector
    // they would prune is also pruned by the new one (its g1 is no smaller).
    const CostVec<K>& g = labels_[id].g;
    std::uint32_t* link = &cl_head_[s];
    while (*link != kNil) {
      Label& c = labels_[*link];
      if (weakly_dominates_truncated(g, c.g)) {
        *link = c.next;
      } else {
        link = &c.next;
      }
    }
    labels_[id].next = cl_head_[s];
    cl_head_[s] = id;
  }
}

template <std::size_t K>
void NamoaDr<K>::unlink_open(std::uint32_t id) {
  Label& l = labels_[id];
  if (l.prev != kNil) {
    labels_[l.prev].next = l.next;
  } else {
    op_head_[l.state] = l.next;
  }
  if (l.next != kNil) labels_[l.next].prev = l.prev;
  l.prev = l.next = kNil;
}

template <std::size_t K>
SearchResult<K> NamoaDr<K>::search(NodeId start, NodeId goal, const std::vector<CostVec<K>>& h,
                                   const SearchOptions& options) {
  const Graph& graph = *graph_;
  if (start >= graph.num_nodes() || goal >= graph.num_nodes()) {
    throw std::out_of_range("NAMOA*dr: node out of range");
  }
  if (h.size() != graph.num_nodes()) {
    throw std::invalid_argument("NAMOA*dr: heuristic size mismatch");
  }
  check_heuristic_range<K>(totals_, h);

  SearchResult<K> result;
  const Deadline deadline(options.timeout_seconds);
  std::fill(op_head_.begin(), op_head_.end(), kNil);
  if constexpr (K == 2) {
    std::fill(g2_min_.begin(), g2_min_.end(), kInfCost);
  } else {
    std::fill(cl_head_.begin(), cl_head_.end(), kNil);
  }
  labels_.clear();
  open_.clear();
  std::vector<std::uint32_t> solution_ids;

  if (h[start][0] != kInfCost) {
    labels_.push_back(Label{CostVec<K>{}, start, kNoParent, kNil, kNil, Status::kOpen});
    op_head_[start] = 0;
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
    if (labels_[id].status == Status::kDead) continue;  // removed from G_op earlier
    const NodeId s = labels_[id].state;
    unlink_open(id);

    // Lazy filtering: discard if a solution (weakly) dominates f.
    if (closed_dominates(goal, f)) {
      labels_[id].status = Status::kDead;
      continue;
    }
    ++result.stats.expansions;
    add_closed(s, id);
    if (s == goal) {
      solution_ids.push_back(id);
      continue;
    }

    const CostVec<K> gs = labels_[id].g;
    for (EdgeId e = graph.first_arc(s); e < graph.end_arc(s); ++e) {
      const NodeId t = graph.head(e);
      const CostVec<K>& ht = h[t];
      if (ht[0] == kInfCost) continue;  // t cannot reach the goal
      const Cost* c = graph.costs_of(e);
      CostVec<K> g;
      CostVec<K> ft;
      for (std::size_t i = 0; i < K; ++i) {
        g[i] = gs[i] + c[i];
        ft[i] = g[i] + ht[i];
      }
      // Truncated checks against G_cl(t) and the solution set.
      if (closed_dominates(t, g) || closed_dominates(goal, ft)) continue;

      // Full check against G_op(t), removing open labels that g dominates.
      // G_op(t) is mutually non-dominated, so if some label weakly dominates
      // g, g cannot dominate any other label of G_op(t); on-the-fly removal
      // is therefore safe.
      bool dominated = false;
      bool removed_any = false;
      for (std::uint32_t o = op_head_[t]; o != kNil;) {
        Label& ol = labels_[o];
        const std::uint32_t next = ol.next;
        if (weakly_dominates(ol.g, g)) {
          dominated = true;
          break;
        }
        if (weakly_dominates(g, ol.g)) {  // strict, since ol.g != g here
          unlink_open(o);
          ol.status = Status::kDead;
          removed_any = true;
        }
        o = next;
      }
      assert(!(dominated && removed_any));
      (void)removed_any;
      if (dominated) continue;

      if (options.max_labels != 0 && labels_.size() >= options.max_labels) {
        result.stats.status = SearchStatus::kMemout;
        break;
      }
      const auto nid = static_cast<std::uint32_t>(labels_.size());
      labels_.push_back(Label{g, t, id, kNil, op_head_[t], Status::kOpen});
      if (op_head_[t] != kNil) labels_[op_head_[t]].prev = nid;
      op_head_[t] = nid;
      open_.push(ft, nid);
      ++result.stats.generated;
    }
    if (result.stats.status != SearchStatus::kOk) break;
  }
  result.stats.runtime_ms = deadline.elapsed_seconds() * 1000.0;
  result.stats.labels_stored = labels_.size();

  // Goal labels are expanded in lexicographic order, each non-dominated by
  // the earlier ones, so solution_ids is already the sorted front.
  result.solutions.reserve(solution_ids.size());
  for (const std::uint32_t sid : solution_ids) {
    Solution<K> sol{labels_[sid].g, {}};
    if (options.extract_paths) sol.path = backtrack(labels_, sid);
    result.solutions.push_back(std::move(sol));
  }
  return result;
}

template <std::size_t K>
SearchResult<K> namoa_dr(const Graph& graph, NodeId start, NodeId goal,
                         const SearchOptions& options) {
  const Graph reverse = graph.reversed();
  Dijkstra backward(reverse);
  std::vector<CostVec<K>> h;
  compute_heuristic<K>(backward, goal, graph.num_nodes(), h);
  NamoaDr<K> search(graph);
  return search.search(start, goal, h, options);
}

template class NamoaDr<2>;
template class NamoaDr<3>;
template SearchResult<2> namoa_dr<2>(const Graph&, NodeId, NodeId, const SearchOptions&);
template SearchResult<3> namoa_dr<3>(const Graph&, NodeId, NodeId, const SearchOptions&);

}  // namespace mosp
