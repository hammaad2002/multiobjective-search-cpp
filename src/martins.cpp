#include "mosp/martins.hpp"

#include <stdexcept>
#include <vector>

#include "mosp/dimacs.hpp"
#include "mosp/open_list.hpp"

namespace mosp {

template <std::size_t K>
SearchResult<K> martins(const Graph& graph, NodeId source, NodeId target,
                        const SearchOptions& options) {
  if (graph.num_objectives() != K) throw std::invalid_argument("martins: objective count mismatch");
  if (source >= graph.num_nodes() || target >= graph.num_nodes()) {
    throw std::out_of_range("martins: node out of range");
  }
  check_cost_range(graph);

  struct Label {
    NodeId state;
    std::uint32_t parent;
    CostVec<K> g;
  };
  std::vector<Label> labels;
  std::vector<std::vector<std::uint32_t>> permanent(graph.num_nodes());
  BinaryHeap<CostVec<K>> queue;
  SearchResult<K> result;
  const Deadline deadline(options.timeout_seconds);

  auto dominated_by_permanent = [&](NodeId v, const CostVec<K>& g) {
    for (const std::uint32_t id : permanent[v]) {
      if (weakly_dominates(labels[id].g, g)) return true;
    }
    return false;
  };

  labels.push_back(Label{source, kNoParent, CostVec<K>{}});
  queue.push(CostVec<K>{}, 0);
  result.stats.generated = 1;

  std::uint64_t pops = 0;
  while (!queue.empty()) {
    if (++pops % kDeadlinePollInterval == 0 && deadline.expired()) {
      result.stats.status = SearchStatus::kTimeout;
      break;
    }
    const std::uint32_t id = queue.pop().id;
    const Label cur = labels[id];
    if (dominated_by_permanent(cur.state, cur.g)) continue;
    permanent[cur.state].push_back(id);
    ++result.stats.expansions;

    for (EdgeId e = graph.first_arc(cur.state); e < graph.end_arc(cur.state); ++e) {
      const NodeId v = graph.head(e);
      const CostVec<K> g = add(cur.g, graph.cost_vec<K>(e));
      if (dominated_by_permanent(v, g)) continue;
      if (options.max_labels != 0 && labels.size() >= options.max_labels) {
        result.stats.status = SearchStatus::kMemout;
        break;
      }
      labels.push_back(Label{v, id, g});
      queue.push(g, static_cast<std::uint32_t>(labels.size() - 1));
      ++result.stats.generated;
    }
    if (result.stats.status != SearchStatus::kOk) break;
  }

  // Permanent labels at the target were made permanent in lexicographic order
  // and none weakly dominates a later one, so they form the Pareto front.
  for (const std::uint32_t id : permanent[target]) {
    Solution<K> s{labels[id].g, {}};
    if (options.extract_paths) s.path = backtrack(labels, id);
    result.solutions.push_back(std::move(s));
  }
  result.stats.labels_stored = labels.size();
  result.stats.runtime_ms = deadline.elapsed_seconds() * 1000.0;
  return result;
}

template SearchResult<2> martins<2>(const Graph&, NodeId, NodeId, const SearchOptions&);
template SearchResult<3> martins<3>(const Graph&, NodeId, NodeId, const SearchOptions&);

}  // namespace mosp
