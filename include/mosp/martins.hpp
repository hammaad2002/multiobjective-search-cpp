// Reference multi-objective label-setting algorithm (Martins 1984 style).
//
// Labels are extracted from a queue in lexicographic order of their cost
// vectors g (no heuristic). An extracted label becomes permanent at its node
// unless some permanent label there weakly dominates it; permanent labels are
// expanded. Dominance is always checked with a full pairwise scan over the
// node's permanent labels. The search runs to completion over the whole
// graph (one-to-all), and the Pareto front for `target` is read off at the
// end. This is deliberately simple and slow: it exists to cross-check the
// fast algorithms on small graphs.
#pragma once

#include <cstddef>

#include "mosp/graph.hpp"
#include "mosp/search.hpp"
#include "mosp/types.hpp"

namespace mosp {

template <std::size_t K>
[[nodiscard]] SearchResult<K> martins(const Graph& graph, NodeId source, NodeId target,
                                      const SearchOptions& options = {});

extern template SearchResult<2> martins<2>(const Graph&, NodeId, NodeId, const SearchOptions&);
extern template SearchResult<3> martins<3>(const Graph&, NodeId, NodeId, const SearchOptions&);

}  // namespace mosp
