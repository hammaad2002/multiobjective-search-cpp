// Bi-Objective A* (BOA*).
//
// C. Hernández, W. Yeoh, J. A. Baier, H. Zhang, L. Suazo, S. Koenig, O. Salzman.
// "Simple and efficient bi-objective search algorithms via fast dominance checks".
// Artificial Intelligence 314 (2023) 103807. Earlier version: C. Hernández Ulloa
// et al., "A Simple and Fast Bi-Objective Search Algorithm", ICAPS 2020.
//
// Implements Algorithm 3 of the journal paper. OPEN is ordered lexicographically
// by (f1, f2). For every state s, g2_min(s) is the smallest g2 of any node
// expanded at s. A node x is pruned, both when generated and (lazily) when
// extracted, if g2(x) >= g2_min(s(x)) or f2(x) >= g2_min(goal). Every
// dominance check is O(1), and no per-state label sets are kept. The result is
// the cost-unique Pareto-optimal solution set.
#pragma once

#include <cstdint>
#include <vector>

#include "mosp/graph.hpp"
#include "mosp/open_list.hpp"
#include "mosp/search.hpp"
#include "mosp/types.hpp"

namespace mosp {

class BoaStar {
 public:
  // `graph` must have exactly two objectives and outlive this object. Buffers
  // are allocated once and reused across search() calls.
  explicit BoaStar(const Graph& graph);

  // h[u] must be a consistent, admissible heuristic towards `goal` for both
  // objectives, with kInfCost marking states that cannot reach the goal.
  [[nodiscard]] SearchResult<2> search(NodeId start, NodeId goal,
                                       const std::vector<CostVec<2>>& h,
                                       const SearchOptions& options = {});

 private:
  struct Node {
    CostVec<2> g;
    NodeId state;
    std::uint32_t parent;
  };

  const Graph* graph_;
  std::vector<std::uint64_t> totals_;  // per-objective total arc cost
  std::vector<Cost> g2_min_;
  std::vector<Node> nodes_;
  BinaryHeap<CostVec<2>> open_;
};

// Convenience wrapper: computes the exact heuristic and runs BOA*.
[[nodiscard]] SearchResult<2> boa_star(const Graph& graph, NodeId start, NodeId goal,
                                       const SearchOptions& options = {});

}  // namespace mosp
