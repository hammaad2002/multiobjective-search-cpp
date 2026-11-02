// Directed multi-objective graph in compressed sparse row (CSR) form.
//
// Outgoing arcs of node u occupy the index range [offsets[u], offsets[u+1]).
// For arc e, heads[e] is its head node and costs[e*k .. e*k+k) its cost vector
// (objectives interleaved so one arc's costs share a cache line).
#pragma once

#include <cstddef>
#include <span>
#include <vector>

#include "mosp/types.hpp"

namespace mosp {

class Graph {
 public:
  Graph() = default;

  // Builds a CSR graph from an arc list. tails/heads have one entry per arc,
  // costs has num_objectives entries per arc (arc-major). Arcs keep their input
  // order within each tail's adjacency range. Node ids are 0-based.
  static Graph from_arcs(NodeId num_nodes, unsigned num_objectives,
                         std::span<const NodeId> tails, std::span<const NodeId> heads,
                         std::span<const Cost> costs);

  // Returns the graph with every arc reversed (same costs).
  [[nodiscard]] Graph reversed() const;

  [[nodiscard]] NodeId num_nodes() const noexcept { return num_nodes_; }
  [[nodiscard]] std::size_t num_arcs() const noexcept { return heads_.size(); }
  [[nodiscard]] unsigned num_objectives() const noexcept { return k_; }

  [[nodiscard]] EdgeId first_arc(NodeId u) const noexcept { return offsets_[u]; }
  [[nodiscard]] EdgeId end_arc(NodeId u) const noexcept { return offsets_[u + 1]; }
  [[nodiscard]] NodeId head(EdgeId e) const noexcept { return heads_[e]; }
  [[nodiscard]] Cost cost(EdgeId e, unsigned objective) const noexcept {
    return costs_[static_cast<std::size_t>(e) * k_ + objective];
  }
  // Pointer to the k costs of arc e. Valid while the graph is alive.
  [[nodiscard]] const Cost* costs_of(EdgeId e) const noexcept {
    return costs_.data() + static_cast<std::size_t>(e) * k_;
  }

  template <std::size_t K>
  [[nodiscard]] CostVec<K> cost_vec(EdgeId e) const noexcept {
    CostVec<K> c{};
    const Cost* p = costs_of(e);
    for (std::size_t i = 0; i < K; ++i) c[i] = p[i];
    return c;
  }

  // Sum of all arc costs for one objective. Every simple path costs at most this.
  [[nodiscard]] std::uint64_t total_cost(unsigned objective) const noexcept;

 private:
  NodeId num_nodes_ = 0;
  unsigned k_ = 0;
  std::vector<EdgeId> offsets_;  // size num_nodes + 1
  std::vector<NodeId> heads_;    // size num_arcs
  std::vector<Cost> costs_;      // size num_arcs * k
};

}  // namespace mosp
