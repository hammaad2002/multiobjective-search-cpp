#include "mosp/graph.hpp"

#include <limits>
#include <stdexcept>
#include <string>

namespace mosp {

Graph Graph::from_arcs(NodeId num_nodes, unsigned num_objectives, std::span<const NodeId> tails,
                       std::span<const NodeId> heads, std::span<const Cost> costs) {
  if (num_objectives == 0) throw std::invalid_argument("graph needs at least one objective");
  if (tails.size() != heads.size()) throw std::invalid_argument("tails/heads size mismatch");
  if (costs.size() != tails.size() * num_objectives) {
    throw std::invalid_argument("costs must have num_objectives entries per arc");
  }
  if (tails.size() >= std::numeric_limits<EdgeId>::max()) {
    throw std::invalid_argument("too many arcs for 32-bit arc ids");
  }

  Graph g;
  g.num_nodes_ = num_nodes;
  g.k_ = num_objectives;
  const std::size_t m = tails.size();

  // Counting sort by tail; stable, so arcs keep their input order per tail.
  g.offsets_.assign(static_cast<std::size_t>(num_nodes) + 1, 0);
  for (std::size_t i = 0; i < m; ++i) {
    if (tails[i] >= num_nodes || heads[i] >= num_nodes) {
      throw std::out_of_range("arc " + std::to_string(i) + " has an endpoint out of range");
    }
    ++g.offsets_[tails[i] + 1];
  }
  for (NodeId u = 0; u < num_nodes; ++u) g.offsets_[u + 1] += g.offsets_[u];

  g.heads_.resize(m);
  g.costs_.resize(m * num_objectives);
  std::vector<EdgeId> next(g.offsets_.begin(), g.offsets_.end() - 1);
  for (std::size_t i = 0; i < m; ++i) {
    const EdgeId e = next[tails[i]]++;
    g.heads_[e] = heads[i];
    for (unsigned j = 0; j < num_objectives; ++j) {
      g.costs_[static_cast<std::size_t>(e) * num_objectives + j] = costs[i * num_objectives + j];
    }
  }
  return g;
}

Graph Graph::reversed() const {
  const std::size_t m = num_arcs();
  std::vector<NodeId> tails(m);
  std::vector<NodeId> heads(m);
  for (NodeId u = 0; u < num_nodes_; ++u) {
    for (EdgeId e = first_arc(u); e < end_arc(u); ++e) {
      tails[e] = heads_[e];
      heads[e] = u;
    }
  }
  return from_arcs(num_nodes_, k_, tails, heads, costs_);
}

std::uint64_t Graph::total_cost(unsigned objective) const noexcept {
  std::uint64_t sum = 0;
  for (std::size_t e = 0; e < num_arcs(); ++e) sum += costs_[e * k_ + objective];
  return sum;
}

}  // namespace mosp
