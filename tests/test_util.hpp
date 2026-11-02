// Helpers shared by the unit tests: small graph builders and random graphs.
#pragma once

#include <algorithm>
#include <cstddef>
#include <initializer_list>
#include <random>
#include <vector>

#include "mosp/graph.hpp"
#include "mosp/types.hpp"

namespace mosp::test {

template <std::size_t K>
struct ArcSpec {
  NodeId tail;
  NodeId head;
  CostVec<K> cost;
};

template <std::size_t K>
Graph make_graph(NodeId n, std::initializer_list<ArcSpec<K>> arcs) {
  std::vector<NodeId> tails;
  std::vector<NodeId> heads;
  std::vector<Cost> costs;
  for (const auto& a : arcs) {
    tails.push_back(a.tail);
    heads.push_back(a.head);
    costs.insert(costs.end(), a.cost.begin(), a.cost.end());
  }
  return Graph::from_arcs(n, static_cast<unsigned>(K), tails, heads, costs);
}

struct RandomGraphParams {
  NodeId num_nodes = 10;
  std::size_t num_arcs = 30;
  Cost min_cost = 1;
  Cost max_cost = 10;
  // If set, costs of different objectives are drawn so that they tend to
  // conflict (c2 ~ max - c1), which produces larger Pareto fronts.
  bool anticorrelated = false;
};

// Random directed multigraph (self-loops and parallel arcs allowed) with
// independent uniformly random costs per objective.
template <std::size_t K>
Graph random_graph(std::uint64_t seed, const RandomGraphParams& p) {
  std::mt19937_64 rng(seed);
  std::uniform_int_distribution<NodeId> node(0, p.num_nodes - 1);
  std::uniform_int_distribution<Cost> cost(p.min_cost, p.max_cost);
  std::uniform_int_distribution<Cost> noise(0, 2);
  std::vector<NodeId> tails(p.num_arcs);
  std::vector<NodeId> heads(p.num_arcs);
  std::vector<Cost> costs(p.num_arcs * K);
  for (std::size_t i = 0; i < p.num_arcs; ++i) {
    tails[i] = node(rng);
    heads[i] = node(rng);
    const Cost c0 = cost(rng);
    for (std::size_t j = 0; j < K; ++j) {
      Cost c = cost(rng);
      if (p.anticorrelated && j == 1) {
        const Cost flipped = p.max_cost + p.min_cost - c0 + noise(rng);
        c = std::max(p.min_cost, std::min(p.max_cost, flipped));
      }
      costs[i * K + j] = (j == 0) ? c0 : c;
    }
  }
  return Graph::from_arcs(p.num_nodes, static_cast<unsigned>(K), tails, heads, costs);
}

// Bidirectional 4-neighbour grid (rows x cols) with random, conflicting arc
// costs: objective 0 uniform in [1, max_cost], objective 1 roughly
// max_cost + 1 - c0 plus noise, further objectives independent. Grids with
// conflicting costs give the large Pareto fronts typical of road networks.
template <std::size_t K>
Graph random_grid(std::uint64_t seed, NodeId rows, NodeId cols, Cost max_cost) {
  std::mt19937_64 rng(seed);
  std::uniform_int_distribution<Cost> cost(1, max_cost);
  std::uniform_int_distribution<int> noise(-static_cast<int>(max_cost) / 5,
                                           static_cast<int>(max_cost) / 5);
  std::vector<NodeId> tails;
  std::vector<NodeId> heads;
  std::vector<Cost> costs;
  auto add_arc = [&](NodeId u, NodeId v) {
    tails.push_back(u);
    heads.push_back(v);
    const Cost c0 = cost(rng);
    costs.push_back(c0);
    if constexpr (K >= 2) {
      const int c1 = static_cast<int>(max_cost) + 1 - static_cast<int>(c0) + noise(rng);
      costs.push_back(static_cast<Cost>(std::clamp(c1, 1, static_cast<int>(max_cost))));
    }
    for (std::size_t j = 2; j < K; ++j) costs.push_back(cost(rng));
  };
  for (NodeId r = 0; r < rows; ++r) {
    for (NodeId c = 0; c < cols; ++c) {
      const NodeId u = r * cols + c;
      if (c + 1 < cols) {
        add_arc(u, u + 1);
        add_arc(u + 1, u);
      }
      if (r + 1 < rows) {
        add_arc(u, u + cols);
        add_arc(u + cols, u);
      }
    }
  }
  return Graph::from_arcs(rows * cols, static_cast<unsigned>(K), tails, heads, costs);
}

}  // namespace mosp::test
