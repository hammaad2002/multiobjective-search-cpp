#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "mosp/heuristic.hpp"
#include "test_util.hpp"

namespace mosp {
namespace {

TEST(Dijkstra, TinyGraph) {
  // 0 -> 1 -> 3 is shortest in objective 0, 0 -> 2 -> 3 in objective 1.
  const Graph g = test::make_graph<2>(5, {{0, 1, {1, 10}},
                                          {1, 3, {1, 10}},
                                          {0, 2, {5, 1}},
                                          {2, 3, {5, 1}},
                                          {3, 0, {1, 1}}});
  Dijkstra fwd(g);
  const std::vector<Cost> d0 = fwd.run(0, 0);
  EXPECT_EQ(d0[3], 2U);
  EXPECT_EQ(d0[2], 5U);
  EXPECT_EQ(d0[4], kInfCost);  // isolated node
  const std::vector<Cost> d1 = fwd.run(0, 1);
  EXPECT_EQ(d1[3], 2U);
  EXPECT_EQ(d1[1], 10U);
}

TEST(Heuristic, ZeroAtGoalAndInfWhenUnreachable) {
  const Graph g = test::make_graph<2>(4, {{0, 1, {3, 4}}, {1, 2, {1, 1}}, {2, 1, {1, 1}}});
  const Graph r = g.reversed();
  Dijkstra bwd(r);
  std::vector<CostVec<2>> h;
  compute_heuristic<2>(bwd, 2, g.num_nodes(), h);
  EXPECT_EQ(h[2], (CostVec<2>{0, 0}));
  EXPECT_EQ(h[1], (CostVec<2>{1, 1}));
  EXPECT_EQ(h[0], (CostVec<2>{4, 5}));
  EXPECT_EQ(h[3], (CostVec<2>{kInfCost, kInfCost}));
}

// Backward Dijkstra on the reversed graph must equal forward Dijkstra from
// every node to the goal, and the result must be consistent on every arc.
TEST(Heuristic, MatchesForwardDijkstraAndIsConsistent) {
  for (std::uint64_t seed = 1; seed <= 200; ++seed) {
    test::RandomGraphParams p;
    p.num_nodes = 5 + static_cast<NodeId>(seed % 40);
    p.num_arcs = p.num_nodes * (1 + seed % 4);
    p.min_cost = (seed % 3 == 0) ? 0 : 1;  // include zero-cost arcs sometimes
    p.max_cost = 20;
    const Graph g = test::random_graph<3>(seed, p);
    const Graph r = g.reversed();
    Dijkstra fwd(g);
    Dijkstra bwd(r);
    const NodeId goal = static_cast<NodeId>(seed % p.num_nodes);
    std::vector<CostVec<3>> h;
    compute_heuristic<3>(bwd, goal, g.num_nodes(), h);

    for (NodeId u = 0; u < g.num_nodes(); ++u) {
      for (unsigned i = 0; i < 3; ++i) {
        ASSERT_EQ(h[u][i], fwd.run(u, i)[goal]) << "seed " << seed << " node " << u;
      }
    }
    for (NodeId u = 0; u < g.num_nodes(); ++u) {
      for (EdgeId e = g.first_arc(u); e < g.end_arc(u); ++e) {
        const NodeId v = g.head(e);
        for (unsigned i = 0; i < 3; ++i) {
          if (h[v][i] == kInfCost) continue;
          ASSERT_LE(h[u][i], g.cost(e, i) + h[v][i]) << "inconsistent at seed " << seed;
        }
      }
    }
  }
}


// g is bounded by the total arc cost (3e9 here) and h(0) = 3e9 towards goal 1,
// so g + h could reach 6e9 and the searches must refuse the query.
TEST(Heuristic, RangeGuardRejectsOverflowingQueries) {
  const Graph g = test::make_graph<2>(2, {{0, 1, {3000000000u, 1}}, {1, 0, {5, 1}}});
  const Graph r = g.reversed();
  Dijkstra bwd(r);
  std::vector<CostVec<2>> h;
  compute_heuristic<2>(bwd, 1, g.num_nodes(), h);
  EXPECT_THROW(check_heuristic_range<2>(total_costs(g), h), std::overflow_error);
  compute_heuristic<2>(bwd, 0, g.num_nodes(), h);  // h(1) = 5: fine
  EXPECT_NO_THROW(check_heuristic_range<2>(total_costs(g), h));
}

}  // namespace
}  // namespace mosp
