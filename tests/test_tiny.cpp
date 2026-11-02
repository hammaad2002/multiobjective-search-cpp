// Hand-built graphs with Pareto fronts worked out by hand. Every algorithm
// in solvers.hpp must reproduce them exactly, and so must brute force.
#include <gtest/gtest.h>

#include <vector>

#include "brute_force.hpp"
#include "solvers.hpp"
#include "test_util.hpp"

namespace mosp {
namespace {

template <std::size_t K>
void expect_front(const Graph& g, NodeId s, NodeId t, const std::vector<CostVec<K>>& expected) {
  ASSERT_EQ((test::brute_force_front<K>(g, s, t)), expected) << "brute force disagrees";
  for (const auto& solver : test::all_solvers<K>()) {
    const SearchResult<K> r = solver.run(g, s, t);
    test::check_result_structure<K>(g, s, t, r, solver.name);
    EXPECT_EQ(r.front(), expected) << solver.name;
  }
}

using F2 = std::vector<CostVec<2>>;
using F3 = std::vector<CostVec<3>>;

TEST(TinyGraphs, Diamond) {
  // Two 2-arc routes and one direct arc; all three are Pareto-optimal.
  const Graph g = test::make_graph<2>(4, {{0, 1, {1, 5}},
                                          {1, 3, {1, 5}},
                                          {0, 2, {5, 1}},
                                          {2, 3, {5, 1}},
                                          {0, 3, {4, 4}}});
  expect_front<2>(g, 0, 3, F2{{2, 10}, {4, 4}, {10, 2}});
}

TEST(TinyGraphs, DominatedRoutesAreDropped) {
  const Graph g = test::make_graph<2>(4, {{0, 1, {1, 1}},
                                          {1, 3, {1, 1}},   // (2,2) dominates everything else
                                          {0, 2, {1, 2}},
                                          {2, 3, {1, 1}},   // (2,3)
                                          {0, 3, {2, 2}},   // equal to (2,2): duplicate
                                          {0, 3, {9, 9}}});
  expect_front<2>(g, 0, 3, F2{{2, 2}});
}

TEST(TinyGraphs, EqualCostRoutesYieldOneSolution) {
  // Three different paths all cost (3,3), plus one trade-off path.
  const Graph g = test::make_graph<2>(5, {{0, 1, {1, 1}},
                                          {1, 4, {2, 2}},
                                          {0, 2, {2, 2}},
                                          {2, 4, {1, 1}},
                                          {0, 3, {1, 2}},
                                          {3, 4, {2, 1}},
                                          {0, 4, {1, 7}}});
  expect_front<2>(g, 0, 4, F2{{1, 7}, {3, 3}});
}

TEST(TinyGraphs, UnreachableGoal) {
  const Graph g = test::make_graph<2>(4, {{0, 1, {1, 1}}, {1, 0, {1, 1}}, {3, 2, {1, 1}}});
  expect_front<2>(g, 0, 2, F2{});
}

TEST(TinyGraphs, StartEqualsGoal) {
  const Graph g = test::make_graph<2>(3, {{0, 1, {1, 1}}, {1, 0, {1, 1}}, {1, 2, {1, 1}}});
  expect_front<2>(g, 1, 1, F2{{0, 0}});
}

TEST(TinyGraphs, ZeroCostCycleAndSelfLoop) {
  const Graph g = test::make_graph<2>(3, {{0, 1, {0, 0}},
                                          {1, 0, {0, 0}},
                                          {1, 1, {0, 0}},
                                          {0, 0, {1, 0}},
                                          {1, 2, {1, 2}},
                                          {0, 2, {2, 1}}});
  expect_front<2>(g, 0, 2, F2{{1, 2}, {2, 1}});
}

TEST(TinyGraphs, ParallelArcs) {
  const Graph g = test::make_graph<2>(2, {{0, 1, {3, 1}}, {0, 1, {1, 3}}, {0, 1, {2, 2}},
                                          {0, 1, {3, 3}}});
  expect_front<2>(g, 0, 1, F2{{1, 3}, {2, 2}, {3, 1}});
}

TEST(TinyGraphs, ChainOfDiamondsHasExponentialFront) {
  // Diamond j offers (2^j, 0) or (0, 2^j); the 2^4 subsets give 16 distinct
  // Pareto-optimal costs (a, 15 - a).
  const Graph g = test::make_graph<2>(9, {{0, 1, {1, 0}}, {0, 2, {0, 1}}, {1, 3, {0, 0}},
                                          {2, 3, {0, 0}}, {3, 4, {2, 0}}, {3, 5, {0, 2}},
                                          {4, 6, {0, 0}}, {5, 6, {0, 0}}, {6, 7, {4, 0}},
                                          {6, 7, {0, 4}}, {7, 8, {8, 0}}, {7, 8, {0, 8}}});
  F2 expected;
  for (Cost a = 0; a < 16; ++a) expected.push_back({a, 15 - a});
  expect_front<2>(g, 0, 8, expected);
}

TEST(TinyGraphs, ThreeObjectives) {
  const Graph g = test::make_graph<3>(4, {{0, 3, {1, 2, 3}},
                                          {0, 3, {3, 2, 1}},
                                          {0, 1, {1, 1, 1}},
                                          {1, 3, {1, 1, 1}},  // (2,2,2)
                                          {0, 2, {1, 2, 2}},
                                          {2, 3, {1, 1, 1}},  // (2,3,3): dominated by (2,2,2)
                                          {0, 3, {2, 1, 9}}});
  expect_front<3>(g, 0, 3, F3{{1, 2, 3}, {2, 1, 9}, {2, 2, 2}, {3, 2, 1}});
}

TEST(TinyGraphs, ThreeObjectivesTiesInFirstObjective) {
  // All solutions share the same first objective, so only objectives 2..3
  // separate them (the regime where truncated dominance checks matter).
  const Graph g = test::make_graph<3>(5, {{0, 1, {1, 1, 4}},
                                          {1, 4, {1, 1, 4}},  // (2,2,8)
                                          {0, 2, {1, 4, 1}},
                                          {2, 4, {1, 4, 1}},  // (2,8,2)
                                          {0, 3, {1, 3, 3}},
                                          {3, 4, {1, 3, 3}},  // (2,6,6)
                                          {0, 4, {2, 7, 7}}});  // dominated by (2,6,6)
  expect_front<3>(g, 0, 4, F3{{2, 2, 8}, {2, 6, 6}, {2, 8, 2}});
}

}  // namespace
}  // namespace mosp
