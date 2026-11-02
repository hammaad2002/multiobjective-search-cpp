#include <gtest/gtest.h>

#include <algorithm>
#include <stdexcept>
#include <tuple>
#include <vector>

#include "mosp/dimacs.hpp"
#include "mosp/graph.hpp"

namespace mosp {
namespace {

constexpr const char* kDist =
    "c tiny test graph\n"
    "p sp 4 5\n"
    "a 1 2 10\n"
    "a 1 3 4\n"
    "a 2 4 1\n"
    "a 3 4 20\n"
    "a 3 2 2\n";

// Same arcs, different weights, different order.
constexpr const char* kTimeShuffled =
    "p sp 4 5\n"
    "a 3 2 9\n"
    "a 1 3 7\n"
    "a 3 4 1\n"
    "a 1 2 3\n"
    "a 2 4 5\n";

TEST(Dimacs, ParsesGr) {
  const DimacsGraph g = parse_gr(kDist);
  EXPECT_EQ(g.num_nodes, 4U);
  ASSERT_EQ(g.tails.size(), 5U);
  EXPECT_EQ(g.tails[0], 0U);
  EXPECT_EQ(g.heads[0], 1U);
  EXPECT_EQ(g.weights[0], 10U);
  EXPECT_EQ(g.tails[4], 2U);
  EXPECT_EQ(g.heads[4], 1U);
  EXPECT_EQ(g.weights[4], 2U);
}

TEST(Dimacs, HandlesCrlfAndBlankLines) {
  const DimacsGraph g = parse_gr("c x\r\n\r\np sp 2 1\r\na 1 2 7\r\n");
  ASSERT_EQ(g.tails.size(), 1U);
  EXPECT_EQ(g.weights[0], 7U);
}

TEST(Dimacs, RejectsMalformedInput) {
  EXPECT_THROW((void)parse_gr("a 1 2 3\n"), std::runtime_error);                // no header
  EXPECT_THROW((void)parse_gr("p sp 2 1\na 1 3 3\n"), std::runtime_error);      // bad node id
  EXPECT_THROW((void)parse_gr("p sp 2 2\na 1 2 3\n"), std::runtime_error);      // arc count
  EXPECT_THROW((void)parse_gr("p sp 2 1\na 1 2 x\n"), std::runtime_error);      // bad weight
  EXPECT_THROW((void)parse_gr("p sp 2 1\nq 1 2 3\n"), std::runtime_error);      // bad line type
}

TEST(Dimacs, ParsesCo) {
  const DimacsCoords c = parse_co(
      "c coords\n"
      "p aux sp co 2\n"
      "v 2 -73530767 41085396\n"
      "v 1 -73530538 41086098\n");
  ASSERT_EQ(c.x.size(), 2U);
  EXPECT_EQ(c.x[0], -73530538);
  EXPECT_EQ(c.y[0], 41086098);
  EXPECT_EQ(c.x[1], -73530767);
  EXPECT_THROW((void)parse_co("p aux sp co 2\nv 1 0 0\n"), std::runtime_error);  // missing node
}

TEST(Graph, CsrLayoutAndReverse) {
  const DimacsGraph d = parse_gr(kDist);
  const DimacsGraph* parts[] = {&d};
  const Graph g = combine_objectives(parts, /*add_hop_objective=*/false);
  EXPECT_EQ(g.num_nodes(), 4U);
  EXPECT_EQ(g.num_arcs(), 5U);
  EXPECT_EQ(g.num_objectives(), 1U);
  // Node 0 (DIMACS 1) has arcs to 1 and 2, in file order.
  ASSERT_EQ(g.end_arc(0) - g.first_arc(0), 2U);
  EXPECT_EQ(g.head(g.first_arc(0)), 1U);
  EXPECT_EQ(g.cost(g.first_arc(0), 0), 10U);
  EXPECT_EQ(g.head(g.first_arc(0) + 1), 2U);
  EXPECT_EQ(g.end_arc(3) - g.first_arc(3), 0U);

  const Graph r = g.reversed();
  EXPECT_EQ(r.num_arcs(), 5U);
  // Node 3 (DIMACS 4) has incoming arcs from 1 (w=1) and 2 (w=20).
  ASSERT_EQ(r.end_arc(3) - r.first_arc(3), 2U);
  std::vector<std::pair<NodeId, Cost>> in;
  for (EdgeId e = r.first_arc(3); e < r.end_arc(3); ++e) in.emplace_back(r.head(e), r.cost(e, 0));
  std::sort(in.begin(), in.end());
  EXPECT_EQ(in[0], (std::pair<NodeId, Cost>{1, 1}));
  EXPECT_EQ(in[1], (std::pair<NodeId, Cost>{2, 20}));
}

TEST(Graph, CombinesObjectivesAcrossArcOrders) {
  const DimacsGraph d = parse_gr(kDist);
  const DimacsGraph t = parse_gr(kTimeShuffled);
  const DimacsGraph* parts[] = {&d, &t};
  const Graph g = combine_objectives(parts, /*add_hop_objective=*/true);
  ASSERT_EQ(g.num_objectives(), 3U);
  // Every arc must carry (dist, time, 1) from the matching DIMACS lines.
  const std::vector<std::tuple<NodeId, NodeId, Cost, Cost>> expected = {
      {0, 1, 10, 3}, {0, 2, 4, 7}, {1, 3, 1, 5}, {2, 3, 20, 1}, {2, 1, 2, 9}};
  std::size_t found = 0;
  for (NodeId u = 0; u < g.num_nodes(); ++u) {
    for (EdgeId e = g.first_arc(u); e < g.end_arc(u); ++e) {
      const auto it = std::find(expected.begin(), expected.end(),
                                std::make_tuple(u, g.head(e), g.cost(e, 0), g.cost(e, 1)));
      EXPECT_NE(it, expected.end()) << "unexpected arc " << u << "->" << g.head(e);
      EXPECT_EQ(g.cost(e, 2), 1U);
      ++found;
    }
  }
  EXPECT_EQ(found, expected.size());
}

TEST(Graph, RejectsDifferentArcSets) {
  const DimacsGraph d = parse_gr(kDist);
  const DimacsGraph other = parse_gr("p sp 4 5\na 1 2 1\na 1 3 1\na 2 4 1\na 3 4 1\na 4 2 1\n");
  const DimacsGraph* parts[] = {&d, &other};
  EXPECT_THROW((void)combine_objectives(parts, false), std::runtime_error);
}

TEST(Graph, CostRangeGuard) {
  // Total arc cost 5e9 does not fit in 32 bits.
  const DimacsGraph big = parse_gr("p sp 2 2\na 1 2 3000000000\na 2 1 2000000000\n");
  const DimacsGraph* parts[] = {&big};
  EXPECT_THROW(check_cost_range(combine_objectives(parts, false)), std::overflow_error);
  // Total 3e9 fits on its own.
  const DimacsGraph fits = parse_gr("p sp 2 2\na 1 2 3000000000\na 2 1 5\n");
  const DimacsGraph* fits_parts[] = {&fits};
  EXPECT_NO_THROW(check_cost_range(combine_objectives(fits_parts, false)));
  const DimacsGraph d = parse_gr(kDist);
  const DimacsGraph* ok[] = {&d};
  EXPECT_NO_THROW(check_cost_range(combine_objectives(ok, false)));
}

}  // namespace
}  // namespace mosp
