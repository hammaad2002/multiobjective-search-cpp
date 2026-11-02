// Randomized cross-checks. Martins is first validated against brute-force
// enumeration on very small graphs; every other algorithm must then
// reproduce Martins' Pareto front exactly on larger random graphs.
#include <gtest/gtest.h>

#include <random>
#include <string>

#include "brute_force.hpp"
#include "mosp/martins.hpp"
#include "solvers.hpp"
#include "test_util.hpp"

namespace mosp {
namespace {

test::RandomGraphParams params_for_seed(std::uint64_t seed, NodeId min_nodes, NodeId max_nodes) {
  std::mt19937_64 rng(seed * 0x9E3779B97F4A7C15ULL + 7);
  test::RandomGraphParams p;
  p.num_nodes = std::uniform_int_distribution<NodeId>(min_nodes, max_nodes)(rng);
  p.num_arcs = p.num_nodes * std::uniform_int_distribution<std::size_t>(1, 4)(rng);
  p.min_cost = (seed % 5 == 0) ? 0 : 1;  // some graphs have zero-cost arcs
  p.max_cost = (seed % 7 == 0) ? 3 : 20;  // small ranges create many ties
  p.anticorrelated = (seed % 2 == 0);
  return p;
}

template <std::size_t K>
void martins_vs_brute_force(std::uint64_t first_seed, std::uint64_t num_seeds) {
  std::size_t nonempty = 0;
  for (std::uint64_t seed = first_seed; seed < first_seed + num_seeds; ++seed) {
    const auto p = params_for_seed(seed, 2, 8);
    const Graph g = test::random_graph<K>(seed, p);
    const NodeId s = static_cast<NodeId>(seed % p.num_nodes);
    const NodeId t = static_cast<NodeId>((seed / 3) % p.num_nodes);
    const std::string ctx = "K=" + std::to_string(K) + " seed=" + std::to_string(seed);
    const SearchResult<K> r = martins<K>(g, s, t);
    test::check_result_structure<K>(g, s, t, r, ctx);
    ASSERT_EQ(r.front(), (test::brute_force_front<K>(g, s, t))) << ctx;
    nonempty += r.solutions.empty() ? 0 : 1;
  }
  // Guard against a generator that only produces trivial instances.
  EXPECT_GT(nonempty, num_seeds / 2);
}

TEST(Random, MartinsMatchesBruteForceK2) { martins_vs_brute_force<2>(1, 1000); }
TEST(Random, MartinsMatchesBruteForceK3) { martins_vs_brute_force<3>(5001, 1000); }

template <std::size_t K>
void solvers_vs_martins(std::uint64_t first_seed, std::uint64_t num_seeds) {
  const auto solvers = test::all_solvers<K>();
  std::size_t max_front = 0;
  for (std::uint64_t seed = first_seed; seed < first_seed + num_seeds; ++seed) {
    const auto p = params_for_seed(seed, 10, 60);
    const Graph g = test::random_graph<K>(seed, p);
    const NodeId s = static_cast<NodeId>(seed % p.num_nodes);
    const NodeId t = static_cast<NodeId>((seed * 7 + 3) % p.num_nodes);
    const auto reference = martins<K>(g, s, t).front();
    max_front = std::max(max_front, reference.size());
    for (const auto& solver : solvers) {
      if (solver.name == "martins") continue;
      const std::string ctx =
          solver.name + " K=" + std::to_string(K) + " seed=" + std::to_string(seed);
      const SearchResult<K> r = solver.run(g, s, t);
      test::check_result_structure<K>(g, s, t, r, ctx);
      ASSERT_EQ(r.front(), reference) << ctx;
    }
  }
  // The instances must be non-trivial for the comparison to mean anything.
  EXPECT_GE(max_front, 5U);
}

TEST(Random, AllSolversMatchMartinsK2) { solvers_vs_martins<2>(10001, 500); }
TEST(Random, AllSolversMatchMartinsK3) { solvers_vs_martins<3>(20001, 500); }

}  // namespace
}  // namespace mosp
