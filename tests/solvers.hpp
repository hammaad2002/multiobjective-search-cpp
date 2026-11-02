// The list of algorithms under test, with a uniform calling convention, plus
// structural checks every result must satisfy.
#pragma once

#include <gtest/gtest.h>

#include <functional>
#include <string>
#include <vector>

#include "brute_force.hpp"
#include "mosp/graph.hpp"
#include "mosp/martins.hpp"
#include "mosp/search.hpp"

namespace mosp::test {

template <std::size_t K>
struct NamedSolver {
  std::string name;
  std::function<SearchResult<K>(const Graph&, NodeId, NodeId)> run;
};

template <std::size_t K>
std::vector<NamedSolver<K>> all_solvers() {
  std::vector<NamedSolver<K>> solvers;
  solvers.push_back({"martins", [](const Graph& g, NodeId s, NodeId t) {
                       return martins<K>(g, s, t);
                     }});
  return solvers;
}

// Checks that a completed result is a valid set of Pareto-optimal solutions:
// sorted lexicographically, mutually non-dominated, no duplicates, and each
// path is an s-t path whose arcs sum to the reported cost.
template <std::size_t K>
void check_result_structure(const Graph& g, NodeId s, NodeId t, const SearchResult<K>& r,
                            const std::string& context) {
  ASSERT_EQ(r.stats.status, SearchStatus::kOk) << context;
  const auto& sols = r.solutions;
  for (std::size_t i = 0; i < sols.size(); ++i) {
    if (i > 0) {
      ASSERT_LT(sols[i - 1].cost, sols[i].cost) << context << ": not sorted/unique";
    }
    for (std::size_t j = 0; j < sols.size(); ++j) {
      if (i == j) continue;
      ASSERT_FALSE(weakly_dominates(sols[j].cost, sols[i].cost)) << context << ": dominated";
    }
    const auto& p = sols[i].path;
    ASSERT_FALSE(p.empty()) << context;
    ASSERT_EQ(p.front(), s) << context;
    ASSERT_EQ(p.back(), t) << context;
    ASSERT_TRUE(path_has_cost(g, p, sols[i].cost)) << context << ": path cost mismatch";
  }
}

}  // namespace mosp::test
