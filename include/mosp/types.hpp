// Basic types shared by all search algorithms: node ids, costs, cost vectors
// and Pareto-dominance predicates.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace mosp {

using NodeId = std::uint32_t;
using EdgeId = std::uint32_t;
using Cost = std::uint32_t;

inline constexpr NodeId kInvalidNode = std::numeric_limits<NodeId>::max();
// Used as "unreachable" in heuristics and as +infinity in g_min style arrays.
inline constexpr Cost kInfCost = std::numeric_limits<Cost>::max();

template <std::size_t K>
using CostVec = std::array<Cost, K>;

// a weakly dominates b: a_i <= b_i for every objective i (equality allowed).
template <std::size_t K>
[[nodiscard]] constexpr bool weakly_dominates(const CostVec<K>& a, const CostVec<K>& b) noexcept {
  for (std::size_t i = 0; i < K; ++i) {
    if (a[i] > b[i]) return false;
  }
  return true;
}

// a (Pareto-)dominates b: a weakly dominates b and a != b.
template <std::size_t K>
[[nodiscard]] constexpr bool dominates(const CostVec<K>& a, const CostVec<K>& b) noexcept {
  return weakly_dominates(a, b) && a != b;
}

// Weak dominance restricted to objectives 1..K-1 (the "truncated" vector used by
// dimensionality reduction; the first objective is ignored).
template <std::size_t K>
[[nodiscard]] constexpr bool weakly_dominates_truncated(const CostVec<K>& a,
                                                        const CostVec<K>& b) noexcept {
  for (std::size_t i = 1; i < K; ++i) {
    if (a[i] > b[i]) return false;
  }
  return true;
}

template <std::size_t K>
[[nodiscard]] constexpr CostVec<K> add(const CostVec<K>& a, const CostVec<K>& b) noexcept {
  CostVec<K> r{};
  for (std::size_t i = 0; i < K; ++i) r[i] = a[i] + b[i];
  return r;
}

// Reduces a list of cost vectors to its Pareto front: removes dominated vectors
// and duplicates, and returns the remainder sorted lexicographically.
// O(n^2) in the worst case; intended for tests and result post-processing.
template <std::size_t K>
[[nodiscard]] std::vector<CostVec<K>> pareto_filter(std::vector<CostVec<K>> v) {
  std::sort(v.begin(), v.end());
  v.erase(std::unique(v.begin(), v.end()), v.end());
  std::vector<CostVec<K>> front;
  for (const auto& c : v) {
    // After lexicographic sorting, only earlier vectors can dominate c.
    bool dominated = false;
    for (const auto& f : front) {
      if (weakly_dominates(f, c)) {
        dominated = true;
        break;
      }
    }
    if (!dominated) front.push_back(c);
  }
  return front;
}

}  // namespace mosp
