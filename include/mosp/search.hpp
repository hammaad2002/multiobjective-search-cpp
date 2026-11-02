// Options, statistics and results shared by all multi-objective searches.
#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "mosp/types.hpp"

namespace mosp {

enum class SearchStatus {
  kOk,       // search completed; the returned front is the full Pareto front
  kTimeout,  // time limit hit; the returned front is partial
  kMemout,   // label limit hit; the returned front is partial
};

[[nodiscard]] constexpr const char* to_string(SearchStatus s) noexcept {
  switch (s) {
    case SearchStatus::kOk: return "ok";
    case SearchStatus::kTimeout: return "timeout";
    case SearchStatus::kMemout: return "memout";
  }
  return "?";
}

struct SearchOptions {
  double timeout_seconds = 0.0;  // <= 0: no time limit
  std::size_t max_labels = 0;    // 0: no limit on stored labels
  bool extract_paths = true;     // reconstruct node sequences for solutions
};

struct SearchStats {
  SearchStatus status = SearchStatus::kOk;
  // Labels removed from OPEN that survived pruning and were expanded (goal
  // labels, which are recorded as solutions instead of expanded, included).
  std::uint64_t expansions = 0;
  // Labels inserted into OPEN, including the start label.
  std::uint64_t generated = 0;
  std::size_t labels_stored = 0;  // size of the label pool at the end
  double runtime_ms = 0.0;        // search only (heuristic time excluded)
};

template <std::size_t K>
struct Solution {
  CostVec<K> cost;
  std::vector<NodeId> path;  // start ... goal; empty if paths were not extracted
};

template <std::size_t K>
struct SearchResult {
  std::vector<Solution<K>> solutions;  // one per Pareto-optimal cost vector, lexicographic order
  SearchStats stats;

  [[nodiscard]] std::vector<CostVec<K>> front() const {
    std::vector<CostVec<K>> f;
    f.reserve(solutions.size());
    for (const auto& s : solutions) f.push_back(s.cost);
    return f;
  }
};

// Wall-clock limit that is cheap to poll.
class Deadline {
 public:
  explicit Deadline(double seconds) : start_(Clock::now()), limit_(seconds) {}
  [[nodiscard]] bool expired() const {
    return limit_ > 0.0 && elapsed_seconds() >= limit_;
  }
  [[nodiscard]] double elapsed_seconds() const {
    return std::chrono::duration<double>(Clock::now() - start_).count();
  }

 private:
  using Clock = std::chrono::steady_clock;
  Clock::time_point start_;
  double limit_;
};

// How often (in OPEN extractions) searches poll the clock.
inline constexpr std::uint64_t kDeadlinePollInterval = 1024;

// Follows parent links from label `id` back to the root and returns the node
// sequence root..id. Labels must expose `state` and `parent` (kNoParent at root).
inline constexpr std::uint32_t kNoParent = 0xFFFFFFFFu;

template <typename Labels>
[[nodiscard]] std::vector<NodeId> backtrack(const Labels& labels, std::uint32_t id) {
  std::vector<NodeId> path;
  for (std::uint32_t cur = id; cur != kNoParent; cur = labels[cur].parent) {
    path.push_back(labels[cur].state);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

}  // namespace mosp
