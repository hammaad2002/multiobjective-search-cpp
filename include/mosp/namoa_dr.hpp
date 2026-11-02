// NAMOA* with dimensionality reduction (NAMOA*dr) for K >= 2 objectives.
//
// F.-J. Pulido, L. Mandow, J.-L. Pérez-de-la-Cruz. "Dimensionality reduction in
// multiobjective shortest path search". Computers & Operations Research 64
// (2015) 60-70. Built on NAMOA* (L. Mandow, J.-L. Pérez de la Cruz,
// "Multiobjective A* search with consistent heuristics", J. ACM 57(5), 2010).
//
// The bi-objective case follows Algorithm 2 of Hernández et al. (AIJ 2023);
// for K > 2 the same structure is used with (K-1)-dimensional truncated
// vectors, as in Pulido et al.:
//
//  * OPEN is ordered lexicographically by f. With a consistent heuristic, a
//    label extracted at state s has g1 no smaller than any label previously
//    expanded at s, and f1 no smaller than any solution found so far.
//    Dominance against G_cl(s) and against the solution set therefore only
//    needs the truncated vectors (objectives 2..K): Tr(g') <= Tr(g) and
//    g1' <= g1 imply g' <= g.
//  * G_cl(s) keeps the non-dominated truncated g-vectors of labels expanded at
//    s (a single value g2_min(s) when K = 2). The solution set is G_cl(goal).
//  * G_op(s) keeps the g-vectors of open labels at s. A new label is checked
//    against it with full K-dimensional dominance, and open labels it
//    dominates are removed eagerly (lazily deleted from the heap). This
//    linear scan is the main cost BOA* avoids.
//  * A label whose f is dominated by a solution when it is extracted is
//    discarded (lazy filtering of OPEN).
//
// Equal-cost paths are not recorded (one path per Pareto-optimal cost vector),
// matching BOA*'s cost-unique output.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "mosp/graph.hpp"
#include "mosp/open_list.hpp"
#include "mosp/search.hpp"
#include "mosp/types.hpp"

namespace mosp {

template <std::size_t K>
class NamoaDr {
  static_assert(K >= 2, "NAMOA*dr needs at least two objectives");

 public:
  // `graph` must have exactly K objectives and outlive this object.
  explicit NamoaDr(const Graph& graph);

  // h[u]: consistent, admissible heuristic towards `goal` per objective
  // (kInfCost for states that cannot reach the goal).
  [[nodiscard]] SearchResult<K> search(NodeId start, NodeId goal,
                                       const std::vector<CostVec<K>>& h,
                                       const SearchOptions& options = {});

 private:
  enum class Status : std::uint8_t { kOpen, kClosed, kDead };
  struct Label {
    CostVec<K> g;
    NodeId state;
    std::uint32_t parent;
    std::uint32_t prev;  // G_op list (doubly linked) while open
    std::uint32_t next;  // G_op list while open, G_cl list (K > 2) once closed
    Status status;
  };
  static constexpr std::uint32_t kNil = 0xFFFFFFFFu;

  // True if Tr(v) is weakly dominated by a truncated vector in G_cl(s).
  [[nodiscard]] bool closed_dominates(NodeId s, const CostVec<K>& v) const;
  void add_closed(NodeId s, std::uint32_t id);
  void unlink_open(std::uint32_t id);

  const Graph* graph_;
  std::vector<std::uint64_t> totals_;   // per-objective total arc cost
  std::vector<std::uint32_t> op_head_;  // G_op(s) list heads
  std::vector<std::uint32_t> cl_head_;  // G_cl(s) list heads (K > 2)
  std::vector<Cost> g2_min_;            // G_cl(s) for K == 2
  std::vector<Label> labels_;
  BinaryHeap<CostVec<K>> open_;
};

extern template class NamoaDr<2>;
extern template class NamoaDr<3>;

// Convenience wrapper: computes the exact heuristic and runs NAMOA*dr.
template <std::size_t K>
[[nodiscard]] SearchResult<K> namoa_dr(const Graph& graph, NodeId start, NodeId goal,
                                       const SearchOptions& options = {});

extern template SearchResult<2> namoa_dr<2>(const Graph&, NodeId, NodeId, const SearchOptions&);
extern template SearchResult<3> namoa_dr<3>(const Graph&, NodeId, NodeId, const SearchOptions&);

}  // namespace mosp
