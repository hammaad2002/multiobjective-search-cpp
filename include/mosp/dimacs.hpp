// Readers for the 9th DIMACS Implementation Challenge shortest-path formats:
//   .gr  "p sp <n> <m>" header, then "a <u> <v> <w>" arcs (1-based node ids)
//   .co  "p aux sp co <n>" header, then "v <id> <x> <y>" coordinates
// Lines starting with 'c' are comments.
#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "mosp/graph.hpp"
#include "mosp/types.hpp"

namespace mosp {

// One single-objective DIMACS graph, arcs in file order, node ids converted to 0-based.
struct DimacsGraph {
  NodeId num_nodes = 0;
  std::vector<NodeId> tails;
  std::vector<NodeId> heads;
  std::vector<Cost> weights;
};

// Node coordinates as stored in the file (integer longitude/latitude * 10^6).
struct DimacsCoords {
  std::vector<std::int32_t> x;
  std::vector<std::int32_t> y;
};

// Parsers throw std::runtime_error with a line number on malformed input.
[[nodiscard]] DimacsGraph parse_gr(std::string_view text);
[[nodiscard]] DimacsCoords parse_co(std::string_view text);
[[nodiscard]] DimacsGraph read_gr(const std::string& path);
[[nodiscard]] DimacsCoords read_co(const std::string& path);

// Merges single-objective graphs over the same arc set into one multi-objective
// graph. Arcs are matched in file order when the (tail, head) sequences agree,
// otherwise by sorting on (tail, head) (parallel arcs then pair up in file
// order). Throws if the arc sets differ. If add_hop_objective is set, a final
// objective with cost 1 per arc is appended.
[[nodiscard]] Graph combine_objectives(std::span<const DimacsGraph* const> objectives,
                                       bool add_hop_objective);

// Throws std::overflow_error if path costs could overflow Cost. Search
// algorithms add a path cost and a heuristic value, each bounded by the total
// arc cost of the objective, so we require 2 * total < kInfCost per objective.
void check_cost_range(const Graph& graph);

// Loads a DIMACS USA road network from data_dir:
//   k = 2: distance (USA-road-d.<map>.gr) and travel time (USA-road-t.<map>.gr)
//   k = 3: distance, travel time and hop count (1 per arc)
// If coords is non-null, USA-road-d.<map>.co is also read and its node count checked.
[[nodiscard]] Graph load_road_network(const std::string& data_dir, const std::string& map,
                                      unsigned num_objectives, DimacsCoords* coords = nullptr);

}  // namespace mosp
