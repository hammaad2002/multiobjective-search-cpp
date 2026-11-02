#include "mosp/dimacs.hpp"

#include <algorithm>
#include <charconv>
#include <fstream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace mosp {
namespace {

// Minimal line tokenizer over a string_view; avoids iostream overhead for
// multi-million-line files.
class LineReader {
 public:
  explicit LineReader(std::string_view text) : text_(text) {}

  bool next_line() {
    if (pos_ >= text_.size()) return false;
    const std::size_t end = text_.find('\n', pos_);
    const std::size_t stop = end == std::string_view::npos ? text_.size() : end;
    line_ = text_.substr(pos_, stop - pos_);
    if (!line_.empty() && line_.back() == '\r') line_.remove_suffix(1);
    pos_ = stop + 1;
    ++line_no_;
    tok_ = 0;
    return true;
  }

  [[nodiscard]] std::string_view line() const { return line_; }
  [[nodiscard]] std::size_t line_no() const { return line_no_; }

  // Next whitespace-separated token on the current line ("" if none).
  std::string_view token() {
    while (tok_ < line_.size() && (line_[tok_] == ' ' || line_[tok_] == '\t')) ++tok_;
    const std::size_t start = tok_;
    while (tok_ < line_.size() && line_[tok_] != ' ' && line_[tok_] != '\t') ++tok_;
    return line_.substr(start, tok_ - start);
  }

  template <typename T>
  T number(const char* what) {
    const std::string_view t = token();
    T value{};
    const auto [ptr, ec] = std::from_chars(t.data(), t.data() + t.size(), value);
    if (t.empty() || ec != std::errc{} || ptr != t.data() + t.size()) {
      fail(std::string("expected ") + what + ", got '" + std::string(t) + "'");
    }
    return value;
  }

  [[noreturn]] void fail(const std::string& msg) const {
    throw std::runtime_error("line " + std::to_string(line_no_) + ": " + msg);
  }

 private:
  std::string_view text_;
  std::string_view line_;
  std::size_t pos_ = 0;
  std::size_t line_no_ = 0;
  std::size_t tok_ = 0;
};

std::string read_file(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("cannot open " + path);
  std::ostringstream ss;
  ss << in.rdbuf();
  return std::move(ss).str();
}

}  // namespace

DimacsGraph parse_gr(std::string_view text) {
  LineReader r(text);
  DimacsGraph g;
  bool have_header = false;
  std::uint64_t declared_arcs = 0;
  while (r.next_line()) {
    const std::string_view kind = r.token();
    if (kind.empty() || kind == "c") continue;
    if (kind == "p") {
      if (have_header) r.fail("duplicate problem line");
      if (r.token() != "sp") r.fail("expected 'p sp <n> <m>'");
      const auto n = r.number<std::uint64_t>("node count");
      declared_arcs = r.number<std::uint64_t>("arc count");
      if (n >= kInvalidNode) r.fail("too many nodes");
      g.num_nodes = static_cast<NodeId>(n);
      g.tails.reserve(declared_arcs);
      g.heads.reserve(declared_arcs);
      g.weights.reserve(declared_arcs);
      have_header = true;
    } else if (kind == "a") {
      if (!have_header) r.fail("arc before problem line");
      const auto u = r.number<std::uint64_t>("tail");
      const auto v = r.number<std::uint64_t>("head");
      const auto w = r.number<std::uint64_t>("weight");
      if (u == 0 || u > g.num_nodes || v == 0 || v > g.num_nodes) r.fail("node id out of range");
      if (w >= kInfCost) r.fail("arc weight too large");
      g.tails.push_back(static_cast<NodeId>(u - 1));
      g.heads.push_back(static_cast<NodeId>(v - 1));
      g.weights.push_back(static_cast<Cost>(w));
    } else {
      r.fail("unknown line type '" + std::string(kind) + "'");
    }
  }
  if (!have_header) throw std::runtime_error("missing 'p sp' line");
  if (g.tails.size() != declared_arcs) {
    throw std::runtime_error("header declares " + std::to_string(declared_arcs) + " arcs, found " +
                             std::to_string(g.tails.size()));
  }
  return g;
}

DimacsCoords parse_co(std::string_view text) {
  LineReader r(text);
  DimacsCoords c;
  std::vector<bool> seen;
  bool have_header = false;
  std::size_t count = 0;
  while (r.next_line()) {
    const std::string_view kind = r.token();
    if (kind.empty() || kind == "c") continue;
    if (kind == "p") {
      if (have_header) r.fail("duplicate problem line");
      if (r.token() != "aux" || r.token() != "sp" || r.token() != "co") {
        r.fail("expected 'p aux sp co <n>'");
      }
      const auto n = r.number<std::uint64_t>("node count");
      if (n >= kInvalidNode) r.fail("too many nodes");
      c.x.assign(n, 0);
      c.y.assign(n, 0);
      seen.assign(n, false);
      have_header = true;
    } else if (kind == "v") {
      if (!have_header) r.fail("coordinate before problem line");
      const auto id = r.number<std::uint64_t>("node id");
      const auto x = r.number<std::int32_t>("x coordinate");
      const auto y = r.number<std::int32_t>("y coordinate");
      if (id == 0 || id > c.x.size()) r.fail("node id out of range");
      if (seen[id - 1]) r.fail("duplicate coordinate for node " + std::to_string(id));
      seen[id - 1] = true;
      c.x[id - 1] = x;
      c.y[id - 1] = y;
      ++count;
    } else {
      r.fail("unknown line type '" + std::string(kind) + "'");
    }
  }
  if (!have_header) throw std::runtime_error("missing 'p aux sp co' line");
  if (count != c.x.size()) {
    throw std::runtime_error("expected " + std::to_string(c.x.size()) + " coordinates, found " +
                             std::to_string(count));
  }
  return c;
}

DimacsGraph read_gr(const std::string& path) {
  try {
    return parse_gr(read_file(path));
  } catch (const std::runtime_error& e) {
    throw std::runtime_error(path + ": " + e.what());
  }
}

DimacsCoords read_co(const std::string& path) {
  try {
    return parse_co(read_file(path));
  } catch (const std::runtime_error& e) {
    throw std::runtime_error(path + ": " + e.what());
  }
}

Graph combine_objectives(std::span<const DimacsGraph* const> objectives, bool add_hop_objective) {
  if (objectives.empty()) throw std::invalid_argument("need at least one objective graph");
  const DimacsGraph& base = *objectives.front();
  const std::size_t m = base.tails.size();
  const unsigned k = static_cast<unsigned>(objectives.size()) + (add_hop_objective ? 1U : 0U);

  // order[j][i] = index into objective j's arc list of the i-th combined arc.
  std::vector<std::vector<std::size_t>> order(objectives.size());
  auto sorted_order = [](const DimacsGraph& g) {
    std::vector<std::size_t> idx(g.tails.size());
    std::iota(idx.begin(), idx.end(), std::size_t{0});
    std::stable_sort(idx.begin(), idx.end(), [&g](std::size_t a, std::size_t b) {
      return g.tails[a] != g.tails[b] ? g.tails[a] < g.tails[b] : g.heads[a] < g.heads[b];
    });
    return idx;
  };

  bool same_order = true;
  for (const DimacsGraph* g : objectives) {
    if (g->num_nodes != base.num_nodes || g->tails.size() != m) {
      throw std::runtime_error("objective graphs differ in node or arc count");
    }
    same_order = same_order && g->tails == base.tails && g->heads == base.heads;
  }
  for (std::size_t j = 0; j < objectives.size(); ++j) {
    if (same_order) {
      order[j].resize(m);
      std::iota(order[j].begin(), order[j].end(), std::size_t{0});
    } else {
      order[j] = sorted_order(*objectives[j]);
    }
  }

  std::vector<NodeId> tails(m);
  std::vector<NodeId> heads(m);
  std::vector<Cost> costs(m * k);
  for (std::size_t i = 0; i < m; ++i) {
    const std::size_t b = order[0][i];
    tails[i] = base.tails[b];
    heads[i] = base.heads[b];
    for (std::size_t j = 0; j < objectives.size(); ++j) {
      const DimacsGraph& g = *objectives[j];
      const std::size_t a = order[j][i];
      if (g.tails[a] != tails[i] || g.heads[a] != heads[i]) {
        throw std::runtime_error("objective graphs have different arc sets");
      }
      costs[i * k + j] = g.weights[a];
    }
    if (add_hop_objective) costs[i * k + k - 1] = 1;
  }
  return Graph::from_arcs(base.num_nodes, k, tails, heads, costs);
}

void check_cost_range(const Graph& graph) {
  for (unsigned j = 0; j < graph.num_objectives(); ++j) {
    const std::uint64_t total = graph.total_cost(j);
    if (total >= kInfCost) {
      throw std::overflow_error("objective " + std::to_string(j) + ": total arc cost " +
                                std::to_string(total) + " is too large for 32-bit path costs");
    }
  }
}

Graph load_road_network(const std::string& data_dir, const std::string& map,
                        unsigned num_objectives, DimacsCoords* coords) {
  if (num_objectives != 2 && num_objectives != 3) {
    throw std::invalid_argument("road networks support 2 or 3 objectives");
  }
  const std::string prefix = data_dir + "/USA-road-";
  const DimacsGraph dist = read_gr(prefix + "d." + map + ".gr");
  const DimacsGraph time = read_gr(prefix + "t." + map + ".gr");
  const DimacsGraph* parts[] = {&dist, &time};
  Graph g = combine_objectives(parts, num_objectives == 3);
  check_cost_range(g);
  if (coords != nullptr) {
    *coords = read_co(prefix + "d." + map + ".co");
    if (coords->x.size() != g.num_nodes()) {
      throw std::runtime_error("coordinate file node count does not match the graph");
    }
  }
  return g;
}

}  // namespace mosp
