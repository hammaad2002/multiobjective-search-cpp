// mosp_bench: runs BOA* or NAMOA*dr on random queries over a DIMACS road
// network and writes per-query statistics to CSV.
//
//   mosp_bench --map NY --algo boa|namoa --objectives 2|3 --queries N
//              --seed S --timeout SEC [--data DIR] [--out FILE]
//              [--max-labels N] [--crosscheck]
//
// Queries are (start, goal) pairs drawn uniformly at random with a
// mt19937_64 seeded by --seed, keeping only pairs where the goal is reachable.
// They depend only on the seed and the graph topology, so different
// algorithms and objective counts see exactly the same queries.
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "mosp/boa_star.hpp"
#include "mosp/dimacs.hpp"
#include "mosp/graph.hpp"
#include "mosp/heuristic.hpp"
#include "mosp/namoa_dr.hpp"
#include "mosp/search.hpp"

namespace {

using namespace mosp;

struct Args {
  std::string map = "NY";
  std::string algo = "boa";
  unsigned objectives = 2;
  unsigned queries = 50;
  std::uint64_t seed = 1;
  double timeout = 60.0;
  std::string data = "data";
  std::string out;
  std::size_t max_labels = 40'000'000;
  bool crosscheck = false;
};

[[noreturn]] void usage(const char* msg) {
  if (msg != nullptr) std::cerr << "error: " << msg << "\n\n";
  std::cerr <<
      "usage: mosp_bench --map NY --algo boa|namoa --objectives 2|3 --queries N\n"
      "                  --seed S --timeout SEC [--data DIR] [--out FILE]\n"
      "                  [--max-labels N] [--crosscheck]\n"
      "\n"
      "  --map         DIMACS map name (files data/USA-road-{d,t}.<MAP>.gr)\n"
      "  --algo        boa (BOA*, 2 objectives) or namoa (NAMOA*dr, 2 or 3)\n"
      "  --objectives  2: distance, time; 3: distance, time, hops\n"
      "  --queries     number of random (start, goal) pairs (default 50)\n"
      "  --seed        query generator seed (default 1)\n"
      "  --timeout     per-query time limit in seconds (default 60)\n"
      "  --data        directory with the DIMACS files (default data)\n"
      "  --out         CSV path (default results/<map>_<algo>_k<k>.csv)\n"
      "  --max-labels  per-query label limit, reported as memout (default 40000000)\n"
      "  --crosscheck  with 2 objectives, also run the other algorithm and\n"
      "                require identical Pareto fronts on every solved query\n";
  std::exit(msg == nullptr ? 0 : 2);
}

Args parse_args(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    const std::string flag = argv[i];
    auto value = [&]() -> std::string {
      if (i + 1 >= argc) usage(("missing value for " + flag).c_str());
      return argv[++i];
    };
    try {
      if (flag == "--map") a.map = value();
      else if (flag == "--algo") a.algo = value();
      else if (flag == "--objectives") a.objectives = static_cast<unsigned>(std::stoul(value()));
      else if (flag == "--queries") a.queries = static_cast<unsigned>(std::stoul(value()));
      else if (flag == "--seed") a.seed = std::stoull(value());
      else if (flag == "--timeout") a.timeout = std::stod(value());
      else if (flag == "--data") a.data = value();
      else if (flag == "--out") a.out = value();
      else if (flag == "--max-labels") a.max_labels = std::stoull(value());
      else if (flag == "--crosscheck") a.crosscheck = true;
      else if (flag == "--help" || flag == "-h") usage(nullptr);
      else usage(("unknown argument " + flag).c_str());
    } catch (const std::logic_error&) {
      usage(("invalid value for " + flag).c_str());
    }
  }
  if (a.algo != "boa" && a.algo != "namoa") usage("--algo must be boa or namoa");
  if (a.objectives != 2 && a.objectives != 3) usage("--objectives must be 2 or 3");
  if (a.algo == "boa" && a.objectives != 2) usage("BOA* supports exactly 2 objectives");
  if (a.crosscheck && a.objectives != 2) usage("--crosscheck needs --objectives 2");
  if (a.queries == 0) usage("--queries must be positive");
  if (a.out.empty()) {
    a.out = "results/" + a.map + "_" + a.algo + "_k" + std::to_string(a.objectives) + ".csv";
  }
  return a;
}

struct Query {
  NodeId start;
  NodeId goal;
};

// Uniform random pairs (start != goal) with goal reachable from start,
// checked by BFS from the goal on the reversed graph.
std::vector<Query> make_queries(const Graph& reverse, unsigned count, std::uint64_t seed) {
  std::mt19937_64 rng(seed);
  std::uniform_int_distribution<NodeId> node(0, reverse.num_nodes() - 1);
  std::vector<Query> queries;
  std::vector<std::uint32_t> mark(reverse.num_nodes(), 0);
  std::vector<NodeId> stack;
  std::uint32_t stamp = 0;
  while (queries.size() < count) {
    const NodeId s = node(rng);
    const NodeId t = node(rng);
    if (s == t) continue;
    ++stamp;
    stack.assign(1, t);
    mark[t] = stamp;
    bool reached = false;
    while (!stack.empty() && !reached) {
      const NodeId u = stack.back();
      stack.pop_back();
      for (EdgeId e = reverse.first_arc(u); e < reverse.end_arc(u); ++e) {
        const NodeId v = reverse.head(e);
        if (mark[v] == stamp) continue;
        mark[v] = stamp;
        if (v == s) {
          reached = true;
          break;
        }
        stack.push_back(v);
      }
    }
    if (reached) queries.push_back({s, t});
  }
  return queries;
}

struct Row {
  unsigned query;
  Query q;
  SearchStats stats;
  std::size_t front_size;
  double heuristic_ms;
};

double median(std::vector<double> v) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const std::size_t n = v.size();
  return n % 2 == 1 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

double mean(const std::vector<double>& v) {
  if (v.empty()) return 0.0;
  double s = 0.0;
  for (const double x : v) s += x;
  return s / static_cast<double>(v.size());
}

template <std::size_t K>
int run(const Args& args, const Graph& graph, const Graph& reverse,
        const std::vector<Query>& queries) {
  SearchOptions opt;
  opt.timeout_seconds = args.timeout;
  opt.max_labels = args.max_labels;
  opt.extract_paths = false;

  Dijkstra backward(reverse);
  std::vector<CostVec<K>> h;
  std::vector<Row> rows;
  std::size_t mismatches = 0;
  std::size_t crosschecked = 0;

  [[maybe_unused]] std::unique_ptr<BoaStar> boa;
  [[maybe_unused]] std::unique_ptr<NamoaDr<K>> namoa;
  if constexpr (K == 2) {
    if (args.algo == "boa" || args.crosscheck) boa = std::make_unique<BoaStar>(graph);
  }
  if (args.algo == "namoa" || args.crosscheck) namoa = std::make_unique<NamoaDr<K>>(graph);

  auto run_algo = [&](const std::string& algo, NodeId s, NodeId t) {
    if constexpr (K == 2) {
      if (algo == "boa") return boa->search(s, t, h, opt);
    }
    return namoa->search(s, t, h, opt);
  };

  std::printf("%5s %8s %8s %8s %8s %11s %13s %13s\n", "query", "start", "goal", "status",
              "front", "time_ms", "expansions", "generated");
  for (unsigned i = 0; i < queries.size(); ++i) {
    const Query q = queries[i];
    const auto h0 = std::chrono::steady_clock::now();
    compute_heuristic<K>(backward, q.goal, graph.num_nodes(), h);
    const double heuristic_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - h0).count();

    const SearchResult<K> r = run_algo(args.algo, q.start, q.goal);
    rows.push_back({i, q, r.stats, r.solutions.size(), heuristic_ms});
    std::printf("%5u %8u %8u %8s %8zu %11.2f %13llu %13llu\n", i, q.start + 1, q.goal + 1,
                to_string(r.stats.status), r.solutions.size(), r.stats.runtime_ms,
                static_cast<unsigned long long>(r.stats.expansions),
                static_cast<unsigned long long>(r.stats.generated));
    std::fflush(stdout);

    if (args.crosscheck && r.stats.status == SearchStatus::kOk) {
      const std::string other = args.algo == "boa" ? "namoa" : "boa";
      const SearchResult<K> r2 = run_algo(other, q.start, q.goal);
      if (r2.stats.status == SearchStatus::kOk) {
        ++crosschecked;
        if (r2.front() != r.front()) {
          ++mismatches;
          std::printf("      CROSSCHECK MISMATCH: %s front has %zu solutions\n", other.c_str(),
                      r2.solutions.size());
        }
      }
    }
  }

  // CSV output. Node ids are written 1-based, as in the DIMACS files.
  const std::filesystem::path out(args.out);
  if (out.has_parent_path()) std::filesystem::create_directories(out.parent_path());
  std::ofstream csv(out);
  if (!csv) throw std::runtime_error("cannot write " + args.out);
  csv << "map,algo,objectives,query,start,goal,status,front_size,runtime_ms,heuristic_ms,"
         "expansions,generated,labels_stored,timeout_s\n";
  for (const Row& row : rows) {
    csv << args.map << ',' << args.algo << ',' << K << ',' << row.query << ','
        << row.q.start + 1 << ',' << row.q.goal + 1 << ',' << to_string(row.stats.status) << ','
        << row.front_size << ',' << row.stats.runtime_ms << ',' << row.heuristic_ms << ','
        << row.stats.expansions << ',' << row.stats.generated << ',' << row.stats.labels_stored
        << ',' << args.timeout << '\n';
  }

  // Aggregate over solved queries; timeouts and memouts are only counted.
  std::vector<double> time;
  std::vector<double> front;
  std::vector<double> exp;
  std::vector<double> gen;
  std::size_t timeouts = 0;
  std::size_t memouts = 0;
  for (const Row& row : rows) {
    if (row.stats.status == SearchStatus::kTimeout) {
      ++timeouts;
    } else if (row.stats.status == SearchStatus::kMemout) {
      ++memouts;
    } else {
      time.push_back(row.stats.runtime_ms);
      front.push_back(static_cast<double>(row.front_size));
      exp.push_back(static_cast<double>(row.stats.expansions));
      gen.push_back(static_cast<double>(row.stats.generated));
    }
  }
  std::printf("\n%s %s k=%zu: %zu queries, %zu solved, %zu timeouts (>%gs), %zu memouts\n",
              args.map.c_str(), args.algo.c_str(), K, rows.size(), time.size(), timeouts,
              args.timeout, memouts);
  if (!time.empty()) {
    std::printf("  solved queries: runtime ms mean %.2f median %.2f max %.2f\n", mean(time),
                median(time), *std::max_element(time.begin(), time.end()));
    std::printf("  front size mean %.1f median %.1f max %.0f\n", mean(front), median(front),
                *std::max_element(front.begin(), front.end()));
    std::printf("  expansions mean %.0f, generated mean %.0f\n", mean(exp), mean(gen));
  }
  if (args.crosscheck) {
    std::printf("  crosscheck: %zu queries compared, %zu mismatches\n", crosschecked, mismatches);
  }
  std::printf("  wrote %s\n", args.out.c_str());
  return mismatches == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  const Args args = parse_args(argc, argv);
  try {
    const auto t0 = std::chrono::steady_clock::now();
    DimacsCoords coords;
    const Graph graph = load_road_network(args.data, args.map, args.objectives, &coords);
    const Graph reverse = graph.reversed();
    const double load_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("map %s: %u nodes, %zu arcs, %u objectives (loaded in %.2f s)\n",
                args.map.c_str(), graph.num_nodes(), graph.num_arcs(), graph.num_objectives(),
                load_s);
    const std::vector<Query> queries = make_queries(reverse, args.queries, args.seed);
    std::printf("algo %s, %u queries, seed %llu, timeout %g s, max labels %zu\n\n",
                args.algo.c_str(), args.queries, static_cast<unsigned long long>(args.seed),
                args.timeout, args.max_labels);
    if (args.objectives == 2) return run<2>(args, graph, reverse, queries);
    return run<3>(args, graph, reverse, queries);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 1;
  }
}
