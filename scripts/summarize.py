#!/usr/bin/env python3
"""Turn mosp_bench CSV files into a markdown summary and plots.

    python3 scripts/summarize.py                      # reads results/*.csv
    python3 scripts/summarize.py --results results --out results

Writes:
    <out>/summary.md                  aggregate table and paired comparisons
    <out>/runtime_vs_front.png        runtime vs Pareto-front size, per map
    <out>/boa_vs_namoa.png            BOA* vs NAMOA*dr runtime on identical queries

Aggregates over runtime, front size, expansions and generated nodes use the
solved queries only; timeouts and memouts are counted separately, never
averaged in.
"""
from __future__ import annotations

import argparse
import csv
import glob
import math
import os
import statistics
from collections import defaultdict

ALGO_LABEL = {"boa": "BOA*", "namoa": "NAMOA*dr"}
# Fixed categorical order (one hue per configuration, never cycled), plus a
# distinct marker per configuration so identity does not rely on color alone.
SERIES = [
    (("boa", 2), "BOA* (k=2)", "#2a78d6", "o"),
    (("namoa", 2), "NAMOA*dr (k=2)", "#eb6834", "s"),
    (("namoa", 3), "NAMOA*dr (k=3)", "#1baf7a", "^"),
]
SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID = "#e4e3df"


def load_rows(results_dir: str) -> list[dict]:
    rows = []
    for path in sorted(glob.glob(os.path.join(results_dir, "*.csv"))):
        with open(path, newline="") as f:
            for r in csv.DictReader(f):
                r["objectives"] = int(r["objectives"])
                r["query"] = int(r["query"])
                r["front_size"] = int(r["front_size"])
                r["runtime_ms"] = float(r["runtime_ms"])
                r["heuristic_ms"] = float(r["heuristic_ms"])
                r["expansions"] = int(r["expansions"])
                r["generated"] = int(r["generated"])
                r["timeout_s"] = float(r["timeout_s"])
                rows.append(r)
    return rows


def fmt(x: float, digits: int = 1) -> str:
    if x >= 1000:
        return f"{x:,.0f}"
    return f"{x:.{digits}f}"


def map_order(maps):
    preferred = ["NY", "BAY"]
    return [m for m in preferred if m in maps] + sorted(m for m in maps if m not in preferred)


def summary_table(groups) -> list[str]:
    lines = [
        "| Map | Algorithm | k | Queries | Solved | Timeouts | Memouts "
        "| Front size mean / median / max "
        "| Runtime ms mean / median / max "
        "| Expansions mean | Generated mean |",
        "|---|---|---:|---:|---:|---:|---:|---|---|---:|---:|",
    ]
    for (m, algo, k), rs in groups:
        ok = [r for r in rs if r["status"] == "ok"]
        to = sum(r["status"] == "timeout" for r in rs)
        mo = sum(r["status"] == "memout" for r in rs)
        timeout_s = rs[0]["timeout_s"]
        if ok:
            fr = [r["front_size"] for r in ok]
            rt = [r["runtime_ms"] for r in ok]
            ex = statistics.mean(r["expansions"] for r in ok)
            ge = statistics.mean(r["generated"] for r in ok)
            front = f"{fmt(statistics.mean(fr))} / {fmt(statistics.median(fr))} / {max(fr)}"
            runtime = (f"{fmt(statistics.mean(rt), 2)} / {fmt(statistics.median(rt), 2)} / "
                       f"{fmt(max(rt), 2)}")
            exs, ges = fmt(ex, 0), fmt(ge, 0)
        else:
            front = runtime = exs = ges = "n/a"
        lines.append(
            f"| {m} | {ALGO_LABEL[algo]} | {k} | {len(rs)} | {len(ok)} "
            f"| {to} (> {timeout_s:g} s) | {mo} | {front} | {runtime} | {exs} | {ges} |")
    return lines


def by_query(rs):
    return {r["query"]: r for r in rs}


def geomean(xs):
    return math.exp(statistics.mean(math.log(x) for x in xs))


def paired_sections(index, maps) -> list[str]:
    out = []
    out.append("### BOA* vs NAMOA*dr, k = 2 (same queries)")
    out.append("")
    out.append("Speedup = NAMOA*dr runtime / BOA* runtime, over queries solved by both.")
    out.append("")
    out.append("| Map | Queries solved by both | Median speedup | Geometric-mean speedup "
               "| Min / max speedup | BOA* faster on | Median expansions ratio |")
    out.append("|---|---:|---:|---:|---|---:|---:|")
    for m in maps:
        boa = by_query(index.get((m, "boa", 2), []))
        nam = by_query(index.get((m, "namoa", 2), []))
        both = [q for q in boa if q in nam and boa[q]["status"] == "ok"
                and nam[q]["status"] == "ok"]
        if not both:
            continue
        sp = [max(nam[q]["runtime_ms"], 1e-3) / max(boa[q]["runtime_ms"], 1e-3) for q in both]
        exr = [nam[q]["expansions"] / max(boa[q]["expansions"], 1) for q in both]
        mismatched = [q for q in both if boa[q]["front_size"] != nam[q]["front_size"]]
        out.append(
            f"| {m} | {len(both)} | {statistics.median(sp):.2f}x | {geomean(sp):.2f}x "
            f"| {min(sp):.2f}x / {max(sp):.2f}x | {sum(s > 1 for s in sp)} / {len(both)} "
            f"| {statistics.median(exr):.2f} |")
        if mismatched:
            out.append(f"| {m} | WARNING: front sizes differ on queries {mismatched} | | | | | |")
    out.append("")
    out.append("### Two vs three objectives, NAMOA*dr (same queries)")
    out.append("")
    out.append("| Map | Queries solved with k=2 | ... and with k=3 | Median front k=2 "
               "| Median front k=3 | Median front ratio k3/k2 | Median runtime ratio k3/k2 |")
    out.append("|---|---:|---:|---:|---:|---:|---:|")
    for m in maps:
        k2 = by_query(index.get((m, "namoa", 2), []))
        k3 = by_query(index.get((m, "namoa", 3), []))
        if not k2 or not k3:
            continue
        solved2 = [q for q in k2 if k2[q]["status"] == "ok"]
        both = [q for q in solved2 if q in k3 and k3[q]["status"] == "ok"]
        if not both:
            out.append(f"| {m} | {len(solved2)} | 0 | | | | |")
            continue
        f2 = [k2[q]["front_size"] for q in both]
        f3 = [k3[q]["front_size"] for q in both]
        fr = [k3[q]["front_size"] / max(k2[q]["front_size"], 1) for q in both]
        rr = [max(k3[q]["runtime_ms"], 1e-3) / max(k2[q]["runtime_ms"], 1e-3) for q in both]
        out.append(
            f"| {m} | {len(solved2)} | {len(both)} | {statistics.median(f2):g} "
            f"| {statistics.median(f3):g} | {statistics.median(fr):.1f}x "
            f"| {statistics.median(rr):.1f}x |")
    return out


def style_axes(ax):
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(TEXT_SECONDARY)
    ax.tick_params(colors=TEXT_SECONDARY, labelsize=9)
    ax.grid(True, which="major", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def plot_runtime_vs_front(index, maps, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, len(maps), figsize=(5.4 * len(maps), 5.6), squeeze=False,
                             facecolor=SURFACE)
    for ax, m in zip(axes[0], maps):
        style_axes(ax)
        timeout_s = None
        for (algo, k), label, color, marker in SERIES:
            rs = index.get((m, algo, k), [])
            ok = [r for r in rs if r["status"] == "ok"]
            lim = [r for r in rs if r["status"] != "ok"]
            if not rs:
                continue
            timeout_s = rs[0]["timeout_s"]
            ax.scatter([max(r["front_size"], 1) for r in ok],
                       [max(r["runtime_ms"], 1e-3) for r in ok],
                       s=34, marker=marker, color=color, edgecolors=SURFACE, linewidths=0.8,
                       label=f"{label}: {len(ok)} solved", zorder=3)
            if lim:
                # Unsolved queries: hollow markers at the time they were stopped,
                # x = size of the partial front found so far (a lower bound).
                ax.scatter([max(r["front_size"], 1) for r in lim],
                           [max(r["runtime_ms"], 1e-3) for r in lim],
                           s=34, marker=marker, facecolors="none", edgecolors=color,
                           linewidths=1.2, label=f"{label}: {len(lim)} not solved (partial front)",
                           zorder=3)
        if timeout_s:
            ax.axhline(timeout_s * 1000, color=TEXT_SECONDARY, linestyle="--", linewidth=1)
            ax.annotate(f"timeout {timeout_s:g} s", xy=(1.0, timeout_s * 1000),
                        xycoords=("axes fraction", "data"), xytext=(-4, 4),
                        textcoords="offset points", ha="right", fontsize=8, color=TEXT_SECONDARY)
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.set_xlabel("Pareto front size (solutions)", color=TEXT, fontsize=10)
        ax.set_ylabel("Search time (ms, log scale)", color=TEXT, fontsize=10)
        ax.set_title(f"{m}", color=TEXT, fontsize=11, loc="left")
        ax.legend(fontsize=8, frameon=False, loc="upper left", bbox_to_anchor=(0.0, -0.16),
                  labelcolor=TEXT, ncol=1)
    fig.suptitle("Runtime vs. Pareto front size (one marker per query)", color=TEXT,
                 fontsize=12, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(path, dpi=150, facecolor=SURFACE)
    plt.close(fig)


def plot_paired(index, maps, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(5.2, 4.8), facecolor=SURFACE)
    style_axes(ax)
    lo, hi = math.inf, 0.0
    colors = ["#2a78d6", "#eb6834"]
    markers = ["o", "s"]
    for i, m in enumerate(maps[:2]):
        boa = by_query(index.get((m, "boa", 2), []))
        nam = by_query(index.get((m, "namoa", 2), []))
        both = [q for q in boa if q in nam and boa[q]["status"] == "ok"
                and nam[q]["status"] == "ok"]
        if not both:
            continue
        xs = [max(boa[q]["runtime_ms"], 1e-3) for q in both]
        ys = [max(nam[q]["runtime_ms"], 1e-3) for q in both]
        lo, hi = min(lo, *xs, *ys), max(hi, *xs, *ys)
        ax.scatter(xs, ys, s=34, marker=markers[i], color=colors[i], edgecolors=SURFACE,
                   linewidths=0.8, label=f"{m} ({len(both)} queries)", zorder=3)
    if hi > 0:
        ax.plot([lo / 2, hi * 2], [lo / 2, hi * 2], color=TEXT_SECONDARY, linestyle="--",
                linewidth=1, zorder=2)
        ax.annotate("equal runtime", xy=(lo, lo), xytext=(8, -2), textcoords="offset points",
                    ha="left", va="top", fontsize=8, color=TEXT_SECONDARY)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("BOA* search time (ms)", color=TEXT, fontsize=10)
    ax.set_ylabel("NAMOA*dr search time (ms)", color=TEXT, fontsize=10)
    ax.set_title("BOA* vs NAMOA*dr, k = 2, identical queries\n"
                 "(points above the dashed line: BOA* is faster)",
                 color=TEXT, fontsize=10, loc="left")
    ax.legend(fontsize=8, frameon=False, loc="upper left", labelcolor=TEXT)
    fig.tight_layout()
    fig.savefig(path, dpi=150, facecolor=SURFACE)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--results", default="results", help="directory with mosp_bench CSVs")
    ap.add_argument("--out", default=None, help="output directory (default: --results)")
    ap.add_argument("--no-plots", action="store_true", help="only write summary.md")
    args = ap.parse_args()
    out_dir = args.out or args.results
    os.makedirs(out_dir, exist_ok=True)

    rows = load_rows(args.results)
    if not rows:
        raise SystemExit(f"no CSV files in {args.results}")
    index = defaultdict(list)
    for r in rows:
        index[(r["map"], r["algo"], r["objectives"])].append(r)
    maps = map_order({r["map"] for r in rows})
    order = {key: i for i, (key, *_rest) in enumerate(SERIES)}
    groups = sorted(index.items(),
                    key=lambda kv: (maps.index(kv[0][0]), order.get((kv[0][1], kv[0][2]), 99)))

    md = ["# Benchmark summary", "",
          "Generated by `scripts/summarize.py` from the CSV files in this directory. "
          "Runtime, front size, expansions and generated nodes are over solved queries only; "
          "the heuristic (backward Dijkstra) time is excluded from runtime.", ""]
    md += summary_table(groups)
    md += [""]
    md += paired_sections(index, maps)
    md += [""]
    with open(os.path.join(out_dir, "summary.md"), "w") as f:
        f.write("\n".join(md))
    print("\n".join(md))

    if not args.no_plots:
        plot_runtime_vs_front(index, maps, os.path.join(out_dir, "runtime_vs_front.png"))
        plot_paired(index, maps, os.path.join(out_dir, "boa_vs_namoa.png"))
        print(f"\nwrote plots to {out_dir}")


if __name__ == "__main__":
    main()
