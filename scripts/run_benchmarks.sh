#!/usr/bin/env bash
# Runs the full benchmark reported in the README and writes CSVs to results/.
#
#   scripts/run_benchmarks.sh [queries] [timeout_seconds] [seed]
#
# Configurations, run one after another on a single core: for each of NY and
# BAY, BOA* (k=2, cross-checked against NAMOA*dr), NAMOA*dr (k=2) and
# NAMOA*dr (k=3). All runs use the same seed, so they see identical queries.
set -euo pipefail

QUERIES="${1:-50}"
TIMEOUT="${2:-60}"
SEED="${3:-1}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BENCH="${BENCH:-$ROOT/build/mosp_bench}"
OUT="$ROOT/results"
mkdir -p "$OUT"

{
  echo "date: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "cpu: $(lscpu | sed -n 's/^Model name: *//p')"
  echo "kernel: $(uname -sr)"
  echo "compiler: $(c++ --version | head -1)"
  echo "build flags: $(grep -E '^CMAKE_CXX_FLAGS_RELEASE|^CMAKE_BUILD_TYPE' "$ROOT/build/CMakeCache.txt" | tr '\n' ' ')"
  echo "march=native: $(grep -E '^MOSP_NATIVE' "$ROOT/build/CMakeCache.txt" | cut -d= -f2)"
  echo "git commit: $(git -C "$ROOT" rev-parse --short HEAD)"
  echo "queries: $QUERIES, timeout: ${TIMEOUT}s, seed: $SEED"
} > "$OUT/system.txt"
cat "$OUT/system.txt"

for map in NY BAY; do
  "$BENCH" --map "$map" --algo boa   --objectives 2 --queries "$QUERIES" --seed "$SEED" \
           --timeout "$TIMEOUT" --crosscheck --data "$ROOT/data" --out "$OUT/${map}_boa_k2.csv"
  "$BENCH" --map "$map" --algo namoa --objectives 2 --queries "$QUERIES" --seed "$SEED" \
           --timeout "$TIMEOUT" --data "$ROOT/data" --out "$OUT/${map}_namoa_k2.csv"
  "$BENCH" --map "$map" --algo namoa --objectives 3 --queries "$QUERIES" --seed "$SEED" \
           --timeout "$TIMEOUT" --data "$ROOT/data" --out "$OUT/${map}_namoa_k3.csv"
done
