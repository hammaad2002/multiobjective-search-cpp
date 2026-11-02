#!/usr/bin/env bash
# Downloads 9th DIMACS Implementation Challenge road networks into data/.
#
#   scripts/download_data.sh            # NY and BAY
#   scripts/download_data.sh NY COL     # any maps from the challenge
#
# For each map it fetches the distance graph (USA-road-d.<MAP>.gr), the
# travel-time graph (USA-road-t.<MAP>.gr) and the coordinates
# (USA-road-d.<MAP>.co; one file shared by both graphs) from the official
# challenge site, http://www.diag.uniroma1.it/challenge9/download.shtml.
# Set DIMACS_BASE_URL to use a different location with the same layout.
set -euo pipefail

BASE_URL="${DIMACS_BASE_URL:-http://www.diag.uniroma1.it/challenge9/data}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_DIR="${DATA_DIR:-$ROOT/data}"
MAPS=("$@")
if [ ${#MAPS[@]} -eq 0 ]; then MAPS=(NY BAY); fi

# Published sizes (nodes arcs) of the maps used in the README, for a sanity check.
declare -A EXPECTED=([NY]="264346 733846" [BAY]="321270 800172")

mkdir -p "$DATA_DIR"

fetch() {  # fetch <remote path relative to BASE_URL> <local file name>
  local url="$BASE_URL/$1" out="$DATA_DIR/$2"
  if [ -s "$out" ]; then
    echo "  $2 already present"
    return
  fi
  echo "  downloading $url"
  local tmp
  tmp="$(mktemp "$DATA_DIR/.download.XXXXXX")"
  if ! curl -fL --retry 3 --connect-timeout 30 -o "$tmp" "$url"; then
    rm -f "$tmp"
    echo "error: download failed: $url" >&2
    exit 1
  fi
  gunzip -c "$tmp" > "$out.part"
  rm -f "$tmp"
  mv "$out.part" "$out"
}

header() {  # prints "<nodes> <arcs>" from the problem line of a .gr file
  awk '$1 == "p" { print $3, $4; exit }' "$1"
}

for map in "${MAPS[@]}"; do
  echo "$map:"
  fetch "USA-road-d/USA-road-d.$map.gr.gz" "USA-road-d.$map.gr"
  fetch "USA-road-t/USA-road-t.$map.gr.gz" "USA-road-t.$map.gr"
  fetch "USA-road-d/USA-road-d.$map.co.gz" "USA-road-d.$map.co"

  d_header="$(header "$DATA_DIR/USA-road-d.$map.gr")"
  t_header="$(header "$DATA_DIR/USA-road-t.$map.gr")"
  if [ "$d_header" != "$t_header" ]; then
    echo "error: $map distance ($d_header) and time ($t_header) graphs differ in size" >&2
    exit 1
  fi
  arcs_found="$(grep -c '^a ' "$DATA_DIR/USA-road-d.$map.gr")"
  if [ "$arcs_found" != "${d_header#* }" ]; then
    echo "error: $map header declares ${d_header#* } arcs but file has $arcs_found" >&2
    exit 1
  fi
  if [ -n "${EXPECTED[$map]:-}" ] && [ "${EXPECTED[$map]}" != "$d_header" ]; then
    echo "error: $map has nodes/arcs '$d_header', expected '${EXPECTED[$map]}'" >&2
    exit 1
  fi
  echo "  ok: ${d_header% *} nodes, ${d_header#* } arcs"
done
