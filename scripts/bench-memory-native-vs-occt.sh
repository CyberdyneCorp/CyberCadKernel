#!/usr/bin/env bash
# bench-memory-native-vs-occt.sh — build + run the native-vs-OCCT RUNTIME MEMORY
# bench on the HOST (macOS arm64) against Homebrew OCCT, driving each cc_* op under
# BOTH engines. The memory sibling of scripts/bench-native-vs-occt.sh (latency) and
# scripts/bench-binary-size.sh (size) — the third leg of the drop-OCCT payoff.
#
# WHY HOST: this measures RUNTIME RAM, the tightest iPad constraint. Host phys_footprint
# / peak-RSS is measured with a stable allocator; the native/OCCT RATIO is the portable
# signal (device absolute differs; OCCT loads large static data — see the findings doc
# docs/BENCH-memory-native-vs-occt.md).
#
# METHOD (clean high-water marks — see the harness header):
#   * The harness (tests/sim/native_vs_occt_mem.cpp) is compiled ONCE (same link recipe
#     as bench-native-vs-occt.sh: whole kernel + OCCT adapter + -lTKHLR, measurement-only
#     link, no product change).
#   * PER-OP: to get an UNCONTAMINATED peak, each (op,size,engine) is run in its OWN
#     PROCESS invocation (--single). ru_maxrss is a per-process high-water mark that never
#     falls, so one op per process is the only honest way to attribute a peak. Each row
#     reports peak RSS, phys_footprint, and a per-op footprint delta (footprint at the
#     op's live peak minus footprint just before — the cleanest engine-to-engine signal).
#   * PROCESS-LEVEL: each engine also runs the WHOLE representative script once
#     (--script) so the process peak RSS captures the engine's full working set +
#     (for OCCT) its static/global footprint.
#
# NON-SHIPPING measurement harness. Prints the scraped [MEMROW]/[PROCROW] lines + tables.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT_PREFIX="${OCCT_PREFIX:-/opt/homebrew/opt/opencascade}"
OCCT_INC="$OCCT_PREFIX/include/opencascade"
OCCT_LIB="$OCCT_PREFIX/lib"
OUT="$REPO/build-bench"; mkdir -p "$OUT"

[ -d "$OCCT_INC" ] || { echo "OCCT headers not found at $OCCT_INC (brew install opencascade)"; exit 1; }

HARNESS="$REPO/tests/sim/native_vs_occt_mem.cpp"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# Kernel sources: all src/**/*.cpp except metal, numerics, tetgen (same set as the
# latency runner — see bench-native-vs-occt.sh header).
KERNEL_SRCS=()
while IFS= read -r src; do KERNEL_SRCS+=("$src"); done < <(
  find "$REPO/src" -name '*.cpp' \
    | grep -vE '/compute/metal/|/native/numerics/|/native/mesh/tet_mesher\.cpp$' \
    | sort)
[ "${#KERNEL_SRCS[@]}" -gt 0 ] || { echo "no kernel sources under src"; exit 1; }

# OCCT toolkit link set — the macOS desktop recipe's list PLUS TKHLR (occt_drafting.cpp).
TKS="TKDESTEP TKDEIGES TKXSBase TKDE TKXCAF TKLCAF TKCAF TKCDF TKMesh TKShHealing \
     TKOffset TKFillet TKBool TKPrim TKBO TKTopAlgo TKGeomAlgo TKHLR TKBRep TKGeomBase \
     TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

BIN="$OUT/native_vs_occt_mem"
echo "── compiling native-vs-OCCT MEMORY bench for host (arm64), Homebrew OCCT at $OCCT_PREFIX"
echo "   kernel: ${#KERNEL_SRCS[@]} src TU(s) + harness"
clang++ -arch arm64 -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/include" -I"$REPO/src" -I"$OCCT_INC" \
  "$HARNESS" "${KERNEL_SRCS[@]}" \
  -L"$OCCT_LIB" $LFLAGS -lc++ \
  -Wl,-rpath,"$OCCT_LIB" \
  -o "$BIN"

mb() { awk -v b="$1" 'BEGIN{printf "%.2f", b/1048576.0}'; }

echo
echo "── running PER-OP memory bench (one process per op/size/engine → clean high-water)"
# Collect [MEMROW] lines keyed by op|size|engine so the table can pair OCCT vs native.
ROWS_FILE="$OUT/memrows.txt"; : > "$ROWS_FILE"
# Drive the plan the harness emits (so the op matrix lives in ONE place, the harness).
while read -r _tag op size served; do
  op="${op#op=}"; size="${size#size=}"
  for engine in 0 1; do
    line="$("$BIN" --single --op "$op" --size "$size" --engine "$engine" | grep '^\[MEMROW\]' || true)"
    [ -n "$line" ] && echo "$line" >> "$ROWS_FILE" && echo "$line"
  done
done < <("$BIN" --list | grep '^\[PLAN\]')

echo
echo "── running PROCESS-LEVEL script (one-shot peak RSS per engine)"
PROC_FILE="$OUT/procrows.txt"; : > "$PROC_FILE"
for engine in 0 1; do
  line="$("$BIN" --script --engine "$engine" | grep '^\[PROCROW\]' || true)"
  [ -n "$line" ] && echo "$line" >> "$PROC_FILE" && echo "$line"
done

# ── field extractor: read "key=val" tokens from a [MEMROW]/[PROCROW] line ─────────
field() { echo "$1" | tr ' ' '\n' | grep "^$2=" | cut -d= -f2; }

echo
echo "== PER-OP MEMORY (host, Homebrew OCCT; per-op footprint delta = KB; ratio = OCCT/native) =="
printf "%-16s %-7s %12s %12s %10s %12s %12s\n" \
  "op" "size" "OCCT dKB" "native dKB" "ratio" "OCCT RSS MB" "nat RSS MB"
# Iterate the plan again to preserve order; for each op/size pair pull its two rows.
while read -r _tag op size served; do
  op="${op#op=}"; size="${size#size=}"
  occt_line="$(grep " op=$op " "$ROWS_FILE" | grep " size=$size " | grep ' engine=occt ' || true)"
  nat_line="$(grep " op=$op "  "$ROWS_FILE" | grep " size=$size " | grep ' engine=native ' || true)"
  o_delta="$(field "$occt_line" op_delta_bytes)"; o_rss="$(field "$occt_line" peak_rss_bytes)"; o_srv="$(field "$occt_line" served)"
  n_delta="$(field "$nat_line" op_delta_bytes)";  n_rss="$(field "$nat_line" peak_rss_bytes)";  n_srv="$(field "$nat_line" served)"

  # KB for the deltas; note/ratio honesty for declined/forwarded.
  o_kb="-"; n_kb="-"; ratio="-"; o_rssmb="-"; n_rssmb="-"
  [ -n "$o_delta" ] && [ "$o_delta" != "-" ] && o_kb="$(awk -v b="$o_delta" 'BEGIN{printf "%.0f", b/1024.0}')"
  [ -n "$n_delta" ] && [ "$n_delta" != "-" ] && n_kb="$(awk -v b="$n_delta" 'BEGIN{printf "%.0f", b/1024.0}')"
  [ -n "$o_rss" ]   && [ "$o_rss" != "-" ]   && o_rssmb="$(mb "$o_rss")"
  [ -n "$n_rss" ]   && [ "$n_rss" != "-" ]   && n_rssmb="$(mb "$n_rss")"
  if [ "$n_srv" = "forwarded" ]; then
    n_kb="FWD"; ratio="(fwd->OCCT)"
  elif [ "$o_srv" = "occt-declined" ]; then
    o_kb="DECL"; ratio="native-only"
  elif [ "$o_kb" != "-" ] && [ "$n_kb" != "-" ] && [ "$n_delta" -gt 0 ] 2>/dev/null; then
    ratio="$(awk -v o="$o_delta" -v n="$n_delta" 'BEGIN{printf "%.2fx", o/n}')"
  fi
  printf "%-16s %-7s %12s %12s %10s %12s %12s\n" \
    "$op" "$size" "$o_kb" "$n_kb" "$ratio" "$o_rssmb" "$n_rssmb"
done < <("$BIN" --list | grep '^\[PLAN\]')

echo
echo "== PROCESS-LEVEL PEAK RSS (same representative script, whole process) =="
printf "%-24s %14s %18s %18s\n" "engine" "peak RSS MB" "baseline FP MB" "end FP MB"
for engine_name in occt native; do
  line="$(grep " engine=$engine_name " "$PROC_FILE" || true)"
  [ -z "$line" ] && continue
  rss="$(field "$line" peak_rss_bytes)"; base="$(field "$line" baseline_footprint_bytes)"; endfp="$(field "$line" end_footprint_bytes)"
  printf "%-24s %14s %18s %18s\n" "$engine_name" "$(mb "$rss")" "$(mb "$base")" "$(mb "$endfp")"
done
# process-level ratio (extract one line per engine; field() on a single line)
OCCT_LINE="$(grep 'engine=occt ' "$PROC_FILE" | head -1 || true)"
NAT_LINE="$(grep 'engine=native ' "$PROC_FILE" | head -1 || true)"
OCCT_RSS="$(field "$OCCT_LINE" peak_rss_bytes)"
NAT_RSS="$(field "$NAT_LINE" peak_rss_bytes)"
if [ -n "$OCCT_RSS" ] && [ -n "$NAT_RSS" ] && [ "$NAT_RSS" -gt 0 ] 2>/dev/null; then
  RATIO="$(awk -v o="$OCCT_RSS" -v n="$NAT_RSS" 'BEGIN{printf "%.2fx", o/n}')"
  OCCT_MB="$(mb "$OCCT_RSS")"; NAT_MB="$(mb "$NAT_RSS")"
  echo
  printf "process peak-RSS ratio (OCCT / native) = %s   (OCCT %s MB vs native %s MB)\n" \
    "$RATIO" "$OCCT_MB" "$NAT_MB"
fi
echo
echo "== memory bench complete =="
