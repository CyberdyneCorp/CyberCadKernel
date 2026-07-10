#!/usr/bin/env bash
# Compile + run the MOAT M6 SHEET-METAL DIFFERENTIAL-FUZZING harness
# (tests/sim/native_sheetmetal_fuzz.mm) for the iOS simulator. This is an independent native
# domain on the differential-fuzzing completeness bar, extending the landed M6 fuzzers
# (curved-boolean, STEP round-trip, construction, blend, wrap-emboss, mass-properties,
# geometry-services, transform-chains, reference/datum, direct-modeling, transformed-boolean,
# orthographic-HLR, shape-healing, section, curved-blend, draft-angle, interference,
# freeform-boolean, variable-section sweep, N-sided fill) to the newly-native sheet-metal
# first slice (src/native/sheetmetal/{base_flange,edge_flange,unfold,common}.h) the CyberCad
# app drives through the public cc_sheet_base_flange / cc_sheet_edge_flange / cc_sheet_unfold
# facade. Those ops have a hand-picked SELF-TEST (native_sheetmetal_selftest, 5 fixtures) but
# no *fuzz* domain — this closes it.
#
# ── THERE IS NO OCCT SHEET-METAL ORACLE ───────────────────────────────────────────────────
# OCCT core has NO sheet-metal module, so — UNLIKE every other native fuzzer — this harness
# does NOT compare against OCCT and does NOT drive engine 0. It drives the SHIPPING cc_*
# facade under the NATIVE engine (cc_set_engine(1)) only. The ARBITER is CLOSED FORM +
# INVARIANTS: base flange volume == |profileArea|·thickness (exact); folded part volume ==
# base + bend(½·θ·((r+t)²−r²)·W) + wall(h·t·W) (a true cylinder bend meshed to a deflection,
# converging from below within the SAME 1.5% band common.h::verifySolid gates); the fold→
# unfold AREA INVARIANT (developed area == baseArea + BA·W + flangeArea, BA=θ·(r+k·t)); and
# every built part passes cc_check_solid (valid closed 2-manifold) + watertight + Euler χ=2.
# A "DISAGREE" = native volume != closed form OR the area invariant violated OR cc_check_solid
# fails — a REAL NATIVE BUG (the closed form / invariant is ground truth), not an oracle
# discrepancy. An out-of-slice pose (self-colliding fold / wrong edge / degenerate param /
# non-fold unfold) HONEST-DECLINES to a native NULL — counted separately, NEVER a bar failure.
#
# The sheet-metal ops (src/native/sheetmetal/**) are OCCT-FREE and do NOT need the NUMSCI
# substrate. We still compile the WHOLE kernel (facade + core + engine[native+occt adapter]
# + src/native/**) under -DCYBERCAD_HAS_OCCT and link the OCCT toolkit, because the facade's
# create_default_engine is provided by the OCCT adapter under that define — exactly like
# run-sim-native-sheetmetal.sh. The harness itself never enters an OCCT path.
#
# The bar: DISAGREED==0 AND each of the three ops (base-flange / edge-flange-fold / unfold)
# with >=1 AGREED. The FIXED bands (planar-exact <1e-6, curved-bend <1.5%, area-invariant
# <1e-6) are NEVER widened. The generator is seeded ONLY by an explicit FUZZ_SEED (argv/env)
# — NO clock, NO rand(): same seed -> byte-identical batch. On run-sim-suite.sh's SKIP list
# (own main(), std::_Exit).
#
# Usage: scripts/run-sim-native-sheetmetal-fuzz.sh [SEED] [N]
#   SEED  explicit uint64 RNG seed. If omitted, runs the TWO default seeds below (the >=2-
#         seed bar proof); the script fails if ANY seed fails. Also honoured via FUZZ_SEED.
#   N     number of generated cases (default 72). Also honoured via FUZZ_N env.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

N="${2:-${FUZZ_N:-72}}"
if [ "${1:-}" != "" ]; then SEEDS=("$1"); elif [ "${FUZZ_SEED:-}" != "" ]; then SEEDS=("$FUZZ_SEED"); else SEEDS=("0x5EE7EA1F00" "0xB3ADF01DCC"); fi

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_sheetmetal_fuzz.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# The whole kernel: facade + core + engine (NativeEngine + OCCT adapter + the always-compiled
# stub, which no-ops its create_default_engine under OCCT) + src/native/**. Same file set
# build-xcframework.sh compiles into the slice.
KERNEL_SRCS=()
while IFS= read -r src; do KERNEL_SRCS+=("$src"); done \
  < <(find "$REPO/src" -name '*.cpp' | sort)
[ "${#KERNEL_SRCS[@]}" -gt 0 ] || { echo "no kernel sources under src"; exit 1; }

# OCCT toolkit link set (most-derived -> base). Includes TKHLR/TKShHealing so the OCCT adapter
# TUs (compiled into the kernel) resolve even though the harness never calls them.
TKS="TKDESTEP TKDEIGES TKXSBase TKDE TKMesh TKHLR TKShHealing TKOffset TKFillet TKBool \
     TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling M6 SHEET-METAL differential-fuzz harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   kernel  : ${#KERNEL_SRCS[@]} src TU(s) (facade + core + engine[native+occt] + native math)"
echo "   engine  : cc_set_engine(1)=NativeEngine (sheet metal is native-only; NO OCCT oracle)"
echo "   arbiter : CLOSED FORM + fold->unfold AREA INVARIANT + closed-2-manifold/watertight/χ=2 validity"
echo "   seeds   : ${SEEDS[*]}   N : $N"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/include" \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  -x c++ "${KERNEL_SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_sheetmetal_fuzz"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi

rc=0
for SEED in "${SEEDS[@]}"; do
  echo "── running in simulator $UDID (seed=$SEED N=$N)"
  if ! xcrun simctl spawn "$UDID" "$OUT/native_sheetmetal_fuzz" "$SEED" "$N"; then rc=1; fi
done
exit $rc
