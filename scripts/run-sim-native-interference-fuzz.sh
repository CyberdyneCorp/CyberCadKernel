#!/usr/bin/env bash
# Compile + run the MOAT M6-breadth-17 INTERFERENCE / CLASH DIFFERENTIAL-FUZZING harness
# (tests/sim/native_interference_fuzz.mm) for the iOS simulator, inside a booted simulator
# via `xcrun simctl spawn`. This is the SEVENTEENTH native domain on the differential-fuzzing
# completeness bar — it certifies the newly-native `cc_interference` surface (the OCCT-FREE,
# header-only clash classifier src/native/analysis/interference.h) under randomised poses,
# the coverage the five hand-picked fixtures in native_interference_parity.mm do not provide.
#
# The harness DETERMINISTICALLY generates random PAIRS of solids (box / n-gon prism /
# cylinder / sphere at random dims) at random relative rigid placements (translate + rotate)
# spanning the three interference regimes — CLEAR (disjoint) / TOUCHING (near-flush) / CLASH
# (interpenetrating) — then differentials native `meshInterference` against the OCCT oracle
# (BRepAlgoAPI_Common volume + BRepExtrema_DistShapeShape distance) AND, where the pair has
# one, a CLOSED-FORM arbiter (box∩box intersection-box volume; sphere/sphere lens volume +
# centre distance). Each trial is classified:
#   AGREED            — native state == OCCT state (and the closed form, where present).
#   HONESTLY-DECLINED — native meshInterference -> Unknown -> falls through to OCCT.
#   DISAGREED         — native crisp state != OCCT crisp state on a HARD boundary and the
#                       closed-form arbiter sides with OCCT (SILENT WRONG CLASH). FAILS bar.
#   ORACLE-INACCURATE — native matches the closed form while OCCT does not (native vindicated).
#   FACET-CONVERGENT  — a curved-pair TOUCH<->CLEAR straddle within the deflection facet band.
#   BOTH-DECLINED     — both engines declined.
# The bar: DISAGREED == 0 with every populated [family x regime] cell truly exercised. The
# contact band (max(1e-9*scale, 2*deflection)) is the classifier's own; NEVER widened. The
# generator is seeded ONLY by an explicit FUZZ_SEED (argv/env) — NO clock, NO rand(): same
# seed -> byte-identical batch (splitmix64 -> xoshiro256**).
#
# interference.h is header-only (math/vec + mesh inline primitives + the B3 classifier), so
# like run-sim-native-interference.sh this links only the native math TUs, NOT the whole
# kernel. src/native / src/engine / include stay BYTE-UNCHANGED — additive test/sim only.
# On run-sim-suite.sh's SKIP list (own main(), std::_Exit).
#
# Usage: scripts/run-sim-native-interference-fuzz.sh [SEED] [N]
#   SEED  explicit uint64 RNG seed (default 0xC1A54FEED). Also honoured via FUZZ_SEED env.
#   N     number of generated cases (default 72).          Also honoured via FUZZ_N env.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

SEED="${1:-${FUZZ_SEED:-0xC1A54FEED}}"
N="${2:-${FUZZ_N:-72}}"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_interference_fuzz.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# ── native math TUs (interference.h aggregate references them) ────────────────
MATH_SRCS=()
while IFS= read -r src; do MATH_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp' | sort)

# ── OCCT oracle toolkits ──────────────────────────────────────────────────────
# BRepPrimAPI_MakeBox/Cylinder/Sphere/Prism -> TKPrim; BRepBuilderAPI_MakePolygon/MakeFace/
# Transform -> TKTopAlgo/TKBRep; BRepAlgoAPI_Common (BOPAlgo) -> TKBO, which pulls in
# ShapeUpgrade_UnifySameDomain (result simplification) -> TKShHealing; BRepExtrema_Dist
# ShapeShape -> TKBRep; BRepGProp -> TKGeomAlgo; GProp/gp -> TKMath; base TKernel. Most-
# derived -> base link order. TKHLR kept alongside TKShHealing for the healing paths BO
# transitively references (mirrors the sibling fuzzers' link set).
TKS="TKBO TKShHealing TKHLR TKPrim TKTopAlgo TKBRep TKGeomAlgo TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling M6-breadth-17 INTERFERENCE/CLASH differential-fuzz native-vs-OCCT harness (iphonesimulator arm64)"
echo "   harness : $HARNESS"
echo "   math    : ${#MATH_SRCS[@]} source(s)"
echo "   oracle  : OCCT BRepAlgoAPI_Common + BRepExtrema_DistShapeShape + closed-form (box/sphere)"
echo "   seed    : $SEED   N : $N"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  -x c++ "${MATH_SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_interference_fuzz"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID (seed=$SEED N=$N)"
xcrun simctl spawn "$UDID" "$OUT/native_interference_fuzz" "$SEED" "$N"
