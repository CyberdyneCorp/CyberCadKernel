#!/usr/bin/env bash
# Compile + run the MOAT M6-breadth-5 WRAP-EMBOSS DIFFERENTIAL-FUZZING harness
# (tests/sim/native_wrap_emboss_fuzz.mm) for the iOS simulator. This is the FIFTH native
# domain of the differential-fuzzing completeness bar, extending the landed M6 curved-boolean
# fuzzer + M6b STEP-import fuzzer + M6c construction fuzzer + M6d blend fuzzer to the
# OCCT-FREE feature library src/native/feature (wrap_emboss.h).
#
# The harness DETERMINISTICALLY generates random-but-VALID wrap-emboss inputs from the
# families the native path CLAIMS — a rectangular PAD (emboss, material added), a rectangular
# DEBOSS pocket, and a convex N-gon emboss/deboss — all wrapped onto a CYLINDER lateral face,
# plus SPARSE out-of-scope inputs (a non-cylindrical base, a >2π footprint, a deboss depth ≥ R,
# a self-intersecting loop) to exercise the native DECLINE branch. Each input is built via the
# OCCT-FREE native builder (native cylinder + feature::wrap_emboss, measured by the native
# tessellator) and compared against the PRIMARY oracle — the CLOSED-FORM curvature-corrected
# changed volume A·|Rout²−R²|/(2R) — and, for the rectangle families where clean, a SECONDARY
# OCCT-boolean reconstruction of the same solid (base cylinder FUSED / CUT with a wrapped shell
# wedge, measured exactly by BRepGProp). Each trial is classified:
#   AGREED            — native watertight + volume matches the closed form (and, rectangles,
#                       volume + area match the OCCT reconstruction) within a FIXED tol
#   HONESTLY-DECLINED — native NULL / non-watertight on an in-scope input (self-verify → OCCT)
#   DISAGREED         — native watertight but OUTSIDE the closed-form ground truth (SILENT WRONG)
#   ORACLE-INACCURATE — native matches the closed form while the OCCT reconstruction does not
#   BOTH-DECLINED     — an out-of-scope input native refuses (no wrong result)
# The bar: DISAGREED == 0 (and no out-of-scope guard leak). Any DISAGREE prints the seed +
# case index + family/param tuple as a reproducible regression find. The generator is seeded
# ONLY by an explicit FUZZ_SEED (argv/env) — NO clock, NO rand(): same seed → byte-identical batch.
#
# OCCT has NO single wrap-emboss API, so the CLOSED-FORM changed volume is the PRIMARY oracle
# (as in the construction/blend fuzzers); the OCCT boolean reconstruction is a SECONDARY cross-
# check + the only independent AREA oracle, and is CLEAN only for the rectangle footprints (a
# rectangle wraps to an exact angular sector). The native builder facets the whole cylinder at a
# fine, FIXED deflection, so native-vs-oracle is an inscribed, deflection-bounded difference kept
# far under a FIXED, never-widened tolerance (the measured max bias is logged in the summary).
#
# Like the construct/blend fuzzers this domain needs NO numsci: the native feature + construct +
# tessellate + topology + boolean + math live in src/native/** — all OCCT-FREE and header-only
# (math bezier/bspline are the only compiled native TUs). It links ONLY the OCCT oracle toolkits
# (MakeCylinder + BRepAlgoAPI Fuse/Cut + BRepGProp + BRepCheck). src/native stays OCCT-FREE; this
# harness is additive test/sim code only. On run-sim-suite.sh's SKIP list (own main(), std::_Exit).
#
# Usage: scripts/run-sim-native-wrap-emboss-fuzz.sh [SEED] [N]
#   SEED  explicit uint64 RNG seed (default 0x5745E6B055). Also honoured via FUZZ_SEED env.
#   N     number of generated cases (default 120).           Also honoured via FUZZ_N env.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

SEED="${1:-${FUZZ_SEED:-0x5745E6B055}}"
N="${2:-${FUZZ_N:-120}}"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_wrap_emboss_fuzz.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# ── native TUs to compile (OCCT-FREE: math bezier/bspline; feature/construct/tessellate/
#    topology/boolean are header-only) ─────────────────────────────────────────────────
NATIVE_SRCS=(
  "$REPO/src/native/math/bezier.cpp"
  "$REPO/src/native/math/bspline.cpp"
)
for f in "${NATIVE_SRCS[@]}"; do [ -f "$f" ] || { echo "missing native TU: $f"; exit 1; }; done

# ── OCCT oracle toolkits: BRepPrimAPI (TKPrim) + BRepAlgoAPI (TKBO/TKBool) + BRepGProp
#    (TKGeomAlgo) + BRepCheck (TKTopAlgo) + BRep base + geometry base ─────────────────────
TKS="TKMesh TKShHealing TKOffset TKFillet TKBool TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep \
     TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling M6-breadth-5 WRAP-EMBOSS differential-fuzz harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} TU(s) (math bezier/bspline) [OCCT-FREE, no numsci; feature/construct/tessellate/topology/boolean header-only]"
echo "   oracle  : closed-form changed volume (PRIMARY) + OCCT MakeCylinder + BRepAlgoAPI Fuse/Cut + BRepGProp reconstruction (SECONDARY, rectangles)"
echo "   seed    : $SEED   N : $N"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  -x c++ "${NATIVE_SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_wrap_emboss_fuzz"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID (seed=$SEED N=$N)"
xcrun simctl spawn "$UDID" "$OUT/native_wrap_emboss_fuzz" "$SEED" "$N"
