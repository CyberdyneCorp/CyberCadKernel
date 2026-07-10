#!/usr/bin/env bash
# Compile + run the MOAT M6-breadth-14 SHAPE-HEALING DIFFERENTIAL-FUZZING harness
# (tests/sim/native_healing_fuzz.mm) for the iOS simulator. This is the FOURTEENTH native
# domain on the differential-fuzzing completeness bar, extending the landed M6 fuzzers
# (curved-boolean, STEP round-trip, construction, blend, wrap-emboss, mass-properties,
# geometry-services, transform, reference-geometry, direct-modeling, transformed-boolean,
# section, HLR) to the native OCCT-FREE shape HEALER (heal::healShell, src/native/heal/*).
#
# The harness DETERMINISTICALLY generates random VALID base solids whose exact geometry is
# KNOWN (unit cube / random box / random N-gon prism → closed-form volume+area), injects
# one random SHAPE-PRESERVING defect (sew-jitter / flipped face / near-miss seam gap in &
# out of budget / short-edge split / collinear vertex / one-or-two missing planar faces /
# two-adjacent missing faces / beyond-tolerance gap), and heals it BOTH ways — native
# heal::healShell (with the family opt-in flag) and OCCT cyber::occt::sewAndFix
# (BRepBuilderAPI_Sewing → ShapeFix, in src/engine/occt/occt_shapefix.cpp) at the same
# tolerance — then classifies:
#   AGREED            — native Healed matches the closed-form truth AND OCCT concurs; OR
#                       native Unhealed while OCCT also fails to close (parity of decline);
#                       OR native honestly declines a defect OCCT aggressively repairs to
#                       the SAME honest truth (native MORE conservative — safe deferral).
#   HONESTLY-DECLINED — native Unhealed (input unchanged) → defer to OCCT.
#   DISAGREED         — native watertight Healed but vol/area != closed-form truth
#                       (a SILENT WRONG REPAIR — the failure this harness exists to catch).
#   ORACLE-INACCURATE — native matches exact math while OCCT does not (native vindicated).
#   BOTH-DECLINED     — a defect both engines refuse to close.
# The bar: DISAGREED == 0 AND ORACLE_UNRELIABLE == 0, each base + defect family with ≥1
# non-error trial, across ≥2 seeds. The equal-or-more-conservative contract is load-bearing:
# native must never emit a watertight solid that differs from the known truth; a decline is
# always safe. FIXED tolerances (tol 1e-4, band 1e-3, bridge budget 1e-2) are NEVER widened.
# The generator is seeded ONLY by an explicit FUZZ_SEED (argv/env) — NO clock, NO rand():
# same seed → byte-identical batch.
#
# Links (exactly like run-sim-native-heal.sh): the OCCT-FREE native TUs (heal is
# header-only + heal.cpp; math carries bezier/bspline .cpp; topology/tessellate header-only)
# plus the SINGLE OCCT oracle TU occt_shapefix.cpp (self-contained: BRepBuilderAPI_Sewing /
# ShapeFix / BRepCheck_Analyzer / BRepGProp only). No numsci, no cc_* facade, no OcctEngine
# spine — healing is internal. src/native / src/engine / include stay UNTOUCHED; this
# harness REPRODUCES the native heal path rather than modifying it. On run-sim-suite.sh's
# SKIP list (own main(), std::_Exit).
#
# Usage: scripts/run-sim-native-healing-fuzz.sh [SEED] [N]
#   SEED  explicit uint64 RNG seed. If omitted, runs the TWO default seeds below (the ≥2-seed
#         bar proof); the script fails if ANY seed fails. Also honoured via FUZZ_SEED env.
#   N     number of generated cases (default 120). Also honoured via FUZZ_N env.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

N="${2:-${FUZZ_N:-120}}"
if [ "${1:-}" != "" ]; then SEEDS=("$1"); elif [ "${FUZZ_SEED:-}" != "" ]; then SEEDS=("$FUZZ_SEED"); else SEEDS=("0x4845414C4F" "0xC0FFEEBEEF11"); fi

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_healing_fuzz.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# OCCT-free native TUs (heal.cpp + math bezier/bspline) + the OCCT oracle TU.
NATIVE_SRCS=()
while IFS= read -r src; do NATIVE_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/heal" "$REPO/src/native/math" -name '*.cpp' | sort)
ORACLE_SRC="$REPO/src/engine/occt/occt_shapefix.cpp"
[ -f "$ORACLE_SRC" ] || { echo "missing OCCT oracle TU: $ORACLE_SRC"; exit 1; }

# Oracle libraries (most-derived → base): BRepBuilderAPI_Sewing / BRepCheck_Analyzer in
# TKTopAlgo; ShapeFix_Shell / ShapeFix_Solid / ShapeAnalysis_Shell in TKShHealing;
# BRepGProp / GProp in TKGeomAlgo; BRep in TKBRep; geometry base in TKGeomBase/TKG3d/TKG2d;
# gp_* in TKMath; TKernel underneath.
TKS="TKShHealing TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling M6-breadth-14 SHAPE-HEALING differential-fuzz native-vs-OCCT harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} OCCT-free source(s) (heal.cpp + math bezier/bspline) [no numsci; topology/tessellate header-only]"
echo "   oracle  : OCCT BRepBuilderAPI_Sewing + ShapeFix_Shell/Solid + BRepGProp (occt_shapefix.cpp)"
echo "   seeds   : ${SEEDS[*]}   N : $N"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios14.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/include" \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  "${NATIVE_SRCS[@]}" "$ORACLE_SRC" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_healing_fuzz"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi

rc=0
for SEED in "${SEEDS[@]}"; do
  echo "── running in simulator $UDID (seed=$SEED N=$N)"
  if ! xcrun simctl spawn "$UDID" "$OUT/native_healing_fuzz" "$SEED" "$N"; then rc=1; fi
done
exit $rc
