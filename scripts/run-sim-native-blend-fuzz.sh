#!/usr/bin/env bash
# Compile + run the MOAT M6d BLEND DIFFERENTIAL-FUZZING harness
# (tests/sim/native_blend_fuzz.mm) for the iOS simulator. This is the M6-breadth-3
# completeness bar: it extends the landed M6 curved-boolean fuzzer + M6b STEP-import fuzzer
# + M6c construction fuzzer to a FOURTH native domain — the OCCT-FREE blend library
# src/native/blend (planar-dihedral + curved cyl<->cap fillet/chamfer).
#
# The harness DETERMINISTICALLY generates random-but-VALID blend inputs from the families
# the native path CLAIMS — planar-dihedral chamfer/fillet of a box edge; constant- and
# variable-linear-radius fillet, and symmetric + asymmetric cone-frustum chamfer, of a
# convex cylinder<->cap circular rim — plus ONE SPARSE out-of-scope input (a fillet radius
# with Rc<2r, outside the native ring-torus scope) to exercise the native DECLINE branch.
# Each input is built BOTH via the OCCT-FREE native builder (native primitive + native blend,
# measured by the native tessellator) AND via the OCCT oracle (BRepPrimAPI_MakeBox/MakeCylinder
# + BRepFilletAPI_MakeFillet/MakeChamfer, measured exactly by BRepGProp) on the SAME geometric
# edge/rim, classifying each trial:
#   AGREED            — native watertight + vol/area/solid-count within tol of OCCT
#   HONESTLY-DECLINED — native NULL / non-watertight (self-verify discards) → OCCT ships
#   DISAGREED         — native watertight but OUTSIDE the analytic ground truth (SILENT WRONG)
#   ORACLE-INACCURATE — native matches exact math while OCCT does not (native vindicated)
#   BOTH-DECLINED     — an out-of-scope input both engines refuse (no wrong result)
# The bar: DISAGREED == 0 (and no core-family ORACLE_UNRELIABLE). Any DISAGREE prints the
# seed + case index + family/param tuple as a reproducible regression find. The generator is
# seeded ONLY by an explicit FUZZ_SEED (argv/env) — NO clock, NO rand(): same seed → batch.
#
# Every AGREE family carries a CLOSED-FORM removed volume (a torus-canal fillet's Pappus
# removed volume; a cone-frustum chamfer's pi*d1*d2*(Rc-d2/3); a box-edge prism/groove), used
# as an analytic arbiter so a native result that matches exact math while OCCT is the outlier
# is logged ORACLE-INACCURATE, never blamed on the native builder. The planar chamfer is a
# planar cut (native == OCCT EXACT); every other family facets a curved blend at a fine, FIXED
# blend deflection, so native-vs-OCCT is deflection-bounded within a FIXED, never-widened
# tolerance (the measured max bias is logged in the summary).
#
# The CONCAVE stepped-shaft fillet + offset/shell (also claimed by native_blend.h) are left to
# the curated parity harnesses for this FIRST blend-fuzz slice — an honest DOMAIN-level decline
# noted in the .mm header + OpenSpec change.
#
# Like the construct fuzzer this domain needs NO numsci: the native blend + construct +
# tessellate + topology + boolean + math live in src/native/** — all OCCT-FREE and header-only
# (math bezier/bspline are the only compiled native TUs). It links ONLY the OCCT oracle
# toolkits (MakeBox/MakeCylinder + BRepFilletAPI + BRepGProp + BRepCheck). src/native stays
# OCCT-FREE; this harness is additive test/sim code only. On run-sim-suite.sh's SKIP list (own
# main(), std::_Exit).
#
# Usage: scripts/run-sim-native-blend-fuzz.sh [SEED] [N]
#   SEED  explicit uint64 RNG seed (default 0x5744EE9911). Also honoured via FUZZ_SEED env.
#   N     number of generated cases (default 96).           Also honoured via FUZZ_N env.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

SEED="${1:-${FUZZ_SEED:-0x5744EE9911}}"
N="${2:-${FUZZ_N:-96}}"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_blend_fuzz.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# ── native TUs to compile (OCCT-FREE: math bezier/bspline; blend/construct/tessellate/
#    topology/boolean are header-only) ─────────────────────────────────────────────────
NATIVE_SRCS=(
  "$REPO/src/native/math/bezier.cpp"
  "$REPO/src/native/math/bspline.cpp"
)
for f in "${NATIVE_SRCS[@]}"; do [ -f "$f" ] || { echo "missing native TU: $f"; exit 1; }; done

# ── OCCT oracle toolkits: BRepPrimAPI (TKPrim) + BRepFilletAPI (TKFillet) + BRepGProp
#    (TKGeomAlgo) + BRepCheck/BRepBuilderAPI (TKTopAlgo/TKBRep) + geometry base ──────────
TKS="TKMesh TKShHealing TKOffset TKFillet TKBool TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep \
     TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling M6d BLEND differential-fuzz native-vs-OCCT harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} TU(s) (math bezier/bspline) [OCCT-FREE, no numsci; blend/construct/tessellate/topology/boolean header-only]"
echo "   oracle  : OCCT BRepPrimAPI_MakeBox/MakeCylinder + BRepFilletAPI_MakeFillet/MakeChamfer + BRepGProp + BRepCheck"
echo "   seed    : $SEED   N : $N"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  -x c++ "${NATIVE_SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_blend_fuzz"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID (seed=$SEED N=$N)"
xcrun simctl spawn "$UDID" "$OUT/native_blend_fuzz" "$SEED" "$N"
