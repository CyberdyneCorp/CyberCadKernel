#!/usr/bin/env bash
# Compile + run the MOAT M6c CONSTRUCTION DIFFERENTIAL-FUZZING harness
# (tests/sim/native_construct_fuzz.mm) for the iOS simulator. This is the M6-breadth-2
# completeness bar: it extends the landed M6 curved-boolean fuzzer + M6b STEP-import fuzzer
# to a THIRD native domain — the OCCT-FREE swept-solid construction library
# src/native/construct (build_loft_sections / build_sweep).
#
# The harness DETERMINISTICALLY generates random-but-VALID construction inputs from the
# families the native path CLAIMS — equal- AND mismatched-count PLANAR N-section ruled loft
# (coaxial regular-n-gon frustums / prismatoid stacks) and STRAIGHT-path constant-frame
# sweep (prisms) — plus two SPARSE out-of-scope inputs (a non-planar loft section; a
# non-planar sweep spine) to exercise the native DECLINE branch. Each input is built BOTH
# via the OCCT-FREE native builder (measured by the native tessellator) AND via the OCCT
# oracle (BRepOffsetAPI_ThruSections ruled solid / BRepOffsetAPI_MakePipe, measured exactly
# by BRepGProp), classifying each trial:
#   AGREED            — native watertight + vol/area/solid-count within tol of OCCT
#   HONESTLY-DECLINED — native NULL / non-watertight (self-verify discards) → OCCT ships
#   DISAGREED         — native watertight but OUTSIDE the analytic ground truth (SILENT WRONG)
#   ORACLE-INACCURATE — native matches exact math while OCCT does not (native vindicated)
#   BOTH-DECLINED     — an out-of-scope input both engines refuse (no wrong result)
# The bar: DISAGREED == 0 (and no core-family ORACLE_UNRELIABLE). Any DISAGREE prints the
# seed + case index + family/param tuple as a reproducible regression find. The generator is
# seeded ONLY by an explicit FUZZ_SEED (argv/env) — NO clock, NO rand(): same seed → batch.
#
# Two claimed sub-families (twisted/rotated-section loft; smooth-curved planar sweep) are
# DELIBERATELY EXCLUDED from the batch comparison — their native-mesh-vs-OCCT-exact match is
# only deflection-bounded, not exact — and are covered instead by the curated parity harnesses
# native_loft_parity / native_sweep_parity. See the .mm header + OpenSpec change.
#
# UNLIKE the curved-boolean fuzzer, this domain needs NO numsci: the native construct +
# tessellate + topology + math live in src/native/** — all OCCT-FREE and header-only (math
# bezier/bspline are the only compiled native TUs). It links ONLY the OCCT oracle toolkits
# (ThruSections/MakePipe + BRepGProp + BRepCheck). src/native stays OCCT-FREE; this harness
# is additive test/sim code only. On run-sim-suite.sh's SKIP list (own main(), std::_Exit).
#
# Usage: scripts/run-sim-native-construct-fuzz.sh [SEED] [N]
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

HARNESS="$REPO/tests/sim/native_construct_fuzz.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# ── native TUs to compile (OCCT-FREE: math bezier/bspline; construct/tessellate/topology
#    are header-only) ─────────────────────────────────────────────────────────────────
NATIVE_SRCS=(
  "$REPO/src/native/math/bezier.cpp"
  "$REPO/src/native/math/bspline.cpp"
)
for f in "${NATIVE_SRCS[@]}"; do [ -f "$f" ] || { echo "missing native TU: $f"; exit 1; }; done

# ── OCCT oracle toolkits: BRepOffsetAPI_ThruSections/MakePipe (TKOffset) + BRepGProp
#    (TKGeomAlgo) + BRepCheck/BRepBuilderAPI (TKTopAlgo/TKBRep) + geometry base ─────────
TKS="TKMesh TKShHealing TKOffset TKFillet TKBool TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep \
     TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling M6c CONSTRUCTION differential-fuzz native-vs-OCCT harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} TU(s) (math bezier/bspline) [OCCT-FREE, no numsci; construct/tessellate/topology header-only]"
echo "   oracle  : OCCT BRepOffsetAPI_ThruSections + BRepOffsetAPI_MakePipe + BRepGProp + BRepCheck"
echo "   seed    : $SEED   N : $N"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  -x c++ "${NATIVE_SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_construct_fuzz"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID (seed=$SEED N=$N)"
xcrun simctl spawn "$UDID" "$OUT/native_construct_fuzz" "$SEED" "$N"
