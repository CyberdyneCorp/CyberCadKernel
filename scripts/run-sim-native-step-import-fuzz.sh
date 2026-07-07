#!/usr/bin/env bash
# Compile + run the MOAT M6b DIFFERENTIAL-FUZZING harness
# (tests/sim/native_step_import_fuzz.mm) for the iOS simulator. This is the M6-breadth
# completeness bar: it extends the landed M6 curved-boolean fuzzer to a SECOND native
# domain — the OCCT-FREE ISO-10303-21 STEP READER (src/native/exchange/step_reader).
#
# The harness DETERMINISTICALLY generates random-but-VALID native B-rep solids from the
# in-scope families whose native-write -> OCCT-read round-trip is a CLEAN oracle
# (box / n-gon prism / cylinder / frustum-cone / holed box). Two other writer-producible
# families are DELIBERATELY EXCLUDED and logged (see the .mm header + OpenSpec change):
# SPHERE (OCCT re-imports the native VERTEX_LOOP sphere inconsistently -> untrustworthy
# oracle) and RULED LOFT (native writer honestly declines the bilinear B_SPLINE_SURFACE).
# Each generated solid is EXPORTED to ONE on-disk STEP file via the native writer, then
# re-IMPORTS that SAME file BOTH via the native OCCT-free reader AND via the OCCT
# STEPControl_Reader oracle, classifying each trial:
#   AGREED            — native reader watertight + vol/area/solid-count within tol of OCCT
#   HONESTLY-DECLINED — native reader NULL / non-watertight → OCCT fallback (oracle valid)
#   DISAGREED         — native reader watertight but OUTSIDE tol (a SILENT WRONG IMPORT)
# The bar: DISAGREED == 0. Any DISAGREE prints the seed + case index + family/param tuple
# as a reproducible regression find. The generator is seeded ONLY by an explicit FUZZ_SEED
# (argv/env) — NO clock, NO rand(): same seed → byte-identical batch.
#
# UNLIKE the sibling curved-boolean fuzzer, this domain needs NO numsci: the native writer
# + reader + tessellator depend only on src/native/{exchange,math,topology,tessellate,
# construct} — all OCCT-FREE. It links ONLY the OCCT oracle toolkits (STEPControl_Reader +
# BRepGProp + BRepCheck) and compiles the native exchange + math TUs. src/native stays
# OCCT-FREE; this harness is additive test/sim code only. On run-sim-suite.sh's SKIP list
# (own main(), std::_Exit).
#
# Usage: scripts/run-sim-native-step-import-fuzz.sh [SEED] [N]
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

HARNESS="$REPO/tests/sim/native_step_import_fuzz.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# ── native TUs to compile (OCCT-FREE: exchange writer+reader + math; topology/
#    tessellate/construct are header-only) ─────────────────────────────────────────
NATIVE_SRCS=(
  "$REPO/src/native/exchange/step_writer.cpp"
  "$REPO/src/native/exchange/step_reader.cpp"
  "$REPO/src/native/math/bezier.cpp"
  "$REPO/src/native/math/bspline.cpp"
)
for f in "${NATIVE_SRCS[@]}"; do [ -f "$f" ] || { echo "missing native TU: $f"; exit 1; }; done

# ── OCCT oracle toolkits: STEPControl_Reader (TKDESTEP + TKXSBase) + BRepGProp/
#    BRepCheck (TKTopAlgo/TKBRep) + geometry base ────────────────────────────────────
TKS="TKDESTEP TKXSBase TKDE TKShHealing TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling M6b STEP-import differential-fuzz native-vs-OCCT harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} TU(s) (exchange step_writer/step_reader + math bezier/bspline) [OCCT-FREE, no numsci]"
echo "   oracle  : OCCT STEPControl_Reader + BRepGProp + BRepCheck"
echo "   seed    : $SEED   N : $N"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  -x c++ "${NATIVE_SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_step_import_fuzz"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID (seed=$SEED N=$N)"
xcrun simctl spawn "$UDID" "$OUT/native_step_import_fuzz" "$SEED" "$N"
