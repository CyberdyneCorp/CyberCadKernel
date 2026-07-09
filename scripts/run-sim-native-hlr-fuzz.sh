#!/usr/bin/env bash
# Compile + run the MOAT M6-breadth-13 ORTHOGRAPHIC-HLR / DRAFTING DIFFERENTIAL-FUZZING
# harness (tests/sim/native_hlr_fuzz.mm) for the iOS simulator. This is the THIRTEENTH
# native domain on the differential-fuzzing completeness bar, extending the landed M6
# fuzzers to the HLR / drafting service the CyberCad app's drawing views drive
# (cc_hlr_project). GS1 HLR already has a curated per-solid PARITY fixture
# (native_hlr_parity.mm) and the geometry-services fuzzer holds the OCCT-free HLR CORE to
# a box-corner closed form — but neither turns the SHIPPING cc_* HLR differential into a
# seeded batch of RANDOM solids at RANDOM rigid poses from RANDOM view directions. This
# closes that gap.
#
# The harness DETERMINISTICALLY generates random VALID solids from six families (box,
# n-gon prism, cylinder, cone/frustum, sphere, freeform-decline) at random rigid poses,
# projects each from a random view direction through cc_hlr_project under BOTH engines
# (cc_set_engine(0)=OCCT HLRBRep_Algo oracle, cc_set_engine(1)=NativeEngine orthographic
# HLR + silhouette core), and compares the visible/hidden 2D drawing-plane segment SETS
# (counts, total lengths, visible/hidden partition point-coverage) to a deflection-matched
# tolerance, with a CLOSED-FORM silhouette-tangency arbiter (n·view=0) for the cylinder +
# sphere families. Each trial is classified:
#   AGREED             — both drew, counts/length/partition within tol
#   HONESTLY-DECLINED  — native EMPTY (freeform B-spline bands) -> OCCT oracle draws
#   DISAGREED          — native drew but OUTSIDE tol / a misclassified segment (SILENT WRONG)
#   ORACLE_UNRELIABLE  — native matches the closed-form silhouette, OCCT is the outlier
#                        (native vindicated — prior fuzzers found native MORE correct)
# The bar: DISAGREED == 0. Any DISAGREE prints the seed + case index + full param tuple as
# a reproducible regression find. The tolerances are FIXED and NEVER widened. The
# generator is seeded ONLY by an explicit FUZZ_SEED (argv/env) — NO clock, NO rand(): same
# seed → byte-identical batch.
#
# Like native_hlr_parity / native_directmodel_fuzz (and UNLIKE the internal-C++ fuzzers)
# this drives the SHIPPING PATH through the public cc_* facade under BOTH engines, so it
# must link the WHOLE kernel — facade + core + engine (NativeEngine + the OCCT adapter,
# the fallthrough target under CYBERCAD_HAS_OCCT) + src/native/** — plus the full OCCT
# toolkit set incl. TKHLR (HLRBRep_Algo / HLRBRep_HLRToShape, the HLR oracle
# occt_drafting.cpp links). NO numsci: the HLR path (orthographic_hlr.h + silhouette.h +
# the tessellator occluder) is not numsci-gated, so — unlike run-sim-native-directmodel-
# fuzz.sh — this mirrors run-sim-native-hlr.sh's plain-kernel slice.
#
# We compile every src/**/*.cpp fresh (same set + flags as build-xcframework.sh's slice:
# -std=c++20 -DCYBERCAD_HAS_OCCT -I include -I src -I opencascade) so the harness always
# tests the CURRENT NativeEngine / cc_set_engine sources. No duplicate
# create_default_engine(): the stub guards its definition behind `#ifndef
# CYBERCAD_HAS_OCCT`, so with OCCT only the OCCT adapter provides it. src/native stays
# OCCT-FREE; this harness is additive test/sim code only. On run-sim-suite.sh's SKIP list
# (own main(), links the whole kernel + OCCT directly, std::_Exit).
#
# Usage: scripts/run-sim-native-hlr-fuzz.sh [SEED] [N]
#   SEED  explicit uint64 RNG seed. If omitted, runs the TWO default seeds below (the >=2-
#         seed bar proof); the script fails if ANY seed fails. Also honoured via FUZZ_SEED.
#   N     number of generated cases (default 60). Also honoured via FUZZ_N env.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

N="${2:-${FUZZ_N:-60}}"
if [ "${1:-}" != "" ]; then SEEDS=("$1"); elif [ "${FUZZ_SEED:-}" != "" ]; then SEEDS=("$FUZZ_SEED"); else SEEDS=("0x4D6F617436" "0x171313C0FFEE"); fi

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_hlr_fuzz.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# The whole kernel: facade + core + engine (NativeEngine + OCCT adapter + the always-
# compiled stub, which no-ops its create_default_engine under OCCT) + src/native/**.
KERNEL_SRCS=()
while IFS= read -r src; do KERNEL_SRCS+=("$src"); done \
  < <(find "$REPO/src" -name '*.cpp' | sort)
[ "${#KERNEL_SRCS[@]}" -gt 0 ] || { echo "no kernel sources under src"; exit 1; }

# OCCT toolkit link set (most-derived → base). TKHLR provides HLRBRep_Algo /
# HLRBRep_HLRToShape — the HLR oracle occt_drafting.cpp links; identical set to
# run-sim-native-hlr.sh.
TKS="TKHLR TKDESTEP TKDEIGES TKXSBase TKDE TKMesh TKShHealing TKOffset TKFillet TKBool \
     TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling M6-breadth-13 ORTHOGRAPHIC-HLR differential-fuzz harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   kernel  : ${#KERNEL_SRCS[@]} src TU(s) (facade + core + engine[native+occt] + native math)"
echo "   engines : cc_set_engine(0)=OCCT HLRBRep oracle / cc_set_engine(1)=NativeEngine"
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
  -o "$OUT/native_hlr_fuzz"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi

rc=0
for SEED in "${SEEDS[@]}"; do
  echo "── running in simulator $UDID (seed=$SEED N=$N)"
  if ! xcrun simctl spawn "$UDID" "$OUT/native_hlr_fuzz" "$SEED" "$N"; then rc=1; fi
done
exit $rc
