#!/usr/bin/env bash
# Compile + run the MOAT M6 N-SIDED-FILL DIFFERENTIAL-FUZZING harness
# (tests/sim/native_ngon_fill_fuzz.mm) for the iOS simulator. This is an independent native
# domain on the differential-fuzzing completeness bar, extending the landed M6 fuzzers
# (curved-boolean, STEP round-trip, construction, blend fillet/chamfer, wrap-emboss, mass-
# properties, geometry-services, transform-chains, reference/datum, direct-modeling,
# transformed-boolean, orthographic-HLR, shape-healing, section, curved-blend, draft-angle,
# interference) to the newly-native bounded SURFACE PATCH (src/native/surface/{ngon_fill,
# fill_solid}.h) the CyberCad app's N-sided fill / hole-completion tool drives through the
# public cc_fill_ngon facade. That op has a hand-picked PARITY fixture
# (native_surfacing_parity, 4 fixtures + 1 decline) but no *fuzz* domain — this closes it.
#
# Unlike the internal-C++ oracle-slice fuzzers, this harness — like native_draft_faces_fuzz —
# drives the SHIPPING PATH through the public cc_* facade under BOTH engines
# (cc_set_engine(0)=OCCT BRepFill_Filling oracle, cc_set_engine(1)=NativeEngine
# Coons/Gregory tessellated patch with an OCCT fallback on honest decline) and compares the
# two results (area / bbox-containment) AND a CLOSED-FORM polygon-area arbiter (exact for the
# planar-Ngon + hole-completion families) plus an OCCT-independent analytic-boundary residual.
# So it must link the WHOLE kernel — facade + core + engine (NativeEngine + the OCCT adapter)
# + src/native/** — plus the full OCCT toolkit set.
#
# We compile every src/**/*.cpp fresh (same set + flags as build-xcframework.sh's slice:
# -std=c++20 -DCYBERCAD_HAS_OCCT -I include -I src -I opencascade) so the harness always
# tests the CURRENT NativeEngine / cc_set_engine sources. No duplicate
# create_default_engine(): the stub guards its definition behind `#ifndef CYBERCAD_HAS_OCCT`,
# so with OCCT only the OCCT adapter provides it.
#
# NUMSCI IS NOT REQUIRED. NativeEngine::fill_ngon evaluates the OCCT-FREE header-only
# surface::fillNGon (Coons/Gregory transfinite interpolant over src/native/{math,tessellate,
# boolean}) — it is NOT CYBERCAD_HAS_NUMSCI-gated. Every numsci reference in the kernel is
# behind that guard, so the whole kernel links WITHOUT the numsci substrate when the flag is
# undefined (like run-sim-native-curved-blend-fuzz.sh). We still keep TKHLR/TKShHealing in
# the link set to satisfy the always-linked drafting/healing adapters.
#
# Each trial is classified AGREED / HONESTLY-DECLINED (native scoped-out -> OCCT) / BOTH-
# DECLINED / ORACLE-INACCURATE (native matches exact polygon area, OCCT the outlier — native
# vindicated) / DISAGREED (a real finding) / ORACLE_UNRELIABLE (a native-valid non-planar
# patch with no trustworthy oracle). The bar: DISAGREED==0 AND ORACLE_UNRELIABLE==0, each of
# the four families (planar-Ngon / planar-hole-completion / saddle-nonplanar / arc-boundary)
# with >=1 AGREED. The FIXED bands (planar area vs closed-form <1e-4, vs OCCT <1e-3; non-
# planar area vs OCCT <1.2e-1, bbox-containment <8e-2, boundary residual <1e-6) are NEVER
# widened. The generator is seeded ONLY by an explicit FUZZ_SEED (argv/env) — NO clock, NO
# rand(): same seed -> byte-identical batch. On run-sim-suite.sh's SKIP list (own main(),
# std::_Exit).
#
# Usage: scripts/run-sim-native-ngon-fill-fuzz.sh [SEED] [N]
#   SEED  explicit uint64 RNG seed. If omitted, runs the TWO default seeds below (the >=2-
#         seed bar proof); the script fails if ANY seed fails. Also honoured via FUZZ_SEED.
#   N     number of generated cases (default 72). Also honoured via FUZZ_N env.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

N="${2:-${FUZZ_N:-72}}"
if [ "${1:-}" != "" ]; then SEEDS=("$1"); elif [ "${FUZZ_SEED:-}" != "" ]; then SEEDS=("$FUZZ_SEED"); else SEEDS=("0xF117A11FEE" "0x5EEDF111A6"); fi

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_ngon_fill_fuzz.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# The whole kernel: facade + core + engine (NativeEngine + OCCT adapter + the always-
# compiled stub, which no-ops its create_default_engine under OCCT) + src/native/**. Same
# file set build-xcframework.sh compiles into the slice.
KERNEL_SRCS=()
while IFS= read -r src; do KERNEL_SRCS+=("$src"); done \
  < <(find "$REPO/src" -name '*.cpp' | sort)
[ "${#KERNEL_SRCS[@]}" -gt 0 ] || { echo "no kernel sources under src"; exit 1; }

# OCCT toolkit link set — the harness exercises the OCCT adapter through the facade (fill via
# BRepFill_Filling -> TKFillet/TKOffset/TKBool, meshing, mass/bbox queries, GeomAPI, and —
# since occt_exchange.cpp compiles in — STEP/IGES). Link the broad toolkit set the whole-
# kernel fuzzers use (most-derived -> base). TKHLR/TKShHealing keep the always-linked
# drafting/healing adapters satisfied.
TKS="TKHLR TKDESTEP TKDEIGES TKXSBase TKDE TKMesh TKShHealing TKOffset TKFillet TKBool \
     TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling M6 N-SIDED-FILL differential-fuzz harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   kernel  : ${#KERNEL_SRCS[@]} src TU(s) (facade + core + engine[native+occt] + native math)"
echo "   engines : cc_set_engine(0)=OCCT BRepFill_Filling oracle / cc_set_engine(1)=NativeEngine Coons/Gregory patch"
echo "   oracle  : OCCT via facade + CLOSED-FORM polygon-area arbiter (planar) + analytic-boundary residual"
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
  -o "$OUT/native_ngon_fill_fuzz"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi

rc=0
for SEED in "${SEEDS[@]}"; do
  echo "── running in simulator $UDID (seed=$SEED N=$N)"
  if ! xcrun simctl spawn "$UDID" "$OUT/native_ngon_fill_fuzz" "$SEED" "$N"; then rc=1; fi
done
exit $rc
