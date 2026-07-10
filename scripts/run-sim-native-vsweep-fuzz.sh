#!/usr/bin/env bash
# Compile + run the MOAT M6 VARIABLE-SECTION SWEEP DIFFERENTIAL-FUZZING harness
# (tests/sim/native_vsweep_fuzz.mm) for the iOS simulator. This is an independent native
# domain on the differential-fuzzing completeness bar, extending the landed M6 fuzzers
# (curved-boolean, STEP round-trip, construction, blend fillet/chamfer, wrap-emboss, mass-
# properties, geometry-services, transform-chains, reference/datum, direct-modeling,
# transformed-boolean, orthographic-HLR, shape-healing, section, curved-blend, draft-angle,
# interference, …) to the newly-native VARIABLE-SECTION / guide+spine sweep
# (src/native/construct/sweep.h build_variable_sweep / build_variable_sweep_tube) the CyberCad
# app's variable-boss / shaping-sweep tool drives through the public cc_variable_sweep facade.
# That op has a hand-picked PARITY fixture (native_vsweep_parity, 4 fixtures + 1 deferred) but
# no *fuzz* domain — this closes that gap.
#
# Like run-sim-native-vsweep.sh (the parity sibling), this harness drives the SHIPPING PATH
# through the public cc_* facade under BOTH engines (cc_set_engine(0)=OCCT
# BRepOffsetAPI_MakePipeShell multi-section oracle, cc_set_engine(1)=NativeEngine RMF/perp-
# framed guide-scaled morph tube with an OCCT fallback on honest decline) and compares the two
# results AND a THIRD engine-independent CLOSED-FORM volume arbiter where one exists:
#   circle->circle straight morph  = truncated cone  πH/3(r0²+r0r1+r1²)
#   polygon / section-A->B straight = prismatoid Simpson ∫A(f)·H df  (EXACT — A(f) low-degree)
#   guided straight                = prismatoid with the guide-scale²(f) law folded in
#   circle->circle curved arc      = NO closed form → OCCT-arbitrated (deflection band)
# It additionally reads the native BUILDER's decline signal (build_variable_sweep NULL) to keep
# the HONESTLY-DECLINED bucket meaningful. So it must link the WHOLE kernel — facade + core +
# engine (NativeEngine + the OCCT adapter) + src/native/** — plus the full OCCT toolkit set.
#
# NO NUMSCI. The native variable_sweep path (build_variable_sweep / build_variable_sweep_tube /
# build_loft_along_rail) is OCCT-free AND numsci-free — mirroring run-sim-native-vsweep.sh,
# which links neither. We compile every src/**/*.cpp fresh (same set + flags as
# build-xcframework.sh's slice: -std=c++20 -DCYBERCAD_HAS_OCCT -I include -I src -I opencascade)
# so the harness always tests the CURRENT sweep.h / NativeEngine / cc_set_engine sources. No
# duplicate create_default_engine(): the stub guards its definition behind
# `#ifndef CYBERCAD_HAS_OCCT`, so with OCCT only the OCCT adapter provides it.
#
# Each trial is classified AGREED / HONESTLY-DECLINED (native scoped-out → OCCT) / BOTH-
# DECLINED / ORACLE-INACCURATE (native matches exact math, OCCT the outlier — native
# vindicated) / DISAGREED (a real finding) / ORACLE_UNRELIABLE (oracle untrustworthy AND native
# missed). The bar: DISAGREED==0 AND ORACLE_UNRELIABLE==0, each of the five families (circle/
# polygon/section morph × straight; guided straight; circle-morph curved) with >=1 AGREED. The
# FIXED bands (volX_poly<2e-3 native-vs-exact-math, volX_circle<1.2e-2 for the 64-gon
# inscription, volO<5e-2 areaO<8e-2 native-vs-OCCT) are NEVER widened. The generator is seeded
# ONLY by an explicit FUZZ_SEED (argv/env) — NO clock, NO rand(): same seed → byte-identical
# batch. On run-sim-suite.sh's SKIP list (own main(), std::_Exit).
#
# Usage: scripts/run-sim-native-vsweep-fuzz.sh [SEED] [N]
#   SEED  explicit uint64 RNG seed. If omitted, runs the TWO default seeds below (the >=2-seed
#         bar proof); the script fails if ANY seed fails. Also honoured via FUZZ_SEED.
#   N     number of generated cases (default 72). Also honoured via FUZZ_N env.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

N="${2:-${FUZZ_N:-72}}"
if [ "${1:-}" != "" ]; then SEEDS=("$1"); elif [ "${FUZZ_SEED:-}" != "" ]; then SEEDS=("$FUZZ_SEED"); else SEEDS=("0x5EE9C0FFEE" "0x1CE5EED19"); fi

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_vsweep_fuzz.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# The whole kernel: facade + core + engine (NativeEngine + OCCT adapter + the always-compiled
# stub, which no-ops its create_default_engine under OCCT) + src/native/** (math TUs;
# topology/tessellate/construct are header-only). Same file set build-xcframework.sh compiles.
KERNEL_SRCS=()
while IFS= read -r src; do KERNEL_SRCS+=("$src"); done \
  < <(find "$REPO/src" -name '*.cpp' | sort)
[ "${#KERNEL_SRCS[@]}" -gt 0 ] || { echo "no kernel sources under src"; exit 1; }

# OCCT toolkit link set — the harness exercises the full OCCT adapter through the facade
# (BRepOffsetAPI_MakePipeShell → TKOffset, booleans, meshing, mass/bbox queries, and — since
# occt_exchange.cpp compiles in — STEP/IGES), so link the SAME broad toolkit set
# run-sim-native-vsweep.sh uses (most-derived → base). TKHLR/TKShHealing keep the always-linked
# drafting/healing adapters satisfied at link.
TKS="TKHLR TKDESTEP TKDEIGES TKXSBase TKDE TKMesh TKShHealing TKOffset TKFillet TKBool \
     TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling M6 VARIABLE-SECTION SWEEP differential-fuzz harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   kernel  : ${#KERNEL_SRCS[@]} src TU(s) (facade + core + engine[native+occt] + native math)"
echo "   engines : cc_set_engine(0)=OCCT BRepOffsetAPI_MakePipeShell oracle / cc_set_engine(1)=NativeEngine"
echo "   oracle  : OCCT via facade + CLOSED-FORM volume arbiter (truncated cone / prismatoid Simpson)"
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
  -o "$OUT/native_vsweep_fuzz"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi

rc=0
for SEED in "${SEEDS[@]}"; do
  echo "── running in simulator $UDID (seed=$SEED N=$N)"
  if ! xcrun simctl spawn "$UDID" "$OUT/native_vsweep_fuzz" "$SEED" "$N"; then rc=1; fi
done
exit $rc
