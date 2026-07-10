#!/usr/bin/env bash
# Compile + run the MOAT M6-breadth WRAP-EMBOSS FREEFORM-BASE DIFFERENTIAL-FUZZING harness
# (tests/sim/native_wrap_emboss_freeform_fuzz.mm) for the iOS simulator. This closes the gap
# the landed cylinder-only native_wrap_emboss_fuzz.mm left: it certifies the NEW F5 arm of
# src/native/feature/wrap_emboss.h — a RAISED circular pole boss on a FREEFORM sphere-cap
# dome (a NON-developable base) — alongside the cylinder base as a developable control, both
# driven through the SHIPPING cc_wrap_emboss facade under BOTH engines (cc_set_engine(0)=OCCT,
# cc_set_engine(1)=NativeEngine).
#
# The harness DETERMINISTICALLY generates random-but-VALID wrap-emboss poses per base×mode
# cell — {cylinder, sphere-cap} × {raised(boss=1), recessed(boss=0)} — plus SPARSE
# out-of-envelope exercisers (box base, >2π footprint, deboss depth ≥ R, self-intersecting
# pentagram, boss reaching the dome rim) that route to the native NULL → OCCT / closed-form-
# declines honest branch. Each pose is embossed through cc_wrap_emboss under both engines and
# arbitrated:
#   * CYLINDER (developable): PRIMARY the closed-form changed volume A·|Rout²−R²|/(2R); OCCT
#     (same cc_wrap_emboss under engine 0) is a SECONDARY vol+area cross-check.
#   * SPHERE-CAP (NON-developable): OCCT's cc_wrap_emboss DECLINES the sphere wall (asserted),
#     so the CLOSED-FORM shell-sector delta 2π(1−cosφ0)·((R+h)³−R³)/3 is the SOLE boss
#     arbiter, added to the base-dome volume OCCT DOES measure exactly (cc_mass_properties).
# Each trial is classified AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE_UNRELIABLE /
# BOTH-DECLINED. The bar: DISAGREED == 0 && ORACLE_UNRELIABLE == 0, each in-scope cell ≥1
# AGREED, no guard-leak SURPRISE. The generator is seeded ONLY by an explicit FUZZ_SEED
# (argv/env) — NO clock, NO rand(): same seed → byte-identical batch.
#
# Like the sibling native_wrap_emboss_parity harness this drives the SHIPPING PATH: the public
# cc_* facade under both engines. So it links the WHOLE kernel — facade + core + engine
# (NativeEngine + the OCCT adapter, the fallthrough target under CYBERCAD_HAS_OCCT) +
# src/native/** — plus the full OCCT toolkit set (the OCCT adapter's wrap_emboss reaches
# BRepBuilderAPI_Sewing / ShapeFix / BRepAlgoAPI_Fuse, revolve/prism, meshing, mass queries;
# TKHLR/TKShHealing retained for the link). The native wrap-emboss path is OCCT-FREE and NOT
# CYBERCAD_HAS_NUMSCI-gated, so this needs NO numsci substrate.
#
# Companion to run-sim-suite.sh; on that suite's SKIP list (own main(), links the whole kernel
# + OCCT directly rather than the packaged static lib).
#
# Usage: scripts/run-sim-native-wrap-emboss-freeform-fuzz.sh [SEED] [N]
#   SEED  explicit uint64 RNG seed. Also honoured via FUZZ_SEED env.
#   N     number of generated cases per seed (default 64). Also honoured via FUZZ_N env.
# With NO SEED argument it runs TWO default seeds (N each) and fails if EITHER fails.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_wrap_emboss_freeform_fuzz.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# The whole kernel: facade + core + engine (NativeEngine + OCCT adapter + the always-compiled
# stub, no-op create_default_engine under OCCT) + src/native/** (math TUs; feature/topology/
# tessellate/construct/boolean header-only). Same file set build-xcframework.sh compiles.
KERNEL_SRCS=()
while IFS= read -r src; do KERNEL_SRCS+=("$src"); done < <(find "$REPO/src" -name '*.cpp' | sort)
[ "${#KERNEL_SRCS[@]}" -gt 0 ] || { echo "no kernel sources under src"; exit 1; }

# OCCT toolkit link set (most-derived → base), same broad set the parity runner uses.
TKS="TKDESTEP TKDEIGES TKXSBase TKDE TKMesh TKShHealing TKHLR TKOffset TKFillet TKBool \
     TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling M6-breadth WRAP-EMBOSS FREEFORM-BASE differential-fuzz harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   kernel  : ${#KERNEL_SRCS[@]} src TU(s) (facade + core + engine[native+occt] + native math)"
echo "   oracle  : closed-form changed volume (cylinder) / shell-sector delta (sphere-cap, SOLE arbiter — OCCT declines the sphere wall)"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios14.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/include" \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  "${KERNEL_SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_wrap_emboss_freeform_fuzz"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi

N="${2:-${FUZZ_N:-64}}"
run_one() {
  local seed="$1"
  echo "── running in simulator $UDID (seed=$seed N=$N)"
  xcrun simctl spawn "$UDID" "$OUT/native_wrap_emboss_freeform_fuzz" "$seed" "$N"
}

if [ "${1:-}" != "" ] || [ "${FUZZ_SEED:-}" != "" ]; then
  run_one "${1:-$FUZZ_SEED}"
else
  # TWO default seeds; fail if EITHER fails.
  rc=0
  run_one "0x5745E6F00B" || rc=1
  run_one "0xB0551FEED2" || rc=1
  exit $rc
fi
