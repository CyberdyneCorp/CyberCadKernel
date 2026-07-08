#!/usr/bin/env bash
# Compile + run the MOAT M6-breadth-8 TRANSFORM-CHAIN DIFFERENTIAL-FUZZING harness
# (tests/sim/native_transform_fuzz.mm) for the iOS simulator. This is the EIGHTH native
# domain on the differential-fuzzing completeness bar, extending the landed M6 fuzzers —
# native_boolean_fuzz.mm, native_step_import_fuzz.mm, native_construct_fuzz.mm,
# native_blend_fuzz.mm, native_mass_props_fuzz.mm and native_wrap_emboss_fuzz.mm — to the
# RIGID/SIMILARITY TRANSFORM layer the app's translate/rotate/scale/mirror/place tools read
# (cc_translate_shape / cc_rotate_shape_about / cc_scale_shape / cc_mirror_shape /
# cc_place_on_frame → the native topology::Shape::located(math::Transform) path).
#
# The harness DETERMINISTICALLY generates a random VALID base solid (BOX / NGON prism /
# CYLINDER / SPHERE / coaxial LOFT), applies a random CHAIN of translate / rotate(any axis) /
# uniform-scale / mirror(any plane) as ONE composed native math::Transform, meshes the located
# solid and measures it, then compares against BOTH the OCCT oracle (BRepBuilderAPI_Transform
# with the SAME gp_Trsf chain, measured by BRepGProp) AND a THIRD engine-independent closed-form
# analytic similarity image (vol'=S^3*vol, area'=S^2*area, centroid'=L*C+t, topology invariant,
# mirror flips the signed-volume sign). Each trial is classified:
#   AGREED            — native watertight + |vol|>0, native vol/area/centroid match the analytic
#                       image, topology preserved, handedness (mirror-parity) correct, OCCT concurs.
#   HONESTLY-DECLINED — a singular (zero-scale) transform collapses the native solid; OCCT ships.
#   DISAGREED         — native valid but its transform result / handedness / topology is WRONG.
#   ORACLE-INACCURATE — native matches the analytic image, OCCT does not (native vindicated).
#   BOTH-DECLINED     — a singular-transform exerciser both engines refuse.
#   ORACLE_UNRELIABLE — a core case whose OCCT oracle does not match the closed form.
# The bar: DISAGREED==0 AND ORACLE_UNRELIABLE==0, each base family + each transform KIND with
# ≥1 AGREED, and the mirror HANDEDNESS-FLIP positively confirmed ≥1 time. The generator is
# seeded ONLY by an explicit FUZZ_SEED (argv/env) — NO clock, NO rand(): same seed → identical batch.
#
# Like native_mass_props_fuzz this domain needs NO numsci: the native construct + tessellate +
# topology + math transform live in src/native/** (OCCT-FREE, header-only bar bezier/bspline).
# It links ONLY the OCCT oracle toolkits. src/native / src/engine stay UNTOUCHED; this harness
# DRIVES the native located()+tessellate path rather than modifying it. On run-sim-suite.sh's
# SKIP list (own main()).
#
# Usage: scripts/run-sim-native-transform-fuzz.sh [SEED] [N]
#   SEED  explicit uint64 RNG seed. If omitted, runs the TWO default seeds below (the ≥2-seed
#         bar proof); the script fails if ANY seed fails. Also honoured via FUZZ_SEED env.
#   N     number of generated cases (default 160). Also honoured via FUZZ_N env.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

N="${2:-${FUZZ_N:-160}}"
if [ "${1:-}" != "" ]; then SEEDS=("$1"); elif [ "${FUZZ_SEED:-}" != "" ]; then SEEDS=("$FUZZ_SEED"); else SEEDS=("0x7A5C0FFEE2" "0xB19D0C2A77"); fi

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_transform_fuzz.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

NATIVE_SRCS=(
  "$REPO/src/native/math/bezier.cpp"
  "$REPO/src/native/math/bspline.cpp"
)
for f in "${NATIVE_SRCS[@]}"; do [ -f "$f" ] || { echo "missing native TU: $f"; exit 1; }; done

TKS="TKMesh TKShHealing TKOffset TKFillet TKBool TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep \
     TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling M6-breadth-8 TRANSFORM-CHAIN differential-fuzz native-vs-OCCT harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} TU(s) (math bezier/bspline) [OCCT-FREE, no numsci; construct/tessellate/topology/transform header-only]"
echo "   oracle  : OCCT BRepBuilderAPI_Transform(gp_Trsf) + BRepGProp"
echo "   seeds   : ${SEEDS[*]}   N : $N"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  -x c++ "${NATIVE_SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_transform_fuzz"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi

rc=0
for SEED in "${SEEDS[@]}"; do
  echo "── running in simulator $UDID (seed=$SEED N=$N)"
  if ! xcrun simctl spawn "$UDID" "$OUT/native_transform_fuzz" "$SEED" "$N"; then rc=1; fi
done
exit $rc
