#!/usr/bin/env bash
# Compile + run the MOAT M6-breadth-9 REFERENCE/DATUM-GEOMETRY DIFFERENTIAL-FUZZING
# harness (tests/sim/native_reference_geometry_fuzz.mm) for the iOS simulator. This is
# the NINTH native domain on the differential-fuzzing completeness bar, extending the
# landed M6 fuzzers — native_boolean_fuzz.mm, native_step_import_fuzz.mm,
# native_construct_fuzz.mm, native_blend_fuzz.mm, native_wrap_emboss_fuzz.mm,
# native_mass_props_fuzz.mm, native_geometry_services_fuzz.mm and native_transform_fuzz.mm
# — to the REFERENCE / DATUM GEOMETRY layer the CyberCad app's datum/reference tools read
# (cc_face_axis / cc_ref_axis_from_face / cc_ref_plane_from_face / cc_ref_axis_from_edge /
# cc_tangent_chain / cc_outer_rim_chain / cc_offset_face_boundary → the OCCT-FREE,
# header-only src/native/reference/reference.h services).
#
# The harness DETERMINISTICALLY generates a random VALID base solid (BOX / NGON prism /
# CYLINDER / CONE frustum) via the OCCT-FREE native builders, applies a random RIGID pose
# (rotate + translate, NO scale/mirror — so every datum transforms EXACTLY) via
# Shape::located(math::Transform), and drives every reference op on the posed native solid,
# comparing each result against BOTH the OCCT oracle (gp_Cylinder/gp_Cone::Axis, gp_Pln,
# gp_Lin, BRepOffsetAPI_MakeOffset, D1 tangent) AND a THIRD engine-independent closed-form
# analytic arbiter (the known construction datum transformed by the same pose in plain fp64).
# Each op trial is classified:
#   AGREED            — native datum matches the closed-form analytic image AND OCCT.
#   HONESTLY-DECLINED — native returns nullopt/empty on a case reference.h scopes out
#                       (circular cap offset, freeform edge in a tangent walk) where the
#                       closed form also says "no closed-form datum".
#   DISAGREED         — native returned a datum that does NOT match the analytic image.
#   ORACLE-INACCURATE — native matches the analytic image, OCCT does not (native vindicated).
#   BOTH-DECLINED     — a scoped-out exerciser both engines refuse.
#   ORACLE_UNRELIABLE — a core case whose OCCT oracle does not match the closed form.
# The bar: DISAGREED==0 AND ORACLE_UNRELIABLE==0, each base family + each op with ≥1 AGREED.
# The generator is seeded ONLY by an explicit FUZZ_SEED (argv/env) — NO clock, NO rand():
# same seed → byte-identical batch.
#
# Like native_reference_parity/native_transform_fuzz this domain needs NO numsci: the
# native reference/construct/tessellate/topology/math path lives in src/native/**
# (OCCT-FREE, header-only bar math bezier/bspline). It links ONLY the OCCT oracle toolkits.
# src/native / src/engine stay UNTOUCHED; this harness DRIVES the reference services +
# located() path rather than modifying it. On run-sim-suite.sh's SKIP list (own main()).
#
# Usage: scripts/run-sim-native-reference-geometry-fuzz.sh [SEED] [N]
#   SEED  explicit uint64 RNG seed. If omitted, runs the TWO default seeds below (the ≥2-seed
#         bar proof); the script fails if ANY seed fails. Also honoured via FUZZ_SEED env.
#   N     number of generated cases (default 96). Also honoured via FUZZ_N env.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

N="${2:-${FUZZ_N:-96}}"
if [ "${1:-}" != "" ]; then SEEDS=("$1"); elif [ "${FUZZ_SEED:-}" != "" ]; then SEEDS=("$FUZZ_SEED"); else SEEDS=("0x9EF12A0055" "0xC0DEDA7A11"); fi

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_reference_geometry_fuzz.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# native math TUs (the header aggregate references bezier/bspline). No numsci link.
NATIVE_SRCS=(
  "$REPO/src/native/math/bezier.cpp"
  "$REPO/src/native/math/bspline.cpp"
)
for f in "${NATIVE_SRCS[@]}"; do [ -f "$f" ] || { echo "missing native TU: $f"; exit 1; }; done

# OCCT oracle toolkits. gp_Cone/gp_Cylinder/gp_Pln/gp_Lin + BRepAdaptor → TKGeomBase/TKBRep;
# BRepPrimAPI_Make{Box,Cylinder,Cone,Prism} → TKPrim; BRepOffsetAPI_MakeOffset → TKOffset;
# BRepBuilderAPI_Transform → TKTopAlgo; TKHLR/TKShHealing keep the always-linked drafting
# adapter satisfied. Most-derived → base order.
TKS="TKHLR TKOffset TKFillet TKBool TKShHealing TKMesh TKPrim TKBO TKTopAlgo TKGeomAlgo \
     TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling M6-breadth-9 REFERENCE/DATUM-GEOMETRY differential-fuzz native-vs-OCCT harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} TU(s) (math bezier/bspline) [OCCT-FREE, no numsci; reference/construct/topology/math header-only]"
echo "   oracle  : OCCT gp_Cyl/Cone::Axis + gp_Pln + gp_Lin + BRepOffsetAPI_MakeOffset + D1"
echo "   seeds   : ${SEEDS[*]}   N : $N"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  -x c++ "${NATIVE_SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_reference_geometry_fuzz"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi

rc=0
for SEED in "${SEEDS[@]}"; do
  echo "── running in simulator $UDID (seed=$SEED N=$N)"
  if ! xcrun simctl spawn "$UDID" "$OUT/native_reference_geometry_fuzz" "$SEED" "$N"; then rc=1; fi
done
exit $rc
