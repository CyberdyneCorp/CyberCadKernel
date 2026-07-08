#!/usr/bin/env bash
# Compile + run the MOAT M6-breadth-6 MASS-PROPERTIES DIFFERENTIAL-FUZZING harness
# (tests/sim/native_mass_props_fuzz.mm) for the iOS simulator. This is the SIXTH native
# domain on the differential-fuzzing completeness bar, extending the landed M6 fuzzers —
# native_boolean_fuzz.mm (curved booleans), native_step_import_fuzz.mm (STEP round-trip),
# native_construct_fuzz.mm (loft/sweep) and native_blend_fuzz.mm (fillet/chamfer/offset/
# shell) — to the mesh-based mass-properties query layer the app's MassReadout / Inertia /
# Measure panels read (NativeEngine::mass_properties).
#
# The harness DETERMINISTICALLY generates random-but-VALID solids across the native mass
# families — BOX / NGON prism (planar, exact), CYLINDER / CONE (apex) / SPHERE / REVOLVE
# tube (curved, deflection-bounded), coaxial-frustum LOFT (planar, exact) — plus a sparse
# degenerate decline-exerciser. Each solid is built by the OCCT-FREE native builders and
# measured by REPRODUCING NativeEngine::mass_properties EXACTLY (mesh @ kPropertyDeflection
# 0.005 → surfaceArea / |enclosedVolume| / signed-tetra centroid / watertight-validity),
# AND by the OCCT oracle (BRepPrimAPI / ThruSections built in the SAME frame, measured by
# BRepGProp VolumeProperties + SurfaceProperties + GProp_PrincipalProps), then classified:
#   AGREED            — native valid + vol/area/centroid within the family tol of the
#                       CLOSED-FORM analytic truth (planar tight; curved deflection bound).
#   HONESTLY-DECLINED — native mesh non-watertight / NULL → no valid mass → OCCT ships (the
#                       CONE FRUSTUM sub-family lands here — its native mesh does not weld
#                       watertight at 0.005; OCCT's cone is valid).
#   DISAGREED         — native valid but OUTSIDE the analytic truth (SILENT WRONG MASS).
#   ORACLE-INACCURATE — native matches exact math while OCCT does not (native vindicated).
#   BOTH-DECLINED     — a degenerate input both engines refuse.
#   ORACLE_UNRELIABLE — a core-family OCCT build that does NOT match the closed form.
# The bar: DISAGREED == 0 AND ORACLE_UNRELIABLE == 0 with each AGREE family (BOX / NGON /
# CYLINDER / CONE / SPHERE / LOFT / REVOLVE) having ≥1 AGREED trial. INERTIA is an HONEST
# NATIVE DECLINE for every body (native principal_moments = CC_NATIVE_BODY_UNSUPPORTED; the
# OCCT moments are logged as telemetry only). The generator is seeded ONLY by an explicit
# FUZZ_SEED (argv/env) — NO clock, NO rand(): same seed → byte-identical batch.
#
# Like native_construct_fuzz this domain needs NO numsci: the native construct + tessellate
# + topology live in src/native/** — OCCT-FREE and header-only (math bezier/bspline are the
# only compiled native TUs). It links ONLY the OCCT oracle toolkits (BRepPrimAPI/ThruSections
# + BRepGProp + BRepCheck). src/native / src/engine stay UNTOUCHED; this harness REPRODUCES
# the native mass path rather than modifying it. On run-sim-suite.sh's SKIP list (own main()).
#
# Usage: scripts/run-sim-native-mass-props-fuzz.sh [SEED] [N]
#   SEED  explicit uint64 RNG seed (default 0x6D3A11C05B). Also honoured via FUZZ_SEED env.
#   N     number of generated cases (default 120).          Also honoured via FUZZ_N env.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

SEED="${1:-${FUZZ_SEED:-0x6D3A11C05B}}"
N="${2:-${FUZZ_N:-120}}"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_mass_props_fuzz.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# native TUs to compile (OCCT-FREE: math bezier/bspline; construct/tessellate/topology header-only)
NATIVE_SRCS=(
  "$REPO/src/native/math/bezier.cpp"
  "$REPO/src/native/math/bspline.cpp"
)
for f in "${NATIVE_SRCS[@]}"; do [ -f "$f" ] || { echo "missing native TU: $f"; exit 1; }; done

# OCCT oracle toolkits: BRepPrimAPI (TKPrim) + ThruSections (TKOffset) + BRepGProp (TKGeomAlgo)
# + BRepCheck/BRepBuilderAPI (TKTopAlgo/TKBRep) + geometry base.
TKS="TKMesh TKShHealing TKOffset TKFillet TKBool TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep \
     TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling M6-breadth-6 MASS-PROPERTIES differential-fuzz native-vs-OCCT harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} TU(s) (math bezier/bspline) [OCCT-FREE, no numsci; construct/tessellate/topology header-only]"
echo "   oracle  : OCCT BRepPrimAPI + BRepOffsetAPI_ThruSections + BRepGProp + GProp_PrincipalProps"
echo "   seed    : $SEED   N : $N"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  -x c++ "${NATIVE_SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_mass_props_fuzz"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID (seed=$SEED N=$N)"
xcrun simctl spawn "$UDID" "$OUT/native_mass_props_fuzz" "$SEED" "$N"
