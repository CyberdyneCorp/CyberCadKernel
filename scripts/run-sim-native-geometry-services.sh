#!/usr/bin/env bash
# Compile + run the MOAT M6-breadth-7 GEOMETRY-SERVICES DIFFERENTIAL-FUZZING harness
# (tests/sim/native_geometry_services_fuzz.mm) for the iOS simulator. This is the SEVENTH
# native domain on the differential-fuzzing completeness bar, extending the six landed M6
# fuzzers — native_boolean_fuzz.mm (curved booleans), native_step_import_fuzz.mm (STEP
# round-trip), native_construct_fuzz.mm (loft/sweep), native_blend_fuzz.mm (fillet/chamfer/
# offset/shell), native_wrap_emboss_fuzz.mm (pads/pockets) and native_mass_props_fuzz.mm
# (mesh mass-properties) — to the geometry-services (GS) analysis/section/drafting layer the
# app's Measure / Curvature / Section / Drawing / Inertia / Check panels read.
#
# The harness DETERMINISTICALLY generates random VALID inputs across the reachable GS services
# and drives each OCCT-FREE native service DIRECTLY plus its OCCT oracle on the SAME geometry:
#   GS3 distance   an::minDistance         vs BRepExtrema_DistShapeShape   (closed-form arbiter)
#   GS4 curvature  an::surfaceCurvature    vs GeomLProp_SLProps            (analytic arbiter)
#   GS2 section    sec::sectionByPlane     vs BRepAlgoAPI_Section+BRepGProp(closed-form arbiter)
#   GS5 inertia    an::principalInertia    vs GProp_PrincipalProps         (exact box tensor)
#   GS6 validity   an::checkSolidMesh      vs construction ground truth (+ BRepCheck on valid)
#   GS1 HLR        drafting::projectOrthographic vs the box-corner closed form (9 vis + 3 hid)
# EVERY service samples its OBLIQUE/TILTED regime (skew segments, tilted faces, oblique cut
# planes incl. the plane_conics oblique-CYLINDER cut → HONEST-NATIVE-DECLINE, rotated solids,
# oblique views). Each trial is classified AGREE / HONEST-NATIVE-DECLINE / DISAGREE /
# ORACLE-INACCURATE / BOTH-DECLINE at a FIXED (never-widened) tolerance. The bar: DISAGREE==0
# with real per-service AGREE coverage (incl. an oblique trial) — run here over ≥2 distinct seeds.
#
# distance.h rides numerics/closest_point at compile time, so this builds the numsci iossim
# archive first (like run-sim-native-analysis.sh) and compiles numerics.cpp + math/*.cpp under
# -DCYBERCAD_HAS_NUMSCI, then links the OCCT oracle toolkits (BRepExtrema + GeomLProp + Geom_* +
# BRepAlgoAPI_Section + BRepGProp + GProp_PrincipalProps + BRepCheck + BRepPrimAPI + ShapeAnalysis).
# src/native / src/engine stay UNTOUCHED; the harness REPRODUCES the native GS path rather than
# modifying it. On run-sim-suite.sh's SKIP list (own main(), std::_Exit).
#
# Usage: scripts/run-sim-native-geometry-services.sh [SEED1] [SEED2] [ROUNDS]
#   SEED1/SEED2  two explicit uint64 RNG seeds (defaults 0x6E5A11C07D, 0xA17F00D2B3).
#   ROUNDS       rounds per seed (each round = one trial per service; default 40 → 240 trials).
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"
NUMPP="${NUMPP_DIR:-/Users/leonardoaraujo/work/NumPP}"
SCIPP="${SCIPP_DIR:-/Users/leonardoaraujo/work/SciPP}"
NUMSCI_OUT="${CYBERCAD_NUMSCI_DIR:-${NUMSCI_DIR:-$REPO/build-numsci/iossim}}"

SEED1="${1:-${FUZZ_SEED:-0x6E5A11C07D}}"
SEED2="${2:-0xA17F00D2B3}"
ROUNDS="${3:-${FUZZ_N:-40}}"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }
[ -d "$NUMPP/include/numpp" ] || { echo "NumPP not found at $NUMPP"; exit 1; }
[ -d "$SCIPP/include/scipp" ] || { echo "SciPP not found at $SCIPP"; exit 1; }

HARNESS="$REPO/tests/sim/native_geometry_services_fuzz.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# ── numsci iossim substrate (distance.h → numerics/closest_point) ─────────────
NUMSCI_LIB="$NUMSCI_OUT/libnumsci_iossim_arm64.a"
if [ ! -f "$NUMSCI_LIB" ] || [ -n "${NUMSCI_REBUILD:-}" ]; then
  echo "── building NumPP/SciPP substrate (iossim) via scripts/build-numsci.sh"
  "$REPO/scripts/build-numsci.sh" iossim
fi
[ -f "$NUMSCI_LIB" ] || { echo "numsci archive not built: $NUMSCI_LIB"; exit 1; }
NUMSCI_GEN="$NUMSCI_OUT/gen"
[ -d "$NUMSCI_GEN/numpp" ] || { echo "numsci gen headers missing: $NUMSCI_GEN"; exit 1; }

# ── native TUs (OCCT-free: numeric facade + math impl; section/inertia/validity/hlr header-only)
NUMERICS_SRC="$REPO/src/native/numerics/numerics.cpp"
[ -f "$NUMERICS_SRC" ] || { echo "missing $NUMERICS_SRC"; exit 1; }
MATH_SRCS=()
while IFS= read -r src; do MATH_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp' | sort)
[ "${#MATH_SRCS[@]}" -gt 0 ] || { echo "no native math sources under src/native/math"; exit 1; }

# ── OCCT oracle toolkits (union across the six services) ──────────────────────
TKS="TKMesh TKShHealing TKOffset TKFillet TKBool TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep \
     TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling M6-breadth-7 GEOMETRY-SERVICES differential-fuzz native-vs-OCCT harness (iphonesimulator arm64)"
echo "   harness  : $HARNESS"
echo "   numerics : $NUMERICS_SRC (-DCYBERCAD_HAS_NUMSCI)"
echo "   math     : ${#MATH_SRCS[@]} source(s)  substrate: $NUMSCI_LIB"
echo "   oracle   : BRepExtrema + GeomLProp + BRepAlgoAPI_Section + BRepGProp + GProp_PrincipalProps + BRepCheck + BRepPrimAPI"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT -DCYBERCAD_HAS_NUMSCI \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -I"$NUMPP/include" \
  -I"$SCIPP/include" \
  -I"$NUMSCI_GEN" \
  -x objective-c++ "$HARNESS" \
  -x c++ "$NUMERICS_SRC" \
  "${MATH_SRCS[@]}" \
  -x none "$NUMSCI_LIB" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_geometry_services_fuzz"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi

RC=0
for SEED in "$SEED1" "$SEED2"; do
  echo "── running in simulator $UDID (seed=$SEED rounds=$ROUNDS)"
  xcrun simctl spawn "$UDID" "$OUT/native_geometry_services_fuzz" "$SEED" "$ROUNDS" || RC=$?
done
exit $RC
