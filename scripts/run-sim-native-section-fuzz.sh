#!/usr/bin/env bash
# Compile + run the MOAT M6-breadth-12 SECTION-CURVES DIFFERENTIAL-FUZZING harness
# (tests/sim/native_section_fuzz.mm) for the iOS simulator. This is the TWELFTH native
# domain on the differential-fuzzing completeness bar, extending the eleven landed M6
# fuzzers — native_boolean_fuzz.mm (curved booleans), native_step_import_fuzz.mm (STEP
# round-trip), native_construct_fuzz.mm (loft/sweep), native_blend_fuzz.mm (blends),
# native_wrap_emboss_fuzz.mm, native_mass_props_fuzz.mm, native_geometry_services_fuzz.mm,
# native_transform_fuzz.mm, native_reference_geometry_fuzz.mm, native_directmodel_fuzz.mm
# and native_transformed_boolean_fuzz.mm — to the native planar SECTION-CURVE service
# (cybercad::native::section::sectionByPlane) the app's drawing / section-view path reads
# through the additive cc_section_plane facade.
#
# The harness DETERMINISTICALLY generates random-but-VALID primitives AND cut planes across
# the section AGREE families — BOX (axis cut → rectangle), CYL perpendicular (→ circle),
# CYL axial (→ rectangle), CYL OBLIQUE (→ ellipse, in-band), SPHERE (→ circle) — plus a
# DECLINE exerciser (plane missing / coincident / tangent). Each native solid is built by
# the OCCT-FREE native ShapeBuilder fixtures and sectioned by the native service; the
# matched OCCT solid is built independently with BRepPrimAPI and sectioned by
# BRepAlgoAPI_Section (loop count via ShapeAnalysis_FreeBounds, edge length via
# GCPnts_AbscissaPoint::Length, capped area via BRepGProp::SurfaceProperties). Each trial is
# arbitrated against a THIRD engine-independent CLOSED-FORM conic (rectangle / circle /
# ellipse perimeter + area) and classified:
#   AGREED            — native section = closed-form analytic = OCCT within the family tol
#                       (tight 1e-9 for straight/circular; 1e-4 for the ellipse perimeter —
#                       the Ramanujan-II + OCCT arc-length bound native_section_parity
#                       proved, NEVER widened).
#   HONESTLY-DECLINED — native reported Empty/Declined (plane missing / coincident /
#                       tangent, or a numerically marginal cut its self-verify rejects) →
#                       OCCT ships. First-class, logged, NOT a bar failure.
#   DISAGREED         — native produced a section OUTSIDE the analytic truth, or produced a
#                       section on a config it should have declined (SILENT WRONG SECTION).
#   ORACLE_UNRELIABLE — native matches exact math while OCCT does not / OCCT produced no
#                       usable section where native+analytic cover it (native vindicated).
# The bar: DISAGREED == 0 AND ORACLE_UNRELIABLE == 0 with each AGREE family (BOX /
# CYL_PERP / CYL_AXIAL / CYL_OBL / SPHERE) having ≥1 AGREED trial and the decline exerciser
# ≥1 HONESTLY-DECLINED trial. The generator is seeded ONLY by an explicit FUZZ_SEED
# (argv/env) — NO clock, NO rand(): same seed → byte-identical batch.
#
# Like native_section_parity this domain needs NO numsci: the native section service is the
# OCCT-FREE SSI Stage-S1 header path + header-only topology, so this script compiles only
# the OCCT-free native math TUs (bezier/bspline) alongside the header-only section /
# topology / ssi-S1 headers, and links the OCCT oracle toolkits (BRepPrimAPI +
# BRepAlgoAPI_Section + ShapeAnalysis_FreeBounds + BRepGProp). src/native / src/engine /
# include stay UNTOUCHED; this harness DRIVES the native section boundary rather than
# modifying it. On run-sim-suite.sh's SKIP list (own main(), OCCT slice).
#
# Usage: scripts/run-sim-native-section-fuzz.sh [SEED] [N]
#   SEED  explicit uint64 RNG seed (default: runs the two default seeds below).
#         Also honoured via FUZZ_SEED env. If given, ONLY that seed is run.
#   N     number of generated cases per seed (default 96). Also honoured via FUZZ_N env.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

# ≥2 default seeds; a single explicit seed (argv/env) overrides to just that one.
DEFAULT_SEEDS=(0x5EC7104FEED 0x1CE5EC12ABC)
if [ $# -ge 1 ]; then SEEDS=("$1"); elif [ -n "${FUZZ_SEED:-}" ]; then SEEDS=("$FUZZ_SEED"); else SEEDS=("${DEFAULT_SEEDS[@]}"); fi
N="${2:-${FUZZ_N:-96}}"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_section_fuzz.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# ── native TUs to compile (OCCT-free math; section/topology/ssi-S1 are header-only) ──
NATIVE_SRCS=()
while IFS= read -r src; do NATIVE_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp')

# ── OCCT oracle toolkits (same set as run-sim-native-section.sh) ───────────────────
# BRepPrimAPI_Make{Box,Cylinder,Sphere} → TKPrim; BRepAlgoAPI_Section → TKBO (deps
# TKBool/TKTopAlgo); ShapeAnalysis_FreeBounds → TKShHealing; BRepBuilderAPI_MakeFace →
# TKTopAlgo; BRepGProp/GProp → TKGeomAlgo/TKBRep. Listed most-derived → base.
TKS="TKBO TKPrim TKShHealing TKBool TKFillet TKOffset TKTopAlgo TKGeomAlgo TKBRep \
     TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling M6-breadth-12 SECTION-CURVES differential-fuzz native-vs-OCCT harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} TU(s) (math bezier/bspline) [OCCT-FREE, no numsci; section/topology/ssi-S1 header-only]"
echo "   oracle  : OCCT BRepPrimAPI + BRepAlgoAPI_Section + ShapeAnalysis_FreeBounds + BRepGProp"
echo "   seeds   : ${SEEDS[*]}   N : $N"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  -x c++ "${NATIVE_SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_section_fuzz"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi

rc=0
for SEED in "${SEEDS[@]}"; do
  echo "── running in simulator $UDID (seed=$SEED N=$N)"
  if ! xcrun simctl spawn "$UDID" "$OUT/native_section_fuzz" "$SEED" "$N"; then
    echo "!! seed $SEED FAILED the bar"; rc=1
  fi
done
exit $rc
