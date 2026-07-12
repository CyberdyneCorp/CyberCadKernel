#!/usr/bin/env bash
# Compile the BOOL-INT L3 two-freeform-solid NURBS boolean orchestrator
# native-vs-OCCT parity harness for the iOS simulator and run it in a booted simulator
# via `xcrun simctl spawn`.
#
# This is SIM GATE (b) of the two-gate M2-freeform-freeform model. The native multi-face
# corner-clip weld (src/native/boolean/multi_face_weld.h, OCCT-FREE) composes B1 recognise
# → buildSeamGraph → splitFaceJunction → multiFaceCornerClip → M0 self-verify into ONE
# watertight result solid for CUT (A−B), COMMON (A∩B) and FUSE (A∪B) of a bowl-lidded
# convex-quad prism A against a corner box B whose x=0 AND y=0 faces each slice A's Bézier
# wall. The harness reconstructs the SAME A in OCCT (sewn 6-face solid) and the SAME corner
# box B as a BRepPrimAPI box, runs BRepAlgoAPI_Cut/Common/Fuse (the ORACLE), and compares
# the native result of each op (measured by the native M0 tessellator) vs OCCT on volume,
# area, watertightness, topology (Euler χ), bbox, one-sided Hausdorff and a classify batch —
# with FIXED, curved-tessellation-bounded tolerances (never widened). Gate (a) (host, no
# OCCT) is tests/native/test_native_multi_seam.cpp.
#
# The seam trace needs the substrate, so this script builds the numsci iossim archive first
# and compiles ssi/{seeding,marching}.cpp + numerics.cpp under -DCYBERCAD_HAS_NUMSCI, then
# links the OCCT oracle slice (Geom_BezierSurface/Curve + BRepBuilderAPI sewing +
# BRepAlgoAPI_Cut/Common/Fuse + BRepPrimAPI box + BRepGProp + BRepBndLib + BRepExtrema).
#
# The weld is INTERNAL — NO cc_* entry point; asserted at the cybercad::native C++ boundary.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_nurbs_solid_boolean_parity.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# ── numsci iossim substrate ──────────────────────────────────────────────────────
if [ -z "${CYBERCAD_NUMSCI_DIR:-}" ] && [ ! -d "$REPO/build-numsci/iossim/gen" ]; then
  echo "── building numsci iossim substrate (scripts/build-numsci.sh iossim)"
  "$REPO/scripts/build-numsci.sh" iossim
fi
pick_first() { for p in "$@"; do [ -e "$p" ] && { printf '%s\n' "$p"; return 0; }; done; return 0; }
NUMSCI_DIR="${CYBERCAD_NUMSCI_DIR:-}"
if [ -n "$NUMSCI_DIR" ] && [ -d "$NUMSCI_DIR/gen" ]; then
  NUMSCI_GEN="$NUMSCI_DIR/gen"; NUMSCI_LIB="$(pick_first "$NUMSCI_DIR"/libnumsci_*.a)"
else
  NUMSCI_GEN="$REPO/build-numsci/iossim/gen"
  NUMSCI_LIB="$(pick_first "$REPO"/build-numsci/iossim/libnumsci_*.a "$REPO"/build-numsci/*iossim*.a "$REPO"/eval/libnumsci_full_iossim_arm64.a)"
fi
[ -d "$NUMSCI_GEN" ] || { echo "numsci gen tree not found ($NUMSCI_GEN). Run scripts/build-numsci.sh iossim"; exit 1; }
[ -n "$NUMSCI_LIB" ] || { echo "numsci iossim archive not found. Run scripts/build-numsci.sh iossim"; exit 1; }
NUMPP="${NUMPP_DIR:-/Users/leonardoaraujo/work/NumPP}/include"
SCIPP="${SCIPP_DIR:-/Users/leonardoaraujo/work/SciPP}/include"

# ── native TUs (math + ssi/{seeding,marching} + numerics) ────────────────────────
NATIVE_SRCS=()
while IFS= read -r src; do NATIVE_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp'; \
      echo "$REPO/src/native/ssi/seeding.cpp"; \
      echo "$REPO/src/native/ssi/marching.cpp"; \
      echo "$REPO/src/native/numerics/numerics.cpp")

# ── OCCT oracle toolkits ─────────────────────────────────────────────────────────
# BRepAlgoAPI_Fuse → TKBO; BRepPrimAPI_MakeBox → TKPrim; BRepAlgoAPI (fuzzy/history) pulls
# TKBool; BRepBuilderAPI_* / BRepLib / BRepExtrema → TKTopAlgo; BRepGProp → TKGeomAlgo;
# Geom_Bezier* → TKG3d; Geom2d_* → TKG2d; gp_*/GProp/Bnd → TKMath; TKernel underneath.
# TKShHealing (ShapeAnalysis_*) is pulled in by the sewing/wire visitors. Ordering
# mirrors the proven full-suite subset: most-derived → base.
TKS="TKMesh TKShHealing TKBool TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling BOOL-INT L3 two-freeform-solid NURBS boolean orchestrator native-vs-OCCT parity harness (iphonesimulator arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} TU(s) (math + ssi/{seeding,marching} + numerics) [NUMSCI]"
echo "   numsci  : $NUMSCI_LIB"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT -DCYBERCAD_HAS_NUMSCI=1 \
  -I"$REPO/src" -I"$REPO/tests" \
  -I"$OCCT/include/opencascade" \
  -I"$NUMSCI_GEN" -I"$NUMPP" -I"$SCIPP" \
  -x objective-c++ "$HARNESS" \
  -x c++ "${NATIVE_SRCS[@]}" \
  -x none "$NUMSCI_LIB" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_nurbs_solid_boolean_parity"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_nurbs_solid_boolean_parity"
