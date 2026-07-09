#!/usr/bin/env bash
# Compile the MOAT M2 curved-wall freeform half-space CUT/COMMON
# native-vs-OCCT parity harness for the iOS simulator and run it in a booted simulator
# via `xcrun simctl spawn`.
#
# This is SIM GATE (b) of the two-gate B4 model. The native cut
# (src/native/boolean/half_space_cut.h, OCCT-FREE) composes B1 recognise → M1 seam trace
# → B2 face split → B4 analytic-face split + cross-section cap weld → B3 confirm → M0
# self-verify into ONE watertight keep-side solid (a bowl-lidded convex-quad prism cut by
# the plane x=0, keep x≤0). The harness reconstructs the SAME operand in OCCT (sewn
# 6-face solid) and cuts it with BRepAlgoAPI_Cut against a large box, then compares the
# native result (measured by the native M0 tessellator) vs OCCT on volume, area,
# watertightness, topology (Euler χ), bbox, and one-sided Hausdorff — with FIXED,
# curved-tessellation-bounded tolerances (never widened). Gate (a) (host, no OCCT) is
# tests/native/test_native_first_freeform_boolean.cpp.
#
# Like the B2 face-split harness, the seam trace needs the substrate, so this script
# builds the numsci iossim archive first and compiles ssi/{seeding,marching}.cpp +
# numerics.cpp under -DCYBERCAD_HAS_NUMSCI, then links the OCCT oracle slice
# (Geom_BezierSurface/Curve + BRepBuilderAPI sewing + BRepAlgoAPI_Cut + BRepPrimAPI box +
# BRepGProp + BRepBndLib + BRepExtrema).
#
# B4 is INTERNAL — NO cc_* entry point; asserted at the cybercad::native C++ boundary.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_curved_wall_cut_parity.mm"
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
  NUMSCI_LIB="$(pick_first "$REPO"/build-numsci/*iossim*.a "$REPO"/eval/libnumsci_full_iossim_arm64.a)"
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
# BRepAlgoAPI_Cut → TKBO; BRepPrimAPI_MakeBox → TKPrim; BRepAlgoAPI (fuzzy/history) pulls
# TKBool; BRepBuilderAPI_* / BRepLib / BRepExtrema → TKTopAlgo; BRepGProp → TKGeomAlgo;
# Geom_Bezier* → TKG3d; Geom2d_* → TKG2d; gp_*/GProp/Bnd → TKMath; TKernel underneath.
# TKShHealing (ShapeAnalysis_*) is pulled in by the sewing/wire visitors. Ordering
# mirrors the proven full-suite subset: most-derived → base.
TKS="TKMesh TKShHealing TKBool TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling B4 first freeform boolean native-vs-OCCT parity harness (iphonesimulator arm64)"
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
  -o "$OUT/native_curved_wall_cut_parity"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_curved_wall_cut_parity"
