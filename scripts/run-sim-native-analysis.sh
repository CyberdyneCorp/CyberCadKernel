#!/usr/bin/env bash
# Compile the MOAT M-GS GS3/GS4 native-vs-OCCT parity harness
# (tests/sim/native_analysis_parity.mm) for the iOS simulator and run it in a
# booted simulator via `xcrun simctl spawn`.
#
# This is SIM GATE (b) of the two-gate discipline. The native ANALYSIS services
# (src/native/analysis/*.h, OCCT-FREE, header-only, on the landed native NURBS /
# topology evaluators) are asserted against the OCCT ORACLE on IDENTICAL geometry:
#   * GS4 surface curvature  vs GeomLProp_SLProps (Gaussian/mean/principal)
#   * GS4 edge curvature      vs GeomLProp_CLProps (Curvature)
#   * GS3 minimum distance    vs BRepExtrema_DistShapeShape (Value)
#   * GS3 angle               vs an independent gp_Dir closed form
# Gate (a) (host, no OCCT) is tests/native/test_native_analysis.cpp.
#
# The freeform-distance seed-and-refine minimizer rides numerics/closest_point, so
# this builds the numsci iossim archive first and compiles numerics.cpp +
# math/*.cpp under -DCYBERCAD_HAS_NUMSCI, then links the OCCT oracle slice
# (Geom_* elementary/BSpline/Bezier + GeomLProp + BRepBuilderAPI + BRepExtrema).
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"
NUMPP="${NUMPP_DIR:-/Users/leonardoaraujo/work/NumPP}"
SCIPP="${SCIPP_DIR:-/Users/leonardoaraujo/work/SciPP}"
NUMSCI_OUT="${NUMSCI_DIR:-$REPO/build-numsci/iossim}"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }
[ -d "$NUMPP/include/numpp" ] || { echo "NumPP not found at $NUMPP"; exit 1; }
[ -d "$SCIPP/include/scipp" ] || { echo "SciPP not found at $SCIPP"; exit 1; }

HARNESS="$REPO/tests/sim/native_analysis_parity.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# ── numsci iossim substrate ──────────────────────────────────────────────────
NUMSCI_LIB="$NUMSCI_OUT/libnumsci_iossim_arm64.a"
if [ ! -f "$NUMSCI_LIB" ] || [ -n "${NUMSCI_REBUILD:-}" ]; then
  echo "── building NumPP/SciPP substrate (iossim) via scripts/build-numsci.sh"
  "$REPO/scripts/build-numsci.sh" iossim
fi
[ -f "$NUMSCI_LIB" ] || { echo "numsci archive not built: $NUMSCI_LIB"; exit 1; }
NUMSCI_GEN="$NUMSCI_OUT/gen"
[ -d "$NUMSCI_GEN/numpp" ] || { echo "numsci gen headers missing: $NUMSCI_GEN"; exit 1; }

# ── native TUs (numeric facade + OCCT-free math implementation) ───────────────
NUMERICS_SRC="$REPO/src/native/numerics/numerics.cpp"
[ -f "$NUMERICS_SRC" ] || { echo "missing $NUMERICS_SRC"; exit 1; }
MATH_SRCS=()
while IFS= read -r src; do MATH_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp' | sort)
[ "${#MATH_SRCS[@]}" -gt 0 ] || { echo "no native math sources under src/native/math"; exit 1; }

# ── OCCT oracle toolkits ──────────────────────────────────────────────────────
# BRepExtrema_DistShapeShape / BRepBuilderAPI_* → TKTopAlgo, TKBRep; GeomLProp →
# TKGeomBase; Geom_* (elementary + BSpline + Bezier) → TKG3d; Geom2d → TKG2d;
# gp_*/GProp/ElSLib → TKMath; base TKernel. Most-derived → base link order.
TKS="TKTopAlgo TKBRep TKGeomAlgo TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling M-GS analysis native-vs-OCCT parity harness (iphonesimulator arm64)"
echo "   harness  : $HARNESS"
echo "   numerics : $NUMERICS_SRC (-DCYBERCAD_HAS_NUMSCI)"
echo "   math     : ${#MATH_SRCS[@]} source(s)"
echo "   substrate: $NUMSCI_LIB"
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
  -o "$OUT/native_analysis_parity"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_analysis_parity"
