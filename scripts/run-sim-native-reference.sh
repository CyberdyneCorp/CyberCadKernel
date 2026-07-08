#!/usr/bin/env bash
# Compile the MOAT M-REF native-vs-OCCT reference/topology parity harness
# (tests/sim/native_reference_parity.mm) for the iOS simulator and run it in a
# booted simulator via `xcrun simctl spawn`.
#
# SIM GATE (b) of the two-gate discipline. The OCCT-FREE, header-only reference
# services (src/native/reference/reference.h) are asserted against the OCCT ORACLE
# on IDENTICAL primitives:
#   * refPlaneFromFace   vs gp_Pln (BRepAdaptor_Surface)
#   * faceAxis / refAxisFromFace vs gp_Cylinder::Axis
#   * refAxisFromEdge    vs gp_Lin (BRepAdaptor_Curve)
#   * outerRimChain      vs BRepTools::OuterWire
#   * offsetFaceBoundary vs BRepOffsetAPI_MakeOffset (inward, sharp corners)
#   * tangentChain       vs BRepAdaptor_Curve::D1 tangent oracle
# Gate (a) (host, no OCCT) is tests/native/test_native_reference.cpp.
#
# reference.h uses only the math/topology inline primitives, but its math aggregate
# references the math sources, so the math TUs are compiled in (mirrors
# run-sim-native-query.sh). No numsci link needed.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_reference_parity.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# ── native math TUs (header aggregate references them) ────────────────────────
MATH_SRCS=()
while IFS= read -r src; do MATH_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp' | sort)

# ── OCCT oracle toolkits ──────────────────────────────────────────────────────
# BRepPrimAPI_Make{Box,Cylinder} → TKPrim; BRepOffsetAPI_MakeOffset → TKOffset;
# BRepBuilderAPI_Make{Edge,Face,Polygon} → TKTopAlgo/TKBRep; BRepTools/BRepAdaptor
# → TKBRep/TKGeomBase; GProp/gp → TKMath; base TKernel. Most-derived → base order.
TKS="TKOffset TKFillet TKBool TKShHealing TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep \
     TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling M-REF native-vs-OCCT parity harness (iphonesimulator arm64)"
echo "   harness : $HARNESS"
echo "   math    : ${#MATH_SRCS[@]} source(s)"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  -x c++ "${MATH_SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_reference_parity"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_reference_parity"
