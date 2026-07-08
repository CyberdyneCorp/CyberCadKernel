#!/usr/bin/env bash
# Compile the MOAT M-GS GS2 planar SECTION-CURVES native-vs-OCCT parity harness for
# the iOS simulator, link it against an OCCT oracle slice (BRepPrimAPI primitives +
# BRepAlgoAPI_Section + ShapeAnalysis_FreeBounds wire recovery + BRepGProp linear/
# surface properties), and run it inside a booted simulator via `xcrun simctl spawn`.
#
# This is GATE (b) — the native-vs-OCCT section parity pass (native_section_parity.mm):
# per matched box/cylinder/sphere cut it asserts the native section service agrees with
# OCCT on LOOP COUNT (recovered wires), TOTAL EDGE LENGTH (LinearProperties), CLOSED-NESS,
# and CAPPED AREA (SurfaceProperties). GATE (a) (host, no OCCT, closed-form) is
# tests/native/test_native_section.cpp, built by CTest as test_native_section.
#
# Unlike the SSI S2/S3 harnesses, the native section service is the OCCT-FREE S1 header
# path (plane∩{plane,cylinder,cone,sphere} closed-form) + header-only topology — NO
# numsci substrate is needed. So this script only compiles the OCCT-free native math
# TUs (bspline/bezier) alongside the header-only section/topology/ssi-S1 headers, and
# links the OCCT oracle toolkits.
#
# Section is exercised at the cybercad::native::section C++ boundary (like the S1 SSI
# header-only harness); the cc_section_plane ABI accessor is covered by the host suite +
# the shipping-path build. On the SKIP list of run-sim-suite.sh (own main(), OCCT slice).
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_section_parity.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# ── native TUs to compile (OCCT-free math; section/topology/ssi-S1 are header-only) ──
NATIVE_SRCS=()
while IFS= read -r src; do NATIVE_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp')

# ── OCCT oracle toolkits ─────────────────────────────────────────────────────────
# BRepPrimAPI_Make{Box,Cylinder,Sphere} → TKPrim; BRepAlgoAPI_Section → TKBO (deps
# TKBool/TKTopAlgo); ShapeAnalysis_FreeBounds → TKShHealing; BRepBuilderAPI_MakeFace →
# TKTopAlgo; BRepGProp/GProp → TKGeomAlgo/TKBRep. Listed most-derived → base.
TKS="TKBO TKPrim TKShHealing TKBool TKFillet TKOffset TKTopAlgo TKGeomAlgo TKBRep \
     TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling GS2 section-curves native-vs-OCCT parity harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} TU(s) (math) + header-only section/topology/ssi-S1"
echo "   oracle  : OCCT BRepPrimAPI + BRepAlgoAPI_Section + ShapeAnalysis_FreeBounds + BRepGProp"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  -x c++ "${NATIVE_SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_section_parity"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_section_parity"
