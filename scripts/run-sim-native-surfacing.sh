#!/usr/bin/env bash
# Compile the MOAT SURFACE bounded N-sided fill native-vs-OCCT parity harness for the iOS
# simulator and run it in a booted simulator via `xcrun simctl spawn`.
#
# This is SIM GATE (b) of the two-gate model. The native fill (src/native/surface/,
# OCCT-FREE) evaluates a Coons/Gregory transfinite interpolant of a 3–6-sided ANALYTIC
# boundary loop to a TESSELLATED triangle mesh patch (NOT a NURBS surface — the campaign's
# scope bound). The harness fills the SAME boundary with the OCCT ORACLE BRepFill_Filling
# and compares the native patch (measured by the native tessellator's surfaceArea + a
# vertex bbox) vs OCCT (BRepGProp area + BRepBndLib bbox + BRepExtrema boundary-coincidence)
# with FIXED tolerances (exact for planar, deflection-bounded for curved — never widened).
# Gate (a) (host, no OCCT) is tests/native/test_native_surfacing.cpp.
#
# The surface module is header-only over math + tessellate + boolean + topology and does
# NOT touch the numsci seam tracer, so — unlike the draft/blend harnesses — NO numsci
# archive is needed: it compiles the native math TUs and links the OCCT fill slice only
# (BRepFill_Filling → TKBool/TKBO/TKShHealing + GC_Make* → TKGeomAlgo, box/extrema →
# TKTopAlgo, BRepGProp → TKGeomAlgo, Bnd → TKMath).
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_surfacing_parity.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# ── native TUs (math only — surface/tessellate/boolean/topology are header-only) ──
NATIVE_SRCS=()
while IFS= read -r src; do NATIVE_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp')

# ── OCCT oracle toolkits ─────────────────────────────────────────────────────────
# BRepFill_Filling → TKBool/TKBO (+ TKShHealing/TKOffset it pulls); GC_Make* → TKGeomAlgo;
# BRepBuilderAPI_* / BRepExtrema → TKTopAlgo; BRepGProp → TKGeomAlgo; Bnd/gp → TKMath;
# TKernel underneath. Ordering: most-derived → base.
TKS="TKMesh TKShHealing TKOffset TKFillet TKBool TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling N-sided-fill native-vs-OCCT parity harness (iphonesimulator arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} math TU(s) (surface/tessellate/boolean are header-only)"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/src" -I"$REPO/tests" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  -x c++ "${NATIVE_SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_surfacing_parity"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_surfacing_parity"
