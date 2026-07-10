#!/usr/bin/env bash
# Compile the MOAT M-GS GS7 native-vs-OCCT CLASH / INTERFERENCE parity harness
# (tests/sim/native_interference_parity.mm) for the iOS simulator and run it in a
# booted simulator via `xcrun simctl spawn`.
#
# This is SIM GATE (b) of the two-gate discipline. The native OCCT-FREE, header-only
# classifier (src/native/analysis/interference.h, on the landed B3 membership + M0
# mesh vocabulary) is asserted against the OCCT ORACLE on IDENTICAL box solids:
#   * CLASH / TOUCHING / CLEAR state  vs BRepExtrema_DistShapeShape + BRepAlgoAPI_Common
#   * overlap volume (clash cases)    vs BRepAlgoAPI_Common + BRepGProp (closed form)
# Gate (a) (host, no OCCT) is tests/native/test_native_interference.cpp.
#
# interference.h uses only the math/vec + mesh inline primitives + the B3 classifier
# (all header-only), but the aggregate math header references the math sources, so
# the math TUs are compiled in (mirrors run-sim-native-query.sh). No numsci link.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_interference_parity.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# ── native math TUs (header aggregate references them) ────────────────────────
MATH_SRCS=()
while IFS= read -r src; do MATH_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp' | sort)

# ── OCCT oracle toolkits ──────────────────────────────────────────────────────
# BRepPrimAPI_MakeBox → TKPrim; BRepAlgoAPI_Common (BOPAlgo) → TKBO, which pulls in
# ShapeUpgrade_UnifySameDomain (result simplification) → TKShHealing; BRepExtrema_Dist
# ShapeShape → TKBRep; BRepGProp → TKGeomAlgo; TopoDS/BRep → TKBRep; GProp/gp → TKMath;
# base TKernel. Most-derived → base link order.
TKS="TKBO TKShHealing TKPrim TKTopAlgo TKBRep TKGeomAlgo TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling M-GS GS7 native-vs-OCCT interference parity harness (iphonesimulator arm64)"
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
  -o "$OUT/native_interference_parity"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_interference_parity"
