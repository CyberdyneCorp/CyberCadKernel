#!/usr/bin/env bash
# Compile the native-vs-OCCT SHAPE-HEALING parity harness for the iOS simulator and
# run it inside a booted simulator via `xcrun simctl spawn`.
#
# Phase 4 capability #4 (`native-healing`) verification gate 2 — the native-vs-OCCT
# parity pass (see openspec/NATIVE-REWRITE.md). The OCCT-FREE native healer
# (cybercad::native::heal::healShell, src/native/heal/*) is compared, on IDENTICAL
# deliberately-broken face soups, against the OCCT healing oracle
# (BRepBuilderAPI_Sewing → ShapeFix_Shell → ShapeFix_Solid, in
# src/engine/occt/occt_shapefix.cpp). Healing is INTERNAL (no cc_* entry point), so
# parity is asserted at the C++/heal boundary — like run-sim-native-topology.sh.
#
# Gate 1 (host unit tests, no OCCT) is built by CTest as test_native_heal with plain
# clang++ -std=c++20; this script does NOT need it.
#
# The harness links ONLY: the OCCT-free native TUs (heal is header-only + heal.cpp;
# math carries bspline/bezier .cpp; topology/tessellate are header-only) plus the
# single OCCT oracle TU (occt_shapefix.cpp — self-contained: it uses only
# BRepBuilderAPI_Sewing / ShapeFix / BRepCheck_Analyzer / BRepGProp and depends on no
# other OCCT adapter TU). No cc_* facade, no OcctEngine spine.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_heal_parity.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# OCCT-free native TUs (heal.cpp + math bspline/bezier) + the OCCT oracle TU.
NATIVE_SRCS=()
while IFS= read -r src; do NATIVE_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/heal" "$REPO/src/native/math" -name '*.cpp' | sort)
ORACLE_SRC="$REPO/src/engine/occt/occt_shapefix.cpp"
[ -f "$ORACLE_SRC" ] || { echo "missing OCCT oracle TU: $ORACLE_SRC"; exit 1; }

# Oracle libraries: BRepBuilderAPI_Sewing / BRepCheck_Analyzer live in TKTopAlgo;
# ShapeFix_Shell / ShapeFix_Solid / ShapeAnalysis_Shell in TKShHealing; BRepGProp /
# GProp in TKGeomAlgo; BRep in TKBRep; geometry base in TKGeomBase/TKG3d/TKG2d; gp_*
# in TKMath; TKernel underneath. Most-derived → base.
TKS="TKShHealing TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling native-heal parity harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} OCCT-free source(s) + 1 OCCT oracle TU"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios14.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/include" \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  "${NATIVE_SRCS[@]}" "$ORACLE_SRC" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_heal_parity"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_heal_parity"
