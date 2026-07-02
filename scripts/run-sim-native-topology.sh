#!/usr/bin/env bash
# Compile the native-topology vs OCCT-oracle parity harness for the iOS
# simulator, link it against the native topology + math sources (src/native/**,
# OCCT-FREE) and the OCCT libraries used as the topology oracle (TKBRep/TKPrim/
# TKFillet/... down to TKernel), and run it inside a booted simulator via
# `xcrun simctl spawn`.
#
# This is Phase 4 capability #2 (`native-topology`) verification gate 2 — the
# native-vs-OCCT parity pass (see openspec/NATIVE-REWRITE.md). Gate 1 (host unit
# tests, no OCCT) is built by CTest with plain `clang++ -std=c++20`; this script
# does NOT need it.
#
# The native library is OCCT-FREE and never linked to OCCT for its own sake. The
# harness (tests/sim/native_topology_parity.mm) is the ONLY OCCT-dependent piece
# here: it links the oracle and contains the test-only bridge that walks a real
# TopoDS_Shape into the native model. Nothing under src/native gains an OCCT
# dependency.
#
# Companion to run-sim-suite.sh (the full cc_* facade suite). This harness is on
# that suite's SKIP list — it carries its own main() and links only the
# topology-oracle slice of OCCT, not the whole kernel static lib.
#
# Model: run-sim-native-math.sh. The native topology library is header-only;
# only the native math .cpp TUs (bspline/bezier) need compiling in.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_topology_parity.mm"

# Native implementation TUs. Topology is header-only (nothing under
# src/native/topology to compile); the math library carries bspline/bezier .cpp.
# Both trees are OCCT-free — they compile with no OCCT include.
NATIVE_SRCS=()
while IFS= read -r src; do NATIVE_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/topology" "$REPO/src/native/math" -name '*.cpp' | sort)
[ "${#NATIVE_SRCS[@]}" -gt 0 ] || { echo "no native sources under src/native"; exit 1; }

# Oracle libraries: TopoDS/TopExp/TopTools/BRep_Tool live in TKBRep; the shape
# makers in TKPrim (box/cylinder) and TKFillet (fillet, pulling TKTopAlgo/
# TKGeomAlgo/TKBO/TKBool/TKOffset); geometry adaptors in TKGeomBase/TKG3d/TKG2d;
# gp_*/ElSLib in TKMath; TKernel underneath. Link order is most-derived → base.
#
# TKShHealing is required by both TKBO (BRepAlgoAPI_BuilderAlgo::SimplifyResult
# calls ShapeUpgrade_UnifySameDomain) and TKFillet (ChFi3d_Builder::Compute calls
# ShapeFix::SameParameter). It sits above the geometry base toolkits, so it is
# placed after the algorithm toolkits that reference it and before the base libs
# it depends on (TKGeomAlgo/TKTopAlgo/TKBRep/TKGeomBase/TKG3d/TKG2d/TKMath/
# TKernel).
TKS="TKFillet TKOffset TKBool TKBO TKTopAlgo TKShHealing TKGeomAlgo TKPrim TKBRep \
     TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling native-topology parity harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} source(s) (topology header-only + math TUs)"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios14.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  "${NATIVE_SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_topology_parity"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_topology_parity"
