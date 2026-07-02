#!/usr/bin/env bash
# Compile the native-tessellation vs OCCT-oracle parity harness for the iOS
# simulator, link it against the native tessellate + topology + math sources
# (src/native/**, OCCT-FREE) and the OCCT libraries used as the meshing oracle
# (TKMesh for BRepMesh_IncrementalMesh, down through TKBRep/TKPrim/TKFillet/... to
# TKernel), and run it inside a booted simulator via `xcrun simctl spawn`.
#
# This is Phase 4 capability #3 (`native-tessellation`) verification gate 2 — the
# native-vs-OCCT parity pass (see openspec/NATIVE-REWRITE.md). Gate 1 (host unit
# tests, no OCCT) is built by CTest with plain `clang++ -std=c++20`; this script
# does NOT need it.
#
# The native library is OCCT-FREE and never linked to OCCT for its own sake. The
# harness (tests/sim/native_tessellation_parity.mm) is the ONLY OCCT-dependent
# piece here: it links the oracle and contains the TEST-ONLY bridge that walks a
# real TopoDS_Shape into the native model. Nothing under src/native gains an OCCT
# dependency.
#
# The harness compares the native mesh against a TWO-SIDED oracle: OCCT
# BRepMesh_IncrementalMesh of the SAME shape at the SAME deflection (the reference
# tessellator — bbox/area/volume of its triangulation) AND the exact B-rep
# area/volume from BRepGProp. Tolerance-based, never triangle-identical.
#
# Companion to run-sim-suite.sh (the full cc_* facade suite). This harness is on
# that suite's SKIP list — it carries its own main() and links only the
# meshing-oracle slice of OCCT, not the whole kernel static lib.
#
# Model: run-sim-native-tessellate.sh / run-sim-native-topology.sh. The native
# tessellate + topology libraries are header-only; only the native math .cpp TUs
# (bspline/bezier) need compiling in.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_tessellation_parity.mm"

# Native implementation TUs. Tessellate + topology are header-only (nothing under
# src/native/tessellate or src/native/topology to compile); the math library
# carries bspline/bezier .cpp. All trees are OCCT-free — they compile with no OCCT
# include.
NATIVE_SRCS=()
while IFS= read -r src; do NATIVE_SRCS+=("$src"); done \
  < <(find "$REPO/src/native" -name '*.cpp' | sort)
[ "${#NATIVE_SRCS[@]}" -gt 0 ] || { echo "no native sources under src/native"; exit 1; }

# Oracle libraries. Beyond the topology-oracle slice, BRepMesh_IncrementalMesh and
# Poly_Triangulation live in TKMesh, so TKMesh is added at the front (most-derived).
# TopoDS/TopExp/TopTools/BRep_Tool live in TKBRep; the shape makers in TKPrim
# (box/cylinder/sphere) and TKFillet (fillet, pulling TKTopAlgo/TKGeomAlgo/TKBO/
# TKBool/TKOffset); GeomAPI_ProjectPointOnSurf + geometry adaptors in TKGeomAlgo/
# TKGeomBase/TKG3d/TKG2d; BRepGProp in TKGeomAlgo; gp_*/ElSLib in TKMath; TKernel
# underneath. Link order is most-derived → base.
#
# TKShHealing is required by both TKBO and TKFillet (ShapeUpgrade/ShapeFix); it
# sits above the geometry base toolkits, placed after the algorithm toolkits that
# reference it and before the base libs it depends on.
TKS="TKMesh TKFillet TKOffset TKBool TKBO TKTopAlgo TKShHealing TKGeomAlgo TKPrim \
     TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling native-tessellation parity harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} source(s) (tessellate+topology header-only + math TUs)"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios14.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  "${NATIVE_SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_tessellation_parity"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_tessellation_parity"
