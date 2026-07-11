#!/usr/bin/env bash
# Compile + run the MINIMAL near-tangent / near-coincident DENSE-CSG sim leg
# (tests/sim/native_dense_csg_tangent_sim.mm) for the iOS simulator. This is the SHORT
# sim complement to the primary HOST closed-form battery
# (tests/native/test_native_dense_csg_stress.cpp): it cross-checks a HANDFUL of mapped
# near-tangent dense-soup fuses against a real OCCT BRepAlgoAPI_Fuse and asserts the same
# invariant — DISAGREED == 0 (native never presents a volume outside tol of OCCT as valid).
#
# Deliberately TINY (a fixed 3 cases, no fuzz batch) because the simulator is shared. The
# native planar boolean is header-only and does NOT need the numsci/SSI substrate, so this
# links only native math + boolean/ssi_boolean.cpp (compiled as its no-numsci stub) + OCCT —
# a lighter slice than run-sim-native-boolean-fuzz.sh. src/native stays OCCT-FREE; this
# harness is additive test/sim code only. On run-sim-suite.sh's SKIP list (own main(),
# OCCT slice, std::_Exit).
#
# Usage: scripts/run-sim-native-dense-csg-tangent.sh
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_dense_csg_tangent_sim.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# native TUs: all math + boolean/ssi_boolean.cpp (compiles as a NULL-returning stub with
# no numsci). The planar BSP boolean itself is header-only.
NATIVE_SRCS=()
while IFS= read -r src; do NATIVE_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp'; \
      echo "$REPO/src/native/boolean/ssi_boolean.cpp")

# OCCT oracle toolkits (BRepAlgoAPI_Fuse + BRepPrimAPI + BRepGProp + BRepCheck + Transform).
TKS="TKBool TKBO TKPrim TKShHealing TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling near-tangent dense-CSG sim leg for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} TU(s) (math + boolean/ssi_boolean stub) [NO numsci]"
echo "   oracle  : OCCT BRepAlgoAPI_Fuse + BRepPrimAPI + BRepGProp + BRepCheck"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  -x c++ "${NATIVE_SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_dense_csg_tangent_sim"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_dense_csg_tangent_sim"
