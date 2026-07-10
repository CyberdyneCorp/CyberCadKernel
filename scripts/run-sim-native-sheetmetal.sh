#!/usr/bin/env bash
# Compile the MOAT M-SM sheet-metal first-slice SELF-TEST for the iOS simulator and run
# it inside a booted simulator via `xcrun simctl spawn`.
#
# This is SIM GATE (b) of the two-gate model. UNLIKE the other native parity harnesses,
# sheet metal has NO OCCT ORACLE — OCCT core has no sheet-metal module — so this harness
# does NOT compare against OCCT. It drives the SHIPPING cc_* facade under the NATIVE
# engine (cc_set_engine(1)) and verifies the built parts pass cc_check_solid + that their
# cc_mass_properties volume matches the CLOSED FORM, plus determinism and the unfold area
# invariant (see tests/sim/native_sheetmetal_selftest.mm). Gate (a) (host, no OCCT) is
# tests/native/test_native_sheetmetal.cpp (CTest: test_native_sheetmetal).
#
# The sheet-metal ops (src/native/sheetmetal/**) are OCCT-FREE and do NOT need the NUMSCI
# substrate. We still compile the WHOLE kernel (facade + core + engine[native+occt adapter]
# + src/native/**) under -DCYBERCAD_HAS_OCCT and link the OCCT toolkit, because the facade's
# create_default_engine is provided by the OCCT adapter under that define — exactly like
# run-sim-native-abi.sh. The harness itself never enters an OCCT path.
#
# On run-sim-suite.sh's SKIP list (own main()).
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_sheetmetal_selftest.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

KERNEL_SRCS=()
while IFS= read -r src; do KERNEL_SRCS+=("$src"); done \
  < <(find "$REPO/src" -name '*.cpp' | sort)
[ "${#KERNEL_SRCS[@]}" -gt 0 ] || { echo "no kernel sources under src"; exit 1; }

# OCCT toolkit link set (most-derived → base). Includes TKHLR/TKShHealing so the OCCT
# adapter TUs (compiled into the kernel) resolve even though the harness never calls them.
TKS="TKDESTEP TKDEIGES TKXSBase TKDE TKMesh TKHLR TKShHealing TKOffset TKFillet TKBool \
     TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling sheet-metal self-test for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   kernel  : ${#KERNEL_SRCS[@]} src TU(s) (facade + core + engine[native+occt] + native math)"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios14.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/include" \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  "${KERNEL_SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_sheetmetal_selftest"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_sheetmetal_selftest"
