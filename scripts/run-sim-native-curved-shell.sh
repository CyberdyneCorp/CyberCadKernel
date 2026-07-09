#!/usr/bin/env bash
# Compile the native-vs-OCCT CURVED-SHELL parity harness for the iOS simulator and run it
# inside a booted simulator via `xcrun simctl spawn`.
#
# MOAT M3 (`moat-m3cs-curved-shell`) verification gate 2 — the native-vs-OCCT parity pass.
# The native curved-shell slice (src/native/blend/curved_shell.h: hollow a capped
# cylinder / cone frustum to a uniform wall by an analytic inward offset of the curved
# wall, one cap open) is exercised through the cc_shell facade under BOTH engines
# (cc_set_engine(0)=OCCT BRepOffsetAPI_MakeThickSolid, cc_set_engine(1)=NativeEngine) and
# compared. Native intercepts the tractable capped-cylinder / cone-frustum case (wall
# volume closed-form, deflection-bounded); everything outside the domain falls back to
# OCCT (labelled, verified, never faked).
#
# Like the sibling curved-fillet harness, this drives the SHIPPING PATH: the public cc_*
# facade under both engines, linking the whole kernel + OCCT.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_curved_shell_parity.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

KERNEL_SRCS=()
while IFS= read -r src; do KERNEL_SRCS+=("$src"); done \
  < <(find "$REPO/src" -name '*.cpp' | sort)
[ "${#KERNEL_SRCS[@]}" -gt 0 ] || { echo "no kernel sources under src"; exit 1; }

# OCCT toolkit link set (MakeThickSolid reaches TKOffset; same broad set the curved-fillet
# harness links, most-derived → base).
TKS="TKDESTEP TKDEIGES TKXSBase TKDE TKMesh TKShHealing TKHLR TKOffset TKFillet TKBool \
     TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling native-curved-shell parity harness for iphonesimulator (arm64)"
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
  -o "$OUT/native_curved_shell_parity"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_curved_shell_parity"
