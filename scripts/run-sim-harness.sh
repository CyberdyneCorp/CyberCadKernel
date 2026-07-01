#!/usr/bin/env bash
# Compile the OCCT runtime harness for the iOS simulator, link it against the
# CyberCadKernel simulator slice + the trimmed OCCT libs, and run it inside a
# booted simulator via `xcrun simctl spawn`. Produces real correctness /
# determinism / benchmark output from the OCCT adapter.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
LIB="$OUT/libcybercadkernel-SIMULATORARM64.a"
[ -f "$LIB" ] || { echo "build the xcframework first (scripts/build-xcframework.sh)"; exit 1; }
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

TKS="TKDESTEP TKDEIGES TKXSBase TKDE TKMesh TKShHealing TKOffset TKFillet TKBool \
     TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling + linking harness for iphonesimulator (arm64)"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios14.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 -I"$REPO/include" \
  "$REPO/tests/sim/parity_bench.cpp" "$LIB" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/parity_bench"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/parity_bench"
