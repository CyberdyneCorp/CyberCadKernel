#!/usr/bin/env bash
# Compile the FULL OCCT runtime suite (full_suite.cpp + every checks_*.cpp module,
# excluding parity_bench.cpp) for the iOS simulator, link it against the
# CyberCadKernel simulator slice + the trimmed OCCT libs, and run it inside a
# booted simulator via `xcrun simctl spawn`. Exercises all 57 cc_* entry points
# plus the determinism/benchmark pass. Companion to run-sim-harness.sh, which
# runs the single-file parity_bench harness.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
LIB="$OUT/libcybercadkernel-SIMULATORARM64.a"
[ -f "$LIB" ] || { echo "build the xcframework first (scripts/build-xcframework.sh)"; exit 1; }
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

# Every sim source EXCEPT parity_bench.cpp (its own single-file harness has a
# competing main()). Includes full_suite.cpp + all checks_*.cpp modules.
SRCS=()
while IFS= read -r src; do
  [ "$(basename "$src")" = "parity_bench.cpp" ] && continue
  SRCS+=("$src")
done < <(find "$REPO/tests/sim" -name '*.cpp' | sort)
[ "${#SRCS[@]}" -gt 0 ] || { echo "no suite sources found under tests/sim"; exit 1; }

TKS="TKDESTEP TKDEIGES TKXSBase TKDE TKMesh TKShHealing TKOffset TKFillet TKBool \
     TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling + linking full suite for iphonesimulator (arm64): ${#SRCS[@]} sources"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios14.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 -I"$REPO/include" -I"$REPO/tests/sim" \
  "${SRCS[@]}" "$LIB" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/full_suite"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/full_suite"
