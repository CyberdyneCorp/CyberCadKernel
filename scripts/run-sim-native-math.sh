#!/usr/bin/env bash
# Compile the native-math vs OCCT-oracle parity harness for the iOS simulator,
# link it against the native math sources (src/native/math/*.cpp — OCCT-FREE) and
# the minimal OCCT libraries used as the numeric oracle (TKMath/TKG3d/TKGeomBase/
# TKG2d/TKernel), and run it inside a booted simulator via `xcrun simctl spawn`.
#
# This is Phase 4 capability #1 (`native-math`) verification gate 2 — the
# native-vs-OCCT parity pass (see openspec/NATIVE-REWRITE.md). Gate 1 (host unit
# tests, no OCCT) is tests/test_native_math.cpp, built by CTest with plain
# `clang++ -std=c++20` — this script does NOT need it.
#
# Companion to run-sim-suite.sh (the full cc_* facade suite). This harness is on
# that suite's SKIP list because it carries its own main() and links only the
# geometry-oracle slice of OCCT, not the whole kernel static lib.
#
# Model: run-sim-suite.sh. No dependency on the CyberCadKernel static lib — the
# native math library is header + a couple of .cpp files, compiled straight in.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_math_parity.mm"
# Native math implementation TUs (headers are in the same dir; the rest is
# header-only). Compiled OCCT-free — they carry no OCCT include.
NATIVE_SRCS=()
while IFS= read -r src; do NATIVE_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp' | sort)
[ "${#NATIVE_SRCS[@]}" -gt 0 ] || { echo "no native math sources under src/native/math"; exit 1; }

# Oracle libraries: gp_*/BSplCLib/BSplSLib/ElSLib live in TKMath; Geom_BSpline*
# in TKG3d (with TKGeomBase/TKG2d dependencies); TKernel underneath. Link order
# is most-derived → base for the static archives.
TKS="TKG3d TKGeomBase TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling native-math parity harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} source(s)"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios14.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  "${NATIVE_SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_math_parity"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_math_parity"
