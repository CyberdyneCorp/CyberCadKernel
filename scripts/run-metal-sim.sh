#!/usr/bin/env bash
#
# run-metal-sim.sh
#
# Compile the Metal compute backend + its Phase-0 dependency (the CPU backend /
# ComputeRegistry) + the on-simulator acceptance suite for the iOS simulator
# (arm64), then run it inside a booted simulator via `xcrun simctl spawn`.
#
# This is the acceptance bar for the metal-backend capability: it proves device
# init, unified-memory buffer round-trip, runtime MSL compilation + pipeline
# caching, dispatch parity vs a CPU reference, the fp32/fp64 precision boundary,
# and ComputeRegistry integration — all on the real "Apple iOS simulator GPU".
#
# The suite is OCCT-FREE: it links only the Metal backend, the CPU backend, and
# the Metal/Foundation frameworks. No OCCT libraries are referenced.
#
# Exit code: 0 iff every scenario passes in the simulator.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$REPO/build-ios-metal"; mkdir -p "$OUT"
SDK="$(xcrun --sdk iphonesimulator --show-sdk-path)"
BIN="$OUT/metal_selftest"

CXX=(xcrun --sdk iphonesimulator clang++)
FLAGS=(
  -target arm64-apple-ios16.0-simulator
  -isysroot "$SDK"
  -std=c++20 -O2 -fobjc-arc
  -DCYBERCAD_HAS_METAL=1
  -I"$REPO/include" -I"$REPO/src" -I"$REPO/src/core" -I"$REPO/tests/sim"
)

echo "── compiling + linking metal acceptance suite (arm64 iphonesimulator)"
"${CXX[@]}" "${FLAGS[@]}" \
  "$REPO/src/compute/metal/metal_backend.mm" \
  "$REPO/src/core/compute_backend.cpp" \
  "$REPO/tests/sim/metal_selftest.cpp" \
  -framework Metal -framework Foundation \
  -o "$BIN"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 1[0-9] \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$BIN"
