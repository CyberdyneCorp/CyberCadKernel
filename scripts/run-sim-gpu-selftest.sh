#!/usr/bin/env bash
#
# run-sim-gpu-selftest.sh
#
# Minimal GPU smoke test: compile the Metal compute backend + its Phase-0
# dependency (the CPU backend / ComputeRegistry) + a tiny main that reports the
# Metal device name and calls cyber::metal::metal_backend_selftest(), then run it
# inside a booted iOS simulator (arm64) via `xcrun simctl spawn`.
#
# The self-test proves device init, a unified-memory buffer round-trip, runtime
# MSL compilation, dispatch, and fp32 parity against a CPU reference — all on the
# real "Apple iOS simulator GPU".
#
# This build is OCCT-FREE: it links only the Metal backend, the CPU backend, and
# the Metal/Foundation frameworks.
#
# Exit code: 0 iff the binary built, ran, and the self-test PASSED.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$REPO/build-ios-metal"; mkdir -p "$OUT"
SDK="$(xcrun --sdk iphonesimulator --show-sdk-path)"
BIN="$OUT/sim_gpu_selftest"
MAIN="$OUT/sim_gpu_selftest_main.mm"

# Tiny driver: print the Metal device name, then run the backend self-test.
cat > "$MAIN" <<'EOF'
#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include <cstdio>

#include "compute/metal/metal_backend.h"

int main() {
    @autoreleasepool {
        id<MTLDevice> dev = MTLCreateSystemDefaultDevice();
        if (dev == nil) {
            std::printf("device: <none>\n");
            std::printf("SELFTEST FAIL\n");
            return 1;
        }
        std::printf("device: %s\n", dev.name.UTF8String);

        const bool ok = cyber::metal::metal_backend_selftest();
        std::printf("SELFTEST %s\n", ok ? "PASS" : "FAIL");
        return ok ? 0 : 1;
    }
}
EOF

CXX=(xcrun --sdk iphonesimulator clang++)
FLAGS=(
  -target arm64-apple-ios16.0-simulator
  -isysroot "$SDK"
  -std=c++20 -O2 -fobjc-arc
  -DCYBERCAD_HAS_METAL=1
  -I"$REPO/src" -I"$REPO/include"
)

echo "── compiling + linking sim GPU self-test (arm64 iphonesimulator)"
"${CXX[@]}" "${FLAGS[@]}" \
  "$REPO/src/compute/metal/metal_backend.mm" \
  "$REPO/src/core/compute_backend.cpp" \
  "$MAIN" \
  -framework Metal -framework Foundation \
  -o "$BIN"

# Autodetect a booted simulator; boot one if none is running.
UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-Fa-f-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE '[0-9A-Fa-f-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi

echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$BIN"
