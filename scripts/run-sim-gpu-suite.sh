#!/usr/bin/env bash
#
# run-sim-gpu-suite.sh
#
# Compile the integrated Phase-2 GPU-vs-CPU parity suite (tests/sim/gpu_suite.mm +
# tests/sim/gpu_pick_check.mm), the four Metal GPU modules under
# src/compute/metal/*.mm, and their sole Phase-0 core dependency
# (src/core/compute_backend.cpp) for the iOS simulator (arm64), then run it inside
# a booted simulator via `xcrun simctl spawn`.
#
# Each GPU module (surface-eval, BVH + closestHit, picking, mesh-post normals) is
# dispatched on the real "Apple iOS simulator GPU" and its result asserted against
# an independent CPU reference within an fp32 tolerance. MSL kernels are compiled
# at RUNTIME (newLibraryWithSource) — there is no .metallib precompile step.
#
# The suite is OCCT-FREE and fp32-only: it links only the Metal GPU modules, the
# CPU backend / ComputeRegistry, and the Metal/Foundation frameworks. No OCCT
# libraries are referenced, and no fp64 modeling work is routed to the GPU.
#
# The picking checks live in a separate TU (gpu_pick_check.mm) because gpu_pick.h
# and gpu_surface_eval.h each define a cyber::metal::Vec3f and cannot share a
# translation unit; both TUs report through tests/sim/gpu_check.h.
#
# Exit code: 0 iff the binary built, ran, and every GPU-vs-CPU check passed.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$REPO/build-ios-metal"; mkdir -p "$OUT"
SDK="$(xcrun --sdk iphonesimulator --show-sdk-path)"
BIN="$OUT/gpu_suite"

CXX=(xcrun --sdk iphonesimulator clang++)
FLAGS=(
  -target arm64-apple-ios16.0-simulator
  -isysroot "$SDK"
  -std=c++20 -O2 -fobjc-arc
  -DCYBERCAD_HAS_METAL=1
  -I"$REPO/src" -I"$REPO/include" -I"$REPO/tests/sim"
)

echo "── compiling + linking Phase-2 GPU parity suite (arm64 iphonesimulator)"
"${CXX[@]}" "${FLAGS[@]}" \
  "$REPO/tests/sim/gpu_suite.mm" \
  "$REPO/tests/sim/gpu_pick_check.mm" \
  "$REPO/src/compute/metal/gpu_surface_eval.mm" \
  "$REPO/src/compute/metal/gpu_bvh.mm" \
  "$REPO/src/compute/metal/gpu_pick.mm" \
  "$REPO/src/compute/metal/gpu_mesh_post.mm" \
  "$REPO/src/compute/metal/metal_backend.mm" \
  "$REPO/src/core/compute_backend.cpp" \
  -framework Metal -framework Foundation \
  -o "$BIN"

# Autodetect a booted simulator; boot an available one if none is running.
UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-Fa-f-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE '[0-9A-Fa-f-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$BIN"
