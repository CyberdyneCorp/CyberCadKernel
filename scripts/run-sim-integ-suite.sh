#!/usr/bin/env bash
#
# run-sim-integ-suite.sh
#
# Compile + run the GPU-tessellation INTEGRATION suite (tests/sim/integ_gpu_tess.cpp)
# on the iOS simulator. Unlike run-sim-suite.sh — which links the prebuilt OCCT-only
# static slice — this suite exercises the GPU path stitched into cc_tessellate, so it
# builds the kernel FROM SOURCE with BOTH the OCCT adapter (-DCYBERCAD_HAS_OCCT) and
# the Metal compute backend (-DCYBERCAD_HAS_METAL), compiling the Objective-C++ Metal
# modules under src/compute/metal/*.mm and linking -framework Metal -framework
# Foundation on top of the trimmed OCCT static libs.
#
# It compiles the whole kernel (facade + core + engine/occt adapter + compute/metal
# GPU modules) plus the single-file integration test in one invocation, then runs it
# inside a booted simulator via `xcrun simctl spawn`. Metal MSL kernels are compiled
# at RUNTIME by the backend; there is no .metallib precompile step. When the sim has
# no Metal device the surface-eval module transparently uses its fp32 CPU reference,
# so the GPU face-routing path is still exercised end to end.
#
# Companion to run-sim-suite.sh (OCCT-only, GPU off, 221/221) and run-sim-gpu-suite.sh
# (pure fp32 GPU-vs-CPU module parity, OCCT-free). Exit code: 0 iff the binary built,
# ran, and every [ITEG] assertion passed.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OCCT_INC="$OCCT/include/opencascade"
OUT="$REPO/build-ios-metal"; mkdir -p "$OUT"
BIN="$OUT/integ_gpu_tess"
SDK="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT_INC" ] || { echo "missing OCCT headers: $OCCT_INC (set OCCT_ROOT)"; exit 1; }
[ -d "$OCCT/lib" ] || { echo "missing OCCT libs: $OCCT/lib (set OCCT_ROOT)"; exit 1; }

# Whole kernel from source: every src/**.cpp (facade + core + engine incl. the OCCT
# adapter + stub) and every Metal .mm module, plus the integration test.
SRCS=()
while IFS= read -r src; do SRCS+=("$src"); done < <(find "$REPO/src" -name '*.cpp' | sort)
while IFS= read -r src; do SRCS+=("$src"); done < <(find "$REPO/src" -name '*.mm' | sort)
SRCS+=("$REPO/tests/sim/integ_gpu_tess.cpp")

# Trimmed OCCT link set (same order run-sim-suite.sh uses).
TKS="TKDESTEP TKDEIGES TKXSBase TKDE TKMesh TKShHealing TKOffset TKFillet TKBool \
     TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

CXX=(xcrun --sdk iphonesimulator clang++)
FLAGS=(
  -target arm64-apple-ios16.0-simulator
  -isysroot "$SDK"
  -std=c++20 -O2 -fobjc-arc
  -DCYBERCAD_HAS_OCCT=1 -DCYBERCAD_HAS_METAL=1
  -I"$REPO/include" -I"$REPO/src" -I"$OCCT_INC"
)

echo "── compiling + linking GPU-tessellation integration suite (arm64 iphonesimulator): ${#SRCS[@]} sources"
"${CXX[@]}" "${FLAGS[@]}" \
  "${SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS \
  -framework Metal -framework Foundation -lc++ \
  -o "$BIN"

# Autodetect a booted simulator; boot an available one if none is running.
UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-Fa-f-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE '[0-9A-Fa-f-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$BIN"
