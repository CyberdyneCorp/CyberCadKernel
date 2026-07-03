#!/usr/bin/env bash
# Compile the native STEP-EXPORT round-trip + writer-parity harness for the iOS
# simulator and run it inside a booted simulator via `xcrun simctl spawn`.
#
# This is Phase 4 capability #7 (`native-data-exchange`, the STEP-EXPORT slice)
# verification gate 2 — the CORRECTNESS GATE (see openspec/NATIVE-REWRITE.md #7). The
# native ISO-10303-21 STEP writer (src/native/exchange) is exercised through the
# cc_step_export facade under BOTH engines (cc_set_engine(1)=NativeEngine emits the
# native STEP; cc_set_engine(0)=OCCT emits the reference via STEPControl_Writer). The
# harness then re-reads the written .step files DIRECTLY with OCCT STEPControl_Reader
# and asserts each re-reads to the SAME solid as the source (volume/area/centroid +
# bbox + face/edge counts within tolerance), that the native-written and OCCT-written
# files re-read to EQUIVALENT solids (writer parity), and that a FOREIGN (OCCT-built)
# body exported under the native engine FALLS BACK to STEPControl_Writer (labelled,
# never faked). Gate 1 (host unit tests, no OCCT) is built by CTest as
# test_native_step_writer / test_native_engine with plain `clang++ -std=c++20`; this
# script does NOT need it.
#
# Like the sibling native_boolean_parity / native_blend_parity harnesses, this drives
# the SHIPPING PATH: the public cc_* facade under both engines. So it must link the
# WHOLE kernel — facade + core + engine (NativeEngine + the OCCT adapter, the
# fallthrough target under CYBERCAD_HAS_OCCT) + src/native/** — plus the full OCCT
# toolkit set. The harness ITSELF also links + calls OCCT directly (STEPControl_Reader,
# BRepGProp, BRepBndLib, TopExp_Explorer) to re-read + measure the written files
# independently of the kernel's own import path.
#
# We compile every src/**/*.cpp fresh here (the same set + flags as
# build-xcframework.sh's slice: -std=c++20 -DCYBERCAD_HAS_OCCT -I include -I src -I
# opencascade) rather than reuse a possibly-stale prebuilt static lib, so the harness
# always tests the current NativeEngine / native-exchange sources. No duplicate
# create_default_engine(): the stub guards its definition behind `#ifndef
# CYBERCAD_HAS_OCCT`, so with OCCT only the OCCT adapter provides it.
#
# Companion to run-sim-suite.sh (the full cc_* facade suite). This harness is on that
# suite's SKIP list — it carries its own main() and links the whole kernel + OCCT
# directly rather than the packaged static lib.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_step_parity.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# The whole kernel: facade + core + engine (NativeEngine + OCCT adapter + the always-
# compiled stub, which no-ops its create_default_engine under OCCT) + src/native/**
# (math + exchange TUs; topology/tessellate/construct/boolean/blend are header-only).
# Same file set build-xcframework.sh compiles into the slice.
KERNEL_SRCS=()
while IFS= read -r src; do KERNEL_SRCS+=("$src"); done \
  < <(find "$REPO/src" -name '*.cpp' | sort)
[ "${#KERNEL_SRCS[@]}" -gt 0 ] || { echo "no kernel sources under src"; exit 1; }

# OCCT toolkit link set. The harness exercises the OCCT adapter through the facade
# (construct/prism+revol, STEP/IGES via STEPControl_Writer, meshing, mass/bbox queries)
# AND calls OCCT directly (STEPControl_Reader, BRepGProp, BRepBndLib), so link the SAME
# broad toolkit set the sibling harnesses use (most-derived → base). TKDESTEP provides
# both STEPControl_Writer (the OCCT writer path) and STEPControl_Reader (the harness
# re-read); TKMesh/TKBRep/TKMath cover BRepGProp/BRepBndLib/TopExp.
TKS="TKDESTEP TKDEIGES TKXSBase TKDE TKMesh TKShHealing TKOffset TKFillet TKBool \
     TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling native-STEP parity harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   kernel  : ${#KERNEL_SRCS[@]} src TU(s) (facade + core + engine[native+occt] + native math/exchange)"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios14.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/include" \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  "${KERNEL_SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_step_parity"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_step_parity"
