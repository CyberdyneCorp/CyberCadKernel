#!/usr/bin/env bash
# Compile the native-vs-OCCT GEOMETRY-COMPLETION parity harness (Tier 1 +
# Tier 2#4, the four residual construction areas — spline extrude, torus revolve,
# ruled loft, smooth/twisted sweep) for the iOS simulator and run it inside a booted
# simulator via `xcrun simctl spawn`.
#
# This is the completion-batch sibling of run-sim-native-sweep.sh / run-sim-native-loft.sh.
# It is Phase 4 #4b residual verification gate 2 — the native-vs-OCCT parity pass
# (see openspec/NATIVE-REWRITE.md) for the newly-native completion builders
# (construct/residuals.h spline extrude + torus revolve, loft.h ruled loft, sweep.h
# smooth/twisted sweep) AND their honest OCCT fall-throughs (self-crossing spline,
# spindle torus, mismatched/curved-rail loft, self-intersecting sweep/thread). Gate 1
# (host unit tests, no OCCT) is built by CTest with plain `clang++ -std=c++20`; this
# script does NOT need it.
#
# Like run-sim-native-sweep.sh, this harness drives the SHIPPING PATH: it calls the
# public cc_* facade under BOTH engines (cc_set_engine(0)=OCCT, cc_set_engine(1)=
# NativeEngine) and compares. So it must link the WHOLE kernel — facade + core +
# engine (NativeEngine + the OCCT adapter, which is the fallthrough target under
# CYBERCAD_HAS_OCCT) + src/native/** — plus the full OCCT toolkit set (the OCCT
# adapter reaches booleans, meshing, STEP/IGES, pipe-shell, etc.).
#
# We compile every src/**/*.cpp fresh here (the same set + flags as
# build-xcframework.sh's slice: -std=c++20 -DCYBERCAD_HAS_OCCT -I include -I src -I
# opencascade) rather than reuse a possibly-stale prebuilt static lib, so the harness
# always tests the current residuals.h / loft.h / sweep.h / thread.h / NativeEngine /
# cc_set_engine sources. No duplicate create_default_engine(): the stub guards its
# definition behind `#ifndef CYBERCAD_HAS_OCCT`, so with OCCT only the OCCT adapter
# provides it.
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

HARNESS="$REPO/tests/sim/native_geomcompletion_parity.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# The whole kernel: facade + core + engine (NativeEngine + OCCT adapter + the
# always-compiled stub, which no-ops its create_default_engine under OCCT) +
# src/native/** (math TUs; topology/tessellate/construct/blend/boolean/exchange are
# header-only). Same file set build-xcframework.sh compiles into the slice.
KERNEL_SRCS=()
while IFS= read -r src; do KERNEL_SRCS+=("$src"); done \
  < <(find "$REPO/src" -name '*.cpp' | sort)
[ "${#KERNEL_SRCS[@]}" -gt 0 ] || { echo "no kernel sources under src"; exit 1; }

# OCCT toolkit link set. The harness exercises the full OCCT adapter through the
# facade (construct/prism+revol+pipe+pipe-shell+thrusections, booleans, meshing,
# mass/bbox queries, and — since occt_exchange.cpp is compiled in — STEP/IGES), so
# link the SAME broad toolkit set run-sim-suite.sh / run-sim-native-sweep.sh use
# (most-derived → base).
TKS="TKDESTEP TKDEIGES TKXSBase TKDE TKMesh TKShHealing TKOffset TKFillet TKBool \
     TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling native-geomcompletion parity harness for iphonesimulator (arm64)"
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
  -o "$OUT/native_geomcompletion_parity"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_geomcompletion_parity"
