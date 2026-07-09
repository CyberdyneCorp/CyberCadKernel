#!/usr/bin/env bash
# Compile the native-vs-OCCT CURVED-FILLET parity harness for the iOS simulator and run it
# inside a booted simulator via `xcrun simctl spawn`.
#
# This is Phase 4 capability #6 (`native-blends` / native fillets-offsets) verification
# gate 2 — the native-vs-OCCT parity pass (see openspec/NATIVE-REWRITE.md). The native
# planar blend slice (src/native/blend: chamfer / constant-radius planar-dihedral
# fillet / planar offset-face / uniform shell) is exercised through the cc_fillet_edges
# / cc_chamfer_edges / cc_offset_face / cc_shell facade under BOTH engines
# (cc_set_engine(0)=OCCT, cc_set_engine(1)=NativeEngine) and compared. Native intercepts
# the tractable planar cases (chamfer / offset / shell EXACT, constant-radius fillet
# DEFLECTION-BOUNDED), and everything outside the domain (curved edges, variable radius,
# fillet_face, self-verify failures) FALLS BACK to OCCT (labelled, verified, never
# faked). Gate 1 (host unit tests, no OCCT) is built by CTest as test_native_engine with
# plain `clang++ -std=c++20`; this script does NOT need it.
#
# Like the sibling native_boolean_parity harness, this drives the SHIPPING PATH: the
# public cc_* facade under both engines. So it must link the WHOLE kernel — facade +
# core + engine (NativeEngine + the OCCT adapter, which is the fallthrough target under
# CYBERCAD_HAS_OCCT) + src/native/** — plus the full OCCT toolkit set (the OCCT adapter
# reaches BRepFilletAPI / BRepOffsetAPI, meshing, mass/bbox queries, etc.).
#
# We compile every src/**/*.cpp fresh here (the same set + flags as
# build-xcframework.sh's slice: -std=c++20 -DCYBERCAD_HAS_OCCT -I include -I src -I
# opencascade) rather than reuse a possibly-stale prebuilt static lib, so the harness
# always tests the current NativeEngine / native-blend sources. No duplicate
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

HARNESS="$REPO/tests/sim/native_curved_fillet_parity.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# The whole kernel: facade + core + engine (NativeEngine + OCCT adapter + the always-
# compiled stub, which no-ops its create_default_engine under OCCT) + src/native/**
# (math TUs; topology/tessellate/construct/boolean/blend are header-only). Same file set
# build-xcframework.sh compiles into the slice.
KERNEL_SRCS=()
while IFS= read -r src; do KERNEL_SRCS+=("$src"); done \
  < <(find "$REPO/src" -name '*.cpp' | sort)
[ "${#KERNEL_SRCS[@]}" -gt 0 ] || { echo "no kernel sources under src"; exit 1; }

# OCCT toolkit link set. The harness exercises the OCCT adapter through the facade
# (construct/prism+revol, FILLETS/CHAMFERS/OFFSETS (BRepFilletAPI/BRepOffsetAPI), meshing,
# mass/bbox queries, and — since occt_exchange.cpp is compiled in — STEP/IGES), so link
# the SAME broad toolkit set run-sim-native-boolean.sh uses (most-derived → base).
TKS="TKDESTEP TKDEIGES TKXSBase TKDE TKMesh TKShHealing TKHLR TKOffset TKFillet TKBool \
     TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling native-curved-fillet parity harness for iphonesimulator (arm64)"
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
  -o "$OUT/native_curved_fillet_parity"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_curved_fillet_parity"
