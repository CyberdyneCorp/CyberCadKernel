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

# ── numsci iossim substrate ───────────────────────────────────────────────────────
# The canal-fillet case drives cc_boolean(cylZ, cylX, common) to build a native Steinmetz
# bicylinder body (moat-m2xc-cyl-cyl-common-facade). That body comes from the native SSI
# curved-boolean path (src/native/boolean/ssi_boolean.cpp), whose body is behind
# CYBERCAD_HAS_NUMSCI and consumes the S3 tracer's least_squares corrector + lstsq fit. So
# the WHOLE kernel must be compiled with -DCYBERCAD_HAS_NUMSCI=1 and linked against the
# NumPP/SciPP numsci iossim archive — otherwise ssi_boolean_solid is a decline-stub, the
# native body never builds, and the canal case only reaches the honest native-note branch.
if [ -z "${CYBERCAD_NUMSCI_DIR:-}" ] && [ ! -d "$REPO/build-numsci/iossim/gen" ]; then
  echo "── building numsci iossim substrate (scripts/build-numsci.sh iossim)"
  "$REPO/scripts/build-numsci.sh" iossim
fi
# pick_first <glob...> — first existing path (no `ls | head`, whose SIGPIPE under
# pipefail+set -e would abort the script).
pick_first() { for p in "$@"; do [ -e "$p" ] && { printf '%s\n' "$p"; return 0; }; done; return 0; }
NUMSCI_DIR="${CYBERCAD_NUMSCI_DIR:-}"
if [ -n "$NUMSCI_DIR" ] && [ -d "$NUMSCI_DIR/gen" ]; then
  NUMSCI_GEN="$NUMSCI_DIR/gen"
  NUMSCI_LIB="$(pick_first "$NUMSCI_DIR"/libnumsci_*.a)"
else
  NUMSCI_GEN="$REPO/build-numsci/iossim/gen"
  NUMSCI_LIB="$(pick_first "$REPO"/build-numsci/iossim/libnumsci_*.a "$REPO"/build-numsci/*iossim*.a)"
fi
[ -d "$NUMSCI_GEN" ] || { echo "numsci gen tree not found ($NUMSCI_GEN). Run scripts/build-numsci.sh iossim"; exit 1; }
[ -n "$NUMSCI_LIB" ] || { echo "numsci iossim archive not found. Run scripts/build-numsci.sh iossim"; exit 1; }
NUMPP="${NUMPP_DIR:-/Users/leonardoaraujo/work/NumPP}/include"
SCIPP="${SCIPP_DIR:-/Users/leonardoaraujo/work/SciPP}/include"

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
echo "   kernel  : ${#KERNEL_SRCS[@]} src TU(s) (facade + core + engine[native+occt] + native math) [NUMSCI]"
echo "   numsci  : $NUMSCI_LIB"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT -DCYBERCAD_HAS_NUMSCI=1 \
  -I"$REPO/include" \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -I"$NUMSCI_GEN" -I"$NUMPP" -I"$SCIPP" \
  -x objective-c++ "$HARNESS" \
  -x c++ "${KERNEL_SRCS[@]}" \
  -x none "$NUMSCI_LIB" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_curved_fillet_parity"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_curved_fillet_parity"
