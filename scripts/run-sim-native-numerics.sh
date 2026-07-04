#!/usr/bin/env bash
# Compile the native-numerics (closest-point / Extrema) vs OCCT-oracle parity
# harness for the iOS simulator, link it against the native numeric facade
# (src/native/numerics/numerics.cpp) + native math (src/native/math/*.cpp) + the
# NumPP/SciPP substrate archive + the OCCT Extrema oracle slice, and run it inside
# a booted simulator via `xcrun simctl spawn`.
#
# This is Phase 4 capability #2 (`native-numerics`, numeric-foundations)
# verification gate 2 — the native-vs-OCCT parity pass (openspec/NATIVE-REWRITE.md,
# docs/EVAL-numpp-scipp.md verdict GO-WITH-HARDENING). Gate 1 (host unit tests, no
# OCCT) is separate; this script does NOT need it.
#
# The numeric substrate (NumPP + the SciPP optimize/linalg subset) is referenced
# BY ABSOLUTE PATH exactly like OCCT (never vendored). It is built into a single
# static archive by scripts/build-numsci.sh (which reproduces the eval-verified
# CPU-only recipe: all NUMPP_WITH_*/SCIPP_WITH_* = 0, src/special + src/stats
# EXCLUDED — the Homebrew/iOS libc++ ISO-29124 gap). We (re)build it here for the
# iossim target, then compile:
#   * the harness (tests/sim/native_numerics_parity.mm)          — OCCT + NumSci
#   * src/native/numerics/numerics.cpp (-DCYBERCAD_HAS_NUMSCI)   — the only TU that
#                                                                  touches NumPP/SciPP
#   * src/native/math/*.cpp (OCCT-free)                          — geometry to project
# against the OCCT Extrema oracle slice (TKG3d/TKGeomBase/TKG2d/TKMath/TKernel via
# GeomAPI_ProjectPointOnSurf / GeomAPI_ProjectPointOnCurve).
#
# Only the numerics module is compiled under CYBERCAD_HAS_NUMSCI; the rest of
# src/native still builds without the substrate (the module is guarded), so the
# existing suites are unaffected.
#
# Companion to run-sim-suite.sh (the full cc_* facade suite). This harness is on
# that suite's SKIP list — it carries its own main() and links the geometry-oracle
# slice of OCCT + the NumPP/SciPP archive, not the whole kernel static lib.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

NUMPP="${NUMPP_DIR:-/Users/leonardoaraujo/work/NumPP}"
SCIPP="${SCIPP_DIR:-/Users/leonardoaraujo/work/SciPP}"
NUMSCI_OUT="${NUMSCI_DIR:-$REPO/build-numsci/iossim}"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }
[ -d "$NUMPP/include/numpp" ] || { echo "NumPP not found at $NUMPP"; exit 1; }
[ -d "$SCIPP/include/scipp" ] || { echo "SciPP not found at $SCIPP"; exit 1; }

HARNESS="$REPO/tests/sim/native_numerics_parity.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# ── (1) build the NumPP/SciPP substrate archive for the iOS simulator ──────────
# Reproduces docs/EVAL-numpp-scipp.md (CPU-only, special/stats EXCLUDED).
NUMSCI_LIB="$NUMSCI_OUT/libnumsci_iossim_arm64.a"
if [ ! -f "$NUMSCI_LIB" ] || [ -n "${NUMSCI_REBUILD:-}" ]; then
  echo "── building NumPP/SciPP substrate (iossim) via scripts/build-numsci.sh"
  "$REPO/scripts/build-numsci.sh" iossim --out "$NUMSCI_OUT" --numpp "$NUMPP" --scipp "$SCIPP"
fi
[ -f "$NUMSCI_LIB" ] || { echo "numsci archive not built: $NUMSCI_LIB"; exit 1; }
NUMSCI_GEN="$NUMSCI_OUT/gen"
[ -d "$NUMSCI_GEN/numpp" ] || { echo "numsci gen headers missing: $NUMSCI_GEN"; exit 1; }

# ── (2) native TUs to compile in ───────────────────────────────────────────────
# The numeric facade (the ONLY TU that includes NumPP/SciPP) under the guard, plus
# the OCCT-free native math implementation TUs the projections evaluate.
NUMERICS_SRC="$REPO/src/native/numerics/numerics.cpp"
[ -f "$NUMERICS_SRC" ] || { echo "missing $NUMERICS_SRC"; exit 1; }
MATH_SRCS=()
while IFS= read -r src; do MATH_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp' | sort)
[ "${#MATH_SRCS[@]}" -gt 0 ] || { echo "no native math sources under src/native/math"; exit 1; }

# OCCT oracle libraries: GeomAPI_ProjectPointOnSurf/Curve (and their Extrema_GenExtPS
# / Extrema_ExtPC drivers) are defined in TKGeomAlgo; the Adaptor3d/GeomAdaptor
# surfaces/curves they build on live in TKG3d/TKGeomBase; Geom_* also in TKG3d; the
# Extrema_* + gp_*/ElSLib primitives in TKMath; base TKernel. Link order is
# most-derived → base for the static archives (TKGeomAlgo first so its GeomAPI
# undefined refs resolve against the libs that follow it).
TKS="TKGeomAlgo TKG3d TKGeomBase TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling native-numerics parity harness for iphonesimulator (arm64)"
echo "   harness  : $HARNESS"
echo "   numerics : $NUMERICS_SRC (-DCYBERCAD_HAS_NUMSCI)"
echo "   math     : ${#MATH_SRCS[@]} source(s)"
echo "   substrate: $NUMSCI_LIB"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT -DCYBERCAD_HAS_NUMSCI \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -I"$NUMPP/include" \
  -I"$SCIPP/include" \
  -I"$NUMSCI_GEN" \
  -x objective-c++ "$HARNESS" \
  -x c++ "$NUMERICS_SRC" \
  "${MATH_SRCS[@]}" \
  -x none "$NUMSCI_LIB" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_numerics_parity"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_numerics_parity"
