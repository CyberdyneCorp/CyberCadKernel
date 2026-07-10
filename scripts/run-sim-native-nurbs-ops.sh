#!/usr/bin/env bash
# Compile the exact-NURBS-ops vs OCCT-oracle parity harness for the iOS simulator,
# link it against the native math sources (src/native/math/*.cpp — OCCT-FREE, which
# includes bspline_ops.cpp, the NURBS roadmap Layer-1 knot/degree/split ops) and the
# minimal OCCT libraries used as the reference (TKG3d/TKGeomBase/TKG2d/TKMath/TKernel:
# BSplCLib/BSplSLib in TKMath, Geom_BSplineCurve/Surface in TKG3d), and run it inside
# a booted simulator via `xcrun simctl spawn`.
#
# This is NURBS roadmap Layer 1 (exact-NURBS geometry kernel) verification gate 2 —
# the native-vs-OCCT parity pass (Task 5 of openspec/changes/nurbs-exact-geometry-
# kernel). Gate 1 (host exact-oracle unit tests, no OCCT) is
# tests/native/test_native_nurbs_ops.cpp, and the host differential fuzzer is
# tests/native/test_native_nurbs_ops_fuzz.cpp — both built by CTest with plain
# clang++; this script does NOT need them.
#
# The parity harness diffs each native op (knot insert/refine/remove, degree elevate,
# split, decompose-to-Bezier — curve + surface, rational + non-rational) against its
# OCCT reference (InsertKnot(s)/IncreaseDegree/RemoveKnot(Index-addressed)/Segment),
# converting the kernel's FLAT knot vector to/from OCCT's separate knots+mults arrays.
# Ops that OCCT relocates knots for (Segment/IncreaseDegree) are compared on sampled
# points, not raw knots. Fixed 1e-9 cross-library tolerance — never widened.
#
# Companion to run-sim-suite.sh; on that suite's SKIP list because it carries its own
# main() and links only the geometry-oracle slice of OCCT. Model: run-sim-native-math.sh.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_nurbs_ops_parity.mm"
# Native math implementation TUs (bspline.cpp, bezier.cpp, bspline_ops.cpp, …),
# compiled OCCT-free — they carry no OCCT include.
NATIVE_SRCS=()
while IFS= read -r src; do NATIVE_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp' | sort)
[ "${#NATIVE_SRCS[@]}" -gt 0 ] || { echo "no native math sources under src/native/math"; exit 1; }

TKS="TKG3d TKGeomBase TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling native-nurbs-ops parity harness for iphonesimulator (arm64)"
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
  -o "$OUT/native_nurbs_ops_parity"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_nurbs_ops_parity"
