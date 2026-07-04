#!/usr/bin/env bash
# Compile the native analytic-SSI (Stage S1) vs OCCT-oracle parity harness for the
# iOS simulator, link it against the OCCT SSI oracle slice (GeomAPI_IntSS +
# Geom_* surfaces + point-projection Extrema), and run it inside a booted simulator
# via `xcrun simctl spawn`.
#
# This is the SSI capability (openspec/SSI-ROADMAP.md) Stage S1 verification gate 2
# — the native-vs-OCCT parity pass. Gate 1 (host analytic, no OCCT) is the separate
# tests/native/test_native_ssi.cpp; this script does NOT need it.
#
# SSI is an INTERNAL kernel capability: there is NO cc_* facade entry point and NO
# ABI change — it is verified at the SSI-function level (native intersect_surfaces
# curves vs OCCT GeomAPI_IntSS), exactly like the native-math / topology parity gates.
#
# What is compiled:
#   * the harness (tests/sim/native_ssi_parity.mm)   — OCCT-dependent (the oracle)
#   * src/native/math/*.cpp (bspline/bezier)         — OCCT-free native geometry
# The native SSI layer (src/native/ssi/*) is HEADER-ONLY and pure closed-form
# geometry over src/native/math only — it has NO .cpp to compile and NO NumPP/SciPP
# dependency (native_ssi.h: every S1 reduction stays at degree ≤ 2 and is solved
# in-line; the only case that would need the polynomial-roots substrate — the general
# oblique plane∩torus quartic — is DEFERRED). So this harness links the OCCT oracle
# and the OCCT-free native math TUs only; there is NO -DCYBERCAD_HAS_NUMSCI and NO
# numsci archive here.
#
# OCCT oracle libraries: GeomAPI_IntSS (and the IntPatch/IntAna intersectors it
# drives) plus GeomAPI_ProjectPointOnSurf/Curve live in TKGeomAlgo; the IntPatch arc
# intersectors it pulls in reference BRep_Tool / BRepAdaptor_Curve from TKBRep; the
# Geom_* surfaces/curves and Adaptor3d/GeomAdaptor in TKG3d/TKGeomBase; TKG2d for the
# pcurve machinery; the gp_*/ElSLib/Extrema primitives in TKMath; base TKernel. Link
# order is most-derived → base for the static archives (TKGeomAlgo first so its GeomAPI
# undefined refs resolve against the libs that follow it, with TKBRep after it).
#
# Companion to run-sim-suite.sh (the full cc_* facade suite). This harness is on that
# suite's SKIP list — it carries its own main() and links the SSI-oracle slice of
# OCCT, not the whole kernel static lib.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_ssi_parity.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# ── native TUs to compile in ───────────────────────────────────────────────────
# The native SSI layer is header-only; only the OCCT-free native math implementation
# TUs (bspline/bezier — pulled transitively by the math headers) are compiled here.
MATH_SRCS=()
while IFS= read -r src; do MATH_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp' | sort)
[ "${#MATH_SRCS[@]}" -gt 0 ] || { echo "no native math sources under src/native/math"; exit 1; }

# TKBRep (BRep_Tool / BRepAdaptor_Curve) is required: IntPatch_TheSOnBounds — pulled
# in by the IntPatch arc intersectors behind GeomAPI_IntSS (it walks surface boundary
# arcs as edges) — references it even though our oracle only feeds it Geom_* surfaces.
# Static-archive link order is most-derived → base, so TKBRep sits AFTER TKGeomAlgo
# (whose IntPatch_TheSOnBounds.o has the undefined refs) and BEFORE the geometry base
# toolkits (TKG3d/TKGeomBase/TKMath/TKernel) it depends on.
TKS="TKGeomAlgo TKBRep TKG3d TKGeomBase TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling native-SSI parity harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   ssi     : src/native/ssi/* (header-only)"
echo "   math    : ${#MATH_SRCS[@]} source(s)"
echo "   oracle  : OCCT GeomAPI_IntSS + Geom_* + ProjectPointOn{Surf,Curve}"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  -x c++ "${MATH_SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_ssi_parity"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_ssi_parity"
