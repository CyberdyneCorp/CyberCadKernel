#!/usr/bin/env bash
# Compile the SSI Stage-S3 (marching-line tracer / WLine) native-vs-OCCT parity harness
# for the iOS simulator, link it against the OCCT SSI oracle slice (GeomAPI_IntSS +
# Geom_* surfaces + GeomLProp_SLProps + GCPnts_AbscissaPoint arc-length + point/curve
# projection Extrema) AND the NumPP/SciPP numsci archive (the least_squares corrector +
# lstsq B-spline fit), and run it inside a booted simulator via `xcrun simctl spawn`.
#
# This is the SSI-ROADMAP.md Stage-S3 verification gate 2 — the native-vs-OCCT parity
# pass (native_ssi_marching_parity.mm). It asserts, per transversal branch: same number
# of TRANSVERSAL branches traced; densely-sampled native WLine points lie on the OCCT
# intersection curve (< tol) AND on both surfaces; native curve length ≈ OCCT length
# within a deflection/step tol; closed OCCT loops → native Closed WLine; near-tangent
# branches marked NearTangentTruncated and reported separately (S4). Gate 1 (host, no
# OCCT) is tests/native/test_native_ssi_marching.cpp.
#
# Like the S2 harnesses (and unlike the S1 header-only harness), S3's corrector + fit
# need the substrate, so this script:
#   * builds the numsci iossim archive first (scripts/build-numsci.sh iossim);
#   * compiles src/native/ssi/{seeding.cpp,marching.cpp} +
#     src/native/numerics/numerics.cpp under -DCYBERCAD_HAS_NUMSCI, plus the OCCT-free
#     native math TUs (bspline/bezier);
#   * links libnumsci_full_iossim_arm64.a (NumPP + SciPP-optimize/linalg);
#   * links the OCCT oracle toolkits (same slice + order as run-sim-native-ssi.sh);
#     GeomLProp_SLProps (surface normals) lives in TKGeomBase and GCPnts_AbscissaPoint
#     (arc length) + Geom_BezierSurface in TKGeomAlgo/TKG3d — all already in the slice.
#
# SSI is INTERNAL — NO cc_* entry point; asserted at the cybercad::native::ssi C++
# boundary. On the SKIP list of run-sim-suite.sh (own main(), OCCT+numsci slice).
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_ssi_marching_parity.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# ── build the numsci iossim substrate archive first ──────────────────────────────
if [ -z "${CYBERCAD_NUMSCI_DIR:-}" ] && [ ! -d "$REPO/build-numsci/iossim/gen" ]; then
  echo "── building numsci iossim substrate (scripts/build-numsci.sh iossim)"
  "$REPO/scripts/build-numsci.sh" iossim
fi

# ── numsci substrate (iossim archive + generated config headers) ─────────────────
# Prefer an explicit NUMSCI_DIR (build-numsci.sh iossim output: gen/ + libnumsci_*.a);
# else fall back to the eval/ archive + the build-numsci/iossim gen tree.
# pick_first <glob...> — first existing path matching any argument (no `ls | head`,
# whose SIGPIPE under `pipefail`+`set -e` would abort the script).
pick_first() { for p in "$@"; do [ -e "$p" ] && { printf '%s\n' "$p"; return 0; }; done; return 0; }

NUMSCI_DIR="${CYBERCAD_NUMSCI_DIR:-}"
if [ -n "$NUMSCI_DIR" ] && [ -d "$NUMSCI_DIR/gen" ]; then
  NUMSCI_GEN="$NUMSCI_DIR/gen"
  NUMSCI_LIB="$(pick_first "$NUMSCI_DIR"/libnumsci_*.a)"
else
  NUMSCI_GEN="$REPO/build-numsci/iossim/gen"
  NUMSCI_LIB="$(pick_first "$REPO"/build-numsci/*iossim*.a "$REPO"/eval/libnumsci_full_iossim_arm64.a)"
fi
[ -d "$NUMSCI_GEN" ]  || { echo "numsci gen tree not found ($NUMSCI_GEN). Run scripts/build-numsci.sh iossim"; exit 1; }
[ -n "$NUMSCI_LIB" ]  || { echo "numsci iossim archive not found. Run scripts/build-numsci.sh iossim"; exit 1; }
NUMPP="${NUMPP_DIR:-/Users/leonardoaraujo/work/NumPP}/include"
SCIPP="${SCIPP_DIR:-/Users/leonardoaraujo/work/SciPP}/include"

# ── native TUs to compile ────────────────────────────────────────────────────────
# S3 adds src/native/ssi/marching.cpp (the tracer) alongside seeding.cpp (the S2 input)
# and numerics.cpp (the substrate corrector + fit).
NATIVE_SRCS=()
while IFS= read -r src; do NATIVE_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp'; \
      echo "$REPO/src/native/ssi/seeding.cpp"; \
      echo "$REPO/src/native/ssi/marching.cpp"; \
      echo "$REPO/src/native/numerics/numerics.cpp")

# ── OCCT oracle toolkits (same slice/order as the S1/S2 harnesses) ─────────────────
# GeomLProp_SLProps (surface normals) → TKGeomBase; GCPnts_AbscissaPoint (arc length),
# GeomAdaptor_Curve, Geom_BezierSurface → TKGeomAlgo/TKG3d. All already in the slice.
TKS="TKGeomAlgo TKBRep TKG3d TKGeomBase TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling SSI-S3 marching-tracer native-vs-OCCT parity harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} TU(s) (math + ssi/{seeding,marching} + numerics) [NUMSCI]"
echo "   numsci  : $NUMSCI_LIB"
echo "   oracle  : OCCT GeomAPI_IntSS + Geom_* + GeomLProp_SLProps + GCPnts_AbscissaPoint + ProjectPointOn{Surf,Curve}"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT -DCYBERCAD_HAS_NUMSCI=1 \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -I"$NUMSCI_GEN" -I"$NUMPP" -I"$SCIPP" \
  -x objective-c++ "$HARNESS" \
  -x c++ "${NATIVE_SRCS[@]}" \
  -x none "$NUMSCI_LIB" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_ssi_marching_parity"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_ssi_marching_parity"
