#!/usr/bin/env bash
# Compile the SSI Stage-S4-c (NEAR-TANGENT MARCHING) native-vs-OCCT parity harness for the
# iOS simulator, link it against the OCCT SSI oracle slice (GeomAPI_IntSS + Geom_* surfaces
# + GeomLProp_SLProps + GCPnts_AbscissaPoint arc-length + point/curve projection Extrema)
# AND the NumPP/SciPP numsci archive (the least_squares corrector + lstsq B-spline fit),
# and run it inside a booted simulator via `xcrun simctl spawn`.
#
# This is the SSI-ROADMAP.md Stage-S4-c verification gate 2 — native near-tangent MARCHING
# vs OCCT. The harness is the SAME translation unit as the S3 gate
# (tests/sim/native_ssi_marching_parity.mm): the S3 transversal pairs run unchanged (a
# non-regression witness that S4-c is bit-identical outside the near-tangent band) and two
# S4-c cases are added:
#   * `nt-cross s4c`  — an OFFSET cylinder GRAZES a unit sphere: the intersection is a single
#     closed loop whose transversality sine dips into the near-tangent band but the curve
#     genuinely CONTINUES through it. S3 would TRUNCATE (NearTangent); S4-c MARCHES THROUGH
#     with the fixed-plane-cut corrector. Asserts nearTangentGaps==0, nearTangentCrossed>=1,
#     one Closed loop, every densely-sampled crossed node ON the OCCT locus AND on both
#     surfaces within tol, crossed-arc residual <= onSurfTol — a verified crossing, NOT a
#     fabricated path or an honest truncation. (At the graze OCCT tolerance-splits the loop
#     into branches while native crosses it into one — an honest CONNECTIVITY disagreement AT
#     the tangency; the gate asserts the uncontested facts: the native curve lies on OCCT's
#     locus and it was a crossing, not a truncation.)
#   * `eq-cyl defer`  — two equal cylinders crossing at 90° meet at a saddle BRANCH point
#     (sine -> 0, S4-d). S4-c's crossable gate must REFUSE it: nearTangentCrossed==0,
#     nearTangentGaps>=1 — an honest near-tangent gap deferred to OCCT, never a fabricated
#     crossing.
# Gate 1 (host, no OCCT) is tests/native/test_native_ssi_marching.cpp
# (march_near_tangent_crossed_s4c + march_tangent_curve_not_crossed_s4c).
#
# S4-c's corrector + fit + seeded tangent-contact classifier need the substrate, so this
# script (like run-sim-native-ssi-marching.sh, on which it is modelled):
#   * builds the numsci iossim archive first (scripts/build-numsci.sh iossim);
#   * compiles src/native/ssi/{seeding.cpp,marching.cpp} +
#     src/native/numerics/numerics.cpp under -DCYBERCAD_HAS_NUMSCI, plus the OCCT-free
#     native math TUs (bspline/bezier);
#   * links libnumsci_*_iossim_*.a (NumPP + SciPP-optimize/linalg);
#   * links the OCCT oracle toolkits (same slice + order as run-sim-native-ssi-marching.sh).
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

# ── native TUs to compile (same slice as the S3 marching harness) ─────────────────
NATIVE_SRCS=()
while IFS= read -r src; do NATIVE_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp'; \
      echo "$REPO/src/native/ssi/seeding.cpp"; \
      echo "$REPO/src/native/ssi/marching.cpp"; \
      echo "$REPO/src/native/numerics/numerics.cpp")

# ── OCCT oracle toolkits (same slice/order as the S3 marching harness) ─────────────
TKS="TKGeomAlgo TKBRep TKG3d TKGeomBase TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling SSI-S4-c near-tangent-marching native-vs-OCCT parity harness for iphonesimulator (arm64)"
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
