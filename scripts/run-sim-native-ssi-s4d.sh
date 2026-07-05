#!/usr/bin/env bash
# Compile the SSI Stage-S4-d (BRANCH POINTS / self-crossing loci) native-vs-OCCT parity
# harness for the iOS simulator, link it against the OCCT SSI oracle slice (GeomAPI_IntSS +
# Geom_* surfaces + GeomLProp_SLProps + GCPnts_AbscissaPoint arc-length + point/curve
# projection Extrema) AND the NumPP/SciPP numsci archive (the least_squares corrector, the
# 1-D minimize localizer, and the lstsq B-spline fit), and run it inside a booted simulator.
#
# This is the SSI-ROADMAP Stage-S4-d verification gate 2 — native branch-point handling vs
# OCCT. The harness is the SAME translation unit as the S3/S4-c gates
# (tests/sim/native_ssi_marching_parity.mm): the S3 transversal pairs + the S4-c graze case
# + the S4-c `eq-cyl defer` control all run UNCHANGED (a non-regression witness that S4-d is
# additive — the default path still defers the branch saddle), and one S4-d case is added:
#   * `eq-cyl s4d` — the STEINMETZ bicylinder (two equal R=1 cylinders, axes Z and X crossing
#     orthogonally at the origin), traced with MarchOptions.enableBranchPoints. Its
#     intersection is TWO ellipses (planes x=±z) crossing at two branch points (0,±1,0). The
#     native tracer must LOCALIZE both branch points and ROUTE the arms so the FULL multi-arm
#     intersection is traced. Asserts branchPoints==2 (both saddles on both OCCT surfaces AND
#     on the OCCT locus within tol), every native arc node on the OCCT locus + both surfaces
#     within tol (no fabricated points), nearTangentGaps==0, tracedBranches>=4. OCCT
#     tolerance-splits the figure-8 into its own arc set; the gate asserts the uncontested
#     facts — the native curve lies on OCCT's locus + surfaces and both branch points match.
# Gate 1 (host, no OCCT) is tests/native/test_native_ssi_marching.cpp
# (march_steinmetz_branch_points_s4d + march_tangent_point_never_branches_s4d).
#
# S4-d's corrector + localizer + fit + seeded tangent-contact classifier need the substrate,
# so this script (like run-sim-native-ssi-s4c.sh, on which it is modelled):
#   * builds the numsci iossim archive first (scripts/build-numsci.sh iossim);
#   * compiles src/native/ssi/{seeding.cpp,marching.cpp} + src/native/numerics/numerics.cpp
#     under -DCYBERCAD_HAS_NUMSCI, plus the OCCT-free native math TUs;
#   * links libnumsci_*_iossim_*.a (NumPP + SciPP-optimize/linalg);
#   * links the OCCT oracle toolkits (same slice + order as run-sim-native-ssi-s4c.sh).
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

# ── native TUs to compile (same slice as the S3/S4-c marching harness) ─────────────
NATIVE_SRCS=()
while IFS= read -r src; do NATIVE_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp'; \
      echo "$REPO/src/native/ssi/seeding.cpp"; \
      echo "$REPO/src/native/ssi/marching.cpp"; \
      echo "$REPO/src/native/numerics/numerics.cpp")

# ── OCCT oracle toolkits (same slice/order as the S3/S4-c marching harness) ─────────
TKS="TKGeomAlgo TKBRep TKG3d TKGeomBase TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling SSI-S4-d branch-point native-vs-OCCT parity harness for iphonesimulator (arm64)"
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
