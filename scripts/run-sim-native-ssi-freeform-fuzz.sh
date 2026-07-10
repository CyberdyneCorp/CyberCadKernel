#!/usr/bin/env bash
# Compile the GENERAL FREEFORM SSI differential fuzzer (tests/sim/native_ssi_freeform_fuzz.mm)
# for the iOS simulator, link it against the OCCT SSI oracle slice (GeomAPI_IntSS + Geom_*
# surfaces + point/curve projection Extrema) AND the NumPP/SciPP numsci archive (the
# least_squares corrector + lstsq B-spline fit), and run it inside a booted simulator via
# `xcrun simctl spawn`.
#
# This is the empirical instrument for NURBS roadmap Layer 2 (general NURBS↔NURBS SSI). It
# GENERATES random valid NURBS↔NURBS surface pairs (deterministic splitmix64→xoshiro256**,
# FUZZ_SEED-keyed), runs the native S2 seed_intersection → S3 trace_intersection pipeline
# against OCCT GeomAPI_IntSS as the ORACLE, and classifies EVERY trial into exactly one of
# AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE. The bar is DISAGREED==0 across
# ≥ 2 seeds, N ≥ 40/seed — a high decline rate is FINE and EXPECTED (it is the S4 work-list);
# the fuzzer NEVER widens a tolerance to manufacture agreement. Exit 0 iff DISAGREED==0.
#
# Like the S2/S3 parity harnesses, the corrector + fit need the substrate, so this script:
#   * builds the numsci iossim archive first (scripts/build-numsci.sh iossim) if absent;
#   * compiles src/native/ssi/{seeding.cpp,marching.cpp} + src/native/numerics/numerics.cpp
#     under -DCYBERCAD_HAS_NUMSCI, plus the OCCT-free native math TUs (bspline/bezier);
#   * links the numsci iossim archive (NumPP + SciPP-optimize/linalg);
#   * links the OCCT oracle toolkits (same slice + order as run-sim-native-ssi-marching.sh).
#
# This is a SIM (native-vs-OCCT) TEST HARNESS — it does NOT modify src/native (which stays
# OCCT-free) and adds NO cc_* entry point; asserted at the cybercad::native::ssi C++
# boundary. On the SKIP list of run-sim-suite.sh (own main(), OCCT+numsci slice; .mm files
# are excluded by that suite's *.cpp find regardless).
#
# Usage: run-sim-native-ssi-freeform-fuzz.sh [SEED] [N] [NSEEDS]
#   SEED   base RNG seed (default 0x5515D1FF0F0F; env FUZZ_SEED also honoured)
#   N      trials per seed (default 48, floored at 40)
#   NSEEDS number of seeds (default 2, floored at 2)
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_ssi_freeform_fuzz.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# ── build the numsci iossim substrate archive first (if absent) ───────────────────
if [ -z "${CYBERCAD_NUMSCI_DIR:-}" ] && [ ! -d "$REPO/build-numsci/iossim/gen" ]; then
  echo "── building numsci iossim substrate (scripts/build-numsci.sh iossim)"
  "$REPO/scripts/build-numsci.sh" iossim
fi

# ── numsci substrate (iossim archive + generated config headers) ──────────────────
# pick_first <glob...> — first existing path matching any argument (no `ls | head`, whose
# SIGPIPE under `pipefail`+`set -e` would abort the script).
pick_first() { for p in "$@"; do [ -e "$p" ] && { printf '%s\n' "$p"; return 0; }; done; return 0; }

NUMSCI_DIR="${CYBERCAD_NUMSCI_DIR:-$REPO/build-numsci/iossim}"
NUMSCI_GEN="$NUMSCI_DIR/gen"
NUMSCI_LIB="$(pick_first "$NUMSCI_DIR"/libnumsci_*.a "$REPO"/build-numsci/*iossim*.a "$REPO"/eval/libnumsci_full_iossim_arm64.a)"
[ -d "$NUMSCI_GEN" ] || { echo "numsci gen tree not found ($NUMSCI_GEN). Run scripts/build-numsci.sh iossim"; exit 1; }
[ -n "$NUMSCI_LIB" ] || { echo "numsci iossim archive not found. Run scripts/build-numsci.sh iossim"; exit 1; }
NUMPP="${NUMPP_DIR:-/Users/leonardoaraujo/work/NumPP}/include"
SCIPP="${SCIPP_DIR:-/Users/leonardoaraujo/work/SciPP}/include"

# ── native TUs to compile (math + ssi/{seeding,marching} + numerics substrate) ─────
NATIVE_SRCS=()
while IFS= read -r src; do NATIVE_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp'; \
      echo "$REPO/src/native/ssi/seeding.cpp"; \
      echo "$REPO/src/native/ssi/marching.cpp"; \
      echo "$REPO/src/native/numerics/numerics.cpp")

# ── OCCT oracle toolkits (same slice/order as the S3 marching parity harness) ──────
TKS="TKGeomAlgo TKBRep TKG3d TKGeomBase TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling GENERAL FREEFORM SSI differential fuzzer for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} TU(s) (math + ssi/{seeding,marching} + numerics) [NUMSCI]"
echo "   numsci  : $NUMSCI_LIB"
echo "   oracle  : OCCT GeomAPI_IntSS + Geom_BSplineSurface + ProjectPointOn{Surf,Curve}"
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
  -o "$OUT/native_ssi_freeform_fuzz"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_ssi_freeform_fuzz" "$@"
