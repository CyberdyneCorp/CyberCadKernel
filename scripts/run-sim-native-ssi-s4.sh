#!/usr/bin/env bash
# Compile the SSI Stage-S4 (coincident + tangent-contact CLASSIFICATION) native-vs-OCCT
# parity harness for the iOS simulator, link it against the OCCT analytic classifier slice
# (IntAna_QuadQuadGeo + gp_* quadrics + Geom_* surfaces + GeomLProp_SLProps + projection)
# AND the NumPP/SciPP numsci archive (the seeded differential-geometry classifier), and run
# it inside a booted simulator via `xcrun simctl spawn`.
#
# This is the SSI Stage-S4 verification gate 2 — native S4 CLASSIFICATION vs OCCT
# (native_ssi_s4_classification_parity.mm). It does NOT trace curves (that is S4-c, out of
# scope): it asserts the native (coincident / tangent-point / tangent-curve / transversal)
# bucket matches OCCT's IntAna_ResultType (Same / Point / tangent Line-Circle / Empty) for
# the analytic pairs, and the seeded differential-geometry classifier matches OCCT's local
# normals for the near-tangent freeform/quadric pairs. Gate 1 (host, no OCCT) is
# tests/native/test_native_ssi_s4_classification.cpp.
#
# Like the S2/S3 parity harnesses, the SEEDED half needs the substrate, so this script:
#   * builds the numsci iossim archive first (scripts/build-numsci.sh iossim);
#   * compiles src/native/ssi/seeding.cpp + src/native/numerics/numerics.cpp under
#     -DCYBERCAD_HAS_NUMSCI, plus the OCCT-free native math TUs (bspline/bezier);
#   * links libnumsci_*_iossim_*.a (NumPP + SciPP-optimize/linalg);
#   * links the OCCT oracle toolkits (IntAna lives in TKGeomAlgo; GeomLProp_SLProps in
#     TKGeomBase — same slice/order as run-sim-native-ssi-seeding-parity.sh).
#
# SSI is INTERNAL — NO cc_* entry point; asserted at the cybercad::native::ssi C++
# boundary. On the SKIP list of run-sim-suite.sh (own main(), OCCT+numsci slice).
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_ssi_s4_classification_parity.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# ── build the numsci iossim substrate archive first (the seeded classifier) ───────
if [ -z "${CYBERCAD_NUMSCI_DIR:-}" ] && [ ! -d "$REPO/build-numsci/iossim/gen" ]; then
  echo "── building numsci iossim substrate (scripts/build-numsci.sh iossim)"
  "$REPO/scripts/build-numsci.sh" iossim
fi

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
NATIVE_SRCS=()
while IFS= read -r src; do NATIVE_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp'; \
      echo "$REPO/src/native/ssi/seeding.cpp"; \
      echo "$REPO/src/native/numerics/numerics.cpp")

# ── OCCT oracle toolkits (IntAna_QuadQuadGeo → TKGeomAlgo; GeomLProp → TKGeomBase) ─
TKS="TKGeomAlgo TKBRep TKG3d TKGeomBase TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling SSI-S4 coincident+tangent CLASSIFICATION parity harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} TU(s) (math + ssi/seeding + numerics) [NUMSCI]"
echo "   numsci  : $NUMSCI_LIB"
echo "   oracle  : OCCT IntAna_QuadQuadGeo + gp_* quadrics + Geom_* + GeomLProp_SLProps"
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
  -o "$OUT/native_ssi_s4_classification_parity"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_ssi_s4_classification_parity"
