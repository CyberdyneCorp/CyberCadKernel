#!/usr/bin/env bash
# Compile + run the MOAT M6 DIFFERENTIAL-FUZZING harness (tests/sim/native_boolean_fuzz.mm)
# for the iOS simulator. This is the M6 COMPLETENESS BAR, first slice: it turns the curated
# curved-boolean parity harness (native-pass=18 on hand-picked fixtures) into a SEEDED batch
# of N random-but-VALID operand pairs and asserts ZERO SILENT WRONG RESULTS.
#
# The harness DETERMINISTICALLY generates random-valid cyl/sphere/cone/box operand pairs in
# the RECOGNISED native families (sphere∩sphere lens, cone∩sphere, cone∩cyl coaxial,
# Steinmetz, box∩cyl analytic, through-drill decline-probe), runs EACH {pair, op} through
# BOTH the native path (nb::boolean_solid, OCCT-FREE) AND the OCCT oracle
# (BRepAlgoAPI_{Fuse,Cut,Common}), and classifies each into exactly one of:
#   AGREED            — native watertight + vol/area within tol of OCCT
#   HONESTLY-DECLINED — native NULL / non-watertight → OCCT fallback (oracle valid)
#   DISAGREED         — native watertight but vol/area OUTSIDE tol (a SILENT WRONG RESULT)
# The bar: DISAGREED == 0. Any DISAGREE prints the seed + case index + tuple as a
# reproducible regression find. The generator is seeded ONLY by an explicit FUZZ_SEED
# (argv/env) — NO clock, NO rand(): same seed → byte-identical batch.
#
# Same OCCT+numsci slice as the sibling (scripts/run-sim-native-ssi-curved-boolean.sh):
# it links the OCCT oracle + NumPP/SciPP numsci archive and compiles the S5 driver
# (boolean/ssi_boolean.cpp) + the S3 tracer (ssi/{seeding,marching}.cpp) + the substrate
# (numerics/numerics.cpp) + native math under -DCYBERCAD_HAS_NUMSCI. src/native stays
# OCCT-FREE; this harness is additive test/sim code only. On run-sim-suite.sh's SKIP list
# (own main(), OCCT+numsci slice, std::_Exit).
#
# Usage: scripts/run-sim-native-boolean-fuzz.sh [SEED] [N]
#   SEED  explicit uint64 RNG seed (default 0xC0FFEE1234). Also honoured via FUZZ_SEED env.
#   N     number of generated cases (default 96).            Also honoured via FUZZ_N env.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

SEED="${1:-${FUZZ_SEED:-0xC0FFEE1234}}"
N="${2:-${FUZZ_N:-96}}"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_boolean_fuzz.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# ── build the numsci iossim substrate archive first (unless provided) ─────────────
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
  NUMSCI_LIB="$(pick_first "$REPO"/build-numsci/*iossim*.a "$REPO"/build-numsci/iossim/libnumsci_*.a "$REPO"/eval/libnumsci_full_iossim_arm64.a)"
fi
[ -d "$NUMSCI_GEN" ]  || { echo "numsci gen tree not found ($NUMSCI_GEN). Run scripts/build-numsci.sh iossim"; exit 1; }
[ -n "$NUMSCI_LIB" ]  || { echo "numsci iossim archive not found. Run scripts/build-numsci.sh iossim"; exit 1; }
NUMPP="${NUMPP_DIR:-/Users/leonardoaraujo/work/NumPP}/include"
SCIPP="${SCIPP_DIR:-/Users/leonardoaraujo/work/SciPP}/include"

# ── native TUs to compile (identical slice to the sibling curved-boolean harness) ──
NATIVE_SRCS=()
while IFS= read -r src; do NATIVE_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp'; \
      echo "$REPO/src/native/ssi/seeding.cpp"; \
      echo "$REPO/src/native/ssi/marching.cpp"; \
      echo "$REPO/src/native/numerics/numerics.cpp"; \
      echo "$REPO/src/native/boolean/ssi_boolean.cpp")

# ── OCCT oracle toolkits (BRepAlgoAPI + BRepPrimAPI + BRepGProp + BRepCheck) ────────
TKS="TKBool TKBO TKPrim TKShHealing TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling M6 differential-fuzz native-vs-OCCT harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} TU(s) (math + ssi/{seeding,marching} + numerics + boolean/ssi_boolean) [NUMSCI]"
echo "   numsci  : $NUMSCI_LIB"
echo "   oracle  : OCCT BRepAlgoAPI_{Fuse,Cut,Common} + BRepPrimAPI + BRepGProp + BRepCheck"
echo "   seed    : $SEED   N : $N"
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
  -o "$OUT/native_boolean_fuzz"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID (seed=$SEED N=$N)"
xcrun simctl spawn "$UDID" "$OUT/native_boolean_fuzz" "$SEED" "$N"
