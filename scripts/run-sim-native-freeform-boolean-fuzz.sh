#!/usr/bin/env bash
# Compile + run the MOAT M6 breadth-18 FREEFORM-BOOLEAN DIFFERENTIAL-FUZZING harness
# (tests/sim/native_freeform_boolean_fuzz.mm) for the iOS simulator. This certifies the
# large freeform-boolean surface that landed THIS campaign (src/native/boolean/:
# half_space_cut.h off-centre CUT/COMMON, slab_disjoint_cut.h multi-lump CUT,
# curved_wall_cut.h dome/bowl closed-seam CUT/COMMON, plus the Steinmetz bicylinder COMMON
# through nb::boolean_solid) — the freeform poses the original curved-boolean fuzzer
# (run-sim-native-boolean-fuzz.sh) predates and does NOT cover.
#
# The harness DETERMINISTICALLY generates random VALID cut POSES per pose-family
# (off-centre half-space at a random plane offset; disjoint slab at a random half-width;
# curved-wall plane at a random height; equal-R Steinmetz), runs EACH {operand, pose, op}
# through BOTH the SHIPPING native freeform verb (OCCT-FREE) AND the OCCT oracle
# (BRepAlgoAPI_{Cut,Common,Fuse}) AND a CLOSED-FORM volume arbiter (paraboloid-cap segment,
# polynomial prism integral, Steinmetz 16R³/3, partition identities), and classifies each:
#   AGREED            — native watertight + vol/area within a FIXED tol of OCCT + closed form
#   HONESTLY-DECLINED — native NULL / non-watertight → OCCT fallback (oracle valid)
#   DISAGREED         — native watertight but OUTSIDE tol (a SILENT WRONG RESULT)
#   ORACLE_UNRELIABLE — native matches the closed form, OCCT does NOT (native MORE correct)
# The bar: DISAGREED == 0. Any DISAGREE prints seed + case index + family/op/pose as a
# reproducible regression find. The generator is seeded ONLY by an explicit FUZZ_SEED
# (argv/env) — NO clock, NO rand(): same seed → byte-identical batch.
#
# Same OCCT+numsci slice as the sibling curved-boolean fuzzer: it links the OCCT oracle +
# NumPP/SciPP numsci archive (the M1 seam tracer the freeform verbs consume) and compiles
# the native math + ssi/{seeding,marching} + numerics + boolean/ssi_boolean TUs under
# -DCYBERCAD_HAS_NUMSCI. src/native stays OCCT-FREE; this harness is additive test/sim code
# only. On run-sim-suite.sh's SKIP list (own main(), OCCT+numsci slice, std::_Exit).
#
# Usage: scripts/run-sim-native-freeform-boolean-fuzz.sh [SEED] [N]
#   SEED  explicit uint64 RNG seed (default 0xF4EE0FB001). Also honoured via FUZZ_SEED env.
#   N     number of generated cases (default 72).          Also honoured via FUZZ_N env.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

SEED="${1:-${FUZZ_SEED:-0xF4EE0FB001}}"
N="${2:-${FUZZ_N:-72}}"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_freeform_boolean_fuzz.mm"
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

# ── native TUs to compile (identical slice to the sibling curved-boolean fuzzer) ──
NATIVE_SRCS=()
while IFS= read -r src; do NATIVE_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp'; \
      echo "$REPO/src/native/ssi/seeding.cpp"; \
      echo "$REPO/src/native/ssi/marching.cpp"; \
      echo "$REPO/src/native/numerics/numerics.cpp"; \
      echo "$REPO/src/native/boolean/ssi_boolean.cpp")

# ── OCCT oracle toolkits (BRepAlgoAPI + BRepPrimAPI + BRepGProp + BRepCheck + sewing) ──
TKS="TKBool TKBO TKPrim TKShHealing TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling M6 breadth-18 freeform-boolean differential-fuzz harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} TU(s) (math + ssi/{seeding,marching} + numerics + boolean/ssi_boolean) [NUMSCI]"
echo "   numsci  : $NUMSCI_LIB"
echo "   oracle  : OCCT BRepAlgoAPI_{Cut,Common,Fuse} + BRepPrimAPI + Bezier sew + BRepGProp"
echo "   seed    : $SEED   N : $N"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT -DCYBERCAD_HAS_NUMSCI=1 \
  -I"$REPO/src" \
  -I"$REPO/tests" \
  -I"$OCCT/include/opencascade" \
  -I"$NUMSCI_GEN" -I"$NUMPP" -I"$SCIPP" \
  -x objective-c++ "$HARNESS" \
  -x c++ "${NATIVE_SRCS[@]}" \
  -x none "$NUMSCI_LIB" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_freeform_boolean_fuzz"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID (seed=$SEED N=$N)"
xcrun simctl spawn "$UDID" "$OUT/native_freeform_boolean_fuzz" "$SEED" "$N"
