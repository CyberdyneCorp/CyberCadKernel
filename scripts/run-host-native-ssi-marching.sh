#!/usr/bin/env bash
#
# run-host-native-ssi-marching.sh — LINUX HOST port of the SSI Stage-S3 (marching-line
# tracer / WLine) native-vs-OCCT parity harness, i.e. Gate B runnable without a Mac.
#
# It compiles THE SAME TU as scripts/run-sim-native-ssi-marching.sh —
# tests/sim/native_ssi_marching_parity.mm, which is pure C++ despite the .mm extension
# (no #import / @interface / Foundation) — against the SYSTEM OpenCASCADE instead of the
# trimmed arm64-iOS OCCT, with the linuxhost numsci archive instead of the iossim one.
# Because the harness source, the native TU set, the assertions and the tolerances are
# IDENTICAL, a green run here is the same contract the simulator gate asserts.
#
# NOT a replacement for the simulator gate. Two honest differences remain, so the sim
# run stays the authority before a change is archived:
#   * OCCT BUILD — system OCCT (Debian/Ubuntu libocct 7.6.x) vs the trimmed arm64-iOS
#     OCCT the sim gate links. Same algorithms, but a different build/version, so an
#     oracle-side numeric difference is possible.
#   * TOOLCHAIN/ABI — x86-64 GCC/libstdc++ vs arm64 clang/libc++. Different FP codegen
#     (FMA contraction, libm) can move the last bits of a near-tangent corrector.
# Use this as the fast pre-check that catches real regressions in seconds; confirm on the
# simulator before archiving.
#
# Prereqs (Debian/Ubuntu):
#   sudo apt install libocct-foundation-dev libocct-modeling-data-dev \
#                    libocct-modeling-algorithms-dev
#   scripts/build-numsci.sh linuxhost
#
# Usage:
#   scripts/run-host-native-ssi-marching.sh
#   OCCT_INCLUDE=/path/to/opencascade scripts/run-host-native-ssi-marching.sh
#
set -uo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$REPO/build-host-parity"; mkdir -p "$OUT"
CXX="${CXX:-g++}"

OCCT_INCLUDE="${OCCT_INCLUDE:-/usr/include/opencascade}"
[ -d "$OCCT_INCLUDE" ] || {
  echo "OCCT headers not found at $OCCT_INCLUDE."
  echo "Install: sudo apt install libocct-foundation-dev libocct-modeling-data-dev libocct-modeling-algorithms-dev"
  exit 1; }

HARNESS="$REPO/tests/sim/native_ssi_marching_parity.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# ── numsci substrate (linuxhost archive + generated config headers) ──────────────
if [ -z "${CYBERCAD_NUMSCI_DIR:-}" ] && [ ! -d "$REPO/build-numsci/linuxhost/gen" ]; then
  echo "── building numsci linuxhost substrate (scripts/build-numsci.sh linuxhost)"
  "$REPO/scripts/build-numsci.sh" linuxhost
fi

pick_first() { for p in "$@"; do [ -e "$p" ] && { printf '%s\n' "$p"; return 0; }; done; return 0; }

NUMSCI_DIR="${CYBERCAD_NUMSCI_DIR:-$REPO/build-numsci/linuxhost}"
NUMSCI_GEN="$NUMSCI_DIR/gen"
NUMSCI_LIB="$(pick_first "$NUMSCI_DIR"/libnumsci_*.a)"
[ -d "$NUMSCI_GEN" ] || { echo "numsci gen tree not found ($NUMSCI_GEN). Run scripts/build-numsci.sh linuxhost"; exit 1; }
[ -n "$NUMSCI_LIB" ] || { echo "numsci linuxhost archive not found. Run scripts/build-numsci.sh linuxhost"; exit 1; }
NUMPP="${NUMPP_DIR:-/home/leonardo/work/NumPP}/include"
SCIPP="${SCIPP_DIR:-/home/leonardo/work/SciPP}/include"

# ── native TUs to compile (IDENTICAL set to the sim gate) ────────────────────────
NATIVE_SRCS=()
while IFS= read -r src; do NATIVE_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp'; \
      echo "$REPO/src/native/ssi/seeding.cpp"; \
      echo "$REPO/src/native/ssi/marching.cpp"; \
      echo "$REPO/src/native/numerics/numerics.cpp")

# ── OCCT oracle toolkits (same slice/order as the sim harness) ───────────────────
TKS="TKGeomAlgo TKBRep TKG3d TKGeomBase TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling SSI-S3 marching-tracer native-vs-OCCT parity harness for LINUX HOST"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} TU(s) (math + ssi/{seeding,marching} + numerics) [NUMSCI]"
echo "   numsci  : $NUMSCI_LIB"
echo "   oracle  : system OCCT at $OCCT_INCLUDE"
"$CXX" -std=c++20 -O2 -w \
  -DCYBERCAD_HAS_OCCT -DCYBERCAD_HAS_NUMSCI=1 \
  -I"$REPO/src" \
  -I"$OCCT_INCLUDE" \
  -I"$NUMSCI_GEN" -I"$NUMPP" -I"$SCIPP" \
  -x c++ "$HARNESS" "${NATIVE_SRCS[@]}" \
  -x none "$NUMSCI_LIB" \
  $LFLAGS \
  -o "$OUT/native_ssi_marching_parity" || { echo "── COMPILE FAILED"; exit 1; }

echo "── running host parity harness"
"$OUT/native_ssi_marching_parity"
