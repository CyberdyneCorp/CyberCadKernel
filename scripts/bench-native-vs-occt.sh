#!/usr/bin/env bash
# bench-native-vs-occt.sh — build + run the native-vs-OCCT LATENCY bench on the HOST
# (macOS arm64) against Homebrew OCCT, driving each cc_* op under BOTH engines.
#
# WHY HOST: deterministic CPU timing (no booted-simulator scheduler/thermal noise);
# the native/OCCT RATIO is the portable signal. Absolute device latency differs but is
# not what this measures. See tests/sim/native_vs_occt_bench.cpp header + the findings
# doc docs/BENCH-native-vs-occt.md.
#
# This links the WHOLE kernel — facade + core + engine (NativeEngine + OCCT adapter) +
# src/native/** — against Homebrew OCCT, exactly like the app's macOS desktop dylib
# recipe (scripts/build-macos-dylib.sh / CMake CYBERCAD_MACOS_OCCT), with ONE addition:
# the -lTKHLR toolkit, which occt_drafting.cpp (HLR projection) needs and the desktop
# dylib recipe's TK list omits — this is a MEASUREMENT link only, no product change.
#
# Excludes (matching the macOS recipe): src/compute/metal/* (no -DCYBERCAD_HAS_METAL on
# macOS), src/native/numerics/* (CYBERCAD_HAS_NUMSCI off here — nothing else references
# it), and src/native/mesh/tet_mesher.cpp (AGPL TetGen, optional).
#
# NON-SHIPPING measurement harness. Prints [ROW] lines the doc scrapes + a summary table.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT_PREFIX="${OCCT_PREFIX:-/opt/homebrew/opt/opencascade}"
OCCT_INC="$OCCT_PREFIX/include/opencascade"
OCCT_LIB="$OCCT_PREFIX/lib"
OUT="$REPO/build-bench"; mkdir -p "$OUT"

[ -d "$OCCT_INC" ] || { echo "OCCT headers not found at $OCCT_INC (brew install opencascade)"; exit 1; }

HARNESS="$REPO/tests/sim/native_vs_occt_bench.cpp"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# Kernel sources: all src/**/*.cpp except metal, numerics, tetgen (see header).
KERNEL_SRCS=()
while IFS= read -r src; do KERNEL_SRCS+=("$src"); done < <(
  find "$REPO/src" -name '*.cpp' \
    | grep -vE '/compute/metal/|/native/numerics/|/native/mesh/tet_mesher\.cpp$' \
    | sort)
[ "${#KERNEL_SRCS[@]}" -gt 0 ] || { echo "no kernel sources under src"; exit 1; }

# OCCT toolkit link set — the macOS desktop recipe's list PLUS TKHLR (occt_drafting.cpp).
TKS="TKDESTEP TKDEIGES TKXSBase TKDE TKXCAF TKLCAF TKCAF TKCDF TKMesh TKShHealing \
     TKOffset TKFillet TKBool TKPrim TKBO TKTopAlgo TKGeomAlgo TKHLR TKBRep TKGeomBase \
     TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling native-vs-OCCT bench for host (arm64), Homebrew OCCT at $OCCT_PREFIX"
echo "   kernel: ${#KERNEL_SRCS[@]} src TU(s) + harness"
clang++ -arch arm64 -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/include" -I"$REPO/src" -I"$OCCT_INC" \
  "$HARNESS" "${KERNEL_SRCS[@]}" \
  -L"$OCCT_LIB" $LFLAGS -lc++ \
  -Wl,-rpath,"$OCCT_LIB" \
  -o "$OUT/native_vs_occt_bench"

echo "── running host bench"
"$OUT/native_vs_occt_bench"
