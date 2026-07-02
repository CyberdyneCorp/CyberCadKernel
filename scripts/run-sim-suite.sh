#!/usr/bin/env bash
# Compile the FULL OCCT runtime suite (full_suite.cpp + every checks_*.cpp module,
# excluding parity_bench.cpp) for the iOS simulator, link it against the
# CyberCadKernel simulator slice + the trimmed OCCT libs, and run it inside a
# booted simulator via `xcrun simctl spawn`. Exercises all 57 cc_* entry points
# plus the determinism/benchmark pass. Companion to run-sim-harness.sh, which
# runs the single-file parity_bench harness.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
LIB="$OUT/libcybercadkernel-SIMULATORARM64.a"
[ -f "$LIB" ] || { echo "build the xcframework first (scripts/build-xcframework.sh)"; exit 1; }
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

# Every sim source EXCEPT the standalone single-file harnesses that carry their
# own competing main() and/or need the GPU/Metal include tree this OCCT-only link
# does not provide. Each of these has its own runner: parity_bench.cpp
# (run-sim-harness.sh), metal_selftest.cpp (run-metal-sim.sh), integ_gpu_tess.cpp
# (run-sim-integ-suite.sh), and the Phase-3 suite — phase3_suite.cpp (its own
# main()) plus its checks_*.cpp modules (which record against phase3_checks.h's
# Ctx, not checks.h's) — has run-sim-phase3-suite.sh. What remains is the OCCT-only
# suite: full_suite.cpp + the Phase-0/1 checks_*.cpp modules (221 assertions).
SKIP="parity_bench.cpp metal_selftest.cpp integ_gpu_tess.cpp \
      phase3_suite.cpp checks_reference_geometry.cpp checks_wrap_emboss.cpp \
      checks_thread_boolean.cpp checks_full_round_fillet.cpp checks_g2_fillet.cpp"
SRCS=()
while IFS= read -r src; do
  case " $SKIP " in *" $(basename "$src") "*) continue;; esac
  SRCS+=("$src")
done < <(find "$REPO/tests/sim" -name '*.cpp' | sort)
[ "${#SRCS[@]}" -gt 0 ] || { echo "no suite sources found under tests/sim"; exit 1; }

TKS="TKDESTEP TKDEIGES TKXSBase TKDE TKMesh TKShHealing TKOffset TKFillet TKBool \
     TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling + linking full suite for iphonesimulator (arm64): ${#SRCS[@]} sources"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios14.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 -I"$REPO/include" -I"$REPO/tests/sim" \
  "${SRCS[@]}" "$LIB" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/full_suite"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/full_suite"
