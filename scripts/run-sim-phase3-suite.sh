#!/usr/bin/env bash
#
# run-sim-phase3-suite.sh
#
# Compile + run the Phase-3 FEATURE suite (tests/sim/phase3_suite.cpp + the five
# checks_<feature>.cpp modules) on the iOS simulator against the real OCCT adapter.
#
# Unlike run-sim-suite.sh — which links the prebuilt OCCT-only static slice — this
# suite builds the kernel FROM SOURCE with the OCCT adapter (-DCYBERCAD_HAS_OCCT),
# so the Phase-3 OCCT feature TUs (occt_wrap_emboss.cpp, occt_reference_geometry.cpp,
# occt_thread_boolean.cpp, occt_full_round_fillet.cpp, occt_g2_fillet.cpp) are
# picked up as their feature agents land implementations, with no xcframework
# rebuild step. Metal is OFF here (the Phase-3 features are pure OCCT); the .mm GPU
# modules are excluded from the source set.
#
# It compiles the whole kernel (facade + core + engine incl. the OCCT adapter +
# stub) plus the Phase-3 suite in one invocation, links the trimmed OCCT static
# libs, then runs it inside a booted simulator via `xcrun simctl spawn`. Exit code
# is 0 iff the binary built, ran, and every [PASS]/[FAIL] assertion passed
# (deferred cases do NOT fail the suite — they are honest fallbacks).
#
# Companion to run-sim-suite.sh (Phase-0/1 OCCT suite, 221 assertions).
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OCCT_INC="$OCCT/include/opencascade"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
BIN="$OUT/phase3_suite"
SDK="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT_INC" ] || { echo "missing OCCT headers: $OCCT_INC (set OCCT_ROOT)"; exit 1; }
[ -d "$OCCT/lib" ] || { echo "missing OCCT libs: $OCCT/lib (set OCCT_ROOT)"; exit 1; }

# Whole kernel from source: every src/**.cpp (facade + core + engine incl. the OCCT
# adapter + stub). The Metal .mm modules are intentionally NOT compiled (Metal off).
SRCS=()
while IFS= read -r src; do SRCS+=("$src"); done < <(find "$REPO/src" -name '*.cpp' | sort)

# Phase-3 suite: the driver + its five feature check modules.
SRCS+=("$REPO/tests/sim/phase3_suite.cpp")
SRCS+=("$REPO/tests/sim/checks_reference_geometry.cpp")
SRCS+=("$REPO/tests/sim/checks_wrap_emboss.cpp")
SRCS+=("$REPO/tests/sim/checks_thread_boolean.cpp")
SRCS+=("$REPO/tests/sim/checks_full_round_fillet.cpp")
SRCS+=("$REPO/tests/sim/checks_g2_fillet.cpp")

# Trimmed OCCT link set (same order run-sim-suite.sh uses).
TKS="TKDESTEP TKDEIGES TKXSBase TKDE TKMesh TKShHealing TKOffset TKFillet TKBool \
     TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling + linking Phase-3 feature suite (arm64 iphonesimulator): ${#SRCS[@]} sources"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SDK" \
  -std=c++20 -O2 -DCYBERCAD_HAS_OCCT=1 \
  -I"$REPO/include" -I"$REPO/src" -I"$REPO/tests/sim" -I"$OCCT_INC" \
  "${SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$BIN"

# Autodetect a booted simulator; boot an available one if none is running.
UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-Fa-f-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE '[0-9A-Fa-f-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$BIN"
