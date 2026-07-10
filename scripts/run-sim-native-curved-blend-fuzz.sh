#!/usr/bin/env bash
# Compile + run the MOAT M6-breadth-15 CURVED-BLEND DIFFERENTIAL-FUZZING harness
# (tests/sim/native_curved_blend_fuzz.mm) for the iOS simulator, inside a booted simulator
# via `xcrun simctl spawn`. This is the FIFTEENTH native domain on the differential-fuzzing
# completeness bar, extending the fourteen landed M6 fuzzers (curved-boolean, STEP round-
# trip, construction, blend fillet/chamfer, wrap-emboss, mass-properties, geometry-services,
# transform-chains, reference/datum, direct-modeling, transformed-boolean, HLR, healing,
# section) to the large NEW M3 CURVED-BLEND native surface landed this session — the analytic
# curved FILLET / SHELL / OFFSET-face paths (src/native/blend/{curved_fillet.h, curved_shell.h,
# curved_offset.h, canal_fillet.h}) reached through the SHIPPING cc_* facade.
#
# The harness DETERMINISTICALLY generates random VALID analytic-revolve base solids (capped
# cylinder / cone frustum / sphere-cap dome) at random valid parameters and drives the nine
# curved-blend families {FILLET, SHELL, OFFSET} x {cyl, cone, sphere} through the PUBLIC
# cc_fillet_edges / cc_shell / cc_offset_face facade under BOTH engines (cc_set_engine) —
# the shipping path the app calls. Each trial is compared native (cc_set_engine(1)) vs the
# OCCT oracle (FILLET/SHELL: OCCT through the facade = BRepFilletAPI / BRepOffsetAPI_
# MakeThickSolid; OFFSET: OCCT built DIRECTLY = BRepPrimAPI, since the shipped OCCT
# cc_offset_face is PLANAR-ONLY and declines a curved wall) AND vs a CLOSED-FORM analytic
# volume (the PRIMARY arbiter), then classified:
#   AGREED            — native valid (watertight, chi=2, correct grow/shrink) + volume within
#                       the deflection-convergence band of BOTH the closed form AND OCCT.
#   HONESTLY-DECLINED — native cc_*->0/invalid (out-of-envelope pose) while OCCT ships → OCCT.
#   DISAGREED         — native valid but OUTSIDE the closed-form truth (SILENT WRONG BLEND).
#   ORACLE-INACCURATE — native matches exact math while OCCT does not (native vindicated).
#   ORACLE_UNRELIABLE — a core-family OCCT oracle that does not match the closed form AND
#                       native also missed (investigate, never launder).
#   BOTH-DECLINED     — an out-of-envelope pose both engines refuse.
# The bar: DISAGREED == 0 AND ORACLE_UNRELIABLE == 0 with each of the nine core families
# having >=1 AGREED trial. The FIXED deflection-convergence bands (volO<2e-2, volX<2e-2,
# area<4e-2) — the same the per-op curved parity harnesses validated — are NEVER widened.
# The generator is seeded ONLY by an explicit FUZZ_SEED (argv/env) — NO clock, NO rand():
# same seed → byte-identical batch (splitmix64 → xoshiro256**).
#
# Like the sibling curved-offset parity harness, this drives the SHIPPING PATH: the public
# cc_* facade, linking the WHOLE kernel + OCCT. src/native / src/engine / include stay BYTE-
# UNCHANGED — this harness is additive test/sim code only. On run-sim-suite.sh's SKIP list
# (own main(), std::_Exit).
#
# Usage: scripts/run-sim-native-curved-blend-fuzz.sh [SEED] [N]
#   SEED  explicit uint64 RNG seed (default 0xB1E7D0FEED). Also honoured via FUZZ_SEED env.
#   N     number of generated cases (default 72).            Also honoured via FUZZ_N env.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

SEED="${1:-${FUZZ_SEED:-0xB1E7D0FEED}}"
N="${2:-${FUZZ_N:-72}}"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_curved_blend_fuzz.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

KERNEL_SRCS=()
while IFS= read -r src; do KERNEL_SRCS+=("$src"); done \
  < <(find "$REPO/src" -name '*.cpp' | sort)
[ "${#KERNEL_SRCS[@]}" -gt 0 ] || { echo "no kernel sources under src"; exit 1; }

# OCCT toolkit link set (BRepPrimAPI + BRepGProp reach TKPrim/TKGeomAlgo; the broad set the
# curved-shell/offset harnesses link, most-derived → base). TKHLR/TKShHealing kept for the
# healing/HLR paths the kernel pulls in.
TKS="TKDESTEP TKDEIGES TKXSBase TKDE TKMesh TKShHealing TKHLR TKOffset TKFillet TKBool \
     TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling M6-breadth-15 CURVED-BLEND differential-fuzz native-vs-OCCT harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   kernel  : ${#KERNEL_SRCS[@]} src TU(s) (facade + core + engine[native+occt] + native math)"
echo "   oracle  : OCCT via facade (BRepFilletAPI / MakeThickSolid) + OCCT direct (BRepPrimAPI) + closed-form"
echo "   seed    : $SEED   N : $N"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios14.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/include" \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  "${KERNEL_SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_curved_blend_fuzz"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID (seed=$SEED N=$N)"
xcrun simctl spawn "$UDID" "$OUT/native_curved_blend_fuzz" "$SEED" "$N"
