#!/usr/bin/env bash
# Compile the native cc_thread_apply native-vs-OCCT parity harness for the iOS simulator
# and run it in a booted simulator via `xcrun simctl spawn`.
#
# This is SIM GATE (b) for the native `threadApply` verb (src/native/boolean/thread_apply.h,
# OCCT-FREE): recognise[cylinder] → facet → planar-BSP boolean_solid → four-part self-verify
# (watertight + Euler χ=2 + consistently-oriented + two-sided closed-form-volume band) with
# an honest OCCT fall-through. The harness reconstructs the SAME shaft cylinder + box cutter
# + helical thread in OCCT, runs BRepAlgoAPI_Cut/Fuse + BRepGProp (the ORACLE), and asserts:
#   * the tractable PLANAR-CUTTER baseline (cylinder − box) WELDS through the verb and matches
#     the OCCT cut volume within the deflection band (the BSP machinery is sound); and
#   * the multi-turn HELICAL THREAD native-DECLINES (the near-tangent helical BSP fragments +
#     the orientation-inconsistent native thread operand) while OCCT's per-turn accumulate
#     adds the thread — the correct honest fallthrough (native never emits a leaky/wrong solid).
#
# The verb is INTERNAL — NO cc_* entry point; asserted at the cybercad::native C++ boundary.
# The planar BSP needs the numsci substrate, so this builds the numsci iossim archive first
# and compiles ssi/{seeding,marching}.cpp + numerics.cpp under -DCYBERCAD_HAS_NUMSCI.
#
# Companion to run-sim-suite.sh; on that suite's SKIP list (own main() + whole-slice link).
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_thread_apply_parity.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# ── numsci iossim substrate ──────────────────────────────────────────────────────
if [ -z "${CYBERCAD_NUMSCI_DIR:-}" ] && [ ! -d "$REPO/build-numsci/iossim/gen" ]; then
  echo "── building numsci iossim substrate (scripts/build-numsci.sh iossim)"
  "$REPO/scripts/build-numsci.sh" iossim
fi
pick_first() { for p in "$@"; do [ -e "$p" ] && { printf '%s\n' "$p"; return 0; }; done; return 0; }
NUMSCI_DIR="${CYBERCAD_NUMSCI_DIR:-}"
if [ -n "$NUMSCI_DIR" ] && [ -d "$NUMSCI_DIR/gen" ]; then
  NUMSCI_GEN="$NUMSCI_DIR/gen"; NUMSCI_LIB="$(pick_first "$NUMSCI_DIR"/libnumsci_*.a)"
else
  NUMSCI_GEN="$REPO/build-numsci/iossim/gen"
  NUMSCI_LIB="$(pick_first "$REPO"/build-numsci/iossim/libnumsci_*.a)"
fi
[ -d "$NUMSCI_GEN" ] || { echo "numsci gen tree not found ($NUMSCI_GEN). Run scripts/build-numsci.sh iossim"; exit 1; }
[ -n "$NUMSCI_LIB" ] || { echo "numsci iossim archive not found. Run scripts/build-numsci.sh iossim"; exit 1; }
NUMPP="${NUMPP_DIR:-/Users/leonardoaraujo/work/NumPP}/include"
SCIPP="${SCIPP_DIR:-/Users/leonardoaraujo/work/SciPP}/include"

# ── native TUs (math + ssi/{seeding,marching} + numerics) ────────────────────────
NATIVE_SRCS=()
while IFS= read -r src; do NATIVE_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp'; \
      echo "$REPO/src/native/ssi/seeding.cpp"; \
      echo "$REPO/src/native/ssi/marching.cpp"; \
      echo "$REPO/src/native/boolean/ssi_boolean.cpp"; \
      echo "$REPO/src/native/numerics/numerics.cpp")

# ── OCCT oracle toolkits ─────────────────────────────────────────────────────────
# BRepAlgoAPI_Fuse/Cut → TKBO/TKBool; BRepPrimAPI_MakeCylinder/Box → TKPrim; MakePipeShell
# → TKOffset; BRepBuilderAPI_* / Transform → TKTopAlgo; BRepGProp → TKGeomAlgo; GeomAPI /
# Geom_BSpline → TKGeomBase/TKG3d; TKG2d/TKMath/TKernel underneath; TKShHealing for sewing.
TKS="TKMesh TKShHealing TKOffset TKBool TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling native cc_thread_apply native-vs-OCCT parity harness (iphonesimulator arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} TU(s) (math + ssi/{seeding,marching} + numerics) [NUMSCI]"
echo "   numsci  : $NUMSCI_LIB"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT -DCYBERCAD_HAS_NUMSCI=1 \
  -I"$REPO/src" -I"$REPO/tests" \
  -I"$OCCT/include/opencascade" \
  -I"$NUMSCI_GEN" -I"$NUMPP" -I"$SCIPP" \
  -x objective-c++ "$HARNESS" \
  -x c++ "${NATIVE_SRCS[@]}" \
  -x none "$NUMSCI_LIB" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_thread_apply_parity"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_thread_apply_parity"
