#!/usr/bin/env bash
# Compile the MOAT feature DRAFT ANGLE native-vs-OCCT parity harness for the iOS
# simulator and run it in a booted simulator via `xcrun simctl spawn`.
#
# This is SIM GATE (b) of the two-gate model. The native draft
# (src/native/feature/draft_faces.h, OCCT-FREE) derives each drafted plane from the
# original face geometry (pivot on its trace with the neutral plane) and applies it as an
# inward split-plane trim, then re-audits the composite (watertight / χ=2 / oriented /
# shrink). The harness reconstructs the SAME box in OCCT and drafts it with the ORACLE
# BRepOffsetAPI_DraftAngle, then compares the native result (measured by the native M0
# tessellator) vs OCCT (BRepGProp) on volume, area, watertightness, topology (Euler χ),
# bbox, and one-sided Hausdorff — with FIXED, curved-tessellation-bounded tolerances
# (never widened). Gate (a) (host, no OCCT) is tests/native/test_native_draft_faces.cpp.
#
# The draft's inward trims reuse splitByPlane's tilted probe, which traces the numsci-only
# seam, so this script builds the numsci iossim archive first and compiles
# ssi/{seeding,marching}.cpp + numerics.cpp + ssi_boolean.cpp under -DCYBERCAD_HAS_NUMSCI,
# then links the OCCT oracle slice (BRepOffsetAPI_DraftAngle → TKOffset/TKFillet, box +
# BRepGProp + BRepBndLib + BRepExtrema).
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_draft_faces_parity.mm"
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
  NUMSCI_LIB="$(pick_first "$REPO"/build-numsci/*iossim*.a "$REPO"/eval/libnumsci_full_iossim_arm64.a)"
fi
[ -d "$NUMSCI_GEN" ] || { echo "numsci gen tree not found ($NUMSCI_GEN). Run scripts/build-numsci.sh iossim"; exit 1; }
[ -n "$NUMSCI_LIB" ] || { echo "numsci iossim archive not found. Run scripts/build-numsci.sh iossim"; exit 1; }
NUMPP="${NUMPP_DIR:-/Users/leonardoaraujo/work/NumPP}/include"
SCIPP="${SCIPP_DIR:-/Users/leonardoaraujo/work/SciPP}/include"

# ── native TUs (math + ssi/{seeding,marching} + numerics + ssi_boolean) ──────────
# splitByPlane's analytic branch calls boolean_solid → ssi_boolean_solid, whose
# definition lives in src/native/boolean/ssi_boolean.cpp.
NATIVE_SRCS=()
while IFS= read -r src; do NATIVE_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp'; \
      echo "$REPO/src/native/ssi/seeding.cpp"; \
      echo "$REPO/src/native/ssi/marching.cpp"; \
      echo "$REPO/src/native/boolean/ssi_boolean.cpp"; \
      echo "$REPO/src/native/numerics/numerics.cpp")

# ── OCCT oracle toolkits ─────────────────────────────────────────────────────────
# BRepOffsetAPI_DraftAngle → TKOffset (+ TKFillet locally-offset chain); BRepPrimAPI_MakeBox
# → TKPrim; BRepBuilderAPI_* / BRepExtrema → TKTopAlgo; BRepGProp → TKGeomAlgo;
# gp_*/GProp/Bnd → TKMath; TKernel underneath. TKShHealing/TKBool/TKBO are pulled by the
# draft's internal offset/boolean. Ordering: most-derived → base.
TKS="TKMesh TKShHealing TKOffset TKFillet TKBool TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling draft-angle native-vs-OCCT parity harness (iphonesimulator arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} TU(s) (math + ssi/{seeding,marching} + numerics + ssi_boolean) [NUMSCI]"
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
  -o "$OUT/native_draft_faces_parity"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_draft_faces_parity"
