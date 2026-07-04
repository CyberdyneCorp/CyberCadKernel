#!/usr/bin/env bash
# Compile the SSI Stage-S5-a (SSI-curve-driven curved boolean) native-vs-OCCT parity
# harness for the iOS simulator, link it against the OCCT boolean oracle (BRepAlgoAPI_
# {Fuse,Cut,Common} + BRepPrimAPI primitives + BRepGProp mass props + BRepCheck) AND the
# NumPP/SciPP numsci archive (the S3 tracer's least_squares corrector + lstsq B-spline
# fit that the S5-a path consumes), and run it inside a booted simulator via
# `xcrun simctl spawn`.
#
# This is the S5-a verification gate 2 — the native-vs-OCCT parity pass
# (native_ssi_curved_boolean_parity.mm). For each of {cyl∩cyl equal (Steinmetz),
# cyl∩cyl unequal (through-drill), sphere∩box, cone∩box} and each op {Fuse,Cut,Common}
# it compares the NATIVE S5 result (nb::boolean_solid → the SSI-curve-driven
# ssi_boolean_solid path) against OCCT BRepAlgoAPI_{Fuse,Cut,Common}: same watertight/
# closed shell, volume within tol, surface area within tol, valid shape. Where the native
# path returns NULL or a candidate the mandatory self-verify rejects (the honest S5-a
# slice boundary — near-tangent Steinmetz, non-watertight drill seam, box operand not a
# curved solid) it asserts the honest fall-back and RECORDS it (never counted as a native
# pass). Gate 1 (host, analytic ground truth, no OCCT) is
# tests/native/test_native_ssi_curved_boolean.cpp.
#
# Like the S3 marching harness, the S5-a path's tracer needs the substrate, so this
# script:
#   * builds the numsci iossim archive first (scripts/build-numsci.sh iossim);
#   * compiles src/native/boolean/ssi_boolean.cpp (the S5-a driver) +
#     src/native/ssi/{seeding.cpp,marching.cpp} (the S3 tracer it consumes) +
#     src/native/numerics/numerics.cpp (the substrate corrector + fit) + the OCCT-free
#     native math TUs (bspline/bezier), all under -DCYBERCAD_HAS_NUMSCI. The topology /
#     tessellate / construct / boolean-header libraries are header-only.
#   * links libnumsci_full_iossim_arm64.a (NumPP + SciPP-optimize/linalg);
#   * links the OCCT oracle toolkits — the SSI marching slice PLUS the boolean/primitive
#     toolkits (TKBO/TKBool/TKPrim/TKTopAlgo/TKShHealing) the BRepAlgoAPI oracle reaches.
#
# S5-a is INTERNAL — NO cc_* entry point; asserted at the cybercad::native::boolean C++
# boundary. On the SKIP list of run-sim-suite.sh (own main(), OCCT+numsci slice).
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_ssi_curved_boolean_parity.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# ── build the numsci iossim substrate archive first ──────────────────────────────
if [ -z "${CYBERCAD_NUMSCI_DIR:-}" ] && [ ! -d "$REPO/build-numsci/iossim/gen" ]; then
  echo "── building numsci iossim substrate (scripts/build-numsci.sh iossim)"
  "$REPO/scripts/build-numsci.sh" iossim
fi

# ── numsci substrate (iossim archive + generated config headers) ─────────────────
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
# S5-a adds src/native/boolean/ssi_boolean.cpp (the SSI-curve-driven driver) alongside
# the S3 tracer (ssi/{seeding,marching}.cpp) + substrate (numerics.cpp) + native math.
NATIVE_SRCS=()
while IFS= read -r src; do NATIVE_SRCS+=("$src"); done \
  < <(find "$REPO/src/native/math" -name '*.cpp'; \
      echo "$REPO/src/native/ssi/seeding.cpp"; \
      echo "$REPO/src/native/ssi/marching.cpp"; \
      echo "$REPO/src/native/numerics/numerics.cpp"; \
      echo "$REPO/src/native/boolean/ssi_boolean.cpp")

# ── OCCT oracle toolkits ─────────────────────────────────────────────────────────
# The SSI marching slice PLUS the boolean/primitive toolkits the BRepAlgoAPI oracle
# reaches: TKBO/TKBool (BOPAlgo + BRepAlgoAPI), TKPrim (BRepPrimAPI box/cyl/sphere/cone),
# TKTopAlgo/TKShHealing (topology algorithms + shape checks), TKGeomAlgo/TKBRep/TKGeom*.
# Ordered most-derived → base.
TKS="TKBool TKBO TKPrim TKShHealing TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling SSI-S5-a curved-boolean native-vs-OCCT parity harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   native  : ${#NATIVE_SRCS[@]} TU(s) (math + ssi/{seeding,marching} + numerics + boolean/ssi_boolean) [NUMSCI]"
echo "   numsci  : $NUMSCI_LIB"
echo "   oracle  : OCCT BRepAlgoAPI_{Fuse,Cut,Common} + BRepPrimAPI + BRepGProp + BRepCheck"
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
  -o "$OUT/native_ssi_curved_boolean_parity"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi
echo "── running in simulator $UDID"
xcrun simctl spawn "$UDID" "$OUT/native_ssi_curved_boolean_parity"
