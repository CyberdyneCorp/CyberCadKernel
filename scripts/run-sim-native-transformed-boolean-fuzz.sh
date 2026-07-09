#!/usr/bin/env bash
# Compile + run the MOAT M6-breadth-11 TRANSFORMED-BOOLEAN DIFFERENTIAL-FUZZING harness
# (tests/sim/native_transformed_boolean_fuzz.mm) for the iOS simulator. This is the
# ELEVENTH native domain on the differential-fuzzing completeness bar, extending the landed
# M6 fuzzers — native_boolean_fuzz.mm, native_step_import_fuzz.mm, native_construct_fuzz.mm,
# native_blend_fuzz.mm, native_wrap_emboss_fuzz.mm, native_mass_props_fuzz.mm,
# native_geometry_services_fuzz.mm, native_transform_fuzz.mm, native_reference_geometry_fuzz.mm
# and native_directmodel_fuzz.mm — to the INTERACTION of a TRANSFORM composed with a BOOLEAN.
#
# native_boolean_fuzz booleans AXIS-ALIGNED operands (identity Location); native_transform_fuzz
# transforms a SINGLE solid. Neither composes the two. But the native planar BSP-CSG boolean
# receives each operand's topology::Shape WITH the solid-level Location a transform baked on it
# (cc_translate_shape / cc_rotate_shape_about / cc_mirror_shape → Shape::located()); the polygon
# extraction the BSP consumes must fold that Location into every face's world polygon + normal.
# A dropped / mis-composed / mis-oriented (mirror handedness) Location yields a boolean on the
# WRONG operand geometry — a silent-wrong no single-domain fuzzer can surface. This closes it.
#
# Like the directmodel/HLR harnesses, this drives the SHIPPING PATH through the public cc_*
# facade under BOTH engines (cc_set_engine(0)=OCCT oracle, cc_set_engine(1)=NativeEngine) and
# arbitrates each trial against a THIRD engine-independent CLOSED-FORM invariant in plain fp64:
# a rigid transform T commutes with a boolean, T(A)∘T(B)==T(A∘B), and preserves volume+area, so
# the TRANSFORMED-boolean's |RT| must equal the untransformed |R0| exactly (native operands are
# all-PLANAR prisms → exact meshes). So it links the WHOLE kernel — facade + core + engine
# (NativeEngine + the OCCT adapter) + src/native/** — plus the full OCCT toolkit set.
#
# We compile every src/**/*.cpp fresh (same set + flags as build-xcframework.sh's slice:
# -std=c++20 -DCYBERCAD_HAS_OCCT -I include -I src -I opencascade) so the harness always tests
# the CURRENT NativeEngine / cc_set_engine sources. No duplicate create_default_engine(): the
# stub guards its definition behind `#ifndef CYBERCAD_HAS_OCCT`, so with OCCT only the OCCT
# adapter provides it.
#
# NUMSCI IS REQUIRED. The kernel's freeform half-space cut / SSI paths the NativeEngine composes
# are NUMSCI-only (guarded by CYBERCAD_HAS_NUMSCI); the planar BSP boolean itself does not need
# it, but the shared NativeEngine TU does not compile without the substrate. So — like the
# sibling cc_*-facade fuzzers — this builds the numsci iossim archive first and compiles the
# whole kernel under -DCYBERCAD_HAS_NUMSCI=1.
#
# Each trial is classified AGREED / HONESTLY-DECLINED (native planar BSP scopes out a near-
# tangent/degenerate composed op → OCCT ships) / BOTH-DECLINED (empty COMMON etc.) /
# ORACLE_UNRELIABLE (OCCT breaks its own rigid invariant while native upholds it — native
# vindicated, gated off) / DISAGREED (a real finding). The bar: DISAGREED==0 AND
# ORACLE_UNRELIABLE==0, each operand family + each op + each transform kind with >=1 AGREED.
# The generator is seeded ONLY by an explicit FUZZ_SEED (argv/env) — NO clock, NO rand(): same
# seed → byte-identical batch. On run-sim-suite.sh's SKIP list (own main(), std::_Exit).
#
# Usage: scripts/run-sim-native-transformed-boolean-fuzz.sh [SEED] [N]
#   SEED  explicit uint64 RNG seed. If omitted, runs the TWO default seeds below (the >=2-
#         seed bar proof); the script fails if ANY seed fails. Also honoured via FUZZ_SEED.
#   N     number of generated cases (default 96). Also honoured via FUZZ_N env.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

N="${2:-${FUZZ_N:-96}}"
if [ "${1:-}" != "" ]; then SEEDS=("$1"); elif [ "${FUZZ_SEED:-}" != "" ]; then SEEDS=("$FUZZ_SEED"); else SEEDS=("0xB007C0DE11" "0x1DEA5EED77"); fi

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_transformed_boolean_fuzz.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# ── numsci iossim substrate (required by the NativeEngine TU) ─────────────────────────
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
  NUMSCI_LIB="$(pick_first "$REPO"/build-numsci/*iossim*.a "$REPO"/build-numsci/iossim/libnumsci_*.a "$REPO"/eval/libnumsci_full_iossim_arm64.a)"
fi
[ -d "$NUMSCI_GEN" ] || { echo "numsci gen tree not found ($NUMSCI_GEN). Run scripts/build-numsci.sh iossim"; exit 1; }
[ -n "$NUMSCI_LIB" ] || { echo "numsci iossim archive not found. Run scripts/build-numsci.sh iossim"; exit 1; }
NUMPP="${NUMPP_DIR:-/Users/leonardoaraujo/work/NumPP}/include"
SCIPP="${SCIPP_DIR:-/Users/leonardoaraujo/work/SciPP}/include"

# The whole kernel: facade + core + engine (NativeEngine + OCCT adapter + the always-
# compiled stub, which no-ops its create_default_engine under OCCT) + src/native/**. Same
# file set build-xcframework.sh compiles into the slice.
KERNEL_SRCS=()
while IFS= read -r src; do KERNEL_SRCS+=("$src"); done \
  < <(find "$REPO/src" -name '*.cpp' | sort)
[ "${#KERNEL_SRCS[@]}" -gt 0 ] || { echo "no kernel sources under src"; exit 1; }

# OCCT toolkit link set — the harness exercises the OCCT adapter through the facade
# (construct prism, rigid transforms, BRepAlgoAPI booleans, meshing, mass queries), so
# link the SAME broad toolkit set the sibling cc_*-facade fuzzers use (most-derived →
# base). TKHLR/TKShHealing keep the always-linked drafting/healing adapters satisfied.
TKS="TKHLR TKDESTEP TKDEIGES TKXSBase TKDE TKMesh TKShHealing TKOffset TKFillet TKBool \
     TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling M6-breadth-11 TRANSFORMED-BOOLEAN differential-fuzz harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   kernel  : ${#KERNEL_SRCS[@]} src TU(s) (facade + core + engine[native+occt] + native math)"
echo "   engines : cc_set_engine(0)=OCCT oracle / cc_set_engine(1)=NativeEngine"
echo "   seeds   : ${SEEDS[*]}   N : $N"
echo "   numsci  : $NUMSCI_LIB"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT -DCYBERCAD_HAS_NUMSCI=1 \
  -I"$REPO/include" \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -I"$NUMSCI_GEN" -I"$NUMPP" -I"$SCIPP" \
  -x objective-c++ "$HARNESS" \
  -x c++ "${KERNEL_SRCS[@]}" \
  -x none "$NUMSCI_LIB" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_transformed_boolean_fuzz"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi

rc=0
for SEED in "${SEEDS[@]}"; do
  echo "── running in simulator $UDID (seed=$SEED N=$N)"
  if ! xcrun simctl spawn "$UDID" "$OUT/native_transformed_boolean_fuzz" "$SEED" "$N"; then rc=1; fi
done
exit $rc
