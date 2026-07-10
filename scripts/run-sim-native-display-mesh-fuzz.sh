#!/usr/bin/env bash
# Compile + run the MOAT M6 RENDER DISPLAY-MESH DIFFERENTIAL-FUZZING harness
# (tests/sim/native_display_mesh_fuzz.mm) for the iOS simulator. An independent native
# domain on the differential-fuzzing completeness bar, extending the landed M6 fuzzers
# (curved-boolean, STEP round-trip, construction, blend, wrap-emboss, mass-properties,
# geometry-services, transform-chains, reference/datum, direct-modeling, transformed-boolean,
# orthographic-HLR, shape-healing, section, curved-blend, draft-angle, interference,
# freeform-boolean, variable-section sweep, N-sided fill, sheet-metal) to the render-quality
# DISPLAY mesh (src/native/render/display_mesh.h) the CyberCad app drives through the public
# cc_display_mesh facade. That path has a hand-picked host GATE (a) (test_native_display_mesh,
# fixed shapes) but no *fuzz* domain — this closes it.
#
# ── THE ORACLE: NATIVE vs OCCT SOURCE TESSELLATION + CLOSED FORM ───────────────────────────
# cc_display_mesh POST-PROCESSES the ACTIVE engine's correctness tessellation into a shading
# mesh (smooth normals + crease hard edges + optional UVs/LOD). The post-process is engine-
# agnostic; the ONLY difference between the OCCT display mesh (engine 0, BRepMesh source) and
# the native display mesh (engine 1, SolidMesher source) is the source triangulation. Two
# meshers legitimately emit DIFFERENT triangle lists, so the harness does NOT compare triangle
# lists byte-for-byte. It drives cc_display_mesh under BOTH engines (cc_set_engine) over the
# SAME seeded bodies and asserts the INVARIANTS that must hold regardless of the mesher —
# finite / unit-normal / non-degenerate / fold-watertight / UV∈[0,1] — plus the CLOSED-FORM
# deflection bound on the analytic families (sphere/cylinder/cone/box: display-vertex distance
# to the exact surface ≤ 6·deflection, Hausdorff budget under LOD) and native-vs-OCCT bbox
# parity. A "DISAGREE" = native violates a must-hold invariant OCCT holds, OR native breaks
# the closed-form bound — a SILENT WRONG display mesh. An empty/undisplayable body under BOTH
# engines HONEST-DECLINES. The OCCT source mesh breaking the closed-form bound while native
# holds it is ORACLE-INACCURATE (native more correct) — logged, never a bar failure.
#
# We compile the WHOLE kernel (facade + core + engine[native + OCCT adapter] + src/native/**)
# under -DCYBERCAD_HAS_OCCT and link the OCCT toolkit — the OCCT adapter provides the oracle
# source tessellation (engine 0) and create_default_engine. src/native/** stays OCCT-FREE.
#
# The bar: DISAGREED==0 AND every analytic family (sphere/cylinder/cone/box) with >=1 AGREED.
# The FIXED bands (surf<6x-defl, bbox<6x-defl, LOD-hausdorff<8x-defl, tri-ratio<40x) are NEVER
# widened. The generator is seeded ONLY by an explicit FUZZ_SEED (argv/env) — NO clock, NO
# rand(): same seed -> byte-identical batch. On run-sim-suite.sh's SKIP list (own main(),
# std::_Exit).
#
# Usage: scripts/run-sim-native-display-mesh-fuzz.sh [SEED] [N]
#   SEED  explicit uint64 RNG seed. If omitted, runs the TWO default seeds below (the >=2-
#         seed bar proof); the script fails if ANY seed fails. Also honoured via FUZZ_SEED.
#   N     number of generated cases (default 72). Also honoured via FUZZ_N env.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-ios"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"

N="${2:-${FUZZ_N:-72}}"
if [ "${1:-}" != "" ]; then SEEDS=("$1"); elif [ "${FUZZ_SEED:-}" != "" ]; then SEEDS=("$FUZZ_SEED"); else SEEDS=("0xD15B1A57EE" "0x0FF1CE9A11"); fi

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }

HARNESS="$REPO/tests/sim/native_display_mesh_fuzz.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }

# The whole kernel: facade + core + engine (NativeEngine + OCCT adapter + stub) + src/native/**.
KERNEL_SRCS=()
while IFS= read -r src; do KERNEL_SRCS+=("$src"); done \
  < <(find "$REPO/src" -name '*.cpp' | sort)
[ "${#KERNEL_SRCS[@]}" -gt 0 ] || { echo "no kernel sources under src"; exit 1; }

# OCCT toolkit link set (most-derived -> base).
TKS="TKDESTEP TKDEIGES TKXSBase TKDE TKMesh TKHLR TKShHealing TKOffset TKFillet TKBool \
     TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done

echo "── compiling M6 DISPLAY-MESH differential-fuzz harness for iphonesimulator (arm64)"
echo "   harness : $HARNESS"
echo "   kernel  : ${#KERNEL_SRCS[@]} src TU(s) (facade + core + engine[native+occt] + native render/tessellate)"
echo "   engines : cc_set_engine(0)=OCCT source mesh (oracle) vs cc_set_engine(1)=NativeEngine (candidate)"
echo "   arbiter : INVARIANTS (finite/unit-normal/non-degenerate/fold-watertight/UV) + CLOSED-FORM deflection bound + bbox parity"
echo "   seeds   : ${SEEDS[*]}   N : $N"
xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT" \
  -std=c++20 -O2 \
  -DCYBERCAD_HAS_OCCT \
  -I"$REPO/include" \
  -I"$REPO/src" \
  -I"$OCCT/include/opencascade" \
  -x objective-c++ "$HARNESS" \
  -x c++ "${KERNEL_SRCS[@]}" \
  -L"$OCCT/lib" $LFLAGS -lc++ \
  -o "$OUT/native_display_mesh_fuzz"

UDID="$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1 || true)"
if [ -z "$UDID" ]; then
  UDID="$(xcrun simctl list devices available | grep -oE 'iPhone 16 \([0-9A-F-]{36}\)' | grep -oE '[0-9A-F-]{36}' | head -1)"
  echo "── booting simulator $UDID"; xcrun simctl boot "$UDID"; sleep 5
fi

rc=0
for SEED in "${SEEDS[@]}"; do
  echo "── running in simulator $UDID (seed=$SEED N=$N)"
  if ! xcrun simctl spawn "$UDID" "$OUT/native_display_mesh_fuzz" "$SEED" "$N"; then rc=1; fi
done
exit $rc
