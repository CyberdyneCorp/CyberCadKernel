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
# (run-sim-integ-suite.sh), the Phase-4 native-math parity harness
# native_math_parity.mm (run-sim-native-math.sh — its own main(), links only the
# geometry-oracle slice of OCCT), the Phase-4 native-topology parity harness
# native_topology_parity.mm (run-sim-native-topology.sh — its own main(), links
# only the topology-oracle slice of OCCT), the Phase-4 native-tessellation parity
# harnesses native_tessellate_parity.mm (run-sim-native-tessellate.sh) and
# native_tessellation_parity.mm (run-sim-native-tessellation.sh — its own main(),
# links only the meshing-oracle slice of OCCT: BRepMesh + BRepGProp, comparing the
# native mesh to the OCCT BRepMesh AND the exact B-rep area/volume), the Phase-4
# native-construction parity harnesses native_construct_parity.mm
# (run-sim-native-construct.sh), native_loft_parity.mm (run-sim-native-loft.sh, Tier B)
# native_sweep_parity.mm (run-sim-native-sweep.sh, Tier C — sweep / pipe-shell) and
# native_thread_parity.mm (run-sim-native-thread.sh, Tier D — threads / tapered shank),
# the Phase-4 native-BOOLEANS (#5) parity harness native_boolean_parity.mm
# (run-sim-native-boolean.sh — its own main(); drives cc_boolean under both engines via
# cc_set_engine, asserting native box fuse/cut/common EXACT vs OCCT + curved/near-
# coincident/disjoint OCCT fall-through, links the whole kernel + full OCCT);
# its own main(); unlike the oracle-slice harnesses it drives the cc_* facade under
# BOTH engines via cc_set_engine and links the whole kernel + full OCCT), the Phase-4
# native-BOOLEANS (#5) deferred-residual-#2 CURVED-slice parity harness
# native_curved_boolean_parity.mm (run-sim-native-curved-boolean.sh — its own main();
# drives cc_boolean under both engines via cc_set_engine, asserting native axis-aligned
# box⟷axis-∥-cylinder cut(through+blind)/fuse(boss)/common vs OCCT at the deflection-
# bounded analytic volume + a NON-axis-aligned cylinder and a sphere-box OCCT
# fall-through, links the whole kernel + full OCCT), the Phase-4
# native-BLENDS (#6) parity harness native_blend_parity.mm (run-sim-native-blend.sh —
# its own main(); drives cc_fillet_edges / cc_chamfer_edges / cc_offset_face / cc_shell
# under both engines via cc_set_engine, asserting native planar chamfer/offset/shell
# EXACT + constant-radius fillet deflection-bounded vs OCCT + a curved-edge fillet OCCT
# fall-through, links the whole kernel + full OCCT), the Phase-4
# native-DATA-EXCHANGE (#7) STEP-EXPORT parity harness native_step_parity.mm
# (run-sim-native-step.sh — its own main(); drives cc_step_export under both engines via
# cc_set_engine, asserting the NATIVE ISO-10303-21 STEP file re-reads through OCCT
# STEPControl_Reader to the SAME solid vs the source (vol/area/centroid + bbox + face/
# edge counts), that the native-written and OCCT-written files re-read to EQUIVALENT
# solids (writer parity), and that a FOREIGN OCCT-built body exported under native FALLS
# BACK to STEPControl_Writer; links the whole kernel + full OCCT and calls OCCT directly
# to re-read the files), and the
# Phase-3 suite — phase3_suite.cpp
# (its own main()) plus its checks_*.cpp modules (which record against
# phase3_checks.h's Ctx, not checks.h's) — has run-sim-phase3-suite.sh. What remains
# is the OCCT-only suite: full_suite.cpp + the Phase-0/1 checks_*.cpp modules (221
# assertions). (native_*_parity.mm are .mm files and already excluded by the *.cpp
# find below; they are listed here to make the intent explicit.)
SKIP="parity_bench.cpp metal_selftest.cpp integ_gpu_tess.cpp native_math_parity.mm \
      native_topology_parity.mm native_tessellate_parity.mm native_tessellation_parity.mm \
      native_construct_parity.mm native_construct_profiles_parity.mm native_loft_parity.mm \
      native_sweep_parity.mm native_thread_parity.mm native_boolean_parity.mm \
      native_curved_boolean_parity.mm \
      native_blend_parity.mm native_step_parity.mm \
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
