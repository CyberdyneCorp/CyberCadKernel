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
# the MOAT M4 import-tail Form-B assembly-instancing harness
# native_step_mapped_item_parity.mm (run-sim-native-step-mapped-item.sh — its own main();
# links the whole kernel + OCCT directly, authors a MAPPED_ITEM / REPRESENTATION_MAP file
# and reads it native-vs-OCCT), and the
# native-DATA-EXCHANGE (#7) STEP-EXPORT parity harness native_step_parity.mm
# (run-sim-native-step.sh — its own main(); drives cc_step_export under both engines via
# cc_set_engine, asserting the NATIVE ISO-10303-21 STEP file re-reads through OCCT
# STEPControl_Reader to the SAME solid vs the source (vol/area/centroid + bbox + face/
# edge counts), that the native-written and OCCT-written files re-read to EQUIVALENT
# solids (writer parity), and that a FOREIGN OCCT-built body exported under native FALLS
# BACK to STEPControl_Writer; links the whole kernel + full OCCT and calls OCCT directly
# to re-read the files), the
# native GEOMETRY-COMPLETION (Tier 1 + Tier 2#4) parity harness
# native_geomcompletion_parity.mm (run-sim-native-geomcompletion.sh — its own main();
# drives the four residual construction areas under both engines via cc_set_engine:
# NATIVE spline extrude / torus revolve / ruled loft / smooth+twisted sweep vs the OCCT
# oracle for mass vol/area/centroid + bbox + watertight tessellate, PLUS one honest OCCT
# fall-through per area — self-crossing spline, spindle torus, mismatched/curved-rail
# loft, self-intersecting sweep + fine-pitch thread), the
# Phase-4 native-NUMERICS (#2, numeric-foundations) closest-point / Extrema parity
# harness native_numerics_parity.mm (run-sim-native-numerics.sh — its own main();
# projects 3D points onto native Plane / Cylinder / Sphere / B-spline surface /
# B-spline curve via src/native/numerics project_point_to_surface/curve and compares
# nearest distance + foot point + parameter to OCCT Extrema through
# GeomAPI_ProjectPointOnSurf / GeomAPI_ProjectPointOnCurve; compiled under
# -DCYBERCAD_HAS_NUMSCI with src/native/numerics/numerics.cpp linking the NumPP/SciPP
# substrate archive + the OCCT geometry-oracle slice, NOT the whole kernel), and the
# SSI Stage-S1 analytic-SSI native-vs-OCCT parity harness native_ssi_parity.mm
# (run-sim-native-ssi.sh — its own main(); builds each supported elementary-surface
# pair natively AND as an OCCT Geom_Surface, compares native intersect_surfaces vs
# GeomAPI_IntSS for curve count/type + densely-sampled on-surface & curve-coincidence
# deltas, and asserts the honest NotAnalytic deferral on skew cylinder∩cylinder while
# OCCT still produces a curve; header-only SSI over src/native/math, links the OCCT
# GeomAPI_IntSS oracle slice, NOT the whole kernel),
# MOAT M-DM DM3 + DM4 — native_dm3_dm4_parity.mm (run-sim-native-dm3-dm4.sh — its own
# main(); drives replaceFaceOffsetTilt vs the OCCT move-face oracle and projectPointOnFace
# vs GeomAPI_ProjectPointOnSurf; header-only DM3/DM4 verbs + the numsci-gated DM2 re-solve
# substrate, links the OCCT oracle slice, NOT the whole kernel).
# Phase-3 suite — phase3_suite.cpp
# (its own main()) plus its checks_*.cpp modules (which record against
# phase3_checks.h's Ctx, not checks.h's) — has run-sim-phase3-suite.sh. What remains
# is the OCCT-only suite: full_suite.cpp + the Phase-0/1 checks_*.cpp modules (221
# assertions). (native_*_parity.mm are .mm files and already excluded by the *.cpp
# find below; they are listed here to make the intent explicit.)
SKIP="parity_bench.cpp metal_selftest.cpp integ_gpu_tess.cpp native_math_parity.mm \
      native_topology_parity.mm native_tessellate_parity.mm native_tessellation_parity.mm \
      native_construct_parity.mm native_construct_profiles_parity.mm native_loft_parity.mm \
      native_sweep_parity.mm native_construct_tails_parity.mm native_thread_parity.mm native_boolean_parity.mm \
      native_curved_boolean_parity.mm \
      native_blend_parity.mm native_step_parity.mm \
      native_geomcompletion_parity.mm native_numerics_parity.mm \
      native_analysis_parity.mm \
      native_reference_parity.mm \
      native_ssi_parity.mm \
      native_ssi_seeding_recall.mm native_ssi_seeding_parity.mm \
      native_ssi_marching_parity.mm native_ssi_curved_boolean_parity.mm \
      native_boolean_fuzz.mm native_step_import_fuzz.mm \
      native_construct_fuzz.mm native_blend_fuzz.mm \
      native_wrap_emboss_fuzz.mm \
      native_mass_props_fuzz.mm native_transform_fuzz.mm \
      native_geometry_services_fuzz.mm \
      native_reference_geometry_fuzz.mm \
      native_directmodel_fuzz.mm \
      native_ssi_s4f_completeness_parity.mm \
      native_first_freeform_boolean_parity.mm \
      native_multi_seam_freeform_boolean_parity.mm \
      native_curved_wall_cut_parity.mm \
      native_walled_bowl_midwall_parity.mm \
      native_chain_seam_parity.mm \
      native_split_plane_parity.mm \
      native_dm3_dm4_parity.mm \
      native_step_mapped_item_parity.mm \
      native_heal_parity.mm \
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
