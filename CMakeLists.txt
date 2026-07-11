cmake_minimum_required(VERSION 3.20)

# The host build uses Homebrew LLVM clang++ for full C++20 (coroutines,
# <stop_token>, <jthread>, <barrier>, <latch>). Override with -DCMAKE_CXX_COMPILER=...
# for other toolchains (e.g. the iOS cross-compile). See docs/build.md.
project(CyberCadKernel LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# OFF => host-buildable, no OCCT (stub engine). ON => compile the OCCT adapter
# (iOS only; needs a trimmed OCCT — NOT host-buildable).
option(CYBERCAD_HAS_OCCT "Build the OCCT engine adapter" OFF)
option(CYBERCAD_BUILD_TESTS "Build the CTest unit tests" ON)

# OFF => no GPU backend (host build stays CPU-only). ON => compile the Metal
# compute backend under src/compute/metal/*.mm (iOS only; needs -framework Metal
# -framework Foundation and an Objective-C++ toolchain). Never enabled for the
# host CPU-only build.
option(CYBERCAD_HAS_METAL "Build the Metal compute backend (iOS only)" OFF)

# OFF => the numeric facade (src/native/numerics/*) is EXCLUDED and NumPP/SciPP
# are not referenced — the rest of the kernel builds exactly as before (Phase-4
# #2 is additive/guarded). ON => compile src/native/numerics on top of the
# NumPP + SciPP-optimize/linalg substrate (OCCT-free), define CYBERCAD_HAS_NUMSCI,
# add the substrate include trees and link libnumsci_<target>.a. Build the
# archive first with scripts/build-numsci.sh {host|iossim}, then point
# CYBERCAD_NUMSCI_DIR at its output. NumPP/SciPP are referenced by ABSOLUTE PATH
# (like OCCT), never vendored. See docs/EVAL-numpp-scipp.md.
option(CYBERCAD_HAS_NUMSCI "Build the src/native/numerics facade on NumPP + SciPP" OFF)

# Output dir of scripts/build-numsci.sh: holds gen/ (config headers) +
# libnumsci_<target>.a. Only used when CYBERCAD_HAS_NUMSCI=ON.
set(CYBERCAD_NUMSCI_DIR "" CACHE PATH "build-numsci.sh output dir (gen/ + libnumsci_*.a)")
# NumPP / SciPP source roots (absolute, like the OCCT path). Their include/ trees
# are added when CYBERCAD_HAS_NUMSCI=ON.
set(CYBERCAD_NUMPP_DIR "/Users/leonardoaraujo/work/NumPP"
    CACHE PATH "NumPP source root (referenced by absolute path, not vendored)")
set(CYBERCAD_SCIPP_DIR "/Users/leonardoaraujo/work/SciPP"
    CACHE PATH "SciPP source root (referenced by absolute path, not vendored)")

# OFF => the tet mesher (src/native/mesh/tet_mesher.cpp) is EXCLUDED and NO AGPL
# TetGen code is compiled or linked — the default MIT build is unaffected and
# cc_tet_mesh returns a clean "unavailable" error. ON => compile ONLY that one TU
# against the external TetGen headers, define CYBERCAD_HAS_TETGEN and link the
# libtetgen_<target>.a static archive built by scripts/build-tetgen.sh. TetGen is
# OPTIONAL, EXTERNAL (referenced by absolute path, NEVER vendored) and AGPL-3.0;
# shipping a closed app that links it requires a TetGen commercial license. The
# always-on native mesh-quality module (src/native/mesh/quality.cpp) has NO TetGen
# dependency and is compiled in every configuration.
option(CYBERCAD_HAS_TETGEN "Build the src/native/mesh tet mesher on TetGen (AGPL, optional)" OFF)
# Output dir of scripts/build-tetgen.sh: holds libtetgen_<target>.a. Only used when
# CYBERCAD_HAS_TETGEN=ON.
set(CYBERCAD_TETGEN_DIR "" CACHE PATH "build-tetgen.sh output dir (libtetgen_*.a)")
# TetGen source root (its headers are referenced by ABSOLUTE PATH — like OCCT and
# NumPP/SciPP — and never vendored). Scoped to tet_mesher.cpp only.
set(CYBERCAD_TETGEN_SRC_DIR "/home/leonardo/work/tetgen"
    CACHE PATH "TetGen source root (headers referenced by absolute path, not vendored)")

# ── M8 scoped-unlink DRY-RUN rehearsal (NON-SHIPPING measurement artifact) ─────
# OFF (default) => unchanged: the shipping engine selection is untouched. In the
# host build create_default_engine() returns the stub; a caller opts in to the
# native engine via cc_set_engine(1).
# ON  => a THROWAWAY measurement configuration for MOAT M8. It changes ONLY the
# build's DEFAULT active engine: create_default_engine() returns a NativeEngine
# whose sole fallback is the STUB (never OcctEngine), so EVERY cc_* op — even in
# a test that never calls cc_set_engine(1) — exercises the exact post-unlink
# native-only + stub-fallback path (native serve, else the stub's honest
# engine_unsupported decline). This rehearses `drop-occt` WITHOUT deleting
# src/engine/occt and WITHOUT changing any shipped default. It is mutually
# exclusive with CYBERCAD_HAS_OCCT (the rehearsal is the no-OCCT world) and must
# NEVER be enabled in a shipped configuration. See
# openspec/changes/moat-m8dry-unlink-rehearsal/ and openspec/DROP-OCCT-READINESS.md §6.
option(CYBERCAD_M8_REHEARSAL "NON-SHIPPING: default engine = NativeEngine over stub (M8 unlink rehearsal)" OFF)

# OFF => unchanged (host stub or iOS cross-compile driven by the options above).
# ON  => the PROVEN macOS desktop build: a SHARED libcybercadkernel.dylib built
# from the same sources as the OCCT build (facade + core + engine + engine/occt),
# EXCLUDING src/compute/metal/* (Metal is off on macOS; cc_set_gpu_tessellation is
# a safe no-op), compiled -DCYBERCAD_HAS_OCCT for arm64 against Homebrew OCCT
# (v7.9.3) and linked to the TK* libraries with an rpath. This codifies the manual
# recipe in scripts/build-macos-dylib.sh. It forces CYBERCAD_HAS_OCCT=ON, points
# the OCCT include dir at the Homebrew prefix, and adds the link libraries the iOS
# path leaves to its own toolchain. The host (stub) and iOS configs are unchanged
# when this is OFF.
option(CYBERCAD_MACOS_OCCT "Build the macOS desktop OCCT dylib (Homebrew OCCT)" OFF)

# Homebrew OpenCASCADE prefix used only when CYBERCAD_MACOS_OCCT=ON. Override for a
# non-default Homebrew location (e.g. Intel /usr/local/opt/opencascade).
set(CYBERCAD_HOMEBREW_OCCT_PREFIX "/opt/homebrew/opt/opencascade"
    CACHE PATH "Homebrew OpenCASCADE prefix for the macOS desktop dylib")

if(CYBERCAD_MACOS_OCCT)
    # The macOS desktop build compiles the OCCT adapter against Homebrew headers
    # and links a real shared library, so force the OCCT option on and aim the
    # adapter's include dir at the Homebrew prefix (before the library is defined).
    set(CYBERCAD_HAS_OCCT ON CACHE BOOL "" FORCE)
    set(CYBERCAD_OCCT_INCLUDE_DIR
        "${CYBERCAD_HOMEBREW_OCCT_PREFIX}/include/opencascade" CACHE PATH "" FORCE)
    set(BUILD_SHARED_LIBS ON)
endif()

# Include dir for the trimmed arm64-iOS OCCT the adapter compiles against. Only
# used when CYBERCAD_HAS_OCCT=ON (iOS cross-compile); override for a different
# OCCT install. See docs/occt-build.md in the app repo.
set(CYBERCAD_OCCT_INCLUDE_DIR
    "/Users/leonardoaraujo/work/cybercad/build/occt/install-SIMULATORARM64/include/opencascade"
    CACHE PATH "OCCT headers for the OCCT engine adapter (iOS only)")

# ── library sources ───────────────────────────────────────────────────────────
# All src/**.cpp EXCEPT src/engine/occt/* (added only when CYBERCAD_HAS_OCCT=ON)
# and src/native/numerics/* (added only when CYBERCAD_HAS_NUMSCI=ON — that module
# is the sole consumer of NumPP/SciPP and must stay out of the default build).
file(GLOB_RECURSE CYBERCAD_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
list(FILTER CYBERCAD_SOURCES EXCLUDE REGEX "/src/engine/occt/")
list(FILTER CYBERCAD_SOURCES EXCLUDE REGEX "/src/native/numerics/")
# src/native/mesh/tet_mesher.cpp is the SOLE TetGen (AGPL) consumer — excluded from
# the default MIT build glob, added back only under CYBERCAD_HAS_TETGEN below.
# quality.cpp and any other src/native/mesh/*.cpp stay in the default glob
# (always-on, TetGen-free). Only the $-anchored basename is filtered.
list(FILTER CYBERCAD_SOURCES EXCLUDE REGEX "/src/native/mesh/tet_mesher\\.cpp$")

# Static everywhere (host stub, iOS) EXCEPT the macOS desktop build, which links a
# real shared libcybercadkernel.dylib the Python/ctypes binding loads.
if(CYBERCAD_MACOS_OCCT)
    add_library(cybercadkernel SHARED ${CYBERCAD_SOURCES})
else()
    add_library(cybercadkernel STATIC ${CYBERCAD_SOURCES})
endif()

target_include_directories(cybercadkernel
    PUBLIC  "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")

find_package(Threads REQUIRED)
target_link_libraries(cybercadkernel PUBLIC Threads::Threads)

# M8 rehearsal is the no-OCCT native-default world: it must never be combined
# with the OCCT adapter (that would defeat the "what breaks without OCCT" measure).
if(CYBERCAD_M8_REHEARSAL AND CYBERCAD_HAS_OCCT)
    message(FATAL_ERROR
        "CYBERCAD_M8_REHEARSAL is a NON-SHIPPING no-OCCT measurement config and is "
        "mutually exclusive with CYBERCAD_HAS_OCCT. Enable exactly one.")
endif()
if(CYBERCAD_M8_REHEARSAL)
    target_compile_definitions(cybercadkernel PRIVATE CYBERCAD_M8_REHEARSAL=1)
endif()

if(CYBERCAD_HAS_OCCT)
    file(GLOB CYBERCAD_OCCT_SOURCES CONFIGURE_DEPENDS
         "${CMAKE_CURRENT_SOURCE_DIR}/src/engine/occt/*.cpp")
    target_sources(cybercadkernel PRIVATE ${CYBERCAD_OCCT_SOURCES})
    target_compile_definitions(cybercadkernel PRIVATE CYBERCAD_HAS_OCCT=1)
    target_include_directories(cybercadkernel PRIVATE "${CYBERCAD_OCCT_INCLUDE_DIR}")
    # OCCT link libs are supplied by the iOS toolchain (see docs/occt-build.md in
    # the app repo); not resolved for the host build.
endif()

# ── macOS desktop OCCT dylib (Homebrew) ───────────────────────────────────────
# Codifies the proven manual recipe: arm64, C++20, -DCYBERCAD_HAS_OCCT (already
# forced above), Homebrew OCCT headers (added via CYBERCAD_OCCT_INCLUDE_DIR in the
# OCCT block), and the exact TK* link list against the Homebrew lib dir with an
# rpath so the dylib finds OCCT at load time. Metal stays excluded on macOS.
if(CYBERCAD_MACOS_OCCT)
    set(CYBERCAD_HOMEBREW_OCCT_LIB "${CYBERCAD_HOMEBREW_OCCT_PREFIX}/lib")

    target_compile_options(cybercadkernel PRIVATE -arch arm64 -O2)
    # SHARED library => -fPIC is implied; kept explicit to mirror the recipe.
    set_target_properties(cybercadkernel PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        OSX_ARCHITECTURES arm64)

    # The exact TK* order from the proven recipe. -lTKHLR is required by
    # occt_drafting.cpp (HLRBRep_* hidden-line removal): Homebrew OCCT 7.9.x split
    # the HLR algorithms out of TKTopAlgo into their own TKHLR library, so it must
    # be listed explicitly or the dylib link fails with unresolved HLRBRep_* symbols.
    target_link_libraries(cybercadkernel PRIVATE
        -L"${CYBERCAD_HOMEBREW_OCCT_LIB}"
        -lTKDESTEP -lTKDEIGES -lTKXSBase -lTKDE -lTKXCAF -lTKLCAF -lTKCAF
        -lTKCDF -lTKMesh -lTKShHealing -lTKOffset -lTKFillet -lTKBool -lTKPrim
        -lTKHLR -lTKBO -lTKTopAlgo -lTKGeomAlgo -lTKBRep -lTKGeomBase -lTKG3d -lTKG2d
        -lTKMath -lTKernel
        -Wl,-rpath,"${CYBERCAD_HOMEBREW_OCCT_LIB}")
endif()

# ── numeric facade on NumPP + SciPP (guarded) ─────────────────────────────────
# Additive: only src/native/numerics/*.cpp is compiled (with CYBERCAD_HAS_NUMSCI
# defined) and only this module sees NumPP/SciPP headers + the static archive.
# The rest of the kernel is untouched, so every existing suite is unaffected.
if(CYBERCAD_HAS_NUMSCI)
    file(GLOB CYBERCAD_NUMSCI_SOURCES CONFIGURE_DEPENDS
         "${CMAKE_CURRENT_SOURCE_DIR}/src/native/numerics/*.cpp")
    target_sources(cybercadkernel PRIVATE ${CYBERCAD_NUMSCI_SOURCES})
    target_compile_definitions(cybercadkernel PRIVATE CYBERCAD_HAS_NUMSCI=1)
    # Match the archive's -fvisibility=hidden (build-numsci.sh) so the substrate's
    # weak symbols (e.g. numpp::kDefaultComplex guard var) don't trip the "weak
    # symbol cannot be overridden" link warning from a visibility mismatch.
    set_source_files_properties(${CYBERCAD_NUMSCI_SOURCES} PROPERTIES
        COMPILE_OPTIONS "-fvisibility=hidden;-fvisibility-inlines-hidden")

    # Substrate include trees (referenced by absolute path, like OCCT) + the
    # generated backends-OFF config headers produced by scripts/build-numsci.sh.
    if(NOT CYBERCAD_NUMSCI_DIR)
        message(FATAL_ERROR
            "CYBERCAD_HAS_NUMSCI=ON requires -DCYBERCAD_NUMSCI_DIR=<build-numsci output>. "
            "Run scripts/build-numsci.sh {host|iossim} first.")
    endif()
    target_include_directories(cybercadkernel PRIVATE
        "${CYBERCAD_NUMPP_DIR}/include"
        "${CYBERCAD_SCIPP_DIR}/include"
        "${CYBERCAD_NUMSCI_DIR}/gen")

    # Link libnumsci_<target>.a (host or iOS-sim archive from build-numsci.sh).
    file(GLOB CYBERCAD_NUMSCI_ARCHIVE
         "${CYBERCAD_NUMSCI_DIR}/libnumsci_*.a")
    if(NOT CYBERCAD_NUMSCI_ARCHIVE)
        message(FATAL_ERROR
            "No libnumsci_*.a in ${CYBERCAD_NUMSCI_DIR}. Run scripts/build-numsci.sh first.")
    endif()
    target_link_libraries(cybercadkernel PUBLIC ${CYBERCAD_NUMSCI_ARCHIVE})
endif()

# ── tet mesher on TetGen (guarded, AGPL) ──────────────────────────────────────
# Additive AGPL fence: ONLY src/native/mesh/tet_mesher.cpp is compiled (with the
# TetGen headers + -DTETLIBRARY scoped to that single source via
# set_source_files_properties), and ONLY this module links libtetgen. No other TU
# can include <tetgen.h>. quality.cpp (always-on) is untouched. TetGen sources are
# referenced by absolute path and never vendored.
if(CYBERCAD_HAS_TETGEN)
    target_sources(cybercadkernel PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src/native/mesh/tet_mesher.cpp")
    # Per-source AGPL scoping: the TetGen include dir + defines reach ONLY this TU.
    set_source_files_properties(
        "${CMAKE_CURRENT_SOURCE_DIR}/src/native/mesh/tet_mesher.cpp"
        PROPERTIES COMPILE_DEFINITIONS "CYBERCAD_HAS_TETGEN=1;TETLIBRARY")
    set_property(SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/src/native/mesh/tet_mesher.cpp"
        APPEND PROPERTY INCLUDE_DIRECTORIES "${CYBERCAD_TETGEN_SRC_DIR}")
    # The facade TU needs CYBERCAD_HAS_TETGEN too, so its #ifdef call-site gating
    # compiles the real path. The facade never includes tetgen.h (TETLIBRARY stays
    # per-source above), it only chooses which native symbol to call.
    target_compile_definitions(cybercadkernel PRIVATE CYBERCAD_HAS_TETGEN=1)

    if(NOT CYBERCAD_TETGEN_DIR)
        message(FATAL_ERROR
            "CYBERCAD_HAS_TETGEN=ON requires -DCYBERCAD_TETGEN_DIR=<build-tetgen output>. "
            "Run scripts/build-tetgen.sh first.")
    endif()
    file(GLOB CYBERCAD_TETGEN_ARCHIVE "${CYBERCAD_TETGEN_DIR}/libtetgen_*.a")
    if(NOT CYBERCAD_TETGEN_ARCHIVE)
        message(FATAL_ERROR
            "No libtetgen_*.a in ${CYBERCAD_TETGEN_DIR}. Run scripts/build-tetgen.sh first.")
    endif()
    target_link_libraries(cybercadkernel PUBLIC ${CYBERCAD_TETGEN_ARCHIVE})
endif()

# ── Metal compute backend (iOS only) ──────────────────────────────────────────
# Compile the Objective-C++ Metal backend ONLY when CYBERCAD_HAS_METAL=ON. The
# host build (OFF) never sees these .mm files and stays CPU-only. The GPU modules
# themselves are self-contained fp32 numeric code and MUST NOT link OCCT.
#
# When CYBERCAD_HAS_METAL and CYBERCAD_HAS_OCCT are BOTH ON (the iOS GPU build),
# the OCCT adapter and the Metal GPU modules end up in the SAME `cybercadkernel`
# target, so the OCCT-side GPU tessellation TU (src/engine/occt/occt_gpu_tessellate.cpp)
# links the surface-eval module directly. The target-wide CYBERCAD_HAS_METAL
# define below reaches every TU (including the OCCT .cpp files), which is what
# switches the per-face GPU path on inside occt_tessellate.cpp / occt_engine.cpp.
# With either option OFF the host build is unchanged (no .mm, no define, OCCT-only
# tessellation).
if(CYBERCAD_HAS_METAL)
    enable_language(OBJCXX)
    file(GLOB CYBERCAD_METAL_SOURCES CONFIGURE_DEPENDS
         "${CMAKE_CURRENT_SOURCE_DIR}/src/compute/metal/*.mm")
    target_sources(cybercadkernel PRIVATE ${CYBERCAD_METAL_SOURCES})
    target_compile_definitions(cybercadkernel PRIVATE CYBERCAD_HAS_METAL=1)
    set_source_files_properties(${CYBERCAD_METAL_SOURCES} PROPERTIES
        LANGUAGE OBJCXX
        COMPILE_OPTIONS "-fobjc-arc")
    target_link_libraries(cybercadkernel PUBLIC
        "-framework Metal" "-framework Foundation")
endif()

# ── tests ─────────────────────────────────────────────────────────────────────
if(CYBERCAD_BUILD_TESTS)
    enable_testing()

    # Path to the app's reference ABI header, so the contract test can compare
    # struct layouts against the real source of truth. Overridable.
    set(CYBERCAD_REFERENCE_ABI_HEADER
        "/Users/leonardoaraujo/work/cybercad/CyberCad/Kernel/Bridge/KernelBridgeAPI.h"
        CACHE FILEPATH "Reference KernelBridgeAPI.h for the ABI contract test")

    set(CYBERCAD_TESTS
        test_registry
        test_guard
        test_scheduler
        test_compute_backend
        test_parallel_policy
        test_parallel_toggle
        test_abi
        test_cross_engine_guard
        test_native_math
        test_native_nurbs_ops
        test_native_nurbs_ops_fuzz
        test_native_trimmed_nurbs
        test_native_topology
        test_native_tessellate
        test_native_display_mesh
        test_native_display_mesh_fuzz
        test_native_construct
        test_native_profile
        test_native_residuals
        test_native_loft
        test_native_sweep
        test_native_vsweep
        test_native_thread
        test_native_boolean
        test_native_dense_csg_stress
        test_native_curved_boolean
        test_native_freeform_membership
        test_native_freeform_operand
        test_native_ssi
        test_native_ssi_s4_classification
        test_native_heal
        test_native_blend
        test_native_analytic_fillet
        test_native_curved_shell
        test_native_fillet_corner
        test_native_corner_chamfer
        test_native_chamfer_face
        test_native_wrap_emboss
        test_native_sheetmetal
        test_native_surfacing
        test_native_step_writer
        test_native_step_reader
        test_native_step_pmi
        test_native_stl
        test_native_gltf
        test_native_gltf_fuzz
        test_native_quality
        test_native_drafting
        test_native_section
        test_native_reference
        test_native_step
        test_native_inertia
        test_native_validity
        test_native_interference
        test_native_project
        test_native_solid_enum
        test_native_engine)

    # The numeric-facade test links NumPP/SciPP, so it is added ONLY when the
    # substrate is built in (CYBERCAD_HAS_NUMSCI=ON). With the option OFF the
    # existing suite is unchanged.
    if(CYBERCAD_HAS_NUMSCI)
        list(APPEND CYBERCAD_TESTS test_native_numerics)
        set(test_native_numerics_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_numerics.cpp")
        # NURBS roadmap Layer 7 — fitting / approximation (src/native/math/
        # bspline_fit). The fit routines solve linear systems through the numsci
        # facade (lin_solve / lstsq), so the module + its host gate build only under
        # CYBERCAD_HAS_NUMSCI (mirroring test_native_ssi_seeding). Airtight oracles:
        # interpolation exactness (pass-through ~1e-9), round-trip recovery of a KNOWN
        # B-spline (~1e-8), monotone approximation-error convergence, parametrization
        # sanity + honest degenerate guards. Also RATIONAL interpolation with PRESCRIBED
        # weights (homogeneous lift): Euclidean pass-through ~1e-9, KNOWN rational
        # circle/half-cylinder recovered POINTWISE, input weights recovered, non-positive/
        # mismatched-weight guards. Weight ESTIMATION (Ma–Kruth) remains a residual.
        list(APPEND CYBERCAD_TESTS test_native_nurbs_fit)
        set(test_native_nurbs_fit_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_nurbs_fit.cpp")
        # NURBS roadmap Layer 6 — skinning / lofting (src/native/math/bspline_skin).
        # Composes Layer 1 (elevate/refine for section compatibility) + Layer 7 (curve
        # interpolation across sections via lin_solve), so the module + its host gate
        # build only under CYBERCAD_HAS_NUMSCI (mirroring test_native_nurbs_fit). Airtight
        # oracles: section containment (surface iso-curve at v_k == section k pointwise
        # ~1e-8), compatibility correctness (compat sections equal originals exactly),
        # KNOWN-surface round-trip, and honest degenerate/rational guards. Non-rational
        # scope; rational skinning + general Gordon/network/exact-sweep are residuals.
        list(APPEND CYBERCAD_TESTS test_native_nurbs_skin)
        set(test_native_nurbs_skin_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_nurbs_skin.cpp")
        # NURBS roadmap Layer 6 — swept surfaces (src/native/math/bspline_sweep). A section
        # curve swept along a trajectory: translational sweep is the EXACT closed-form
        # tensor product (section × degree-1 path); the general sweep places the section at
        # K stations via a rotation-minimizing moving frame and SKINS them (composes
        # bspline_skin, hence lin_solve — under CYBERCAD_HAS_NUMSCI, mirroring the skin
        # gate). Airtight oracles: translational exactness (S(.,v) == section+v*sweep
        # pointwise ~1e-12), station containment (surface contains each transformed section
        # ~1e-8), known ruled/cylinder-like patches vs closed-form, anti-twist frame sanity,
        # and honest degenerate/rational guards. Non-rational scope; rotational/exact-
        # BRepFill/variable-section sweeps are residuals.
        list(APPEND CYBERCAD_TESTS test_native_nurbs_sweep)
        set(test_native_nurbs_sweep_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_nurbs_sweep.cpp")
        # NURBS roadmap Layer 6 — GORDON / NETWORK surface (src/native/math/bspline_gordon).
        # A surface through a NETWORK of K u-curves and L v-curves that intersect at a K×L
        # grid, built as the boolean sum G = S_u ⊕ S_v ⊖ T (skin of u-curves + skin of
        # v-curves − tensor interpolant of the grid, §10.5). Composes bspline_skin +
        # surface interpolation, hence lin_solve — under CYBERCAD_HAS_NUMSCI, mirroring the
        # skin/sweep gates. Airtight oracles: network containment (surface contains every u-
        # and v-curve pointwise ~1e-8), grid intersection on-surface (~1e-8), known-surface
        # round-trip (extract a u/v iso-curve network from a KNOWN tensor surface → Gordon →
        # recover the original pointwise ~1e-8), and honest inconsistent/degenerate/rational
        # declines. Non-rational scope; rational + irregular/N-sided networks + exact
        # BRepFill residual.
        list(APPEND CYBERCAD_TESTS test_native_nurbs_gordon)
        set(test_native_nurbs_gordon_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_nurbs_gordon.cpp")
        # NURBS roadmap Layer 5 — surface OFFSET (src/native/math/bspline_offset).
        # Samples the true offset locus S + d·N and fits a non-rational NURBS surface
        # through it with the Layer-7 fitter (interpolateSurface → lin_solve), refining
        # the grid until the fit's deviation from the locus is within tol; the distance
        # oracle uses closest_point_on_surface — so the module + its host gate build only
        # under CYBERCAD_HAS_NUMSCI (mirroring test_native_nurbs_fit). Airtight oracles:
        # every offset point at distance |d| from S (dense projection), analytic cylinder
        # (radius r±d) + exact plane, monotone error convergence, and an honest
        # self-intersection guard (offset past the curvature radius is DECLINED, never
        # returned folded). Non-rational offset; solid thicken/shell + fold trimming residual.
        list(APPEND CYBERCAD_TESTS test_native_nurbs_offset)
        set(test_native_nurbs_offset_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_nurbs_offset.cpp")
        # NURBS roadmap Layer 5 — SOLID THICKEN / SHELL (src/native/math/bspline_thicken).
        # Composes the Layer-5 offset (bspline_offset → lin_solve fit + its 2nd-fundamental-
        # form fold guard) into a CLOSED, watertight triangle shell (tessellate::Mesh): the
        # original panel S + the offset panel S+d·N + four ruled side walls, sewn with shared
        # exact boundary vertices so it is watertight by construction, then verified (χ=2, no
        # boundary edges, coherent orientation). The distance oracle uses
        # closest_point_on_surface — so the module + its host gate build only under
        # CYBERCAD_HAS_NUMSCI (mirroring test_native_nurbs_offset). Airtight oracles: the
        # solid is WATERTIGHT (χ=2/zero boundary edges) for flat/curved/rational patches,
        # enclosed volume == area·|d| exactly for a flat box (and ≈ for a thin slab / matches
        # the annular-wedge closed form for a cylinder shell), the offset cap sits at distance
        # |d| from S (matching offsetSurface), and an honest fold guard (thicken past the
        # curvature radius is DECLINED, never returned folded/self-intersecting). Non-rational
        # offset panel; robust self-intersecting-shell + multi-face shell + rational residual.
        list(APPEND CYBERCAD_TESTS test_native_nurbs_thicken)
        set(test_native_nurbs_thicken_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_nurbs_thicken.cpp")
        # NURBS roadmap Layer 5 — MULTI-FACE SOLID THICKEN / SHELL (src/native/math/
        # bspline_shell). Generalises the single-patch thicken to a small B-rep of 2+
        # edge-adjacent NURBS faces: offset EACH face (composing bspline_offset → the same
        # lin_solve fit + 2nd-fundamental-form fold guard), then along every SHARED interior
        # edge the two offset faces MEET DIRECTLY (welded at the shared offset edge) — NO
        # side wall — while side walls are built ONLY on the OUTER boundary. The whole set
        # welds (tolerance-bucketed spatial weld, corners where 3+ faces meet collapse to one
        # vertex) into ONE closed watertight triangle shell (tessellate::Mesh), closure proven
        # (χ=2, zero boundary edges, coherent orientation). Builds only under
        # CYBERCAD_HAS_NUMSCI (mirroring test_native_nurbs_thicken). Airtight oracles: a
        # 2-patch coplanar-pair / L-shape thickens to ONE closed solid (χ=2, no interior
        # double-wall), enclosed volume == total_area·|d| exactly for two coplanar rectangles
        # (== Σ per-face area·|d|, shared edge NOT double-counted), the shared interior edge
        # carries NO wall (every offset-side seam edge used twice), and fold / degenerate /
        # inconsistent-adjacency / zero-thickness declines. Non-rational offset panel; robust
        # self-intersecting-shell recovery + rational offset residual.
        list(APPEND CYBERCAD_TESTS test_native_nurbs_shell)
        set(test_native_nurbs_shell_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_nurbs_shell.cpp")
        # SSI Stage S2 subdivision seeding — the refine (least_squares) is under
        # CYBERCAD_HAS_NUMSCI, so its host suite builds only in that config.
        list(APPEND CYBERCAD_TESTS test_native_ssi_seeding)
        set(test_native_ssi_seeding_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_ssi_seeding.cpp")
        # SSI Stage S3 marching-line tracer — the corrector (least_squares) + B-spline
        # fit (lstsq) are under CYBERCAD_HAS_NUMSCI, so its host suite builds only here.
        list(APPEND CYBERCAD_TESTS test_native_ssi_marching)
        set(test_native_ssi_marching_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_ssi_marching.cpp")
        # HOST exact-oracle SSI differential fuzzer — the no-OCCT, machine-precision
        # complement to the SIM/OCCT freeform SSI fuzzer. Leg 1 is S1 analytic breadth
        # (closed-form oracle, always-compiled core), leg 2 is a NURBS↔analytic
        # known-answer test driving the real seed_intersection + trace_intersection
        # pipeline, whose definitions call the least_squares / lstsq substrate — so this
        # host suite builds only under CYBERCAD_HAS_NUMSCI (mirroring test_native_ssi_seeding).
        list(APPEND CYBERCAD_TESTS test_native_ssi_exact_fuzz)
        set(test_native_ssi_exact_fuzz_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_ssi_exact_fuzz.cpp")
        # MOAT M2b / B2 freeform face-split — host GATE (a), the OCCT-free analytic
        # tiling proof. Its fixture seam is the real S3 trace (trace_intersection),
        # whose definition is under CYBERCAD_HAS_NUMSCI, so this host suite builds only
        # in that config (like the S3 marching suite). Header-only splitter; no lib change.
        list(APPEND CYBERCAD_TESTS test_native_face_split)
        set(test_native_face_split_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_face_split.cpp")
        # SSI Stage S4-e chart singularities (sphere pole / cone apex crossing) — the
        # point-based crossing corrector rides the same least_squares substrate, so its
        # host suite builds only under CYBERCAD_HAS_NUMSCI (like the S3 marching suite).
        list(APPEND CYBERCAD_TESTS test_native_ssi_s4e_singularities)
        set(test_native_ssi_s4e_singularities_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_ssi_s4e_singularities.cpp")
        # SSI Stage S4-f completeness + loop-robustness (robust true-return closure, self-
        # intersection guard, adaptive completeness critic) — rides the same least_squares /
        # lstsq substrate, so its host suite builds only under CYBERCAD_HAS_NUMSCI.
        list(APPEND CYBERCAD_TESTS test_native_ssi_s4f_completeness)
        set(test_native_ssi_s4f_completeness_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_ssi_s4f_completeness.cpp")
        # SSI Stage S5-a curved boolean — consumes the S3 TraceSet, so it is gated on
        # CYBERCAD_HAS_NUMSCI too and its host suite builds only in that config.
        list(APPEND CYBERCAD_TESTS test_native_ssi_boolean)
        set(test_native_ssi_boolean_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_ssi_boolean.cpp")
        # SSI Stage S5-a ANALYTIC-ground-truth tests (Steinmetz 16r³/3 + inclusion–
        # exclusion monotone relations, no OCCT). Consumes the S3 TraceSet, so it is
        # gated on CYBERCAD_HAS_NUMSCI too and its host suite builds only in that config.
        list(APPEND CYBERCAD_TESTS test_native_ssi_curved_boolean)
        set(test_native_ssi_curved_boolean_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_ssi_curved_boolean.cpp")
        # NURBS roadmap Layer 3 — exact-NURBS B-rep boolean STAGE-READINESS MEASUREMENT
        # HARNESS (openspec/L3-EXACT-NURBS-BOOLEAN-READINESS.md). MEASUREMENT ONLY: it
        # drives real NURBS↔NURBS operand pairs through the EXISTING kernel pieces an
        # exact-NURBS boolean composes — SSI seed/trace (stage 1), constructPcurve
        # (stage 2), classify (stages 3/4), pcurveFidelity (stage 5) — and reports how
        # far native gets per stage vs a closed-form (rational-NURBS-as-quadric) oracle,
        # no OCCT. Stages 1/2 call seed_intersection / trace_from_seeds / constructPcurve
        # (always-visible declarations whose defs are under CYBERCAD_HAS_NUMSCI), so the
        # TU needs the macro but NOT the substrate include trees, like the Layer-8 gate;
        # it registers only in the numsci build so its stage-1/2 legs are exercised. It
        # modifies no src/native and no cc_* ABI; a broken invariant (a fabricated trace /
        # non-round-tripping pcurve / accepted drifted seam) exits 1, else 0.
        list(APPEND CYBERCAD_TESTS test_native_l3_boolean_readiness)
        set(test_native_l3_boolean_readiness_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_l3_boolean_readiness.cpp")
        # MOAT M2-assembly / B4 first freeform↔analytic half-space CUT — host GATE (a),
        # the OCCT-free composition proof: B1 recognise → M1 trace → B2 split → B4
        # analytic-face-split + section-cap weld → self-verify, matching the closed-form
        # CUT volume. Its operand's freeform seam is the real S3 trace (trace_intersection),
        # so this host suite builds only under CYBERCAD_HAS_NUMSCI. Header-only B4; no lib change.
        list(APPEND CYBERCAD_TESTS test_native_first_freeform_boolean)
        set(test_native_first_freeform_boolean_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_first_freeform_boolean.cpp")
        # MOAT M2-breadth / COMMON — host GATE (a), the SECOND freeform boolean operator as
        # the complementary keep-side of the landed half-space CUT (no new geometry verb),
        # gated on the closed-form partition identity + the complementary volume. Its operand's
        # freeform seam is the real S3 trace, so this host suite builds only under
        # CYBERCAD_HAS_NUMSCI. Additive sibling to the M2-assembly CUT suite; no lib change.
        list(APPEND CYBERCAD_TESTS test_native_freeform_boolean_breadth)
        set(test_native_freeform_boolean_breadth_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_freeform_boolean_breadth.cpp")
        # MOAT M2-FUSE — host GATE (a), the FIRST two-operand freeform boolean: a
        # bowl-lidded freeform prism A FUSED/CUT/COMMON with a FINITE all-planar box B for
        # the single-curved-cut pose. Composes recognise[B1] → inter-solid seam → split
        # both operands → B3-confirm → weld → self-verify, matching the closed-form union /
        # cut / common volumes. Its inter-solid seam is the real S3 trace, so this host
        # suite builds only under CYBERCAD_HAS_NUMSCI. Additive header-only verbs
        # (inter_solid_seam.h + two_operand.h); no lib change, no cc_* ABI change.
        list(APPEND CYBERCAD_TESTS test_native_two_operand_freeform_boolean)
        set(test_native_two_operand_freeform_boolean_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_two_operand_freeform_boolean.cpp")
        # MOAT M2-multiseam — host GATE (a), the FIRST multi-seam two-operand freeform
        # boolean: a bowl-lidded prism A against a corner box B whose TWO adjacent faces
        # slice A's Bézier wall (a two-arc, one-junction seam graph). Proves buildSeamGraph
        # in ISOLATION (two cutting faces, analytic junction J on both planes, arcs joined
        # + each B2-split) against the closed-form corner oracle, and records the measured
        # junction-vertex weld as the next blocker (freeformBooleanMultiSeam → NULL → OCCT).
        # Its arcs are the real S3 trace, so this host suite builds only under
        # CYBERCAD_HAS_NUMSCI. Additive header-only verbs (seam_graph.h + multi_seam.h);
        # no lib change, no cc_* ABI change.
        list(APPEND CYBERCAD_TESTS test_native_multi_seam)
        set(test_native_multi_seam_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_multi_seam.cpp")
        # MOAT M2 blocker #4 (≥3-seam) — host GATE (a): the OCCT-free proof that the
        # THREE-arc, TWO-junction seam-graph CHAIN builder lands in ISOLATION. `A` = the
        # bowl-lidded quad prism; `B` = a box straddling one EDGE so three faces slice
        # `A`'s Bézier wall. Proves buildChainSeamGraph (three iso-arcs — two parallel
        # ends + one orthogonal middle — two analytic junctions on both planes + in
        # domain, arcs joined into one bent boundary→J1→J2→boundary chain) against the
        # closed-form strip oracle; the 2-junction weld is the sharpened next blocker.
        # Its arcs are the real S3 trace, so this host suite builds only under
        # CYBERCAD_HAS_NUMSCI. Additive header-only verb (seam_graph_chain.h); no lib
        # change, no cc_* ABI change.
        list(APPEND CYBERCAD_TESTS test_native_chain_seam)
        set(test_native_chain_seam_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_chain_seam.cpp")
        # MOAT M2b / B2 SMOOTH-TRIM generalisation — host GATE (a), the OCCT-free
        # closed/circular-seam tiling proof. A quad-trimmed Bézier bowl sliced by a
        # HORIZONTAL plane (z=c) has a CLOSED CIRCULAR seam interior to the face; the
        # additive splitFaceSmoothTrim partitions it into the enclosed disk + the
        # annulus (the seam as a hole), tiling to machine ε at the closed-form circle
        # area, both sub-faces meshing watertight at multiple deflections. Byte-frozen
        # B2 splitFace still DECLINES the same seam (CrossingsNot2, crossings==0) — the
        # contrast asserted. Its seam is the real S3 trace, so this host suite builds
        # only under CYBERCAD_HAS_NUMSCI. Additive header-only verb (smooth_trim_split.h);
        # no lib change, no cc_* ABI change.
        list(APPEND CYBERCAD_TESTS test_native_smooth_trim_split)
        set(test_native_smooth_trim_split_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_smooth_trim_split.cpp")
        # MOAT M2 curved-wall freeform half-space CUT / COMMON — host GATE (a), the
        # OCCT-free closed-form VOLUME proof of the weld that CONSUMES the B2 smooth-trim
        # enabler. A bowl-cup solid (a Bézier bowl trimmed by a rim CIRCLE + a flat
        # top-lid disk) cut by the HORIZONTAL plane z=c has a CLOSED CIRCULAR seam on the
        # freeform wall; curvedWallHalfSpaceCut composes recognise[B1] → trace[M1] →
        # splitFaceSmoothTrim[B2] → analytic wall split[B4] → flat circular cap synth →
        # M0 weld → watertight+volume self-verify into the CUT (Below) and COMMON (Above)
        # keep sides, each welding watertight (Euler χ=2) at the closed-form volume
        # (π·ρ²·c/2 and V(full)−that), with partition closure V(below)+V(above)=V(full).
        # Non-cutting-plane + non-freeform declines to NULL→OCCT. Its seam is the real S3
        # trace, so this host suite builds only under CYBERCAD_HAS_NUMSCI. Additive
        # header-only verb (curved_wall_cut.h); no lib change, no cc_* ABI change.
        # Also covers the M2 MID-WALL pose (walled_bowl_midwall_fixture.h): a steep bowl
        # over a convex quad + 4 planar walls + a base cut by a horizontal plane whose
        # seam is a closed interior circle AND which splits the walls, so the cross-section
        # cap is an ANNULUS (outer = wall-section polygon, inner HOLE = seam) — welding
        # watertight (χ=2) and converging to (H0+c)·A_Q − c·π·ρ²/2.
        list(APPEND CYBERCAD_TESTS test_native_curved_wall_cut)
        set(test_native_curved_wall_cut_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_curved_wall_cut.cpp")
        # NURBS roadmap LAYER 3, SLICE 1 (L3-S1) — host GATE (a), the OCCT-free
        # closed-form proof of the FIRST exact-NURBS B-rep boolean: a genuine NURBS
        # (Kind::BSpline degree-2) face SPLIT BY A PLANE, welded watertight. A NURBS-
        # walled bowl-cup cut by the horizontal plane z=c has a CLOSED CIRCULAR seam on
        # the NURBS wall; nurbsFacePlaneSplit composes NURBS-adapter trace[stage 1] →
        # WLine-(u,v)-read fidelity gate[stage 2, routed around constructPcurve] →
        # splitFaceSmoothTrim[stage 3] → half-space keep[stage 4] → flat cap synth +
        # M0 curved↔flat weld[stage 5] → watertight+volume self-verify, into the CUT
        # (Below) and COMMON (Above) keep sides, each welding watertight at the closed-
        # form volume (π·ρ²·c/2 and V(full)−that) with partition closure V(below)+
        # V(above)=V(full). A non-NURBS wall / no-closed-seam plane HONEST-DECLINES to
        # NULL. Its seam is the real S3 trace, so this host suite builds only under
        # CYBERCAD_HAS_NUMSCI. Additive header-only verb (nurbs_plane_split.h);
        # no lib change, no cc_* ABI change.
        list(APPEND CYBERCAD_TESTS test_native_nurbs_plane_split)
        set(test_native_nurbs_plane_split_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_nurbs_plane_split.cpp")
        # MOAT M2 freeform↔freeform CLOSED-SEAM CUT / COMMON — host GATE (a), the
        # OCCT-free proof that the recognise[B1] → trace[M1] → split[B2 smooth-trim] →
        # classify[B3] pipeline composes for TWO curved operands over a shared CLOSED
        # curved seam, and that the weld verb HONEST-DECLINES to NULL (never a leaky
        # solid) because the two-CURVED-side closed-seam weld is gated on the byte-frozen
        # M0 tessellator (curved↔flat welds via the M0w pin; curved↔curved does not). Two
        # coaxial paraboloid bowl-cups whose curved walls meet in ONE closed circle ρ =
        # √(H/2a); the seam is the real S3 trace, so this host suite builds only under
        # CYBERCAD_HAS_NUMSCI. Additive header-only verb (freeform_freeform_cut.h); no lib
        # change, no cc_* ABI change.
        list(APPEND CYBERCAD_TESTS test_native_freeform_freeform_cut)
        set(test_native_freeform_freeform_cut_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_freeform_freeform_cut.cpp")
        # MOAT M2b freeform↔analytic DISJOINT (MULTI-LUMP) CUT — host GATE (a), the
        # OCCT-free proof that freeformSlabDisjointCut composes the landed inter-solid-seam
        # weld into the FIRST native freeform boolean whose result is TWO disconnected
        # bodies (a Compound of two watertight Solids — a bowl-lidded prism parted by a
        # central slab), and that its mandatory TWO-SIDED self-verify HONEST-DECLINES the
        # byte-frozen keep-face machinery's off-centre volume over-estimate (never a
        # wrong/leaky solid). The composition traces the real M1 seam per slab face, so this
        # host suite builds only under CYBERCAD_HAS_NUMSCI. Additive header-only verb
        # (slab_disjoint_cut.h); no lib change, no cc_* ABI change.
        list(APPEND CYBERCAD_TESTS test_native_slab_disjoint_cut)
        set(test_native_slab_disjoint_cut_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_slab_disjoint_cut.cpp")
        # MOAT (Class-B) native cc_thread_apply — host GATE (a), the OCCT-free proof that
        # the recognise[cylinder] → facet → planar-BSP boolean_solid → four-part self-verify
        # (watertight + Euler χ=2 + consistently-oriented + two-sided closed-form-volume band)
        # pipeline WELDS the tractable planar-cutter baseline (cylinder − box) and HONEST-
        # DECLINES a multi-turn helical thread with a measured reason (the near-tangent
        # helical BSP fragments + the orientation-inconsistent thread operand). Reuses the
        # planar BSP (boolean_solid → ssi_boolean_solid), so this host suite builds only under
        # CYBERCAD_HAS_NUMSCI. Additive header-only verb (thread_apply.h); no lib change, no
        # cc_* ABI change.
        list(APPEND CYBERCAD_TESTS test_native_thread_apply)
        set(test_native_thread_apply_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_thread_apply.cpp")
        # MOAT M-GS GS3/GS4 analysis services (src/native/analysis) — host GATE (a),
        # the OCCT-free closed-form/brute-force proof of minimum distance, angle and
        # surface/edge curvature (+ honest declines). The freeform seed-and-refine
        # distance path uses the numerics/closest_point minimizer, so this host suite
        # builds only under CYBERCAD_HAS_NUMSCI. Header-only analysis module; no lib change.
        list(APPEND CYBERCAD_TESTS test_native_analysis)
        set(test_native_analysis_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_analysis.cpp")
        # The additive cc_measure_* / cc_*_curvature FACADE over the NativeEngine —
        # host GATE (a) driving the real C ABI on a native box. The distance cell is
        # numsci-gated in the engine, so this facade suite builds under HAS_NUMSCI.
        list(APPEND CYBERCAD_TESTS test_native_analysis_facade)
        set(test_native_analysis_facade_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_analysis_facade.cpp")
        # MOAT M-DM DM1 — host GATE (a), the OCCT-free partition-closure proof of the
        # FIRST native direct-modeling verb (planar half-space SPLIT). splitByPlane
        # composes the landed freeformHalfSpaceCut + boolean_solid into the two pieces of
        # a plane cut; the freeform composition traces the real S3 seam, so this host
        # suite builds only under CYBERCAD_HAS_NUMSCI. Header-only verb; no lib change.
        list(APPEND CYBERCAD_TESTS test_native_split_plane)
        set(test_native_split_plane_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_split_plane.cpp")
        # MOAT M-DM DM2 — host GATE (a), the OCCT-free closed-form proof of the native
        # `replace_face_to_plane` (move a planar face to a target plane, re-solving the
        # neighbours). directmodel::replaceFaceToPlane composes the landed splitByPlane +
        # boolean_solid(Fuse) + build_prism into a parallel pull / push and a tilted
        # grow-then-trim; δ is affine over the planar face so ΔV = A_F·d̄ is the exact
        # oracle. The tilted re-solve's splitByPlane probes the real S3 seam, so this
        # host suite builds only under CYBERCAD_HAS_NUMSCI. Header-only verb; no lib change.
        list(APPEND CYBERCAD_TESTS test_native_replace_face)
        set(test_native_replace_face_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_replace_face.cpp")
        # MOAT M-DM DM3 — host GATE (a), the OCCT-free closed-form proof of the native
        # GENERAL `replace_face` (offset+tilt). replaceFaceOffsetTilt derives the target
        # plane from the picked face's own plane and re-solves via the DM2 verb; the
        # pure-offset slice has the exact V₀+A·offset oracle, and a non-zero tilt / curved
        # neighbour / no-op declines. The re-solve probes the real S3 seam, so this host
        # suite builds only under CYBERCAD_HAS_NUMSCI. Header-only verb; no lib change.
        list(APPEND CYBERCAD_TESTS test_native_replace_face_general)
        set(test_native_replace_face_general_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_replace_face_general.cpp")
        # MOAT feature DRAFT ANGLE — host GATE (a), the OCCT-free closed-form proof of
        # the native draft (taper planar side faces about a neutral plane). draftFaces
        # derives the tilted target plane per face (pivot on the trace with the neutral
        # plane) and re-solves via the DM2 verb; the box wedge / frustum taper has the
        # exact closed-form oracle, and a curved base / non-planar neutral / cap face /
        # degenerate angle declines. The re-solve probes the real S3 seam, so this host
        # suite builds only under CYBERCAD_HAS_NUMSCI. Header-only verb; no lib change.
        list(APPEND CYBERCAD_TESTS test_native_draft_faces)
        set(test_native_draft_faces_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_draft_faces.cpp")
    endif()

    # The tet-mesher test drives the real TetGen backend, so it is added ONLY when
    # CYBERCAD_HAS_TETGEN=ON. With the option OFF it is never built (the always-on
    # test_native_quality still covers the OCCT-free quality module).
    if(CYBERCAD_HAS_TETGEN)
        list(APPEND CYBERCAD_TESTS test_native_tet)
        set(test_native_tet_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_tet.cpp")
    endif()

    # Most tests live directly under tests/; native-library tests live under
    # tests/native/. Map the exceptions here (basename → source path).
    # NURBS roadmap Layer 1 — exact-NURBS geometry kernel (src/native/math/
    # bspline_ops). Host GATE: the OCCT-free, substrate-free closed-form oracle
    # (every knot/degree/split op preserves the curve & surface pointwise to
    # ~1e-12; insert↔remove and elevate↔reduce round-trips; honest decline for
    # irreducible inputs) plus the rational-evaluation hardening cases. Header-only
    # over math (no numsci link); always-on native suite.
    set(test_native_nurbs_ops_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_nurbs_ops.cpp")
    # MOAT NURBS L1 differential FUZZER (host arm) — the OCCT-free, seeded batch that
    # generalises the curated exact-oracle contract over RANDOM valid NURBS curves +
    # surfaces (rational + non-rational) run through RANDOM op-composition chains
    # (insert/refine/elevate/split/reparam/decompose), asserting geometry preservation
    # pointwise end-to-end, plus insert↔remove / elevate↔reduce round-trips and honest
    # decline. DISAGREED==0 bar across ≥2 seeds. Header-only over math; always-on suite.
    set(test_native_nurbs_ops_fuzz_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_nurbs_ops_fuzz.cpp")
    set(test_native_topology_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_topology.cpp")
    # NURBS roadmap Layer 8 — trimmed-NURBS B-rep data model + pcurve robustness
    # (src/native/topology/trimmed_nurbs). Host GATE: the OCCT-free airtight-oracle
    # proof (rect-sub-region + circular-hole containment In/Out/OnBoundary; a hole
    # point → Out; iso-curve pcurve fidelity S(p(t))==C(t) ~1e-9 and a wrong pcurve
    # DETECTED; honest degenerate/open/self-touching declines). Always-on legs use only
    # math + topology; the pcurve-CONSTRUCTION round-trip leg (projection + fit) calls
    # numerics::closest_point_on_surface + bspline_fit, so it is compiled only when
    # CYBERCAD_HAS_NUMSCI is defined (the macro is added to the TU below; it needs no
    # NumPP/SciPP substrate include tree — it calls only always-visible topology decls).
    set(test_native_trimmed_nurbs_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_trimmed_nurbs.cpp")
    set(test_native_tessellate_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_tessellate.cpp")
    # Render-quality DISPLAY mesh (src/native/render) — host GATE (a), the OCCT-free
    # closed-form-normal proof (cylinder radial wall normals to ~1e-6, cap↔wall crease
    # split, sphere all-smooth convergent normals, box 24 corner splits + 6 face
    # normals, crease-angle threshold, LOD tri-reduction within the Hausdorff bound,
    # UVs in [0,1] seam-consistent). Post-processes the existing tessellator's Mesh;
    # no tessellator change. Header-only over math + tessellate/mesh; always-on suite.
    set(test_native_display_mesh_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_display_mesh.cpp")
    # MOAT M6 render display-mesh DIFFERENTIAL-FUZZ (host arm) — the OCCT-free, seeded
    # batch generalising the fixed-shape display-mesh contract over random analytic
    # primitives (sphere/cylinder/cone/box) × varied deflection × crease/LOD/UV. Oracle =
    # CLOSED-FORM surface distance (deflection bound respected), plus finite/unit-normal/
    # non-degenerate/fold-watertight/UV-range invariants. DISAGREED==0 bar. Header-only over
    # render + construct + tessellate + topology (no numsci link); always-on native suite.
    set(test_native_display_mesh_fuzz_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_display_mesh_fuzz.cpp")
    set(test_native_construct_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_construct.cpp")
    set(test_native_profile_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_profile.cpp")
    set(test_native_residuals_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_residuals.cpp")
    set(test_native_loft_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_loft.cpp")
    set(test_native_sweep_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_sweep.cpp")
    # moat-vsweep — variable-section / guide+spine sweep host GATE (a): the OCCT-free
    # closed-form proof (circle→circle straight morph = truncated cone; constant section =
    # prism; guide-scaled square = frustum; curved-arc morph watertight + χ=2 stable) + the
    # honest declines. Header-only over construct/tessellate/topology; no numsci link, so it
    # lives in the always-on native suite.
    set(test_native_vsweep_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_vsweep.cpp")
    set(test_native_thread_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_thread.cpp")
    set(test_native_boolean_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_boolean.cpp")
    # near-tangent / near-coincident DENSE-CSG stress battery + failure-boundary map —
    # host GATE, OCCT-FREE closed-form-volume oracle. Drives the native boolean on dense
    # triangle-soup operands (mirroring thread_apply::facetSolid) across the tangent-angle
    # / coincidence-gap / overlap-ε sweep, asserts the measured three-band failure map and
    # the hard invariant DISAGREED==0 (never a silent-wrong result). Header-only over
    # boolean/construct/tessellate/topology; no numsci link, so it lives in the always-on
    # native suite next to test_native_boolean.
    set(test_native_dense_csg_stress_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_dense_csg_stress.cpp")
    set(test_native_curved_boolean_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_curved_boolean.cpp")
    set(test_native_freeform_membership_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_freeform_membership.cpp")
    # MOAT M2 / B1 freeform operand descriptor + recogniseFreeformSolid gate — host
    # GATE (a), the OCCT-free admission/round-trip/decline proof. Header-only B1; the
    # test also exercises the exposed M0/B2/B3 handles. No numsci link (it never calls
    # the S3 tracer), so it lives in the always-on native suite.
    set(test_native_freeform_operand_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_freeform_operand.cpp")
    set(test_native_ssi_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_ssi.cpp")
    set(test_native_ssi_s4_classification_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_ssi_s4_classification.cpp")
    set(test_native_heal_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_heal.cpp")
    set(test_native_blend_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_blend.cpp")
    # MOAT M3 analytic face-fillet — host GATE (a), the OCCT-free proof of the native
    # full-round (prismatic tangent-cylinder cap) + the MEASURED fillet_face corner-weld
    # decline (gates on M2). Reuses only blend/construct/tessellate/topology (no S3
    # tracer), so it lives in the always-on native suite. Header-only ops; no lib change.
    set(test_native_analytic_fillet_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_analytic_fillet.cpp")
    # MOAT M3 CURVED SHELL — host GATE (a), the OCCT-free proof that curved_shell hollows a
    # capped cylinder / cone frustum to a uniform wall (analytic inward offset of the curved
    # wall, one cap open) watertight at the CLOSED-FORM wall volume (converging as deflection
    # refines) + the honest declines (stepped shaft / both caps / picked wall / t too large).
    # Pure assembly layer (no tessellator change). Header-only over blend/construct/
    # tessellate/topology; no numsci link, always-on native suite.
    set(test_native_curved_shell_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_curved_shell.cpp")
    # MOAT M2/M3 spherical FILLET-CORNER weld — host GATE (a): the OCCT-free proof that
    # fillet_corner rounds every convex edge bounding a prism cap, welding the tangent-
    # cylinder strips with a spherical corner patch (shared great-circle arcs) watertight
    # + consistently oriented at the closed-form volume (converging as deflection refines)
    # + the honest declines. Pure assembly layer (no tessellator change). Header-only over
    # blend/construct/tessellate/topology; no numsci link, always-on native suite.
    set(test_native_fillet_corner_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_fillet_corner.cpp")
    # MOAT M2 convex-corner chamfer weld — host GATE (a): the OCCT-free closed-form
    # proof that chamfer_corner welds 1/2/3 mutually-adjacent convex edges sharing a
    # corner watertight at the EXACT inclusion-exclusion volume (planar, tessellator-
    # independent) + the honest declines. Header-only over blend/construct/tessellate/
    # topology; no numsci link, so it lives in the always-on native suite.
    set(test_native_corner_chamfer_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_corner_chamfer.cpp")
    # MOAT SURFACE bounded N-sided fill (src/native/surface) — host GATE (a), the OCCT-free
    # proof of the tessellated Coons/Gregory fill patch: planar N-gon area = exact polygon
    # area, a box's missing planar face restored watertight at the EXACT original volume, a
    # non-planar saddle boundary interpolated (on-boundary residual 0) + welded watertight +
    # consistently oriented + area converging as gridN doubles, and the honest declines
    # (>6-sided / degenerate / collinear-arc boundary). Header-only over surface/boolean/
    # tessellate/topology/construct; no numsci link, always-on native suite.
    set(test_native_surfacing_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_surfacing.cpp")
    # MOAT M2 FULL-FACE chamfer weld (chamfer_face): chamfer every convex edge bounding a
    # picked planar face by assembling the landed corner weld. On a box face all four
    # corners are 2-edge dihedrals (never a triple), so it welds watertight at the EXACT
    # inclusion-exclusion volume (planar, tessellator-independent) + the honest declines.
    # Header-only over blend/construct/tessellate/topology; no numsci link, always-on.
    set(test_native_chamfer_face_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_chamfer_face.cpp")
    set(test_native_wrap_emboss_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_wrap_emboss.cpp")
    # MOAT M-SM sheet metal (first slice) — host GATE (a), the OCCT-free closed-form
    # proof: base flange volume = profileArea·thickness; single edge flange (cylindrical
    # bend + wall) watertight / χ=2 / oriented at base + ½θ((r+t)²−r²)·W + h·t·W; unfold
    # develops to (baseRun + BA + h)·W with BA = θ(r + k·t), area-invariant under
    # fold→unfold; and the measured honest-declines (bad param / non-straight bend line /
    # non-recognised base / self-collision). Header-only over construct/tessellate/
    # topology; no numsci link, always-on native suite.
    set(test_native_sheetmetal_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_sheetmetal.cpp")
    set(test_native_step_writer_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_step_writer.cpp")
    set(test_native_step_reader_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_step_reader.cpp")
    set(test_native_step_pmi_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_step_pmi.cpp")
    set(test_native_stl_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_stl.cpp")
    # MOAT exchange — glTF 2.0 (.gltf JSON + .glb binary) + USDZ (STORE-zip, 64-byte
    # data-aligned) mesh export, the iPad AR / QuickLook / share / render handoff. Host
    # GATE (a) round-trip (native mesh → glTF/USDZ → re-parse → same vert/tri/bbox) +
    # GATE (b) structural validity (glTF-2.0 accessor/bufferView/alignment/min-max, the
    # .glb chunk layout, the .usdz zip container). Facade-driven under the native engine;
    # pure additive native writers (tessellator untouched); no numsci link.
    set(test_native_gltf_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_gltf.cpp")
    # MOAT M6 glTF ROUND-TRIP differential fuzzer — the no-silent-wrong completeness
    # bar for the OCCT-FREE native glTF 2.0 writer (src/native/exchange/gltf_writer).
    # A WRITER has no OCCT equivalence to diff against, so the arbiter is ROUND-TRIP /
    # SELF-CONSISTENCY: a self-contained glTF-2.0 + .glb structural validator plus a
    # geometry-fidelity round-trip (parse the emitted buffers, compare vert/tri/
    # connectivity/bbox against an independent reimplementation of the writer's
    # documented compaction contract). Seeded splitmix64→xoshiro256** generator over
    # analytic primitives, multi-solids, fans, soup, empty/1-tri/bad-index edge cases,
    # and tiny/huge coordinate scales. Calls the native writer directly (no engine, no
    # numsci link); always-on native suite. BAR: DISAGREED==0.
    set(test_native_gltf_fuzz_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_gltf_fuzz.cpp")
    set(test_native_quality_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_quality.cpp")
    # MOAT M-GS GS1 orthographic HLR — host GATE (a), the OCCT-free analytic proof
    # (box from an isometric corner → 9 visible + 3 hidden segments; edge splitting
    # at visibility transitions). Header-only drafting core over the M0 occluder
    # mesh; no numsci link, so it lives in the always-on native suite.
    set(test_native_drafting_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_drafting.cpp")
    # MOAT M-GS GS2 planar SECTION CURVES — host GATE (a), the OCCT-free closed-form
    # proof (box rectangle / cylinder cross-section circle πR² / cylinder axial
    # rectangle 2R·H / sphere & cone circles + honest declines: oblique cylinder cut,
    # coincident plane). Header-only section module over M1-SSI + topology consumed
    # read-only; no numsci link, so it lives in the always-on native suite.
    set(test_native_section_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_section.cpp")
    # MOAT M-REF reference / topology reads — host GATE (a), the OCCT-free closed-form
    # proof of the datum + topology-reference services (src/native/reference): a box's
    # face normals / datum planes / linear-edge axes / outer-rim loops, a cylinder's
    # face axis, tangent-chain growth on a smooth contour, and a rectangle offset
    # boundary — all asserted to machine precision — plus the honest declines
    # (non-planar plane, non-linear axis, growing convex offset). Header-only over
    # topology consumed read-only; no numsci link.
    set(test_native_reference_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_reference.cpp")
    set(test_native_step_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_step.cpp")
    # MOAT M-GS GS5 native INERTIA — host GATE (a), the OCCT-free closed-form proof
    # of the signed-tetra second-moment + eigen inertia (box exact to machine eps;
    # cylinder/sphere converge to the closed form as deflection→0) plus the native
    # cc_principal_moments facade wiring + the watertight-precondition honest decline.
    # Header-only analysis math over hand-built meshes; no numsci link.
    set(test_native_inertia_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_inertia.cpp")
    # MOAT M-GS GS6 native VALIDITY — host GATE (a), the known-state fixture proof of
    # cc_check_solid: a valid box PASSES; deliberately-broken fixtures (open shell,
    # flipped face, degenerate collapse, self-intersecting boxes) each FAIL via the
    # specific check, with the coplanar-overlap honest decline. Header-only mesh
    # checker + the cc_check_solid facade wiring; no numsci link.
    set(test_native_validity_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_validity.cpp")
    # MOAT M-GS GS7 native CLASH / INTERFERENCE — host GATE (a): the OCCT-free
    # closed-form proof of cc_interference over the NativeEngine. Overlapping boxes
    # → CLASH at the exact intersection-box volume (native boolean COMMON) + witness;
    # disjoint → CLEAR at the exact gap; face-touching → TOUCHING (zero-volume
    # contact); nested → CLASH; and a non-watertight operand → honest Unknown decline.
    # Header-only mesh classifier + the cc_interference facade wiring; no numsci link.
    set(test_native_interference_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_interference.cpp")
    # MOAT M-DM DM4 native point PROJECTION — host GATE (a), the OCCT-free closed-form
    # proof of projectPointOnFace: plane (drop the normal component) / cylinder & sphere
    # (radial push to R) feet + distances, and honest declines (cone / ambiguous axis /
    # sphere centre / foreign face id). Pure analytic geometry over hand-built B-rep
    # faces; no numsci link, so it lives in the always-on native suite.
    set(test_native_project_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_project.cpp")
    # MOAT ABI-parity — host GATE (a): the OCCT-free proof of the app-parity connected-
    # solid enumeration (cc_shape_solid_count / cc_shape_solid_at) on a known 2-lump
    # compound + the FACADE single-solid path, and the honest native decline of the
    # app-parity loft variants. Header-only topology/tessellate/construct; no numsci link.
    set(test_native_solid_enum_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_solid_enum.cpp")

    foreach(test ${CYBERCAD_TESTS})
        if(DEFINED ${test}_SRC)
            set(test_src "${${test}_SRC}")
        else()
            set(test_src "${CMAKE_CURRENT_SOURCE_DIR}/tests/${test}.cpp")
        endif()
        add_executable(${test} "${test_src}")
        target_link_libraries(${test} PRIVATE cybercadkernel)
        target_include_directories(${test} PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}/src"
            "${CMAKE_CURRENT_SOURCE_DIR}/tests")
        add_test(NAME ${test} COMMAND ${test})
    endforeach()

    # The SSI Stage S5-a boolean test reaches into the substrate-gated ssidetail::
    # helpers (recogniseCurvedSolid / classifyPoint), so its TU must see
    # CYBERCAD_HAS_NUMSCI and the substrate include trees — matching how the library
    # is built. (The S2/S3 tests only call always-visible declarations, so they do
    # not need this.) Guarded on the option + target existence.
    # The S4-a classification test's SEEDED assertions are guarded by CYBERCAD_HAS_NUMSCI;
    # it only calls always-visible declarations (seed_intersection / classify_degeneracy),
    # so unlike the boolean test it needs the macro but NOT the substrate include trees.
    if(CYBERCAD_HAS_NUMSCI AND TARGET test_native_ssi_s4_classification)
        target_compile_definitions(test_native_ssi_s4_classification PRIVATE CYBERCAD_HAS_NUMSCI=1)
    endif()

    # The Layer-7 fitting gate includes bspline_fit.h, which under CYBERCAD_HAS_NUMSCI
    # pulls in the numerics facade header (numerics.h — vec.h + std only, no NumPP/SciPP
    # headers), so its TU needs the macro but NOT the substrate include trees.
    if(CYBERCAD_HAS_NUMSCI AND TARGET test_native_nurbs_fit)
        target_compile_definitions(test_native_nurbs_fit PRIVATE CYBERCAD_HAS_NUMSCI=1)
    endif()

    # The Layer-8 trimmed-NURBS gate is ALWAYS built (its containment + fidelity +
    # degenerate legs are substrate-free). Its pcurve-CONSTRUCTION leg calls the
    # numsci-gated constructPcurve, so under CYBERCAD_HAS_NUMSCI the TU gets the macro
    # (it calls only always-visible topology declarations — no NumPP/SciPP header — so
    # it needs the macro but NOT the substrate include trees, like the Layer-7 gate).
    if(CYBERCAD_HAS_NUMSCI AND TARGET test_native_trimmed_nurbs)
        target_compile_definitions(test_native_trimmed_nurbs PRIVATE CYBERCAD_HAS_NUMSCI=1)
    endif()

    # The Layer-3 exact-NURBS-boolean readiness harness calls the numsci-gated SSI
    # seed/trace + constructPcurve declarations (stages 1/2), so under CYBERCAD_HAS_NUMSCI
    # its TU gets the macro. Like the Layer-8 gate it uses only always-visible ssi/topology
    # declarations (makeNurbsAdapter / seed_intersection / trace_from_seeds / constructPcurve
    # / classify / pcurveFidelity) — no NumPP/SciPP header — so it needs the macro but NOT
    # the substrate include trees.
    if(CYBERCAD_HAS_NUMSCI AND TARGET test_native_l3_boolean_readiness)
        target_compile_definitions(test_native_l3_boolean_readiness PRIVATE CYBERCAD_HAS_NUMSCI=1)
    endif()

    # The L3-S1 exact-NURBS face-split-by-plane gate traces the NURBS wall ∩ plane seam
    # via the numsci-gated ssi::trace_intersection (through nurbs_plane_split.h), so
    # under CYBERCAD_HAS_NUMSCI its TU gets the macro. It calls only always-visible ssi /
    # topology / tessellate declarations (makeBSplineAdapter / trace_intersection +
    # header-only boolean/tessellate verbs) — no NumPP/SciPP header — so it needs the
    # macro but NOT the substrate include trees, like the L3 readiness gate above.
    if(CYBERCAD_HAS_NUMSCI AND TARGET test_native_nurbs_plane_split)
        target_compile_definitions(test_native_nurbs_plane_split PRIVATE CYBERCAD_HAS_NUMSCI=1)
    endif()

    # The Layer-6 skinning gate includes bspline_skin.h (which under CYBERCAD_HAS_NUMSCI
    # only pulls bspline_ops.h — no substrate header), so its TU needs the macro but NOT
    # the substrate include trees, exactly like the Layer-7 fitting gate above.
    if(CYBERCAD_HAS_NUMSCI AND TARGET test_native_nurbs_skin)
        target_compile_definitions(test_native_nurbs_skin PRIVATE CYBERCAD_HAS_NUMSCI=1)
    endif()

    # The Layer-6 swept-surface gate includes bspline_sweep.h (which under CYBERCAD_HAS_NUMSCI
    # pulls bspline_skin.h → bspline_ops.h — no substrate header), so its TU needs the macro
    # but NOT the substrate include trees, exactly like the Layer-6 skinning gate above.
    if(CYBERCAD_HAS_NUMSCI AND TARGET test_native_nurbs_sweep)
        target_compile_definitions(test_native_nurbs_sweep PRIVATE CYBERCAD_HAS_NUMSCI=1)
    endif()
    # The Layer-6 Gordon/network gate includes bspline_gordon.h (which under
    # CYBERCAD_HAS_NUMSCI pulls bspline_skin.h → bspline_ops.h + numerics.h), so its TU
    # needs the macro to compile the SEEDED gordonSurface path (matches the library build).
    if(CYBERCAD_HAS_NUMSCI AND TARGET test_native_nurbs_gordon)
        target_compile_definitions(test_native_nurbs_gordon PRIVATE CYBERCAD_HAS_NUMSCI=1)
    endif()
    # The Layer-5 offset gate includes bspline_offset.h + numerics.h (numerics.h is
    # vec.h + std only, no NumPP/SciPP headers), so its TU needs the macro but NOT the
    # substrate include trees, exactly like the Layer-6/7 gates above.
    if(CYBERCAD_HAS_NUMSCI AND TARGET test_native_nurbs_offset)
        target_compile_definitions(test_native_nurbs_offset PRIVATE CYBERCAD_HAS_NUMSCI=1)
    endif()
    # The Layer-5 thicken gate includes bspline_thicken.h (→ bspline_ops.h + tessellate/
    # mesh.h) + bspline_offset.h + numerics.h — all vec.h/std only, no NumPP/SciPP headers —
    # so its TU needs the macro but NOT the substrate include trees, like the offset gate.
    if(CYBERCAD_HAS_NUMSCI AND TARGET test_native_nurbs_thicken)
        target_compile_definitions(test_native_nurbs_thicken PRIVATE CYBERCAD_HAS_NUMSCI=1)
    endif()
    # The Layer-5 MULTI-FACE shell gate includes bspline_shell.h (→ bspline_ops.h +
    # tessellate/mesh.h) + bspline_offset.h — all vec.h/std only, no NumPP/SciPP headers —
    # so its TU needs the macro but NOT the substrate include trees, like the thicken gate.
    if(CYBERCAD_HAS_NUMSCI AND TARGET test_native_nurbs_shell)
        target_compile_definitions(test_native_nurbs_shell PRIVATE CYBERCAD_HAS_NUMSCI=1)
    endif()

    if(CYBERCAD_HAS_NUMSCI AND TARGET test_native_ssi_boolean)
        target_compile_definitions(test_native_ssi_boolean PRIVATE CYBERCAD_HAS_NUMSCI=1)
        target_include_directories(test_native_ssi_boolean PRIVATE
            "${CYBERCAD_NUMPP_DIR}/include"
            "${CYBERCAD_SCIPP_DIR}/include"
            "${CYBERCAD_NUMSCI_DIR}/gen")
    endif()

    # The S5-a analytic-ground-truth test reaches into the same substrate-gated
    # ssidetail:: helpers (recogniseCurvedSolid), so its TU also needs
    # CYBERCAD_HAS_NUMSCI and the substrate include trees.
    if(CYBERCAD_HAS_NUMSCI AND TARGET test_native_ssi_curved_boolean)
        target_compile_definitions(test_native_ssi_curved_boolean PRIVATE CYBERCAD_HAS_NUMSCI=1)
        target_include_directories(test_native_ssi_curved_boolean PRIVATE
            "${CYBERCAD_NUMPP_DIR}/include"
            "${CYBERCAD_SCIPP_DIR}/include"
            "${CYBERCAD_NUMSCI_DIR}/gen")
    endif()

    if(EXISTS "${CYBERCAD_REFERENCE_ABI_HEADER}")
        get_filename_component(CYBERCAD_REFERENCE_ABI_DIR
                               "${CYBERCAD_REFERENCE_ABI_HEADER}" DIRECTORY)
        target_include_directories(test_abi PRIVATE "${CYBERCAD_REFERENCE_ABI_DIR}")
        target_compile_definitions(test_abi PRIVATE CYBERCAD_HAVE_REFERENCE_ABI=1)
    endif()
endif()
