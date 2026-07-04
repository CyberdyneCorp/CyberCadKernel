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

    # The exact TK* order from the proven recipe.
    target_link_libraries(cybercadkernel PRIVATE
        -L"${CYBERCAD_HOMEBREW_OCCT_LIB}"
        -lTKDESTEP -lTKDEIGES -lTKXSBase -lTKDE -lTKXCAF -lTKLCAF -lTKCAF
        -lTKCDF -lTKMesh -lTKShHealing -lTKOffset -lTKFillet -lTKBool -lTKPrim
        -lTKBO -lTKTopAlgo -lTKGeomAlgo -lTKBRep -lTKGeomBase -lTKG3d -lTKG2d
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
        test_native_math
        test_native_topology
        test_native_tessellate
        test_native_construct
        test_native_profile
        test_native_residuals
        test_native_loft
        test_native_sweep
        test_native_thread
        test_native_boolean
        test_native_curved_boolean
        test_native_ssi
        test_native_blend
        test_native_step_writer
        test_native_step
        test_native_engine)

    # The numeric-facade test links NumPP/SciPP, so it is added ONLY when the
    # substrate is built in (CYBERCAD_HAS_NUMSCI=ON). With the option OFF the
    # existing suite is unchanged.
    if(CYBERCAD_HAS_NUMSCI)
        list(APPEND CYBERCAD_TESTS test_native_numerics)
        set(test_native_numerics_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_numerics.cpp")
    endif()

    # Most tests live directly under tests/; native-library tests live under
    # tests/native/. Map the exceptions here (basename → source path).
    set(test_native_topology_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_topology.cpp")
    set(test_native_tessellate_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_tessellate.cpp")
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
    set(test_native_thread_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_thread.cpp")
    set(test_native_boolean_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_boolean.cpp")
    set(test_native_curved_boolean_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_curved_boolean.cpp")
    set(test_native_ssi_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_ssi.cpp")
    set(test_native_blend_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_blend.cpp")
    set(test_native_step_writer_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_step_writer.cpp")
    set(test_native_step_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_step.cpp")

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

    if(EXISTS "${CYBERCAD_REFERENCE_ABI_HEADER}")
        get_filename_component(CYBERCAD_REFERENCE_ABI_DIR
                               "${CYBERCAD_REFERENCE_ABI_HEADER}" DIRECTORY)
        target_include_directories(test_abi PRIVATE "${CYBERCAD_REFERENCE_ABI_DIR}")
        target_compile_definitions(test_abi PRIVATE CYBERCAD_HAVE_REFERENCE_ABI=1)
    endif()
endif()
