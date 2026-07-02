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

# Include dir for the trimmed arm64-iOS OCCT the adapter compiles against. Only
# used when CYBERCAD_HAS_OCCT=ON (iOS cross-compile); override for a different
# OCCT install. See docs/occt-build.md in the app repo.
set(CYBERCAD_OCCT_INCLUDE_DIR
    "/Users/leonardoaraujo/work/cybercad/build/occt/install-SIMULATORARM64/include/opencascade"
    CACHE PATH "OCCT headers for the OCCT engine adapter (iOS only)")

# ── library sources ───────────────────────────────────────────────────────────
# All src/**.cpp EXCEPT src/engine/occt/* (added only when CYBERCAD_HAS_OCCT=ON).
file(GLOB_RECURSE CYBERCAD_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
list(FILTER CYBERCAD_SOURCES EXCLUDE REGEX "/src/engine/occt/")

add_library(cybercadkernel STATIC ${CYBERCAD_SOURCES})

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
        test_abi)

    foreach(test ${CYBERCAD_TESTS})
        add_executable(${test} "${CMAKE_CURRENT_SOURCE_DIR}/tests/${test}.cpp")
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
