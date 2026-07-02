#!/usr/bin/env bash
# Build the macOS (arm64) desktop shared library — libcybercadkernel.dylib — the
# proven way, via the CMake option CYBERCAD_MACOS_OCCT=ON.
#
# This codifies the PROVEN manual recipe:
#   * compile every src/**/*.cpp EXCEPT the Metal TUs (src/compute/metal/*),
#   * -DCYBERCAD_HAS_OCCT, C++20, -arch arm64, against Homebrew OCCT (v7.9.3)
#     headers at $OCCT_PREFIX/include/opencascade,
#   * link a SHARED dylib against the TK* libraries in $OCCT_PREFIX/lib with an
#     rpath so OCCT resolves at load time.
# Metal is EXCLUDED on macOS (no -DCYBERCAD_HAS_METAL); cc_set_gpu_tessellation is
# a safe no-op there.
#
# Output: build-mac/libcybercadkernel.dylib
#
# Usage:  scripts/build-macos-dylib.sh          # configure + build (incremental)
#         CLEAN=1 scripts/build-macos-dylib.sh   # wipe build-mac first
#
# Robust in both bash and zsh: no unquoted-var word-splitting is relied on (CMake
# globs the sources; the TK* list lives in CMakeLists.txt).
set -euo pipefail

# Resolve the repo root from this script's location (works in bash and zsh).
if [[ -n "${BASH_SOURCE:-}" ]]; then
  SELF="${BASH_SOURCE[0]}"
else
  SELF="${(%):-%N}"   # zsh: path to the currently sourced/run script
fi
SCRIPT_DIR="$(cd "$(dirname "$SELF")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"

OCCT_PREFIX="${OCCT_PREFIX:-/opt/homebrew/opt/opencascade}"
OCCT_INC="$OCCT_PREFIX/include/opencascade"
OCCT_LIB="$OCCT_PREFIX/lib"
BUILD="$REPO/build-mac"

if [[ ! -d "$OCCT_INC" ]]; then
  echo "error: OCCT headers not found at $OCCT_INC" >&2
  echo "install with: brew install opencascade" >&2
  exit 1
fi
if [[ ! -d "$OCCT_LIB" ]]; then
  echo "error: OCCT libraries not found at $OCCT_LIB" >&2
  exit 1
fi

[[ "${CLEAN:-0}" == "1" ]] && rm -rf "$BUILD"
mkdir -p "$BUILD"

echo "==> configure (CYBERCAD_MACOS_OCCT=ON, Homebrew OCCT at $OCCT_PREFIX)"
cmake -S "$REPO" -B "$BUILD" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCYBERCAD_MACOS_OCCT=ON \
  -DCYBERCAD_HOMEBREW_OCCT_PREFIX="$OCCT_PREFIX" \
  -DCYBERCAD_BUILD_TESTS=OFF

echo "==> build cybercadkernel"
cmake --build "$BUILD" --target cybercadkernel -- -j"$(sysctl -n hw.ncpu 2>/dev/null || echo 4)"

# CMake names a SHARED lib libcybercadkernel.dylib; land it directly under
# build-mac/ regardless of any per-config subdir the generator may use.
DYLIB="$(find "$BUILD" -name 'libcybercadkernel.dylib' -type f | head -n1)"
if [[ -z "$DYLIB" ]]; then
  echo "error: libcybercadkernel.dylib not produced" >&2
  exit 1
fi
if [[ "$DYLIB" != "$BUILD/libcybercadkernel.dylib" ]]; then
  cp -f "$DYLIB" "$BUILD/libcybercadkernel.dylib"
fi

echo "done: $BUILD/libcybercadkernel.dylib"
otool -L "$BUILD/libcybercadkernel.dylib" | sed -n '1,3p'
