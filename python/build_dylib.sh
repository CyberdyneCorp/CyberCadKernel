#!/usr/bin/env bash
# Build the desktop (macOS arm64) shared library the Python binding loads.
#
# This codifies the PROVEN manual recipe: compile every src/**/*.cpp EXCEPT the
# Metal TUs (src/compute/metal/*) with -DCYBERCAD_HAS_OCCT against Homebrew OCCT,
# then link a dylib against the TK* libraries. Metal is EXCLUDED on macOS
# (no -DCYBERCAD_HAS_METAL); cc_set_gpu_tessellation is a safe no-op there.
#
# Output: build-mac/libcybercadkernel.dylib (loaded by cybercadkernel._ffi).
#
# Usage:  python/build_dylib.sh            # incremental
#         CLEAN=1 python/build_dylib.sh    # from scratch
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OCCT_PREFIX="${OCCT_PREFIX:-/opt/homebrew/opt/opencascade}"
OCCT_INC="$OCCT_PREFIX/include/opencascade"
OCCT_LIB="$OCCT_PREFIX/lib"
BUILD="$REPO/build-mac"
OBJ="$BUILD/obj"

if [[ ! -d "$OCCT_INC" ]]; then
  echo "error: OCCT headers not found at $OCCT_INC" >&2
  echo "install with: brew install opencascade" >&2
  exit 1
fi

[[ "${CLEAN:-0}" == "1" ]] && rm -rf "$BUILD"
mkdir -p "$OBJ"

CXXFLAGS=(-std=c++20 -DCYBERCAD_HAS_OCCT -arch arm64 -fPIC -O2
  -I "$REPO/include" -I "$REPO/src" -I "$OCCT_INC")

# Compile every C++ TU under src/ EXCEPT the Metal backend (macOS excludes it).
# Use a while-read loop: zsh does NOT word-split unquoted vars, and object names
# are flattened so files with the same basename in different dirs don't collide.
objs=()
count=0
while IFS= read -r src; do
  case "$src" in
    "$REPO"/src/compute/metal/*) continue ;;
  esac
  rel="${src#"$REPO"/src/}"
  obj="$OBJ/${rel//\//__}.o"
  if [[ ! -f "$obj" || "$src" -nt "$obj" ]]; then
    echo "  CXX $rel"
    clang++ "${CXXFLAGS[@]}" -c "$src" -o "$obj"
  fi
  objs+=("$obj")
  count=$((count + 1))
done < <(find "$REPO/src" -name '*.cpp' | sort)

echo "compiled/collected $count translation units"

TKLIBS=(-lTKDESTEP -lTKDEIGES -lTKXSBase -lTKDE -lTKXCAF -lTKLCAF -lTKCAF
  -lTKCDF -lTKMesh -lTKShHealing -lTKOffset -lTKFillet -lTKBool -lTKPrim
  -lTKBO -lTKTopAlgo -lTKGeomAlgo -lTKBRep -lTKGeomBase -lTKG3d -lTKG2d
  -lTKMath -lTKernel)

echo "  LINK libcybercadkernel.dylib"
clang++ -dynamiclib -arch arm64 "${objs[@]}" \
  -L"$OCCT_LIB" "${TKLIBS[@]}" -lc++ \
  -Wl,-rpath,"$OCCT_LIB" \
  -o "$BUILD/libcybercadkernel.dylib"

echo "done: $BUILD/libcybercadkernel.dylib"
