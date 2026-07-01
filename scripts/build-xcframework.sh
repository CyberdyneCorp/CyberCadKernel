#!/usr/bin/env bash
# Build CyberCadKernel as an iOS xcframework (device arm64 + simulator arm64),
# WITH the OCCT adapter, so the CyberCad app can link it in place of its inline
# KernelBridge.mm (the link-swap, add-kernel-foundation task 6.1). Compile-only
# packaging: it cannot run here (OCCT libs are iOS slices). Rebuild locally —
# the .xcframework is a build artifact and is gitignored.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT_ROOT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}"
OUT="$REPO/build-ios"
MIN_IOS="${MIN_IOS:-13.0}"

# One slice: $1 platform tag (SIMULATORARM64|OS64), $2 clang target, $3 sdk name
build_slice() {
  local tag="$1" target="$2" sdk="$3"
  local inc="$OCCT_ROOT/install-$tag/include/opencascade"
  local sysroot; sysroot="$(xcrun --sdk "$sdk" --show-sdk-path)"
  local objdir="$OUT/obj-$tag"; rm -rf "$objdir"; mkdir -p "$objdir"
  echo "── slice $tag ($target, sdk=$sdk)"
  [ -d "$inc" ] || { echo "missing OCCT headers: $inc" >&2; exit 1; }
  local n=0
  while IFS= read -r src; do
    xcrun --sdk "$sdk" clang++ -target "$target" -isysroot "$sysroot" \
      -std=c++20 -Os -fvisibility=hidden -DCYBERCAD_HAS_OCCT \
      -I"$REPO/include" -I"$REPO/src" -I"$inc" \
      -c "$src" -o "$objdir/$(basename "${src%.cpp}").o"
    n=$((n+1))
  done < <(find "$REPO/src" -name '*.cpp' | sort)
  ar -crs "$OUT/libcybercadkernel-$tag.a" "$objdir"/*.o
  echo "   $n TUs -> libcybercadkernel-$tag.a"
}

rm -rf "$OUT/CyberCadKernel.xcframework"
mkdir -p "$OUT"
build_slice SIMULATORARM64 "arm64-apple-ios${MIN_IOS}-simulator" iphonesimulator
build_slice OS64           "arm64-apple-ios${MIN_IOS}"           iphoneos

xcodebuild -create-xcframework \
  -library "$OUT/libcybercadkernel-SIMULATORARM64.a" -headers "$REPO/include" \
  -library "$OUT/libcybercadkernel-OS64.a"           -headers "$REPO/include" \
  -output "$OUT/CyberCadKernel.xcframework"

echo "OK: $OUT/CyberCadKernel.xcframework"
