#!/usr/bin/env bash
#
# build-numsci.sh — build NumPP + the SciPP optimize/linalg subset as a single
# static archive the kernel links for HOST or the arm64 iOS simulator.
#
# This is the OCCT-free numeric substrate for Phase-4 #2 (numeric-foundations).
# It reproduces the eval-verified recipe (docs/EVAL-numpp-scipp.md, eval/REPRO.md):
#   * CPU-only — every NUMPP_WITH_* / SCIPP_WITH_* backend flag is 0 (no BLAS /
#     LAPACK / Metal / CUDA / OpenCL / Vulkan). The linalg *_null backend stubs
#     satisfy the solve/lstsq/svd/qr vtable with no external LAPACK.
#   * SciPP src/special + src/stats are EXCLUDED — they call C++17 ISO-29124
#     special-math (std::legendre / cyl_bessel_j / expint) that Homebrew/iOS
#     libc++ does not implement. The kernel does not use them.
#   * NumPP + SciPP are referenced BY ABSOLUTE PATH (like OCCT), never vendored.
#
# Outputs (under the chosen --out dir, default <kernel>/build-numsci):
#   gen/numpp/backend/config.hpp, gen/numpp/version.hpp
#   gen/scipp/config.hpp,         gen/scipp/version.hpp
#   libnumsci_<target>.a
#
# The kernel CMake (CYBERCAD_HAS_NUMSCI=ON) consumes both the gen/ include tree
# and libnumsci_<target>.a via -DCYBERCAD_NUMSCI_DIR=<out>.
#
# Usage:
#   scripts/build-numsci.sh host        # Homebrew LLVM clang++, libc++
#   scripts/build-numsci.sh iossim      # arm64-apple-ios*-simulator
#   scripts/build-numsci.sh host --out /path/to/out
#
set -uo pipefail

TARGET="${1:-host}"
shift || true

NUMPP="${NUMPP_DIR:-/Users/leonardoaraujo/work/NumPP}"
SCIPP="${SCIPP_DIR:-/Users/leonardoaraujo/work/SciPP}"
KERNEL="$(cd "$(dirname "$0")/.." && pwd)"
OUT=""
IOS_VERSION="16.0"

while [ $# -gt 0 ]; do
  case "$1" in
    --out) OUT="$2"; shift 2;;
    --numpp) NUMPP="$2"; shift 2;;
    --scipp) SCIPP="$2"; shift 2;;
    *) echo "unknown arg: $1"; exit 2;;
  esac
done

[ -d "$NUMPP/include/numpp" ] || { echo "NumPP not found at $NUMPP"; exit 1; }
[ -d "$SCIPP/include/scipp" ] || { echo "SciPP not found at $SCIPP"; exit 1; }

case "$TARGET" in
  host)
    CXX="${CXX:-/opt/homebrew/opt/llvm/bin/clang++}"
    AR="ar"
    TARGET_FLAGS=(-std=c++20 -stdlib=libc++ -O2 -fvisibility=hidden)
    ARCHIVE="libnumsci_host.a"
    [ -n "$OUT" ] || OUT="$KERNEL/build-numsci/host"
    ;;
  iossim)
    SDK="$(xcrun --sdk iphonesimulator --show-sdk-path)"
    CXX="$(xcrun --sdk iphonesimulator --find clang++)"
    AR="$(xcrun --sdk iphonesimulator --find libtool)"
    TARGET_FLAGS=(-target "arm64-apple-ios${IOS_VERSION}-simulator" -isysroot "$SDK"
                  -std=c++20 -stdlib=libc++ -O2 -fvisibility=hidden)
    ARCHIVE="libnumsci_iossim_arm64.a"
    [ -n "$OUT" ] || OUT="$KERNEL/build-numsci/iossim"
    ;;
  *) echo "usage: $0 {host|iossim} [--out DIR] [--numpp DIR] [--scipp DIR]"; exit 2;;
esac

GEN="$OUT/gen"; OBJ="$OUT/obj"; LOG="$OUT/log"
mkdir -p "$GEN/numpp/backend" "$GEN/scipp" "$OBJ" "$LOG"

# ── generated config / version headers (all backends OFF) ──────────────────────
cat > "$GEN/numpp/backend/config.hpp" <<'EOF'
#pragma once
#define NUMPP_WITH_BLAS 0
#define NUMPP_WITH_LAPACK 0
#define NUMPP_WITH_METAL 0
#define NUMPP_WITH_VULKAN 0
#define NUMPP_WITH_CUDA 0
#define NUMPP_WITH_OPENCL 0
EOF
# Mirror NumPP's version.hpp (constants only; used for informational reporting).
{
  echo '#pragma once'
  echo '#define NUMPP_VERSION_MAJOR 1'
  echo '#define NUMPP_VERSION_MINOR 6'
  echo '#define NUMPP_VERSION_PATCH 0'
  echo '#define NUMPP_VERSION_STRING "1.6.0"'
  echo 'namespace numpp { inline constexpr const char* version() { return NUMPP_VERSION_STRING; } }'
} > "$GEN/numpp/version.hpp"

cat > "$GEN/scipp/config.hpp" <<'EOF'
#pragma once
#define SCIPP_WITH_BLAS 0
#define SCIPP_WITH_LAPACK 0
#define SCIPP_WITH_CUDA 0
#define SCIPP_WITH_OPENCL 0
#define SCIPP_WITH_METAL 0
EOF
{
  echo '#pragma once'
  echo '#define SCIPP_VERSION_MAJOR 1'
  echo '#define SCIPP_VERSION_MINOR 2'
  echo '#define SCIPP_VERSION_PATCH 0'
  echo '#define SCIPP_VERSION_STRING "1.2.0"'
  echo 'namespace scipp {'
  echo 'inline constexpr int version_major = SCIPP_VERSION_MAJOR;'
  echo 'inline constexpr int version_minor = SCIPP_VERSION_MINOR;'
  echo 'inline constexpr int version_patch = SCIPP_VERSION_PATCH;'
  echo 'inline constexpr const char* version_string = SCIPP_VERSION_STRING;'
  echo '}'
} > "$GEN/scipp/version.hpp"

INC=(-I "$NUMPP/include" -I "$SCIPP/include" -I "$GEN")
COMPILE_FLAGS=("${TARGET_FLAGS[@]}" -DNUMPP_BUILDING -Wall -Wextra -c)

# ── translation units (eval-verified working set) ─────────────────────────────
# FULL NumPP (CPU-only incl. the *_null backend stubs) — from src/CMakeLists.txt.
NUMPP_TUS=(
  core/dtype.cpp core/casting.cpp core/shape.cpp core/ndarray.cpp core/creation.cpp core/limits.cpp
  manip/manip.cpp manip/manip_extra.cpp
  stats/stats.cpp stats/stats_extra.cpp stats/integrate.cpp
  grids/grids.cpp construct/construct.cpp
  indexing/indexing.cpp indexing/indexing_extra.cpp
  mathx/mathx.cpp mathx/typecheck.cpp mathx/closeto.cpp
  poly/poly.cpp poly/poly1d.cpp polynomial/polynomial.cpp
  tensor/tensor.cpp umath/ufunc.cpp umath/errstate.cpp
  linalg/linalg.cpp linalg/batched.cpp linalg/tensor_solve.cpp linalg/array_api.cpp
  fft/fft.cpp
  random/random.cpp random/distributions.cpp random/distributions2.cpp random/distributions3.cpp random/bitgenerators.cpp
  polynomial/orthopoly.cpp polynomial/orthopoly_calc.cpp polynomial/poly_classes.cpp
  strings/char_extra.cpp strings/char_extra2.cpp
  io/printopts.cpp
  lib/striding.cpp lib/striding2.cpp lib/memory_overlap.cpp lib/iteration.cpp
  io/npy.cpp textio/textio.cpp io/npz.cpp io/format.cpp
  sorting/sorting.cpp sorting/sorting_extra.cpp
  strings/strings.cpp strings/char.cpp strings/string_dtype.cpp
  datetime/datetime.cpp datetime/busday.cpp
  testing/testing.cpp
  ma/masked.cpp ma/masked_arith.cpp ma/masked_reduce.cpp ma/masked_hardsoft.cpp
  struct/struct.cpp interop/interop.cpp backend/backend.cpp
  backend/blas_null.cpp backend/gpu_null.cpp backend/lapack_null.cpp
)
# SciPP: optimize + linalg ONLY (special + stats EXCLUDED — libc++ ISO-29124 gap).
SCIPP_TUS=(
  optimize/roots_scalar.cpp optimize/minimize_scalar.cpp optimize/minimize.cpp
  optimize/least_squares.cpp optimize/linprog.cpp optimize/nnls.cpp
  linalg/basic.cpp linalg/decomp.cpp linalg/eigen.cpp linalg/matfuncs.cpp linalg/special.cpp
)

ok=0; fail=0; OBJS=(); declare -a FAILED
compile() {
  local root="$1" rel="$2" base out log
  base="$(echo "$rel" | tr '/' '_').o"; out="$OBJ/$base"; log="$LOG/${base}.log"
  if "$CXX" "${COMPILE_FLAGS[@]}" "${INC[@]}" "$root/src/$rel" -o "$out" >"$log" 2>&1; then
    ok=$((ok+1)); OBJS+=("$out")
  else
    echo "FAIL $rel  (see $log)"; fail=$((fail+1)); FAILED+=("$rel")
  fi
}

echo "[build-numsci] target=$TARGET  cxx=$CXX"
echo "[build-numsci] NumPP=$NUMPP  SciPP=$SCIPP  out=$OUT"
for t in "${NUMPP_TUS[@]}"; do compile "$NUMPP" "$t"; done
for t in "${SCIPP_TUS[@]}"; do compile "$SCIPP" "$t"; done
echo "[build-numsci] compiled OK=$ok FAIL=$fail (${#NUMPP_TUS[@]} NumPP + ${#SCIPP_TUS[@]} SciPP TUs)"
if [ "$fail" -gt 0 ]; then printf '  FAILED: %s\n' "${FAILED[@]}"; exit 1; fi

rm -f "$OUT/$ARCHIVE"
case "$TARGET" in
  host)   "$AR" -rcs "$OUT/$ARCHIVE" "${OBJS[@]}" >"$LOG/archive.log" 2>&1;;
  iossim) "$AR" -static -o "$OUT/$ARCHIVE" "${OBJS[@]}" >"$LOG/archive.log" 2>&1;;
esac
if [ $? -eq 0 ]; then
  echo "[build-numsci] archived -> $OUT/$ARCHIVE"
else
  echo "[build-numsci] archive FAILED"; cat "$LOG/archive.log"; exit 1
fi
echo "[build-numsci] DONE. Configure the kernel with:"
echo "    -DCYBERCAD_HAS_NUMSCI=ON -DCYBERCAD_NUMSCI_DIR=$OUT \\"
echo "    -DCYBERCAD_NUMPP_DIR=$NUMPP -DCYBERCAD_SCIPP_DIR=$SCIPP"
