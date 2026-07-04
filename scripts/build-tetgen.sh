#!/usr/bin/env bash
#
# build-tetgen.sh — build the external TetGen library as a single static archive
# the kernel links ONLY when CYBERCAD_HAS_TETGEN=ON (the optional AGPL tet-mesher
# backend, GitHub issue #1).
#
# TetGen is OPTIONAL, EXTERNAL and licensed AGPL-3.0. Its sources live OUTSIDE this
# repo (default /home/leonardo/work/tetgen) and are referenced BY ABSOLUTE PATH —
# they are NEVER copied or vendored into CyberCadKernel. Shipping a closed
# application that links the resulting archive requires a TetGen commercial license.
# The default MIT kernel build never runs this script and never links libtetgen.
#
# Verified recipe (FP robustness): predicates.cxx at -O0 (exact-arithmetic
# predicates must not be reordered by the optimizer), tetgen.cxx at -O2, both with
# -DTETLIBRARY so tetgenio/tetrahedralize are exposed as a library API.
#
# Output (under the chosen --out dir, default <kernel>/build-tet/host):
#   libtetgen_<target>.a
# The kernel CMake (CYBERCAD_HAS_TETGEN=ON) consumes it via
#   -DCYBERCAD_TETGEN_DIR=<out> -DCYBERCAD_TETGEN_SRC_DIR=<tetgen source root>
#
# Usage:
#   scripts/build-tetgen.sh host                  # clang++ (Linux host)
#   scripts/build-tetgen.sh host --out /path/out
#   scripts/build-tetgen.sh host --tetgen /path/to/tetgen
#
set -uo pipefail

TARGET="${1:-host}"
shift || true

TETGEN="${TETGEN_DIR:-/home/leonardo/work/tetgen}"
KERNEL="$(cd "$(dirname "$0")/.." && pwd)"
OUT=""

while [ $# -gt 0 ]; do
  case "$1" in
    --out) OUT="$2"; shift 2;;
    --tetgen) TETGEN="$2"; shift 2;;
    *) echo "unknown arg: $1"; exit 2;;
  esac
done

[ -f "$TETGEN/tetgen.cxx" ] || { echo "TetGen sources not found at $TETGEN"; exit 1; }
[ -f "$TETGEN/predicates.cxx" ] || { echo "predicates.cxx not found at $TETGEN"; exit 1; }
[ -f "$TETGEN/tetgen.h" ] || { echo "tetgen.h not found at $TETGEN"; exit 1; }

case "$TARGET" in
  host)
    CXX="${CXX:-clang++}"
    AR="${AR:-ar}"
    ARCHIVE="libtetgen_host.a"
    [ -n "$OUT" ] || OUT="$KERNEL/build-tet/host"
    ;;
  *) echo "usage: $0 {host} [--out DIR] [--tetgen DIR]"; exit 2;;
esac

OBJ="$OUT/obj"; LOG="$OUT/log"
mkdir -p "$OBJ" "$LOG"

echo "[build-tetgen] target=$TARGET  cxx=$CXX"
echo "[build-tetgen] TetGen=$TETGEN  out=$OUT"

# tetgen.cxx at -O2, predicates.cxx at -O0 (exact-FP robustness). Both -DTETLIBRARY.
# Sources referenced by absolute path — never copied into the repo.
if ! "$CXX" -std=c++20 -O2 -DTETLIBRARY -I"$TETGEN" -c "$TETGEN/tetgen.cxx" \
      -o "$OBJ/tetgen.o" >"$LOG/tetgen.log" 2>&1; then
  echo "[build-tetgen] compile FAILED (tetgen.cxx); see $LOG/tetgen.log"
  cat "$LOG/tetgen.log"; exit 1
fi
if ! "$CXX" -std=c++20 -O0 -DTETLIBRARY -I"$TETGEN" -c "$TETGEN/predicates.cxx" \
      -o "$OBJ/predicates.o" >"$LOG/predicates.log" 2>&1; then
  echo "[build-tetgen] compile FAILED (predicates.cxx); see $LOG/predicates.log"
  cat "$LOG/predicates.log"; exit 1
fi

rm -f "$OUT/$ARCHIVE"
if "$AR" rcs "$OUT/$ARCHIVE" "$OBJ/tetgen.o" "$OBJ/predicates.o" >"$LOG/archive.log" 2>&1; then
  echo "[build-tetgen] archived -> $OUT/$ARCHIVE"
else
  echo "[build-tetgen] archive FAILED"; cat "$LOG/archive.log"; exit 1
fi

echo "[build-tetgen] DONE. Configure the kernel with:"
echo "    -DCYBERCAD_HAS_TETGEN=ON -DCYBERCAD_TETGEN_DIR=$OUT \\"
echo "    -DCYBERCAD_TETGEN_SRC_DIR=$TETGEN"
