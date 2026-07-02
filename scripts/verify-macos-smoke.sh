#!/usr/bin/env bash
# Compile and run the tiny C smoke test against build-mac/libcybercadkernel.dylib.
# Asserts REAL geometry: brep_available=1, box volume=1000, cut volume=875.
set -euo pipefail

if [[ -n "${BASH_SOURCE:-}" ]]; then
  SELF="${BASH_SOURCE[0]}"
else
  SELF="${(%):-%N}"
fi
SCRIPT_DIR="$(cd "$(dirname "$SELF")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"

BUILD="$REPO/build-mac"
DYLIB="$BUILD/libcybercadkernel.dylib"
SMOKE_SRC="$SCRIPT_DIR/smoke-macos-dylib.c"
SMOKE_BIN="$BUILD/smoke-macos-dylib"

if [[ ! -f "$DYLIB" ]]; then
  echo "error: $DYLIB not found; run scripts/build-macos-dylib.sh first" >&2
  exit 1
fi

echo "==> compile smoke test"
clang "$SMOKE_SRC" \
  -I "$REPO/include" \
  -L "$BUILD" -lcybercadkernel \
  -Wl,-rpath,"$BUILD" \
  -o "$SMOKE_BIN"

echo "==> run smoke test"
"$SMOKE_BIN"
