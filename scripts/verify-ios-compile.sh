#!/usr/bin/env bash
#
# verify-ios-compile.sh
#
# Verifies that the CyberCadKernel OCCT adapter COMPILES AND LINKS for the iOS
# simulator (arm64) against the real, cross-compiled OCCT static libraries.
#
# It cannot RUN anything (the OCCT libs are iOS-simulator slices, not host
# libs). It only proves the adapter compiles against real OCCT headers and that
# the objects archive/link.
#
# What it does:
#   1. Compiles every .cpp that participates in the WITH-OCCT build
#      (facade + core + engine/* including engine/occt/*), with
#      -DCYBERCAD_HAS_OCCT, targeting arm64-apple-ios13.0-simulator.
#   2. Archives all objects into build-ios/libcybercadkernel.a.
#   3. Link check: compiles a tiny main that calls cc_brep_available() and links
#      it against the archive + the OCCT TK* libs (best-effort; reported
#      separately from the compile/archive gate).
#
# Exit code: 0 iff every TU compiled AND the archive built. The executable link
# is a best-effort extra and does not gate the exit code.

set -u

ROOT="/Users/leonardoaraujo/work/CyberCadKernel"
OCCT_ROOT="/Users/leonardoaraujo/work/cybercad/build/occt/install-SIMULATORARM64"
OCCT_INC="${OCCT_ROOT}/include/opencascade"
OCCT_LIB="${OCCT_ROOT}/lib"
BUILD="${ROOT}/build-ios"

SDK_PATH="$(xcrun --sdk iphonesimulator --show-sdk-path)"

CXX=(xcrun --sdk iphonesimulator clang++)
COMMON_FLAGS=(
  -target arm64-apple-ios13.0-simulator
  -isysroot "${SDK_PATH}"
  -std=c++20
  -DCYBERCAD_HAS_OCCT
  -I"${ROOT}/include"
  -I"${ROOT}/src"
  -I"${OCCT_INC}"
)

# TK* toolkits per /Users/leonardoaraujo/work/cybercad/docs/occt-build.md
# (core set + STEP/IGES exchange), plus -lc++.
TK_LIBS=(
  -lTKernel -lTKMath -lTKG2d -lTKG3d -lTKGeomBase -lTKBRep
  -lTKGeomAlgo -lTKTopAlgo -lTKBO -lTKPrim -lTKBool -lTKFillet
  -lTKOffset -lTKShHealing -lTKMesh
  -lTKDESTEP -lTKXSBase -lTKDEIGES
  -lc++
)

rm -rf "${BUILD}"
mkdir -p "${BUILD}"

# Sanity: OCCT headers/libs must be present.
if [[ ! -d "${OCCT_INC}" ]]; then
  echo "FATAL: OCCT include dir not found: ${OCCT_INC}"
  exit 2
fi
if [[ ! -d "${OCCT_LIB}" ]]; then
  echo "FATAL: OCCT lib dir not found: ${OCCT_LIB}"
  exit 2
fi

# Collect every WITH-OCCT translation unit: facade + core + engine (incl occt).
# In the WITH-OCCT build all of src/ is compiled with -DCYBERCAD_HAS_OCCT
# (the stub stays compiled but its create_default_engine() is #ifdef'd out).
SOURCES=()
while IFS= read -r line; do
  SOURCES+=("${line}")
done < <(find "${ROOT}/src" -name '*.cpp' | sort)

echo "=== Compiling ${#SOURCES[@]} translation units (arm64 iphonesimulator, -DCYBERCAD_HAS_OCCT) ==="

OBJECTS=()
FAIL=0
for src in "${SOURCES[@]}"; do
  rel="${src#${ROOT}/src/}"
  obj="${BUILD}/$(echo "${rel}" | tr '/' '_').o"
  obj="${obj%.cpp.o}.o"
  printf '  CXX %s\n' "${rel}"
  if ! "${CXX[@]}" "${COMMON_FLAGS[@]}" -c "${src}" -o "${obj}"; then
    echo "!!! COMPILE FAILED: ${rel}"
    FAIL=1
  else
    OBJECTS+=("${obj}")
  fi
done

if [[ ${FAIL} -ne 0 ]]; then
  echo "=== RESULT: COMPILE FAILURE ==="
  exit 1
fi

echo "=== All ${#OBJECTS[@]} TUs compiled. Archiving ==="
ARCHIVE="${BUILD}/libcybercadkernel.a"
if ! xcrun --sdk iphonesimulator ar rcs "${ARCHIVE}" "${OBJECTS[@]}"; then
  echo "=== RESULT: ARCHIVE FAILURE ==="
  exit 1
fi
xcrun --sdk iphonesimulator ranlib "${ARCHIVE}" >/dev/null 2>&1 || true
echo "Archive built: ${ARCHIVE}"

echo "=== Link check: tiny main -> cc_brep_available() linked against archive + OCCT ==="
MAIN_SRC="${BUILD}/linkcheck_main.cpp"
cat > "${MAIN_SRC}" <<'EOF'
#include "cybercadkernel/cc_kernel.h"
int main(void) {
    return cc_brep_available();
}
EOF

LINK_STATUS="UNKNOWN"
LINK_LOG="${BUILD}/linkcheck.log"
if "${CXX[@]}" "${COMMON_FLAGS[@]}" \
    "${MAIN_SRC}" \
    "${ARCHIVE}" \
    -L"${OCCT_LIB}" "${TK_LIBS[@]}" \
    -o "${BUILD}/linkcheck" > "${LINK_LOG}" 2>&1; then
  LINK_STATUS="LINK OK (executable produced: ${BUILD}/linkcheck)"
else
  LINK_STATUS="LINK FAILED (see ${LINK_LOG})"
fi

echo "=== RESULT: COMPILE+ARCHIVE OK ==="
echo "Link check: ${LINK_STATUS}"
echo "----- link log (tail) -----"
[[ -f "${LINK_LOG}" ]] && tail -n 40 "${LINK_LOG}"
exit 0
