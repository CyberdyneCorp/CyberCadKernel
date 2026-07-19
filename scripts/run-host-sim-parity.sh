#!/usr/bin/env bash
#
# run-host-sim-parity.sh — run a tests/sim native-vs-OCCT parity harness on the LINUX HOST.
#
# Most of the sim harnesses are pure C++ despite the .mm extension (92 of 94 carry no #import /
# @interface / Foundation / Metal), so they compile unchanged against a Linux OCCT. This is the
# generic port of scripts/run-host-native-ssi-marching.sh, which did it for one harness.
#
#   scripts/run-host-sim-parity.sh native_curved_wall_cut_parity
#   scripts/run-host-sim-parity.sh native_ssi_marching_parity
#   scripts/run-host-sim-parity.sh --list
#
# WHY A SOURCE-BUILT OCCT. Debian/Ubuntu's libocct-foundation-dev 7.6.3 ships
# Poly_ArrayOfNodes.hxx (a 7.7-era header) but NOT its dependency NCollection_AliasedArray.hxx.
# Poly_Triangulation.hxx includes it, so every mesh-touching harness fails to compile against the
# distro package — 6 of 94 harnesses, but exactly the mesher/boolean gates. Upstream OCCT-7.6 does
# ship the header. Build it with scripts/../ (see OCCT_ROOT below):
#     git clone --depth 1 --branch OCCT-7.6 https://github.com/Open-Cascade-SAS/OCCT.git
#     cmake -S . -B build -DBUILD_MODULE_Visualization=OFF -DBUILD_MODULE_Draw=OFF \
#           -DBUILD_MODULE_ApplicationFramework=OFF -DUSE_FREETYPE=OFF -DUSE_TCL=OFF ...
# Point OCCT_ROOT at the install prefix. Falls back to the system OCCT, which is fine for
# harnesses that do not touch Poly_Triangulation.
#
# NOT a replacement for the simulator gate. The OCCT build and the toolchain/ABI both differ from
# the trimmed arm64-iOS OCCT the sim links, so this is a fast pre-check that catches real
# regressions in minutes; confirm on the simulator before archiving a change.
#
set -uo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$REPO/build-host-parity"; mkdir -p "$OUT"
CXX="${CXX:-g++}"

if [ "${1:-}" = "--list" ]; then
  echo "sim harnesses that carry no Objective-C / Metal (host-portable candidates):"
  for f in "$REPO"/tests/sim/*.mm; do
    grep -qE "#import|@interface|@autoreleasepool|NSString|<Foundation/|<Metal/" "$f" || \
      echo "  $(basename "$f" .mm)"
  done
  exit 0
fi

HARNESS_NAME="${1:-}"
[ -n "$HARNESS_NAME" ] || { echo "usage: $0 <harness-name> | --list"; exit 2; }
HARNESS="$REPO/tests/sim/${HARNESS_NAME%.mm}.mm"
[ -f "$HARNESS" ] || { echo "missing harness: $HARNESS"; exit 1; }
if grep -qE "#import|@interface|@autoreleasepool|NSString|<Foundation/|<Metal/" "$HARNESS"; then
  echo "$HARNESS_NAME uses Objective-C / Metal — simulator only, cannot run on the host."; exit 1
fi

# ── OCCT: prefer a source build (see header), else the system package ────────────
OCCT_ROOT="${OCCT_ROOT:-/home/leonardo/work/occt-build/install}"
if [ -d "$OCCT_ROOT/include/opencascade" ]; then
  OCCT_INC="$OCCT_ROOT/include/opencascade"; OCCT_LIBDIR="$OCCT_ROOT/lib"
elif [ -d /usr/include/opencascade ]; then
  OCCT_INC=/usr/include/opencascade; OCCT_LIBDIR=""
  if grep -qE "Poly_Triangulation|BRepMesh|Poly_Array" "$HARNESS"; then
    echo "WARNING: $HARNESS_NAME touches Poly_Triangulation/BRepMesh and only the system OCCT was"
    echo "         found. Debian 7.6.3 omits NCollection_AliasedArray.hxx, so this will not compile."
    echo "         Build OCCT from source and set OCCT_ROOT — see the header of this script."
  fi
else
  echo "no OCCT found (set OCCT_ROOT, or apt install libocct-*-dev)"; exit 1
fi

# ── numsci substrate ─────────────────────────────────────────────────────────────
if [ -z "${CYBERCAD_NUMSCI_DIR:-}" ] && [ ! -d "$REPO/build-numsci/linuxhost/gen" ]; then
  "$REPO/scripts/build-numsci.sh" linuxhost || exit 1
fi
NUMSCI_DIR="${CYBERCAD_NUMSCI_DIR:-$REPO/build-numsci/linuxhost}"
NUMSCI_GEN="$NUMSCI_DIR/gen"
NUMSCI_LIB="$(ls "$NUMSCI_DIR"/libnumsci_*.a 2>/dev/null | head -1)"
[ -n "$NUMSCI_LIB" ] || { echo "numsci archive not found. Run scripts/build-numsci.sh linuxhost"; exit 1; }
NUMPP="${NUMPP_DIR:-/home/leonardo/work/NumPP}/include"
SCIPP="${SCIPP_DIR:-/home/leonardo/work/SciPP}/include"

# ── kernel code: prefer the OCCT-linked archive, else compile the native TU subset.
#
# A harness that drives the cc_* facade needs the WHOLE kernel with a live engine adapter — the
# native math/ssi TUs alone do not link it, and even when they do the facade falls back to the
# stub and every operation returns engine_unsupported. Build that archive with:
#     cmake -S . -B build-host-occt -DCYBERCAD_LINUX_OCCT=ON -DCYBERCAD_HAS_NUMSCI=ON ...
#     cmake --build build-host-occt --target cybercadkernel
# Harnesses that only touch the native layer work either way, so the archive is preferred
# whenever it exists and the TU subset remains the no-setup fallback.
KERNEL_ARCHIVE="$REPO/build-host-occt/libcybercadkernel.a"
NATIVE_SRCS=()
if [ -f "$KERNEL_ARCHIVE" ]; then
  echo "   kernel : $KERNEL_ARCHIVE (OCCT adapter live)"
  # STALENESS GUARD. The harness compiles against the CURRENT headers but links this PREBUILT
  # archive. If a header changed a struct layout since the archive was built, the two disagree
  # and the result is a SEGFAULT at run time with no diagnostic — which is exactly what happened
  # when SurfaceAdapter gained a field. Fail loudly instead.
  NEWER="$(find "$REPO/src" -name '*.h' -newer "$KERNEL_ARCHIVE" -print -quit 2>/dev/null)"
  if [ -n "$NEWER" ]; then
    echo "── STALE ARCHIVE: $(basename "$NEWER") is newer than libcybercadkernel.a."
    echo "   Layouts may disagree and the harness would segfault. Rebuild first:"
    echo "     cmake --build build-host-occt --target cybercadkernel -j\$(nproc)"
    exit 1
  fi
else
  KERNEL_ARCHIVE=""
  while IFS= read -r src; do NATIVE_SRCS+=("$src"); done \
    < <(find "$REPO/src/native/math" -name '*.cpp'; \
        echo "$REPO/src/native/ssi/seeding.cpp"; \
        echo "$REPO/src/native/ssi/marching.cpp"; \
        echo "$REPO/src/native/numerics/numerics.cpp")
  echo "   kernel : ${#NATIVE_SRCS[@]} native TU(s) — no build-host-occt archive, cc_* facade will STUB"
fi

# ── OCCT toolkits: union of the slices the sim scripts use, most-derived → base.
# TKFillet/TKFeat/TKOffset carry BRepFilletAPI_MakeChamfer + ChFi3d_*, TKHLR the HLRBRep_* hidden-
# line removal a display-mesh harness needs. The data-exchange slice is appended only on demand,
# so the common case does not pay for it.
TKS="TKFillet TKFeat TKOffset TKHLR TKMesh TKShHealing TKBool TKPrim TKBO TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"
# The data-exchange slice is needed when the HARNESS names STEP/IGES, and ALWAYS when the kernel
# archive is linked — src/engine/occt/occt_exchange.cpp pulls STEPControl_/IGESControl_ regardless
# of what the harness itself mentions.
if [ -n "$KERNEL_ARCHIVE" ] || grep -qE "STEPControl|IGESControl|XCAF|STEPCAF" "$HARNESS"; then
  TKS="$TKS TKXDESTEP TKXDEIGES TKSTEP TKSTEP209 TKSTEPAttr TKSTEPBase TKIGES TKXSBase TKXCAF TKLCAF TKCAF TKCDF"
fi
LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done
[ -n "$OCCT_LIBDIR" ] && LFLAGS="-L$OCCT_LIBDIR $LFLAGS"

echo "── compiling $HARNESS_NAME for LINUX HOST"
echo "   oracle : $OCCT_INC"
"$CXX" -std=c++20 -O2 -w \
  -DCYBERCAD_HAS_OCCT -DCYBERCAD_HAS_NUMSCI=1 \
  -I"$REPO/src" -I"$REPO/tests" -I"$REPO/include" -I"$OCCT_INC" \
  -I"$NUMSCI_GEN" -I"$NUMPP" -I"$SCIPP" \
  -x c++ "$HARNESS" ${NATIVE_SRCS[@]+"${NATIVE_SRCS[@]}"} \
  -x none ${KERNEL_ARCHIVE:+"$KERNEL_ARCHIVE"} "$NUMSCI_LIB" \
  $LFLAGS \
  -o "$OUT/$HARNESS_NAME" || { echo "── COMPILE FAILED"; exit 1; }

echo "── running"
[ -n "$OCCT_LIBDIR" ] && export LD_LIBRARY_PATH="$OCCT_LIBDIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
"$OUT/$HARNESS_NAME"
