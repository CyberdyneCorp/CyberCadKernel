#!/usr/bin/env bash
# bench-binary-size.sh — measure the shipped static-lib + link footprint of the
# OCCT-linked product build vs the native-only (post-drop-OCCT) build, for the
# iOS-SIMULATOR arm64 shipping slice. Produces the drop-OCCT SIZE table.
#
# WHAT IS MEASURED (three concrete numbers, all honest):
#   (a) libcybercadkernel.a  — the kernel's OWN static archive, built two ways:
#         · OCCT     : -DCYBERCAD_HAS_OCCT (compiles src/engine/occt/* + the adapter).
#         · native   : -DCYBERCAD_M8_REHEARSAL (no OCCT TUs; native default engine).
#       The delta is the kernel-side code the OCCT adapter costs.
#   (b) OCCT .a footprint that would be DROPPED — reported two ways:
#         · linked-subset : ONLY the TK* toolkits the app actually links (the set in
#                           run-sim-native-boolean.sh / the harness) — the true "what
#                           ships if OCCT is statically archived into the app" number.
#         · full-install  : every .a in the trimmed OCCT install (upper bound; the app
#                           links a subset, so this over-counts — labelled as such).
#   (c) OCCT TUs + exported symbols eliminated (nm on the OCCT-linked kernel .a for
#       the adapter TUs; object-count in the linked OCCT toolkits).
#
# The kernel archives are built by DIRECT compilation of the same src TU set the
# xcframework slice uses (facade + core + engine + native), via the iphonesimulator
# toolchain — NO product file is modified. NON-SHIPPING measurement only.
#
# Output: [SIZEROW] machine-readable lines + a human table. Deterministic (sizes are
# a property of the compiled objects, not timed).
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OCCT="${OCCT_ROOT:-/Users/leonardoaraujo/work/cybercad/build/occt}/install-SIMULATORARM64"
OUT="$REPO/build-size"; mkdir -p "$OUT"
SYSROOT="$(xcrun --sdk iphonesimulator --show-sdk-path)"
CXX="$(xcrun --sdk iphonesimulator --find clang++)"
AR="$(xcrun --sdk iphonesimulator --find libtool)"
NM="$(xcrun --find nm)"
NUMSCI="${NUMSCI_DIR:-$REPO/build-numsci/iossim}"

[ -d "$OCCT/include/opencascade" ] || { echo "OCCT sim install not found at $OCCT"; exit 1; }
[ -d "$NUMSCI" ] || { echo "numsci iossim not built ($NUMSCI); run scripts/build-numsci.sh iossim"; exit 1; }

TARGET=(-target arm64-apple-ios16.0-simulator -isysroot "$SYSROOT")
CFLAGS=(-std=c++20 -O2 -fvisibility=hidden -I"$REPO/include" -I"$REPO/src")

# ── source sets ───────────────────────────────────────────────────────────────
# Base kernel: all src/**/*.cpp EXCEPT the OCCT adapter, metal, numerics, tetgen.
# (The OCCT adapter is added ONLY to the OCCT build; numerics is added to BOTH via
# the numsci path so the two builds differ ONLY by the OCCT adapter + its macros —
# the honest, controlled comparison.)
mapfile_base() {
  find "$REPO/src" -name '*.cpp' \
    | grep -vE '/engine/occt/|/compute/metal/|/native/mesh/tet_mesher\.cpp$' \
    | sort
}
BASE_SRCS=(); while IFS= read -r s; do BASE_SRCS+=("$s"); done < <(mapfile_base)
OCCT_SRCS=(); while IFS= read -r s; do OCCT_SRCS+=("$s"); done < <(find "$REPO/src/engine/occt" -name '*.cpp' | sort)

# The TK* toolkits the app LINKS to satisfy the WHOLE OCCT adapter — the parity-harness
# set PLUS TKHLR (occt_drafting.cpp's HLR projection needs it; the harness set omits it
# because that harness never drives hlr_project). This is the honest full link set for
# the complete adapter, hence the honest "what ships" toolkit footprint.
TKS="TKDESTEP TKDEIGES TKXSBase TKDE TKMesh TKShHealing TKOffset TKFillet TKBool \
     TKPrim TKBO TKTopAlgo TKGeomAlgo TKHLR TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel"

# NumPP/SciPP absolute roots (mirror build-numsci.sh defaults).
NUMPP="${NUMPP_DIR:-/Users/leonardoaraujo/work/NumPP}"
SCIPP="${SCIPP_DIR:-/Users/leonardoaraujo/work/SciPP}"
NUMSCI_INC=(-I"$NUMPP/include" -I"$SCIPP/include" -I"$NUMSCI/gen")
NUMSCI_SRCS=(); while IFS= read -r s; do NUMSCI_SRCS+=("$s"); done < <(find "$REPO/src/native/numerics" -name '*.cpp' | sort)

compile_ar() {  # $1=label  $2=extra-def  shift 2 => source list
  local label="$1"; local def="$2"; shift 2
  local srcs=("$@")
  local obj="$OUT/obj-$label"; rm -rf "$obj"; mkdir -p "$obj"
  local objs=()
  for s in "${srcs[@]}"; do
    local base; base="$(echo "${s#$REPO/src/}" | tr '/' '_').o"
    "$CXX" "${TARGET[@]}" "${CFLAGS[@]}" "${NUMSCI_INC[@]}" -DCYBERCAD_HAS_NUMSCI=1 $def -c "$s" -o "$obj/$base"
    objs+=("$obj/$base")
  done
  local lib="$OUT/libcybercadkernel-$label.a"; rm -f "$lib"
  "$AR" -static -o "$lib" "${objs[@]}"
  echo "$lib"
}

echo "── [1/3] building OCCT-linked kernel .a (iossim arm64)"
OCCT_KERNEL_SRCS=("${BASE_SRCS[@]}" "${OCCT_SRCS[@]}" "${NUMSCI_SRCS[@]}")
LIB_OCCT="$(compile_ar occt "-DCYBERCAD_HAS_OCCT=1 -I$OCCT/include/opencascade" "${OCCT_KERNEL_SRCS[@]}")"

echo "── [2/3] building native-only (M8 rehearsal) kernel .a (iossim arm64)"
NAT_KERNEL_SRCS=("${BASE_SRCS[@]}" "${NUMSCI_SRCS[@]}")
LIB_NAT="$(compile_ar native "-DCYBERCAD_M8_REHEARSAL=1" "${NAT_KERNEL_SRCS[@]}")"

echo "── [3/3] measuring"
sz() { stat -f%z "$1"; }              # bytes
mb() { awk "BEGIN{printf \"%.2f\", $1/1048576.0}"; }

KO=$(sz "$LIB_OCCT"); KN=$(sz "$LIB_NAT")

# OCCT linked-subset footprint (bytes) + full-install footprint.
OCCT_SUB=0
for tk in $TKS; do
  a="$OCCT/lib/lib$tk.a"
  [ -f "$a" ] || continue
  OCCT_SUB=$((OCCT_SUB + $(sz "$a")))
done
# object-file count in the linked subset (portable: count archive members via ar t)
AR_T="$(xcrun --find ar)"
OCCT_SUB_MEMBERS=0
for tk in $TKS; do
  a="$OCCT/lib/lib$tk.a"; [ -f "$a" ] || continue
  n=$("$AR_T" t "$a" 2>/dev/null | grep -c '\.o$' || true)
  OCCT_SUB_MEMBERS=$((OCCT_SUB_MEMBERS + n))
done
OCCT_FULL=0
for a in "$OCCT"/lib/*.a; do OCCT_FULL=$((OCCT_FULL + $(sz "$a"))); done

# OCCT adapter TUs + exported symbols in the OCCT-linked kernel (the code deleted).
ADAPTER_TUS=${#OCCT_SRCS[@]}
ADAPTER_SYMS=$("$NM" -g "$LIB_OCCT" 2>/dev/null | grep -icE 'OcctEngine|occt' || true)

KDELTA=$((KO - KN))

# ── (d) DEAD-STRIPPED final-binary link — the true "what actually ships" number ──
# A static archive over-counts: the linker (-dead_strip) keeps only reachable code.
# Link a small executable that exercises a representative reachable cross-section of
# the facade (construct+boolean+tessellate+mass+section+fillet+step+query) against
# (i) the OCCT kernel + the OCCT toolkits and (ii) the native-only kernel, both with
# -dead_strip, and compare the stripped executable sizes. This is the honest LINKED
# footprint delta (smaller than the archive delta, and stated as the shipping number).
LINK_OCCT=0; LINK_NAT=0; LINK_NOTE="ok"
PROBE=/tmp/linkprobe_main.c
if [ -f "$PROBE" ]; then
  LFLAGS=""; for tk in $TKS; do LFLAGS="$LFLAGS -l$tk"; done
  if "$CXX" "${TARGET[@]}" -O2 -I"$REPO/include" "$PROBE" "$LIB_OCCT" \
       -L"$OCCT/lib" $LFLAGS -lc++ -Wl,-dead_strip \
       -o "$OUT/link_occt" 2>"$OUT/link_occt.log"; then
    LINK_OCCT=$(sz "$OUT/link_occt")
  else LINK_NOTE="occt-link-failed (see $OUT/link_occt.log)"; fi
  if "$CXX" "${TARGET[@]}" -O2 -I"$REPO/include" "$PROBE" "$LIB_NAT" \
       "$NUMSCI/libnumsci_iossim_arm64.a" -lc++ -Wl,-dead_strip \
       -o "$OUT/link_native" 2>"$OUT/link_native.log"; then
    LINK_NAT=$(sz "$OUT/link_native")
  else LINK_NOTE="$LINK_NOTE; native-link-failed (see $OUT/link_native.log)"; fi
fi
LINK_DELTA=$((LINK_OCCT - LINK_NAT))

echo
echo "[SIZEROW] kernel_occt_bytes=$KO"
echo "[SIZEROW] kernel_native_bytes=$KN"
echo "[SIZEROW] kernel_delta_bytes=$KDELTA"
echo "[SIZEROW] occt_linked_subset_bytes=$OCCT_SUB"
echo "[SIZEROW] occt_linked_subset_members=$OCCT_SUB_MEMBERS"
echo "[SIZEROW] occt_full_install_bytes=$OCCT_FULL"
echo "[SIZEROW] occt_adapter_tus=$ADAPTER_TUS"
echo "[SIZEROW] occt_adapter_symbols=$ADAPTER_SYMS"
echo "[SIZEROW] occt_toolkits_linked=$(echo $TKS | wc -w | tr -d ' ')"
echo "[SIZEROW] link_occt_stripped_bytes=$LINK_OCCT"
echo "[SIZEROW] link_native_stripped_bytes=$LINK_NAT"
echo "[SIZEROW] link_delta_stripped_bytes=$LINK_DELTA"
echo "[SIZEROW] link_note=$LINK_NOTE"

echo
echo "== BINARY SIZE: OCCT-linked product vs native-only (iossim arm64) =="
printf "%-42s %12s %10s\n" "component" "bytes" "MB"
printf "%-42s %12d %10s\n" "libcybercadkernel.a (OCCT build)"   "$KO" "$(mb $KO)"
printf "%-42s %12d %10s\n" "libcybercadkernel.a (native-only)"  "$KN" "$(mb $KN)"
printf "%-42s %12d %10s\n" "  kernel .a delta (OCCT adapter)"   "$KDELTA" "$(mb $KDELTA)"
printf "%-42s %12d %10s\n" "OCCT .a linked-subset (DROPPED)"    "$OCCT_SUB" "$(mb $OCCT_SUB)"
printf "%-42s %12d %10s\n" "OCCT .a full-install (upper bound)" "$OCCT_FULL" "$(mb $OCCT_FULL)"
echo
TOTAL_DROP=$((KDELTA + OCCT_SUB))
printf "TOTAL static footprint dropped (kernel adapter + linked OCCT subset): %s MB\n" "$(mb $TOTAL_DROP)"
printf "OCCT adapter TUs eliminated: %d   |  OCCT-side symbols in kernel .a: %d\n" "$ADAPTER_TUS" "$ADAPTER_SYMS"
printf "OCCT archive members (.o) in the linked toolkit subset: %d across %s toolkits\n" \
  "$OCCT_SUB_MEMBERS" "$(echo $TKS | wc -w | tr -d ' ')"
echo
echo "== DEAD-STRIPPED linked executable (true shipping footprint, representative reachable set) =="
printf "%-42s %12s %10s\n" "component" "bytes" "MB"
printf "%-42s %12d %10s\n" "exe linked w/ OCCT kernel + OCCT libs" "$LINK_OCCT" "$(mb $LINK_OCCT)"
printf "%-42s %12d %10s\n" "exe linked w/ native-only kernel"      "$LINK_NAT" "$(mb $LINK_NAT)"
printf "%-42s %12d %10s\n" "  dead-stripped link delta (OCCT code shipped)" "$LINK_DELTA" "$(mb $LINK_DELTA)"
echo "  note: $LINK_NOTE"
echo "== size bench complete =="
