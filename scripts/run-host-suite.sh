#!/usr/bin/env bash
#
# run-host-suite.sh — build-then-run a host (OCCT-free) test suite from build-host-verify.
#
#   scripts/run-host-suite.sh test_native_ssi_seeding
#   scripts/run-host-suite.sh test_native_patch_gap test_native_ssi   # several, in order
#   scripts/run-host-suite.sh --list
#   scripts/run-host-suite.sh --all                                   # every registered suite
#
# WHY THIS EXISTS — a stale binary reports a verdict for code that is no longer there.
#
# The suites are plain executables under build-host-verify/, so the obvious thing is to run one
# directly. That is exactly the trap. `cmake --build <dir> --target <t>` relinks only <t>; every
# OTHER suite binary keeps pointing at whatever libcybercadkernel.a looked like when it was last
# linked. Touch a header, build one target, run a different one, and you are reading a verdict
# from a kernel that no longer exists.
#
# This is not hypothetical. Measured in one session: test_native_multiseam_asym reported a
# FAILURE that its own freshly-linked source PASSES (5/0), which was then bisected against a
# clean worktree and misattributed to an unrelated kernel change; and separately a probe compiled
# with seam_strip.h at one constant, linked against an archive built at another, produced a
# CONFIRMED-looking order-dependence defect that does not exist (the header is header-only, so
# the constant inlines per translation unit and the binary ran both values at once). Same archive
# bytes, opposite deterministic answers. Three false conclusions, all from the same root cause.
#
# scripts/run-host-sim-parity.sh already guards its own lane, and that guard fired correctly
# twice in the same session. This is the missing counterpart for the host lane.
#
# THE GUARD IS "BUILD IT FIRST", NOT "WARN IF OLD". A timestamp warning still lets the run
# proceed on a stale binary if the check is imperfect; building first makes staleness
# unrepresentable, because CMake resolves the dependency graph. The mtime check below is a
# belt-and-braces backstop for the case where the build is somehow a no-op.
#
set -uo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${CYBERCAD_HOST_BUILD:-$REPO/build-host-verify}"
JOBS="${JOBS:-$(nproc 2>/dev/null || echo 4)}"

if [ ! -d "$BUILD" ]; then
  echo "── no host build dir at $BUILD"
  echo "   configure one first, e.g.:"
  echo "     cmake -S . -B build-host-verify -DCYBERCAD_HAS_NUMSCI=ON \\"
  echo "       -DCYBERCAD_NUMSCI_DIR=\$PWD/build-numsci/linuxhost \\"
  echo "       -DCYBERCAD_NUMPP_DIR=... -DCYBERCAD_SCIPP_DIR=..."
  exit 2
fi

list_suites() {
  # The registered test targets, straight from CMake rather than a hand-kept list.
  cmake --build "$BUILD" --target help 2>/dev/null |
    sed -n 's/^\.\.\. \(test_[A-Za-z0-9_]*\)$/\1/p' | sort -u
}

if [ "${1:-}" = "--list" ]; then
  echo "host suites registered in $BUILD:"
  list_suites | sed 's/^/  /'
  exit 0
fi

TARGETS=()
if [ "${1:-}" = "--all" ]; then
  mapfile -t TARGETS < <(list_suites)
  [ "${#TARGETS[@]}" -gt 0 ] || { echo "── no test targets found"; exit 2; }
elif [ "$#" -ge 1 ]; then
  TARGETS=("$@")
else
  echo "usage: $0 <suite>... | --all | --list"
  exit 2
fi

ARCHIVE="$BUILD/libcybercadkernel.a"
FAILED=()
for t in "${TARGETS[@]}"; do
  echo "── building $t"
  if ! cmake --build "$BUILD" --target "$t" -j"$JOBS" > /tmp/host-suite-$$.log 2>&1; then
    echo "── BUILD FAILED: $t"
    grep -E "error:" /tmp/host-suite-$$.log | head -10
    rm -f /tmp/host-suite-$$.log
    FAILED+=("$t (build)")
    continue
  fi
  rm -f /tmp/host-suite-$$.log

  BIN="$BUILD/$t"
  [ -x "$BIN" ] || { echo "── no executable at $BIN"; FAILED+=("$t (missing)"); continue; }

  # Backstop: after a successful build the binary must not predate the archive it links.
  # If this ever fires, the dependency graph is wrong — do not run, the verdict is untrustworthy.
  if [ -f "$ARCHIVE" ] && [ "$ARCHIVE" -nt "$BIN" ]; then
    echo "── STALE AFTER BUILD: $t is older than $ARCHIVE."
    echo "   The build did not relink it, so its result would describe a kernel that is gone."
    echo "   Rebuild the whole tree:  cmake --build $BUILD -j$JOBS"
    FAILED+=("$t (stale)")
    continue
  fi

  echo "── running $t"
  if "$BIN"; then :; else FAILED+=("$t (failed)"); fi
  echo
done

if [ "${#FAILED[@]}" -gt 0 ]; then
  echo "── ${#FAILED[@]} suite(s) not clean:"
  printf '     %s\n' "${FAILED[@]}"
  exit 1
fi
echo "── all ${#TARGETS[@]} suite(s) clean"
