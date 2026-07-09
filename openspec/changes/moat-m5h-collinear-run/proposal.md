# Proposal — moat-m5h-collinear-run (MOAT M5 tail — multi-collinear-vertex run to a fixpoint)

## Why

The landed opt-in collinear-vertex pass (`src/native/heal/collinear_vert.h`, capability
`native-healing`) removes a redundant collinear boundary vertex — the classic STEP
"T-vertex" / seam-split artifact — by one disjoint left-to-right sweep of each face loop.
The sweep advances past the removed vertex's successor (`i += 2`) to keep removals disjoint
within a pass, which is correct for a SINGLE extra vertex.

But a STEP exporter / mesh→B-rep conversion frequently drops MORE THAN ONE collinear vertex
onto one straight span — it "over-splits" a single straight edge `A→C` into three-or-more
FULL-LENGTH sub-edges `A→B1→B2→…→C`, every `Bₖ` exactly on the line. On such a run the
disjoint sweep removes only every OTHER vertex in a pass: for `A→B1→B2→C` it drops `B1` and
then skips past `B2` (its successor after the removal window), leaving `B2` standing. `B2`
is still an unshared interior vertex the neighbour face does not carry, so the tolerant sew
STILL cannot share the run and the shell is returned `Unhealed` — the exact defect the pass
was meant to close, only over-split by one more vertex.

The fix has a **pristine closed-form oracle** identical in kind to the landed single-vertex
one: a known-good unit cube whose top-face boundary run carries TWO or THREE collinear
vertices (each exactly on the line) heals to `V = 1.0` exactly iff EVERY redundant vertex on
the run is removed. Removing all-but-one still leaves the shell open; removing a vertex that
turns a real corner must still be refused.

## What changes

- `src/native/heal/collinear_vert.h` — split the per-loop removal into a single
  `detail::removeLoopVertsPass` (the existing disjoint sweep) and iterate it to a **FIXPOINT**
  inside `detail::removeLoopVerts`: re-run the pass on the previous pass's SURVIVORS until a
  pass removes nothing (or the loop drops to a triangle). Each pass reads neighbours from the
  current survivors, so once `B1` is gone the next pass sees `B2`'s neighbours as `A,C` and
  removes it too. Termination is guaranteed: survivors strictly decrease while any removal
  occurs (≤ `n` passes). The single-vertex behaviour is UNCHANGED (one pass removes it, the
  second removes nothing and terminates).
- No signature, `HealOptions`, `HealMetrics`, or `heal.cpp` change: the fixpoint is internal
  to `removeLoopVerts`, whose contract (remove every removable redundant collinear vertex on
  a loop) is now actually met for runs of length ≥ 2. `nRemovedCollinearVerts` now counts the
  whole run.

## Gates

- **Gate 1 — HOST analytic** (`tests/native/test_native_heal.cpp`, no OCCT): a unit cube
  whose +Z face carries a run of TWO collinear vertices (`t=0.3, 0.6`) heals to `V=1.0` with
  `nRemovedCollinearVerts == 2` (proving BOTH removed, not one); a run of THREE (`t=0.25,
  0.5, 0.75`) heals with `nRemovedCollinearVerts == 3` (proving > 1 fixpoint iteration);
  flag OFF declines honestly (`GapBeyondTolerance`, input unchanged); and a unit-layer drive
  of `removeLoopVerts` collapses a two-vertex run to a square while keeping an off-line real
  corner in a mixed run.
- **Gate 2 — SIM parity vs OCCT** (`tests/sim/native_heal_parity.mm`, booted iOS sim): the
  same two-vertex-run soup, native removal (`removed == 2`) matches OCCT `sewAndFix` (both
  watertight `V ≈ 1`); with the flag OFF native declines while OCCT closes — native
  EQUAL-OR-MORE-CONSERVATIVE, OCCT's closure the same honest unit cube.

## Impact

- Affected specs: `native-healing` (one MODIFIED requirement — the collinear-vertex removal
  now specified to collapse a RUN of two-or-more consecutive redundant collinear vertices to
  a fixpoint, with two added scenarios; every existing scenario preserved).
- Affected code: `src/native/heal/collinear_vert.h` (internal fixpoint refactor, no ABI /
  option / metric change); `tests/native/test_native_heal.cpp`,
  `tests/sim/native_heal_parity.mm`, `openspec/MOAT-ROADMAP.md` (M5 status).
- `src/native/**` stays OCCT-free; `cc_*` ABI unchanged (healing is internal); the
  tessellator is untouched; the primary weld tolerance is never widened; the single-vertex
  and default-OFF behaviour is byte-identical (one pass then terminate; no-op when disabled).
- Declined (honest asymptotic tail, deferred to OCCT `ShapeFix`): a vertex that turns a REAL
  (non-collinear) corner, a removal needing the neighbour face re-projected, pcurve
  reconstruction, self-intersecting-wire repair, and arbitrary broken industrial B-rep.
