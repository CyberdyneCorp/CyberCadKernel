# Proposal — moat-m5h-healing-breadth (MOAT M5 tail — short-edge merge)

## Why

The native shape healer (`src/native/heal/*`, capability `native-healing`) heals the
coincident / degenerate / orientation defect family exactly, plus three landed opt-in
tail slices: bounded beyond-tolerance gap bridging, single planar-hole capping, and
multi planar-hole capping. Each is default-OFF, never widens the weld tolerance, and
defers honestly to OCCT `ShapeFix` when out of scope.

One high-value, STEP-import-relevant defect class the landed slices still DECLINE is a
**spurious short (sub-feature) edge on an otherwise-straight boundary run**: a boundary
vertex a STEP exporter / mesh→B-rep conversion split into two distinct vertices a tiny
distance apart, inserting a NON-zero micro-edge whose length sits ABOVE the weld
`tolerance` (so `dropZeroLengthSides` in `degenerate.h` leaves it — that pass removes
only ≤`tolerance` consecutive corners) yet below any reasonable feature size. Because
that micro-edge carries an interior vertex the NEIGHBOUR face does not have (its matching
span is one straight edge), the tolerant sew cannot share the run: boundary edges survive
and the shell is returned `Unhealed` (honestly, with a measured residual). OCCT closes
exactly this with `ShapeFix_Wire::FixSmall` / sewing at a larger tolerance.

This is distinct from the two landed rewrite passes:

- `gap_bridge.h` snaps **cross-face** unpaired corners onto a partner (a *seam* near-miss
  *between two faces*). The short-edge defect is *within a single face's wire* — a
  redundant vertex on a straight run, not a seam gap.
- `degenerate.h::dropZeroLengthSides` removes only **≤`tolerance`** consecutive corners
  (a genuine zero-length side). A 5e-3 micro-edge with `tolerance = 1e-4` is untouched.

The defect has a **pristine closed-form oracle**: a known-good unit cube with one
boundary side split by a *collinear* micro-edge is still exactly a unit cube; collapsing
the redundant edge restores volume `V = 1.0` and watertightness EXACTLY (no
re-approximation). App relevance is direct: imported-STEP cleanup routinely carries
sub-feature spurious edges from tessellated / converted geometry.

## What Changes

A new **opt-in, default-OFF, bounded** healing pass — SHORT-EDGE COLLAPSE — added
STRICTLY ADDITIVELY to the OCCT-free native healer:

- **New header** `src/native/heal/short_edge.h` (header-only, OCCT-free):
  `collapseShortEdges(soup, tol, mergeLen)` removes a redundant COLLINEAR short edge
  `B→C` (wire neighbours `A`, `D`) from a face loop when it lies in the bounded band
  `(tol, min(mergeLen, ¼·min(|A−B|,|C−D|))]` AND both `B` and `C` lie within `tol` of the
  straight line `A→D`. It removes both `B` and `C`, restoring the straight span `A→D`
  the neighbour face already carries, so `vertex_unify` then shares `A`/`D` and the shell
  closes. Reuses the same local-feature-fraction guarantee (`kShortEdgeFeatureFraction =
  0.25`) as `gap_bridge.h`.
- **New option** `HealOptions.shortEdgeMergeLen` (default `0.0` ⇒ pass is a no-op; the
  field is appended LAST so every existing positional aggregate-initialization is
  byte-identical) and **new metrics** `nCollapsedShortEdges` / `maxCollapsedShortEdge`
  (additive fields on `HealMetrics`).
- **Pipeline wiring** in `heal.cpp`: the pass runs on the working soup BEFORE the first
  sew (it rewrites per-face corner loops), dead-guarded by `shortEdgeMergeLen > 0.0`, so
  with the default the path is byte-identical to the landed slices.
- **No new `UnhealedReason`**: a short edge outside the bound stays the honest landed
  decline (`OpenShell` / `GapBeyondTolerance`), input unchanged.
- **The UNCHANGED mandatory self-verify remains authoritative**: a collapsed candidate is
  reported `Healed` only if it still tessellates watertight with positive enclosed volume.

Verification (two gates, OCCT is the oracle):

- **Gate 1 — HOST analytic** (`tests/native/test_native_heal.cpp`, no OCCT): a
  deliberately-defected unit-cube fixture (`cubeTopShortEdge`) heals to exact `V = 1.0`
  ONLY with the opt-in flag; flag OFF declines honestly with the input unchanged; the
  feature cap and the collinearity layer refuse to erase real feature; a unit test drives
  `collapseLoop` directly.
- **Gate 2 — SIM parity vs OCCT** (`tests/sim/native_heal_parity.mm`, booted iOS sim): the
  same defected soup, native collapse matches OCCT `sewAndFix` at a tolerance that swallows
  the micro-edge (both watertight `V ≈ 1`); with the flag OFF native declines while OCCT
  aggressively closes — native is EQUAL-OR-MORE-CONSERVATIVE (never a wrong repair), and
  OCCT's aggressive closure is the same honest unit cube (no correctness lost by deferring).

## Impact

- Affected specs: `native-healing` (one ADDED requirement + its scenarios).
- Affected code: `src/native/heal/short_edge.h` (new), `src/native/heal/heal.cpp`,
  `src/native/heal/heal_result.h`, `src/native/heal/native_heal.h` (additive);
  `tests/native/test_native_heal.cpp`, `tests/sim/native_heal_parity.mm`,
  `scripts/run-sim-suite.sh` (SKIP entry), `openspec/MOAT-ROADMAP.md` (M5 status).
- `src/native/**` stays OCCT-free; `cc_*` ABI unchanged (healing is internal, no facade
  entry); the tessellator is untouched; the primary weld tolerance is never widened;
  existing heal callers are byte-identical with the flag OFF.
- Declined (honest asymptotic tail, deferred to OCCT `ShapeFix`): a short edge that turns
  a REAL (non-collinear) corner, a short edge whose removal would need the neighbour face
  re-projected, pcurve reconstruction, self-intersecting-wire repair, and arbitrary broken
  industrial B-rep.
