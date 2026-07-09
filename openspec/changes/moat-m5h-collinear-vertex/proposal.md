# Proposal — moat-m5h-collinear-vertex (MOAT M5 tail — collinear-vertex removal)

## Why

The native shape healer (`src/native/heal/*`, capability `native-healing`) heals the
coincident / degenerate / orientation defect family exactly, plus four landed opt-in
tail slices: bounded beyond-tolerance gap bridging, single planar-hole capping, multi
planar-hole capping, and short-edge collapse. Each is default-OFF, never widens the weld
tolerance, and defers honestly to OCCT `ShapeFix` when out of scope.

One high-value, STEP-import-relevant defect class the landed slices still DECLINE is a
**single redundant COLLINEAR vertex on an otherwise-straight boundary run** — the classic
"T-vertex" / seam-split artifact. A STEP exporter / mesh→B-rep conversion drops an EXTRA
vertex `B` onto a face's straight span `A→C`, so that face lists `A→B→C` (two edges) while
the NEIGHBOUR face carries the same span as ONE straight edge `A→C`. `B` turns no corner
(it lies within `tolerance` of the line `A→C`) and BOTH incident edges `|A−B|` and `|B−C|`
may be FULL-LENGTH real edges. Because the neighbour does not carry `B`, the tolerant sew
cannot share the run: boundary edges survive and the shell is returned `Unhealed`
(honestly, with a measured residual). OCCT closes exactly this with
`ShapeUpgrade_UnifySameDomain` / `ShapeFix_Wire`.

This is distinct from every landed pass:

- `degenerate.h::dropZeroLengthSides` removes only a **≤`tolerance`** (near-ZERO-length)
  side — a duplicated corner. Here `|A−B|` and `|B−C|` are full-length real edges.
- `gap_bridge.h` snaps **cross-face** unpaired corners onto a partner (a *seam* near-miss
  *between two faces*). The redundant vertex is *within a single face's wire*.
- `short_edge.h` removes a redundant collinear SUB-FEATURE MICRO-EDGE `B→C` — a tiny span
  bounded at `¼·neighbour`, removing TWO corners, needing `≥ 5` loop corners. Here a
  SINGLE vertex `B` sits between two full-length edges; `collapseLoop`'s length band
  (`e ≤ ¼·min(neighbour)`) rejects a full-length `B→C`, so pass 8 cannot reach it.

The defect has a **pristine closed-form oracle**: a known-good unit cube with one extra
collinear vertex dropped onto a top-face boundary run at a non-midpoint parameter (both
sub-edges full-length) heals to `V = 1.0` exactly iff the redundant vertex is removed; a
vertex pushed off the line (a real corner) must be preserved and the heal declined.

## What changes

- New header `src/native/heal/collinear_vert.h` (header-only, OCCT-free):
  `removeCollinearVertices(soup, tol)` + `detail::removeLoopVerts` +
  `detail::perpAndParam`, with the four-layer bound (opt-in flag, collinearity within
  tolerance AND strict-between projection, loop stays `≥ 3` corners, deferral to the
  UNCHANGED self-verify) documented in the header. Introduces NO length parameter — exact
  collinearity is the sole geometric criterion.
- `HealOptions.removeCollinearVerts` (bool) appended LAST (default `false`) so every
  existing positional aggregate-init is byte-identical; `HealMetrics.nRemovedCollinearVerts`
  + `maxCollinearVertDev` added additively (`heal_result.h`).
- Wired into `heal.cpp` BEFORE the first sew, dead-guarded by `removeCollinearVerts`; the
  include added to `native_heal.h` + pass #9 documented in its header.

## Gates

- **Gate 1 — HOST analytic** (`tests/native/test_native_heal.cpp`, no OCCT): a unit cube
  whose +Z face carries one collinear T-vertex at `t=0.3` heals to `V=1.0` with the flag ON
  (`nRemovedCollinearVerts > 0`, `nCollapsedShortEdges == 0` — NOT the short-edge pass);
  flag OFF declines honestly (`GapBeyondTolerance`, input unchanged); an off-line real
  corner is never removed; and a unit-layer drive of `removeLoopVerts` removes a collinear
  vertex, keeps an off-line notch, and refuses a backtracking spur (`t ∉ (0,1)`).
- **Gate 2 — SIM parity vs OCCT** (`tests/sim/native_heal_parity.mm`, booted iOS sim): the
  same defected soup, native removal matches OCCT `sewAndFix` (both watertight `V ≈ 1`);
  with the flag OFF native declines while OCCT aggressively closes — native is
  EQUAL-OR-MORE-CONSERVATIVE (never a wrong repair), and OCCT's aggressive closure is the
  same honest unit cube (no correctness lost by deferring).

## Impact

- Affected specs: `native-healing` (one ADDED requirement + its scenarios).
- Affected code: `src/native/heal/collinear_vert.h` (new), `src/native/heal/heal.cpp`,
  `src/native/heal/heal_result.h`, `src/native/heal/native_heal.h` (additive);
  `tests/native/test_native_heal.cpp`, `tests/sim/native_heal_parity.mm`,
  `openspec/MOAT-ROADMAP.md` (M5 status).
- `src/native/**` stays OCCT-free; `cc_*` ABI unchanged (healing is internal, no facade
  entry); the tessellator is untouched; the primary weld tolerance is never widened;
  existing heal callers are byte-identical with the flag OFF.
- Declined (honest asymptotic tail, deferred to OCCT `ShapeFix`): a vertex that turns a
  REAL (non-collinear) corner, a removal that would need the neighbour face re-projected,
  pcurve reconstruction, self-intersecting-wire repair, and arbitrary broken industrial
  B-rep.
