# native-healing

## MODIFIED Requirements

### Requirement: Bounded, opt-in removal of a redundant collinear boundary vertex

The healer SHALL provide an **opt-in** pass that removes REDUNDANT COLLINEAR vertices on an
otherwise-straight boundary run — the classic "T-vertex" / seam-split artifact an exporter /
mesh→B-rep conversion drops onto a face's straight span `A→C`, so that face lists `A→B→C`
(two edges) while the NEIGHBOUR face carries the same span as ONE straight edge `A→C` —
WITHOUT weakening the primary weld `tolerance` and WITHOUT changing the face's boundary
geometry beyond `tolerance`. The pass SHALL be controlled by `HealOptions.removeCollinearVerts`
(bool): when `false` the pass SHALL be a **no-op** and `healShell` SHALL behave identically
to the landed slices (every existing caller byte-identical); when `true` redundant collinear
vertices MAY be removed. The pass SHALL introduce NO length parameter — exact collinearity is
the sole geometric criterion, so BOTH incident edges `|A−B|` and `|B−C|` MAY be full-length
real edges (which is why the short-edge collapse pass, whose length band caps the removed
span at `¼·neighbour`, cannot reach this defect).

A vertex `B` with wire neighbours `A = prev(B)` and `C = next(B)` SHALL be removed — so the
run becomes the straight edge `A→C` — ONLY when ALL of the following hold; a candidate
failing any condition SHALL be left in place:

1. **Collinearity within tolerance.** The perpendicular distance of `B` from the straight
   line through `A` and `C` is `≤ tolerance`. A vertex that turns a real corner (off the
   `A-C` line by more than `tolerance`) SHALL NOT be removed — removing it would change the
   face's boundary by more than the weld tolerance.
2. **Strict-between projection.** `B` projects strictly BETWEEN `A` and `C` (parameter
   `0 < t < 1` along `A→C`). A backtracking spur that folds past a neighbour (`t ≤ 0` or
   `t ≥ 1`), even if it lies on the infinite line, SHALL NOT be treated as a redundant
   interior vertex.
3. **Loop stays a polygon.** The removal SHALL leave the face loop with `≥ 3` corners.

Within ONE removal sweep, removals SHALL be kept disjoint (a removed vertex's neighbours `A`
and `C` SHALL NOT themselves be removed in the same sweep). The sweep SHALL then be iterated
to a **FIXPOINT**: it SHALL re-run on the previous sweep's SURVIVING corners until a sweep
removes nothing (or the loop reaches a triangle). Consequently, a RUN of two-or-more
consecutive redundant collinear vertices on one straight span (`A→B1→B2→…→C`, every `Bₖ`
within `tolerance` of the line `A→C` and projecting strictly between `A` and `C`) SHALL be
collapsed FULLY — leaving all-but-one on the run, which a single disjoint sweep would do, is
NOT sufficient because the surviving interior vertex still blocks the sew. Because each sweep
reads neighbours from the current survivors, a vertex left standing by the disjoint skip
(e.g. `B2` after `B1` is removed) SHALL be removed on the next sweep. The fixpoint SHALL
remove ONLY corners collinear-within-tolerance among the current survivors, so it SHALL
never remove a real corner, and it SHALL terminate (the survivor count strictly decreases
while any removal occurs). The pass SHALL run BEFORE the primary sew (it rewrites per-face
corner loops). Removing every within-tolerance-collinear vertex on the run restores the
EXACT straight span `A→C` the neighbour face already carries, so vertex unification then
shares `A` and `C` and the shell closes.

After removal, the candidate SHALL pass the UNCHANGED mandatory self-verify (`isWatertight`
AND `enclosedVolume > 0` across the deflection ladder) before it is reported `Healed`; a
candidate that fails self-verify SHALL be `Unhealed` (`SelfVerifyFailed`) with the input
unchanged. The primary weld `tolerance` SHALL NEVER be widened by this pass, and NO new
`UnhealedReason` SHALL be introduced — a defect outside the bound SHALL stay the honest
landed decline (`OpenShell` / `GapBeyondTolerance`) with the input unchanged. Completeness
beyond redundant collinear vertices SHALL be framed as **asymptotic** (per `MOAT-ROADMAP.md`
M5): a vertex that turns a real corner, a removal needing the neighbour face re-projected,
pcurve reconstruction, and self-intersecting-wire repair remain out of scope and defer to
OCCT `ShapeFix`.

The pass SHALL report `HealMetrics.nRemovedCollinearVerts` (the TOTAL vertices removed
across the fixpoint, i.e. the whole run) and `HealMetrics.maxCollinearVertDev` (the largest
perpendicular deviation of a removed vertex, `≤ tolerance`).

#### Scenario: a redundant collinear vertex is removed into a watertight solid

- GIVEN a cube face soup with one face's straight boundary run `A→C` carrying an extra
  vertex `B` on the line (perpendicular deviation `≤ tolerance`, projecting strictly
  between `A` and `C`), with both incident edges full-length, whose interior vertex the
  neighbour face does not carry (so the shell is otherwise left open)
- WHEN `healShell` runs with `removeCollinearVerts == true`
- THEN the pass SHALL remove `B`, restore the straight span `A→C`, share the run, and — on
  passing the unchanged self-verify — return `Healed` with a watertight, positive-volume
  solid, `nRemovedCollinearVerts > 0`, `maxCollinearVertDev ≤ tolerance`, and residual `0`

#### Scenario: a run of two-or-more collinear vertices is collapsed to a fixpoint

- GIVEN a cube face soup whose one face's straight boundary run `A→C` carries a RUN of TWO
  OR MORE consecutive collinear vertices (`A→B1→B2→…→C`, every `Bₖ` within `tolerance` of the
  line and projecting strictly between `A` and `C`, all incident edges full-length), whose
  interior vertices the neighbour face does not carry
- WHEN `healShell` runs with `removeCollinearVerts == true`
- THEN the fixpoint SHALL remove EVERY vertex on the run (not merely every other one),
  `nRemovedCollinearVerts` SHALL equal the number of run vertices, the span SHALL become the
  single straight edge `A→C`, and the shell SHALL heal to a watertight, positive-volume solid
  with residual `0`

#### Scenario: the pass is a no-op when disabled

- GIVEN the same defected soup (single vertex or a run)
- WHEN `healShell` runs with `removeCollinearVerts == false` (the default)
- THEN the pass SHALL NOT run, `nRemovedCollinearVerts == 0`, and the result SHALL be the
  honest landed decline (`GapBeyondTolerance`, residual `> tolerance`) with the input
  UNCHANGED — byte-identical to the healer before this pass existed

#### Scenario: a vertex that turns a real corner is never removed

- GIVEN a soup whose extra vertex lies OFF the straight line through its wire neighbours by
  more than `tolerance` (a genuine corner, not a redundant collinear vertex), possibly amid a
  run of otherwise-collinear vertices
- WHEN `healShell` runs with `removeCollinearVerts == true`
- THEN the collinearity layer SHALL refuse to remove that vertex on every fixpoint sweep
  (the real corner is preserved), even while collinear vertices elsewhere on the loop are
  removed

#### Scenario: a backtracking spur is never treated as redundant

- GIVEN a boundary run whose extra vertex lies on the infinite line through its neighbours
  but projects OUTSIDE the segment (`t ≤ 0` or `t ≥ 1`, a fold-back spur)
- WHEN the removal is evaluated
- THEN the strict-between projection layer SHALL refuse the removal and the vertex SHALL be
  left in place

#### Scenario: native is equal-or-more-conservative than the OCCT oracle

- GIVEN the same defected soup (single vertex or a run) with the flag OFF, healed natively
  and, independently, sewn by the OCCT oracle (`BRepBuilderAPI_Sewing` + `ShapeFix`) whose
  sewer may aggressively drop the collinear vertices and close the shell
- WHEN both results are compared
- THEN native SHALL decline honestly (never emitting a different watertight solid than the
  truth) while OCCT MAY close, and when OCCT closes its solid SHALL be the same honest unit
  cube — so native deferring costs no correctness and the opt-in flag recovers exactly that
  win
