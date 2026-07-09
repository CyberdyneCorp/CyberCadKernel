# native-healing

## ADDED Requirements

### Requirement: Bounded, opt-in short-edge collapse of a redundant collinear sub-feature edge

The healer SHALL provide an **opt-in, merge-length-bounded** pass that removes a spurious
SHORT edge — a redundant vertex on an otherwise-straight boundary run that an exporter /
mesh→B-rep conversion split into a NON-zero micro-edge whose length is ABOVE the weld
`tolerance` (so degenerate-edge removal, which drops only ≤`tolerance` consecutive corners,
leaves it) — WITHOUT weakening the primary weld tolerance and WITHOUT changing the face's
boundary geometry. The pass SHALL be controlled by `HealOptions.shortEdgeMergeLen` (model
units): when `shortEdgeMergeLen == 0.0` the pass SHALL be a **no-op** and `healShell` SHALL
behave identically to the landed slices (every existing caller byte-identical); when
`shortEdgeMergeLen > 0.0` it is the maximum absolute edge length, **beyond `tolerance`**,
that the healer MAY collapse.

A short edge `B→C` with wire neighbours `A = prev(B)` and `D = next(C)` SHALL be collapsed
— removing BOTH `B` and `C` so the run becomes the straight span `A→D` — ONLY when ALL of
the following hold; a candidate failing any condition SHALL be left in place:

1. **Effective bound.** Its length `e = |B−C|` satisfies `tolerance < e ≤
   min(shortEdgeMergeLen, kShortEdgeFeatureFraction · min(|A−B|, |C−D|))`, where
   `kShortEdgeFeatureFraction = 0.25` is a fixed documented constant. This
   **local-feature-size cap** SHALL hold regardless of how large `shortEdgeMergeLen` is set,
   so a short edge comparable to either neighbour edge is NEVER collapsed.
2. **Collinearity within tolerance.** Both `B` and `C` lie within `tolerance` of the
   straight line through `A` and `D` (perpendicular distance). A short edge that turns a
   real corner (an endpoint off the `A-D` line) SHALL NOT be collapsed — only a redundant
   collinear split is removed, and removing it restores the EXACT straight span the
   neighbour face already carries.
3. **Loop stays a polygon.** Collapsing SHALL leave the face loop with `≥ 3` corners.

The collapse SHALL remove the micro-edge by dropping BOTH endpoints (restoring `A→D`), NOT
by merging them to a single interior vertex (which would leave the run unshared with the
neighbour face). The pass SHALL run BEFORE the primary sew (it rewrites per-face corner
loops) and SHALL be order-independent within a face loop.

After collapsing, the candidate SHALL pass the UNCHANGED mandatory self-verify
(`isWatertight` AND `enclosedVolume > 0` across the deflection ladder) before it is reported
`Healed`; a collapsed candidate that fails self-verify SHALL be `Unhealed`
(`SelfVerifyFailed`) with the input unchanged. The primary weld `tolerance` SHALL NEVER be
widened by this pass, and NO new `UnhealedReason` SHALL be introduced — a short edge outside
the bound SHALL stay the honest landed decline (`OpenShell` / `GapBeyondTolerance`) with the
input unchanged. Completeness beyond a redundant collinear short edge SHALL be framed as
**asymptotic** (per `MOAT-ROADMAP.md` M5): a short edge that turns a real corner, a collapse
needing the neighbour face re-projected, pcurve reconstruction, and self-intersecting-wire
repair remain out of scope and defer to OCCT `ShapeFix`.

The pass SHALL report `HealMetrics.nCollapsedShortEdges` (short edges removed) and
`HealMetrics.maxCollapsedShortEdge` (the longest edge collapsed, `≤` the effective bound).

#### Scenario: a redundant collinear short edge is collapsed into a watertight solid

- GIVEN a cube face soup with one face's boundary run split by a COLLINEAR short edge of
  length `e` with `tolerance < e ≤ shortEdgeMergeLen` and `e < 0.25 · min(neighbour edges)`,
  whose interior split vertices the neighbour face does not carry (so the shell is otherwise
  left open)
- WHEN `healShell` runs with `shortEdgeMergeLen > 0.0`
- THEN the pass SHALL remove the micro-edge, restore the straight span, share the run, and
  the candidate SHALL self-verify watertight with the correct positive enclosed volume
- AND `HealResult.status` SHALL be `Healed` with `nCollapsedShortEdges > 0` and
  `maxCollapsedShortEdge ≈ e`

#### Scenario: default merge length leaves the landed slices unchanged (no-op)

- GIVEN the same split soup whose micro-edge length `e > tolerance`
- WHEN `healShell` runs with `shortEdgeMergeLen == 0.0`
- THEN the short-edge pass SHALL NOT run, the primary weld tolerance SHALL NOT be widened,
  nothing SHALL be collapsed (`nCollapsedShortEdges == 0`), and the healer SHALL return
  `Unhealed` with the input shape UNCHANGED exactly as the landed slices do

#### Scenario: the local-feature cap refuses a collapse the merge length alone would allow

- GIVEN a split soup and a `shortEdgeMergeLen` large enough that the micro-edge length `e`
  exceeds `0.25 · min(neighbour edges)`
- WHEN `healShell` runs
- THEN that collapse SHALL be refused (the cap, not the caller's merge length, governs),
  `nCollapsedShortEdges == 0`, and the heal SHALL decline with the input unchanged rather
  than erase real feature

#### Scenario: a short edge that turns a real corner is never collapsed

- GIVEN a soup whose short edge's endpoints lie OFF the straight line through its wire
  neighbours (a genuine notch, not a redundant collinear split)
- WHEN `healShell` runs with `shortEdgeMergeLen` large enough to admit the length
- THEN the collinearity layer SHALL refuse the collapse (`nCollapsedShortEdges == 0`) and the
  face's boundary geometry SHALL be preserved, with the input unchanged

#### Scenario: native is equal-or-more-conservative than the OCCT oracle

- GIVEN the same split soup with the flag OFF, healed natively and, independently, sewn by
  the OCCT oracle (`BRepBuilderAPI_Sewing` + `ShapeFix`) whose sewer may aggressively close
  the collinear split even at a nominal tolerance below the micro-edge length
- WHEN both results are compared
- THEN native SHALL decline honestly (never emitting a different watertight solid than the
  truth) while OCCT MAY close, and when OCCT closes its solid SHALL be the same honest unit
  cube — so native deferring costs no correctness and the opt-in flag recovers exactly that
  win
