# native-topology

## ADDED Requirements

### Requirement: Parameter-space trimmed-region boolean

The topology module SHALL provide `trimRegionBoolean(regionA, regionB, op, opts)`
(`src/native/topology/trim_boolean.{h,cpp}`) computing the 2-D boolean — `op ∈ {Union, Intersect,
Difference}` — of two trim REGIONS (each an outer loop plus hole loops, `TrimLoop` pcurve segments)
sharing a surface (u,v) parameter domain, and returning the result region as a set of oriented
`ResultLoop` polylines plus its total signed area, with a `TrimBoolStatus` (`Ok`, `Empty`,
`Degenerate`, `Invalid`).

The computation SHALL be the standard planar region-boolean arc-walk (Greiner–Hormann): flatten each
loop's pcurve segments into a UV polygon using the SAME seam-consistent evaluator `classify()`
ray-casts (a public `flattenTrimLoop()` on `trimmed_nurbs`, a thin wrapper over the existing flatten
— so a rational circle/ellipse pcurve flattens with no sag via the homogeneous lift); compute the
pairwise A-edge / B-edge crossings and split both regions' loops at every crossing; classify each
arc inside/outside the OTHER region by the SAME even-odd ray-cast rule `classify()` uses; and walk
the selected arcs into closed loops. Output loop orientation SHALL be normalised by NESTING depth —
an OUTER boundary CCW (positive signed area), a HOLE CW (negative signed area) — so the total signed
area equals the true region area.

For the airtight closed-form cases the result SHALL be EXACT to the stated tolerance: disjoint /
overlapping polygonal regions to ≤1e-10, and a circular-lens intersection (rational-circle pcurves)
to ≤1e-8 of the closed-form circular-lens area.

A DEGENERATE overlap — a coincident boundary edge (the two loops share a boundary segment) or a
tangential-only touch (boundaries meet at a point with no transversal crossing) — SHALL be
HONEST-DECLINED (`Degenerate`, no output loops), never resolved into a fabricated region. No
tolerance SHALL be widened to force a crossing. A malformed input (empty / non-closeable loop) SHALL
return `Invalid`.

The module SHALL make no `shape.h` / `cc_*` change, SHALL keep `src/native` OCCT-free, and SHALL
leave the existing `trimmed_nurbs` API byte-unchanged (only the additive `flattenTrimLoop` wrapper is
exposed).

#### Scenario: Disjoint squares — union, intersection, difference

- GIVEN two non-overlapping square trim regions A and B
- WHEN combined
- THEN `Union` SHALL yield BOTH squares (total area = |A| + |B|), `Intersect` SHALL be `Empty` (area
  0), and `Difference` (A∖B) SHALL yield A (area |A|) — each area exact to ≤1e-10.

#### Scenario: Overlapping squares — exact polygonal areas

- GIVEN two overlapping axis-aligned square regions with known crossing params
- WHEN combined
- THEN `A∩B`, `A∪B` and `A∖B` SHALL each yield the exact polygonal region whose area equals the
  closed-form value (|A∩B|, |A|+|B|−|A∩B|, |A|−|A∩B|) to ≤1e-10 — the square-area error SHALL be
  machine-zero.

#### Scenario: Hole handling — annulus nested-loop result

- GIVEN region A = an annulus (an outer loop with a square hole) and a disk region B
- WHEN B lies entirely inside A's hole THEN `A∩B` SHALL be `Empty` and `A∪B` SHALL have area
  |A| + |B| (B a disjoint island in the hole)
- AND WHEN B straddles A's outer wall (not touching the hole) THEN `A∩B` SHALL be the correct nested
  region with the closed-form signed area (≤1e-10).

#### Scenario: Circular-lens intersection (rational pcurves)

- GIVEN two overlapping circular trim regions (rational-circle pcurves) with centre distance d and
  radius r
- WHEN intersected
- THEN the result SHALL be the lens-shaped region whose area matches the closed-form circular-lens
  area `2r²·cos⁻¹(d/2r) − (d/2)·√(4r²−d²)` to ≤1e-8.

#### Scenario: Coincident / tangential overlap declines honestly

- GIVEN two square regions that share a boundary edge exactly (a coincident boundary)
- WHEN combined with any op
- THEN `trimRegionBoolean` SHALL return `Degenerate` with no output loops — never a fabricated
  region.
