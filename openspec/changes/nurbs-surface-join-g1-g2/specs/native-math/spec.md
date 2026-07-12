# native-math

## ADDED Requirements

### Requirement: Surface G1/G2 continuity join across a shared edge

The native math library SHALL provide, in an OCCT-free module
(`src/native/math/bspline_join.{h,cpp}`, namespace `cybercad::native::math`,
`CYBERCAD_HAS_NUMSCI`-gated), routines that ENFORCE cross-boundary continuity between two ALREADY-BUILT
adjacent tensor-product NURBS patches `A` and `B` that share a boundary edge (position-continuous / C0),
by repositioning the near-boundary control rows so the patches meet **G1 (tangent-plane continuous)** or
**G2 (curvature continuous)** across the edge, with MINIMAL control-point movement and the shared
boundary curve frozen. The routines SHALL be **additive**: every existing surface-builder API SHALL
remain byte-unchanged.

An `EdgeSpec` SHALL name, for each of `A` and `B`, which boundary (`U0`/`U1`/`V0`/`V1`) is the shared
edge and whether their along-edge parameters run the same way or reversed. The join SHALL view each
surface as control ROWS parallel to the shared edge: row 0 (the boundary itself, frozen), row 1 (one
step in, the cross-boundary first-derivative control), row 2 (two steps in, the cross-boundary
second-derivative control).

`makeCompatibleAlongEdge` SHALL knot-merge and degree-match `A` and `B` along the shared edge so both
carry the same along-edge degree and knot vector, using the exact Layer-1 `elevateDegreeSurface` and
`refineKnotSurface` (the represented geometry of neither surface changes). It SHALL decline a malformed
patch, a rationality mismatch, a degenerate along-edge knot domain, or a shared boundary that is not
actually coincident within tolerance.

`joinG1` SHALL, with `P0[i]` the frozen boundary pole and `A1off[i] = A_row1[i] − P0[i]` at along-edge
station `i`, set `B_row1[i] = P0[i] − s·A1off[i]` for a single global proportionality `s > 0` chosen as
the closed-form least-squares minimiser of the total row-1 movement, so B's cross-boundary tangent is
antiparallel-collinear to A's and the unit normal is continuous across the edge. Row 0 SHALL be frozen
(C0 and the boundary curve preserved). If the required maximum control-point displacement exceeds a
caller-supplied cap, `joinG1` SHALL decline honestly (never widen a tolerance). Input already G1 within
tolerance SHALL be a no-op (movement 0).

`joinG2` SHALL first enforce G1, then set `B_row2[i] = P0[i] + s²·A2off[i] − 2s(s+1)·A1off[i]` (with
`A2off[i] = A_row2[i] − P0[i]`) so the cross-boundary second difference of B equals `s²·(that of A)` and
the normal curvature is continuous across the edge, with G1 (row 1) and C0 (row 0) preserved. It SHALL
decline when a patch lacks a third control row (cross-degree < 2 cannot carry G2) or when the required
movement exceeds the cap. Input already G2 SHALL be a no-op.

Each join SHALL report the achieved continuity residual (G1: max unit-normal mismatch in radians; G2:
max relative normal-curvature mismatch), the maximum control-point movement applied, and the shared-
boundary displacement (which SHALL stay at machine zero).

#### Scenario: Coplanar patches are a no-op

- GIVEN two co-planar tensor-product NURBS patches already meeting G-infinity across a shared edge
- WHEN `joinG1` or `joinG2` is invoked
- THEN the routine SHALL return `ok = true` with `noop = true`, `maxMovement == 0`, and a continuity
  residual already below tolerance — no control point SHALL be moved

#### Scenario: A C0 crease is made G1 with the boundary frozen

- GIVEN two patches sharing an edge with position continuity only (a genuine crease, unit-normal
  mismatch across the edge greater than 1e-3)
- WHEN `joinG1` is invoked with a permissive movement cap
- THEN after the join the unit normal SHALL be continuous across the shared edge on a dense sample to
  within 1e-7 radians, the shared boundary curve SHALL be unchanged to within 1e-12, and the reported
  residual and boundary deviation SHALL reflect this

#### Scenario: G2 makes normal curvature continuous and keeps G1

- GIVEN two patches sharing a C0 (crease) edge, each with a third control row (cross-degree ≥ 2)
- WHEN `joinG2` is invoked with a permissive movement cap
- THEN after the join the normal curvature SHALL be continuous across the shared edge to within a
  relative 1e-5, G1 (unit-normal continuity) SHALL still hold to within 1e-7 radians, and the shared
  boundary curve SHALL be unchanged to within 1e-12

#### Scenario: Two halves of a split cylinder are already G2 (no-op)

- GIVEN a smooth cubic-in-u cylinder wall split along its interior knot line into two Bézier halves that
  share the split-line edge
- WHEN `joinG1` or `joinG2` is invoked
- THEN the routine SHALL return `ok = true` with `noop = true` and `maxMovement == 0` (the halves are
  the same surface restricted to complementary spans, already C-infinity across the split)

#### Scenario: Over-cap movement is an honest decline

- GIVEN two patches whose G1/G2 enforcement would require control-point movement beyond the caller's cap
- WHEN `joinG1` or `joinG2` is invoked with that cap
- THEN the routine SHALL return `ok = false` with a reason and the required movement reported, and SHALL
  NOT distort the surface or widen a tolerance to pass

#### Scenario: Incompatible or non-coincident edges decline

- GIVEN two patches that are malformed, differ in rationality, have a degenerate along-edge knot domain,
  or whose named shared boundaries are not actually coincident (not C0)
- WHEN `makeCompatibleAlongEdge`, `joinG1`, or `joinG2` is invoked
- THEN the routine SHALL return `ok = false` with a reason, never a wrong surface and never a crash
