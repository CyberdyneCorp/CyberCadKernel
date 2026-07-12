# native-math

## ADDED Requirements

### Requirement: N-sided boundary closed-loop consistency verification

The native math library SHALL provide, in an OCCT-free module
(`src/native/math/bspline_nsided.{h,cpp}`, namespace `cybercad::native::math`,
`CYBERCAD_HAS_NUMSCI`-gated), a routine that verifies a set of N boundary B-spline curves
(`edges[0..N-1]`, in loop order, each on `[0,1]` running from corner `V[i]` to corner `V[i+1]`) forms
a CONSISTENT closed N-gon. The routine SHALL require `N ≥ 3`, every edge to be **non-rational** and
well-formed (clamped flat knot vector, degree ≥ 1, ≥ 2 control points), and the loop to close on
consecutive shared corners: `edges[i](1)` SHALL equal `edges[(i+1) mod N](0)` for every `i`, to within
a caller-supplied tolerance. It SHALL report the maximum corner mismatch. On any violation the routine
SHALL return `ok = false` with a human-readable reason, never a silently-broken loop and never a crash.

#### Scenario: A consistent closed N-gon verifies with a machine-precision corner error

- GIVEN N ≥ 3 boundary curves whose consecutive corners coincide (for example the straight edges of a regular polygon, or curves each pinned to shared corner points)
- WHEN the boundary is verified
- THEN the routine SHALL return `ok = true` with `maxCornerError` at machine precision (~1e-9 or below) and `n` equal to the number of edges

#### Scenario: A non-closed, rational, N<3, or malformed boundary declines honestly

- GIVEN a boundary in which one corner is displaced (so the loop does not close), or containing a rational curve (non-empty weights), or with fewer than three edges, or with a malformed / degenerate curve (degree 0 or a bad knot vector)
- WHEN the boundary is verified
- THEN the routine SHALL return `ok = false` with a reason and (for the displaced case) a large `maxCornerError`, without crashing

### Requirement: N-sided boundary fill by midpoint subdivision into N Coons sub-patches

The module SHALL construct, from a consistent closed N-gon boundary, a set of **N non-rational**
tensor-product B-spline Coons sub-patches whose UNION INTERPOLATES all N input boundary curves, by
Catmull-Clark-style midpoint subdivision. It SHALL first verify the boundary (declining honestly on a
non-closed / rational / degenerate / N<3 boundary). It SHALL compute the N corners `V[i] = edges[i](0)`,
the N edge midpoints `M[i] = edges[i](0.5)` (via the Layer-1 exact `splitCurve`), and the centroid
`C = mean(V[i])`. For each corner `V[i]` it SHALL build the quad centred on `V[i]` — with outer edges
the first half of `edges[i]` (running `V[i]→M[i]`) and the second half of `edges[i-1]` reversed
(running `V[i]→M[i-1]`), both obtained by the exact Layer-1 `splitCurve` + `reparamCurve`, and interior
edges the straight spokes `M[i-1]→C` and `M[i]→C` — and fill it with the Coons builder
(`bspline_coons::coonsPatch`). The core guarantee: each boundary edge `edges[k]` SHALL be reproduced by
exactly two sub-patch outer iso-edges (its first half by sub-patch `k`, its second half by sub-patch
`k+1`), so the union contains every boundary curve pointwise; adjacent sub-patches SHALL meet C0 along
the shared interior spoke `M[i]→C` (the SAME straight segment) and all SHALL pass through the centroid
`C`. A degenerate spoke (a midpoint coinciding with the centroid) or any sub-quad the Coons builder
rejects SHALL return `ok = false` with a reason, without crashing.

#### Scenario: The N sub-patches together contain every boundary curve pointwise

- GIVEN a consistent non-rational closed N-gon boundary (for example a planar pentagon, a planar triangle, or a mildly-curved in-plane hexagon)
- WHEN the N-sided fill is built
- THEN the result SHALL be N non-rational tensor-product sub-patches whose outer iso-edges reproduce all N boundary curves pointwise on a dense sample to within ~1e-9 (achieved: machine precision, ~1e-15)

#### Scenario: A planar N-gon yields flat coplanar patches on the boundary's plane

- GIVEN a boundary whose N edges are coplanar (for example a regular polygon or a curved-but-in-plane polygon in the `z=0` plane)
- WHEN the N-sided fill is built
- THEN every point of every sub-patch SHALL lie on that plane to within ~1e-12 (achieved 0), and the centroid SHALL lie in that plane

#### Scenario: Adjacent sub-patches meet C0 at the interior spokes and share the centroid

- GIVEN a successful N-sided fill
- WHEN the interior seams are sampled
- THEN adjacent sub-patches SHALL coincide pointwise along the shared interior spoke `M[i]→C` (sub-patch `i`'s `u=1` iso-curve equals sub-patch `i+1`'s `v=1` iso-curve) to within ~1e-12 (achieved 0), AND every sub-patch's `(u=1,v=1)` corner SHALL equal the centroid `C` exactly

#### Scenario: For N=4 the subdivision agrees with the single Coons patch on the boundary

- GIVEN a four-sided boundary
- WHEN the N-sided fill (4 sub-patches by the same subdivision) and the single Coons patch of that boundary are both built
- THEN the 4-sub-patch union SHALL contain the four boundary edges pointwise (~1e-9), and the single Coons patch SHALL reproduce them exactly, so the two constructions agree on the boundary

#### Scenario: Non-closed and degenerate boundaries decline honestly

- GIVEN a boundary with a displaced corner (open loop), or a rational edge, or fewer than three edges, or a malformed edge
- WHEN the N-sided fill is requested
- THEN the routine SHALL return `ok = false` with a reason, without crashing, and SHALL NOT emit patches that silently miss their own boundary curves

### Requirement: Non-rational N-sided fill scope with rational, curved-interior, and G1/G2 plate blends as explicit residuals

The N-sided module SHALL produce **non-rational** tensor-product B-spline sub-patches (the output
`weights` vectors are empty) from **non-rational** boundaries only. It SHALL NOT fabricate weights or
claim rational-N-sided capability. The midpoint-subdivision fill SHALL match the boundary POSITION
exactly and be C0 at the interior spokes and centroid; it SHALL NOT claim tangent (G1) or curvature
(G2) continuity across the interior seams, nor an energy-minimized curved interior. Rational / weighted
N-sided fill, the Gregory / energy-minimizing PLATE blends that achieve G1/G2 continuity across the
interior spokes (and to adjacent surfaces), and curved-N-sided interior fairing (the interior spokes
are straight lines to the centroid) are documented residuals for later slices, recorded in
`docs/NURBS-SCOPE.md`.

#### Scenario: N-sided geometry is non-rational and not faked-G1/G2 or faked-rational

- GIVEN any successful N-sided fill
- WHEN the result is inspected
- THEN every sub-patch's `weights` vector SHALL be empty (non-rational), and the module SHALL NOT attach fabricated weights, nor claim tangent/curvature continuity across the interior spokes, nor claim an energy-minimized curved interior
