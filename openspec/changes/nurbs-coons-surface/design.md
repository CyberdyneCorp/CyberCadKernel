# Design — nurbs-coons-surface

## Placement & conventions

New module `src/native/math/bspline_coons.{h,cpp}`, namespace `cybercad::native::math`, beside
`bspline_skin.{h,cpp}` / `bspline_gordon.{h,cpp}`. Reuses `math::Point3` (`native/math/vec.h`), the
evaluator `curvePoint` (`bspline.h`, corner evaluation), the **Layer-1 exact** curve ops
(`elevateDegreeCurve` / `refineKnotCurve`) and surface ops (`elevateDegreeSurface` /
`refineKnotSurface` from `bspline_ops.h`) for boundary + summand compatibility, and the **Layer-1
data types** `BsplineCurveData` (four boundaries in) / `BsplineSurfaceData` (surface out). OCCT-free,
fp64, deterministic. Added to the `native_math.h` aggregator.

**numsci gate.** Unlike skin/gordon, the Coons construction uses **no linear solve** — the ruled and
bilinear summands are built directly from the boundary control nets, and the boolean-sum combination
is exact Layer-1 knot/degree arithmetic. The `.cpp` is nonetheless placed under
`CYBERCAD_HAS_NUMSCI` so it sits uniformly with the rest of the numsci-gated Layer-6 surfacing family
(the header declares everything; with the guard OFF the implementation TU is inert and the functions
are absent). `CYBERCAD_HAS_NUMSCI` is defined library-wide, so `bspline_coons.cpp` — in the default
`src/native` glob — sees it when the option is ON.

Conventions match the rest of the kernel: **flat clamped knot vectors** (degree+1 end multiplicity,
length `nPoles + degree + 1`); **row-major, U-outer** surface poles `pole(i,j) = poles[i*nPolesV + j]`;
**non-rational** (weights empty).

## The four-sided boundary

`CoonsBoundary` holds the four boundary curves of a topological quad on the unit square `[0,1]²`:
`c0(u)` (the edge `v=0`), `c1(u)` (the edge `v=1`), `d0(v)` (the edge `u=0`), `d1(v)` (the edge
`u=1`). `c0`/`c1` run in u; `d0`/`d1` run in v. The four shared corners are
`P00 = c0(0) = d0(0)`, `P10 = c0(1) = d1(0)`, `P01 = c1(0) = d0(1)`, `P11 = c1(1) = d1(1)`. All four
curves are non-rational and parametrized on `[0,1]`.

## Boundary-corner consistency (`verifyCoonsBoundary`)

The bilinear boolean sum only interpolates the boundary when the four curves actually form a closed
quad. `verifyCoonsBoundary`:

1. **Non-rational + well-formed** — every boundary has empty weights, degree ≥ 1, a knot vector of
   length `nPoles + degree + 1`, and ≥ 2 poles. A rational or malformed boundary declines.
2. **Corner coincidence** — evaluate the eight boundary endpoints on the clamped `[0,1]` domain and
   check the four corner identities `‖c0(0) − d0(0)‖`, `‖c0(1) − d1(0)‖`, `‖c1(0) − d0(1)‖`,
   `‖c1(1) − d1(1)‖ ≤ tol`. Report the worst mismatch (`maxCornerError`).

On any violation: `ok = false` with a `reason`; never a silently-wrong quad.

## The boolean sum (`coonsPatch`)

`Coons = L_u ⊕ L_v ⊖ B`, from *The NURBS Book* §10.5 / the classic Coons construction:

1. **Verify** the boundary; decline on mismatched-corner / rational / degenerate input.
2. **Compatibilize the opposing pairs** (exact Layer-1). `c0`/`c1` are made compatible in u (same
   degree, same knot vector, same control-point count `Nc` via `elevateDegreeCurve` +
   `refineKnotCurve`); `d0`/`d1` likewise in v (count `Nd`). Both ops preserve geometry exactly, so
   the curves still equal their originals.
3. **Build the three summands.**
   - `L_v(u,v) = (1−v)·c0(u) + v·c1(u)` — the ruled surface between `c0` and `c1`: `degreeU = c`-degree
     with `c`-knots and `Nc` poles in u; `degreeV = 1` with knots `{0,0,1,1}` and 2 poles in v
     (`pole(i,0) = c0.poles[i]`, `pole(i,1) = c1.poles[i]`). Row-major U-outer.
   - `L_u(u,v) = (1−u)·d0(v) + u·d1(v)` — the ruled surface between `d0` and `d1`: `degreeU = 1` with
     `{0,0,1,1}` and 2 poles in u (`pole(0,j) = d0.poles[j]`, `pole(1,j) = d1.poles[j]`);
     `degreeV = d`-degree with `d`-knots and `Nd` poles in v. Row-major U-outer.
   - `B(u,v)` — the degree-(1,1) bilinear tensor product of the four corners: 2×2 poles, both knot
     vectors `{0,0,1,1}`, `pole(0,0)=P00`, `pole(1,0)=P10`, `pole(0,1)=P01`, `pole(1,1)=P11`.
4. **Unify to a common basis** (exact Layer-1). `unifyDirection(a, b, dir)` raises both surfaces to
   the common max degree (`elevateDegreeSurface`) then merges to the union knot vector
   (`refineKnotSurface`) in `dir`; both ops are geometry-preserving. Fold the three summands pairwise
   (two passes, mirroring the Gordon unification) until `L_v`, `L_u`, `B` share one basis in both
   directions.
5. **Combine.** `poles(Coons) = poles(L_v) + poles(L_u) − poles(B)` pointwise, weights empty.

## Why the boolean sum interpolates the boundary

On `v = 0`: `L_v(u,0) = c0(u)` (the ruled surface reduces to its `v=0` boundary). `L_u(u,0) =
(1−u)·d0(0) + u·d1(0) = (1−u)·P00 + u·P10` — the straight line between the two `v=0` corners. `B(u,0) =
(1−u)·P00 + u·P10` — the SAME line (the bilinear term is exactly the corner interpolant `L_u`/`L_v`
share along the edge). So `L_u(u,0) − B(u,0) = 0`, giving `Coons(u,0) = c0(u)`. By symmetry
`Coons(u,1) = c1(u)`, `Coons(0,v) = d0(v)`, `Coons(1,v) = d1(v)`. Corner interpolation follows.

**Coons is exact for bilinearly-blended surfaces.** If the source surface IS a Coons/ruled/bilinear
surface, its interior equals its own bilinear blend of its boundary, so feeding its four boundary
iso-curves back through the Coons builder reconstructs it pointwise. For a general tensor-product
surface the Coons patch reproduces the boundary exactly but the interior is the bilinear blend (by
definition — a property, not an error).

## Oracles (host, closed-form)

1. **Boundary containment** (core) — dense sample of each edge iso-curve vs the corresponding
   boundary curve, ~1e-9 (achieved ~1.8e-15).
2. **Corner interpolation** — the four surface corners vs the boundary corners, ~1e-12 (achieved 0).
3. **Flat patch** — a coplanar boundary → every surface point on the plane, ~1e-12 (achieved 0);
   a rectangular straight-edge boundary → the exact planar bilinear patch.
4. **Round-trip** — a ruled/bilinear surface recovered pointwise from its boundary (~1e-9, achieved
   ~1.8e-15); a general surface's four boundaries contained pointwise (~9e-16).
5. **Honest declines** — mismatched corner (large `maxCornerError`), rational boundary, malformed
   boundary → `ok = false` with a reason.

## Scope boundaries (residuals)

- **Rational / weighted** Coons patches (weighted boundaries) — not attempted; empty weights only.
- **N-sided fill** (5+ boundaries, degenerate-corner / triangular patches) — the bilinear
  construction is four-sided by definition.
- **G1/G2 plate blends** (Gregory / energy-minimizing patches with tangent/curvature continuity to
  adjacent surfaces) — the bilinearly-blended Coons matches boundary POSITION only.

All recorded in `docs/NURBS-SCOPE.md`.
