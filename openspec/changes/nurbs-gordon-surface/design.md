# Design — nurbs-gordon-surface

## Placement & conventions

New module `src/native/math/bspline_gordon.{h,cpp}`, namespace `cybercad::native::math`, beside
`bspline_skin.{h,cpp}` and `bspline_sweep.{h,cpp}`. Reuses `math::Point3` (`native/math/vec.h`), the
evaluators (`curvePoint`, `findSpan`, `basisFuns` from `bspline.h`), the **Layer-1 exact surface
ops** (`elevateDegreeSurface`, `refineKnotSurface` from `bspline_ops.h`) for summand compatibility,
the **Layer-6 compatibility** (`makeSectionsCompatible` from `bspline_skin.h`) to normalize each
family, and the **Layer-1 data types** `BsplineCurveData` (network curves in) / `BsplineSurfaceData`
(surface out). OCCT-free, fp64, deterministic. Added to the `native_math.h` aggregator.

**numsci gate.** The family/grid interpolations solve collocation systems through the numsci facade
(`numerics::lin_solve`), so the whole `.cpp` is under `CYBERCAD_HAS_NUMSCI`, exactly like
`bspline_skin.cpp`: the header declares everything; with the guard OFF the implementation TU is inert
and the functions are absent. `CYBERCAD_HAS_NUMSCI` is defined library-wide
(`target_compile_definitions(cybercadkernel PRIVATE CYBERCAD_HAS_NUMSCI=1)`), so `bspline_gordon.cpp`
— though in the default `src/native` glob — sees it when the option is ON.

Conventions match the rest of the kernel: **flat clamped knot vectors** (degree+1 end multiplicity,
length `nPoles + degree + 1`); **row-major, U-outer** surface poles `pole(i,j) = poles[i*nPolesV + j]`;
**non-rational** (weights empty).

## The curve network

`CurveNetwork` holds `K` u-direction curves `uCurves[k] = C_k(u)`, each with a v-parameter
`vParams[k]`, and `L` v-direction curves `vCurves[l] = D_l(v)`, each with a u-parameter `uParams[l]`.
The two families intersect at the grid `Q_{k,l}` where `C_k(uParams[l]) == D_l(vParams[k])`. Sizes:
`uCurves.size() == vParams.size() == K`, `vCurves.size() == uParams.size() == L`.

## Network consistency (`verifyNetwork`)

The boolean sum only interpolates the network when the network is a consistent grid. `verifyNetwork`:

1. **Sizes / minimums** — `K ≥ 2`, `L ≥ 2`, `vParams.size() == K`, `uParams.size() == L`.
2. **Non-rational + well-formed** — every curve has empty weights, degree ≥ 1, a knot vector length
   `nPoles + degree + 1`, and ≥ 1 pole. A rational or malformed curve declines.
3. **Monotone stations** — `vParams` strictly increasing over the u-curves, `uParams` strictly
   increasing over the v-curves (a proper grid). A non-monotone / duplicate station declines.
4. **Grid consistency** — for every `(k, l)`, `‖C_k(uParams[l]) − D_l(vParams[k])‖ ≤ tol`; the worst
   mismatch is `maxGridError`. The grid point `Q_{k,l}` is the average of the two evaluations.

On success `grid` holds the `K×L` intersection points (row-major, K outer). On failure `ok=false`
with a `reason` and `maxGridError` — honest, never a silently-wrong grid.

## Boolean sum (Algorithm — §10.5)

`gordonSurface(network, tol, uInterpDegree, vInterpDegree)`:

1. `verifyNetwork` → decline on inconsistent/degenerate input (carry the reason).
2. `makeSectionsCompatible(uCurves)` / `makeSectionsCompatible(vCurves)` → each family shares a
   degree, along-curve knots, and control-point count (exact Layer-1 elevate+refine).
3. **Three summands, interpolated at the PRESCRIBED station params** (this is the key difference from
   `skinSurface`, which auto-picks chord-length params — the Gordon sum requires all three to share
   the same parametrization):
   - `S_u = interpFamilyAcross(compatU, vParams, qV, alongIsU=true)` — loft the `K` u-curves across
     v at `vParams`; u-shape carried in U, v-interpolation across (degree `qV = clamp(vInterpDegree,
     1, K−1)`, averaging knots of `vParams`).
   - `S_v = interpFamilyAcross(compatV, uParams, qU, alongIsU=false)` — loft the `L` v-curves across
     u at `uParams`; v-shape carried in V, u-interpolation across (degree `qU`), net transposed.
   - `T  = interpGrid(grid, K, L, uParams, vParams, qU, qV)` — tensor interpolant of the grid at
     `(uParams, vParams)`: interpolate each v-station row across u, then each u-control column across
     v (two `lin_solve` stages).
4. **Common basis** — `unifyDirection(a, b, dir)` raises two surfaces to the common max degree
   (`elevateDegreeSurface`) then merges both onto the union knot vector (`refineKnotSurface`) in one
   direction, both exact. Fold `(S_u,S_v)`, `(S_u,T)`, `(S_v,T)` in U and V, then re-fold `S_u` (it
   may trail after a later unify raised the others), until `sameBasis` holds for all three.
5. **Gordon net** — `poles(G) = poles(S_u) + poles(S_v) − poles(T)`, inheriting the common
   degrees/knots. Weights empty ⇒ non-rational.

## Oracle strategy (why this layer is airtight)

| Property | Exact invariant (HOST, no OCCT) | Achieved |
|---|---|---|
| Network containment | `S(·,v_k) == C_k` and `S(u_l,·) == D_l`, dense sample | ~5e-15 |
| Grid intersection | `S(u_l,v_k) == Q_{k,l}` | ~1e-15 |
| Idempotence (full surface) | Gordon → re-extract network at same params → rebuild ≡ original, POINTWISE | ~4e-15 |
| Known-surface round-trip | Greville iso-curve network of a KNOWN uniform surface → Gordon ≈ source | ~1e-6 |
| Honest declines | inconsistent / <2 / rational / mismatched / non-monotone handled honestly | — |

**Why containment is exact.** After compatibility each family shares one along-curve basis; each
summand's interpolation reproduces the family curve at its station exactly (interpolation is a
closed-form identity, not a fit). At `v = v_k`, `S_u` reduces to `C_k` and `S_v − T` cancel (both
reduce to the same interpolant of the k-th grid row), so `G(·, v_k) = C_k` to solver precision —
hence machine precision. Symmetrically for `G(u_l, ·) = D_l`.

**Why the full-surface round-trip needs idempotence.** Reconstructing a KNOWN surface from an
iso-curve network reproduces the surface *between* the network curves only up to the interpolation
basis: the Gordon builder generates AVERAGING knots over the station params, which are not the
source's ORIGINAL interior knots (`avg(Greville(·))` is not the identity, even for uniform knots), so
the interpolant spaces differ slightly (~1e-6 for a uniform source, ~1e-4 general). This is the same
parametrization confound the Layer-6 skin round-trip documents. The MACHINE-EXACT full-surface
identity is IDEMPOTENCE: build `G1`, extract `G1`'s own network at the SAME station params (not
re-derived Greville) and rebuild `G2` — the averaging bases are then a true fixed point, so
`G1 ≡ G2` pointwise (~4e-15). Network curve *containment* is machine-exact regardless of the source's
knot structure.

## Complexity & structure

Chapter 10's algorithm is index-dense; per the cognitive-complexity policy the compilers/parsers band
(25–35) applies. Each routine is one focused function with the book's algorithm reference in comments;
`interpFamilyAcross` / `interpGrid` / `unifyDirection` / the knot-union / knots-to-insert helpers are
single small functions, not copy-paste. The construction reuses the Layer-1 exact surface ops, the
Layer-6 family compatibility, and the Layer-7 collocation idiom rather than re-implementing them.

## Risks & honest residuals

- **Non-rational only.** Rational/weighted Gordon surfaces (interpolating the network's weights) are
  materially harder and deliberately out of scope; the module returns non-rational surfaces (empty
  weights) and declines on rational curves. Documented in `docs/NURBS-SCOPE.md` Layer-6 row.
- **Regular (grid) networks only.** The boolean sum requires two transversal families intersecting at
  a `K×L` grid. Irregular / N-sided networks, boundary-curve filling, trimmed-boundary patches, and
  plate/energy surfaces are distinct constructions and remain demand-gated residuals — the module
  never fakes them.
- **Averaging-knot reconstruction residual.** A known surface is reconstructed up to ~1e-6 (uniform)
  because the Gordon builder's averaging knots are not the source's original knots; the exact
  continuous BRepFill/GeomFill construction is a separate residual. Network *containment* is exact,
  and the full-surface *identity* is proven exact by idempotence.
- **Consistency pre-condition.** The network must form a consistent grid (curve intersections match
  within `tol`); `verifyNetwork` DECLINES on an inconsistent network rather than emitting a surface
  that misses its own curves — honest, never silently-wrong.
