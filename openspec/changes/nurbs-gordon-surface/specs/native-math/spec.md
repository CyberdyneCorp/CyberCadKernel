# native-math

## ADDED Requirements

### Requirement: Curve-network consistency verification for Gordon surfaces

The native math library SHALL provide, in an OCCT-free module
(`src/native/math/bspline_gordon.{h,cpp}`, namespace `cybercad::native::math`,
`CYBERCAD_HAS_NUMSCI`-gated), a routine that verifies a NETWORK of `K` u-direction B-spline curves
`C_k(u)` (each with a v-parameter `v_k`) and `L` v-direction B-spline curves `D_l(v)` (each with a
u-parameter `u_l`) forms a CONSISTENT grid. The routine SHALL require `K ≥ 2` and `L ≥ 2`, param
arrays sized to match (`K` v-params, `L` u-params), **non-rational** and well-formed curves in both
families, strictly-increasing station parameters in each direction, and grid consistency: for every
`(k, l)` the u-curve at the v-curve's station `C_k(u_l)` SHALL equal the v-curve at the u-curve's
station `D_l(v_k)` (both equal the intersection point `Q_{k,l}`) to within a caller-supplied
tolerance. It SHALL report the maximum grid mismatch and, on success, the `K×L` intersection grid.
On any violation the routine SHALL return `ok = false` with a human-readable reason, never a
silently-wrong grid and never a crash.

#### Scenario: A consistent grid network verifies with a machine-precision grid error

- GIVEN a network whose u-curves and v-curves intersect at a `K×L` grid (for example, iso-curves extracted from a single tensor-product surface)
- WHEN the network is verified
- THEN the routine SHALL return `ok = true` with `maxGridError` at machine precision (~1e-9 or below) and a `K×L` grid of the intersection points

#### Scenario: An inconsistent or degenerate network declines honestly

- GIVEN a network in which a curve is displaced off the grid (so its intersections no longer match), or with fewer than two curves in a direction, or containing a rational curve, or with mismatched param-array sizes, or with non-monotone station parameters
- WHEN the network is verified
- THEN the routine SHALL return `ok = false` with a reason and (for the displaced case) a large `maxGridError`, without crashing

### Requirement: Gordon / network surface interpolating every network curve

The module SHALL construct, from a consistent curve NETWORK (`K ≥ 2` u-curves, `L ≥ 2` v-curves
intersecting at a `K×L` grid) and chosen interpolation degrees, a **non-rational** tensor-product
B-spline surface that INTERPOLATES every network curve — the boolean sum `G = S_u ⊕ S_v ⊖ T` (*The
NURBS Book* §10.5). It SHALL first verify the network (declining honestly on inconsistent/degenerate
input), make each family compatible (shared degree/knots/control-count via the Layer-1 exact
`elevateDegreeCurve`/`refineKnotCurve`), then build three tensor-product summands interpolated at the
PRESCRIBED station params: `S_u` lofting the `K` u-curves across v at the v-params, `S_v` lofting the
`L` v-curves across u at the u-params, and `T` the tensor-product interpolant of the `K×L`
intersection grid at `(uParams, vParams)` — the across-direction interpolations solved via the
numerics facade `lin_solve`. It SHALL raise the three summands to a COMMON degree and merge them onto
COMMON knot vectors in each direction with the exact Layer-1 surface ops (`elevateDegreeSurface` /
`refineKnotSurface`, geometry-preserving), then form the Gordon control net pointwise
`poles(G) = poles(S_u) + poles(S_v) − poles(T)` with empty weights. The core guarantee: the surface's
iso-curve `S(·, v_k)` SHALL equal the u-curve `C_k`, `S(u_l, ·)` SHALL equal the v-curve `D_l`, and
`S(u_l, v_k)` SHALL equal the grid point `Q_{k,l}`. An inconsistent/degenerate network, a rational
curve, or a singular interpolation SHALL return `ok = false` without crashing.

#### Scenario: The Gordon surface contains every network curve and grid point pointwise

- GIVEN a consistent non-rational network of `K` u-curves and `L` v-curves and interpolation degrees
- WHEN the Gordon surface is built
- THEN the result SHALL be a non-rational tensor-product surface whose iso-curve `S(·, v_k)` equals `C_k` at every `u` and whose iso-curve `S(u_l, ·)` equals `D_l` at every `v`, to within ~1e-8 (achieved: machine precision), for every curve, and whose evaluation at each grid station `S(u_l, v_k)` equals the intersection point `Q_{k,l}` to within ~1e-8

#### Scenario: Gordon reconstructs known tensor-product geometry (idempotent round-trip)

- GIVEN a KNOWN tensor-product B-spline surface whose iso-curves at its Greville abscissae are extracted as a network and fed to the Gordon builder, and separately a Gordon surface `G1` whose OWN iso-curve network at its OWN station parameters is re-extracted and rebuilt into `G2`
- WHEN the results are compared
- THEN the Gordon surface built from the KNOWN surface's iso-curve network SHALL recover the source closely (to ~1e-6 for a uniform-knot source — the averaging-knot vs source-knot parametrization confound), AND `G1` and `G2` SHALL evaluate to the same point at every `(u,v)` to within ~1e-8 (achieved: machine precision) — proving the boolean sum reconstructs geometry exactly where the interpolation parametrization is a fixed point

#### Scenario: Inconsistent and degenerate networks decline honestly

- GIVEN an inconsistent network (a curve displaced off the grid), or fewer than two curves in a direction, or a network containing a rational curve, or mismatched param-array sizes, or non-monotone station parameters
- WHEN the Gordon surface is requested
- THEN the routine SHALL return `ok = false` with a reason, without crashing, and SHALL NOT emit a surface that silently misses its own network curves

### Requirement: Non-rational Gordon scope with rational and irregular networks as explicit residuals

The Gordon module SHALL produce **non-rational** tensor-product B-spline surfaces (all weights
implicitly one; the output `weights` vector is empty) from **non-rational** REGULAR (grid) networks.
It SHALL NOT fabricate weights or claim rational-Gordon capability. Rational / weighted Gordon
surfaces (interpolating the network weights), IRREGULAR / N-sided networks (non-grid topologies,
boundary-curve filling, trimmed-boundary patches, plate/energy surfaces), and the exact continuous
GeomFill/BRepFill surfacing residual (beyond the ~1e-6 averaging-knot reconstruction) are documented
residuals for later slices, recorded in `docs/NURBS-SCOPE.md`.

#### Scenario: Gordon geometry is non-rational, not faked-rational or faked-irregular

- GIVEN any successful Gordon surface
- WHEN the result is inspected
- THEN its `weights` vector SHALL be empty (non-rational), and the module SHALL NOT attach fabricated weights, nor emit an irregular / N-sided / trimmed-boundary surface, nor claim those capabilities
