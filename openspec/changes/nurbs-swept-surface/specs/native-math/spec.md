# native-math

## ADDED Requirements

### Requirement: Translational swept surface (exact extrusion)

The native math library SHALL provide, in an OCCT-free module
(`src/native/math/bspline_sweep.{h,cpp}`, namespace `cybercad::native::math`, `CYBERCAD_HAS_NUMSCI`-gated),
a routine that sweeps a **non-rational** B-spline **section** curve along a straight **vector** to
produce the EXACT tensor-product B-spline surface (*The NURBS Book* §10.4, the extrusion /
tabulated-cylinder case). The result SHALL carry the section in the U direction (degree `p`, the
section's knot vector, its `N = poles.size()` control points) and a degree-1 two-pole path in the V
direction (clamped V-knots `{0,0,1,1}`), with the surface net `pole(i,0) = section.poles[i]` and
`pole(i,1) = section.poles[i] + sweep` (row-major, U-outer), and empty weights. Because a degree-1
path on `{0,0,1,1}` is the straight interpolation, the surface `S(u,v)` SHALL equal the section
`C(u)` translated by `v·sweep`, so the construction is EXACT — no fitting and no linear solve. A
rational section (non-empty weights), a malformed section (degree below one, empty poles, or a knot
vector whose length is not `poles + degree + 1`), or a null sweep vector (magnitude below tolerance)
SHALL make the call decline (`ok = false`) without crashing.

#### Scenario: A section swept along a vector is the section translated at every station

- GIVEN a non-rational, well-formed B-spline section curve and a non-null sweep vector
- WHEN the section is swept translationally along that vector
- THEN the result SHALL be a non-rational surface with `degreeV = 1`, `nPolesV = 2`, `degreeU` equal to the section degree, and `nPolesU` equal to the section pole count, AND its iso-curve `S(·, v)` SHALL equal the section translated by `v·sweep` at every parameter `u` and every `v ∈ [0,1]` to within ~1e-12 (achieved: machine precision) — an EXACT extrusion with no fitting

#### Scenario: A straight section swept translationally is the closed-form ruled patch

- GIVEN a straight (degree-1) section from point `A` to point `B` and a sweep vector `d`
- WHEN the section is swept translationally
- THEN the surface SHALL evaluate to the closed-form bilinear ruled patch `S(u,v) = A + u·(B−A) + v·d` at every `(u,v)` to within ~1e-12

#### Scenario: Translational sweep declines on rational, malformed, or null input

- GIVEN a rational section (non-empty weights), OR a malformed section (bad knot-vector length / degree below one / empty poles), OR a null sweep vector
- WHEN a translational sweep is requested
- THEN the routine SHALL return `ok = false` without crashing, and SHALL NOT emit a surface with fabricated weights

### Requirement: General swept surface along a trajectory (transform-then-skin)

The module SHALL construct, from a **non-rational** B-spline **section** curve, a **non-rational**
B-spline **trajectory** (spine) curve, a section plane normal, a station count `K ≥ 2`, and a V
degree, a **non-rational** tensor-product B-spline surface that sweeps the section along the
trajectory (*The NURBS Book* §10.4). It SHALL sample the trajectory at `K` parameters evenly across
its clamped domain, taking each station's point and unit tangent; compute a **rotation-minimizing
moving frame** along the trajectory (double-reflection method — Wang, Jüttler, Zheng & Liu, ACM TOG
2008 — seeded from the section normal projected orthogonal to the start tangent) so the swept profile
does not spin about the spine; transform a copy of the section by each station's rigid frame
(translate its origin to the station point, rotate its plane normal onto the trajectory tangent
within the rotation-minimizing frame); and SKIN the `K` transformed sections into one tensor-product
surface via the Layer-6 skinning routine. The core guarantee: the swept surface SHALL contain each
transformed section at its station parameter (inherited from skinning's containment oracle). Fewer
than two stations, a rational section or trajectory, a malformed section or trajectory, a degenerate
trajectory domain, a stationary trajectory point (null tangent), coincident sampled trajectory points
(no path to sweep along), or a downstream skin failure SHALL return `ok = false` without crashing.

#### Scenario: The swept surface contains each transformed section at its station

- GIVEN a non-rational section, a non-rational trajectory, a section normal, and `K ≥ 2` stations
- WHEN the section is swept along the trajectory
- THEN the result SHALL be a non-rational tensor-product surface whose U degree equals the section degree, with one station parameter `v_k` per station, and its iso-curve `S(·, v_k)` SHALL equal the rigidly transformed section placed at station `k` at every parameter `u` to within ~1e-8 (achieved: machine precision), for every station

#### Scenario: A rotation-minimizing frame sweeps without twist on a straight spine

- GIVEN a section whose plane normal is aligned with a straight trajectory's tangent
- WHEN the section is swept along that straight trajectory with `K ≥ 2` stations
- THEN the rotation-minimizing frame SHALL keep the section orientation fixed (no spurious spin about the spine), so the general swept surface SHALL equal the translational sweep of the same section along the same total displacement at every `(u,v)` to within ~1e-8 (achieved: machine precision), AND on a curved trajectory each station iso-curve SHALL preserve the section's arc length (a rigid frame is length-preserving)

#### Scenario: General sweep declines on degenerate or rational input

- GIVEN fewer than two stations, OR a rational section or trajectory, OR a malformed section or trajectory, OR a trajectory whose sampled points are all coincident (no path)
- WHEN a general sweep is requested
- THEN the routine SHALL return `ok = false` without crashing, and SHALL NOT emit a surface with fabricated weights

### Requirement: Non-rational sweep scope with rational, rotational, exact-continuous, and variable-section sweeps as explicit residuals

The swept-surface module SHALL produce **non-rational** tensor-product B-spline surfaces (all weights
implicitly one; the output `weights` vector is empty) from **non-rational** sections and
trajectories. It SHALL NOT fabricate weights or claim rational-sweep capability. Rational / weighted
sweeps, rotational (revolved) sweeps (a profile revolved about an axis — exact-rational, a distinct
construction), exact GeomFill/BRepFill-class continuous sweeps (the general sweep skins a finite set
of station placements — an approximation that refines with station count, not an analytically exact
swept surface), and variable-section sweeps (a section that morphs along the spine) are documented
residuals for later slices, recorded in `docs/NURBS-SCOPE.md`.

#### Scenario: Swept geometry is non-rational, not faked-rational or faked-exact

- GIVEN any successful translational or general sweep
- WHEN the result is inspected
- THEN its `weights` vector SHALL be empty (non-rational), and the module SHALL NOT attach fabricated weights, nor emit a rotational / exact-continuous / variable-section sweep, nor claim those capabilities
