# native-math

## ADDED Requirements

### Requirement: Variable-section swept surface (scale + twist along the spine)

The swept-surface module SHALL provide a **variable-section** sweep. The module lives in
`src/native/math/bspline_sweep.{h,cpp}` (namespace `cybercad::native::math`, `CYBERCAD_HAS_NUMSCI`-gated,
OCCT-free). The sweep SHALL sweep a B-spline **section** curve along a **non-rational** B-spline
**trajectory** (spine) while applying a per-station uniform **scale** and **twist** to the
rotation-minimizing-frame placed section, then SKIN the placed sections into one tensor-product B-spline
surface (*The NURBS Book*
§10.4, generalized). The variation SHALL be supplied as two sampled fields of length equal to the
station count `K`: `scales[k]` (a strictly-positive uniform scale about the section origin at station
`k`) and `twists[k]` (a rotation in radians about the section's local sweep axis at station `k`). At
each station the section SHALL be transformed by the scale, then the twist (both acting in the section
plane), then the rigid rotation-minimizing frame that translates the section origin to the station
point and rotates its plane normal onto the trajectory tangent. Because a uniform scale composed with
rotations is a **similarity**, it SHALL preserve a rational section's weights exactly; a **non-rational**
variant (`sweepVariable`) produces a non-rational surface via non-rational skinning, and a **rational**
variant (`sweepRationalVariable`, requiring a strictly-positive-weight section) produces a rational
surface via rational skinning. The core guarantee: the surface SHALL contain each scaled+twisted+placed
section at its station parameter (inherited from skinning's containment oracle). Passing an all-ones
`scales` and an all-zero `twists` SHALL reproduce the plain rotation-minimizing-frame sweep exactly.
Fewer than two stations, a scale or twist field whose length differs from the station count, a
non-positive scale, a rational section or trajectory on the non-rational routine (or a non-rational /
non-positive-weight section on the rational routine, or a rational trajectory), a malformed section or
trajectory, a degenerate (coincident-station) trajectory, or a downstream skin failure SHALL return
`ok = false` without crashing.

#### Scenario: A unit-scale, zero-twist variable sweep reproduces the plain RMF sweep exactly

- GIVEN a non-rational section, a non-rational trajectory, a section normal, and `K ≥ 2` stations with `scales[k] = 1` and `twists[k] = 0` for every station
- WHEN the section is swept with the variable-section sweep
- THEN the resulting surface SHALL equal the plain rotation-minimizing-frame sweep (`sweepAlongTrajectory`) of the same section along the same trajectory at every `(u, v)` to within ~1e-12 (achieved: machine precision / bit-exact), AND the rational variant SHALL likewise reproduce the rational RMF sweep

#### Scenario: A rational circle scaled linearly along a straight spine is an exact rational cone frustum

- GIVEN a true rational NURBS circle of radius `R`, a straight trajectory, and a linear scale field `scales[k] = s0 + (s1 − s0)·k/(K−1)` with zero twist
- WHEN the section is swept with the rational variable-section sweep
- THEN the result SHALL be a rational surface that CONTAINS each scaled circle at its station parameter to within ~1e-9 (achieved ~1e-15), AND every point of the station iso-curve SHALL lie at radius `scales[k]·R` from the spine axis — an EXACT rational cone frustum whose station cross-sections are true circles, not facets

#### Scenario: Variable sweep declines on degenerate, mis-sized, or wrongly-routed input

- GIVEN fewer than two stations, OR a scale/twist field whose length differs from the station count, OR a non-positive scale, OR a rational section/trajectory on the non-rational routine (or a non-rational section on the rational routine), OR a coincident-station trajectory
- WHEN a variable-section sweep is requested
- THEN the routine SHALL return `ok = false` without crashing, and SHALL NOT emit a surface with fabricated weights

### Requirement: Two-rail swept surface (section anchored between two rail curves)

The module SHALL provide a **two-rail** sweep that positions, scales, and orients a B-spline **section**
curve so that two of its control points (the **anchors**, given as distinct in-range pole indices
`anchor0` / `anchor1`) ride two **non-rational** B-spline **rail** curves `rail0` / `rail1` at every
station, then SKINS the placed sections into one tensor-product B-spline surface. The rails SHALL share
a common clamped parameter domain; at each of `K ≥ 2` stations sampled evenly across that domain the
placement SHALL: scale the section by `s(t) = |rail1(t) − rail0(t)| / |anchor1 − anchor0|` (so the
anchor chord spans the current rail-to-rail chord); orient the section's anchor chord onto the rail
chord `rail1(t) − rail0(t)`, removing the remaining spin with a rotation-minimizing frame along the
rail-midpoint spine (anti-twist); and translate so `anchor0` lands on `rail0(t)`. The composed map is a
**similarity** (uniform scale + rotation + translation), so a **non-rational** variant (`sweepTwoRail`)
produces a non-rational surface and a **rational** variant (`sweepRationalTwoRail`, requiring a
strictly-positive-weight section and non-rational rails) preserves the section's weights exactly via
rational skinning. The core guarantee: the section's two anchor iso-curves of the resulting surface
SHALL lie on `rail0` and `rail1` respectively at every station, and the surface SHALL contain each
placed section. Fewer than two stations, a rational section or either rational rail on the non-rational
routine (or a non-rational / non-positive-weight section, or rational rails, on the rational routine), a
malformed section or rail, anchor indices out of range or equal, coincident section anchors
(`|anchor1 − anchor0| ≈ 0`), DEGENERATE RAILS (a station where `|rail1(t) − rail0(t)| ≈ 0` — the rails
cross or touch, leaving scale and orientation undefined), a degenerate (coincident-midpoint) rail spine,
or a downstream skin failure SHALL return `ok = false` without crashing.

#### Scenario: The section anchors ride both rails at every station

- GIVEN a well-formed section with two distinct in-range anchor poles, two non-rational rails sharing a domain and a strictly-positive rail-to-rail chord at every sample, and `K ≥ 2` stations
- WHEN the section is swept between the two rails
- THEN the anchor0 iso-curve of the surface SHALL lie on `rail0` and the anchor1 iso-curve SHALL lie on `rail1` at every station to within ~1e-9 (achieved ~1e-16), AND the surface SHALL contain each placed section

#### Scenario: Parallel rails give a ruled strip and diverging rails an exact linear taper

- GIVEN a straight (degree-1) segment section anchored at its two endpoints
- WHEN it is swept between two parallel straight rails, THEN the surface SHALL be the closed-form planar ruled strip to within ~1e-9
- WHEN it is swept between two linearly-diverging straight rails, THEN each on-station iso-curve SHALL equal the linear interpolation `rail0(t) → rail1(t)` (an exact tapered surface) to within ~1e-9, with both anchor endpoints on the rails

#### Scenario: Two-rail sweep declines on degenerate rails, anchors, or wrongly-routed input

- GIVEN rails that cross or coincide (a zero-length rail chord at some station), OR coincident section anchors, OR anchor indices out of range or equal, OR a rational section / rational rail on the non-rational routine (or a non-rational section on the rational routine), OR fewer than two stations, OR a degenerate rail spine
- WHEN a two-rail sweep is requested
- THEN the routine SHALL return `ok = false` without crashing (honest-decline), and SHALL NOT emit a surface with fabricated weights
