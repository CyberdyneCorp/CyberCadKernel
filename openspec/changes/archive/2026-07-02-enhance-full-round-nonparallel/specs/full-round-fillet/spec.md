# full-round-fillet

## MODIFIED Requirements

### Requirement: Full round consumes the target face into a valid watertight solid
When a full round succeeds, the returned body SHALL be
`BRepCheck_Analyzer::IsValid` and watertight (a closed shell with no free
boundary), and the consumed middle face SHALL be GONE — the middle face id SHALL
NOT resolve to a face in the rebuilt body's face set.

The native rolling-ball blend SHALL handle both PLANAR neighbour-wall
configurations that admit a constant-radius rolling ball:

- **(anti-)parallel planar walls** — the constant-radius strip on the mid-plane
  (the pre-existing case); and
- **NON-PARALLEL planar walls meeting at a dihedral** — the two neighbour planes
  still admit a ball of radius `r` tangent to both, whose centre lies on the
  interior (valley-side) dihedral bisector at perpendicular distance `r` from BOTH
  planes; rolling it along the crease sweeps a tangent CYLINDER with axis DIRECTION
  `normalize(n1 × n2)` (the planes' intersection / crease direction) and axis
  LOCATION at that bisector point. The blend SHALL be the portion of that cylinder
  spanning the two seam edges, so the result is watertight, the middle face is
  consumed, and the blend is G1-tangent to both non-parallel neighbours along the
  seams. `r` SHALL be fixed by the strip geometry exactly as the parallel case
  picks it (the seam-edge offset / half the perpendicular strip width); the
  parallel case SHALL be the limiting specialization of this construction (as
  `n1 × n2 → 0` the bisector plane → the strip mid-plane) and SHALL remain
  unchanged.

Truly CURVED (non-planar) neighbour faces remain out of scope: such a case MAY
fall back to a valid standard fillet and be recorded deferred (see the honest-
fallback requirement) rather than asserting a full round that is not tangent.

#### Scenario: Rib round is valid, watertight, and consumes the top face
- GIVEN a body with a narrow middle face between two side faces (e.g. the top of
  a rib), on a booted iOS simulator
- WHEN `cc_full_round_fillet_faces(body, left, middle, right)` runs successfully
- THEN the result SHALL be `BRepCheck_Analyzer::IsValid` and watertight
- AND the `middle` face id SHALL no longer resolve to a face in the returned body

#### Scenario: Non-parallel planar rib rounds into a dihedral tangent cylinder
- GIVEN a body whose middle strip face lies between two PLANAR neighbour walls that
  meet at a real dihedral angle (non-parallel outward normals `n1`, `n2` with
  `|n1 × n2|` above the crease floor), on a booted iOS simulator
- WHEN `cc_full_round_fillet_faces(body, left, middle, right)` runs successfully
- THEN the result SHALL be `BRepCheck_Analyzer::IsValid` and watertight
- AND the `middle` face id SHALL no longer resolve to a face in the returned body
- AND the blend surface SHALL be a CYLINDER whose axis direction matches
  `normalize(n1 × n2)` (the strip / crease direction) within tolerance
- AND that cylinder axis SHALL be at equal perpendicular distance `r` from BOTH
  neighbour planes (the tangent radius), placing its centre on the interior
  dihedral bisector

#### Scenario: Non-parallel blend is G1-tangent to both neighbours at the seams
- GIVEN a successful non-parallel planar full round on the dihedral-rib body, on a
  booted iOS simulator
- WHEN the blend-surface normal and each neighbour-face normal are sampled at
  matching points along both seams
- THEN at every sample the normals SHALL agree within the documented tangency
  tolerance (their dot product ≥ cos(≈1°)) for BOTH the left and right neighbour

#### Scenario: Parallel rib is unchanged by the generalization
- GIVEN the pre-existing (anti-)parallel rib fixture (a box rib whose two side
  walls are parallel), on a booted iOS simulator
- WHEN `cc_full_round_fillet_faces` runs on its top strip
- THEN it SHALL still take the native rolling-ball path and produce the SAME valid
  watertight solid as before (same exact volume and blend-cylinder axis within a
  tight tolerance), with the middle face consumed

#### Scenario: Curved neighbours stay out of scope and fall back
- GIVEN a body whose neighbour walls are NON-planar (curved), on a booted iOS
  simulator
- WHEN `cc_full_round_fillet_faces` runs
- THEN the operation SHALL NOT claim a dihedral tangent-cylinder full round, and
  SHALL instead return a valid standard-fillet fallback recorded as deferred with
  the measured tangency gap (per the honest-fallback requirement)
