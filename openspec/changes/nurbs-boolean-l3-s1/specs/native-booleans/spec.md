# native-booleans

## ADDED Requirements

### Requirement: Exact-NURBS face split by a plane (NURBS roadmap Layer-3, slice 1)

The native boolean library SHALL provide `nurbsFacePlaneSplit(wall, base, P, side,
meshDeflection)` (namespace `cybercad::native::boolean`, OCCT-free, header-only) — the
FIRST exact-NURBS B-rep boolean: a half-space split of an operand whose curved wall is a
genuine **NURBS `FaceSurface`** (`Kind::BSpline`, non-rational first / rational
admitted), cut by a planar half-space. Given the trimmed NURBS `wall` FACE, its flat
closing `base` (a `Kind::Plane` face), the cut plane `P`, and a keep `side`, the verb
SHALL compose the L3-S1 pipeline:

1. **TRACE** `wall ∩ P` into the closed interior seam WLine, building the NURBS operand
   adapter with `ssi::makeBSplineAdapter` (non-rational) or `ssi::makeNurbsAdapter`
   (rational) + `ssi::makePlaneAdapter` and driving `ssi::trace_intersection`.
2. **READ the seam pcurve DIRECTLY** from the WLine's per-node `(u1,v1)` (it SHALL NOT
   call the general `constructPcurve`), gated by a fidelity check that the NURBS surface
   evaluated at each `(u1,v1)` equals that node's 3-D point within a scale-relative
   tolerance (the S(pcurve)==C invariant) AND that the node's on-both-surfaces residual
   is within tolerance — a drifted seam SHALL be REJECTED (`SeamOffSurface`), never
   welded.
3. **SPLIT** the NURBS wall along that seam with `splitFaceSmoothTrim` into the enclosed
   disk + the annulus (seam as a hole).
4. **KEEP** the sub-face on the chosen half-space side of `P` (a closed-form
   signed-distance side test at the sub-face's trim centroid).
5. **WELD** the kept NURBS sub-face + the kept flat base (kept when it lies on the keep
   side) + a synthesized flat cap on `P` bounded by the seam polyline (the M0w
   curved↔FLAT weld) into a Shell → Solid.

The verb SHALL perform a mandatory self-verify — mesh the result (M0) and require it be
**watertight** (a closed 2-manifold) AND have a **positive, finite enclosed volume** —
and SHALL return that Solid on success. On ANY decline it SHALL return a NULL Shape with
a measured `NurbsPlaneSplitDecline` reason and SHALL NEVER emit a leaky or partial
solid. `KeepSide::Below` keeps the material below `P` (CUT); `KeepSide::Above` keeps the
material above `P` (COMMON); the two sides SHALL partition the operand
(`V(below)+V(above)=V(full)`). The verb SHALL remain OCCT-free and host-buildable, SHALL
reference no OCCT / `IEngine` type, SHALL add no `cc_*` facade entry point, and SHALL
NOT modify `boolean/assemble.h`, `boolean/face_split.h`, `src/native/ssi`,
`src/native/topology/trimmed_nurbs`, or `src/native/math` (it composes them
byte-identically).

#### Scenario: A genuine NURBS bowl-cup cut below a plane matches the closed-form volume (host)
- GIVEN a NURBS-walled bowl-cup — a `Kind::BSpline` degree-2 bowl reproducing
  `z = a·(x²+y²)` exactly (trimmed by a rim circle) plus a flat top-lid — and the
  horizontal cut plane `z = c` with `0 < c < a·R²`
- WHEN `nurbsFacePlaneSplit(wall, base, P, KeepSide::Below, deflection)` is invoked on
  the host with the numsci substrate
- THEN it returns a non-NULL `Solid` (decline `Ok`) whose meshed result is watertight
  with Euler χ = 2, whose enclosed volume equals the closed form `π·ρ²·c/2`
  (`ρ = √(c/a)`) within the curved-tessellation band, and whose seam fidelity
  (`S(u,v)==C`) and on-both-surfaces residual are both below the scale-relative
  tolerance (DISAGREED = 0)

#### Scenario: The complementary keep side gives COMMON and the two sides partition the operand (host)
- GIVEN the same NURBS bowl-cup and cut plane `z = c`
- WHEN `nurbsFacePlaneSplit(..., KeepSide::Above, ...)` is invoked
- THEN it returns a watertight `Solid` whose enclosed volume equals `V(full) − π·ρ²·c/2`
  within the band, AND the Below and Above enclosed volumes sum to the full bowl-cup
  volume `π·a·R⁴/2` within the band (partition closure)

#### Scenario: A plane that produces no usable closed seam honest-declines to NULL (host)
- GIVEN the NURBS bowl-cup and a cut plane positioned entirely above the cup
  (`z > a·R²`), so `wall ∩ P` is not a usable closed interior seam
- WHEN `nurbsFacePlaneSplit(...)` is invoked
- THEN it returns a NULL `Shape` with a measured `NurbsPlaneSplitDecline` (a no-seam
  reason such as `SeamUnusable` or `SmoothSplitFailed`), never a fabricated solid

#### Scenario: A non-NURBS wall is rejected before any geometry work (host)
- GIVEN a wall face that is not a `Kind::BSpline` NURBS surface (e.g. a `Kind::Plane`
  face passed as the wall)
- WHEN `nurbsFacePlaneSplit(...)` is invoked
- THEN it returns a NULL `Shape` with decline `WallNotNurbs`

#### Scenario: Native result matches OCCT BRepAlgoAPI_Cut on the same NURBS operand (sim)
- GIVEN the NURBS bowl-cup reconstructed in OCCT as a `Geom_BSplineSurface` degree-2
  bowl trimmed by the rim plus a planar lid, sewn into one solid
- WHEN the native `nurbsFacePlaneSplit` CUT and COMMON results are compared, on a booted
  iOS simulator, against `BRepAlgoAPI_Cut` of the OCCT operand by the same half-space
- THEN the native and OCCT enclosed volumes agree within the curved-tessellation band
  for both keep sides, both native results are watertight with Euler χ = 2, and the OCCT
  volumes cross-check against the closed form
