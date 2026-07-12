# native-booleans

## ADDED Requirements

### Requirement: Exact-NURBS face split by an analytic curved face (NURBS roadmap Layer-3, slice 2)

The native boolean library SHALL provide `nurbsFaceCurvedSplit(wall, base, cutter, side,
meshDeflection)` (namespace `cybercad::native::boolean`, OCCT-free, header-only,
substrate-gated `CYBERCAD_HAS_NUMSCI`) — the SECOND exact-NURBS B-rep boolean: a split of
an operand whose curved wall is a genuine **NURBS `FaceSurface`** (`Kind::BSpline`) by an
**ANALYTIC CURVED** cutter (Cylinder / Sphere / Cone). It extends the L3-S1 plane split
(`nurbsFacePlaneSplit`) from a planar cutter to an analytic curved cutter, so the seam is a
curve on BOTH curved surfaces and the sew is curved-NURBS↔analytic-CURVED. Given the trimmed
NURBS `wall` FACE, its flat closing `base` (a `Kind::Plane` face), the `cutter` solid, and a
keep `side`, the verb SHALL compose the L3-S2 pipeline:

1. **ADMIT** the operands: a `Kind::BSpline` NURBS `wall` (else `WallNotNurbs`), a
   `Kind::Plane` `base` (else `BaseNotPlanar`), and a `cutter` that `recogniseCurvedSolid`
   folds into ONE analytic `CurvedSolid` (Cylinder/Sphere/Cone; else `CutterNotCurved`).
2. **TRACE** `wall ∩ cutter` into the closed interior seam WLine, building the NURBS operand
   adapter (`npsdetail::makeWallAdapter`) and the curved cutter's own
   `CurvedSolid::adapter()`, driving `ssi::trace_intersection`.
3. **READ the seam pcurve DIRECTLY** from the WLine — `(u1,v1)` on the NURBS wall AND
   `(u2,v2)` on the curved cutter (it SHALL NOT call the general `constructPcurve`) — gated
   by a fidelity check that the NURBS surface at each `(u1,v1)` AND the curved cutter at each
   `(u2,v2)` each equal that node's 3-D point within a scale-relative tolerance (the seam
   lies on BOTH surfaces), AND that the node's on-both-surfaces residual is within tolerance.
   A drifted seam on EITHER operand SHALL be REJECTED (`SeamOffSurface`), never welded.
4. **SPLIT** the NURBS wall along that seam with `splitFaceSmoothTrim` into the enclosed disk
   + the annulus (seam as a hole).
5. **KEEP** the sub-face on the chosen side by a CURVED-solid membership test
   (`classifyPoint(cutter, ·)` — inside/outside the cylinder/sphere/cone) at the sub-face's
   trim centroid; an ON (boundary-coincident, ambiguous) membership SHALL decline
   (`KeepFaceUnusable`). `KeepSide::Above` keeps the sub-face INSIDE the cutter (COMMON);
   `KeepSide::Below` keeps the sub-face OUTSIDE the cutter (CUT).
6. **WELD** the kept NURBS sub-face + the kept flat base (kept when its centroid is on the
   keep side) + a synthesized CURVED cap on the cutter surface bounded by the seam — a
   deflection-bounded PLANAR-TRIANGLE FAN whose OUTER ring is the EXACT traced seam nodes
   (bit-identical to the disk's seam chords) and whose interior points are evaluated ON the
   cutter (so the cap follows its curvature) — into a Shell → Solid.

The verb SHALL perform a mandatory self-verify — mesh the result (M0) and require it be
**watertight** (a closed 2-manifold) AND have a **positive, finite enclosed volume** — and
SHALL return that Solid on success. On ANY decline it SHALL return a NULL Shape with a
measured `NurbsCurvedSplitDecline` reason and SHALL NEVER emit a leaky or partial solid; NO
tolerance SHALL be widened. The verb SHALL remain OCCT-free and host-buildable (under the
substrate), SHALL reference no OCCT / `IEngine` type, SHALL add no `cc_*` facade entry point,
and SHALL NOT modify `boolean/nurbs_plane_split.h`, `boolean/ssi_boolean.{h,cpp}`,
`boolean/assemble.h`, `boolean/face_split.h`, `src/native/ssi`, `src/native/topology`, or
`src/native/math` (it composes them byte-identically).

#### Scenario: A genuine NURBS bowl cut by a sphere gives the closed-form lens volume (host)
- GIVEN a NURBS-walled bowl — a `Kind::BSpline` degree-2 paraboloid reproducing
  `z = a·(x²+y²)` exactly (trimmed by a rim circle) plus a flat top-lid — and a genuine
  analytic SPHERE cutter of radius `Rs` centred on the axis at height `zc`, whose lower
  surface meets the paraboloid in ONE closed interior circle of radius `ρ`
- WHEN `nurbsFaceCurvedSplit(wall, base, sphere, KeepSide::Below, deflection)` is invoked on
  the host with the numsci substrate
- THEN it returns a non-NULL `Solid` (decline `Ok`) whose meshed result is watertight with
  Euler χ = 2, whose enclosed volume equals the closed-form lens
  `2π[zc·ρ²/2 − a·ρ⁴/4] − (2π/3)[Rs³ − (Rs²−ρ²)^{3/2}]` within the curved-tessellation band
  and CONVERGES to it monotonely as the deflection is refined, and whose seam fidelity on
  BOTH the NURBS wall (`S_F(u,v)==C`) and the sphere (`S_G(u,v)==C`) and its on-both-surfaces
  residual are all below the scale-relative tolerance (DISAGREED = 0)

#### Scenario: The complementary keep side gives a watertight COMMON piece (host)
- GIVEN the same NURBS bowl and sphere cutter
- WHEN `nurbsFaceCurvedSplit(..., KeepSide::Above, ...)` is invoked
- THEN it returns a watertight `Solid` (Euler χ = 2) whose enclosed volume is positive and
  whose seam lies on both curved surfaces (the inside-the-sphere keep side)

#### Scenario: A non-curved cutter is rejected before any geometry work (host)
- GIVEN a `cutter` that `recogniseCurvedSolid` does not fold into an analytic
  Cylinder/Sphere/Cone (e.g. a planar face passed as the cutter)
- WHEN `nurbsFaceCurvedSplit(...)` is invoked
- THEN it returns a NULL `Shape` with decline `CutterNotCurved`

#### Scenario: A non-NURBS wall is rejected before any geometry work (host)
- GIVEN a wall face that is not a `Kind::BSpline` NURBS surface (e.g. a `Kind::Plane` face
  passed as the wall)
- WHEN `nurbsFaceCurvedSplit(...)` is invoked
- THEN it returns a NULL `Shape` with decline `WallNotNurbs`

#### Scenario: A cutter that produces no usable closed seam honest-declines to NULL (host)
- GIVEN the NURBS bowl and a sphere cutter positioned far away so `wall ∩ cutter` is not a
  usable closed interior seam
- WHEN `nurbsFaceCurvedSplit(...)` is invoked
- THEN it returns a NULL `Shape` with a measured `NurbsCurvedSplitDecline` (a no-seam reason
  such as `SeamUnusable`, `SeamOffSurface`, `SmoothSplitFailed`, or `KeepFaceUnusable`),
  never a fabricated solid

#### Scenario: Native result matches OCCT BRepAlgoAPI_Cut/Common on the same NURBS operand (sim)
- GIVEN the NURBS bowl reconstructed in OCCT as a `Geom_BSplineSurface` degree-2 paraboloid
  trimmed by the rim plus a planar lid (sewn into one cup solid) and the SAME sphere as a
  `BRepPrimAPI_MakeSphere` ball
- WHEN the native `nurbsFaceCurvedSplit` CUT (Below) and COMMON (Above) results are compared,
  on a booted iOS simulator, against `BRepAlgoAPI_Cut(cup, ball)` (the lens) and
  `BRepAlgoAPI_Common(cup, ball)` respectively
- THEN the native and OCCT enclosed volumes agree within the curved-tessellation band for
  both keep sides, both native results are watertight with Euler χ = 2, and the OCCT lens
  volume cross-checks against the closed form
