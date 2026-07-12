# native-booleans

## ADDED Requirements

### Requirement: Exact-NURBS face split by a freeform NURBS face (NURBS roadmap Layer-3, slice 3)

The native boolean library SHALL provide `nurbsFaceFreeformSplit(F, G, op, meshDeflection,
analyticOpVolume)` (namespace `cybercad::native::boolean`, OCCT-free, header-only,
substrate-gated `CYBERCAD_HAS_NUMSCI`) — the THIRD (deepest) exact-NURBS B-rep boolean: a
split of an operand whose curved wall is a genuine **NURBS `FaceSurface`** (`Kind::BSpline`)
by ANOTHER operand whose curved wall is ALSO a genuine NURBS `FaceSurface` (BOTH operands
arbitrary NURBS — the general freeform↔freeform sew). It extends the L3-S2 analytic-curved
split (`nurbsFaceCurvedSplit`) from an analytic cutter to a FREEFORM NURBS cutter, so the
seam is a curve on BOTH freeform surfaces and the sew is curved-NURBS↔curved-NURBS (both
kept caps are curved NURBS sub-faces welding bit-for-bit along the shared seam). Given two
freeform bowl-cup operands `F`, `G` whose single freeform walls meet in ONE CLOSED interior
seam and an `FfOp`, the verb SHALL compose the L3-S3 pipeline:

1. **RECOGNISE** both operands with `recogniseFreeformSolid` (else `NotAdmittedF` /
   `NotAdmittedG`); each operand SHALL present EXACTLY one freeform wall, and that wall SHALL
   be a `Kind::BSpline` (NURBS) `FaceSurface` with poles (else `WallFNotNurbs` /
   `WallGNotNurbs` — a Bézier wall is `freeform_freeform_cut.h`'s case, an analytic cutter is
   L3-S2's).
2. **TRACE** `F.wall ∩ G.wall` into the closed interior seam WLine, building the L3 NURBS
   operand adapter (`npsdetail::makeWallAdapter`) on BOTH walls and driving
   `ssi::trace_intersection`; the WLine node carries `(u1,v1)` on F's wall AND `(u2,v2)` on
   G's wall. A missing / open / non-closed seam (< 3 nodes or not `TraceStatus::Closed`)
   SHALL decline (`SeamUnusable`).
3. **READ the seam pcurve DIRECTLY** from the WLine — `(u1,v1)` on F's NURBS wall AND
   `(u2,v2)` on G's NURBS wall (it SHALL NOT call the general `constructPcurve`) — gated by a
   fidelity check that F's surface at each `(u1,v1)` AND G's surface at each `(u2,v2)` each
   equal that node's 3-D point within a scale-relative tolerance (the seam lies on BOTH
   freeform surfaces), AND that the node's on-both-surfaces residual is within tolerance. A
   drifted seam on EITHER operand SHALL be REJECTED (`SeamOffSurface`), never welded.
4. **SPLIT** F's NURBS wall along the seam `(u1,v1)` and G's NURBS wall along a `(u2,v2)`-
   re-keyed copy of the SAME seam with `splitFaceSmoothTrim`, each into the enclosed disk +
   the annulus (else `SmoothSplitFailedF` / `SmoothSplitFailedG`).
5. **SELECT** the survivor sub-face of each split by a mesh-membership test
   (`classifyPointInMesh`) of its trim centroid in the OTHER operand's pre-cut mesh; for
   `FfOp::Common` the kept sub-faces are F's disk INSIDE G and G's disk INSIDE F (the lens).
   An On/Unknown/wrong-side membership SHALL decline (`ClassifyAmbiguous`).
6. **WELD** the two kept curved NURBS sub-faces — which SHARE the EXACT seam nodes (the
   bit-identical outer ring `splitFaceSmoothTrim` lays on both sides), so the M0 mesher
   position-welds NURBS-disk↔NURBS-disk — into a Shell → Solid, repairing orientation
   coherence (the directed-edge invariant; exactly one cap reversed) so the boundary has
   coherent outward normals.

The verb SHALL perform a mandatory self-verify — the weld SHALL yield a mesh that is
**watertight** (a closed 2-manifold) AND **consistently oriented**, with a **positive, finite
enclosed volume**, within an op-volume UPPER bound (`FfOp::Common` ⊂ F and ⊂ G), and — when
`analyticOpVolume` is supplied — within a deflection-scaled TWO-SIDED band of it (so a
too-small orientation-collapsed volume is rejected) — and SHALL return that Solid on success.
On ANY decline it SHALL return a NULL Shape with a measured `NurbsFreeformSplitDecline` reason
and SHALL NEVER emit a leaky or partial solid; NO tolerance SHALL be widened. The verb SHALL
remain OCCT-free and host-buildable (under the substrate), SHALL reference no OCCT / `IEngine`
type, SHALL add no `cc_*` facade entry point, and SHALL NOT modify
`boolean/nurbs_plane_split.h`, `boolean/nurbs_curved_split.h`, `boolean/freeform_freeform_cut.h`,
`boolean/freeform_operand.h`, `boolean/freeform_membership.h`, `boolean/smooth_trim_split.h`,
`boolean/ssi_boolean.{h,cpp}`, `boolean/assemble.h`, `boolean/face_split.h`, `src/native/ssi`,
`src/native/topology`, or `src/native/math` (it composes them byte-identically).

#### Scenario: Two genuine NURBS bowls give the closed-form COMMON lens volume (host)
- GIVEN two coaxial NURBS-walled bowl-cups — an UP bowl F (a `Kind::BSpline` degree-2
  paraboloid reproducing `z = a·(x²+y²)` exactly, trimmed by a rim, closed by a flat top-lid)
  and a DOWN dome G (a `Kind::BSpline` degree-2 paraboloid reproducing `z = H − a·(x²+y²)`,
  the same rim, a flat bottom-lid) — whose two NURBS walls meet in ONE closed interior circle
  of radius `ρ = √(H/2a)` at `z = H/2`
- WHEN `nurbsFaceFreeformSplit(F, G, FfOp::Common, deflection, V_lens)` is invoked on the host
  with the numsci substrate
- THEN it returns a non-NULL `Solid` (decline `Ok`) whose meshed result is watertight with
  Euler χ = 2 and consistently oriented, whose enclosed volume equals the closed-form lens
  `V = π·H²/(4a)` within the curved-tessellation band and CONVERGES to it monotonely as the
  deflection is refined, and whose seam fidelity on BOTH NURBS walls (`S_F(u,v)==C` AND
  `S_G(u,v)==C`) and its on-both-surfaces residual are all below the scale-relative tolerance
  (DISAGREED = 0)

#### Scenario: The shared seam is a closed circle on BOTH NURBS walls (host)
- GIVEN the same two NURBS bowl-cups
- WHEN the shared seam `F.wall ∩ G.wall` is traced through the NURBS operand adapters
- THEN it is a CLOSED WLine of radius `ρ` on BOTH walls' `(u,v)` (to ~1e-13), on both surfaces
  (the trace residual), at height `z = H/2`, and interior to both rim trims

#### Scenario: A non-NURBS wall is rejected before any weld work (host)
- GIVEN an operand whose single freeform wall is not a `Kind::BSpline` NURBS surface (or an
  operand that presents no single freeform wall), or a null operand
- WHEN `nurbsFaceFreeformSplit(...)` is invoked
- THEN it returns a NULL `Shape` with a measured decline (`WallFNotNurbs` / `WallGNotNurbs` /
  `NotAdmittedF` / `NotAdmittedG`)

#### Scenario: Non-intersecting operands honest-decline to NULL (host)
- GIVEN two NURBS bowl-cups positioned so their walls do not intersect (no shared seam)
- WHEN `nurbsFaceFreeformSplit(...)` is invoked
- THEN it returns a NULL `Shape` with decline `SeamUnusable`, never a fabricated solid

#### Scenario: The apex-ambiguous CUT leg honest-declines to NULL (host)
- GIVEN the same two NURBS bowl-cups
- WHEN `nurbsFaceFreeformSplit(F, G, FfOp::Cut, ...)` is invoked (a leg OUT of the L3-S3
  envelope — its apex-adjacent survivor membership is ambiguous, as in the Bézier case)
- THEN it returns a NULL `Shape` with a measured weld/membership decline
  (`ClassifyAmbiguous` / `NotWatertight` / `VolumeInconsistent`), never a leaky or wrong solid

#### Scenario: Native result matches OCCT BRepAlgoAPI_Common on the same NURBS operands (sim)
- GIVEN the two NURBS bowl-cups reconstructed in OCCT (each a `Geom_BSplineSurface` degree-2
  paraboloid trimmed by the rim plus a planar lid, sewn into one cup solid)
- WHEN the native `nurbsFaceFreeformSplit` COMMON result is compared, on a booted iOS
  simulator, against `BRepAlgoAPI_Common(F, G)`
- THEN the native and OCCT enclosed volumes agree within the curved-tessellation band, the
  native result is watertight with Euler χ = 2 and consistently oriented, and the OCCT lens
  volume cross-checks against the closed form
