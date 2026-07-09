# native-booleans

## ADDED Requirements

### Requirement: Freeform↔freeform CLOSED-SEAM CUT / COMMON composes the M2 substrate and HONEST-DECLINES the two-curved-side weld

The native boolean layer SHALL provide an additive OCCT-free header-only verb
`freeformFreeformClosedSeamCut(A, B, op, deflection, why)`
(`src/native/boolean/freeform_freeform_cut.h`, namespace `cybercad::native::boolean`) that
composes the landed M2 substrate for the CUT (`A − B`) and COMMON (`A ∩ B`) of TWO
freeform (Bézier) bowl-cup operands whose two CURVED walls intersect in ONE CLOSED curved
seam. The verb SHALL:

1. recognise BOTH operands via B1 `recogniseFreeformSolid` (each with EXACTLY one freeform
   wall and one analytic lid), declining `NotAdmittedA` / `NotAdmittedB` /
   `MultiFreeformFace` / `WallSurfaceUnusable` otherwise;
2. trace the shared seam `A.wall ∩ B.wall` via M1 `ssi::trace_intersection` over two
   `ssi::makeBezierAdapter` surfaces, requiring a CLOSED WLine carrying `(u1,v1)` on `A`
   and `(u2,v2)` on `B` (else `SeamUnusable`);
3. split BOTH walls by the SAME 3-D seam through the byte-frozen B2 smooth-trim
   `splitFaceSmoothTrim` — `A`'s wall by the `(u1,v1)` pcurve and `B`'s wall by a
   `(u2,v2)`-re-keyed copy of the seam — each into a disk + an annulus (else
   `SmoothSplitFailedA` / `SmoothSplitFailedB`);
4. select the survivors for the requested op by B3 `classifyPointInMesh` membership of
   each sub-face's centroid against the OTHER operand's mesh (CUT keeps `A`'s material
   OUTSIDE `B` + `A`'s lids OUTSIDE `B` + `B`'s wall sub-face INSIDE `A`; COMMON keeps
   `A`'s wall sub-face INSIDE `B` + `B`'s wall sub-face INSIDE `A`), declining
   `ClassifyAmbiguous` on an On/Unknown/wrong-side verdict;
5. weld the survivors (M0 `SolidMesher`) and run a MANDATORY self-verify (require the M0
   mesh WATERTIGHT AND a positive finite enclosed volume within the op-volume bound — CUT
   ⊂ `A`, COMMON ⊂ `A` and ⊂ `B`), returning the welded solid ONLY when it passes.

On ANY failure the verb SHALL return a NULL `Shape` (→ OCCT `BRepAlgoAPI_{Cut,Common}`)
with a typed `FfCutDecline`, SHALL NEVER emit a leaky / partial / wrong solid, and SHALL
NOT weaken any tolerance to force a pass. In particular, because the two-CURVED-side
closed-seam weld (a shared closed seam between two independently-tessellated CURVED
sub-faces) is gated on the byte-frozen M0 tessellator — the M0w seam pin welds a CURVED
sub-face to a FLAT chord the other side already sits on, but does NOT weld two curved
sides — the verb SHALL HONEST-DECLINE (`NotWatertight` or `ClassifyAmbiguous`) rather than
emit a non-watertight solid. The change SHALL be strictly ADDITIVE: it SHALL NOT modify
`splitFaceSmoothTrim`, B2 `splitFace`, `recogniseFreeformSolid`, `classifyPointInMesh`,
the M0 tessellator, M1, or any landed boolean header, and SHALL consume their primitives
BYTE-IDENTICAL. The verb SHALL remain OCCT-free, SHALL introduce no `cc_*` ABI surface,
and SHALL keep its per-function cognitive complexity within the backend band.

#### Scenario: The pipeline traces + splits both curved walls and the closed-form partition is consistent (host, no OCCT)

- GIVEN two coaxial paraboloid bowl-cups — an UP bowl-cup `A` (bowl `z = a·r²` + a flat top
  lid) and a DOWN dome-cup `B` (dome `z = H − a·r²` + a flat bottom lid), placed so `B`
  slices `A` only through the two curved walls — built on the host with NO OCCT
- WHEN the recognise → trace → B2 smooth-trim pipeline runs on both operands
- THEN both operands B1-admit with exactly one freeform wall + one analytic lid; the real
  M1 seam is a CLOSED circle of radius `ρ = √(H/2a)` at `z = H/2`, interior to both rim
  trims, whose nodes lie on BOTH walls' `(u,v)` to ~1e-13 and on both surfaces to the trace
  residual; each wall's smooth-trim disk area equals the closed-form `π·ρ²`; and the
  closed-form oracles tile exactly (`V(A−B) + V(A∩B) = V(A)`, `V(A∩B) = π·H²/(4a)`)

#### Scenario: CUT and COMMON HONEST-DECLINE to NULL — never a leaky solid (host, no OCCT)

- GIVEN the two-bowl-cup operands and the composed verb
- WHEN `freeformFreeformClosedSeamCut(A, B, {Cut,Common}, d)` runs across a deflection sweep
- THEN each call returns a NULL `Shape` with a measured `FfCutDecline`
  (`NotWatertight` or `ClassifyAmbiguous`) — the two-curved-side closed-seam weld is
  tessellator-gated, so the disciplined outcome is NULL → OCCT, NEVER a leaky/partial solid;
  and a non-operand declines `NotAdmittedA`, a non-intersecting second operand declines
  `SeamUnusable`

#### Scenario: The shared seam is grounded on OCCT and the honest fallthrough is verified (sim, native-vs-OCCT)

- GIVEN the SAME two bowl-cups reconstructed in OCCT (each a `Geom_BezierSurface` bowl +
  a planar lid on the shared rim, sewn, interior-classified outward) on a booted iOS simulator
- WHEN the native seam is traced and OCCT `BRepAlgoAPI_Cut(A,B)` / `Common(A,B)` (the ORACLE)
  are computed
- THEN each native seam node lies ON BOTH OCCT Bézier surfaces (BRepExtrema ≤ 1e-4, measured
  ~1e-14); OCCT `Cut` = `V(A) − π·H²/(4a)` and `Common` = `π·H²/(4a)` to ≤ 2% (so the
  closed-form host oracle is the correct answer OCCT owns); the native verb DECLINES CUT and
  COMMON to NULL (the correct honest fallthrough — the native path never emits a leaky/wrong
  solid); and a non-intersecting operand yields native `SeamUnusable` and an OCCT no-op
