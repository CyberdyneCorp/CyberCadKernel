# native-booleans

## ADDED Requirements

### Requirement: Freeform↔freeform CLOSED-SEAM COMMON welds a coherently-oriented lens at the closed-form volume; CUT honest-declines

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
5. weld the survivors (M0 `SolidMesher`) with an ORIENTATION-COHERENCE REPAIR — because the
   two survivor caps each inherit their parent wall's orientation (`A` opens UP, `B` opens
   DOWN) a naive weld is watertight (UNDIRECTED) but orientation-INCONSISTENT (its signed
   volume is wrong), so the verb SHALL enforce the DIRECTED-edge invariant
   `tess::isConsistentlyOriented` (every seam half-edge used once forward and once reversed;
   0 same-direction duplicates), flipping exactly one cap to obtain a coherent outward-normal
   boundary, and SHALL DECLINE (`NotWatertight`) if no single flip yields a consistently-
   oriented 2-manifold — then run a MANDATORY self-verify: the M0 mesh WATERTIGHT AND
   consistently-oriented AND a positive finite enclosed volume within the op-volume UPPER
   bound (CUT ⊂ `A`, COMMON ⊂ `A` and ⊂ `B`) AND, when the analytic op-volume is supplied,
   within a TWO-SIDED deflection-bounded band of it (`VolumeInconsistent` otherwise),
   returning the welded solid ONLY when it passes.

On ANY failure the verb SHALL return a NULL `Shape` (→ OCCT `BRepAlgoAPI_{Cut,Common}`)
with a typed `FfCutDecline` (including the additive `VolumeInconsistent`), SHALL NEVER emit
a leaky / partial / orientation-inconsistent / wrong-volume solid, and SHALL NOT weaken any
tolerance to force a pass. With the M0-rim tessellator weld in place the two-CURVED-side
closed seam welds watertight at the working deflections, so COMMON (the lens) SHALL weld a
coherently-oriented solid whose volume matches the closed-form `π·H²/(4a)` within the
deflection band and CONVERGES MONOTONICALLY as deflection refines (a direct positive volume
whose smooth caps under-estimate it). CUT SHALL likewise weld a coherently-oriented solid
whose volume matches the closed-form `V(A) − π·H²/(4a)` within the deflection band, once the
survivor membership probe RESPECTS HOLES (the annulus interior-sample vote, not the apex-
adjacent outer centroid). Because the CUT volume is a CANCELLATION DIFFERENCE (the shell
enclosing ≈V(A) minus the B-disk curved ceiling ≈V(lens)), the two independently-tessellated
curved caps' O(deflection) signed-volume residuals partially cancel, so the CUT relative
error does NOT shrink strictly monotonically — it stays inside a SHRINKING ENVELOPE (a fixed
~2% band per level) whose best-so-far minimum converges to within ~1% of the closed form.
This is normal adaptive-tessellation behaviour, not a mis-weld: every level is watertight,
boundary-edge-free, consistently oriented, and under-estimates the closed form. The change
SHALL be
strictly ADDITIVE: it SHALL NOT modify `splitFaceSmoothTrim`, B2 `splitFace`,
`recogniseFreeformSolid`, `classifyPointInMesh`, the M0 tessellator (mesher/CDT/rim weld),
M1, or any landed boolean header, and SHALL consume their primitives BYTE-IDENTICAL (the
only additive touch outside the verb is the new directed-edge checks in `tessellate/mesh.h`,
leaving the existing volume/watertight checks untouched). The verb SHALL remain OCCT-free,
SHALL introduce no `cc_*` ABI surface, and SHALL keep its per-function cognitive complexity
within the backend band.

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

#### Scenario: COMMON welds at the closed-form volume and converges; CUT honest-declines (host, no OCCT)

- GIVEN the two-bowl-cup operands and the composed verb
- WHEN `freeformFreeformClosedSeamCut(A, B, Common, d, why, V(A∩B))` runs across a deflection
  sweep, and `freeformFreeformClosedSeamCut(A, B, Cut, d)` runs likewise
- THEN COMMON returns a NON-NULL `Shape` with `why == Ok` that is WATERTIGHT AND
  consistently-oriented (`tess::isConsistentlyOriented`), whose meshed volume equals the
  closed-form lens `π·H²/(4a)` within the deflection-bounded band, under-estimates it (a
  smooth cap triangulation), and whose relative error SHRINKS MONOTONICALLY as the deflection
  refines; CUT likewise returns a NON-NULL WATERTIGHT, boundary-edge-free, consistently-
  oriented `Shape` with `why == Ok` whose volume equals the closed-form `V(A) − π·H²/(4a)`
  within the band and under-estimates it, but — being a cancellation difference — its relative
  error need NOT shrink strictly monotonically: it stays inside a fixed ~2% SHRINKING-ENVELOPE
  band per level with a best-so-far minimum within ~1% of the closed form; NEITHER leg emits a
  leaky/partial/wrong-volume solid; and a non-operand declines `NotAdmittedA`, a
  non-intersecting second operand declines `SeamUnusable`

#### Scenario: The shared seam is grounded on OCCT and native COMMON matches the OCCT oracle (sim, native-vs-OCCT)

- GIVEN the SAME two bowl-cups reconstructed in OCCT (each a `Geom_BezierSurface` bowl +
  a planar lid on the shared rim, sewn, interior-classified outward) on a booted iOS simulator
- WHEN the native seam is traced and OCCT `BRepAlgoAPI_Cut(A,B)` / `Common(A,B)` (the ORACLE)
  are computed
- THEN each native seam node lies ON BOTH OCCT Bézier surfaces (BRepExtrema ≤ 1e-4, measured
  ~1e-14); OCCT `Cut` = `V(A) − π·H²/(4a)` and `Common` = `π·H²/(4a)` to ≤ 2%; the native
  COMMON WELDS and its meshed volume matches OCCT `BRepAlgoAPI_Common` + `BRepGProp` within
  the deflection-bounded band, converging as deflection refines, while native CUT DECLINES to
  NULL (the correct honest fallthrough — the native path never emits a leaky/wrong solid);
  and a non-intersecting operand yields native `SeamUnusable` and an OCCT no-op
