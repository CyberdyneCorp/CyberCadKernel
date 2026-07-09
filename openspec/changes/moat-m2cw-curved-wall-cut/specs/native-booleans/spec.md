# native-booleans

## ADDED Requirements

### Requirement: Curved-wall freeform half-space CUT / COMMON weld over a closed circular seam, or DECLINE

The native boolean library SHALL provide an OCCT-free, header-only curved-wall
half-space cut verb (`src/native/boolean/curved_wall_cut.h`,
`curvedWallHalfSpaceCut`) that, GIVEN a solid whose SOLE freeform wall is cut by a plane
`P` along a CLOSED SMOOTH curve INTERIOR to that wall (a dome/bowl cut by a horizontal
plane → a circle — the case byte-frozen B2 `splitFace` DECLINES and the B2 smooth-trim
sibling `splitFaceSmoothTrim` resolves), composes recognise[B1]
(`recogniseFreeformSolid`) → trace[M1] (the byte-unchanged
`hscdetail::traceWallSeam`, which returns the closed seam WLine) → split[B2 smooth-trim]
(`splitFaceSmoothTrim` → the disk `faceInside` + the annulus `faceOutside`) → analytic
wall split[B4] (byte-frozen `hscdetail::cutAnalyticFace`) → cross-section CAP synthesis
(ONE flat planar cap on `P` bounded by the seam polyline, built from the SAME straight
chords the smooth-trim split laid on the freeform sub-face, its normal facing the discard
side) → M0 weld into ONE keep-side Solid. `KeepSide::Below` SHALL keep the material where
the cut removes the cap above the seam (CUT); `KeepSide::Above` SHALL keep the sliced-off
cap (COMMON). The verb SHALL be a strictly ADDITIVE SIBLING of `freeformHalfSpaceCut`: it
SHALL NOT modify `freeformHalfSpaceCut`, `splitFaceSmoothTrim`, B2 `splitFace`,
`splitFaceJunction`, the recognisers, the M0 tessellator, M1, or any landed weld path,
and it SHALL consume their primitives BYTE-IDENTICAL.

`curvedWallHalfSpaceCut` SHALL run a mandatory self-verify — mesh the welded result with
the M0 tessellator and require it be WATERTIGHT (a closed 2-manifold) AND enclose a
positive, finite volume — and SHALL return a NULL Shape with a typed
`CurvedWallCutDecline` (the measured blocker) whenever: the operand is not admitted
(`NotAdmitted`); there is no freeform wall (`NoFreeformFace`) or more than one
(`MultiFreeformFace`); the freeform wall is not a Bézier with poles
(`WallSurfaceUnusable`); the traced seam is missing / has fewer than three nodes / is not
a closed interior loop (`SeamUnusable`); the smooth-trim split declines
(`SmoothSplitFailed`); a crossed analytic face declines (`AnalyticCutFailed`); the
synthesized cap is degenerate (`CapDegenerate`); fewer than two survivor faces remain
(`WeldOpen`); the welded result is not watertight (`NotWatertight`); or the enclosed
volume is non-positive / NaN (`VolumeInconsistent`). No tolerance SHALL be weakened to
force a pass; a NULL result (→ OCCT fall-through) SHALL be preferred over any leaky or
partial solid. The verb SHALL remain OCCT-free, SHALL introduce no `cc_*` ABI surface,
and SHALL keep its per-function cognitive complexity within the backend band.

#### Scenario: The CUT keep side welds watertight at the closed-form volume and converges (host, no OCCT)

- GIVEN a steep degree-2 Bézier bowl-cup (a bowl trimmed by a rim circle plus a flat top-lid disk) and the real S3 seam WLine of the bowl intersected with a horizontal plane `z = c` — a CLOSED CIRCLE of radius `ρ = √(c/a)` interior to the rim trim — built on the host with NO OCCT
- WHEN `curvedWallHalfSpaceCut(operand, P, KeepSide::Below, d)` runs across a deflection sweep
- THEN it SHALL return a single closed solid whose M0 mesh is WATERTIGHT with Euler characteristic `χ = 2` at every deflection, whose enclosed volume equals the closed form `π·ρ²·c/2` within the curved-tessellation band and CONVERGES monotonically to it as the deflection tightens

#### Scenario: The COMMON keep side welds watertight at the complementary closed-form volume (host, no OCCT)

- GIVEN the same bowl-cup and closed circular seam, built on the host with NO OCCT
- WHEN `curvedWallHalfSpaceCut(operand, P, KeepSide::Above, d)` runs at its robust deflection
- THEN it SHALL return a single closed solid whose M0 mesh is WATERTIGHT with Euler `χ = 2` at the closed-form volume `V(z≥c) = V(full) − V(z≤c)`, and the closed-form partition identity `V(z≤c) + V(z≥c) = V(full)` SHALL hold exactly

#### Scenario: A deflection at which the weld is not watertight DECLINES to NULL (host, no OCCT)

- GIVEN the bowl-cup and a keep side whose weld does not mesh watertight at a given deflection (the COMMON annulus↔lid rim weld is deflection-fragile)
- WHEN `curvedWallHalfSpaceCut` runs at that deflection
- THEN it SHALL return a NULL Shape with `NotWatertight` (→ OCCT fall-through) and SHALL NEVER return a leaky or partial solid, and SHALL NOT weaken any tolerance to force a pass

#### Scenario: A non-cutting plane or a non-operand DECLINES to NULL (host, no OCCT)

- GIVEN a plane entirely above the bowl (no closed interior seam), or a null / non-solid shape, built on the host with NO OCCT
- WHEN `curvedWallHalfSpaceCut` runs
- THEN it SHALL return a NULL Shape with a typed decline (`SeamUnusable` / `NotAdmitted`) and emit no solid

#### Scenario: Native-vs-OCCT parity of the CUT / COMMON weld (sim, OCCT oracle)

- GIVEN the SAME bowl-cup operand reconstructed in OCCT (a `Geom_BezierSurface` bowl trimmed by the rim circle plus a planar lid disk, sewn into an outward-oriented solid) on a booted iOS simulator
- WHEN OCCT cuts it by `BRepAlgoAPI_Common` against the keep-half box and the native `curvedWallHalfSpaceCut` result is measured by the native M0 tessellator
- THEN the native CUT (Below, multiple deflections) and COMMON (Above, robust deflection) SHALL match the OCCT result on VOLUME (relative, within the curved band, each cross-checked to the closed form), AREA (relative), WATERTIGHTNESS (closed 2-manifold), TOPOLOGY (Euler `χ = 2`), BBOX (per-axis, spatial band), and one-sided HAUSDORFF (native→OCCT, spatial band), with FIXED tolerances that are never widened, and the fragile-deflection case SHALL DECLINE to NULL
