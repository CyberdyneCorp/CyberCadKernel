# native-ssi

## ADDED Requirements

### Requirement: Additive full-seam-coverage helper for the two-operand box-face∩freeform-wall seam, prior controls byte-frozen

Any SSI helper the two-operand freeform boolean requires SHALL be added strictly
additively. If the boolean needs a helper to trace the single box-face∩freeform-wall
seam over the two-operand param box (so the marcher covers the whole transversal seam
without exiting the box early), that helper SHALL be added to the SSI marcher STRICTLY
ADDITIVELY: as a NEW option field (e.g. a param-box union spanning
the two operands' overlap) defaulted to the CURRENT behaviour, leaving every prior
seeding and marching control BYTE-FROZEN. The helper SHALL be proven INERT on every
existing SSI caller — the elementary analytic conic family, the subdivision seeder, the
predictor-corrector marcher, the tangent/branch/chart-singularity robustness paths — by
byte-identical seeding/marching outputs versus the pre-change baseline BEFORE it is used
by the two-operand boolean. The helper SHALL reuse the EXISTING
`ssi::trace_intersection` / `traceWallSeam` machinery for the plane∩Bézier seam (the same
seam the landed single-operand CUT already traces); it SHALL introduce NO new curved-SSI
capability and NO `cc_*` ABI surface, and SHALL keep `src/native/**` free of OCCT includes.
If no helper is needed (the existing `traceWallSeam` param box already covers the seam),
this requirement SHALL be satisfied vacuously and NO SSI source SHALL change.

#### Scenario: The additive helper leaves every prior SSI control byte-identical (host)

- GIVEN the SSI seeding and marching controls and their full existing suite (elementary conic family, subdivision seeder, predictor-corrector marcher, tangent/branch/chart-singularity paths) before and after this change
- WHEN each is exercised at the same inputs and compared against the pre-change baseline
- THEN every seeding/marching output SHALL be byte-identical (the helper is proven inert), the addition SHALL be a NEW defaulted option that changes no existing caller, and there SHALL be zero OCCT includes under `src/native/**`

#### Scenario: The two-operand box-face seam is traced by the existing plane∩Bézier machinery (host)

- GIVEN the box's cutting-plane and operand `A`'s Bézier wall in the single-curved-seam pose on a host build with NO OCCT
- WHEN the seam is traced via the existing `traceWallSeam` / `ssi::trace_intersection` machinery over the two-operand param box
- THEN it SHALL return one well-formed transversal `WLine` (`points.size() ≥ 2`, status `Closed`/`BoundaryExit`) — the same seam capability the landed single-operand CUT already uses, with NO new curved-SSI capability introduced
