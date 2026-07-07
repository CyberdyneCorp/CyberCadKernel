# native-exchange

## ADDED Requirements

### Requirement: Sim-verified STEP admission of a foreign trimmed B-spline surface face via a faithful curved-edge pcurve arm and reconstruction guard, else DECLINE

The STEP reader SHALL admit ONE foreign `B_SPLINE_SURFACE_WITH_KNOTS` surface
(rational `weights` included) bounded by a real, genuinely trimmed `EDGE_LOOP` — a
foreign trimmed B-spline/NURBS patch — as a native trimmed `Kind::BSpline` face **only
when a faithful 2D pcurve is reconstructed and verified for EVERY boundary edge**, so
that the landed native trimmed-freeform mesh path (`native-tessellation`,
`trimmedFreeformMesh`) — which SHALL NOT be modified — can mesh it watertight. This
fulfils the STEP-reader admission that the M0 keystone DEFERRED to the OCCT-linked
simulator gate.

For each boundary edge on a `Kind::BSpline` face surface the reader SHALL reconstruct
the pcurve by inverting the surface at the edge's sampled points (`projectBSplineUV`, a
grid-seeded damped-Newton inversion on the analytic surface derivatives — no OCCT, no
external solver): a **straight-in-`(u,v)`** edge (rim / seam / isoparametric trim) SHALL
become a UV `Line` through the two projected endpoints, byte-identical to the reader's
prior generic-linear pcurve for that case; a **genuinely curved** boundary edge SHALL
become a UV `B_SPLINE` pcurve whose 2D poles are densified projected samples and whose
degree and knots are preserved from the 3D edge curve, evaluated as-is by the landed
`trim.h::pcurveValue` `K::BSpline` evaluator.

The reader SHALL then run a **faithful-reconstruction guard** on every boundary edge:
using the SAME `pcurveValue` evaluator the mesher will flatten, and a rational-aware
forward evaluation of the B-spline surface (`math::nurbsSurfacePoint` over the surface
poles / knots / `weights`), it SHALL re-evaluate `S_face(pcurve(t)) = C_edge(t)` at
several parameters across the edge range and require the gap to be within a
**scale-relative** tolerance (`1e-6 · max(1, scale)`, `scale` = the surface control-net
extent) that SHALL NOT be weakened. If ANY boundary edge fails the guard — the surface
inversion did not converge, the reconstructed pcurve does not lie on the patch within
tolerance, or the boundary gap exceeds tolerance — the reader SHALL `decline()` the face
(NULL → OCCT), exactly as it declines any other non-faithful reduction.

An admitted face SHALL be subject to the engine's mandatory watertight + volume/area
self-verify against the OCCT oracle downstream; a native result that is not watertight or
off-volume SHALL be DISCARDED → OCCT, so a wrong or leaky solid is never emitted. The
reader SHALL remain OCCT-free (`src/native/**` links no OCCT / `IEngine` / `EngineShape`
type), the native tessellator SHALL NOT be modified, no tolerance SHALL be weakened, and
the `cc_*` ABI SHALL be unchanged (additive reader behaviour only). The existing Plane /
Cylinder / Cone / Sphere pcurve arms and the bare-periodic full-sphere / full-torus /
full-revolution B-spline admission paths SHALL remain byte-identical.

#### Scenario: A foreign trimmed B-spline face with faithful curved pcurves imports and meshes watertight matching BRepMesh (sim, parity)

- GIVEN a foreign STEP file carrying a `B_SPLINE_SURFACE_WITH_KNOTS` face bounded by a genuinely trimmed `EDGE_LOOP` with a curved boundary edge whose pcurve reconstructs faithfully, imported on a booted iOS simulator with OCCT linked and the native engine active (`cc_set_engine(1)`)
- WHEN the reader reconstructs each boundary pcurve, passes the `S_face(pcurve(t)) = C_edge(t)` guard, admits the face, and the landed M0 tessellator meshes the resulting solid
- THEN the native solid's enclosed volume, surface area, watertight status, triangle envelope, and sub-shape topology SHALL match the OCCT `STEPControl_Reader` re-import + `BRepMesh_IncrementalMesh` oracle within tolerance (the foreign trimmed patch that previously could not be faithfully admitted now meshes watertight)

#### Scenario: A boundary edge whose pcurve does not reconstruct faithfully declines to OCCT (sim)

- GIVEN a foreign STEP file whose trimmed `B_SPLINE_SURFACE_WITH_KNOTS` face has at least one boundary edge whose pcurve cannot be reconstructed within the scale-relative tolerance (non-converging inversion, off-surface reconstructed pcurve, or beyond-tolerance boundary gap)
- WHEN the reader evaluates the faithful-reconstruction guard for that edge
- THEN the reader SHALL `decline()` the face (NULL → OCCT), the file SHALL round-trip through OCCT `STEPControl_Reader` unchanged, no tolerance SHALL have been weakened, and no approximate or leaky native face SHALL be emitted — the honest decline is reported

#### Scenario: The reconstruction guard accepts a faithful pcurve and rejects an off-surface edge (host analytic, no OCCT)

- GIVEN a native-built trimmed `Kind::BSpline` face with a closed-form curved boundary, built on the host with NO OCCT linked, plus a deliberately perturbed off-surface variant of one boundary edge
- WHEN the reader reconstructs each pcurve and runs the `S_face(pcurve(t)) = C_edge(t)` guard (rational eval via `math::nurbsSurfacePoint`)
- THEN the guard SHALL ACCEPT the faithful face (every sampled `t` within `1e-6 · max(1, scale)`, and the meshed solid is watertight with the closed-form enclosed volume within tolerance) AND SHALL REJECT the perturbed variant (`decline()` fires) — proven against a closed-form oracle with no OCCT symbol linked

#### Scenario: The engine self-verify discards a non-watertight admitted face (sim)

- GIVEN a foreign trimmed B-spline face that passes the per-edge pcurve guard but whose native mesh does not close watertight (or whose volume/area does not match the OCCT oracle)
- WHEN the engine runs its mandatory watertight + volume/area self-verify
- THEN the native result SHALL be DISCARDED and the import SHALL fall through to OCCT, so a wrong or leaky mesh is never emitted downstream

#### Scenario: Existing STEP round-trips and non-B-spline pcurve arms are unchanged (zero-regression)

- GIVEN the existing STEP round-trip fixtures (box / cylinder / sphere / native B-spline-wall solids) and the full simulator suite
- WHEN the reader is exercised with the new B-spline-surface pcurve arm and guard in place
- THEN a straight-in-`(u,v)` B-spline-wall edge SHALL emit a pcurve byte-identical to the prior generic-linear reconstruction, the Plane / Cylinder / Cone / Sphere arms and the bare-periodic admission paths SHALL be unchanged, and `run-sim-suite` (221/221) and STEP import (77/77) SHALL stay green
