## Why

Feature **#6 CURVED BLENDS** has landed the CIRCULAR-crease blends: the CONSTANT-radius
convex + concave rolling-ball FILLET and the VARIABLE-radius FILLET (a G1-tangent torus
canal on a cylinder↔coaxial-cap rim), and the SYMMETRIC-distance CHAMFER (a C0
cone-frustum bevel on the same rim). Every one of them lives on a CIRCULAR crease with a
CIRCULAR (torus / cone-frustum) blend surface.

The remaining blend breadth is OFF the circle:

- the circular chamfer is still SYMMETRIC only (`d1 = d2`) — a real chamfer commonly
  wants DIFFERENT setbacks on the two faces;
- the fillet is still CIRCULAR-crease only — a cylinder cut by an OBLIQUE plane, or a
  cylinder meeting another cylinder, has a NON-circular crease (an ellipse, or a general
  space curve) that today falls straight through to OCCT.

This change lands the last blend-breadth batch as THREE honest tracks, each gated by the
engine's mandatory watertight + sane-volume self-verify, each returning NULL → OCCT (with
the measured gap REPORTED) outside its slice.

### T1 — Asymmetric two-distance chamfer (highest confidence)

Extend the symmetric cone-frustum chamfer (`curved_chamfer.h`) to `d1 ≠ d2` on the SAME
convex circular cylinder↔cap rim:

- the cylinder wall is set back AXIALLY by `d1` → the **cylinder seam** CIRCLE at radius
  `Rc`, axial `H − s·d1`;
- the cap is set back RADIALLY by `d2` → the **cap seam** CIRCLE at radius `Rc − d2`,
  axial `H`;
- the band between the two setback circles is an **OBLIQUE CONE FRUSTUM** — a ruled
  truncated cone whose meridian is the straight segment `(Rc, H − s·d1) → (Rc − d2, H)`,
  slant `(Δradius, Δaxial) = (−d2, s·d1)`;
- the bevel is still **C0, NOT G1**, but the two seam angles now DIFFER: the frustum
  outward normal is `radial·d1 + axial·s·d2` (normalized), so
  `cos∠_cyl = d1/√(d1²+d2²)` at the wall seam and `cos∠_cap = d2/√(d1²+d2²)` at the cap
  seam (equal to `1/√2` only when `d1 = d2`, and NEVER `1` — a chamfer is a flat bevel);
- the chamfer REMOVES material: the removed corner is the right triangle legs `d1×d2`
  (area `d1·d2/2`, centroid radial `Rc − d2/3`), exact removed volume
  `V_removed = π·d1·d2·(Rc − d2/3)` (Pappus), reducing to the symmetric
  `π·d²·(Rc − d/3)` at `d1 = d2 = d`.

`cc_chamfer_edges` carries only ONE distance, so T1 lands behind a NEW ADDITIVE facade
entry `cc_chamfer_edges_asym(body, edgeIds, edgeCount, distance1, distance2)`; the
existing `cc_chamfer_edges` (and the symmetric native path behind it) is byte-unchanged.
The symmetric distance is the exact `d1 = d2` case of the same oblique-frustum builder.

### T2 — Non-circular (elliptical) crease fillet (medium confidence, narrow slice)

A CONSTANT-radius rolling-ball fillet on the ELLIPTICAL rim where a cylinder (radius `Rc`,
axis `A`) meets an OBLIQUE plane (normal at angle `θ` to `A`, `0 < θ < 90°`). The plane∩
cylinder crease is an ELLIPSE (semi-minor `Rc`, semi-major `Rc/sinθ`) — exactly the conic
the native SSI S1 `plane_conics` handler returns in closed form. Rolling a ball of radius
`r` on the air side keeps it at distance `r` from BOTH faces, so:

- the ball-centre **spine** is `(offset plane, shifted `r` along the outward normal) ∩
  (offset cylinder radius `Rc − r`)` — ANOTHER plane∩cylinder ellipse (semi-minor
  `Rc − r`), closed form;
- the **cylinder-contact** curve is an ellipse ON the cylinder (radius `Rc`); the
  **plane-contact** curve is an ellipse ON the plane — both the perpendicular feet of the
  spine onto each face, closed form;
- the fillet surface is the GENERAL CANAL — the envelope of the constant-`r` sphere family
  centred on the spine ellipse — realized as swept `r`-CIRCLES in the planes NORMAL to the
  spine tangent, tiled deflection-bounded and trimmed between the two contact ellipses;
- the builder SELF-VERIFIES G1: the canal is TANGENT to the cylinder along the
  cylinder-contact ellipse (surface normal = cylinder radial, `cos = 1`) and TANGENT to
  the plane along the plane-contact ellipse (surface normal = plane normal, `cos = 1`).

Behind the UNCHANGED `cc_fillet_edges`. The engine's `fillet_edges` dispatch gains an
elliptical-crease candidate AFTER the circular candidates. Declines (NULL → OCCT) when the
plane is axis-perpendicular (a circle → the existing torus path) or axis-parallel (2 lines
→ planar), when the crease is not an ellipse, or when `r` reaches the crease's tightest
curvature radius (`ρ_min ≈ Rc·sinθ` at the semi-major vertices — the canal would
self-intersect on the concave side).

### T3 — Cylinder↔cylinder canal fillet (lowest confidence — narrow slice OR honest decline)

A constant-radius fillet on the CURVED↔CURVED crease between two intersecting cylinders.
The crease is a GENERAL space curve — the SSI marching curve (e.g. two equal orthogonal
cylinders give the Steinmetz curve) — and the ball-centre spine is another general SSI
curve (`offset cyl1 (Rc1 − r) ∩ offset cyl2 (Rc2 − r)`). The canal is swept `r`-circles in
the normal planes along the fitted spine, trimmed between the two cylinder-contact curves,
G1 to both cylinders.

This is the hardest track: a general (non-analytic) crease from marching
(`CYBERCAD_HAS_NUMSCI`-gated), general contact curves, watertightness + G1 of a general
swept canal, and envelope self-intersection where the crease curvature is tight. It is
attempted ONLY for the narrowest robust slice — EQUAL-radius PERPENDICULAR cylinders
(symmetric Steinmetz), `r` safely below the crease curvature — and the builder is RETAINED
only if it self-verifies watertight + G1 + correct SHRINK volume on its fixture. If the
slice cannot be built robustly, T3 is an HONEST DECLINE: cyl↔cyl fillet stays
OCCT-fallthrough, documented, the measured OCCT-parity gap REPORTED, with NO always-NULL
dead builder retained (matching the project's honest-decline-without-dead-code discipline
from the recent hard SSI slices).

## What Changes

- **T1 (chamfer, new additive ABI).** Add `cc_chamfer_edges_asym(CCShapeId body,
  const int* edgeIds, int edgeCount, double distance1, double distance2)` to the facade
  and a matching `IEngine::chamfer_edges_asym` (default unsupported; OCCT override →
  `BRepFilletAPI_MakeChamfer::Add(d1, d2, edge, face)`; `NativeEngine` override →
  native). Generalize `src/native/blend/curved_chamfer.h`: promote
  `buildChamferedCylinder(g, d, defl)` to the OBLIQUE frustum
  `buildChamferedCylinderAsym(g, d1, d2, defl)` (symmetric = `d1 = d2`), and add
  `curved_chamfer_edge_asym(solid, edgeIds, edgeCount, d1, d2, deflection)`. The seams
  become `(Rc, H − s·d1)` and `(Rc − d2, H)`; the bevel normal `radial·d1 + axial·s·d2`;
  the self-verify asserts `cos∠_cyl = d1/√(d1²+d2²)`, `cos∠_cap = d2/√(d1²+d2²)`, both
  `≠ 1` (C0), and the removed volume `π·d1·d2·(Rc − d2/3)`. Guards: `Rc − d2 > 0`, wall
  covers `H − s·d1`, `d1, d2 > 0`, `edgeCount == 1`, convex cylinder↔coaxial-cap rim.
  `NativeEngine::chamfer_edges_asym` runs it under
  `blendResultVerified(..., wantGrow=false)` (SHRINK), else honest error → OCCT.

- **T2 (fillet, unchanged ABI).** Add a new OCCT-free header
  `src/native/blend/elliptical_fillet.h` (`elliptical_fillet_edge(solid, edgeIds,
  edgeCount, r, deflection)`) that recognizes a cylinder↔OBLIQUE-plane ELLIPTICAL rim
  (via the SSI `plane_conics` ellipse), builds the spine + both contact ellipses in closed
  form, sweeps the general `r`-circle canal in spine-normal planes, and self-verifies G1
  at both contact ellipses + watertight + SHRINK. Wire it into `NativeEngine::fillet_edges`
  AFTER the circular convex/concave candidates, gated by
  `blendResultVerified(..., wantGrow=false)`. Declines (NULL) outside the oblique-plane
  ellipse slice or when `r ≥ ρ_min ≈ Rc·sinθ`.

- **T3 (fillet, unchanged ABI, honest gate).** Add `src/native/blend/cylcyl_fillet.h`
  (`cylcyl_fillet_edge(...)`, `CYBERCAD_HAS_NUMSCI`-gated) that computes the cyl∩cyl crease
  + the offset-cyl∩offset-cyl spine via SSI marching, sweeps the general `r`-circle canal,
  and self-verifies G1 at both cylinder-contact curves + watertight + SHRINK. Wire it into
  `NativeEngine::fillet_edges` AFTER the elliptical candidate under the SAME SHRINK guard.
  If the narrow slice (equal-radius orthogonal cylinders) does NOT build watertight + G1 +
  correct volume on its fixture, the header is NOT retained and cyl↔cyl fillet stays a
  documented OCCT-fallthrough — an honest decline with the measured gap REPORTED and NO
  dead code.

- **Engine wiring.** `NativeEngine::fillet_edges` extends its planar → curved-convex →
  curved-concave dispatch with elliptical (T2) and — if landed — cyl↔cyl (T3) candidates,
  each accepted ONLY through the existing SHRINK self-verify. `NativeEngine::chamfer_edges`
  is UNCHANGED (symmetric path); the new `chamfer_edges_asym` override handles T1. A NULL
  builder result OR a failed self-verify DISCARDS the candidate; a native body that cannot
  be forwarded returns an honest error so the OCCT engine serves the call.

- **Native code stays OCCT-free.** T1 uses closed-form circles + `math::Ax3` (no NUMSCI).
  T2 uses the SSI S1 `plane_conics` ellipse + closed-form spine/contact ellipses + swept
  circles (no NUMSCI). T3 uses the SSI marching substrate (`CYBERCAD_HAS_NUMSCI`-gated).
  OCCT is referenced ONLY in the OCCT fallback engine.

**ABI:** ADDITIVE only. `cc_fillet_edges` / `cc_chamfer_edges` keep their exact
signatures; T2/T3 are new NATIVE paths behind the unchanged `cc_fillet_edges`; T1 adds the
new `cc_chamfer_edges_asym` entry (existing entries byte-unchanged). No POD struct changes.

## Capabilities

### New Capabilities
<!-- none — this EXTENDS the living native-blends capability (asymmetric chamfer +
     non-circular / cyl-cyl fillet) and adds ONE additive facade entry to kernel-facade;
     no new capability is introduced. -->

### Modified Capabilities
- `native-blends`: add a native ASYMMETRIC two-distance chamfer (`cc_chamfer_edges_asym`,
  an OBLIQUE cone-frustum bevel between the setback circles `(Rc, H − s·d1)` and
  `(Rc − d2, H)`, C0 at the two DIFFERENT seam angles `cos = d1/√(d1²+d2²)` /
  `d2/√(d1²+d2²)`, removed volume `π·d1·d2·(Rc − d2/3)`, SHRINK-verified); a native
  NON-CIRCULAR (elliptical) crease fillet (`cc_fillet_edges` on a cylinder↔oblique-plane
  ELLIPTICAL rim — a general `r`-circle canal on an ellipse spine, G1 at the two contact
  ellipses, SHRINK-verified); and a native cyl↔cyl-canal fillet (`cc_fillet_edges` on a
  cylinder↔cylinder crease — a general `r`-circle canal on the SSI marching spine, G1 at
  the two cylinder-contact curves) OR its documented HONEST DECLINE. Everything outside
  the named slices (a non-oblique / non-elliptical / freeform crease, `r ≥ ρ_min`,
  `Rc ≤ d2`, a concave rim, a multi-edge selection, a non-native body) returns NULL →
  OCCT `BRepFilletAPI`, gap REPORTED. Delivered behind the unchanged
  `cc_fillet_edges` / `cc_chamfer_edges` plus the new additive `cc_chamfer_edges_asym`.
- `kernel-facade`: add the ADDITIVE plain-C entry `cc_chamfer_edges_asym(body, edgeIds,
  edgeCount, distance1, distance2)` — a two-distance chamfer — alongside the unchanged
  `cc_chamfer_edges`; same registry / exception-to-status guard model, no existing
  signature or POD layout changed.

## Impact

- **ABI**: additive. `cc_chamfer_edges_asym` is a NEW plain-C entry (two distances);
  `cc_fillet_edges` / `cc_chamfer_edges` keep their signatures and POD structs. T2/T3 are
  internal native paths behind `cc_fillet_edges`; every path has OCCT fallback.
- **Build**: T1 generalizes `src/native/blend/curved_chamfer.h` (oblique frustum) — no
  new dependency, closed-form, NUMSCI-OFF. T2 adds `src/native/blend/elliptical_fillet.h`
  consuming SSI S1 `plane_conics` (`intersectPlaneCylinder` → Ellipse) + `native-math`
  (swept circles) — NUMSCI-OFF. T3 adds `src/native/blend/cylcyl_fillet.h` consuming the
  SSI marching substrate (`trace_intersection`, `CYBERCAD_HAS_NUMSCI`-gated; an empty TU
  with NUMSCI off) OR is not retained (honest decline). All aggregated by
  `native_blend.h`. `NativeEngine::fillet_edges` extends its dispatch; a new
  `chamfer_edges_asym` override is added; both under the existing
  `blendResultVerified(wantGrow=false)` SHRINK guard. Consumes `native-boolean`
  (`assembleSolid` weld) + `native-tessellate` (watertight mesher — NOT modified). Builds
  with NUMSCI OFF (T1/T2) and NUMSCI ON (T1/T2 regression + T3 gate).
- **Verification**: two gates. **host** (OCCT-free CTest): T1 — the oblique frustum + both
  setback circles on their surfaces, watertight, volume `|body| − π·d1·d2·(Rc − d2/3)`,
  the two DIFFERENT bevel angles (`cos = d1/√(d1²+d2²)` / `d2/√(d1²+d2²)`, both `≠ 1`), and
  symmetric `d1 = d2` reproducing the existing removed volume. T2 — the spine + both
  contact ellipses on their surfaces, watertight, SHRINK, G1 at both contact ellipses
  (`cos = 1`), and the `r ≥ ρ_min` / non-oblique decline → NULL. T3 — the equal-radius
  orthogonal-cyl narrow slice watertight + G1 + SHRINK on its fixture (NUMSCI ON), OR the
  documented decline → NULL with the gap REPORTED. **sim** native-vs-OCCT parity
  (`run-sim-native-curved-chamfer.sh` / `run-sim-native-curved-fillet.sh` + `.mm`
  extended): T1 vs `BRepFilletAPI_MakeChamfer::Add(d1, d2, edge, face)`; T2 vs
  `BRepFilletAPI_MakeFillet` on the oblique-plane rim; T3 vs `BRepFilletAPI_MakeFillet` on
  the cyl↔cyl rim (or the decline-parity fall-through). Because T1's oblique frustum is
  EXACT, its parity is TIGHT (angular deflection); T2/T3 are curved-parity bounds.
- **Roadmap**: closes `ROADMAP.md` #6 blend breadth (asymmetric chamfer + non-circular /
  cyl-cyl fillet) and rides the `SSI-ROADMAP.md` on-ramp (S1 plane∩cylinder ellipse for
  T2, S3 marching cyl∩cyl for T3).
- **Regression**: additive only. The symmetric circular chamfer
  (`run-sim-native-curved-chamfer.sh` 9/9), the constant + variable circular fillet
  (`run-sim-native-curved-fillet.sh` 23/23), the planar blends
  (`run-sim-native-blend.sh` 16/16), native booleans, SSI S1–S5, healing, import,
  marching, and phase-3 suites are UNTOUCHED. `cc_chamfer_edges` (symmetric) is
  byte-identical; the new paths fire ONLY for the named asymmetric / elliptical / cyl-cyl
  inputs, tried AFTER the existing candidates decline, gated by the SHRINK self-verify.
- **Risk / honesty**: T1 is an EXACT oblique frustum (tight parity) — high confidence. T2
  is a narrow but exact-closed-form ellipse slice — medium confidence, declines at the
  curvature bound. T3 is the hardest — a narrow slice OR an HONEST DECLINE with NO dead
  code and the measured gap REPORTED. NEVER a weakened tolerance, NEVER a faked patch,
  NEVER G1 asserted for a chamfer or C0 accepted for a fillet; everything outside a slice
  returns NULL → OCCT with the gap REPORTED.
