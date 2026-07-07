# Design — add-native-fillet-chamfer-breadth (#6 curved blends, off-the-circle breadth)

## Context

The native blend family covers CIRCULAR-crease blends: the convex + concave + variable
rolling-ball FILLET (`curved_fillet.h`, a G1-tangent torus canal on a cylinder↔coaxial-cap
rim) and the SYMMETRIC CHAMFER (`curved_chamfer.h`, a C0 cone-frustum bevel on the same
rim). All of them share one classifier (`detail::facesOnRim` + `detail::rimGeom`), one
frame (`math::Ax3`), one facet budget (`sagittaSteps`), one sampler (`ringPoint`), and one
weld (`nb::assembleSolid`), and all are accepted only through the engine's mandatory
watertight + sane-volume self-verify `blendResultVerified(result, body, wantGrow)`.

The remaining breadth is OFF the circle. This change adds three tracks, each clean-room
(OCCT is the ORACLE only), each honest per-track:

- **T1** extends the symmetric cone-frustum chamfer to `d1 ≠ d2` (an OBLIQUE frustum) —
  the safest generalization, closed form, EXACT.
- **T2** generalizes the fillet canal from a CIRCULAR crease (torus) to a NON-circular
  ELLIPTICAL crease (cylinder ∩ oblique plane) — a general canal whose spine and both
  contact curves are ellipses the SSI S1 `plane_conics` handler already computes.
- **T3** attempts the CYL↔CYL crease (a general SSI marching curve) — a general canal on a
  non-analytic spine; the hardest, landed as a narrow slice OR an honest decline.

## Goals / Non-Goals

**Goals**
- T1: a native, OCCT-free ASYMMETRIC two-distance chamfer on the convex circular
  cylinder↔cap rim — an OBLIQUE cone frustum between `(Rc, H − s·d1)` and `(Rc − d2, H)`,
  C0 at the two DIFFERENT bevel angles, welded watertight, SHRINK-verified, behind a NEW
  ADDITIVE `cc_chamfer_edges_asym`. Symmetric `d1 = d2` is the exact special case.
- T2: a native, OCCT-free CONSTANT-radius fillet on a cylinder↔OBLIQUE-plane ELLIPTICAL
  rim — a general `r`-circle canal on the ellipse spine, G1 at the two contact ellipses,
  SHRINK-verified, behind the unchanged `cc_fillet_edges`.
- T3: a native, OCCT-free constant-radius fillet on the narrowest robust cyl↔cyl slice
  (equal-radius orthogonal cylinders) — a general `r`-circle canal on the SSI marching
  spine, G1 at both cylinder-contact curves, SHRINK-verified — OR a documented HONEST
  DECLINE (no dead code, gap REPORTED).
- All behind the mandatory self-verify; OCCT fallback for the rest.

**Non-Goals (return NULL → OCCT, never faked)**
- T1: a CONCAVE circular rim, a cylinder↔cylinder chamfer, a non-circular / tilted /
  non-coaxial / freeform crease, `Rc ≤ d2`, a wall shorter than `d1`, a multi-edge
  selection.
- T2: a non-oblique plane (axis-perpendicular → circle = the existing torus path;
  axis-parallel → 2 lines = planar), a non-elliptical crease, a CONCAVE elliptical rim,
  `r ≥ ρ_min ≈ Rc·sinθ` (canal self-intersection), a freeform adjacent face.
- T3: unequal-radius or non-orthogonal cylinders beyond the robust slice, a crease with
  branch points / self-intersections, `r` above the crease curvature, a marching failure
  (no NUMSCI, or the trace does not close) — all NULL → OCCT.
- Any track whose measured OCCT-parity gap exceeds the bound is declared out of slice
  (NULL → OCCT) and the gap REPORTED — never passed with a loosened tolerance.

## T1 — Asymmetric two-distance chamfer (clean-room, EXACT)

Take the same convex circular rim the symmetric chamfer handles (cylinder radius `Rc`,
axis `A` frame Z, coaxial planar cap at axial `H`, rim the circle radius `Rc` at `H`,
convex dihedral, `s = ±1` the axial sign from the far end toward the cap). An ASYMMETRIC
chamfer sets the two faces back by DIFFERENT distances:

- the cylinder wall set back AXIALLY by `d1` → the **cylinder seam** circle: radius `Rc`,
  axial `H − s·d1` (ON the cylinder wall by construction, `radius = Rc` exact);
- the cap set back RADIALLY by `d2` → the **cap seam** circle: radius `Rc − d2`, axial `H`
  (ON the cap plane by construction, `axial = H` exact).

The bevel bridges the two setback circles with a straight line in every meridian
half-plane — from `(Rc, H − s·d1)` to `(Rc − d2, H)` in `(radius, axial)`. Revolving it
about `A` sweeps an **OBLIQUE CONE FRUSTUM**:

```
radius(τ) = Rc − d2·τ ,   axial(τ) = (H − s·d1) + s·d1·τ ,   τ ∈ [0, 1]
point(u, τ) = ringPoint(A, radius(τ), u, axial(τ)) ,          u ∈ [0, 2π)
```

- `τ = 0` → radius `Rc`, axial `H − s·d1`  → the CYLINDER seam.
- `τ = 1` → radius `Rc − d2`, axial `H`     → the CAP seam.

Meridian slant `(Δradius, Δaxial) = (−d2, s·d1)`. The frustum outward normal is
PERPENDICULAR to the slant, pointing away from the axis and toward the removed corner:

```
n_frustum(u) = radial(u)·d1 + axial·(s·d2) ,  normalized ,  radial(u) = x̂·cos u + ŷ·sin u
```

(check: `n · slant = d1·(−d2) + (s·d2)·(s·d1) = −d1·d2 + d1·d2 = 0`, `s² = 1`; radial
coefficient `d1 > 0` is outward). The two seam angles now DIFFER:

- at the **cylinder seam** (`τ = 0`, cylinder normal RADIAL): `cos∠_cyl = d1/√(d1²+d2²)`;
- at the **cap seam** (`τ = 1`, cap normal `s·ẑ`): `cos∠_cap = d2/√(d1²+d2²)`.

Both equal `1/√2` only when `d1 = d2`, and NEITHER is `1` (a chamfer is a straight bevel —
C0). This is the load-bearing self-verify: the builder asserts the CORRECT per-seam bevel
angle and explicitly asserts NOT tangent (`cos ≠ 1`). Asserting G1 here would be WRONG.

### Removed volume (exact, Pappus)

The removed corner is the RIGHT triangle with vertices

```
P0 = (Rc,      H)         sharp corner (right angle)
P1 = (Rc,      H − s·d1)  cylinder setback point
P2 = (Rc − d2, H)         cap setback point
```

legs `d1` (axial) × `d2` (radial), area `A_tri = d1·d2/2`, centroid radial
`R̄ = (Rc + Rc + (Rc − d2))/3 = Rc − d2/3`. By Pappus:

```
V_removed = 2π · R̄ · A_tri = π · d1 · d2 · (Rc − d2/3)
```

reducing to the symmetric `π·d²·(Rc − d/3)` at `d1 = d2 = d`. The chamfered solid volume
is `|body| − π·d1·d2·(Rc − d2/3)`, gated by the SHRINK self-verify.

### Guards

`edgeCount == 1`, `d1 > kBlendEps`, `d2 > kBlendEps`, the cap circle real
(`Rc − d2 > eps`, so the frustum does not cross the axis), and the far end beyond the
cylinder seam (`s·((H − s·d1) − hFar) > 0`, the wall covers the axial setback). Any
failure ⇒ NULL → OCCT.

### Facade + engine plumbing (the additive ABI)

`cc_chamfer_edges` carries ONE distance and is BYTE-UNCHANGED. T1 adds
`cc_chamfer_edges_asym(body, edgeIds, edgeCount, distance1, distance2)` — the additive
sibling (precedent: `cc_fillet_edges_variable`, `cc_fillet_edges_g2`). It maps to a new
`IEngine::chamfer_edges_asym` (default `engine_unsupported`; OCCT override →
`BRepFilletAPI_MakeChamfer::Add(d1, d2, edge, face)` where `face` selects which face
carries `d1`; `NativeEngine` override → `nblend::curved_chamfer_edge_asym`). The native
override:

```cpp
ShapeResult NativeEngine::chamfer_edges_asym(EngineShape body, const int* e, int ec,
                                             double d1, double d2) {
  if (!isNative(body)) return fallback().chamfer_edges_asym(body, e, ec, d1, d2);
  const auto* h = static_cast<const NativeShape*>(body.get());
  ntopo::Shape result = nblend::curved_chamfer_edge_asym(h->shape, e, ec, d1, d2);
  if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/false))
    return track(wrapNative(std::move(result)));
  return make_error(
    "native chamfer_edges_asym: no verified watertight result for this native body "
    "(non-circular / concave / cyl-cyl / tilted / freeform / Rc<=d2 → OCCT-only)");
}
```

`curved_chamfer.h` grows `buildChamferedCylinderAsym(g, d1, d2, defl)` (the existing
`buildChamferedCylinder` becomes `buildChamferedCylinderAsym(g, d, d, defl)`) and
`curved_chamfer_edge_asym(...)`. The symmetric path is unchanged (either kept as a thin
`d1 = d2` forwarder or left as-is).

## T2 — Non-circular (elliptical) crease fillet (clean-room, closed form)

A cylinder (radius `Rc`, axis `A`) cut by an OBLIQUE plane (unit normal `n`, angle
`θ = acos|n·â|`, `0 < θ < 90°`) meets it in an ELLIPSE `E0` lying on BOTH surfaces:
semi-minor `b = Rc` (in the plane, along the cylinder's cross-cut direction) and
semi-major `a = Rc/sinθ` (along the oblique-stretch direction). The native SSI S1
`intersectPlaneCylinder` returns exactly this `Ellipse` (centre, two axes, `a`, `b`).

Rolling a ball of radius `r` on the air (convex) side keeps its centre at distance `r`
from the plane AND at radial distance `Rc − r` from the cylinder axis. So:

- **spine** `S = (plane shifted `r` along the outward normal `n_out`) ∩ (cylinder radius
  `Rc − r`)` — ANOTHER plane∩cylinder ELLIPSE (same axis directions, semi-minor `Rc − r`,
  semi-major `(Rc − r)/sinθ`), from the SAME `plane_conics` handler on the shifted plane +
  shrunk cylinder. Closed form.
- **cylinder-contact** ellipse `C_cyl` = the foot of the perpendicular from each spine
  point to the cylinder = the point at radius `Rc` sharing the spine point's azimuth and
  axial coordinate → an ellipse ON the cylinder (radius `Rc`).
- **plane-contact** ellipse `C_pl` = the foot of the perpendicular from each spine point to
  the plane = spine point `− r·n_out` → an ellipse ON the plane.

The fillet surface is the CANAL — the envelope of the constant-`r` sphere family centred
on `S`. Its characteristic circle at each spine station `S(t)` is the `r`-circle in the
plane NORMAL to the spine tangent `T(t) = S'(t)`, passing through both contact points. It
is built as swept `r`-CIRCLES:

```
for each spine station t (N angular stations from sagittaSteps on the ellipse arc length):
  T = normalize(S'(t))                       // spine tangent
  build an orthonormal (e1, e2) ⟂ T
  the char. circle centre = S(t), radius r
  the two trim feet: φ_cyl (toward the cylinder normal), φ_pl (toward −n_out)
  emit the r-arc from φ_cyl to φ_pl (M meridian steps from sagittaSteps(r, Δφ, defl))
```

trimmed between `C_cyl` and `C_pl`, tiled into deflection-bounded planar triangles sharing
`N` angular stations with the rebuilt cylinder wall and the trimmed oblique cap, welded via
`assembleSolid`.

### G1 self-verify (the load-bearing correctness statement — a fillet IS tangent)

Unlike the chamfer, a fillet is G1. The builder SELF-VERIFIES, at every station: the canal
surface normal at the cylinder-contact foot equals the cylinder RADIAL normal (`cos = 1`),
and the canal surface normal at the plane-contact foot equals the plane normal `n`
(`cos = 1`). The two contact ellipses lie ON their faces by construction (radius `Rc`;
`n·(p) = plane offset`). If ANY station fails tangency or on-surface, return NULL → OCCT.

### Decline conditions (curvature bound + non-ellipse)

The ellipse crease has tightest curvature radius at the semi-major vertices
`ρ_min = b²/a = Rc²/(Rc/sinθ) = Rc·sinθ`. If `r ≥ ρ_min` the canal self-intersects on the
concave side (the offset ellipse `Rc − r` degenerates / the swept circles overlap) → NULL.
If the plane is axis-perpendicular (`θ → 90°`, the crease is a CIRCLE → the existing torus
path owns it) or axis-parallel (`θ → 0°`, the crease is 2 lines → planar), if the crease
edge is not an `Ellipse`, or if the dihedral is CONCAVE, return NULL → OCCT. (θ range is
bounded away from the two limits by a scale tolerance.)

### Engine wiring (T2)

`NativeEngine::fillet_edges` extends its dispatch (planar → curved-convex → curved-concave)
with an elliptical candidate AFTER the circular ones (a circular rim is classified by the
existing convex/concave builders first, so those paths stay byte-identical):

```cpp
// … after nblend::fillet_edges (planar), curved_fillet_edge (convex+concave circular) …
result = nblend::elliptical_fillet_edge(h->shape, e, ec, r);      // T2
if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/false))
  return track(wrapNative(std::move(result)));
```

## T3 — Cylinder↔cylinder canal fillet (hardest — narrow slice OR honest decline)

Two intersecting cylinders (axes `A1`, `A2`, radii `Rc1`, `Rc2`) meet in a GENERAL crease
curve. For the robust slice — EQUAL radii `Rc1 = Rc2 = Rc`, PERPENDICULAR axes — it is the
symmetric Steinmetz curve (two planar ellipses meeting at the saddle points). The crease
is obtained from the SSI marching tracer `trace_intersection(cyl1, cyl2)`
(`CYBERCAD_HAS_NUMSCI`-gated), fitted to a B-spline `WLine`.

Rolling a ball of radius `r`: the spine `S = offset-cyl1 (radius Rc − r) ∩ offset-cyl2
(radius Rc − r)` is ANOTHER general SSI curve, from a SECOND marching trace. The canal is
swept `r`-circles in the planes NORMAL to the spine tangent (same construction as T2 but on
a general, non-analytic spine), trimmed between the two cylinder-contact curves (feet on
`cyl1` and `cyl2`), G1 to both cylinders.

### The honest gate (no dead code)

T3 is genuinely hard: a non-analytic crease + spine from marching, general contact curves,
watertightness + G1 of a general swept canal, and envelope self-intersection near the
saddle points where the crease curvature is tight. The gate:

1. Attempt ONLY the narrow slice (equal-radius perpendicular cylinders, `r` safely below
   the crease's min curvature radius, the two marching traces both `Closed`).
2. Build the swept canal, self-verify watertight + G1 at BOTH cylinder-contact curves
   (`cos = 1`) + SHRINK (`0 < Vr < Vo`, removed material) + parity vs OCCT
   `BRepFilletAPI_MakeFillet` within the curved bound on the fixture.
3. If it passes → RETAIN `cylcyl_fillet.h` + the engine candidate (a real native slice).
4. If it does NOT pass robustly on the fixture → DO NOT retain an always-NULL builder.
   cyl↔cyl fillet stays a documented OCCT-fallthrough (T3 = honest decline), the measured
   gap REPORTED. This mirrors the recent hard SSI slices that correctly declined WITHOUT
   dead code.

Wiring (only if landed):

```cpp
#ifdef CYBERCAD_HAS_NUMSCI
result = nblend::cylcyl_fillet_edge(h->shape, e, ec, r);          // T3 (narrow slice)
if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/false))
  return track(wrapNative(std::move(result)));
#endif
```

The saddle points (where the two cylinders' crease self-touches) are a hard sub-case even
within the narrow slice; if they cannot be welded watertight the whole track declines
honestly rather than emit a near-degenerate fan.

## Module shape

```
src/native/blend/
  curved_chamfer.h       // T1. buildChamferedCylinder → buildChamferedCylinderAsym(g,
                         //   d1, d2, defl) (symmetric = d1=d2); + curved_chamfer_edge_asym.
                         //   Oblique frustum, per-seam bevel angle, Pappus d1·d2 volume.
  elliptical_fillet.h    // T2. NEW. elliptical_fillet_edge(...). SSI plane_conics ellipse
                         //   crease + spine + contact ellipses (closed form) + swept
                         //   r-circle canal + G1 self-verify. NUMSCI-OFF.
  cylcyl_fillet.h        // T3. NEW *iff landed*. cylcyl_fillet_edge(...), NUMSCI-gated.
                         //   SSI marching crease + spine + swept r-circle canal + G1.
                         //   NOT retained if the narrow slice does not self-verify.
  curved_fillet.h        // REUSED (classifier + helpers); no change.
  native_blend.h         // add #include for elliptical_fillet.h (+ cylcyl_fillet.h iff landed).
```

Cognitive-complexity: T1 is a one-parameter generalization of an existing flat loop
(systems band, ≤ the current builder). T2's canal is a single swept-circle loop over a
closed-form ellipse spine, isolated behind the classifier guard-clauses (systems band,
documented). T3, if landed, is the marching-fed swept canal (systems band, NUMSCI-gated,
documented) — else absent.

## Engine wiring summary (the load-bearing change)

- `chamfer_edges` (symmetric) — UNCHANGED.
- `chamfer_edges_asym` (NEW override) — native T1 → SHRINK self-verify → honest error →
  OCCT.
- `fillet_edges` — extends planar → curved-convex → curved-concave with elliptical (T2)
  and, iff landed, cyl↔cyl (T3), each under `blendResultVerified(wantGrow=false)`; the
  existing circular candidates are byte-identical (tried first).
- A NULL builder result OR a failed self-verify DISCARDS the candidate; a native body that
  cannot be forwarded returns an honest error so the OCCT engine serves the call. No new
  guard type, no weakened tolerance.

## Verification model (two gates)

- **Host (no OCCT):**
  - T1: build a native capped cylinder; `curved_chamfer_edge_asym(d1, d2)`; assert the two
    setback circles on their surfaces, watertight, volume `|body| − π·d1·d2·(Rc − d2/3)`,
    the two DIFFERENT bevel angles (`cos = d1/√(d1²+d2²)` at the wall, `d2/√(d1²+d2²)` at
    the cap, both `≠ 1`, C0), and `d1 = d2` reproducing the symmetric removed volume.
    Default + NUMSCI-ON.
  - T2: build a native cylinder cut by an oblique plane; `cc_fillet_edges(r)`; assert the
    spine + both contact ellipses on their surfaces, watertight, SHRINK, G1 at both contact
    ellipses (`cos = 1`), and the `r ≥ Rc·sinθ` / axis-perpendicular / axis-parallel /
    concave decline → NULL. Default + NUMSCI-ON.
  - T3: NUMSCI-ON — the equal-radius orthogonal-cyl narrow slice watertight + G1 + SHRINK
    on its fixture, OR the documented decline → NULL with the gap REPORTED. NUMSCI-OFF: the
    header is an empty TU (or absent) and the call declines → OCCT.
- **Sim native-vs-OCCT parity** (extend `run-sim-native-curved-chamfer.sh` /
  `run-sim-native-curved-fillet.sh` + `.mm`):
  - T1: `cc_chamfer_edges_asym(d1, d2)` native vs OCCT `BRepFilletAPI_MakeChamfer` +
    `Add(d1, d2, edge, face)` on a cylinder top rim, ≥ 2 `(d1, d2)` fixtures with
    `d1 ≠ d2`. TIGHT bound (angular deflection) — the oblique frustum is EXACT.
  - T2: `cc_fillet_edges(r)` native vs OCCT `BRepFilletAPI_MakeFillet` on a
    cylinder↔oblique-plane elliptical rim, ≥ 1 obliquity fixture. Curved-parity bound;
    a fixture beyond tol is declared out of slice (NULL → OCCT), gap REPORTED.
  - T3: `cc_fillet_edges(r)` native vs OCCT `BRepFilletAPI_MakeFillet` on the
    equal-radius orthogonal cyl↔cyl rim (iff landed); else the decline-parity fall-through
    (identical under both engines), gap REPORTED.
  - No regression: `run-sim-native-curved-chamfer.sh` 9/9,
    `run-sim-native-curved-fillet.sh` 23/23, `run-sim-native-blend.sh` 16/16,
    `run-sim-suite.sh` green.

## Decisions

- **T1 is the safest generalization — one parameter, EXACT.** Promote the symmetric
  frustum to `d1 ≠ d2` (an oblique frustum). The seams, the bevel normal, and the removed
  volume all generalize in closed form; symmetric is the exact `d1 = d2` case. Behind a
  NEW ADDITIVE `cc_chamfer_edges_asym` (ABI never breaks) because the existing entry
  carries only one distance.
- **T2 reduces the ellipse fillet to closed-form conics.** The crease, the spine, and both
  contact curves are all plane∩cylinder ELLIPSES the SSI S1 handler already computes — no
  solver, no NUMSCI. The only genuinely new geometry is the swept `r`-circle canal in
  spine-normal planes. Declines cleanly at the curvature bound `r ≥ Rc·sinθ`.
- **T3 is a narrow slice OR an honest decline — never dead code.** The cyl↔cyl crease is a
  non-analytic marching curve; the track is landed only if the equal-radius orthogonal
  slice self-verifies watertight + G1 + SHRINK on its fixture. Otherwise cyl↔cyl fillet
  stays OCCT-fallthrough, documented, gap REPORTED, with no always-NULL builder retained.
- **G1 for fillets, C0 for chamfers — asserted, never confused.** T2/T3 self-verify
  tangency (`cos = 1`) at both contact curves; T1 self-verifies the two chamfer angles
  (`cos = d1/√(d1²+d2²)` / `d2/√(d1²+d2²)`, both `≠ 1`). Asserting the wrong continuity is
  a hard failure → NULL.
- **`wantGrow=false` (SHRINK) — all three remove material.** Same guard the symmetric
  chamfer and the convex fillet use; no new guard type.

## Risks / Trade-offs

- **T2 canal self-intersection near the curvature bound.** As `r → Rc·sinθ` the swept
  circles overlap on the concave side; the builder guards `r < ρ_min` with a scale margin
  and returns NULL → OCCT below it. Bounded and honest.
- **T3 marching robustness.** The crease + spine come from the SSI marching tracer, which
  needs NUMSCI and a transversal, closing trace; the saddle points of the Steinmetz curve
  are a hard weld sub-case. If the trace does not close or the saddle cannot be welded
  watertight, the whole track declines honestly rather than emit a fragile canal.
- **Slice narrowness vs honesty.** T2 ships only the oblique-plane ellipse; T3 only the
  equal-radius orthogonal cyl↔cyl (or declines). Accepted — the alternative is faking a
  non-circular blend, which the project forbids. The measured OCCT-fallback gap is REPORTED,
  never masked with a weakened tolerance.
- **Wrong-blend-type confusion guard.** T1's oblique frustum and a torus share no
  continuity signature (C0 vs G1), and the host tests check the removed volume against the
  frustum's `π·d1·d2·(Rc − d2/3)` (distinct from any torus); T2/T3 assert G1 (`cos = 1`) at
  both contact curves and check the SHRINK volume, so a chamfer can never be accepted where
  a fillet is expected or vice-versa.
- **Additive ABI surface.** `cc_chamfer_edges_asym` is the only new entry; existing entries
  and POD layouts are byte-unchanged, so no consumer breaks.
