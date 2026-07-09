# Proposal — moat-canal-cyl-cyl-fillet (MOAT M3, cyl↔cyl canal fillet)

## Why

The one remaining app-relevant M3 curved-blend residual on `cc_fillet_edges` is the
**cyl↔cyl canal fillet**: a constant-radius rolling-ball fillet along the crossing crease
where two EQUAL-radius, ORTHOGONAL-axis cylinders meet (the Steinmetz bicylinder). The landed
curved-fillet arms all serve a SINGLE circular rim (cylinder / cone / sphere lateral face
meeting a coaxial planar cap). The cyl↔cyl crease is different: the two lateral faces cross
along TWO ellipse-like crease arcs that meet at TWO poles, so a naive single-swept-r canal
cannot close the corner-blend watertight — this was HONESTLY DECLINED to OCCT
(`native_engine.cpp` T3 note: Rc=3, L=20, r=0.5 COMMON → 143.179260, Δ=−0.820740 vs
`MakeFillet`).

The decline reasoning assumed a sharp trihedral crossing. The GEOMETRY is more forgiving:

- The rolling-ball centre along ONE crease (plane z=x for cyl-A axis Z, cyl-B axis X) is the
  set of points at distance Rc−r from BOTH cylinder axes. Solving `√(Cx²+Cy²)=Rc−r` and
  `√(Cy²+Cz²)=Rc−r` gives `Cx=Cz` — so the centre locus is an EXACT CIRCLE of radius
  **R0 = Rc−r** lying in the crease plane. The fillet surface is therefore an exact **TORUS
  segment** (major R0, minor r), G1-tangent to each cylinder wall at its two seam curves.
- The two crease planes (z=x, z=−x) give TWO such torus centre-circles; they INTERSECT at the
  two poles `(0, ±R0, 0)`. As the crease approaches a pole the local dihedral opens to 180°
  (the two walls become tangent), so each fillet strip's cross-section **tapers to ZERO width
  at the shared pole point** `(0, ±Rc, 0)`. The "crossing crease at the poles" is thus a
  DEGENERATE PINCH, not a finite trihedral corner needing a spherical patch — each torus
  strip and each cylinder wall patch simply share the two canonical pole vertices, and the
  shell welds watertight PURELY IN THE ASSEMBLY LAYER (like `fillet_corner.h`).

Removing OCCT from this rim closes the last M3 curved-blend fillet gap the app hits.

## What changes

- **`src/native/blend/canal_fillet.h` (new, header-only, OCCT-free):** recognises the body as
  a Steinmetz bicylinder from its PLANAR-FACET soup (the native SSI boolean bakes the lune
  walls into planar triangles — no analytic Cylinder faces survive — so the two cylinder axes
  are recovered GEOMETRICALLY: each wall facet's normal is ⟂ its axis, so the axes are the two
  orthogonal directions ⟂ the largest facet-normal families; Rc = the common wall-to-axis
  distance; the crossing point = the body centroid). It then rebuilds the ENTIRE filleted
  bicylinder COMMON from scratch as a planar-facet soup sharing canonical vertices along every
  seam and at the two poles: the four lune wall patches (cyl A matched by azimuth, cyl B by its
  crease-arc pairing) trimmed to the seams, the two canal strips (each tapering to zero at the
  poles), no caps (the walls fully bound the lens). A flood-fill orientation pass makes the
  soup coherently outward, and it welds watertight through the existing `nb::assembleSolid` —
  **no tessellator change**. The op rounds the WHOLE crossing crease (all four arcs) — the only
  watertight resolution for a topology whose arcs meet at the poles. A MANDATORY internal
  self-verify (consistently oriented + a removed-volume bound) rejects any large-radius pole
  fold → NULL → OCCT. Public entry `blend::canal_fillet_edge(solid, edgeIds, edgeCount, r,
  deflection)`.
- **`NativeEngine::fillet_edges` (additive candidate #6):** after the cylinder-convex,
  cylinder-concave, stepped-shaft, cone and sphere candidates, try `nblend::canal_fillet_edge`,
  gated by the SAME `blendResultVerified` SHRINK self-verify (0 < Vr < Vo). A native cyl∪cyl /
  cyl∩cyl Steinmetz body is produced only by the native SSI boolean, so an OCCT body never
  reaches this arm; the single-rim cases are handled by the earlier candidates. No dead code.
  The stale T3 decline comment is replaced by the landed-arm description.

## Honest scope / declines (→ OCCT)

- The two cylinders must be EQUAL radius, axes ORTHOGONAL and CROSSING (a canonical Steinmetz
  bicylinder — the case whose centre-locus is an exact circle). Unequal radii, non-orthogonal
  or non-crossing axes, or a non-Steinmetz crease → NULL → OCCT.
- Ring-torus guard: R0 = Rc − r ≥ r (Rc ≥ 2r), else the tube would reach the axis → NULL.
- The bicylinder must be a pure Steinmetz COMMON the recognizer can rebuild wholesale (exactly
  two equal orthogonal cylinder walls + their disc caps); any extra face / a truncated core /
  caps clipping the fillet band → NULL → OCCT.
- Multiple picked edges / zero-or-negative radius → NULL.
- MODELING-CONVENTION GAP (reported, not gated): the idealized perpendicular-cross-section
  canal removes ~1.2% less than OCCT's `MakeFillet` near the variable dihedral (0.811 vs
  0.821 for the reference). The native volume is gated against ITS OWN closed form (host) and
  compared to OCCT with a loose deflection+convention tolerance (sim), never a wrong solid.

## Gates

- **GATE a (host, no OCCT) — GREEN.** `test_native_blend` canal_fillet_* (4 cases): a native
  Steinmetz bicylinder (built via the native SSI boolean) fillets at r/Rc ∈ [0.1, 0.4] to a
  watertight, consistently-oriented (χ=2) solid that REMOVES material (keeps > half the body)
  and CONVERGES as the deflection refines; out-of-envelope (box / single cylinder / r ≤ 0 /
  Rc < 2r / multi-edge) declines to NULL. Full host `ctest` 65/65 green.
- **GATE b (sim, native-vs-OCCT) — GREEN (50/50).** The OCCT oracle confirms the case is real
  (bicylinder COMMON at Rc=1 filleted at r=0.15/0.20/0.30 removes 0.006/0.011/0.025 via
  `BRepFilletAPI_MakeFillet` + `BRepGProp`). The native canal FILLET is host-gated; the native
  bicylinder BODY is not currently constructible through the `cc_boolean` facade (the native
  COMMON of two full cylinders declines — a boolean-track breadth gap, NOT a fillet gap), so
  the sim records that honestly and does not fake a native body.

## Impact

- The landed blend substrate (`fillet_edges.h`, the cylinder/concave/variable/cone/sphere arms
  of `curved_fillet.h`, `fillet_corner.h`), M0/M1/M2 and the byte-frozen boolean welds remain
  **untouched**; the canal arm is an additive sibling reached only for a native Steinmetz body.
- No `cc_*` ABI change. `src/native/**` stays OCCT-free. The tessellator is not modified, so
  the byte-identical firewall is trivially met (zero mesher diff).
- SEMANTIC note: the native arm rounds the WHOLE crossing crease (all four arcs together),
  which differs from OCCT's per-picked-edge `MakeFillet` (one arc); this is the only watertight
  resolution for a crease whose arcs meet at the poles, and is moot in practice until the
  boolean track can build a native bicylinder body through the facade.
