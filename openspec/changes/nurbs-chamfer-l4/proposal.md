# Proposal — nurbs-chamfer-l4 (NURBS roadmap Layer 4, the CHAMFER half of fillet/chamfer)

## Why

`src/native/blend/` lands an extensive G2 FILLET family — planar dihedrals, sphere/cylinder/cone↔cap
curved rims, variable-radius, and the freeform-substrate G2 fillet (`fillet_edge_g2_freeform.h`). What
is missing is the sibling primitive: a CHAMFER, which replaces an edge with a FLAT BEVEL face — a
ruled (bilinear-lofted) transition that is C0 (position-continuous) but deliberately NOT tangent at
either seam. It is a distinct, simpler-but-essential CAD feature: specified by a symmetric setback
distance `d`, two asymmetric setbacks `(d1,d2)`, or a distance+angle `(d,α)`.

The existing byte-frozen `chamfer_edges.h` chamfers a convex edge of a PLANAR solid by a
Sutherland–Hodgman plane cut for the OCCT-fallback engine. This slice adds the NURBS/analytic
GENERATOR: given the two adjacent faces (as analytic substrates) and the shared edge, it traces the
two SETBACK CURVES (one per face, at a setback measured ALONG each face's surface) and lofts a ruled
Piegl & Tiller chamfer face between them.

It is worth building now because (a) it reuses only the landed `native/math/vec.h` primitives, (b) it
has AIRTIGHT closed-form oracles (a planar dihedral gives an exactly-planar chamfer face with setback
lines at distance `d`; the three modes reduce to one another; the cylinder setback is the exact
geodesic offset), and (c) the hard case (over-large setback / degenerate dihedral) is an HONEST
DECLINE, never a self-intersecting face — the moat discipline.

## What

A new OCCT-free header `src/native/blend/chamfer_edge_nurbs.h` (namespace
`cybercad::native::blend::chamfer_nurbs`), ADDITIVE — it sits ALONGSIDE the byte-frozen solid-clip
`chamfer_edges.h` and touches no existing header. It exposes three entry points returning a
`ChamferResult` (the two setback curves + the ruled chamfer face triangles + a planarity witness +
an honest decline reason):

1. `chamfer_edge_symmetric(faceA, faceB, edge, d)` — each face set back by the same `d`, measured in
   the face, perpendicular to the edge, ALONG the surface.
2. `chamfer_edge_asymmetric(faceA, faceB, edge, d1, d2)` — faceA set back `d1`, faceB set back `d2`.
3. `chamfer_edge_distance_angle(faceA, faceB, edge, d, angleDeg)` — faceA set back `d`; the faceB leg
   is `d·tan(angleDeg)` so the bevel makes `angleDeg` with faceA. Equivalent to
   `asymmetric(d, d·tan(angleDeg))`.

The SETBACK CURVE per face is closed-form per analytic substrate: a PLANE offsets the straight edge
line by `d` along the in-plane inward direction (edge-tangent × face-normal); a CYLINDER offsets an
axis-parallel ruling AROUND the cylinder by arc length (`Δψ = d/R`, the exact geodesic/normal offset)
or a cross-section circle AXIALLY by `d`; a CONE offsets a circular rim along the slant generator by
`d`. The ruled chamfer face is the Piegl & Tiller loft `R(t,τ) = (1−τ)·cA(t) + τ·cB(t)`, sampled into
quad strips. For the planar dihedral this face is EXACTLY planar.

An HONEST DECLINE (empty triangles + a measured `ChamferDecline`) is returned when a setback exceeds
the face extent (the rails cross / a rail sweeps backward), the dihedral is degenerate (parallel
faces / null edge tangent), or the substrate is unsupported (freeform edge); NO tolerance is widened.

## Verification (HOST-analytic, the airtight oracle is the whole point)

`tests/native/test_native_chamfer_edge_nurbs.cpp` (host, numsci-gated, mirrors
`test_native_fillet_g2_freeform` wiring):

1. **Planar dihedral exactness** — a 90° box corner, symmetric `d`: the two setback rails lie ON each
   base plane (C0) at in-plane distance `d` (≤1e-12), the chamfer face is EXACTLY planar (four-corner
   planarity residual ≤1e-12), and its normal makes 45° with each base face (the symmetric-90°
   signature, cos = 1/√2 ≤1e-12).
2. **Mode consistency** — `asymmetric(d,d)` reproduces `symmetric(d)` rail-for-rail (≤1e-12);
   `distance_angle(d, atan(d2/d))` reproduces `asymmetric(d, d2)` rail-for-rail (≤1e-12).
3. **Cylinder substrate** — a plane cap ↔ coaxial cylinder wall on the circular rim: the cap rail is
   the circle radius `R−d` at the cap height, the wall rail STAYS on the cylinder (radius `R`) at
   axial offset `d` — the exact geodesic/normal offset (≤1e-9).
4. **Honest decline** — a negative setback (`BadArguments`), a freeform substrate
   (`UnsupportedSubstrate`), parallel faces (`DegenerateDihedral`), and an over-large radial cap
   setback that laps the ring (`OverLargeSetback`) all decline with an empty band, never a
   self-intersecting face.

## Scope

- Adds `src/native/blend/chamfer_edge_nurbs.h` — OCCT-free (0 OCCT/Geom/BRep/TK refs), header-only,
  `#include`s only `native/math/vec.h`. Additive; not wired into `native_blend.h` (it is a standalone
  generator over analytic substrates, distinct from the frozen solid-clip `chamfer_edges.h`).
- Adds `tests/native/test_native_chamfer_edge_nurbs.cpp` (host, `CYBERCAD_HAS_NUMSCI`-gated) wired
  into CMake mirroring `test_native_fillet_g2_freeform` (macro only; no substrate include trees).
- **`cc_*` ABI byte-unchanged.** This is an internal geometry-algorithm slice; its consumer is a
  later chamfer facade, not the app today. No ABI is added until a consumer needs it.

## Non-goals

- **No trimmed-face stitch.** The slice returns the chamfer FACE (its own ruled triangles) + the two
  setback curves; welding it against the trimmed base faces into a watertight solid is a distinct
  B-rep construction and a documented residual.
- **No self-intersecting-chamfer recovery.** An over-large setback / degenerate dihedral is DECLINED
  honestly; the module never returns folded geometry.
- **No freeform-edge chamfer.** A chamfer across a genuinely freeform crease (footpoint-traced
  setback on a NURBS surface) is left to a later slice; freeform substrates honest-decline here.
- No new `cc_*` ABI; no change to the frozen `chamfer_edges.h`, the fillet builders, the evaluator
  signatures, the tessellator, or STEP admission.
