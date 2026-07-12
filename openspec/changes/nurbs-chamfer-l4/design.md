# Design — nurbs-chamfer-l4

## Context

The chamfer is the flat-bevel sibling of the fillet. Where the G2 fillet family rolls a ball into a
crease and lays a tangent-continuous quintic section, a chamfer sets each face back by a prescribed
distance and bridges the two setback curves with a ruled (flat) face. It is C0 at each seam, NOT
tangent — a deliberate geometric inversion of the fillet's G1/G2 continuity.

The existing `chamfer_edges.h` is BYTE-FROZEN and consumed by `shell.h`, `corner_chamfer_weld.h`,
`native_blend.h`, the `fillet_edges*` helpers, and the `cc_chamfer_edges` facade. It chamfers a
convex edge of a PLANAR solid via a Sutherland–Hodgman plane cut for the OCCT-fallback engine. This
slice must NOT touch it; it adds a distinct, standalone NURBS/analytic GENERATOR in a new header and a
new sub-namespace (`chamfer_nurbs`), so the additive / ABI-frozen invariant holds.

## Goals / Non-goals

- Goal: a closed-form, OCCT-free chamfer GENERATOR over analytic substrates (plane/cylinder/cone) that
  produces the two setback curves + a ruled chamfer face, with airtight oracles and honest decline.
- Non-goal: B-rep stitch, freeform-edge chamfer, self-intersection recovery, any `cc_*` ABI.

## Decisions

### Setback measured ALONG the surface (geodesic), not chordally

A chamfer setback `d` is defined "in the face, perpendicular to the edge" — meaning SURFACE distance,
not straight-line distance. On a plane the two coincide (the in-plane offset line). On a curved
substrate they differ, and the correct construction is the along-surface (geodesic/normal) offset:
- CYLINDER, axis-parallel ruling edge: the surface distance from the ruling is arc length `R·Δψ`, so
  a setback `d` is the ruling at angular offset `Δψ = d/R` (rotate the edge point about the axis).
- CYLINDER, cross-section circle edge: the axial direction is a straight geodesic, so an axial slide
  of `d` IS the surface distance `d`.
- CONE, circular rim edge: the slant generator is a straight geodesic on the developable cone, so
  sliding the rim point along the in-wall generator by `d` is the exact along-surface offset.
- PLANE: the straight in-plane offset by `d`.

This is what makes the cylinder oracle airtight at ≤1e-9: the wall rail STAYS on the cylinder (radius
`R`) at axial offset `d`, which is only true for the geodesic offset — a chordal offset would leave
the surface.

### Ruled (Piegl & Tiller) chamfer face

Given the two setback rails `cA(t), cB(t)` in station correspondence, the chamfer face is the ruled
loft `R(t,τ) = (1−τ)·cA(t) + τ·cB(t)`. It is linear in `τ`, so one `τ`-strip per `t`-interval is
exact; sampled into two triangles per quad. For a planar dihedral both rails are straight lines and
`R` is EXACTLY planar — the four-corner best-fit-plane residual is ~1e-18 in practice, well under the
1e-12 oracle. The face meets each base face C0 (shared rail points on the base surface), not tangent —
the correct chamfer continuity.

### Three modes reduce to one leg-pair

All three entry points resolve to a `(dA, dB)` leg pair fed to one `buildChamfer`:
- symmetric(d) → (d, d);
- asymmetric(d1, d2) → (d1, d2);
- distance_angle(d, α) → (d, d·tan(α)), where α is measured from the faceA tangent plane.
So `asymmetric(d,d) ≡ symmetric(d)` and `distance_angle(d, atan(d2/d)) ≡ asymmetric(d, d2)` hold to
rounding (≤1e-12) — the mode-equivalence oracles fall out of the single code path.

### Honest decline (no self-intersecting face)

Two over-large failure modes are caught intrinsically, without needing the face extents:
- (a) the setback chord `cA(t)→cB(t)` collapses (rails coincident) or flips sign vs the first station
  (bevel width inverts);
- (b) a rail sweeps the WRONG way — its per-station step reverses relative to the edge step. This
  catches a radial planar setback pushed PAST the axis (the rail laps to the mirror quadrant) or a
  circumferential wrap that laps the ring, which mode (a) alone misses for a uniform overshoot.
Plus: degenerate dihedral (parallel faces / null edge tangent → `t × n` null), non-positive setback,
and freeform substrate all decline with a measured reason. No tolerance is widened.

## Risks / trade-offs

- The over-large detector is a geometric (rail-sweep) heuristic, not a full face-boundary
  intersection test — it is CONSERVATIVE for the supported substrates (it fires before any fold is
  emitted) and correct for the closed-form cases the module handles; a genuinely freeform boundary is
  out of scope (declined as `UnsupportedSubstrate`).
- The module returns the chamfer BAND + rails, not a stitched solid — the caller owns the B-rep weld.
