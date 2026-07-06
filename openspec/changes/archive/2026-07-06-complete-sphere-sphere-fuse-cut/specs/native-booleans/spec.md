# native-booleans

Complete the transversal sphere∩sphere curved-boolean op-set in
`src/native/boolean/ssi_boolean.cpp` (`openspec/SSI-ROADMAP.md` S5). The COMMON of two
overlapping spheres is ALREADY native (`buildLensCommon` — two inside-the-other spherical
caps welded along the ONE traced seam circle). This change adds native **Fuse** and
**Cut** for the SAME single-seam sphere pair — the same seam geometry with a DIFFERENT cap
selection — so the family is 3/3 native.

The seam circle C splits each sphere into an INNER cap (apex nearest the other centre) and
an OUTER cap (apex at the far pole). COMMON = inner-A + inner-B (the lens); FUSE =
outer-A + outer-B (the peanut outer shell); CUT `A − B` = outer-A + inner-B REVERSED (the
inner cap of B, normal flipped inward, bounds the scooped cavity). All three share C and
weld watertight along it.

Both new ops DECLINE (NULL → OCCT) outside their verified envelope; tangent / coincident /
near-containment spheres remain the S4 boundary. Internal: **no `cc_*` ABI change** —
invoked behind the existing `cc_boolean` op codes. `src/native/**` stays OCCT-free; the
path is compiled under `CYBERCAD_HAS_NUMSCI`. No change to `src/native/tessellate`, the
planar BSP-CSG, the analytic `curved.h`, the through-drill `buildCommon/Fuse/Cut`, or the
`buildLensCommon` COMMON path (which stays byte-identical).

## ADDED Requirements

### Requirement: SSI-driven native Fuse for the transversal sphere∩sphere lens

The native boolean library SHALL compute `cc_boolean(a, b, op=0)` fuse (`A ∪ B`) NATIVELY
for the transversal sphere∩sphere pair the S5-c COMMON path (`buildLensCommon`) already
recognises: both operands recognised as `Sphere` solids, and the S3 trace a SINGLE closed
seam circle with `nearTangentGaps == 0`. It SHALL reuse the SAME decimated shared seam
(`decimateSeam(seam, seamNodeTarget(seam))`), the SAME shared `VertexPool` weld, and the
SAME radial-ring planar-triangle cap discipline as `buildLensCommon`, and SHALL assemble
the union boundary as the two **OUTER** spherical caps — the cap of A whose apex is A's far
pole (`cA − RA·unit(cA→cB)`) and the cap of B whose apex is B's far pole
(`cB + RB·unit(cA→cB)`) — each oriented with the sphere OUTWARD radial normal, welded along
the ONE shared seam.

The builder SHALL keep each outer cap only if its far-pole apex classifies strictly OUTSIDE
the other solid (the transversal-lens fuse survival rule, via the S5-a curved
point-in-solid test `classifyPoint`); a far pole robustly INSIDE the other sphere
(containment) or ON it (tangent), a non-sphere or multi-seam input, or a weld that cannot
close SHALL return a NULL `Shape` (→ OCCT). The tolerance SHALL NOT be weakened to force a
pass.

The result SHALL be a native `topology::Shape` of type `Solid` carrying true `Sphere` face
kinds, watertight (every edge shared by exactly two faces), whose enclosed volume equals
`vol(A) + vol(B) − vol(A ∩ B)` within a relative tolerance sized to the curved-face
tessellation deflection, where `vol(A ∩ B)` is the native lens COMMON (`buildLensCommon`).
The builder SHALL remain OCCT-free and reference no OCCT / `IEngine` / `EngineShape` type,
SHALL be compiled under `CYBERCAD_HAS_NUMSCI`, and SHALL add or change no `cc_*` entry
point, signature, or POD struct.

#### Scenario: The sphere∩sphere fuse grows to the peanut with the correct volume (host)
- GIVEN two overlapping spheres A (radius `RA`) and B (radius `RB`) whose centres are a
  distance `d` apart with `|RA − RB| < d < RA + RB` (a transversal lens), traced as ONE
  closed seam circle with `nearTangentGaps == 0`
- WHEN `ssi_boolean_solid(A, B, Op::Fuse)` runs (the through-drill `buildFuse` declines the
  single seam, `buildLensFuse` takes over)
- THEN it returns a watertight `Solid` (`boundaryEdgeCount == 0`, every edge shared by
  exactly two faces) bounded by the two OUTER spherical caps sharing the ONE seam
- AND its enclosed volume equals `4/3·π(RA³ + RB³) − lens` (with `lens = V_cap(A) +
  V_cap(B)`, `V_cap = π h² (3R − h)/3`) within the deflection-sized band — a GROW
  (`Vr > max(vol(A), vol(B))`)
- AND every seam node lies on BOTH sphere surfaces within tolerance.

#### Scenario: A far pole inside the other sphere declines to OCCT (host)
- GIVEN two spheres where one is largely contained in the other (a far pole classifies
  INSIDE the other solid), or tangent (a far pole ON the other surface)
- WHEN `ssi_boolean_solid(A, B, Op::Fuse)` runs
- THEN `buildLensFuse` returns a NULL `Shape` (the fuse survival rule needs both far poles
  strictly outside) and the engine falls through to OCCT `BRepAlgoAPI_Fuse` — reported, not
  faked, tolerance not weakened.

#### Scenario: The engine discards a wrong-volume fuse candidate (host)
- GIVEN a fuse candidate whose welded shell volume does not match `vol(A) + vol(B) −
  vol(A ∩ B)` (a mis-welded seam or wrong cap selection)
- WHEN the engine's generic set-algebra self-verify runs (`expected = va + vb − vc`, `vc` =
  native lens COMMON)
- THEN the candidate FAILS the watertight + correct-volume guard and is DISCARDED → OCCT;
  the engine never emits an unverified sphere fuse.

### Requirement: SSI-driven native Cut for the transversal sphere∩sphere lens

The native boolean library SHALL compute `cc_boolean(a, b, op=1)` cut (`A − B`, `A` the
minuend) NATIVELY for the transversal sphere∩sphere pair the S5-c COMMON path already
recognises (both `Sphere`, one closed seam, `nearTangentGaps == 0`). It SHALL reuse the
SAME decimated shared seam, `VertexPool`, and radial-ring planar-triangle cap discipline as
`buildLensCommon`, and SHALL assemble the difference boundary as the **OUTER** cap of A
(apex = A's far pole, outward radial normal) plus the **INNER** cap of B (apex nearest A)
emitted **REVERSED** (inward radial normal) so it bounds the scooped cavity, both welded
along the ONE shared seam. Operand order SHALL be honoured (CUT is not symmetric),
matching `BRepAlgoAPI_Cut(a, b)`.

The builder SHALL proceed only if A's far-pole apex classifies OUTSIDE B AND B's inner apex
classifies INSIDE A (the transversal-lens cut survival rule, via `classifyPoint`); a
tangent / degenerate / wrong-side apex, a non-sphere or multi-seam input, or a weld that
cannot close SHALL return a NULL `Shape` (→ OCCT). The tolerance SHALL NOT be weakened.

The result SHALL be a native `topology::Shape` `Solid` with true `Sphere` face kinds,
watertight (every edge shared by exactly two faces), whose enclosed volume equals
`vol(A) − vol(A ∩ B)` within the deflection-sized relative tolerance, where `vol(A ∩ B)` is
the native lens COMMON. The builder SHALL remain OCCT-free, reference no OCCT / `IEngine` /
`EngineShape` type, be compiled under `CYBERCAD_HAS_NUMSCI`, and add or change no `cc_*`
entry point, signature, or POD struct.

#### Scenario: The sphere∩sphere cut scoops the lens with the correct volume (host)
- GIVEN two overlapping spheres A (radius `RA`, minuend) and B (radius `RB`), transversal
  lens, ONE closed seam, `nearTangentGaps == 0`
- WHEN `ssi_boolean_solid(A, B, Op::Cut)` runs (the through-drill `buildCut` declines the
  single seam, `buildLensCut` takes over)
- THEN it returns a watertight `Solid` bounded by A's OUTER cap (outward) plus B's INNER cap
  REVERSED (inward, bounding the cavity), sharing the ONE seam
- AND its enclosed volume equals `4/3·π·RA³ − lens` within the deflection-sized band — a
  SHRINK (`Vr < vol(A)`)
- AND every seam node lies on both sphere surfaces within tolerance.

#### Scenario: The reversed inner cap bounds the cavity, verified against the native lens (host)
- GIVEN the sphere∩sphere cut candidate above
- WHEN the engine's generic set-algebra self-verify runs (`expected = va − vc`, `vc` =
  native lens COMMON `buildLensCommon`)
- THEN a candidate whose reversed INNER-B cap is mis-oriented (outward, not bounding the
  cavity) yields the wrong enclosed volume, FAILS the guard, and is DISCARDED → OCCT — the
  correct candidate matches `vol(A) − lens` and is accepted native.

#### Scenario: The COMMON path is unchanged by the fuse/cut completion (host)
- GIVEN the SAME overlapping sphere pair
- WHEN `ssi_boolean_solid(A, B, Op::Common)` runs after the generalisation of
  `appendSphereCap` (defaulted `outer=false, reversed=false`)
- THEN `buildLensCommon` produces the byte-identical two-inner-cap lens (same volume, area,
  and vertices as before this change) — the COMMON native pass does not regress.
