# native-booleans

Complete the Steinmetz bicylinder curved-boolean op-set in
`src/native/boolean/ssi_boolean.cpp` (`openspec/SSI-ROADMAP.md` S5-d). The COMMON of two equal-
radius cylinders whose axes cross orthogonally is ALREADY native (`buildSteinmetzCommon` — the
four inside-the-other lune patches welded along the four traced branch arcs + the two branch-
point poles, verified against the exact analytic bicylinder volume `16 R³/3`). This change adds
native **Fuse** and **Cut** for the SAME branched trace — the same four arcs with a DIFFERENT
fragment (lune) selection plus the original disc end caps — so the family is 3/3 native.

The four branch arcs split each cylinder wall into an INSIDE region (the lune, inside the other
cylinder) and an OUTSIDE region; each cylinder also carries its two original disc end caps, which
lie well outside the intersection band and are untouched by the arcs. COMMON = the four INSIDE
lunes (the bicylinder). FUSE = both cylinders' OUTSIDE walls + all four disc end caps (the outer
envelope). CUT `A − B` = A's OUTSIDE wall + A's two disc caps + B's INSIDE lunes REVERSED (inward
radial normal, bounding the carved channel through A). All three share the four arcs and the two
poles and weld watertight along them.

Both new ops DECLINE (NULL → OCCT) outside their verified envelope; short cylinders (caps within
the seam band), non-equal-radius / non-orthogonal / non-crossing pairs, and unresolved branched
traces remain the S4 / OCCT boundary. Internal: **no `cc_*` ABI change** — invoked behind the
existing `cc_boolean` op codes. `src/native/**` stays OCCT-free; the path is compiled under
`CYBERCAD_HAS_NUMSCI`. No change to `src/native/tessellate`, the planar BSP-CSG, the analytic
`curved.h`, the through-drill / sphere-lens builders, or the `buildSteinmetzCommon` COMMON path
(which stays byte-identical).

## ADDED Requirements

### Requirement: SSI-driven native Fuse for the Steinmetz bicylinder pair

The native boolean library SHALL compute `cc_boolean(a, b, op=0)` fuse (`A ∪ B`) NATIVELY for
the Steinmetz bicylinder pair the S5-d COMMON path (`buildSteinmetzCommon`) already recognises:
both operands recognised as `Cylinder` solids of near-equal radius whose axes cross orthogonally
(`steinmetzPreGate`), and the branch-enabled S4-d re-trace a resolved branched `TraceSet` with
`branchPoints == 2`, exactly four `BranchArc` arms on both cylinders, and `nearTangentGaps == 0`
(`recogniseSteinmetzTrace`). It SHALL reuse the SAME oriented + pole-axis-resampled four arcs
(`orientArc`, `resampleArcByAxis`), the SAME shared `VertexPool` weld with the two poles pooled
ONCE, and the SAME radial-ring planar-triangle lune discipline (`appendLunePatch`) as
`buildSteinmetzCommon`, and SHALL assemble the union boundary as the two OUTSIDE lune patches of
EACH cylinder (the wall regions outside the other cylinder, oriented with the cylinder OUTWARD
radial normal) PLUS both cylinders' two original disc end caps (full circles at `v = vLo` and
`v = vHi`, oriented along the outward axial direction), all sharing the four arcs and two poles.

The builder SHALL keep each outside lune only if its centroid classifies strictly OUTSIDE the
other solid (the fuse survival rule, via the S5-a curved point-in-solid test `classifyPoint`); a
centroid robustly ON the other wall (tangent), a short cylinder whose end-cap plane falls within
the intersection seam band, a non-Steinmetz or unresolved branched input, or a weld that cannot
close SHALL return a NULL `Shape` (→ OCCT). The tolerance SHALL NOT be weakened to force a pass.

The result SHALL be a native `topology::Shape` of type `Solid` carrying true `Cylinder` (wall)
and planar (cap) face kinds, watertight (every edge shared by exactly two faces), whose enclosed
volume equals `vol(A) + vol(B) − vol(A ∩ B)` within a relative tolerance sized to the curved-face
tessellation deflection, where `vol(A ∩ B)` is the native bicylinder COMMON (`buildSteinmetzCommon`
= `16 R³/3`). The builder SHALL remain OCCT-free and reference no OCCT / `IEngine` / `EngineShape`
type, SHALL be compiled under `CYBERCAD_HAS_NUMSCI`, and SHALL add or change no `cc_*` entry
point, signature, or POD struct.

#### Scenario: The Steinmetz fuse grows to the outer envelope with the correct volume (host)
- GIVEN two equal-radius cylinders A (radius `R`, length `L_A`) and B (radius `R`, length `L_B`)
  whose axes cross orthogonally, whose branch-enabled S4-d `TraceSet` has `branchPoints == 2`,
  four `BranchArc` arms, and `nearTangentGaps == 0`, and whose end caps lie outside the `|y| ≤ R`
  intersection band
- WHEN `ssi_boolean_solid(A, B, Op::Fuse)` runs (the default trace declines near-tangent, the
  branch-enabled re-trace resolves the four arcs, and `buildSteinmetzFuse` assembles the outer
  shell)
- THEN it returns a watertight `Solid` (`boundaryEdgeCount == 0`, every edge shared by exactly
  two faces) bounded by both cylinders' two OUTSIDE lune walls sharing the four arcs, closed by
  all four original disc end caps
- AND its enclosed volume equals `π R²(L_A + L_B) − 16 R³/3` within the deflection-sized band —
  a GROW (`Vr > max(vol(A), vol(B))`)
- AND every arc node lies on BOTH cylinder surfaces within tolerance, and the two branch-point
  poles are pooled ONCE (shared by all surviving lune fragments).

#### Scenario: A short cylinder whose caps clip the seam declines to OCCT (host)
- GIVEN a Steinmetz pair one of whose cylinders is so short that an end-cap plane falls within
  the `|y| ≤ R` intersection seam band (the disjoint-cap assumption fails)
- WHEN `ssi_boolean_solid(A, B, Op::Fuse)` runs
- THEN `appendCylinderEndCaps` reports the clip and `buildSteinmetzFuse` returns a NULL `Shape`,
  and the engine falls through to OCCT `BRepAlgoAPI_Fuse` — reported, not faked, tolerance not
  weakened.

#### Scenario: The engine discards a wrong-volume Steinmetz fuse candidate (host)
- GIVEN a fuse candidate whose welded shell volume does not match
  `vol(A) + vol(B) − vol(A ∩ B)` (a mis-selected lune or a mis-placed cap)
- WHEN the engine's generic set-algebra self-verify runs (`expected = va + vb − vc`, `vc` =
  native bicylinder COMMON `buildSteinmetzCommon = 16 R³/3`; the `op == 2`-only analytic
  Steinmetz oracle does NOT intercept fuse)
- THEN the candidate FAILS the watertight + correct-volume guard and is DISCARDED → OCCT; the
  engine never emits an unverified Steinmetz fuse.

### Requirement: SSI-driven native Cut for the Steinmetz bicylinder pair

The native boolean library SHALL compute `cc_boolean(a, b, op=1)` cut (`A − B`, `A` the minuend)
NATIVELY for the Steinmetz bicylinder pair the S5-d COMMON path already recognises (both near-
equal-radius `Cylinder`, orthogonal crossing axes, a resolved 2-branch-point / four-`BranchArc`
`TraceSet` with `nearTangentGaps == 0`). It SHALL reuse the SAME oriented + pole-axis-resampled
four arcs, `VertexPool` (poles pooled once), and radial-ring planar-triangle lune discipline as
`buildSteinmetzCommon`, and SHALL assemble the difference boundary as A's two OUTSIDE lune walls
(outward radial normal) plus A's two original disc end caps, plus B's two INSIDE lune patches
emitted REVERSED (inward radial normal, `outwardSign = −1`) so they bound the carved channel
through A, all sharing the four arcs and two poles. Operand order SHALL be honoured (CUT is not
symmetric), matching `BRepAlgoAPI_Cut(a, b)`.

The builder SHALL proceed only if A's outside-lune centroids classify OUTSIDE B AND B's inside-
lune centroids classify INSIDE A (the cut survival rule, via `classifyPoint`); a tangent /
degenerate / wrong-side centroid, a short cylinder whose cap clips the seam band, a non-Steinmetz
or unresolved branched input, or a weld that cannot close SHALL return a NULL `Shape` (→ OCCT).
The tolerance SHALL NOT be weakened.

The result SHALL be a native `topology::Shape` `Solid` with true `Cylinder` (wall) and planar
(cap) face kinds, watertight (every edge shared by exactly two faces), whose enclosed volume
equals `vol(A) − vol(A ∩ B)` within the deflection-sized relative tolerance, where `vol(A ∩ B)`
is the native bicylinder COMMON. The builder SHALL remain OCCT-free, reference no OCCT /
`IEngine` / `EngineShape` type, be compiled under `CYBERCAD_HAS_NUMSCI`, and add or change no
`cc_*` entry point, signature, or POD struct.

#### Scenario: The Steinmetz cut carves the channel with the correct volume (host)
- GIVEN two equal-radius cylinders A (radius `R`, length `L_A`, minuend) and B (radius `R`),
  orthogonal crossing axes, a resolved 2-branch-point / four-arc `TraceSet` with
  `nearTangentGaps == 0`
- WHEN `ssi_boolean_solid(A, B, Op::Cut)` runs (branch-enabled re-trace resolves the arcs and
  `buildSteinmetzCut` assembles the shell)
- THEN it returns a watertight `Solid` bounded by A's two OUTSIDE lune walls (outward) + A's two
  disc caps + B's two INSIDE lune walls REVERSED (inward, bounding the channel), all sharing the
  four arcs
- AND its enclosed volume equals `π R² L_A − 16 R³/3` within the deflection-sized band — a SHRINK
  (`Vr < vol(A)`)
- AND every arc node lies on both cylinder surfaces within tolerance, and the two poles are
  pooled ONCE.

#### Scenario: The reversed inner lunes bound the channel, verified against the native common (host)
- GIVEN the Steinmetz cut candidate above
- WHEN the engine's generic set-algebra self-verify runs (`expected = va − vc`, `vc` = native
  bicylinder COMMON `buildSteinmetzCommon`; the `op == 2`-only analytic oracle does NOT intercept
  cut)
- THEN a candidate whose reversed INSIDE-B lunes are mis-oriented (outward, not bounding the
  channel) yields the wrong enclosed volume, FAILS the guard, and is DISCARDED → OCCT — the
  correct candidate matches `vol(A) − 16 R³/3` and is accepted native.

#### Scenario: The COMMON path is unchanged by the fuse/cut completion (host)
- GIVEN the SAME equal-radius orthogonal Steinmetz pair
- WHEN `ssi_boolean_solid(A, B, Op::Common)` runs after the shared arc/pole prologue is factored
  into `orientResampleArcs`
- THEN `buildSteinmetzCommon` produces the byte-identical four-inside-lune bicylinder (same
  volume `16 R³/3`, area, and vertices as before this change) — the COMMON native pass does not
  regress.
