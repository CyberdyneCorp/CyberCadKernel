# Proposal — nurbs-fillet-l4-freeform-g2 (NURBS roadmap Layer 4, freeform substrate)

## Why

`src/native/blend/` already lands G2 (curvature-continuous) rolling-ball fillets on ANALYTIC
substrates: planar dihedrals (convex + concave, `fillet_edges_g2.{h}` / `_concave`), the sphere↔cap
and cylinder/cone↔cap curved rims (`curved_fillet_g2.h`), and variable-radius on planar and
curved substrates (`fillet_edges_g2_variable.h`). Every one uses the quintic-Bézier cross-section
with the pole rule `q = (5/4)·κ·h²` to match the base face's END CURVATURE, reading `κ` from a
KNOWN analytic form (a plane → 0, a sphere → 1/R, a straight-ruled cyl/cone → 0 along the ruling).

The remaining gap is the deepest one: a G2 fillet where the TWO base faces are **general freeform
NURBS surfaces** (not analytic primitives). There the end curvature is no longer a closed-form
constant — it must be read from the LOCAL second fundamental form of each freeform surface at the
rolling-ball contact point. This slice closes that gap with the SAME quintic-section / pole-rule
machinery, generalised to the freeform second fundamental form, so every existing analytic G2
result is REPRODUCED as a special case and genuinely-freeform bump substrates get a true G2 fillet.

It is worth building now because (a) it reuses the landed Layer-1 NURBS surface evaluator
(`nurbsSurfaceDerivs`, order 2 → both fundamental forms) and the proven quintic pole rule, (b) it
has an **airtight analytic-reduction oracle** (a NURBS plane/sphere must reproduce the existing
analytic G2 fillet to ≤1e-9), and (c) the hard case (a ball that will not fit / a fold) is an
**honest decline**, never a self-intersecting fillet — the moat discipline.

## What

A new OCCT-free header `src/native/blend/fillet_edge_g2_freeform.h` (namespace
`cybercad::native::blend`), additive — it is included by the `native_blend.h` aggregate and touches
no existing header. It exposes `fillet_edge_g2_freeform(faceA, faceB, radius, seed, nSectionSamples)`
returning a `FreeformFilletResult` (the skinned fillet triangles + per-station continuity data +
an honest decline reason). The three pieces:

1. **Contact / spine marching (the CENTRE LOCUS).** The rolling-ball spine is the set of centres
   equidistant `r` from both freeform faces. Choosing a contact point on ONE curved face
   over-constrains (a ball touching faceA at a chosen point generally does not touch faceB), so the
   free variable is the ball CENTRE `c`: for a centre guess we drop the FOOTPOINT (nearest point)
   onto each surface via a Gauss–Newton projection, giving the contacts `pA,pB` and the offset
   distances, then Newton-adjust `c` in `span{n̂A,n̂B}` so both distances equal `r`. The station's
   centre is on the spine; the march steps `c` along the local spine tangent `n̂A × n̂B`.
2. **G2 quintic cross-section (freeform end curvatures).** At each station the section plane is
   spanned by the two contact normals. The quintic `B(s)` has `P0=pA`, `P5=pB`, end tangents in each
   face's tangent plane (G1), and end curvature matching each face's NORMAL CURVATURE in the section
   plane, read from the freeform second fundamental form
   `κ_n(d) = (L·du² + 2M·du·dv + N·dv²)/(E·du² + 2F·du·dv + G·dv²)` (E,F,G the first fundamental form
   `Sᵤ·Sᵤ,Sᵤ·Sᵥ,Sᵥ·Sᵥ`; L,M,N the second `Sᵤᵤ·n̂,Sᵤᵥ·n̂,Sᵥᵥ·n̂`; all from `nurbsSurfaceDerivs` order
   2). The pole offset is the SAME rule `q_i = (5/4)·κ_i·h²`, so a plane (`L=M=N=0`) collapses to the
   collinear triple and an umbilic sphere reads `1/R` in every direction — the analytic reductions.
3. **Skin.** Consecutive stations' quintic sections are lofted into triangle strips.

An HONEST DECLINE (empty triangles + a measured `FreeformFilletDecline`) is returned when any station
will not seat (the ball won't fit — `r` past the local concave limit), the section folds, or the
curvature read is non-finite; NO tolerance is ever widened.

## Verification (HOST-analytic, the airtight oracle is the whole point)

`tests/native/test_native_fillet_g2_freeform.cpp` (host, numsci-gated, mirrors
`test_native_nurbs_fit` wiring):

1. **Analytic-reduction** — two NURBS PLANES read normal curvature 0 and build the zero-end-curvature
   quintic (section end curvature ≤1e-9); a rational-NURBS SPHERE reads its umbilic normal curvature
   `1/R` in every direction (≤1e-9) — proving the general path reproduces the analytic G2 result.
2. **G2-to-face** — at every seated station the section tangent lies in the face tangent plane (G1,
   ≤1e-6 rad) and the section end curvature equals the face's normal curvature in the section plane
   (relative ≤1e-4).
3. **Genuinely freeform** — two bicubic bump faces seat a G2 fillet with genuinely non-zero
   curvatures matched to the same bounds; an over-radius ball HONEST-DECLINES (no triangles emitted,
   never a self-intersecting fillet).

## Scope

- Adds `src/native/blend/fillet_edge_g2_freeform.h` — OCCT-free (0 OCCT/Geom/BRep/TK refs),
  header-only, `#include`s only `native/math/bspline.h` + `native/math/vec.h`. Added to
  `native_blend.h`.
- Adds `tests/native/test_native_fillet_g2_freeform.cpp` (host, `CYBERCAD_HAS_NUMSCI`-gated) wired
  into CMake mirroring `test_native_nurbs_fit` (macro only; no substrate include trees).
- **`cc_*` ABI byte-unchanged.** This is an internal geometry-algorithm slice; its consumer is a
  later freeform-fillet facade, not the app today. No ABI is added until a consumer needs it.

## Non-goals

- **No trimmed-face stitch.** The slice returns the fillet BAND (its own oriented triangles); welding
  it against the trimmed base faces into a watertight solid is a distinct B-rep construction and a
  documented residual.
- **No self-intersecting-fillet recovery.** A ball that will not fit / a folded section is DECLINED
  honestly; the module never returns folded geometry, and does not attempt to trim a fold into a
  valid fillet.
- **No automatic spine seeding.** The caller supplies the initial centre + sign convention +
  spine-run guide (the configuration-specific seeding); the module owns the general per-station seat
  + section + skin. General crease-finding across arbitrary freeform pairs is a residual.
- No new `cc_*` ABI; no change to the analytic G2 builders, the evaluator signatures, the
  tessellator, or STEP admission.
