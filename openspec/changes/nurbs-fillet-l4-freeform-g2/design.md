# Design — nurbs-fillet-l4-freeform-g2

## Context

The analytic G2 fillet family (`fillet_edges_g2`, `curved_fillet_g2`, `fillet_edges_g2_variable`) all
share one core: a quintic-Bézier cross-section whose end curvature is set by the pole rule
`q = (5/4)·κ·h²`. Only the source of `κ` differs — a plane gives 0, a sphere `1/R`, a straight ruling
0. This slice replaces that per-primitive `κ` lookup with a general read of the freeform surface's
LOCAL normal curvature, so ONE section builder covers every substrate and the analytic cases fall out
as special values.

## Key decision — the CENTRE LOCUS is the marching variable, not a face contact point

The first (rejected) design fixed a contact station `(uA,vA)` on faceA and Newton-solved faceB's
contact so the two rolling-ball centres coincide. On a CURVED faceA this over-constrains: a ball
touching faceA at an arbitrarily chosen point in general does not touch faceB at all, so the residual
`cB − cA` cannot reach zero (its component normal to faceB's tangent plane is an irreducible gap). A
debug trace showed the residual stalling at ~0.15 for the bump crease — a correct diagnosis, not a
solver bug.

The fix is the standard rolling-ball formulation: the free variable is the ball CENTRE `c` (3 DOF),
constrained by `dist(c, A) = r` and `dist(c, B) = r` (2 equations) — leaving a 1-DOF spine curve. Per
station we:
1. project `c` onto each surface (`footpoint`, a Gauss–Newton on `(q−S)·Sᵤ = (q−S)·Sᵥ = 0`),
2. read the signed offset distances `dA = (c−pA)·n̂A`, `dB = (c−pB)·n̂B` (the footpoint guarantees
   `c−p` is parallel to the surface normal),
3. move the centre `δc = α·n̂A + β·n̂B` (a 2×2 Gram solve) to drive `dA,dB → r`. The third DOF (along
   `n̂A × n̂B`) is left free — that IS the spine direction, so the min-norm move in `span{n̂A,n̂B}`
   keeps the centre on the spine.

The march then steps the centre along `n̂A × n̂B` (oriented to run with the caller's `spineDir`
guide) and the next station's footpoint/centre Newton refines it back onto the true spine. Warm-
starting the footpoint params across stations keeps each seat a few Newton steps.

This converges to machine precision on the planar corner (exact seat, `κ=0`), the rational sphere
(`κ=1/R` to 2e-16), and the bicubic bump crease (`κ≈±0.10` read from the second fundamental form, G2
residuals ~1e-15).

## Curvature-offset sign

The pole offset `q·N̂` uses `N̂ = −n̂` (toward the ball centre / into the material), so a positive `κ`
curls the section the same way the substrate curves. This reduces EXACTLY to the analytic builders: a
plane (`κ=0`) → `q=0` (collinear triple, identical to `fillet_edges_g2`); an umbilic sphere
(`κ=1/R`) → `q=(5/4)(1/R)h²` toward the sphere centre (identical to `curved_fillet_g2`'s wall end).

## Scope boundary — the module owns the general path, the caller owns the seeding

Finding the crease between two ARBITRARY freeform faces (which parameter region, which normal
orientation, where the spine starts) is configuration-specific and hard; this module takes that as a
`FreeformFilletSeed` (initial centre, sign convention `sA,sB`, spine-run guide, station count) and
owns the GENERAL per-station seat + section + skin + honest-decline. That split keeps the hard,
proven core testable against airtight analytic oracles while leaving general crease-finding as an
explicit residual.

## Alternatives considered

- **Fit a rational NURBS fillet surface** (instead of a triangle band) — deferred; the band is the
  deliverable the later trimmed-face stitch consumes, and skinning a NURBS surface through the
  quintic sections is a separable Layer-6 step.
- **Trim a folded fillet into a valid one** — rejected for this slice; a fold is an honest decline
  (the moat discipline forbids emitting self-intersecting geometry), and fold recovery is materially
  harder.
