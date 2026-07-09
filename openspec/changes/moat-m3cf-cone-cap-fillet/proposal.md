# Proposal — moat-m3cf-cone-cap-fillet (MOAT M3, curved-edge fillet on a cone frustum)

## Why

`cc_fillet_edges` is the app's HEAVIEST curved-blend gap (@15 sites, readiness Class-A
residual M3). The landed native curved-fillet slice serves the circular rim between a
**cylinder** wall and a coaxial planar cap (convex + concave + variable radius). The next
tractable analytic-curved envelope on the SAME op is the circular rim of a **cone frustum**
(a tapered plug / tapered boss / truncated cone) meeting a coaxial planar cap — a tapered
shaft rim, which the app produces via `cc_solid_revolve` of a trapezoidal profile.

A constant-radius rolling-ball fillet on a convex cone↔cap circular rim is exactly analytic:
the ball centre traces a circle coaxial with the cone, so the blend surface is a coaxial
**torus band** (major radius = the ball-centre radius, minor radius r), swept over the tilted
minor angle from the cone-wall seam to the cap seam, G1-tangent to both. The cylinder is the
σ=0 special case of this construction. Only OCCT served this rim before; there is no reason
to keep the LGPL engine on its critical path for the common tapered-rim case.

## What changes

- **`src/native/blend/curved_fillet.h` (additive):** new `detail::coneInfo` /
  `detail::coneCapGeom` / `detail::buildFilletedCone` and the public entry
  `blend::cone_fillet_edge(solid, edgeIds, edgeCount, r, deflection)`. It recognises the
  picked circular rim as the convex crease between a coaxial cone-frustum wall and a coaxial
  planar cap, validating the body **wholesale** (a full revolve fragments the wall/cap into
  angular sectors, so single-face matching cannot be used): every face must be a coaxial
  cone of the SAME σ / reference radius, or an axis-normal plane at one of exactly TWO
  distinct heights, and the rim radius must match the cone radius at the rim height. It
  rebuilds the capped frustum as a planar-facet soup sharing the same N angular samples
  across every seam (far cap + cone wall up to the wall-tangent circle + torus band +
  trimmed cap) and welds watertight through the existing `assembleSolid` — **no tessellator
  change**.
- **`NativeEngine::fillet_edges` (additive candidate #4):** after the cylinder-convex,
  cylinder-concave and stepped-shaft candidates, try `nblend::cone_fillet_edge`, gated by
  the SAME `blendResultVerified` SHRINK self-verify (0 < Vr < Vo). Cone faces exist only on
  native bodies of revolution, so an OCCT body never reaches this arm and the cylinder case
  (σ=0) is handled by the earlier `curved_fillet_edge` candidate; there is no dead code.

## Honest scope / declines (→ OCCT)

- Ring-torus guard: the ball-centre radius must be ≥ r (a steep taper whose centre circle is
  smaller than the tube radius would self-intersect the axis) → NULL.
- The wall-tangent circle must stay on the frustum wall (between the far end and the rim).
- The body must be a PURE capped frustum: a cylinder (σ=0, owned by `curved_fillet_edge`),
  a stepped shaft, a multi-frustum (two cones with different σ), a tilted cap, a concave
  cone rim, or any freeform face → NULL → OCCT.
- Multiple picked edges / zero radius → NULL.

## Impact

- The landed blend substrate (`fillet_edges.h`, the cylinder/concave/variable arms of
  `curved_fillet.h`), M0/M1/M2 and the byte-frozen boolean welds remain **untouched**; the
  cone arm is an additive sibling of the cylinder arm.
- `src/native/**` stays OCCT-free (descriptive comments only).
- `cc_*` ABI unchanged (native path only, behind the existing `fillet_edges` facade seam).
- The tessellator (`src/native/tessellate/`) is consumed as-is, UNTOUCHED.
- Readiness: `cc_fillet_edges` moves the analytic cone-frustum cap-rim envelope from
  OCCT-forward to native; the freeform / concave-cone / spindle residual stays OCCT.
