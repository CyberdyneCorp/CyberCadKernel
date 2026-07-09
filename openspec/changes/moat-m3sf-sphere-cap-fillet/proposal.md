# Proposal — moat-m3sf-sphere-cap-fillet (MOAT M3, curved-edge fillet on a sphere)

## Why

`cc_fillet_edges` is the app's HEAVIEST curved-blend gap (@15 sites, readiness Class-A
residual M3). The app fillets the **curved rim of a revolve**
(`RevolveEdgeFilletIntegrationTests`). The landed native curved-fillet arms serve the
circular rim of a **cylinder** (convex/concave/variable) and a **cone frustum** meeting a
coaxial planar cap. The next tractable analytic-curved envelope on the SAME op is the
circular rim of a **sphere** — a truncated ball / dome / spherical plug with a flat cap,
which the app produces via `cc_solid_revolve_profile` of an arc segment capped by a line.

A constant-radius rolling-ball fillet on a convex sphere↔cap circular rim is exactly
analytic: the ball centre lies at distance R−r from the sphere centre and r below the cap,
so by coaxial symmetry it traces a circle coaxial with the sphere. The blend surface is
therefore a coaxial **torus band** (major radius = the centre-circle radius, minor radius r)
swept over the minor angle from the sphere-wall seam to the cap seam, G1-tangent to both.
Only OCCT served this rim before; there is no reason to keep the LGPL engine on its critical
path for the common dome-rim case.

## What changes

- **`src/native/blend/curved_fillet.h` (additive):** new `detail::SphereInfo` /
  `detail::sphereInfo` / `detail::sphereCapGeom` / `detail::buildFilletedSphere` and the
  public entry `blend::sphere_fillet_edge(solid, edgeIds, edgeCount, r, deflection)`. It
  recognises the picked circular rim as the convex crease between a coaxial sphere wall and a
  coaxial planar cap, validating the body **wholesale** (a full revolve fragments the wall/
  cap into angular sectors, so single-face matching cannot be used): every face must be a
  coaxial sphere of the SAME centre / radius, or an axis-normal plane at exactly ONE height
  (the cap), and the rim radius must match √(R²−h²). It rebuilds the truncated ball as a
  planar-facet soup sharing the same N angular samples across every seam (faceted sphere wall
  from the south pole up to the wall-tangent latitude + torus band + trimmed cap) and welds
  watertight through the existing `assembleSolid` — **no tessellator change**.
- **`NativeEngine::fillet_edges` (additive candidate #5):** after the cylinder-convex,
  cylinder-concave, stepped-shaft and cone candidates, try `nblend::sphere_fillet_edge`,
  gated by the SAME `blendResultVerified` SHRINK self-verify (0 < Vr < Vo). Sphere faces
  exist only on native bodies of revolution, so an OCCT body never reaches this arm and the
  cylinder/cone cases are handled by the earlier candidates; there is no dead code.

## Honest scope / declines (→ OCCT)

- Ring-torus guard: the ball-centre circle radius must be ≥ r (a shallow cap whose centre
  circle is smaller than the tube radius) → NULL.
- The sphere-wall seam latitude must stay strictly below the cap (a full hemisphere edge case
  where the seam meets the equator, or a cap so near the pole the seam leaves the wall) →
  NULL.
- The body must be a PURE truncated ball: a cylinder / cone (owned by their arms), a body
  mixing a sphere with a cylinder/cone, a tilted cap, a concave sphere rim, a two-cap
  spherical zone, or any freeform face → NULL → OCCT.
- Multiple picked edges / zero radius → NULL.
- The sphere arm only ever sees bodies the native `solid_revolve_profile` actually built; a
  few specific arc-cap `(R, zc)` combinations the native revolve declines to construct (a
  pre-existing revolve scope limit, unrelated to the fillet) never reach this arm — the sim
  gate therefore exercises dome params inside the supported revolve envelope.

## Impact

- The landed blend substrate (`fillet_edges.h`, the cylinder/concave/variable/cone arms of
  `curved_fillet.h`), M0/M1/M2 and the byte-frozen boolean welds remain **untouched**; the
  sphere arm is an additive sibling of the cone arm.
- `src/native/**` stays OCCT-free (descriptive comments only).
- `cc_*` ABI unchanged (native path only, behind the existing `fillet_edges` facade seam).
- The tessellator (`src/native/tessellate/`) is consumed as-is, UNTOUCHED.
- Readiness: `cc_fillet_edges` moves the analytic sphere cap-rim envelope from OCCT-forward
  to native; the freeform / concave-sphere / spindle residual stays OCCT.
