# Proposal — moat-m3cs-curved-shell (MOAT M3, curved-wall shell)

## Why

`cc_shell` is a Class-A residual M3 curved gap (@12 sites, readiness). The landed native
shell (`shell.h`) hollows only a CONVEX PLANAR solid (a box / convex prism) via a
half-space cavity + planar BSP-CSG cut; any curved body declines → OCCT
`BRepOffsetAPI_MakeThickSolid`. The app hits this whenever it shells a turned part — a cup,
a sleeve, a tapered bushing — i.e. a capped cylinder or capped cone frustum, produced via
`cc_solid_extrude_profile` (full-circle) or `cc_solid_revolve`.

A uniform-thickness hollow of a capped cylinder / cone frustum is exactly analytic: the
curved wall offsets inward to a coaxial inner cylinder / cone (cylinder radius Rc→Rc−t; cone
reference radius →−t/cosσ, the perpendicular inward offset of the tilted wall), the kept
planar cap offsets inward by t, and the removed cap stays flush (the opening). The wall
volume then has a **closed form** (an exact host arbiter) and OCCT `MakeThickSolid` is the
sim oracle. Only OCCT served this before; there is no reason to keep the LGPL engine on the
critical path for the common turned-part shell.

## What changes

- **`src/native/blend/curved_shell.h` (new, additive):** `detail::recogniseShellBody`
  (wholesale capped-cylinder / capped-frustum recognizer — every face a single coaxial
  cylinder/cone + axis-normal planar caps at exactly two heights), `detail::removedCapHeight`
  (the picked cap must be EXACTLY one of the two caps), `detail::buildCurvedShell` (rebuild
  the hollow tube — outer wall, kept cap outer disk, inner offset wall, kept cap inner disk,
  open-end rim annulus — as a deflection-bounded planar-facet soup sharing the same N angular
  samples across every seam), and the public entry
  `blend::curved_shell(solid, faceIds, faceCount, thickness, deflection)`. Welds watertight
  through the existing `assembleSolid` — **no tessellator change**.
- **`NativeEngine::shell` (additive candidate #2):** after the planar convex-shell
  candidate, try `nblend::curved_shell`, gated by the SAME `blendResultVerified` SHRINK
  self-verify (0 < Vr < Vo). Cone faces exist only on native bodies of revolution, so an OCCT
  body never reaches this arm; the planar case is handled by the earlier candidate — no dead
  code.

## Honest scope / declines (→ OCCT)

- The body must be a PURE capped cylinder or capped cone frustum (one coaxial curved wall +
  two axis-normal planar caps). A stepped shaft / multi-cylinder / multi-frustum, a sphere,
  a freeform wall, or a tilted (non-perpendicular) cap → NULL.
- EXACTLY ONE cap must be removed (opened). Removing both caps, removing the curved wall
  (a picked non-planar face), or removing zero faces → NULL.
- The thickness must leave a positive cavity: inner radius > 0 AND wall height > t → NULL
  otherwise (a wall thicker than the body).

## Impact

- The landed blend substrate (`shell.h` planar arm, `curved_fillet.h`), M0/M1/M2 and the
  byte-frozen boolean welds remain **untouched**; the curved shell is an additive sibling of
  the planar shell, reusing the `curved_fillet.h` cylinder/cone face recognisers.
- `src/native/**` stays OCCT-free (descriptive comments only).
- `cc_*` ABI unchanged (native path only, behind the existing `cc_shell` facade seam).
- The tessellator (`src/native/tessellate/`) is consumed as-is, UNTOUCHED.
- Readiness: `cc_shell` moves the analytic capped-cylinder / cone-frustum shell envelope from
  OCCT-forward to native; the non-convex-planar / stepped / sphere / freeform residual stays
  OCCT.
