# Proposal — moat-f2-sphere-shell (F2, curved-face shell breadth: sphere-cap dome)

## Why

`cc_shell` is the app @12 residual gap (readiness: "convex-planar native; curved/non-convex
→ OCCT"). The landed native shell handles a CONVEX PLANAR solid (`shell.h`) and — since
`moat-m3cs-curved-shell` — a capped CYLINDER / CONE FRUSTUM (`curved_shell.h`). The next
tractable curved substrate the app hits when it shells a turned part is the SPHERE-CAP dome:
a hemisphere / spherical-cap bowl produced via `cc_solid_revolve_profile` (an on-axis arc).
Only OCCT `BRepOffsetAPI_MakeThickSolid` served this before; the sphere was an explicit
decline in `curved_shell.h` and readiness.

A uniform-thickness hollow of a sphere-cap dome is exactly analytic: the OFFSET of a sphere
is a CONCENTRIC sphere, so the inner surface is the sphere of radius Ri = Ro − t centred at
the same point, running from the pole down to the SAME cap plane (a flush opening). The wall
volume then has a **closed form** — the outer spherical segment minus the inner segment
(segment above the plane at axial `a`: `π(2R³/3 − R²a + a³/3)`) — an exact host arbiter, with
OCCT `MakeThickSolid` the sim oracle.

## What changes

- **`src/native/blend/curved_shell.h` (additive):** `detail::recogniseShellSphere` (wholesale
  sphere-cap recognizer — every face a coaxial Sphere of the same centre/radius plus an
  axis-normal Plane at EXACTLY ONE height), `detail::removedSphereCap` (the picked face must
  be exactly the single cap), `detail::buildSphereShell` (rebuild the hollow bowl — outer
  sphere wall pole→cap, concentric inner sphere wall pole→cap with inward normals, open-rim
  annulus at the cap — as a deflection-bounded planar-facet soup sharing N angular samples),
  and a second arm in the public `blend::curved_shell` entry that tries the sphere path when
  the cylinder/cone recognizer declines. Welds watertight through the existing `assembleSolid`
  — **no tessellator change**. Reuses `curved_fillet.h` `detail::sphereInfo` + `ringPoint` +
  `sagittaSteps`.
- **`NativeEngine::shell`:** UNCHANGED — it already calls `nblend::curved_shell` as candidate
  #2, gated by the SHRINK self-verify; the sphere path is subsumed inside that entry.

## Honest scope / declines (→ OCCT)

- The body must be a PURE sphere-cap dome (one coaxial Sphere + EXACTLY ONE axis-normal
  planar cap). A spherical ZONE (two cap planes), an off-centre / multi-radius sphere, a
  tilted cap → NULL.
- EXACTLY the single cap must be opened; picking the sphere WALL or zero faces → NULL (a
  fully-closed hollow sphere has no OCCT oracle through `cc_shell`, which requires a removed
  face, so it is not offered).
- The thickness must leave a positive cavity: Ri = Ro − t > 0 AND the inner sphere must still
  cross the cap plane → NULL otherwise.
- A MIXED planar-and-curved (box-with-a-curved-face) or NON-CONVEX planar body is DECLINED
  (measured): a uniform per-face inward offset self-intersects at reflex/mixed edges and has
  no analytic concentric re-weld here — it stays OCCT-forward.

## Impact

- The landed shell substrate (`shell.h` planar arm, `curved_shell.h` cylinder/cone arm),
  M0/M1/M2 and the byte-frozen boolean welds remain **untouched**; the sphere arm is an
  additive sibling.
- `src/native/**` stays OCCT-free (descriptive comments only).
- `cc_*` ABI unchanged (native path only, behind the existing `cc_shell` facade seam).
- The tessellator (`src/native/tessellate/`) is consumed as-is, UNTOUCHED.
- Readiness: `cc_shell` moves the analytic sphere-cap dome / bowl shell from OCCT-forward to
  native; the non-convex-planar / box-with-curved-face / freeform residual stays OCCT.
