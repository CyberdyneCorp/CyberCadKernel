# Proposal — moat-f3-cone-sphere-offset (F3, curved offset_face breadth: cone frustum + sphere)

## Why

`cc_offset_face` is the app @10 residual gap. The landed native curved arm
(`moat-m3co-curved-offset-face`, `src/native/blend/curved_offset.h`) offsets the CYLINDER
lateral wall of a capped cylinder RADIALLY (radius Rc → Rc+d) — the offset of a cylinder is a
coaxial cylinder, a pure radius bump. The other two analytic curved walls a turned part
presents at `cc_offset_face` — a CONE-FRUSTUM wall and a SPHERE wall — were an explicit
decline (`curved_offset.h` scope note + readiness), forwarded to OCCT `BRepOffsetAPI`.

Both extend the SAME self-verified re-radius builder, but are no longer a trivial radius bump:

- A **cone-frustum** wall offset by `d` moves to a COAXIAL cone of the SAME semi-angle σ; the
  wall normal is radial tilted by σ, so the radius at every height shifts by `d/cosσ`, i.e.
  `Rref → Rref + d/cosσ` with the cap heights fixed. Volume has the exact frustum closed form
  `πH/3·(Rb'² + Rb'·Rt' + Rt'²)`.
- A **sphere** wall (sphere-cap dome) offset by `d` moves to a CONCENTRIC sphere `R → R+d`;
  the single cap plane stays fixed and the rim radius follows to `√((R+d)²−a²)`. Volume is the
  spherical-segment closed form `π(2R³/3 − R²a + a³/3)` at cap axial coord `a`.

Each is an exact host arbiter; OCCT `BRepPrimAPI_MakeCone` / `MakeSphere` + `BRepGProp` is the
sim oracle (the shipped OCCT `cc_offset_face` is PLANAR-ONLY, so the ground-truth oracle is
built directly, as in the M3co cylinder harness).

## What changes

- **`src/native/blend/curved_offset.h` (additive):**
  - `detail::cappedConeGeom` — wholesale capped-frustum recognizer about a picked Cone lateral
    face (every face a coaxial Cone of the SAME σ/Rref, plus an axis-normal Plane at exactly
    two heights; both original cap radii positive). `detail::buildCappedCone` — rebuild at
    `Rref + d/cosσ` (same σ, same cap heights) as a planar-facet soup (wall band + two disc
    caps, shared N angular samples).
  - `detail::sphereDomeGeom` — wholesale sphere-cap-dome recognizer about a picked Sphere face
    (every face a coaxial Sphere of the same centre/radius, plus EXACTLY ONE distinct
    axis-normal cap plane that cuts the ball). `detail::buildSphereDome` — rebuild the
    concentric dome at `R+d` (same cap plane) as a planar-facet soup (sphere-wall latitude
    bands pole→cap + one disc cap, shared N angular samples).
  - Two new arms in the public `blend::curved_offset_face` entry: after the cylinder arm
    declines, try the cone arm, then the sphere arm. Welds watertight through the existing
    `assembleSolid` — **no tessellator change**. Reuses `curved_fillet.h` `detail::coneInfo` /
    `coneRadiusAtH` / `sphereInfo` / `ringPoint` / `sagittaSteps`.
- **`NativeEngine::offset_face`:** UNCHANGED — it already calls `nblend::curved_offset_face`
  as candidate #2, gated by the correctly-signed `blendResultVerified` self-verify; the cone
  and sphere paths are subsumed inside that entry.

## Honest scope / declines (→ OCCT)

- Cone: the body must be a PURE capped frustum (one coaxial Cone σ/Rref + exactly two
  axis-normal caps). A stepped / multi-cone shaft, a full cone through the apex (a cap radius
  ≤ 0), a shrink that inverts a cap (`Rt + d/cosσ ≤ 0`), a tilted cap → NULL.
- Sphere: the body must be a PURE sphere-cap dome (coaxial spheres of one centre/radius +
  EXACTLY ONE axis-normal cap that cuts the ball). A spherical ZONE (two caps), an off-centre
  / multi-radius sphere, a shrink through the cap plane (`|a| ≥ R+d`), `R + d ≤ 0` → NULL.
- A picked PLANAR cap face stays with the planar `offset_face` arm; a freeform/other curved
  wall → NULL. A native void is NEVER handed to OCCT.

## Impact

- The landed cylinder offset arm (`curved_offset.h`), the planar `offset_face`, M0/M1/M2 and
  the byte-frozen boolean welds remain **untouched**; the cone/sphere arms are additive
  siblings behind the same `cc_offset_face` facade seam.
- `src/native/**` stays OCCT-free (descriptive comments only).
- `cc_*` ABI unchanged (native path only).
- The tessellator (`src/native/tessellate/`) is consumed as-is, UNTOUCHED.
- Readiness: `cc_offset_face` moves the analytic cone-frustum and sphere-cap-dome curved-wall
  offsets from OCCT-forward to native; the freeform / stepped / spherical-zone residual stays
  OCCT.
