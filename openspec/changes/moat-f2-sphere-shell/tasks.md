# Tasks — moat-f2-sphere-shell

## 1. Native builder (OCCT-free, additive)
- [x] 1.1 Add `detail::recogniseShellSphere` to `curved_shell.h` — wholesale sphere-cap
  recognizer (every face a coaxial Sphere of the same centre/radius + an axis-normal Plane at
  EXACTLY ONE height), reusing `curved_fillet.h` `detail::sphereInfo`.
- [x] 1.2 Add `detail::removedSphereCap` — the picked face(s) must resolve to EXACTLY the
  single cap (a picked sphere wall / zero faces → decline).
- [x] 1.3 Add `detail::buildSphereShell` — rebuild the hollow bowl (outer sphere wall
  pole→cap, concentric inner sphere wall pole→cap inward, open-rim annulus at the cap) as a
  planar-facet soup sharing N angular samples; positive-cavity guards (Ri > 0, inner sphere
  crosses the cap).
- [x] 1.4 Add the sphere arm to the public `blend::curved_shell` entry (tried when the
  cylinder/cone recognizer declines).

## 2. Engine wiring
- [x] 2.1 No change — `NativeEngine::shell` already calls `nblend::curved_shell` (candidate
  #2) under the `blendResultVerified` SHRINK self-verify; the sphere path is subsumed.

## 3. Gate A — host (analytic, no OCCT)
- [x] 3.1 `hemisphere_shell_open_equator_closed_form` — watertight + closed-form wall volume
  `(2π/3)(Ro³−Ri³)`, shrink.
- [x] 3.2 `spherical_cap_dome_shell_closed_form` — shallow (capOff>0) and deep (capOff<0) caps
  vs the segment-difference closed form.
- [x] 3.3 `sphere_shell_converges` — faceted volume error shrinks monotonically as deflection
  refines.
- [x] 3.4 `sphere_shell_declines_out_of_scope` — sphere wall picked / zero faces / thickness ≥
  radius / spherical ZONE (two caps) all return NULL.

## 4. Gate B — sim (native vs OCCT on the booted simulator)
- [x] 4.1 Extend `tests/sim/native_curved_shell_parity.mm` with `runDomeCase`: build the dome
  via `cc_solid_revolve_profile` (on-axis arc + explicit axis-closing edge so BOTH engines
  build it), open the cap, compare NativeEngine vs OCCT `MakeThickSolid` + `BRepGProp` and the
  closed form (hemisphere, deeper Ro, shallow cap, deep cap).

## 5. Regression / discipline
- [x] 5.1 Full host `ctest` stays all-green (67/67).
- [x] 5.2 `src/native/**` stays OCCT-free; tessellator untouched; `cc_*` ABI additive-only.
- [x] 5.3 `openspec validate moat-f2-sphere-shell --strict` passes.
