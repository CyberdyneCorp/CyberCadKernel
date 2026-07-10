# Proposal — moat-f5-freeform-wrap-emboss (F5, freeform-base wrap_emboss)

## Why

`cc_wrap_emboss` @6 wraps a 2D pattern onto a base surface and raises/recesses it. The
landed native slice (`add-native-wrap-emboss`) handles only a CYLINDER lateral base: a
cylinder is DEVELOPABLE, so the arc-length wrapping map preserves area and the embossed
volume is exactly `footprint-area × height`. The app also wraps patterns onto CURVED
(freeform, non-cylinder) bases; those declined to OCCT — and OCCT's own `wrap_emboss`
(`occt_wrap_emboss.cpp`) ALSO declines a non-cylindrical face (it errors unless
`GeomAbs_Cylinder`), so a curved-base wrap is unserved by either path today.

A sphere base is NON-developable: no arc-length map both tiles the wall and keeps a raised
region's volume equal to `area × height`, so the cylinder gate cannot be reused. But the
axisymmetric POLE-CAP case is exactly analytic: raising a circular pole cap of a sphere-cap
dome radially from `R` to `R+height` over the polar-angle window `φ ∈ [0, φ0]` produces a
SPHERICAL-SHELL SECTOR whose volume delta is the closed form
`ΔV = 2π(1 − cos φ0)·((R+height)³ − R³)/3` (solid angle × radial shell). That is watertight
AND analytically verifiable — the same two-gate rigor as the cylinder arm, on a genuinely
curved base.

## What changes

- **`src/native/feature/wrap_emboss.h` (additive):** add a fourth native arm — a RAISED
  pole cap on a sphere-cap dome (`boss=1`). New OCCT-free `detail::sphereDome` recognises a
  PURE sphere-cap dome WHOLESALE (every face a coaxial sphere of the same centre/R, or
  EXACTLY ONE axis-normal cap that cuts the ball — a spherical zone / off-centre /
  multi-radius sphere / cylinder / cone / general spline base declines). The pole-cap
  half-angle `φ0 = ρ/R` is derived from the profile's arc-length in-radius `ρ` (the footprint
  is the axisymmetric pole DISC, which keeps the volume EXACT). `detail::buildSpherePoleBoss`
  emits a deflection-bounded planar-facet soup — base dome wall (cap latitude → φ0),
  boss outer spherical cap at `R+height`, an annular rim frustum `R→R+height` along the φ0
  circle, and the flat disc cap — sharing N longitude samples across the rim seam so it welds
  watertight through the existing `nb::assembleSolid`. The tessellator is UNTOUCHED. A new
  `spherePoleBossVolumeDelta` exposes the closed form for the engine self-verify and gates.
- **`src/engine/native/native_engine.cpp` (additive):** `wrapEmbossVerified` gains the picked
  `faceId` and, for a recognised sphere-cap base (`boss=1`), gates against the spherical-
  shell-sector closed form instead of the developable `area × height` rule; the verify also
  now requires `tess::isConsistentlyOriented` (the landed orientation invariant) before
  trusting the signed volume. The cylinder path is byte-identical.
- **General freeform / cone / B-spline base: honest-declined (measured).** A general
  free-form (spline-surface) base has no analytic arc-length map with a closed-form embossed
  volume, and a raised pattern that FOLLOWS a spline wall needs the freeform-surface
  parametrization + a new curved-annulus tessellator weld — beyond the no-tessellator-change
  discipline. A CONE base is developable but "raise radially by height" offsets along the
  tilted normal, so the raised patch area ≠ `footprint × height` and the exact volume needs a
  per-family cone-shell-sector builder; not landed this stage. Both decline → NULL → OCCT
  (which itself declines a non-cylindrical wrap, so no wrong solid is ever forwarded).
- Regression tests + two-gate proof (below); no `cc_*` ABI change; `src/native/**` stays
  OCCT-free.

## Two-gate proof

- **Gate (a) — host, no OCCT** (`tests/native/test_native_wrap_emboss.cpp`):
  - `wrap_emboss_sphere_pole_boss_watertight_closed_form` — a hemisphere (R=10) pole boss
    (ρ=3 → φ0=0.3, height=2) is watertight and its meshed volume matches
    `v0 + 2π(1−cosφ0)·((R+h)³−R³)/3` to the deflection band (< 1.5%).
  - `wrap_emboss_sphere_pole_boss_delta_helper_matches` — the exposed closed-form helper
    equals the independent formula to 1e-9 on a deep dome.
  - `wrap_emboss_sphere_scope_defers` — a sphere DEBOSS, a rim-reaching φ0, and a spherical
    ZONE (two caps) all return NULL (→ OCCT).
- **Gate (b) — sim native-vs-OCCT on the booted iOS simulator**
  (`tests/sim/native_wrap_emboss_parity.mm`, three sphere domes — hemisphere, deep dome
  cap=−2, shallow cap=+1): each native pole boss is (1) watertight with its native volume
  matching the closed form (measured rel 1.4e-3–2.2e-3), (2) confirmed to be work OCCT's
  `cc_wrap_emboss` DECLINES on the sphere wall (the honest OCCT-path reference), and (3)
  compared to `oBase + ΔV` where `oBase` is the dome volume measured by OCCT `BRepGProp`
  (`cc_mass_properties` under the OCCT engine) — matching within 2% (measured 2.6e-3–4.3e-3).
  All 23 assertions (14 cylinder + 9 sphere) pass.

## Impact

- Affected specs: `native-wrap-emboss` (sphere-cap pole-boss curved-base arm; curved-base
  self-verify closed form).
- Affected code: `src/native/feature/wrap_emboss.h`, `src/engine/native/native_engine.cpp`
  (additive only). Tessellator UNTOUCHED. No `cc_*` ABI change.
