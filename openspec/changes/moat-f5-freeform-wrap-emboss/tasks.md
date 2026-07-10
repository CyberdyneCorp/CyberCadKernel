# Tasks — moat-f5-freeform-wrap-emboss

## 1. Diagnose the freeform-base gap
- [x] 1.1 Confirm the landed slice is cylinder-only and OCCT's own `wrap_emboss` declines a non-cylindrical face (so a curved base is unserved by either path).
- [x] 1.2 Establish that a sphere base is non-developable → the `area × height` gate cannot be reused; derive the axisymmetric pole-cap shell-sector closed form `ΔV = 2π(1−cosφ0)·((R+h)³−R³)/3`.

## 2. Implement the sphere-cap pole-boss arm (additive, src/native, OCCT-free)
- [x] 2.1 `detail::sphereDome` — recognise a PURE sphere-cap dome wholesale (coaxial same-centre/R spheres + EXACTLY ONE axis-normal cap that cuts the ball); zone / off-centre / cylinder / cone / spline → nullopt.
- [x] 2.2 `detail::buildSpherePoleBoss` — base dome wall (cap latitude→φ0) + boss outer spherical cap (R+h) + rim frustum (R→R+h) + flat disc cap, sharing N longitude samples so the rim seam welds; tessellator UNTOUCHED.
- [x] 2.3 `spherePoleBossVolumeDelta` — expose the closed form; return 0 out of scope (rim-reaching φ0 / deboss / non-sphere).
- [x] 2.4 Dispatch a `boss=1` sphere-cap base in `wrap_emboss()` before the cylinder decline; a sphere deboss / other freeform → NULL.

## 3. Engine self-verify (additive)
- [x] 3.1 `wrapEmbossVerified` takes the picked `faceId`; for a recognised sphere-cap base (`boss=1`) gate against the shell-sector closed form; require `tess::isConsistentlyOriented`; cylinder path byte-identical.

## 4. Regression tests + two gates
- [x] 4.1 Host gate (a): hemisphere pole boss watertight + matches closed form < 1.5%; helper matches formula to 1e-9; deboss / rim-reaching φ0 / spherical zone decline.
- [x] 4.2 Sim gate (b): three sphere domes — native closed-form volume (< 1.5%), OCCT declines the sphere wrap, native-vs-OCCT-reference-boss (BRepGProp) < 2%.
- [x] 4.3 Full host ctest stays all-green.

## 5. General freeform / cone / B-spline base
- [x] 5.1 Assess tractability without a tessellator weld → HONEST-DECLINE: a general spline base needs the freeform-surface parametrization + a curved-annulus weld; a cone base needs a per-family cone-shell-sector builder (radial offset along the tilted normal ≠ area×height). Not landed; documented as the sharpened next blocker.

## 6. Spec + commit
- [x] 6.1 OpenSpec change dir + `openspec validate --strict`.
- [x] 6.2 Commit to moat-feat5 (gate numbers in the message).
