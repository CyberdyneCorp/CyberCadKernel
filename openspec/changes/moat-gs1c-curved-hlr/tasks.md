# Tasks — moat-gs1c-curved-hlr

## 1. Native silhouette generators (OCCT-free)
- [x] 1.1 Add `coneSilhouette(frame, refRadius, semiAngle, hMin, hMax, viewDir)` to
      `src/native/drafting/silhouette.h`: solve `cosα·(cos u·Xd + sin u·Yd) = sinα·Zd`
      for the two rulings; emit each as a 2-point polyline over the axial-height trim.
- [x] 1.2 Add cone honest declines: view parallel to axis; view end-on (`|rhs| > 1`).
- [x] 1.3 Add `torusSilhouette(frame, majorRadius, minorRadius, viewDir, deflection)`:
      sweep major angle `u`, solve `v* = atan2(−P(u), Zd)` (+π branch) for the two
      turning contours; discretize to the chord-sagitta bound.
- [x] 1.4 Add torus honest decline: view down the axis (contour degenerates to rims).
- [x] 1.5 Update `native_drafting.h` umbrella comment to the extended scope.

## 2. Engine dispatch
- [x] 2.1 Widen the `hlr_project` face-kind gate to accept `Kind::Cone` and
      `Kind::Torus`; keep BSpline/Bezier declining with a SHARPENED reason (covers
      revolve-built tori, which are B-spline bands).
- [x] 2.2 Compose cone/torus silhouettes into the curved-augmentation branch (cone
      uses the axial-height trim; torus is centre-based), reusing the existing
      self-verify-on-mesh + dedup + occlusion/split path.
- [x] 2.3 Extend `analyticFaceNormal` for Cone/Torus (smooth-edge suppression).

## 3. Gate (a) — host analytic (no OCCT)
- [x] 3.1 `test_native_drafting.cpp`: cone-frustum two straight rulings, each ON the
      cone surface (residual < 1e-12) with `n·view = 0` to machine ε.
- [x] 3.2 Cone axis-parallel + end-on declines.
- [x] 3.3 Torus two closed turning contours, each ON the torus surface (residual
      < 1e-9) with `n·view = 0` to machine ε.
- [x] 3.4 Torus axis-view decline.

## 4. Gate (b) — sim native-vs-OCCT parity (`HLRBRep_Algo`)
- [x] 4.1 `native_hlr_parity.mm`: add cone + frustum revolve solids as `runCurved`
      parity cases (counts / length band / visible⊆oracle-visible classification).
- [x] 4.2 Sharpen the decline case to a revolve-built torus (B-spline bands →
      declined), replacing the old cone-decline (cone is now traced).
- [x] 4.3 Runner `run-sim-native-hlr.sh` already links TKHLR — no change needed.

## 5. Docs + finalize
- [x] 5.1 Update `openspec/MOAT-ROADMAP.md` M-GS GS1 status.
- [x] 5.2 Update the `hlr_project` / GS1 curved-HLR rows in
      `openspec/DROP-OCCT-READINESS.md`.
- [x] 5.3 `openspec validate moat-gs1c-curved-hlr --strict`.
- [x] 5.4 Structural check: `git diff src/native` OCCT-free & additive, `cc_*`
      unchanged, landed polyhedral + cyl/sphere HLR byte-frozen.
