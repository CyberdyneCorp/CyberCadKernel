# Proposal — moat-gs1c-curved-hlr (curved-silhouette HLR breadth: cone/frustum + torus)

## Why

The landed GS1 curved-HLR slice (`moat-hlrc-curved-silhouette`) traces the
closed-form silhouette of a **cylinder** and a **sphere** face and feeds it
through the shared occlusion + visibility-split path, matching OCCT
`HLRBRep_Algo`. Every other analytic curved family — **cone / cone-frustum** and
**torus** — was DECLINED with a single blanket guard in
`NativeEngine::hlr_project`: any face `kind` other than Plane/Cylinder/Sphere
returned an error and forced the body back to OCCT.

That guard is correct but blunt, and it is the last geometry-services residual
gating OCCT-free 2D drawings for the most common turned/rolled mechanical
primitives (tapers, chamfered shafts, O-ring grooves). Two of the declined
families have a **closed-form** orthographic silhouette:

- **Cone / cone-frustum** (apex `O`, half-angle `α`): the lateral silhouette is
  the **two straight contour generators** (rulings) at the angles `u*` where the
  tilted surface normal `n(u) = cosα·(cos u·X + sin u·Y) − sinα·Z` is
  perpendicular to `viewDir`. Each ruling is a straight segment over the
  frustum's axial trim — structurally identical to a cylinder generator, so it
  flows through the existing path unchanged.
- **Torus** (major `R`, minor `r`): the silhouette is the **turning-point
  contour** where `n(u,v) ⟂ viewDir`. Per major angle `u` the silhouette minor
  angle solves `cos v·P(u) + sin v·Zd = 0` (`P(u)=cos u·Xd + sin u·Yd`), giving
  two closed turning contours (outer + inner limbs) traced by sweeping `u`.

The remaining case, a **freeform** (B-spline / Bézier) face, has no closed-form
silhouette and stays HONESTLY DECLINED. Critically, a native `cc_solid_revolve`
of an off-axis arc builds a torus as **rational-B-spline surface-of-revolution
bands** (`Kind::BSpline`), NOT a `Kind::Torus` face — so a revolve-built torus
still declines with a SHARPENED reason, while a `Kind::Torus` face (STEP-imported)
is traced analytically.

## What changes

- Extend `src/native/drafting/silhouette.h` with two additive OCCT-free
  generators: `coneSilhouette` (two straight rulings) and `torusSilhouette` (two
  closed turning contours), each with an honest-decline path (axis-parallel /
  end-on / axis-view-degenerate).
- Widen the `NativeEngine::hlr_project` face-kind gate to accept `Kind::Cone` and
  `Kind::Torus`; compose their traced silhouettes into the same occlusion +
  visibility-split pass as cylinder/sphere. Freeform (BSpline/Bezier) keeps a
  first-class decline with a sharpened reason (covers revolve-built tori).
- Extend `analyticFaceNormal` for Cone/Torus so the smooth-edge suppression that
  the curved-body path relies on works for these bodies.

## Impact

- `src/native/drafting/silhouette.h`, `native_drafting.h` — additive only.
- `src/engine/native/native_engine.cpp` `hlr_project` / `analyticFaceNormal` —
  the curved augmentation branch; polyhedral (planar-only) inputs never enter it,
  so their classification stays BYTE-IDENTICAL.
- `cc_*` ABI unchanged (no new symbols). Cylinder/sphere silhouettes byte-frozen.
- Tests: host-analytic gate (a) extended with cone/frustum + torus closed-form
  scenarios; sim gate (b) extended with cone/frustum parity vs `HLRBRep_Algo` and
  a sharpened torus-B-spline decline.
