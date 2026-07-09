# native-drafting

## ADDED Requirements

### Requirement: Analytic-face silhouette tracing for orthographic HLR (cone/frustum + torus)

The native drafting service SHALL trace the **closed-form silhouette locus** of a
**cone** (including a cone-frustum) and a **torus** analytic face — the curve on
the face where the outward surface normal is perpendicular to the view direction
(`n · viewDir = 0`) — and emit it as first-class world-space projected polyline(s)
that are fed through the SAME occlusion + visibility-split path
(`projectOrthographic`) as topological edges and the previously landed
cylinder/sphere silhouettes, so the visible/hidden classification is reused
verbatim. This SHALL be provided by additive OCCT-FREE helpers in
`src/native/drafting/silhouette.h` (`cybercad::native::drafting`, header-only,
including ONLY `native/math`); the polyhedral `orthographic_hlr.h` code path, the
cylinder/sphere silhouette generators, and their parameters SHALL remain
BYTE-IDENTICAL.

- For a **cone / cone-frustum** (axis frame `Ax3`, reference radius `refR` at
  axial height `h=0`, half-angle `α`, axial-height trim `[hMin,hMax]`) the
  silhouette SHALL be the **two straight contour generators** (rulings) at the
  angles `u* = φ ± acos((sinα·Zd)/(cosα·hypot(Xd,Yd)))` where `φ = atan2(Yd, Xd)`
  and `Xd,Yd,Zd` are the view components in the frame; each ruling SHALL be the
  straight segment between the rim points at `hMin` and `hMax` with rim radius
  `refR + h·tanα`.
- For a **torus** (`Kind::Torus` face; axis frame, major radius `R`, minor radius
  `r`) the silhouette SHALL be the **two closed turning-point contours** whose
  minor angle per major angle `u` is `v*(u) = atan2(−P(u), Zd)` and `v*(u)+π`,
  with `P(u) = cos u·Xd + sin u·Yd`, sampled over `u ∈ [0,2π)` to a
  chord-sagitta deflection bound.

The helpers SHALL return an honest "not traceable" status (`traced=false`) — and
the request SHALL DECLINE, NEVER emitting a guessed or clipped polyline — for: a
**cone view parallel to the axis** (whole side is silhouette, no isolated
generator); a **cone view end-on** (`|(sinα·Zd)/(cosα·hypot(Xd,Yd))| > 1`, no
lateral ruling ⟂ view exists); a **torus view down the axis** (the turning
contour degenerates onto the rim circles); and any **freeform** (`BSpline` /
`Bézier`) face. A **native REVOLVE builds a torus as rational-B-spline bands
(`Kind::BSpline`), NOT a `Kind::Torus` face**, so a revolve-built torus SHALL
decline via the freeform path with a sharpened reason; a `Kind::Torus` face
(STEP-imported) SHALL be traced. The service SHALL NEVER emit mesh-edge segments
approximating a curved outline in place of the closed-form silhouette.

#### Scenario: A cone-frustum yields two straight contour generators on the surface (host, no OCCT)

- GIVEN a cone-frustum (reference radius `refR`, half-angle `α`, axial trim `[hMin,hMax]`) viewed obliquely (view neither parallel to the axis nor end-on)
- WHEN its silhouette is traced
- THEN the result SHALL contain EXACTLY two STRAIGHT rulings (2 points each), every point SHALL lie ON the cone surface within `1e-12`, AND the outward analytic cone normal at each point SHALL satisfy `n · viewDir = 0` within `1e-12`

#### Scenario: A torus yields two turning-point contours on the surface (host, no OCCT)

- GIVEN a ring torus (major `R`, minor `r`, `R > r`) viewed obliquely (view not parallel to the axis)
- WHEN its silhouette is traced
- THEN the result SHALL contain EXACTLY two CLOSED contours, every point SHALL lie ON the torus surface within `1e-9`, AND the outward analytic torus normal at each point SHALL satisfy `n · viewDir = 0` within `1e-9`

#### Scenario: A cone seen end-on or along its axis is declined, not fabricated (host, no OCCT)

- GIVEN a cone viewed parallel to its axis, OR viewed end-on so no lateral contour is perpendicular to the view
- WHEN its silhouette is traced
- THEN the helper SHALL return `traced=false` with a decline reason AND SHALL NOT emit any ruling

#### Scenario: A revolve-built torus (B-spline bands) declines with a sharpened reason (native engine)

- GIVEN a torus built by `cc_solid_revolve` of an off-axis arc (its faces are `Kind::BSpline` surface-of-revolution bands, not `Kind::Torus`)
- WHEN `cc_hlr_project` runs under the native engine
- THEN it SHALL DECLINE with an error naming the freeform/B-spline reason (and that a revolve builds a torus as B-spline bands) AND SHALL return an empty drawing — never a fabricated outline

#### Scenario: A cone/frustum drawing matches the OCCT HLRBRep_Algo oracle (sim gate b)

- GIVEN a full cone and a cone-frustum built identically under both engines and an oblique view
- WHEN `cc_hlr_project` runs under the native engine and under the OCCT engine
- THEN the native visible/hidden segment sets SHALL match the oracle on segment COUNT, total visible/hidden LENGTH within a curve-sized relative band, AND every native visible segment SHALL lie on an oracle visible segment (no misclassification / no fabrication) within a curve-sized geometric tolerance

## MODIFIED Requirements

### Requirement: Honest decline for cases the native path cannot classify correctly

The drafting service SHALL return an honest decline (an error / unsupported
status) — rather than emit a possibly-wrong drawing — for any configuration it
cannot robustly classify. The service TRACES the closed-form silhouette of a
**cylinder**, **sphere**, **cone/frustum**, and **torus** (`Kind::Torus`) face
(emitted through the shared occlusion/split path), and the DECLINED cases SHALL
include: any **freeform** (B-spline/Bézier/NURBS) face — INCLUDING a
**revolve-built torus**, whose faces are rational-B-spline surface-of-revolution
bands (`Kind::BSpline`), declined with a sharpened reason; a **partial/trimmed
quadric** whose silhouette generator leaves the face's `(u,v)` trim window
(ambiguous outline); an **axis-parallel-degenerate** cylinder/cone view within
the near-threshold band; a **cone seen end-on** (no lateral ruling ⟂ view); a
**torus viewed down its axis** (turning contour degenerates to the rim circles);
a **view direction along an edge** (degenerate projection); and any sample whose
occlusion is within tolerance of ambiguous (grazing / coincident faces). A
decline SHALL NEVER be silently converted into a visible-or-hidden guess or a
fabricated/mesh-edge outline, and accepting an input SHALL imply its
classification is correct to the stated tolerances.

#### Scenario: A freeform face is declined, never approximated

- GIVEN a solid carrying a B-spline / Bézier face (including a revolve-built torus, whose faces are B-spline surface-of-revolution bands)
- WHEN `cc_hlr_project` runs under the native engine
- THEN it SHALL DECLINE with an error and return an empty drawing, NEVER a mesh-edge or guessed outline

#### Scenario: A degenerate curved view is declined, not fabricated

- GIVEN a cylinder/cone viewed parallel to its axis, a cone viewed end-on, or a torus viewed down its axis
- WHEN its silhouette is traced
- THEN the service SHALL DECLINE (empty drawing / `traced=false`) rather than emit a near-degenerate or fabricated outline
