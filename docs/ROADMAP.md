# Roadmap

Phase-level view of the **wrap → accelerate → rewrite** trajectory. The
**canonical, change-level roadmap** (with every OpenSpec change, capability,
contract reference, and GitHub issue) is [openspec/ROADMAP.md](../openspec/ROADMAP.md).

```mermaid
flowchart LR
    P0["Phase 0 · Foundation<br/>wrap OCCT ✅"] --> P1["Phase 1 · Multi-core CPU ✅"]
    P1 --> P2["Phase 2 · GPU / Metal ✅"]
    P2 --> P3["Phase 3 · Features OCCT lacks ✅"]
    P3 --> P4["Phase 4 · Native rewrite<br/>drop OCCT ◐"]

    style P0 fill:#1b5e20,color:#fff,stroke:#333
    style P1 fill:#1b5e20,color:#fff,stroke:#333
    style P2 fill:#1b5e20,color:#fff,stroke:#333
    style P3 fill:#1b5e20,color:#fff,stroke:#333
    style P4 fill:#e65100,color:#fff,stroke:#333
```

Legend: ✅ complete (at the simulator acceptance bar) · ◐ in progress · ☐ planned.

_Status: Phases 0–3 ✅ complete; Phase 4 ◐ in progress (substantially native — see
below). Cumulative delivered ≈ 0.9–1.3 person-years of native kernel work._

## Phase 0 — Foundation ✅
Stand up the library and move CyberCad's OCCT bridge behind the `cc_*` facade,
unchanged in behaviour. Seams: `kernel-facade`, `engine-adapter`,
`operation-scheduler`, `compute-backend`. All 57 `cc_*` verified on the simulator.

## Phase 1 — Multi-core acceleration ✅
Turn on OCCT's existing parallel paths behind the facade — parallel booleans
(`SetRunParallel` + tuned fuzzy), parallel meshing (`InParallel`), bounded worker
pool, fine-thread boolean gate. Determinism audit: serial == parallel, bit-identical.

## Phase 2 — GPU acceleration (Metal) ✅
fp32-tolerant, data-parallel work through the compute backend; CPU stays the
source of truth.
- ✅ **Metal compute backend** — device, unified-memory buffers, runtime MSL, fp32 guard.
- ✅ **GPU tessellation** — surface-eval wired into `cc_tessellate` (per-face, OCCT fallback).
- ✅ **Spatial acceleration** — GPU LBVH + ray-pick **and** frustum-pick verified
  vs CPU on the sim (26/26). *(An app-facing `cc_*` cull entry is an optional
  additive follow-up, not a gate.)*

## Phase 3 — Missing features OCCT lacks ✅
Native geometry OCCT can't do, each behind the same facade.
- ✅ **Reference geometry** — datum planes/axes.
- ✅ **Robust wrap-emboss** (#290) — cap-and-side + healed sew.
- ✅ **Robust thread↔shaft boolean** (#286) — feature-based, no minutes-long hang.
- ✅ **G2 blend fillet** (#284) — measured curvature continuity at straight seams.
- ✅ **Full-round fillet** (#285) — rolling-ball for **all planar dihedrals**
  (parallel + non-parallel, G1-tangent both walls); truly-curved neighbours fall
  back by design.

## Phase 4 — Native rewrite → drop OCCT ◐
Replace the OCCT adapter with native C++20, one capability at a time (opt-in via
`cc_set_engine`, OCCT fallback for the rest), each validated against the OCCT
oracle. **Substantially native for planar/analytic geometry; the curved/general
robustness tail keeps OCCT linked.** Canonical detail:
[../openspec/NATIVE-REWRITE.md](../openspec/NATIVE-REWRITE.md); SSI plan:
[../openspec/SSI-ROADMAP.md](../openspec/SSI-ROADMAP.md).

**Native + verified vs OCCT:**
- ✅ Math/geometry, B-rep topology, watertight tessellation.
- ✅ Construction — extrude, revolve, holed + typed-profile (incl. spline) extrude,
  typed-profile + torus revolve, 2- & N-section loft, straight/planar/non-planar
  (RMF) sweep, tapered-shank, watertight helical/tapered threads.
- ✅ Booleans — planar-polyhedron fuse/cut/common (BSP-CSG, exact) + axis-aligned
  box∩cylinder curved slice.
- ✅ Blends — planar chamfer, constant-radius planar-dihedral fillet, offset-face, shell,
  + CURVED-circular chamfer (convex cylinder↔cap rim → cone-frustum straight bevel, C0).
- ✅ **STEP export** (native AP203).
- ✅ **STEP import — native slice, now WIDENED** (OCCT-free Part-21 reader for the elementary/B-spline
  AP203 subset the native writer emits + foreign OCCT-written box/cylinder; healed via the healing
  slice, self-verified watertight else → OCCT). Host round-trip exact + sim OCCT parity, now 65/65.
  Widened along honestly-gated tracks: **multi-solid** files (>1 `MANIFOLD_SOLID_BREP`, no
  transform tree) import as a native `Compound` of watertight solids (rel 2.14e-16 vs OCCT re-import);
  a native **B-spline-FACE** solid round-trips native-export→import EXACT (the deferred bspline-face
  round-trip, closed on the existing `build_prism_profile_spline` op — not a fabricated fixture);
  the reader recognises + maps the **ELLIPSE** curve entity to the native ellipse edge kind; a
  single-level **RIGID PLACED ASSEMBLY** (transform tree via `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`
  → `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION` → `ITEM_DEFINED_TRANSFORMATION` AXIS2 pair)
  imports as a native PLACED `Compound` — each component's placement composed and applied via
  the native topology `Location` (every root brep placed exactly once else NULL); verified vs OCCT
  on a 2-box assembly: 2 solids, vol rel 3.74e-16, bbox Δ=0, faces 12/12. The rigid `isRigid` gate is
  now a `classifyPlacement()` classifier (`MᵀM ≈ k²·I` conformality + det-sign branch) that also
  admits two new affine placement classes: a **UNIFORM-SCALE** component (via
  `CARTESIAN_TRANSFORMATION_OPERATOR_3D`) scales the placed solid by k³ (k=2 → total vol 2728 =
  1000 + 216·8, component watertight), and a **MIRROR** component (reflection, det<0) is
  orientation-complemented with the existing `topo::Orientation` algebra so the tessellator's
  tangent-derived normal points OUTWARD again — the mirrored solid self-verifies watertight with
  POSITIVE volume 1216. Honesty caveat: OCCT's `STEPControl_Writer` **cannot serialize** a
  scaled/mirror assembly location (it silently drops the scale and rigidifies the mirror to a proper
  rotation; the trimmed iOS OCCT throws "Location with scaling transformation is forbidden"), so
  genuine k³/reflection is verified against an **analytic** oracle via the standard STEP scale/mirror
  operator, while the OCCT-authored fixtures are verified honestly as degrading to rigid
  (native == OCCT). The reader also relaxes the assembly gate for **AP242** files: PMI / GD&T /
  draughting / annotation entities and their angle/plane-angle unit contexts are SKIPPED (a
  representation-relationship that reaches no `MANIFOLD_SOLID_BREP` is skipped, not fatal), so an
  AP242 solid + PMI imports the SOLID identically to OCCT (vol 1000, bbox Δ=0, faces 6/6) instead of
  declining the whole file. The reader is **schema-independent** (enters at `DATA;`, never gates on
  `FILE_SCHEMA`), so AP203/AP214/AP242 headers all import. Two general-surface families are also
  accepted where they reduce to an exact native kind and self-verify: a **`TRIMMED_CURVE` edge**
  unwraps onto the native trimmed edge (a `B_SPLINE_CURVE_WITH_KNOTS` basis honoring its
  `PARAMETER_VALUE` knot-span trims; an analytic `LINE`/`CIRCLE`/`ELLIPSE` basis keeping its exact
  vertex-derived range), and a **`SURFACE_OF_REVOLUTION`** now reduces to the matching native
  analytic quadric per its generatrix: a straight generatrix **∥ axis** → an EXACT native
  `Cylinder`, an **oblique** line meeting the axis → a native `Cone` (apex on axis, half-angle from
  the line-axis angle; native import watertight, vol 522.934 vs OCCT 523.599 rel 1.27e-3, bboxΔ=3e-15),
  a line **⟂ axis** → a native `Plane` annulus cap (native import watertight, vol 1568.8 vs OCCT
  1570.77 rel 1.25e-3) — all three watertight == OCCT re-import within faceting tol; an **on-axis
  circle/arc** → a native `Sphere` reduction now imports NATIVELY WATERTIGHT end-to-end
  (`unblock-native-step-sphere`): OCCT writes a whole sphere as ONE `ADVANCED_FACE` bounded by a
  single `VERTEX_LOOP` (a lone degenerate pole vertex, ZERO seam/pole `EDGE_CURVE`s — a bare periodic
  surface). The reader maps that `VERTEX_LOOP` bound to a native `Sphere` face with a NULL outer wire,
  which the tessellator meshes over its natural (`u∈[0,2π], v∈[−π/2,π/2]`) bounds — welding the
  longitude seam + both poles into a WATERTIGHT `Sphere` solid at OCCT parity. Both the direct
  `SPHERICAL_SURFACE` keyword sphere AND the on-axis-circle `SURFACE_OF_REVOLUTION` sphere flip from
  OCCT fallback to native (sim `native raw parsed=1 watertight=1`, nativeVol 902.31 vs occtVol 904.779
  rel 2.73e-3, area rel 1.37e-3; `[NIMPORT]` 65→69). A `VERTEX_LOOP` bound on any NON-sphere surface,
  or a partial spherical zone carrying a surviving real trim edge that cannot close, keeps the honest
  OCCT deferral. Each
  reduction is gated by a faithful-reduction check (the generatrix must lie on the candidate quadric
  within a scale-relative tol) AND the watertight self-verify — no tolerance weakened, no surface
  fabricated. An **off-axis** circle/arc revolution — and the direct **`TOROIDAL_SURFACE`** keyword —
  now reduces to a NEW native `FaceSurface::Kind::Torus` and imports NATIVELY WATERTIGHT end-to-end
  (`add-native-step-torus`): OCCT writes a whole torus as ONE `ADVANCED_FACE` bounded by a FULLY-SEAMED
  `EDGE_LOOP` (equator v-seam + tube u-seam, each forward AND reversed — no pole, no real trim), which
  the reader maps to a native `Kind::Torus` face with a NULL outer wire and the tessellator meshes over
  its natural (`u,v∈[0,2π]²`) bounds, welding BOTH seams into a WATERTIGHT `Torus` solid (sim `native
  torus` vol 1771.77 vs OCCT 1776.53 rel 2.68e-3, V=2π²Rr², 0 boundary edges). The Torus mesh path is
  strictly ADDITIVE — it reuses the proven sphere bare-periodic path via the all-seam loop detector, so
  `face_mesher.h`/`trim.h` are untouched and every existing mesh is byte-identical. A **PARTIAL/trimmed
  torus** (a `TOROIDAL_SURFACE` carrying a real trim rim), an **ellipse / B-spline** generatrix (general
  revolved surface), and a **skew** oblique line (hyperboloid) stay honest DECLINEs → OCCT.
  **Residual → OCCT** (honest): PMI/GD&T **semantics** (never turned into geometry),
  **non-uniform-scale / shear** transforms, deep-nested
  (multi-level) assemblies, a **PARTIAL/trimmed torus**, ellipse-bearing
  solids whose ellipse lies on a quadric (fails the watertight self-verify → whole solid falls back),
  a `SURFACE_OF_REVOLUTION` with no faithful native kind (an ellipse / B-spline generatrix general
  revolved surface, a skew-line hyperboloid; the full-sphere periodic-pole face and the full torus are
  now NATIVE, above), complex/trimmed surfaces, rational/weighted B-splines,
  non-mm units;
  **all IGES import/export stays OCCT / dropped per the earlier decision.**
- ✅ **Numeric foundations (#2)** — adopted **NumPP + SciPP** (MIT C++20 NumPy/SciPy
  ports) as the OCCT-free numeric substrate + native closest-point (Extrema).
- ✅ **SSI S1** — analytic surface-surface intersection (elementary pairs, closed-form
  curves) vs OCCT `GeomAPI_IntSS`; **SSI S2** — subdivision seeding (transversal
  seeds for freeform/skew-quadric pairs, 100% recall vs OCCT); **SSI S3** —
  marching-line tracer (WLine): full transversal intersection curves traced from the
  S2 seeds vs OCCT `IntPatch` (5 pairs / 9 branches, all fully-traced, 0 near-tangent-
  truncated; onSurf ≤ 6.81e-07, length within the step tol).
- ✅ **SSI S4-a/b (coincident-region + tangent-contact CLASSIFICATION)** — typed
  `CoincidentRegion` (`FullSurfaceSame` / `OverlapSubRegion` / `Undecided`) and typed
  `TangentContact` (`TangentPoint` / `TangentCurve` / `NearTangentTransversal` /
  `Undecided`), on both the analytic and seeded paths, verified vs OCCT
  `IntAna_QuadQuadGeo` / `IntPatch` (8 pairs, 0 failed, 0 deferred; emitted point/curve
  on both surfaces ≤ ~1e-16). DETECTION + CLASSIFICATION only.
- ✅ **SSI S4-c FIRST MARCHING-CORE SLICE (near-tangent MARCH-THROUGH)** — the marcher now
  crosses a `NearTangentTransversal` single-branch graze that S3 truncated, instead of
  stopping: a fixed-plane-cut corrector + curvature-aware predictor + fine deflection-bounded
  step, gated by an honesty-preserving crossable gate. A sphere grazed by an offset cylinder
  that S3 truncated at `tangentSinTol=0.25` now traces the FULL closed loop
  (`nearTangentGaps → 0`, 22 near-tangent nodes crossed, every node on both surfaces ≤ 1e-6,
  crossed arc on the OCCT `GeomAPI_IntSS` locus onCurve ≤ 5.6e-5); the equal-cylinder branch
  saddle and genuine `TangentPoint`/`TangentCurve` contacts STILL defer (`nearTangentCrossed
  = 0`) — no point fabricated past a degeneracy. Deeper near-coincident bands / general-freeform
  singularities + higher-order cusps / self-intersection completeness remain the tail (the
  sphere-pole/cone-apex chart singularities are now crossed — S4-e, below).
- ✅ **SSI S4-d FIRST BRANCH-POINT SLICE (self-crossing locus)** — where the intersection
  locus itself crosses, the marcher now LOCALIZES the branch point (`nn::minimize` the
  transversality sine along the approach, re-projected onto both surfaces), ENUMERATES the
  outgoing arms from the relative second fundamental form's tangent-cone quadratic (real
  distinct roots only — never fabricated), ROUTES each arm with the S3 walk, and ASSEMBLES
  the multi-arm curve. The **Steinmetz bicylinder** (two equal-R orthogonal cylinders) that
  S3+S4-c truncated at the saddle is now **FULLY traced**: 2 branch points localized at
  `(0,±1,0)`, 4 arms → 2 crossing ellipses, `nearTangentGaps = 0`, on the OCCT
  `IntPatch`/`GeomAPI_IntSS` locus onCurve ≤ 1.74e-6 / onSurf ≤ 1.07e-8. The isolated
  `TangentPoint` (spheres d=R₁+R₂) STILL ENDS with zero arms (definite tangent cone ⇒ no
  real roots). Only the elementary transversal self-crossing (Steinmetz family) is handled;
  general/freeform branch points and cusps DEFER → OCCT. **Steinmetz is now unblocked.**
- ✅ **SSI S4-e CHART-SINGULARITY SLICES (analytic sphere pole + cone apex, and the FREEFORM NURBS pole)** — where ONE
  surface's own `(u,v)` parametrization degenerates (`‖dU‖ → 0`) while its 3D point + normal
  stay finite — a **sphere parametric pole** (`v = ±π/2`) or a **cone apex** — the intersection
  can be perfectly transversal yet S3's single-surface Jacobian goes rank-1 and truncates. The
  marcher now DETECTS the collapse from a single-surface `‖dU‖/‖dV‖` witness (distinct from the
  S4-c pair sine and S4-d locus flip), STEPS ACROSS with a point-based fixed-plane cut along the
  last-good tangent `t★` (never touching the degenerate `dU`), and maps the far side back by
  chart continuity (sphere pole → opposite meridian `u+π`; cone apex → far nappe `v→−v`). A
  great circle crossing BOTH sphere poles that S3 truncated at half loop (`len ≈ 3.1415`) is now
  **FULLY traced** (`singX=2`, `nearTangentGaps=0`, closed, `len` native 6.2829 vs OCCT 6.2832,
  on locus + both surfaces ≤ 1.51e-07); a double-cone∩plane line through the apex that S3
  step-collapsed at is **FULLY traced across both nappes** (`singX=1`, 159 nodes, `v∈[−2,+2]`,
  on-surface ≤ 6.79e-16). A genuine finite cylinder `v`-cap still exits as a `BoundaryExit`; any
  pole/apex that will not verify on both surfaces DEFERS → OCCT — no crossing fabricated. A
  **SECOND slice** extends this to the FREEFORM analog: a NURBS unit sphere (a `uPeriod==0`
  collapsed-control-row pole — a genuine freeform degeneracy, no analytic `u+π` meridian map)
  ∩ plane is now FULLY traced by re-seeding the far side with `freeformChartInvert` (a
  point-only far-longitude inversion at the fixed near-pole latitude; the corrector never
  touches the degenerate `dU` or the near-zero normal) — `singX=2`, closed, `len` 6.2829 vs
  OCCT 6.2832, on both surfaces ≤ 1.51e-07. A collapsed-row endpoint on a domain boundary (a
  Bézier cone-tip with no far side) correctly still DEFERS `NearTangent` → OCCT. The **curve
  cusp** (marcher velocity `‖n_A×n_B‖ → 0`) was assessed and DECLINED: by the IFT it coincides
  with the pair-tangency S4-c/S4-d regime, so a standalone S4-e cusp witness would be
  unreachable dead code — cusps route to S4-c/S4-d/OCCT, never faked. Remaining: **asymmetric**
  freeform poles whose continued-tangent re-seed does not verify, higher-order / edge / seam
  degeneracies, and a degenerate-pole B-spline SOLID through the boolean pipeline stay the tail.
- ◐ **SSI S5-a/b/c/d/e/f (curved-boolean slices)** — the SSI-curve-driven
  split→classify→weld pipeline (`src/native/boolean/ssi_boolean.{h,cpp}`, consumes the
  S3 `TraceSet` — and, for S5-d, the S4-d branched re-trace) produces **sixteen native
  curved-boolean sub-cases verified vs OCCT `BRepAlgoAPI_{Fuse,Cut,Common}`**: the
  through-drill cylinder∩cylinder COMMON (S5-a) + FUSE + CUT (S5-b), the sphere∩sphere op-set
  now COMPLETE 3/3 native — COMMON + FUSE + CUT (S5-c, equal + unequal radii), and the
  **branched-trace Steinmetz bicylinder op-set now COMPLETE 3/3 native — COMMON + FUSE + CUT
  (S5-d)**, the **CONE surface family — coaxial cone∩cylinder op-set now COMPLETE 3/3 native —
  COMMON + FUSE + CUT (S5-e)**, and the **CONE∩SPHERE family — coaxial cone∩sphere op-set now
  COMPLETE 3/3 native — COMMON + FUSE + CUT (S5-f)** — all watertight, ΔV ≤ 9e-04 (sim `native-pass=18`). **S5-c FUSE/CUT** reuse one
  generalised `appendSphereCap(outer,reversed)`:
  FUSE (A∪B) = the two OUTER (far-pole) caps welded on the shared seam (`V=V(A)+V(B)−lens`);
  CUT (A−B, order-sensitive) = the OUTER cap of A + the INNER cap of B emitted REVERSED (inward
  normal, bounding the scooped cavity) (`V=V(A)−lens`) — verified vs BOTH the analytic closed
  forms AND OCCT (FUSE ΔV ≤ 8.3e-04, CUT ΔV ≤ 9.3e-04); COMMON byte-identical; tangent/
  containment/concentric pairs decline → NULL → OCCT. **S5-d** turns the S4-d branched
  Steinmetz trace into the native COMMON / FUSE / CUT op-set: a `steinmetzPreGate` + branch-enabled
  re-trace + `recogniseSteinmetzTrace` (2 branch points, 4 `BranchArc` arms) drive the shared
  lune/arc split + `VertexPool` weld; COMMON welds the four inside-the-other lunes, FUSE keeps
  both cylinders' OUTSIDE walls + all four caps (`V=V(A)+V(B)−V(common)`), CUT keeps A's OUTSIDE
  wall + A's caps + B's lunes REVERSED (`V=V(A)−V(common)`). Verified vs **BOTH** the exact analytic
  inclusion-exclusion volumes (host) **and** OCCT (sim): COMMON volN = 5.3287 (analytic `16 R³/3`,
  ΔV = 8.75e-04, −0.088%); FUSE volN = 32.385 vs OCCT 32.366 (ΔV = 5.82e-04); CUT volN = 13.526
  vs OCCT 13.516 (ΔV = 7.22e-04) — all inside the 1% bar, no tolerance weakened. **S5-e** opens
  the CONE family with the coaxial cone(frustum)∩cylinder op-set (COMMON + FUSE + CUT), all sharing
  the SAME S1-analytic circle seam (where `r_c(s)=R0+s·tanα` equals `Rc`, apex-free, `nearTangentGaps=0`)
  resampled to ONE pooled ring. `buildConeCylCommon` welds the two inside-the-other bands + the
  overlap caps; `buildConeCylFuse` (A∪B) keeps the OUTER wall regions of both operands + the union
  caps + annular step caps (`V=V(A)+V(B)−V(A∩B)`); `buildConeCylCut` (A−B, cone minuend, order-
  sensitive) keeps A's outer wall + A's caps + the cylinder's inside-A band emitted REVERSED
  (inward normal bounding the carved cavity — a disconnected solid: detached cone tip + conical
  washer) (`V=V(A)−V(A∩B)`). Verified vs a DUAL oracle — the analytic inclusion-exclusion closed
  form (engine `ssiCurvedBooleanVerified` / `booleanResultVerified`, same 1% tol as the Steinmetz
  `16 R³/3` oracle) AND OCCT `BRepAlgoAPI_{Common,Fuse,Cut}`: COMMON volN = 19.107 vs analytic
  19.111355 vs OCCT 19.111 (ΔV = 2.03e-04, ΔA = 9.89e-05); FUSE volN = 41.618 vs analytic 41.62610
  vs OCCT 41.626 (ΔV = 2.04e-04, ΔA = 1.13e-04, a GROW); CUT volN = 13.349 vs analytic 13.35177 vs
  OCCT 13.352 (ΔV = 2.03e-04, ΔA = 1.02e-04, a SHRINK). **S5-f** extends the CONE family to the
  coaxial cone(frustum)∩sphere op-set (COMMON + FUSE + CUT), all sharing a SINGLE closed S1-analytic
  circle seam (`intersectSphereConeCoaxial`, the sphere centre ON the cone axis on the frustum side
  → exactly one interior crossing `s*` that does NOT cross the apex, `nearTangentGaps=0`,
  `curveCount=1`) resampled to ONE pooled ring by a shared `coneSphereSetup` prologue; the cone side
  reuses the S5-e cone-wall split (`appendRevolvedBand` + `appendDiskCap`), the sphere side the
  sphere-lens cap builder (`appendSphereCap`, inner/outer apex + reversed flags). `buildConeSphere
  Common` welds the cone band inside the sphere + the cone disc + the sphere INNER cap (`V=V_frustum
  + V_spherical-segment`); `buildConeSphereFuse` (A∪B) keeps the sphere OUTER cap + the cone OUTER
  wall + the cone disc (`V=V(A)+V(B)−V(A∩B)`, a GROW); `buildConeSphereCut` (A−B, cone minuend)
  keeps A's outer wall + A's disc(s) + the sphere INNER cap emitted REVERSED (a CONNECTED frustum
  with a spherical dimple, `V=V(A)−V(A∩B)`, a SHRINK). Verified vs a DUAL oracle — the analytic
  inclusion-exclusion `V(A∩B)=V_frustum+V_spherical-segment` (engine `ssiCurvedBooleanVerified` /
  `booleanResultVerified`, same 1% tol) AND OCCT `BRepAlgoAPI_{Common,Fuse,Cut}`: COMMON volN =
  5.2546 vs OCCT 5.2558 (ΔV = 2.41e-04, ΔA = 1.28e-04); FUSE volN = 60.686 vs OCCT 60.718 (ΔV =
  5.22e-04, ΔA = 2.61e-04, a GROW); CUT (cone−sphere) volN = 27.202 vs OCCT 27.207 (ΔV = 1.96e-04,
  ΔA = 1.34e-04, a SHRINK). The sphere-minuend `sphere − cone` CUT, the TWO-circle crossing (sphere
  through the cone / spanning the apex), apex-crossing / apex-in-extent frustums, and TRANSVERSAL
  (non-coaxial) cone∩sphere (a quartic space curve) decline → OCCT. Transversal/apex cone pairs and
  cone∩cone also decline → OCCT (a mis-selected band / mis-oriented reversed fragment fails the
  self-verify and falls back — never faked). General (non-Steinmetz) branched pairs, other
  curved-curved families, and non-Steinmetz near-tangent pairs still decline to OCCT — honest,
  measured fallbacks.
- ✅ **Curved blend #6 (constant-radius rolling-ball fillet on a CIRCULAR crease — CONVEX *and*
  CONCAVE)** — the rim where a CYLINDER lateral face meets a coaxial PLANE. A ball of radius `r`
  rolled into that circular crease traces a **TORUS canal** (minor `r`); the native builder
  (`src/native/blend/curved_fillet.h`, OCCT-free) trims the two faces to the analytic tangent
  circles, inserts the quarter-tube torus patch, and rebuilds the whole filleted solid as one
  deflection-bounded planar-facet soup welded watertight via the boolean `assembleSolid`.
  **G1-tangent** at both seams by construction (torus normal radial at the wall seam `v=0`, axial
  at the plane seam `v=π/2`). Two signs of the ball-centre offset:
  - **CONVEX** cyl↔coaxial-cap rim: ball seats outside the corner, major `R = Rc − r`, REMOVES
    material — engine self-verify `0 < Vr < Vo`; requires `Rc ≥ 2r` (ring torus).
  - **CONCAVE** boss-on-plate base rim: ball seats on the material side, major `R = Rc + r`, ADDS
    material — engine self-verify `Vr > Vo` (`wantGrow=true`, same branch offset-face grow uses).
  Verified vs OCCT `BRepFilletAPI` (sim `run-sim-native-curved-fillet.sh` **15/15**,
  `activeNative=1`: convex 9/9 + concave 6/6 `grew=1`, vol rel ≤ 3.8e-3, area rel ≤ 2.1e-3).
- ✅ **Variable-radius curved fillet (CONVEX circular cyl↔cap rim, LINEAR radius law)** — the same
  convex cylinder↔coaxial-cap rim as #6, but the rolling-ball radius varies LINEARLY around the rim,
  `r(θ)=r1+(r2−r1)·θ/2π`. The centre locus is no longer a fixed-offset circle but a SWEPT curve, so
  the two trim seams are NON-circular (varying-radius) curves; the native builder
  (`src/native/blend/curved_fillet.h`, `variable_fillet_edge`, OCCT-free) sweeps a ring of planar
  facets, each station using the local `r(θ)` upright meridian arc, welded watertight, **G1-tangent
  at both varying-radius seams** by construction (`cos=1.0`). Wired in
  `NativeEngine::fillet_edges_variable` behind `cc_fillet_edges_variable`, gated by the same
  correctly-signed `blendResultVerified(wantGrow=false)` self-verify; when it cannot build robustly
  it returns NULL → OCCT `BRepFilletAPI` (evolved). Verified on two fixtures (Rc=5 r1=1→r2=2; Rc=6
  r1=0.75→r2=2.25): watertight, native volume matches the builder's own closed-form SWEPT removed
  volume (rel ≤ 1.1e-3) and is REDUCED vs the sharp cylinder — and is DISTINCT from the OCCT evolved
  oracle, proving the sim exercises native geometry, not an OCCT fall-through. Native-vs-OCCT-evolved
  parity is reported SEPARATELY as a looser line (rel ≤ 1.2e-2, the expected O(r′) interior gap; the
  upright-meridian canal agrees with OCCT's tilted evolved envelope exactly at both seams and in the
  `r1=r2` limit) — never hidden behind the HARD bound. Sim `run-sim-native-curved-fillet.sh` **23/23**
  (15 constant convex+concave controls unchanged + 8 variable checks); host `test_native_blend`
  22/22.
  Residual → OCCT: NON-LINEAR radius laws, CONCAVE variable rim, cyl↔cyl / cyl↔cone canals,
  NON-circular creases (cone/sphere/ellipse/spline rim), blind-hole bottom rim,
  convex `Rc < 2·rmax`, seam-leaves-face, multi-edge (the convex-circular curved *chamfer* is now native — see #6b below).
- ✅ **Curved-circular chamfer #6b (CONVEX cyl↔cap rim → CONE-FRUSTUM straight bevel)** — a chamfer
  cuts a FLAT bevel, so unlike the #6 fillet (a G1-tangent torus arc) it is a CONE FRUSTUM band
  between the two setback circles (cylinder-wall circle at axial setback = `d`, cap circle at radial
  setback = `d`), meeting each face at the chamfer angle **C0, NOT G1** (asserting tangency would be
  wrong for a chamfer). Native builder `src/native/blend/curved_chamfer.h` (`curved_chamfer_edge`,
  OCCT-free) reuses #6's rim recognition, trims the wall to the cylinder setback circle and the cap
  to the radial setback circle, and fills the band with a single-meridian-step deflection-bounded
  frustum, welded watertight; wired in `NativeEngine::chamfer_edges` as planar → curved-circular →
  OCCT, each gated by the SAME shrink self-verify `blendResultVerified(wantGrow=false)`. Because a
  symmetric chamfer IS EXACTLY a cone frustum, native-vs-OCCT `BRepFilletAPI_MakeChamfer`
  (`Add(distance, edge)`) parity is TIGHT (vol rel ≤ 3.25e-3, angular-deflection-bounded), and the
  native volume matches the exact closed-form Pappus removed volume `π·d²·(Rc − d/3)` (rel ≤ 3.25e-3);
  bevel normal cos=1/√2 to BOTH faces, explicitly ≠1. Sim `run-sim-native-curved-chamfer.sh` **9/9**
  (3 fixtures), host `test_native_blend` chamfer cases green.
  Residual → OCCT: non-circular / asymmetric two-distance / concave / cyl↔cyl (curved↔curved) chamfer.
- ✅ **Wrap-emboss #7 FIRST SLICE (rectangular pad on a cylinder lateral face)** — emboss (`boss=1`)
  a rectangular footprint onto a CYLINDER wall. The native builder (`src/native/feature/wrap_emboss.h`,
  OCCT-free) wraps the footprint by the SAME map the OCCT oracle uses (`u = px/R`, `v = py + vMid`),
  builds the raised pad (wrapped OUTER CAP at `R+height` + two circumferential walls + two axial
  walls) and retiles the base wall with the footprint window removed, welding the whole embossed
  solid watertight via `assembleSolid`. Engine self-verify (watertight + volume GROWS by
  `footprint area × height`); verified vs OCCT `cc_wrap_emboss` (sim `run-sim-native-wrap-emboss.sh`
  **6/6**, `activeNative=1`, vol rel ≤ 2.5e-3, area rel ≤ 7.3e-4). DEBOSS, non-rectangular / >4-corner
  profiles, non-cylindrical base, >2π / off-end footprints → OCCT.
- ✅ **Shape healing FIRST NATIVE SLICE (tolerant sew + vertex/tolerance unification + degenerate
  removal + orientation fix)** — an INTERNAL, OCCT-free healer (`src/native/heal/`, `healShell`) that
  stitches a coincident-within-tolerance face soup into a connected, consistently-oriented,
  WATERTIGHT solid: hash-welds near-coincident vertices (`boolean/assemble.h` `VertexPool`
  generalized), shares an edge only when its endpoints unify to the same two shared vertices within
  tolerance (never a fabricated coincidence), drops zero-length edges + sliver faces, and flood-fills
  outward orientation with a global enclosed-volume-sign tie-break. SELF-VERIFIED (watertight +
  `V > 0`) before it is kept; otherwise returns the input UNCHANGED with a typed `Unhealed` reason +
  measured `maxResidualGap`. Verified vs OCCT `BRepBuilderAPI_Sewing` + `ShapeFix_Shell`/`ShapeFix_Solid`
  (host `test_native_heal` 10/10; sim `run-sim-native-heal.sh` **4/4**): in-scope soup-cube/flipped-face
  heal to V=1 watertight matching OCCT, and the un-healable fixtures (gap 1e-2 ≫ tol → `GapBeyondTolerance`
  residual 0.0255; missing face → `OpenShell`) report UNHEALED HONESTLY, matching OCCT leaving the shell
  open at the same tolerance. Engine hook `tryNativeHeal` → self-verify → OCCT fallback; no `cc_*` change.
  **This is the gating foundation for a future native STEP IMPORT** (imported B-rep arrives with exactly
  these coincident-within-tolerance / degenerate / orientation defects). Beyond-tol gaps, missing
  pcurves, self-intersecting wires, and arbitrary broken industrial B-rep stay UNHEALED → OCCT (honest
  asymptotic residual — a measured win vs OCCT on the in-scope family, not a guarantee).

**Still OCCT-backed (the tail that keeps OCCT linked):**
- ☐ SSI **S4-d general/freeform + S4-e general/freeform + S4-f general topology repair** (the moat
  tail: general/freeform branch points, higher-order/curve cusps, general/freeform parametric
  singularities, watertight self-intersection resolution / topology repair, deeper near-coincident
  bands — S4-a/b classification + the S4-c near-tangent march-through + the S4-d Steinmetz
  branch-point slice + the S4-e sphere-pole/cone-apex + FREEFORM NURBS-pole chart-singularity
  slices (curve cusp declined by IFT; asymmetric/higher-order freeform poles remain) + the S4-f robust
  true-return closure / self-intersection guard / adaptive completeness critic (measured recall
  wins on small-loop / many-loop fixtures, honest asymptotic floor + residual) + the S5-d branched
  Steinmetz COMMON boolean already landed, and the sphere∩sphere op-set is now COMPLETE 3/3
  native) → **wider S5
  curved booleans** (Steinmetz fuse/cut, general non-Steinmetz branched pairs,
  more families, consuming the S3 WLines + the S4 typed regions/contacts + multi-arm branch loci).
- ☐ General curved **booleans** & **blends** beyond the first slices (sit on SSI): NON-LINEAR-law /
  CONCAVE-variable / non-circular-crease / cyl↔cyl-canal fillets, non-circular / asymmetric two-distance /
  concave / cyl↔cyl curved chamfer; general curved
  **wrap-emboss** (deboss, non-rectangular profiles, non-cylindrical base, >2π footprints). _(The
  constant-radius circular cyl↔cap fillet #6 (convex + concave), the VARIABLE-radius (linear-law)
  convex circular cyl↔cap fillet, the convex-circular cyl↔cap CONE-FRUSTUM chamfer #6b, and the
  rectangular pad-on-cylinder emboss #7 first slices are now
  native — see above.)_
- ☐ Non-planar/guided/rail sweep robustness; general loft; fine-pitch threads.
- ☐ **Shape healing residual** (beyond-tolerance gap bridging, missing-pcurve reconstruction,
  self-intersecting-wire repair, arbitrary broken industrial B-rep — the coincident-within-tolerance /
  degenerate / orientation first slice is now native, above); **full STEP import** beyond the native
  subset (PMI SEMANTICS, non-uniform/shear transforms, deep-nested assemblies, a PARTIAL/trimmed torus, a SURFACE_OF_REVOLUTION with no faithful native kind (ellipse/B-spline general revolved surface, skew-line hyperboloid), complex/trimmed surfaces → OCCT —
  the native slices landed incl. rigid/uniform-scale/mirror placed assemblies + AP242 geometry with PMI skipped + TRIMMED_CURVE edges + a SURFACE_OF_REVOLUTION reducing to a native cylinder (line ∥ axis) / cone (oblique line) / plane (perpendicular line) + a FULL SPHERE (SPHERICAL_SURFACE and on-axis-circle SURFACE_OF_REVOLUTION, VERTEX_LOOP periodic-pole face → native watertight Sphere) + a FULL TORUS (TOROIDAL_SURFACE and off-axis-circle SURFACE_OF_REVOLUTION, fully-seamed EDGE_LOOP doubly-periodic face → native watertight Kind::Torus), above),
  and **all IGES import/export** (stays OCCT / dropped per the earlier decision).
- ☐ **`drop-occt`** — BLOCKED until the above are native (research-grade, multi-year).

**Effort:** ≈ 0.9–1.3 py delivered (planar/analytic breadth); ≈ **9–18 py remaining**
to genuinely drop OCCT, concentrated in SSI-S4-d(general)…f marching robustness + healing + import.

---

**Guiding rules**

- The `cc_*` ABI never breaks; additive-only. The app is insulated from every phase.
- Native and OCCT-backed implementations coexist behind the adapter so each
  migration is measured (correctness + performance) before OCCT is retired.
- Changes are proposed (`/opsx:propose`) when a phase is about to start, and their
  delta specs are synced into `openspec/specs/` when validated.
