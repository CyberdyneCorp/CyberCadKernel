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
- ✅ Blends — planar chamfer, constant-radius planar-dihedral fillet, offset-face, shell.
- ✅ **STEP export** (native AP203).
- ✅ **STEP import — native slice, now WIDENED** (OCCT-free Part-21 reader for the elementary/B-spline
  AP203 subset the native writer emits + foreign OCCT-written box/cylinder; healed via the healing
  slice, self-verified watertight else → OCCT). Host round-trip exact + sim OCCT parity, now 33/33.
  Widened along four honestly-gated tracks: **multi-solid** files (>1 `MANIFOLD_SOLID_BREP`, no
  transform tree) import as a native `Compound` of watertight solids (rel 2.14e-16 vs OCCT re-import);
  a native **B-spline-FACE** solid round-trips native-export→import EXACT (the deferred bspline-face
  round-trip, closed on the existing `build_prism_profile_spline` op — not a fabricated fixture);
  the reader recognises + maps the **ELLIPSE** curve entity to the native ellipse edge kind; and
  a single-level **RIGID PLACED ASSEMBLY** (transform tree via `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`
  → `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION` → `ITEM_DEFINED_TRANSFORMATION` AXIS2 pair)
  imports as a native PLACED `Compound` — each component's rigid placement composed and applied via
  the native topology `Location` (`isRigid` det≈+1 orthonormal gate, every root brep placed exactly
  once else NULL). Verified vs OCCT on a 2-box assembly: 2 solids, vol rel 3.74e-16, bbox Δ=0, faces
  12/12. The reader is **schema-independent** (enters at `DATA;`, never gates on `FILE_SCHEMA`), so
  AP203/AP214/AP242 headers all import — confirmed on a real OCCT-authored AP214 file.
  **Residual → OCCT** (honest): PMI/GD&T, non-rigid/scaled/mirrored transforms, deep-nested
  (multi-level) assemblies, `TOROIDAL_SURFACE` (no native torus surface kind), ellipse-bearing
  solids whose ellipse lies on a quadric (fails the watertight self-verify → whole solid falls back),
  complex/trimmed profiles, rational/weighted B-splines, non-mm units;
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
- ✅ **SSI S4-e FIRST CHART-SINGULARITY SLICE (sphere pole + cone apex)** — where ONE
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
  pole/apex that will not verify on both surfaces DEFERS → OCCT — no crossing fabricated. Only
  the sphere-pole + cone-apex chart singularities are crossed; general/freeform parametric
  degeneracies and higher-order/curve cusps remain the tail.
- ◐ **SSI S5-a/b/c/d (curved-boolean slices)** — the SSI-curve-driven
  split→classify→weld pipeline (`src/native/boolean/ssi_boolean.{h,cpp}`, consumes the
  S3 `TraceSet` — and, for S5-d, the S4-d branched re-trace) produces **ten native
  curved-boolean sub-cases verified vs OCCT `BRepAlgoAPI_{Fuse,Cut,Common}`**: the
  through-drill cylinder∩cylinder COMMON (S5-a) + FUSE + CUT (S5-b), the sphere∩sphere op-set
  now COMPLETE 3/3 native — COMMON + FUSE + CUT (S5-c, equal + unequal radii), and the
  **branched-trace Steinmetz bicylinder op-set now COMPLETE 3/3 native — COMMON + FUSE + CUT
  (S5-d)** — all watertight, ΔV ≤ 9e-04 (sim `native-pass=12`). **S5-c FUSE/CUT** reuse one
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
  vs OCCT 13.516 (ΔV = 7.22e-04) — all inside the 1% bar, no tolerance weakened. General
  (non-Steinmetz) branched pairs, other curved-curved families, and non-Steinmetz near-tangent
  pairs still decline to OCCT — honest, measured fallbacks.
- ✅ **Curved blend #6 FIRST SLICE (constant-radius rolling-ball fillet on a CIRCULAR crease)** —
  the rim where a CYLINDER lateral face meets a coaxial PLANAR cap. A ball of radius `r` rolled
  into that convex circular crease traces a **TORUS canal** (major `R = Rc − r`, minor `r`); the
  native builder (`src/native/blend/curved_fillet.h`, OCCT-free) trims the wall + cap to the two
  analytic tangent circles, inserts the quarter-tube torus patch, and rebuilds the whole filleted
  solid as one deflection-bounded planar-facet soup welded watertight via the boolean
  `assembleSolid`. **G1-tangent** at both seams by construction (torus normal is radial at the wall
  seam `v=0`, axial at the cap seam `v=π/2`). Engine self-verify (watertight + `0 < Vr < Vo`);
  verified vs OCCT `BRepFilletAPI` (sim `run-sim-native-curved-fillet.sh` **9/9**, `activeNative=1`,
  vol rel ≤ 3.8e-3, area rel ≤ 2.1e-3). Requires `Rc ≥ 2r` (ring torus). CONCAVE rims, VARIABLE
  radius, cyl↔cyl / cyl↔cone canals, NON-circular creases, multi-edge → OCCT.
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
  branch-point slice + the S4-e sphere-pole/cone-apex chart-singularity slice + the S4-f robust
  true-return closure / self-intersection guard / adaptive completeness critic (measured recall
  wins on small-loop / many-loop fixtures, honest asymptotic floor + residual) + the S5-d branched
  Steinmetz COMMON boolean already landed, and the sphere∩sphere op-set is now COMPLETE 3/3
  native) → **wider S5
  curved booleans** (Steinmetz fuse/cut, general non-Steinmetz branched pairs,
  more families, consuming the S3 WLines + the S4 typed regions/contacts + multi-arm branch loci).
- ☐ General curved **booleans** & **blends** beyond the first slices (sit on SSI): CONCAVE / VARIABLE /
  non-circular-crease / cyl↔cyl-canal fillets; general curved **wrap-emboss** (deboss, non-rectangular
  profiles, non-cylindrical base, >2π footprints). _(The circular cyl↔cap fillet #6 and the rectangular
  pad-on-cylinder emboss #7 first slices are now native — see above.)_
- ☐ Non-planar/guided/rail sweep robustness; general loft; fine-pitch threads.
- ☐ **Shape healing residual** (beyond-tolerance gap bridging, missing-pcurve reconstruction,
  self-intersecting-wire repair, arbitrary broken industrial B-rep — the coincident-within-tolerance /
  degenerate / orientation first slice is now native, above); **full STEP import** beyond the native
  subset (PMI, non-rigid/scaled transforms, deep-nested assemblies, complex/trimmed profiles, torus → OCCT —
  the native slices landed incl. rigid placed assemblies + AP214/AP242 headers, above),
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
