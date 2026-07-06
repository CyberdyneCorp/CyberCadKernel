# CyberCadKernel — Phase 4 status (native rewrite, drop OCCT)

Honest, verification-anchored snapshot of Phase 4 — replacing the OCCT adapter
with native C++20, one capability at a time, until OCCT can be unlinked. Method,
verification model, and the full capability sequence live in the sub-roadmap
[`openspec/NATIVE-REWRITE.md`](../openspec/NATIVE-REWRITE.md). Nothing below is
claimed unless it was actually built and run in this environment.

Date: 2026-07-03 · Branch: `main`.

## TL;DR

- **Capability #1 `native-math` — done at the Phase-4 verification bar.** Both
  independent gates are green: host analytic unit tests (no OCCT, no simulator)
  and native-vs-OCCT numeric parity on the booted iOS simulator.
- **Capability #2 `native-topology` — done at the Phase-4 verification bar.**
  B-rep data model + exploration (`TopoDS`/`TopExp`/`BRep_Tool` analogues). Host
  gate green (`test_native_topology`, 13 cases, 0 failed) and native-vs-OCCT
  parity green on the booted iOS simulator (3 shapes × 5 checks = **15 passed,
  0 failed**, max accessor error **0.000e+00**).
- **Capability #3 `native-tessellation` — done at the Phase-4 verification bar.**
  Deflection-driven native mesher (UV-grid face meshing + parameter-space hole
  trimming + solid welding) consuming native-math surface eval and native-topology
  faces. Host gate green (`test_native_tessellate`, 13 cases) and native-vs-OCCT
  `BRepMesh` property-parity green on the booted iOS simulator (**All 20 checks PASS
  across 4 shapes** — box / cylinder / sphere / filleted-box; **ALL four closed solids
  watertight, `boundaryEdges==0`**; area/volume relMesh ≤ **6.0e-3**, relExact ≤
  **1.24e-2**, bbox max corner delta ≤ **4.66e-2**, vertices-on-surface residual ≤
  **5.7e-15**). The curved shared-edge seam (cylinder cap↔side, fillet blends) now
  welds watertight via the two-stage shared-edge mesher.
- **Capability #4 `native-construction` — done at the Phase-4 verification bar
  (core), first engine-wired capability.** Native `cc_solid_extrude` (closed
  polygon → prism) and native `cc_solid_revolve` (LINE-SEGMENT profile → surface of
  revolution) build real native solids and are compared A/B against OCCT through the
  facade. Both gates green: host `test_native_construct` + `test_native_engine`
  (CTest **12/12**) and native-vs-OCCT parity on the iOS sim (**17/17** `[NCONS]`
  checks). Planar prisms are EXACT (vol/area/centroid rel 0.00e+00, identical face
  tiling); curved revolves match within a deflection bound (vol rel ≤ 2.36e-2,
  watertight). Wired behind an ADDITIVE `cc_set_engine` / `cc_active_engine` toggle
  (**default stays OCCT**). **`#4b` Tier A is now also done at the bar** — holed
  (`cc_solid_extrude_holes` / `_polyholes`) and typed-profile
  (`cc_solid_extrude_profile` / `_profile_polyholes` / `cc_solid_revolve_profile`
  for line / arc / on-axis-arc) construction is NATIVE (host CTest 13/13 + sim
  parity 22/22). **Tiers B (2-section ruled loft), C (straight / smooth-planar
  sweep) and D (`cc_tapered_shank` + well-formed `cc_helical_thread` /
  `cc_tapered_thread`) are now also done at the bar** — the thread per-turn seam weld
  is fixed at the mesher level, so well-formed threads mesh `boundaryEdges==0` at every
  deflection and run NATIVE. Still OCCT-fallthrough (not faked): kind-3 SPLINE profile
  edges, off-axis-arc (torus) / spline surface-of-revolution, twisted/guided/rail sweep,
  3+-section / guided / rail loft, and a fine-pitch / self-intersecting thread. Wrap-emboss
  (Tier E) is NO LONGER fully OCCT: its FIRST NATIVE SLICE (a rectangular pad on a cylinder
  lateral face) has since landed native + verified vs OCCT — see the native-wrap-emboss
  result table below (`add-native-wrap-emboss` archived); deboss / non-rectangular /
  non-cylindrical / >2π stay OCCT. The blend family (#6) likewise gained its CURVED
  slices — a constant-radius fillet on a circular cylinder↔plane rim → torus canal, G1-tangent,
  now for BOTH the CONVEX cap rim (major `Rc−r`, removes material) AND the CONCAVE boss-on-plate
  base rim (major `Rc+r`, ADDS material, engine self-verify `wantGrow=true`) — see the
  native-curved-fillet result table below (`add-native-curved-fillet` +
  `add-native-concave-fillet` archived) — and a VARIABLE-radius LINEAR-law convex circular
  cyl↔cap fillet (`cc_fillet_edges_variable`, swept-radius canal, G1 at both varying-radius seams,
  `add-native-variable-fillet` archived). Blind-hole bottom rim, NON-linear law, concave-variable,
  cyl↔cyl canal, non-circular creases and curved-edge chamfer stay OCCT.
- **Capability #5 `native-booleans` — PLANAR-polyhedron slice done at the
  verification bar; curved / general still OCCT-fallthrough (honest).** Native
  `cc_boolean` (fuse / cut / common) for planar-faced solids (axis-aligned boxes,
  prisms) via a BSP-tree CSG (`src/native/boolean/`), guarded by a MANDATORY
  self-verify (`robustlyWatertight` + set-algebra volume) that discards any invalid
  candidate and falls through to OCCT. Both gates green: host `test_native_boolean` +
  `test_native_engine` (CTest **17/17**, no OCCT) and native-vs-OCCT parity on the iOS
  sim (`native_boolean_parity.mm`, **25/25**) — box fuse rel **1.27e-16** / cut
  **2.96e-16** / common **2.22e-16**, contained fuse **0.00e+00** / common **2.22e-16**
  all EXACT + watertight; the self-verify correctly rejects a native∩native disjoint
  out-of-domain result; curved (cyl-box, rel 0.00e+00), near-coincident (0.00e+00) and
  disjoint (0.00e+00) cases fall through to OCCT (delegated, no native interception).
  Booleans remain the longest-lived OCCT dependency for curved / general.
  - **Curved analytic slice (deferred residual #2) — axis-aligned box ⟷ axis-parallel
    cylinder NOW NATIVE at BOTH gates (host + sim parity), archived.** `cc_boolean` cut / fuse / common is native
    when one operand is an axis-aligned box and the other a cylinder whose axis ∥ a box
    axis, radially inside the box: plane-cylinder intersection is analytic (a ⟂ box face
    cuts the cylinder in a CIRCLE), so the builder constructs the closed-form result from
    TRUE `Cylinder` walls + `Circle` rim edges + `Plane` caps (no faceting) — cut → box
    with a round through-hole (`boxVol − πr²h`), common → the clipped cylinder segment
    (`πr²·overlap`), fuse → box + protruding boss (`boxVol + πr²·protrude`). Curved seams
    weld watertight across the deflection ladder (`boundaryEdges==0` at
    {0.1…0.005}, all three axes). Guarded by an ANALYTIC-volume self-verify
    (`curvedBooleanVerified`) that DISCARDS anything off the closed-form volume → OCCT.
    OCCT-free in `src/native/boolean/curved.h`; wired into `boolean_solid` (curved tried
    first, planar path unchanged). Gate 1 green (host CTest **18/18**); sphere / cone /
    NURBS / non-axis-aligned / cyl-cyl / radial-breach / blind-hole / near-tangent all
    DECLINE → OCCT (honest, never faked). **Gate 2 (sim native-vs-OCCT parity) GREEN** —
    `[NCURVBOOL]` **18 checks (6 cases × 3), 0 failed**: three NATIVE analytic-intercept
    cases (through-hole-cut mass o=6429.2 n=6431.25 rel **3.19e-04** / area rel 2.10e-08 /
    watertight 216 tris; boss-fuse o=8392.7 n=8392.19 rel **6.10e-05** / area rel 2.00e-05 /
    watertight 212 tris; common o=1099.56 n=1098.12 rel **1.30e-03** / area rel 5.84e-04 /
    watertight 196 tris) and three OCCT-fallback cases (blind-hole-cut, oblique-cyl-cut,
    sphere-box-cut — forwarded, rel 0 by construction). Living change
    `add-native-curved-booleans` **archived** (validate --strict green). See the
    native-curved-boolean result table below.
- **Capability #6 `native-blends` — tractable planar slice done at the
  verification bar (BOTH gates green); curved / concave / variable / fillet_face
  OCCT-fallthrough (honest).**
  Native `cc_chamfer_edges` / `cc_fillet_edges` (constant radius) / `cc_offset_face` /
  `cc_shell` for the tractable PLANAR cases, built OCCT-free under `src/native/blend/`
  (`blend_geom.h`, `chamfer_edges.h`, `fillet_edges.h`, `offset_face.h`, `shell.h`,
  aggregate `native_blend.h`). Each op edits the solid's oriented-planar-polygon soup
  (the boolean's `extractPolygons`) and re-welds a watertight solid via the boolean's
  `assembleSolid` (T-junction repair + triangulate + weld), then the engine runs a
  MANDATORY SELF-VERIFY (watertight + sane volume sign — chamfer/fillet/shell shrink,
  offset grows/shrinks) and DISCARDS a bad candidate. What lands native:
  **chamfer** = slice the convex corner off with the plane through the two setback
  lines (EXACT vs OCCT for a box corner — 10×10×10 edge chamfer d=2 → vol 980);
  **fillet** = the rolling-ball tangent cylinder on a convex planar dihedral (axis ∥
  crease, radius r, seated tangent to both planes — the Phase-3 dihedral construction),
  tiled into deflection-bounded facets (vol 991.4, between the sharp 1000 and chamfer
  980, watertight); **offset_face** = slide a planar face along its normal, dragging the
  side faces (EXACT slab — top-face +5 → 1500, −4 → 600); **shell** = inset the kept
  walls inward by thickness and native-BSP-cut the cavity (open-top box t=1 → wall vol
  424). Gate 1 GREEN — host `test_native_blend` (10 cases: chamfer box/2-edge exact +
  degenerate/curved fallthrough; fillet watertight-between + curved/degenerate
  fallthrough; offset grow/shrink exact; shell wall exact + oversize fallthrough;
  concave L-prism edge chamfer/fillet → NULL while a convex edge of the same prism still
  works) + 5 new `test_native_engine` facade cases (native chamfer/fillet/offset/shell
  through `cc_set_engine(1)` + variable-radius deferral); host CTest **18/18** (was 17).
  OCCT-fallthrough AS OF THIS PLANAR SLICE (native builder returns NULL / self-verify
  discards → forwarded or honest error, never faked): CURVED-face inputs, CONCAVE edges,
  variable-radius `cc_fillet_edges_variable`, `cc_fillet_face`, an edge shared by ≠2 faces,
  multi-edge fillet interference, non-convex shell, oversized thickness. _(Later slices #6/#6b
  moved the circular cyl↔cap fillet — constant CONVEX + CONCAVE and VARIABLE-radius linear-law
  convex — to native; see the native-curved-fillet result table.)_ New `src/native/blend/`
  functions are 🟢 Excellent (≤10) except the two op drivers `fillet_edges` (13) /
  `chamfer_edges` (11) in the 🟡 Acceptable band (systems-band per-edge loop). **Gate 2
  (sim native-vs-OCCT parity, `native_blend_parity.mm` vs `BRepFilletAPI` /
  `BRepOffsetAPI`) GREEN — `[NBLEND]` 16 passed / 0 failed** through the `cc_*` facade
  under `cc_set_engine(0/1)` (OCCT default restored in teardown): chamfer (vol o=995
  n=995 **rel 2.29e-16**), offset (1500, rel 4.55e-16) and shell (424, rel 4.02e-16)
  EXACT + watertight; constant-radius fillet deflection-bounded (o=997.854 n=997.765
  rel 8.96e-05, watertight); a curved-rim fillet forwarded to OCCT (`[fallback]` rel
  0.00e+00); and the self-verify guard rejecting a thickness-6 shell on a 10³ box
  (id 0, honest error). See the native-blend result table below.
- **Capability #7 `native-exchange` — native STEP EXPORT slice done at the
  verification bar; STEP import + IGES stay OCCT (honest, out of scope).** Native
  `cc_step_export` (engine-wired behind `cc_set_engine(1)`) walks an in-scope native
  B-rep and emits a valid ISO-10303-21 STEP **AP203** file in true millimetres, OCCT-free
  under `src/native/exchange/`. Both gates green: host `test_native_step_writer` (#19) +
  `test_native_step` (#20) + `test_native_engine` (#21) — CTest **21/21**, no OCCT; and the
  sim OCCT re-read round-trip (source → native-written STEP → OCCT re-read) — box relV
  2.27e-16 (6→6 faces, 24→24 edges), cylinder relV 1.27e-03 (9→9 faces), holed-plate relV
  2.90e-04 (7→7 faces, 28→30 edges within tol); writer parity native-vs-OCCT relV ≤ 4.70e-15.
  Native writer active (box 5363 B / cylinder 6893 B / holed-plate 6457 B); a foreign
  OCCT-built body falls through to OCCT `STEPControl_Writer` (re-read relV 0.00e+00). **STEP
  import, IGES export/import, and out-of-scope geometry stay OCCT (never faked).**
- **Shape healing — FIRST NATIVE SLICE done at the verification bar (BOTH gates); the
  arbitrary-broken-B-rep residual stays OCCT (honest, asymptotic).** An INTERNAL, OCCT-free
  healer under `src/native/heal/` (`cybercad::native::heal::healShell`) that stitches a
  coincident-within-tolerance face soup / malformed shell into a connected, consistently-oriented,
  WATERTIGHT solid — or reports UNHEALED honestly and returns the input unchanged. It is INTERNAL
  (no `cc_*` entry point, like SSI): the engine invokes it and parity is asserted at the C++/sim
  boundary. Four sub-operations in dependency order: (1) **vertex/tolerance unification** (the
  `boolean/assemble.h` `VertexPool` spatial hash generalized to arbitrary B-rep vertices), (2)
  **tolerant sewing** (an edge becomes shared iff its endpoints unify to the same two shared vertices
  within tolerance — never a fabricated coincidence), (3) **degenerate removal** (zero-length edges +
  sliver / near-zero-area faces via a min-height test), (4) **orientation fix** (flood-fill consistent
  outward winding across shared edges + a global enclosed-volume-sign tie-break). Every heal is
  SELF-VERIFIED (`tessellate::isWatertight` + `enclosedVolume > 0` across a deflection ladder) before
  it is kept; otherwise a typed `Unhealed` result carries the measured `maxResidualGap` and the ORIGINAL
  shape. Both gates green: host `test_native_heal` (**10 cases, 0 failed** — soup-cube heals to V=1 with
  `nMergedEdges=12`/`nMergedVerts=16` `maxResidualGap==0`, degenerate edge + sliver face dropped, flipped
  face re-oriented, all-inward → global sign flip, within-tol verts merged + beyond-tol rejected, and
  both un-healable fixtures — missing face → `OpenShell`, gap 1e-2 → `GapBeyondTolerance` residual 0.0255
  — report UNHEALED input-unchanged; green NUMSCI OFF **and** ON) + sim native-vs-OCCT parity
  (`run-sim-native-heal.sh`, **`[NHEAL]` 4 passed / 0 failed**): the in-scope soup-cube + flipped-face
  heal to V=1 watertight matching OCCT `BRepBuilderAPI_Sewing` + `ShapeFix_Shell`/`ShapeFix_Solid` (V=1,
  valid), and on the beyond-tol / missing-face fixtures the native UNHEALED verdict MATCHES OCCT leaving
  the shell open (valid=0 watertight=0 at the same tolerance — no fabricated closure). Engine hook
  `tryNativeHeal` (`src/engine/native/native_heal_hook.h`) wires native → self-verify → OCCT fallback
  (`src/engine/occt/occt_shapefix.cpp`); NO `cc_*` change and `src/native/**` stays OCCT-free. **This is
  the gating foundation for a future native STEP IMPORT** — imported B-rep arrives with exactly this
  coincident-within-tolerance / degenerate / orientation defect family. **Honest scope (asymptotic, like
  SSI S4-f):** a MEASURED win vs OCCT on the in-scope defect family, NOT a guarantee on arbitrary broken
  industrial B-rep. Beyond-tolerance gap bridging, missing-pcurve reconstruction, self-intersecting-wire
  repair, and freeform re-approximation stay OUT OF SCOPE → reported UNHEALED, deferred to OCCT `ShapeFix`,
  never faked. Living change `add-native-shape-healing` archived to `openspec/specs/native-healing`.
- **Numeric foundations (native-rewrite capability #2) — NumPP/SciPP adopted as the
  OCCT-free numeric substrate + native closest-point/projection done at the verification
  bar.** NumPP + SciPP (the org's C++20, MIT NumPy/SciPy ports) are ADOPTED by absolute
  path (NOT vendored), CPU-only, guarded by `CYBERCAD_HAS_NUMSCI` (default OFF), consuming
  the SciPP `optimize`/`linalg`(+`spatial`/`integrate`) subset with `special`/`stats`
  EXCLUDED (libc++ ISO-29124 gap). On top sits a thin OCCT-free facade
  (`src/native/numerics/`) exposing the generic solvers (root / `fsolve` / `minimize`(BFGS) /
  `least_squares`(LM) / `solve` / `lstsq`) and native **closest-point / projection** (the
  `Extrema` on-ramp — point→curve and point→surface, multi-start + SciPP refine). Both
  gates green: host `test_native_numerics` (22 analytic + closed-form + brute-force
  assertions, no OCCT) and native-vs-OCCT `Extrema` parity on the booted iOS simulator
  (`native_numerics_parity.mm`) — **All 22 `[NNUM]` cases PASS**, dDist ≤ **1.776e-15**;
  analytic (plane / cylinder / sphere) feet fp-exact (dPoint ≤ 1.707e-10); B-spline feet
  within tolerance (largest deviation `bspline_surf#3` dPoint **3.946e-08** at corner
  u=v=0). Substrate compiles + links 77/77 TUs on HOST and arm64-iOS-simulator. This
  realizes the eval's ~**60–75% effort saving** on #2 (→ ~0.15–0.35 py). NOT bought:
  SSI (near-tangent) stays capability #5; multiple-extrema enumeration and curve-curve /
  surface-surface distance are deferred (single-target projection only). Living change
  `add-native-numerics` archived. See the numeric-foundations result table below.
- **SSI Stage S1 (analytic surface-surface intersection) — done at the verification
  bar (BOTH gates green); general / freeform / near-tangent SSI is S2–S4 (honest).**
  Closed-form intersection curves for the elementary-surface family, OCCT-free and
  header-only under `src/native/ssi/`, built over `src/native/math` only (IntAna-style
  closed form; NO GeomAPI / NO numsci — the SSI unit test does not require NUMSCI).
  SSI is INTERNAL: no `cc_*` entry point is added; parity is asserted at the
  `cybercad::native::ssi` C++ boundary, exactly like native-math / native-topology.
  **17 analytic-native pairs** verified vs the OCCT `GeomAPI_IntSS` oracle (all curve
  TYPES match; on-surface / coincidence residuals ≤ ~4e-15, well inside each pair's
  tol): plane∩plane (Line), plane∩sphere (Circle), plane∩cyl (⟂ Circle / ∥ 2 Lines /
  ∠ Ellipse), plane∩cone (Circle / Ellipse / Parabola / 2 Hyperbola branches),
  plane∩torus (⟂ axis 1–2 circles, ∋ axis 2 meridian circles), sphere∩sphere (Circle),
  coaxial sphere∩cyl / sphere∩cone / cyl∩cone (Circles), parallel cyl∩cyl (2 Lines),
  coaxial cyl∩cyl (coincident). **Honestly DEFERRED** (native returns `NotAnalytic`,
  never faked): skew cyl∩cyl (OCCT emits 7 Ellipse curves — a planar quartic, no
  degree-≤2 reduction) and by the same rule general cone∩cone, non-coaxial cone∩cyl /
  sphere∩cyl / sphere∩cone, oblique plane∩torus (spiric quartic), torus∩curved, and
  all freeform pairs → S2 seeding / S3 marching / S4 robustness. Both gates green:
  host `test_native_ssi` (**11 cases, 0 failed**; NUMSCI OFF CTest **23/23**, NUMSCI ON
  CTest **24/24**) + sim native-vs-OCCT `GeomAPI_IntSS` parity `run-sim-native-ssi.sh`
  (**18 pairs, 0 failed**). No regressions (`run-sim-suite.sh` **221/221**). Living
  change `add-native-ssi-analytic` **archived**. See the SSI-S1 result table below and
  `openspec/SSI-ROADMAP.md` (S1 + S2 done; **S3 marching-line tracer is NEXT**).
- **SSI Stage S2 (subdivision seeding) — done at the verification bar (BOTH gates green,
  TRANSVERSAL); near-tangent / coincident / degenerate seeding is S4 (honest).** Finds ≥1 seed
  point per **transversal** intersection branch for the **freeform** (NURBS / Bézier / B-spline) and
  **non-closed-form quadric** pairs S1 defers as `NotAnalytic`: recursive patch-AABB-overlap
  subdivision → candidate regions → `least_squares` refine on the numerics substrate → dedup to
  ~one seed per branch, OCCT-free in `src/native/ssi/` (refine guarded by `CYBERCAD_HAS_NUMSCI`),
  INTERNAL (no `cc_*`). Both gates green: host `test_native_ssi_seeding` (**6 cases, 0 failed** —
  skew cyl→2, crossing spheres→1, sphere∩Bézier-bump→1, parallel planes→0, tangent spheres→
  `deferredTangent` (no faked seed), deeper resolution recovers a small loop; NUMSCI OFF CTest
  **23/23** with the NUMSCI-gated tests correctly ABSENT, NUMSCI ON CTest **25/25**) + sim
  native-vs-OCCT `GeomAPI_IntSS` **recall** parity (`native_ssi_seeding_recall.mm`): **3/3
  transversal branches recalled at recall 1.00**, tangent = 0 everywhere, max seed on-surface
  residual **3.51e-16** (via `GeomAPI_ProjectPointOnSurf::LowerDistance` on both OCCT surfaces, well
  under the 1e-6 tol). OCCT NbLines (3/2/2) is its arc-split count, not the analytic branch count the
  recall denominator uses. No regressions (`run-sim-suite.sh` **221/221**, xcframework rebuilt with
  `seeding.cpp`). **Honest scope:** TRANSVERSAL only — near-tangent / coincident / degenerate
  seeding ill-conditions the refine → deferred to **S4** (`SeedSet.deferredTangent`, never faked);
  completeness is a measured recall figure (`minPatchFrac` default 1/32 is the recall/cost knob).
  Living change `add-native-ssi-seeding`; see the SSI-S2 result table below and
  `openspec/SSI-ROADMAP.md`.
- **SSI Stage S3 (marching-line tracer / WLine) — done at the verification bar (BOTH gates
  green, TRANSVERSAL); near-tangent / coincident / branch-point marching is S4 (honest).**
  From each S2 seed, walks the intersection curve — tangent `t = normalize(n₁×n₂)`, adaptive
  step, **re-project** each node onto BOTH surfaces via the numerics substrate (Newton/LM),
  march both directions and stitch, until the curve closes (`Closed`) or exits a boundary
  (`BoundaryExit`); fits a B-spline through the polyline; dedups retraced branches → one WLine
  per transversal branch. OCCT-free in `src/native/ssi/{marching.h,marching.cpp}`
  (`cybercad::native::ssi`); corrector / adaptive step / B-spline fit guarded by
  **`CYBERCAD_HAS_NUMSCI`** (`marching.cpp` is an EMPTY TU with NUMSCI off). SSI is INTERNAL —
  no `cc_*` entry point; asserted at the C++ boundary. Consumes the S2 `SeedSet`, produces a
  `TraceSet` of `WLine`s (each node carries (u1,v1,u2,v2) on both surfaces) — the S5 input
  contract. Both gates green: host `test_native_ssi_marching` (**7 cases, 0 failed** — crossing
  spheres → closed circle; plane∩sphere → closed circle; skew cylinders → 2 closed loops (+ seam
  wrap); sphere∩Bézier bump → loop on both freeform+sphere; ramp B-spline∩plane → open segment
  (`BoundaryExit`); tangent spheres → NO curve (deferred, not faked); duplicate seed → 1 WLine;
  every node on both surfaces < 1e-6, fit error < 1e-3; **NUMSCI OFF CTest 23/23** with the
  NUMSCI-gated tests correctly ABSENT, **NUMSCI ON CTest 26/26** adding `test_native_numerics`
  (#24), `test_native_ssi_seeding` (#25), `test_native_ssi_marching` (#26)) + sim native-vs-OCCT
  **curve parity** (`native_ssi_marching_parity.mm` vs `IntPatch` / `GeomAPI_IntSS`): **5 pairs,
  9 branches, 0 failed — all TRANSVERSAL fully-traced, 0 near-tangent-truncated**. Branch counts
  match OCCT on every pair; **5/5 OCCT closed loops reproduced as Closed native WLines**
  (bspline∩plane correctly 0-closed / 4-open). Worst deltas: max on-OCCT-curve **1.60e-06**, max
  on-surface **6.81e-07** (both skew-cyl-unequal), max length delta **2.28e-03** abs / ~0.33% rel
  (bspline∩plane, within the deflection/step tol). No regressions (`run-sim-suite.sh` **221/221**;
  `marching.cpp` additive/guarded, `native_ssi_marching_parity.mm` carries its own `main()`).
  **Honest scope:** TRANSVERSAL only — near-tangent branches are traced *up to* the tangent,
  marked `NearTangent`, counted in `nearTangentGaps` (never a point past it); coincident /
  branch-point / self-intersection deferred to **S4** (the moat). `nearTangentGaps > 0` is the
  honest S4 hand-off signal. Living change `add-native-ssi-marching` **archived**; see the SSI-S3
  result table below and `openspec/SSI-ROADMAP.md` (S4 robustness + S5 curved booleans remain).
- **Capability `native-meshing` (tetrahedral VOLUME meshing + quality, GitHub #1) —
  kernel-only slice.** New module `src/native/mesh/` (namespace `cybercad::native::mesh`):
  `quality.{h,cpp}` (ALWAYS-ON, pure geometry, no OCCT / no TetGen) and
  `tet_mesher.{h,cpp}` (the SOLE AGPL consumer). Additive `cc_*` surface:
  `cc_tet_mesh(body, deflection, opts)` (tessellate a B-rep surface → fill the PLC) and
  `cc_tet_mesh_surface(verts, tris, opts)` (a raw closed TRIANGLE surface, OCCT-free),
  both emitting CalculiX **C3D4** (linear, 4-node) / **C3D10** (quadratic, 10-node) tets
  — corners + mid-edge nodes in `shape10tet` order, positive signed volume enforced,
  mid-edge nodes constructed NATIVELY (no TetGen `-o2`); plus always-on native
  `cc_mesh_quality` (signed volume, 6 dihedral angles, scaled Jacobian, aspect ratio —
  a regular tet scores dihedral 70.53° / scaledJ 1). The tet mesher is backed by the
  **OPTIONAL, EXTERNAL, AGPL-3.0 TetGen** (`/home/leonardo/work/tetgen`, referenced by
  absolute path, NEVER vendored/committed), gated behind a NEW CMake option
  `CYBERCAD_HAS_TETGEN` (default OFF, mirroring `CYBERCAD_HAS_NUMSCI`; the define +
  include dir are per-source-scoped to `tet_mesher.cpp` alone). **The default MIT build
  compiles/links ZERO AGPL code** — with the flag off, `cc_tet_mesh` /
  `cc_tet_mesh_surface` return an empty `CCTetMesh` and set `cc_last_error` ("tet
  meshing unavailable"), never crashing; `cc_mesh_quality` still works. New tests:
  `test_native_quality` (regular-tet golden, sliver flag, inverted detection, C3D10
  mid-node consistency, empty/degenerate) runs in EVERY build; `test_native_tet` (a
  hardcoded unit cube → watertight C3D4/C3D10 at `pq1.4a…`, positive Jacobian, volume
  conservation via the divergence theorem, face manifoldness, quality gate) is
  registered only under `CYBERCAD_HAS_TETGEN=ON`. Built via `scripts/build-tetgen.sh`
  (external sources → `build-tet/host/libtetgen_host.a`; `predicates.cxx` at -O0 for
  exact-FP robustness, `tetgen.cxx` at -O2, both `-DTETLIBRARY`). **Honest scope:** this
  is a **kernel-only** delivery — there is NO FE patch test yet; wiring CalculiX++'s
  `CadMesher` (import / heal / triangulate / tet_mesh / quality / `map_to_model`) is a
  follow-up, NOT this PR. Shipping a closed-source app that links TetGen requires a
  **TetGen commercial license**; the default MIT configuration avoids the obligation by
  not linking it. See the native-meshing result section below.
- **No regressions.** Host build + CTest **28/28** NUMSCI OFF (**35/35** NUMSCI ON), incl.
  `test_native_tessellate`, `test_native_boolean`, `test_native_blend`, `test_native_step_writer`,
  `test_native_step`, and `test_native_heal` (#21); `scripts/run-sim-suite.sh` stays **221 passed,
  0 failed** (re-run against a freshly rebuilt SIMULATORARM64 slice; the OCCT default engine and the
  `cc_*` facade are unchanged — healing is INTERNAL, `native_heal_parity.mm` is on the SKIP list).
  Every control matches its pre-heal baseline: SSI 18, marching 10, S4 8, S5 native-pass=6, blend 16,
  fillet #6 9/9, wrap-emboss #7 6/6, Phase-3 70/70, `openspec validate --all --strict` 30/30.
- **Contained blast radius.** Native math lives under `src/native/math/`, native
  topology under `src/native/topology/`, native tessellation under
  `src/native/tessellate/` (all header-only, unreachable from the facade by design).
  Native construction (`src/native/construct/`) + `NativeEngine`
  (`src/engine/native/`) are engine-wired but reachable ONLY after an explicit
  `cc_set_engine(1)`; the default active engine is unchanged, so no ABI change and
  no behavioural change on the default (OCCT) path.

## Method recap — native rewrite (clean-room, OCCT as oracle)

Native code is implemented **clean-room** from first principles and public
references (*The NURBS Book*: FindSpan A2.1, BasisFuns A2.2, CurvePoint A3.1,
CurveDerivs A3.2, SurfacePoint A3.5, SurfaceDerivs A3.6; de Casteljau for
Bézier). OCCT source is consulted only as a numeric/convention **oracle**
(`gp_*`, `BSplCLib`, `BSplSLib`, `PLib`, `ElSLib`), never copied. fp64
throughout, fixed evaluation order for determinism.

## Verification model — two independent gates over the same code

Because native code carries **no OCCT dependency**, every capability is validated
by two gates, and is "done at the bar" only when BOTH pass AND every existing
suite stays green:

1. **Host unit tests** — the native library compiles and unit-tests with
   `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`, no OCCT, no simulator,
   asserting analytic/known-value results (a known Bézier point, a transform
   identity, an exact elementary-surface normal). First roadmap gate.
2. **Simulator native-vs-OCCT parity** — on a booted iOS simulator (OCCT linked
   ONLY in the parity test), the native result is compared element-by-element
   against the OCCT oracle within a documented tight fp64 tolerance. Second gate.

## native-math result table

**Host analytic gate:** `test_native_math` (compiled with Homebrew clang 22.1.3,
`-std=c++20`, `CYBERCAD_HAS_OCCT=OFF`, `CYBERCAD_HAS_METAL=OFF`) reports
**ALL TESTS PASSED** — 55 analytic assertions across value types, Bézier /
B-spline / NURBS curves, tensor-product surfaces, and elementary surfaces. It is
one of **8/8** CTest targets green (with the 7 pre-existing tests: test_registry,
test_guard, test_scheduler, test_compute_backend, test_parallel_policy,
test_parallel_toggle, test_abi).

**Native-vs-OCCT parity gate** (`tests/sim/native_math_parity.mm`, booted iOS
simulator, arm64): **24 groups, 24 passed, 0 failed.** Max native-vs-OCCT numeric
error per group:

| Group | Max error | Tolerance | Samples |
|---|---|---|---|
| transform-point | 3.773e-14 | 1.0e-09 | 1600 |
| transform-vector | 1.956e-14 | 1.0e-09 | 1600 |
| transform-dir | 5.135e-16 | 1.0e-09 | 1600 |
| bspline-curve-point | 3.053e-15 | 1.0e-09 | 1320 |
| bspline-curve-D1 | 1.876e-14 | 1.0e-08 | 1320 |
| bspline-curve-D2 | 1.936e-14 | 1.0e-07 | 1320 |
| nurbs-curve-point | 3.553e-15 | 1.0e-09 | 1320 |
| nurbs-curve-D1 | 3.371e-14 | 1.0e-08 | 1320 |
| nurbs-curve-D2 | 1.486e-13 | 1.0e-07 | 1320 |
| bspline-surface-point | 1.155e-14 | 1.0e-09 | 2880 |
| bspline-surface-dU | 1.776e-14 | 1.0e-08 | 2880 |
| bspline-surface-dV | 1.773e-14 | 1.0e-08 | 2880 |
| bspline-surface-normal | 9.853e-15 | 1.0e-08 | 2880 |
| nurbs-surface-point | 1.219e-14 | 1.0e-09 | 2880 |
| nurbs-surface-dU | 2.964e-14 | 1.0e-08 | 2880 |
| nurbs-surface-dV | 3.142e-14 | 1.0e-08 | 2880 |
| nurbs-surface-normal | 8.438e-15 | 1.0e-08 | 2880 |
| elem-plane | 2.092e-14 | 1.0e-09 | 300 |
| elem-cylinder | 3.995e-15 | 1.0e-09 | 300 |
| elem-cylinder-normal | 2.109e-15 | 1.0e-09 | 300 |
| elem-cone | 3.437e-15 | 1.0e-09 | 300 |
| elem-cone-normal | 2.026e-15 | 1.0e-08 | 300 |
| elem-sphere | 2.445e-15 | 1.0e-09 | 300 |
| elem-sphere-normal | 3.775e-15 | 1.0e-09 | 300 |

**Overall max numeric error across all groups: 1.486e-13** (nurbs-curve-D2),
~10⁶× under its 1.0e-07 tolerance.

### Files

Native library (OCCT-free, `src/native/math/`):

- `vec.h` — `Vec3` / `Point3` / `Dir3` fp64 value types + vector algebra.
- `transform.h` — 4×4 affine transform (compose / invert / apply to
  point / vector / direction).
- `bezier.h` / `bezier.cpp` — Bézier curve + surface via de Casteljau
  (rational via homogeneous coords + quotient rule).
- `bspline.h` / `bspline.cpp` — FindSpan / BasisFuns / CurvePoint / CurveDerivs /
  de Boor + tensor-product surface eval; NURBS via homogeneous coords.
- `elementary.h` — plane / cylinder / cone / sphere point + unit normal.
- `native_math.h` — umbrella header.

Tests:

- `tests/test_native_math.cpp` — host analytic gate (no OCCT).
- `tests/sim/native_math_parity.mm` — simulator native-vs-OCCT parity gate
  (own `main()`/runner; explicitly SKIPped by `run-sim-suite.sh`).

## native-topology result table

**Host invariant gate:** `test_native_topology` (compiled with Homebrew clang,
`-std=c++20`, `CYBERCAD_HAS_OCCT=OFF`, `CYBERCAD_HAS_METAL=OFF`) reports
**13 cases, 0 failed** — data-model / orientation-compose / location /
sub-shape-sharing / geometry-attachment / stable-id / deterministic-enumeration /
explorer-order / ancestry-symmetry / `BRep_Tool`-accessor / repeat-run-equality
invariants. It is one of **9/9** CTest targets green (with the 8 pre-existing
tests: test_registry, test_guard, test_scheduler, test_compute_backend,
test_parallel_policy, test_parallel_toggle, test_abi, test_native_math). The
test lives under `tests/native/` and is registered with a basename→source
override (`test_native_topology_SRC` → `tests/native/test_native_topology.cpp`).

**Native-vs-OCCT parity gate** (`tests/sim/native_topology_parity.mm`, booted
iOS simulator, arm64): a test-only importer loads OCCT `TopoDS_Shape`s into the
native model and compares against the OCCT oracle (`TopoDS`, `TopAbs`, `TopExp`,
`TopTools`, `BRep_Tool`, `TopLoc_Location`). **3 shapes × 5 checks = 15 passed,
0 failed.**

| Shape | Sub-shapes | mapshapes-order | ancestry (edge→faces) | accessors maxErr (tol 1.0e-09) | orientation |
|---|---|---|---|---|---|
| box | V8 E12 wire6 F6 shell1 solid1 | PASS | 12 edges match | 0.000e+00, surfType match | 34 sub-shapes match |
| cylinder | V2 E3 wire3 F3 shell1 solid1 | PASS | 3 edges match | 0.000e+00, surfType match | 13 sub-shapes match |
| filleted-box | V24 E56 wire26 F26 shell1 solid1 | PASS | 56 edges match | 0.000e+00, surfType match | 134 sub-shapes match |

**Overall max accessor error across all shapes: 0.000e+00** (world points, curve
ranges, and surface parameters read back bit-identically to the OCCT oracle;
surface-type classification matches on every face).

### Files

Native library (OCCT-free, header-only, `src/native/topology/`):

- `shape.h` — `ShapeType` / `Orientation` enums, underlying/use split (shared
  immutable underlying + cheap `(underlying, orientation, location)` use),
  orientation compose, `Location`, and attached geometry (vertex point+tol,
  edge curve+range+pcurves, face surface+ordered wires+tol).
- `explore.h` — deterministic depth-first walk, stable sub-shape ids
  (`MapShapes` analogue), lazy `Explorer`, and `Ancestors`
  (`MapShapesAndAncestors` analogue).
- `accessors.h` — `BRep_Tool`-style free-function accessors (`pnt`, `tolerance`,
  `curve`, `curve_on_surface`, `surface`) resolving geometry through the use's
  location.
- `native_topology.h` — umbrella header.

Tests:

- `tests/native/test_native_topology.cpp` — host invariant gate (no OCCT).
- `tests/sim/native_topology_parity.mm` — simulator native-vs-OCCT parity gate
  (own runner; explicitly SKIPped by `run-sim-suite.sh`).

### Deferred (recorded, not blocking the bar)

- **Non-manifold / degenerate edges** and **seam edges** (two pcurves on the same
  face) are not yet exercised by a fixture — deferred to native-construction,
  which will generate such edges.
- **`curve_on_surface` pcurve subtleties** beyond face-keyed selection (pcurve
  continuity, degenerate edges with no 3D curve) deferred alongside.
- **`CompSolid` `ShapeType`** and **`Internal`/`External` orientations** are
  reserved in the enums but not exercised by a fixture.
- The parity **face-with-a-hole** fixture is deferred (no OCCT holed-face fixture
  in the importer path yet); inner-wire read-back is covered by a host test.

## native-tessellation result table

**Host invariant gate:** `test_native_tessellate` (compiled with Homebrew clang,
`-std=c++20`, `CYBERCAD_HAS_OCCT=OFF`, `CYBERCAD_HAS_METAL=OFF`) reports all
deflection-bound / on-surface / trimming / watertightness / area-volume
convergence / determinism invariant tests green. It is one of **10/10** CTest
targets green (with the 9 pre-existing: test_registry, test_guard, test_scheduler,
test_compute_backend, test_parallel_policy, test_parallel_toggle, test_abi,
test_native_math, test_native_topology). The test lives under `tests/native/`.

**Native-vs-OCCT `BRepMesh` property-parity gate**
(`tests/sim/native_tessellation_parity.mm`, booted iOS simulator, arm64): the
same OCCT shape is meshed by the native mesher and by OCCT
`BRepMesh_IncrementalMesh` at a matched deflection, then compared on
watertightness / manifoldness, bbox, total surface area, and enclosed volume
(triangle count/topology NOT compared — tessellation is an approximation). **All
20 checks PASS across 4 shapes.** `[NTESS]` per-shape results:

| Shape | Watertightness | tris | bbox maxCornerΔ (tol) | area native / occtMesh / exact · relMesh / relExact (tol) | volume native / occtMesh / exact · relMesh / relExact (tol) | vertices-on-surface maxDist (defl) |
|---|---|---|---|---|---|---|
| box | watertight, boundaryEdges=0 | 12 | 0.0000e+00 (2.0e-1) | relMesh 0.000e+00 / relExact 0.000e+00 (2.0e-2) | relMesh 0.000e+00 / relExact 0.000e+00 (2.0e-2) | 0.000e+00 |
| cylinder | watertight, boundaryEdges=0 | 88 | 4.657e-02 | relMesh 2.826e-03 / relExact 5.838e-03 (2.0e-2) | relMesh 6.017e-03 / relExact 1.239e-02 (2.0e-2) | 0.000e+00 |
| sphere | watertight, boundaryEdges=0 | 1680 | 2.950e-02 | relMesh 2.429e-03 / relExact 4.656e-03 (2.0e-2) | relMesh 5.212e-03 / relExact 9.290e-03 (2.0e-2) | 5.687e-15 |
| filleted-box | watertight, boundaryEdges=0 | 332 | 4.440e-16 | relMesh 1.790e-03 / relExact 2.748e-03 (5.0e-2) | relMesh 2.004e-03 / relExact 3.012e-03 (5.0e-2) | 8.882e-16 |

Watertightness: **ALL four closed solids now mesh watertight (`boundaryEdges=0`)** —
box (12 tris), cylinder (88 tris), sphere (1680 tris), filleted-box (332 tris), each
2-manifold with **0 open/boundary edges**. The curved shared-edge stitch is
implemented — the mesher is a two-stage pipeline (STAGE 1 `edge_mesher.h` discretizes
each unique topological edge ONCE into a shared deflection-based 1D fraction list,
cached by the edge's `TShape` node; STAGE 2 `face_mesher.h` pins BOTH adjacent faces'
boundary vertices to those SAME fractions mapped through each face's pcurve, so
`S_face(pcurve(f)) == C_edge(f)`), exactly as OCCT `BRepMesh` builds its edge
discretization before meshing faces, so a cylinder's circular cap↔side seam (formerly
`boundaryFrac~0.119`, 2-manifold-bounded-open) and a fillet's blend seams now weld
closed. Gate-2 now REQUIRES `isWatertight()` for every closed solid — there is no
longer a weaker `manifold-bounded-open` pass. The host Gate-1 regressions
(`cylinder_solid_watertight_curved_seam`, `cylinder_solid_watertight_converges`)
confirm the cylinder solid is watertight (`boundaryEdges==0`) at every deflection with
area/volume converging to the closed form. Vertices-on-surface deflection residuals
are at machine epsilon (≤ 5.7e-15) — every emitted vertex is produced by `native-math`
`value(u,v)`, on the surface by construction.

**Spec conformance:** the `native-tessellation` spec's watertight requirement
("Mesh a whole Solid by stitching shared edges into a watertight mesh" — *"For a
closed solid the resulting mesh SHALL be watertight: every mesh edge SHALL be shared
by exactly two triangles… no naked/boundary edges"*) is now **genuinely met for every
closed solid**, including CURVED shared edges. Previously only planar-aligned (box)
and seam/pole (sphere) edges welded and the requirement was met with a documented
carve-out for curved seams; that carve-out is gone. The host regression hard-requires
`isWatertight()` + `boundaryEdgeCount()==0` for closed solids — there is no weaker
bounded-open acceptance path.

### Files

Native library (OCCT-free, header-only, `src/native/tessellate/`):

- `mesh.h` — `TriMesh`/`FaceMesh`/`SolidMesh` representation (fp64 vertex buffer
  with position + optional normal + per-vertex `(u,v)`, `uint32` CCW triangle
  index buffer, per-triangle face-id tag) + mesh-derived area/volume.
- `surface_eval.h` — deflection-driven UV-grid step selection over `native-math`
  `value`/`normal`/derivatives.
- `edge_mesher.h` — **STAGE 1**: `EdgeCache` — shared per-edge 1D discretization.
  Each unique topological edge is discretized ONCE into a deflection-based fraction
  list (3D-curvature sized), cached by edge `TShape` identity; both adjacent faces
  reuse it. This is the seam that makes CURVED shared edges weld watertight.
- `trim.h` — parameter-space wire flattening (pcurves → UV polygons) + even-odd
  point-in-polygon keep test (outer ∧ ¬holes); `appendEdgeSamplesAtFracs` samples
  an edge's pcurve at the shared STAGE-1 fractions.
- `uv_triangulate.h` — robust ear-clipping triangulation of a UV polygon (with
  bridged holes) for genuinely-trimmed faces (degeneracy-free; no incircle predicate).
- `face_mesher.h` — **STAGE 2**: boundary pinned to the shared edge discretization;
  structured-grid path for full-parametric-rectangle faces (boundary rows = shared
  samples) and ear-clip path for trimmed faces; produces a `FaceMesh`.
- `solid_mesher.h` — per-face meshing via `Explorer` sharing ONE `EdgeCache` +
  spatial-hash vertex weld (`VertexWelder`, weld tol = ½·deflection) into a
  `SolidMesh`.
- `gpu_sample.h` — optional `#ifdef CYBERCAD_HAS_METAL` fp32 UV-grid fill for
  GPU-eligible faces (display-only; correctness stays on the fp64 CPU path).
- `native_tessellate.h` — umbrella header.

Tests:

- `tests/native/test_native_tessellate.cpp` — host invariant gate (no OCCT).
- `tests/native/checks_tessellate.cpp` — shared property-check helpers.
- `tests/sim/native_tessellation_parity.mm` — simulator native-vs-OCCT `BRepMesh`
  property-parity gate (own runner; explicitly SKIPped by `run-sim-suite.sh`).
- `tests/sim/native_tessellate_parity.mm` — companion sim parity source (own
  `main()`; SKIPped by `run-sim-suite.sh`).

### Resolved in this iteration

- **Curved shared-edge stitch** — RESOLVED. The two-stage mesher (STAGE 1
  `edge_mesher.h` shared per-edge 1D discretization, STAGE 2 `face_mesher.h` pins both
  adjacent faces' boundaries to it) places coincident vertices on CURVED shared edges
  (cylinder cap↔side circle, fillet blend seams), so those solids now weld fully
  watertight (`boundaryEdges==0`). Was: 2-manifold-bounded-open.

### Deferred (recorded, not blocking the bar — none affect watertightness)

- **Ear-clip constrained re-triangulation** of boundary-straddling trim cells —
  currently kept/dropped by centroid, so the hole silhouette is resolved to the
  grid step (verified within a few-percent area bound) rather than clipped exactly.
- **GPU fp32 sampling backend** — compiled behind `CYBERCAD_HAS_METAL` but
  correctness only CPU-verified in this environment (host gate runs `METAL=OFF`);
  the GPU path is display-only by design.
- **Adaptive refinement quality / seam / degenerate faces** — grid density is
  deflection-driven per direction; adaptive per-cell subdivision and explicit
  degenerate/seam-face hardening are follow-ups.

## native-construction result table

**First engine-wired capability.** A native `NativeEngine : IEngine`
(`src/engine/native/`) implements exactly two construction ops natively and
**falls through to OCCT** (or the host stub) for everything else. The facade gains
an additive opt-in toggle `cc_set_engine(int)` / `cc_active_engine()` (modelled on
`cc_set_parallel`); the **default active engine stays OCCT**, so no existing suite
changes unless a caller explicitly opts in.

### What is native vs what falls through to OCCT

| `cc_*` build op | Engine | Native geometry |
|---|---|---|
| `cc_solid_extrude` (closed polygon profile) | **NATIVE** | bottom+top `Plane` caps + one planar quad `Plane` side face per profile edge; side edges `Line` |
| `cc_solid_revolve` (LINE-SEGMENT profile) | **NATIVE** | per-segment surface of revolution — parallel→`Cylinder`, perpendicular→`Plane`, oblique→`Cone`; circular edges `Circle`; full 360° closes shell, partial angle adds two `Plane` meridian caps |
| `cc_solid_extrude_holes` (outer + CIRCULAR holes) | **NATIVE** (#4b Tier A) | outer prism + per hole a TRUE `Circle` cap edge + one inward `Cylinder` wall, reversed circle as inner cap wire |
| `cc_solid_extrude_polyholes` (outer + POLYGON holes) | **NATIVE** (#4b Tier A) | outer prism + per hole an inner ring of `Line` edges + N inward `Plane` walls, reversed ring as inner cap wire |
| `cc_solid_extrude_profile` / `_profile_polyholes` (TYPED outer: kind 0 line / 1 arc / 2 full-circle) | **NATIVE** (#4b Tier A) | line→`Plane` side, arc→TRUE `Circle` edge + `Cylinder` wall (one bounded patch per ≤180° span), full-circle→`Cylinder` wall + disc caps; + circular/polygon holes |
| `cc_solid_revolve_profile` (TYPED: line, on-axis arc/semicircle) | **NATIVE** (#4b Tier A) | line→`Plane`/`Cylinder`/`Cone`, on-axis arc→`Sphere` band; full 2π closes, partial adds two `Plane` meridian caps |
| `cc_solid_extrude_profile` kind-3 SPLINE outer edge | OCCT-fallthrough (#4b) | native builder returns NULL; fall-through verified (vol rel 0.00e+00) |
| `cc_solid_revolve_profile` off-axis arc (TORUS) / any spline-revolve | OCCT-fallthrough (#4b) | no native `Torus` surface / spline surface-of-revolution yet; fall-through verified (torus vol rel 0.00e+00) |
| `cc_solid_loft`, `cc_solid_loft_wires` (TWO sections, EQUAL vertex count, PLANAR) | **NATIVE** (#4b Tier B) | ruled skin: one BILINEAR (degree-1 Bézier) side face per corresponding edge pair + two planar caps → watertight solid; mirrors ruled `BRepOffsetAPI_ThruSections` |
| `cc_solid_loft` / `_wires` MISMATCHED vertex counts / a NON-PLANAR section / a point-collapse section / 3+/guided/rail | OCCT-fallthrough (#4b Tier B→C) | native builder returns NULL; forwards to OCCT ThruSections (delegated, not faked) |
| `cc_solid_sweep` (STRAIGHT spine, or SMOOTH CURVED but PLANAR spine) | **NATIVE** (#4b Tier C) | constant-frame ruled tube (matches OCCT MakePipe's planar corrected-Frenet law): straight → EXACT directional prism; smooth-planar → bilinear ruled bands + planar caps, watertight |
| `cc_twisted_sweep` (twist ≈ 0 AND scale ≈ 1) | **NATIVE** (#4b Tier C) | reduces to `build_sweep` (no real twist) |
| `cc_solid_sweep` NON-PLANAR spine / TIGHT-CURVATURE / self-intersecting; `cc_twisted_sweep` REAL twist/scale; `cc_guided_sweep`, `cc_loft_along_rail` | OCCT-fallthrough (#4b Tier C) | native builder returns NULL (guarded / genuine non-constant law / pipe-shell guide-rail); delegated to OCCT, not faked |
| `cc_tapered_shank` | **NATIVE** (#4b Tier D) | pointed-shank silhouette (cone tip → full-radius cylinder → head disk) revolved 360° about WORLD Z by reusing the native `build_revolution` (`Cone`/`Cylinder`/`Plane` faces); tip is a TRUE on-axis apex (angular copies collapse to one shared vertex → no sliver), robustly watertight |
| `cc_helical_thread`, `cc_tapered_thread` (well-formed) | **NATIVE** (#4b Tier D) | radial-V helical tiling (V section transported radially via the AXIS auxiliary-spine law) → three bilinear ruled bands per span + two planar V caps; per-turn seams now weld robustly watertight via the mesher's canonical seam-point snap (`edge_mesher.h` `CanonicalEndpoints` / `face_mesher.h` `BoundaryAnchors`), so `boundaryEdges==0` at every deflection across the full parameter sweep (432/432 helical + 96/96 tapered) |
| `cc_helical_thread` FINE-PITCH / self-intersecting | OCCT-fallthrough (#4b Tier D) | a self-overlapping mesh is non-manifold regardless of weld → fails `robustlyWatertight`, so the engine falls through to OCCT `MakePipeShell` (labelled, verified, never faked; native builder never emits a round-profile fallback) |
| `cc_wrap_emboss` | OCCT-fallthrough (#4b Tier E) | deferred |
| every feature / boolean / query / transform / exchange op | OCCT-fallthrough | out of the construction capability; delegated |

The `NativeEngine` additionally serves native `tessellate`, `mass_properties`,
`bounding_box`, and `subshape_ids` on its OWN native bodies (bbox derived from the
tessellated mesh, since a revolved solid's B-rep vertices sit only at angular
stations); every other method forwards to the fallback unchanged. Feeding a
native-built shape into an OCCT-only op is not supported in this change.

**Host gate (Gate 1):** `test_native_construct` + `test_native_engine` (Homebrew
clang, `-std=c++20`, `CYBERCAD_HAS_OCCT=OFF`, `CYBERCAD_HAS_METAL=OFF`) assert
native builds with NO OCCT — box (exact vol/area/6-faces/centroid/bbox/watertight),
triangle prism (watertight, exact vol = area×depth), L-prism, full-turn tube (9π),
quarter-turn tube (9π/4), cone (4π), all within the deflection bound; plus engine
delegation + `cc_set_engine` toggle + deferred-op fall-through. CTest **12/12**
green (the 10 pre-existing + these two new targets).

**Native-vs-OCCT parity gate (Gate 2)** — `tests/sim/native_construct_parity.mm`,
booted iOS simulator, arm64, driven THROUGH the `cc_*` facade under
`cc_set_engine(0/1)` with the OCCT default restored in teardown. **17/17 `[NCONS]`
checks PASS.** Per-shape native-vs-OCCT deltas:

| Shape | Op | mass vol (o / n) · relVol | area rel | centroidΔ | bbox maxCornerΔ | faces (o / n) | tessellate |
|---|---|---|---|---|---|---|---|
| box | extrude, planar | 30 / 30 · **0.00e+00** | 0.00e+00 | 0.00e+00 | 1.00e-07 | 6 / 6 identical tiling | watertight, 12 tris, meshVolRel 0.00e+00 |
| triangle-prism | extrude, planar | 30 / 30 · **0.00e+00** | 0.00e+00 | 0.00e+00 | 1.00e-07 | 5 / 5 identical | watertight, 8 tris, meshVolRel 0.00e+00 |
| cylinder-tube | revolve 360°, curved | 28.2743 / 27.6063 · **2.36e-02** | 1.24e-02 | 1.11e-15 | 4.37e-02 | 4 / 12 angular tiling (n=3×o) | watertight, 168 tris, meshVolRel 1.55e-02 |
| partial-revolve-90 | revolve 90°, curved | 7.06858 / 6.9344 · **1.90e-02** | 8.19e-03 | 1.51e-02 | 1.00e-07 | 6 / 6 identical | watertight, 44 tris, meshVolRel 1.25e-02 |

Tolerances: planar prisms are EXACT (vol/area/centroid rel = 0, identical face
tiling); curved revolves match OCCT within a deflection bound (vol rel ≤ 2.36e-2,
tol v=5e-2 c=1e-1; bbox tol 1e-1) and are watertight. **Fall-through parity:** with
native active, `cc_boolean(fuse)` returns id=11 vol=14 (expect 14) — delegated to
OCCT, proving no native interception of deferred ops.

**Documented representational difference (not a geometric mismatch):** the native
builder emits per-face edges / per-patch vertices (proper edge/vertex SHARING
deferred) and tiles a full-turn surface of revolution into < π angular patches
(periodic-face construction deferred), so native V/E and the full-turn face count
differ from OCCT's shared/periodic representation while the SOLID is geometrically
identical (volume/area/bbox/watertight all match). The parity gate asserts face
count where tiling matches (prisms, partial revolve) and an integer-multiple
relation for the full-turn revolve.

**Honest scope split — core done, advanced swept solids are follow-up.** The
core #4 change delivered the CORE construction ops (extrude + line-segment revolve)
at the bar. The advanced swept solids — loft, sweep, twisted/guided sweep, threads,
holed/typed-profile extrude, arc/spline revolve — were EXPLICITLY DEFERRED, fall
through to OCCT (not faked), and are tracked as a follow-up (`#4b`) within the
capability. **`#4b` Tier A (holed + typed-profile extrude + typed-profile revolve)
is now done at the bar** — see below. **`#4b` Tier B (2-section ruled loft) is now
done at the bar** — BOTH gates green: Gate 1 (host `test_native_loft` +
`test_native_engine`, CTest **14/14**) and Gate 2 (sim OCCT parity
`native_loft_parity.mm`, **17 passed / 0 failed**) — see below. **`#4b` Tier C
(native sweep) is now also done at the bar** — `cc_solid_sweep` for a straight spine
(EXACT prism) and a smooth-planar spine (constant-frame ruled tube) is NATIVE; both
gates green (host `test_native_sweep` + `test_native_engine` CTest **15/15**; sim
`native_sweep_parity.mm` **11 passed / 0 failed**, both native cases EXACT vs OCCT
MakePipe rel ~1e-16) — see below. **`#4b` Tier D (tapered shank + threads) is now
also done at the bar** — `cc_tapered_shank` AND the well-formed `cc_helical_thread` /
`cc_tapered_thread` are NATIVE (the per-turn seam weld is fixed at the mesher level, so
threads mesh `boundaryEdges==0` at every deflection across the full parameter sweep); a
fine-pitch / self-intersecting thread still falls through to OCCT (honest guard). Tier E
(wrap-emboss), plus the non-planar / tight-curvature / real-twist / guided / rail sweep
cases, remain OCCT-fallthrough.

### `#4b` Tier B — native 2-section RULED loft (`cc_solid_loft` / `cc_solid_loft_wires`)

Built in `src/native/construct/loft.h` (OCCT-FREE, host-buildable), wired through
`NativeEngine::solid_loft` / `solid_loft_wires` behind the same `cc_set_engine(1)`
toggle. NOW NATIVE: a loft of TWO closed section wires with EQUAL vertex counts
(≥3) that are both PLANAR and non-degenerate — corresponding vertices are paired
1:1, each corresponding EDGE pair spans one BILINEAR (degree-1 Bézier, 2×2 poles)
ruled side face, and the two sections are capped with planar faces → a watertight
solid oriented outward. `cc_solid_loft` builds the bottom profile at z=0 and the top
at z=depth; `cc_solid_loft_wires` uses the two 3D wires directly. The bilinear
surface satisfies S(u,0)=A-edge, S(u,1)=B-edge, S(0/1,v)=side edges exactly, so it
welds watertight to its neighbours and caps through the two-stage mesher (no new
tessellator surface machinery — the existing Bézier path meshes it). Mirrors ruled
`BRepOffsetAPI_ThruSections` (the oracle used by the facade's OCCT `solid_loft`).

STILL OCCT-fallthrough (native builder returns NULL → `NativeEngine` forwards the
SAME arguments to OCCT, never faked): MISMATCHED section counts (n_A ≠ n_B — vertex
pairing ambiguous), a NON-PLANAR section wire (a planar cap can't close it), a
section that DEGENERATES to a point/line, and 3+ section / guided / rail lofts
(Tier C).

**Gate 1 (host, no OCCT) green:** `test_native_loft` (9 cases — square→equal-square
prism vol 48 exact; square frustum vol 56; rotated-square TWISTED skin watertight;
two-3D-wire triangle prism vol 18; tilted planar section watertight; + the deferred
cases mismatched-count / non-planar / degenerate / bad-input all NULL) and
`test_native_engine` (2 new facade cases: native square-frustum loft vol 56 with 6
faces, native loft_wires triangle prism vol 18) — CTest **14/14** (all suites
green), all functions in `loft.h` measure cognitive complexity ≤ 7 (Excellent band).

**Gate 2 (sim OCCT parity) GREEN:** `tests/sim/native_loft_parity.mm` +
`scripts/run-sim-native-loft.sh` drive the `cc_*` facade under both engines
(`cc_set_engine(0/1)`, OCCT default restored in teardown) and compare native vs
`BRepOffsetAPI_ThruSections(ruled=true)`. **`[NLOFT]` == 17 passed, 0 failed ==**
Per-op native (n) vs OCCT (o) deltas:

| Shape | Op / path | Engine | mass vol (o / n) · relVol | area rel | centroidΔ | bbox maxCornerΔ (tol) | faces (o / n) | tessellate |
|---|---|---|---|---|---|---|---|---|
| square-frustum | `cc_solid_loft` | **NATIVE** | 56 / 56 · **2.54e-16** | 1.07e-15 | 4.44e-16 (tol v=1e-6 c=1e-6) | 1.00e-07 (1e-6) | 6 / 6 (n=1×o) | watertight, 192 tris, meshVolRel 0.00e+00 |
| hex-prism | `cc_solid_loft` | **NATIVE** | 70.1481 / 70.1481 · **0.00e+00** | 1.41e-16 | 2.22e-16 (tol v=1e-6 c=1e-6) | 1.00e-07 (1e-6) | 8 / 8 (n=1×o) | watertight, 20 tris, meshVolRel 0.00e+00 |
| tri-prism | `cc_solid_loft_wires` | **NATIVE** | 18 / 18 · **0.00e+00** | 0.00e+00 | 2.22e-16 (tol v=1e-6 c=1e-6) | 1.00e-07 (1e-6) | 5 / 5 (n=1×o) | watertight, 8 tris, meshVolRel 0.00e+00 |
| rotated-square-twist | `cc_solid_loft` | **NATIVE** | 14.4379 / 14.5149 · 5.33e-03 | 8.17e-04 | 2.22e-15 (tol v=5e-2 c=5e-2) | 1.00e-07 (5e-2) | 6 / 6 (n=1×o) | watertight, 268 tris, meshVolRel 5.09e-03 |
| mismatched-counts | `cc_solid_loft` (n_A ≠ n_B) | OCCT-fallthrough | 40.1311 / 40.1311 · **0.00e+00** | — | — | — | — | native active=1, delegated to OCCT (fall-through proof) |

Tolerances: planar prisms / a same-plane-count frustum are EXACT (vol/area/centroid
rel ≤ 2.5e-16, identical face tiling n=1×o). The rotated-square TWIST (a genuinely
non-coplanar ruled skin whose OCCT `ThruSections` triangulates the warped quad
differently) matches within a deflection bound (vol rel 5.33e-3, well under its 5e-2
tol) and is watertight. The deferred MISMATCHED-count case (n_A ≠ n_B, vertex pairing
ambiguous — Tier C) delegates transparently to OCCT with native active
(vol rel 0.00e+00) — a fall-through proof, no native interception. Runs on the
simulator (OCCT linked); on `run-sim-suite.sh`'s SKIP list (own `main()`), so the
221-assertion OCCT-only suite count is unperturbed.

### `#4b` Tier C — native sweep (`cc_solid_sweep`, `cc_twisted_sweep`)

Built in `src/native/construct/sweep.h` (OCCT-FREE, host-buildable), wired through
`NativeEngine::solid_sweep` / `twisted_sweep` behind the same `cc_set_engine(1)`
toggle. NOW NATIVE: `cc_solid_sweep` of a closed profile along (a) a STRAIGHT spine
(an EXACT directional prism, always watertight, vol = profileArea × |d|) and (b) a
SMOOTH CURVED but PLANAR spine (a CONSTANT-frame ruled-band tube, capped at both ends,
watertight).

**The frame law is the crux.** The OCCT oracle `BRepOffsetAPI_MakePipe` uses
`GeomFill_CorrectedFrenet`, which for a PLANAR spine collapses to a CONSTANT rotation
(`GeomFill_CorrectedFrenet.cxx`, `isPlanar` → `Law_Constant`): it TRANSLATES the
section with a FIXED orientation, it does NOT keep the section perpendicular to the
tangent. So `detail::constantFrames` freezes the start trihedron's x/y axes across
every station (only the origin advances), builds one BILINEAR (degree-1 Bézier) ruled
band per (profile edge × spine segment) reusing `loft.h`'s `detail::ruledSideFace` with
SHARED per-station vertex rings, and caps both ends with `detail::planarFace` in the
fixed section plane. The enclosed volume is therefore `profileArea × |Δspine · n̂|`
(spine displacement projected onto the FIXED section normal), NOT the Pappus arc-length
volume. (An earlier RMF / double-reflection revision kept the section perpendicular and
produced the Pappus volume — geometrically "nicer" but a REAL mismatch vs the oracle,
correctly rejected by the parity gate; it was removed in favour of the constant frame
that matches the oracle. No `doubleReflectionRMF` / `SweptSurface` / `build_prism_dir`
helper shipped.) `cc_twisted_sweep` is native ONLY when it reduces to the plain sweep
(twist ≈ 0 AND scale ≈ 1 → forwards to `build_sweep`).

STILL OCCT-fallthrough (native builder returns NULL → `NativeEngine` forwards the SAME
arguments to OCCT, never faked): a NON-PLANAR curved spine (OCCT's genuine non-constant
corrected-Frenet law), a TIGHT-CURVATURE / self-intersecting spine (guarded by
`detail::spineTooSharp` — turning radius < profile circumradius or a per-vertex turn
> ~34°), a REAL twist/scale `cc_twisted_sweep`, and the pipe-shell/guide cases
`cc_guided_sweep` / `cc_loft_along_rail`.

**Gate 1 (host, no OCCT) GREEN:** `test_native_sweep` (11 cases — straight prism vol
160 exact / collinear-collapse vol 320 / arbitrary-3D-direction vol 16·L / pentagon
vol area·12 / zero-twist prism vol 160 / smooth-planar-arc watertight + constant-frame
volume `A·|Δspine·n̂|` / constant-frame invariance / degenerate + real-twist +
tight-curvature deferrals all NULL) + `test_native_engine` (`native_sweep_straight_prism`
vol 160, `native_sweep_smooth_arc` vol ≈ 82.57 = the oracle value,
`native_sweep_tight_and_twisted_defer`). Host CTest **15/15** green; `test_native_tessellate`,
`test_native_construct`, `test_native_loft` unchanged. `build_sweep` is a linear assembler
(cognitive complexity 14, Acceptable band); `constantFrames` ~4.

**Gate 2 (sim native-vs-OCCT parity) GREEN:** `tests/sim/native_sweep_parity.mm` +
`scripts/run-sim-native-sweep.sh` drive the `cc_*` facade under both engines
(`cc_set_engine(0/1)`, OCCT default restored in teardown) and compare native vs
`BRepOffsetAPI_MakePipe`. **`[NSWEEP]` == 8 native + 3 fallback = 11 passed, 0 failed ==**
Because native and OCCT now share the SAME constant-frame law + polyline, BOTH native
cases are EXACT (not merely deflection-bounded):

| Shape | Op / path | Engine | mass vol (o / n) · relVol | area rel | centroidΔ | bbox maxCornerΔ (tol) | faces (o / n) | tessellate |
|---|---|---|---|---|---|---|---|---|
| straight-path | `cc_solid_sweep` | **NATIVE** | 160 / 160 · **7.11e-16** | 1.48e-16 | 1.33e-15 (tol v=1e-6 c=1e-6) | 1.00e-07 (1e-6) | 6 / 6 | watertight, 12 tris, meshVolRel 0.00e+00 |
| smooth-arc-path | `cc_solid_sweep` | **NATIVE** | 330.299 / 330.299 · **1.72e-16** | 1.27e-15 | 7.11e-15 (tol v=5e-2 c=2e-1) | 1.00e-07 (2e-1) | 98 / 98 | watertight, 196 tris, meshVolRel 0.00e+00 |
| twisted_sweep real-twist | `cc_twisted_sweep` | OCCT-fallthrough | 93.3333 / 93.3333 · **0.00e+00** | — | — | — | — | native active=1, real twist/scale delegated to OCCT `ThruSections` |
| guided_sweep | `cc_guided_sweep` | OCCT-fallthrough | 290.37 / 290.37 · **0.00e+00** | — | — | — | — | native active=1, pipe-shell guide delegated to OCCT |
| loft_along_rail | `cc_loft_along_rail` | OCCT-fallthrough | 93.3333 / 93.3333 · **0.00e+00** | — | — | — | — | native active=1, pipe-shell rail delegated to OCCT |

Tolerances: both native sweeps are EXACT vs the oracle (vol/area/centroid rel at
machine epsilon ~1e-16, native F = OCCT F). The three deferred cases (real-twist,
guided, loft-rail) delegate transparently to OCCT with native active
(`cc_active_engine()==1`, vol rel 0.00e+00) — a fall-through proof, no native
interception. Runs on the simulator (OCCT linked); `native_sweep_parity.mm` is a `.mm`
already excluded by `run-sim-suite.sh`'s `*.cpp` find (also on the explicit SKIP list),
so the 221-assertion OCCT-only suite count is unperturbed (confirmed still 221).

**No regressions.** Host CTest **15/15** (incl. `test_native_tessellate` green —
box/cylinder/sphere/filleted-box watertight `boundaryEdges==0`, 13/13 cases);
`scripts/run-sim-suite.sh` **== 221 passed, 0 failed ==** against a freshly rebuilt
SIMULATORARM64 slice carrying the Tier-C sweep sources (24 TUs, determinism +
benchmark PASS). Zero source fixes required during verification — both gates passed
as-is.

### Files (Tier C)

- `src/native/construct/sweep.h` — OCCT-free `build_sweep` / `build_twisted_sweep`
  (constant-frame transport, straight-spine collapse, `spineTooSharp` guard) returning
  native `topology::Shape` (NULL ⇒ fall through). Reuses `loft.h` `ruledSideFace` +
  `construct.h` `planarFace`.
- `src/native/construct/native_construct.h` — exposes `build_sweep` /
  `build_twisted_sweep`; doc-comment moves sweep from DEFERRED to SUPPORTED (tractable
  cases), keeps guided/rail/tight-curvature/non-planar/real-twist DEFERRED.
- `src/engine/native/native_engine.{cpp,h}` — `solid_sweep` / `twisted_sweep` →
  native builder, NULL ⇒ fallback; `guided_sweep` / `loft_along_rail` pure fall-through.
- `tests/native/test_native_sweep.cpp` — host Gate-1 (11 cases, no OCCT).
- `tests/sim/native_sweep_parity.mm` + `scripts/run-sim-native-sweep.sh` — sim Gate-2
  native-vs-OCCT parity (own `main()`; SKIPped by `run-sim-suite.sh`).

### `#4b` Tier D — native tapered shank + threads (`cc_tapered_shank`, `cc_helical_thread`, `cc_tapered_thread`)

Built in `src/native/construct/thread.h` (OCCT-FREE, host-buildable, all four functions
cognitive complexity 🟢 Excellent ≤ 5), wired through `NativeEngine` behind the same
`cc_set_engine(1)` toggle. **All three ops now run NATIVE at the verification bar** for
their well-formed parameter ranges; only a FINE-PITCH / self-intersecting thread stays
OCCT-fallthrough (honest guard).

**`cc_tapered_shank` is NATIVE.** A pointed-shank silhouette (cone tip → full-radius
cylinder → head disk) revolved 360° about the **WORLD Z axis** by REUSING the
already-parity-verified native revolve (`construct.h` `build_revolution_framed`, explicit
frame `{origin=0, z=+Z, x=+X, y=+Y}`) — reproducing the OCCT `BRepPrimAPI_MakeRevol`
oracle (mass/centroid/bbox, not just the rotationally symmetric volume). The tip is a
TRUE on-axis apex (the revolve collapses its angular copies to ONE shared vertex → no
sliver breaks the weld — a non-zero tip radius does NOT weld, verified), giving a robustly
watertight solid at every deflection {0.05…0.005} with volume
`⅓π r²·taperHeight + π r²·fullHeight` within the deflection bound.

**`cc_helical_thread` / `cc_tapered_thread` are NOW NATIVE.** The radial-V helical tiling
(a V/triangular section transported RADIALLY along the pitch-line helix via the AXIS
auxiliary-spine law — radial `(cosθ,sinθ,0)`, axial `+Z`, so the V does NOT Frenet-rotate,
mirroring `BRepOffsetAPI_MakePipeShell::SetMode(axisWire,true)`) is tiled into three
bilinear ruled bands per span with shared per-station rings + two planar V end caps. **The
per-turn seam WELD was the last blocker and is now fixed at the mesher level** (topology-
preferred, geometry untouched): the mesher emits, for every straight boundary edge,
CANONICAL seam points interpolated at the shared sample indices between the edge's two
bounding vertices in a fixed lexicographic order (`edge_mesher.h` `CanonicalEndpoints` /
`face_mesher.h` `recordEdgeAnchors`), BIT-IDENTICAL for the two coincident edges regardless
of build order, and SNAPS each seam-lying vertex to its canonical point (`BoundaryAnchors`),
so the conservative single-cell weld fuses them with NO widening of the merge radius. As a
result the well-formed helical (major6…20 / pitch2…4 / turns1…5 / depth0.5…1.5 / spt8…24)
and tapered (top5…8 / tip3…4 / …) threads are ROBUSTLY watertight (`boundaryEdges==0`) at
EVERY deflection in the `robustlyWatertight` ladder across the full swept parameter space
(432/432 helical + 96/96 tapered candidates → native), so the engine keeps them NATIVE.

**Honesty guard (unchanged):** a FINE-PITCH / self-intersecting thread (turns fold through
each other, e.g. major2/pitch0.2/depth3) still fails `robustlyWatertight` — a
self-overlapping mesh is non-manifold no matter how the vertices weld — so it still FALLS
THROUGH to the OCCT `MakePipeShell` oracle (labelled, verified, never faked; the native
builder never emits a round-profile fallback).

**Gate 1 (host, no OCCT) GREEN:** `test_native_thread` (9 cases: shank watertight+volume,
shank ppm³ scaling, shank degenerate NULL, `helical_thread_is_watertight_across_ladder` +
`tapered_thread_is_watertight_across_ladder` — a HARD requirement asserting
`boundaryEdges==0` at EVERY deflection in the ladder {0.1,0.05,0.02,0.01} with the right
V-tiling face count, positive volume sign and turn count, degenerate-params NULL,
pitch-radius-below-axis NULL, tapered-tip-below-axis NULL, plus the
`fine_pitch_self_intersecting_thread_not_supported` guard) + `test_native_engine` facade
cases (native `cc_tapered_shank` watertight vol 1832.6; degenerate shank → fall-through 0;
`native_thread_runs_native_watertight` — well-formed helical + tapered threads run NATIVE
through the facade with valid watertight mass-properties; `native_fine_pitch_thread_falls_
through_to_default` — the self-intersecting thread still defers to the fallback). Host CTest
**18/18**; all prior native suites green (`test_native_construct` / `_profile` / `_loft` /
`_sweep` / `_tessellate` / `_boolean` / `_blend` / `_topology`). No fixes were needed —
everything was green on first run.

**Gate 2 (sim native-vs-OCCT parity) GREEN:** `tests/sim/native_thread_parity.mm` +
`scripts/run-sim-native-thread.sh` through the `cc_*` facade under `cc_set_engine(0/1)`
(OCCT default restored in teardown). `[NTHREAD]` per-op deltas (all PASS, tol vol/area
5e-2, centroid/bbox 1e-1 = 5× deflection 0.02):

| Shape | Op / path | Engine | mass vol (o / n) · relVol | area rel | centroidΔ (tol) | bbox maxCornerΔ (tol) | faces (o→n) | tessellate |
|---|---|---|---|---|---|---|---|---|
| tapered_shank r5/fh20/th10 | `cc_tapered_shank` (revolve 360° about Z) | **NATIVE** | 1837.94 / 1830.27 · **4.17e-03** | 3.64e-03 | 3.85e-02 (v=5e-2 c=1e-1) | 0.00e+00 (1e-1) | F 4→9, E 5→30, V 3→30 | watertight, 144 tris, meshVolRel 3.81e-03 |
| helical_thread mr5/p2/t4/d1 | `cc_helical_thread` | **NATIVE** | 70.2841 / 68.3767 · **2.71e-02** | 1.73e-02 | 4.83e-05 | 1.44e-03 | F 5→194, E 9→774, V 6→195 | watertight, 1286 tris, meshVolRel 1.40e-03 |
| tapered_thread top6/tip4/p2/t4 | `cc_tapered_thread` | **NATIVE** | 70.2677 / 68.3767 · **2.69e-02** | 1.71e-02 | 2.09e-02 | 2.36e-03 | F 5→194, E 9→774, V 6→195 | watertight, 1286 tris, meshVolRel 1.40e-03 |
| helical_thread FINE mr5/p0.3/t8/d1 | `cc_helical_thread` (self-intersecting) | OCCT-fallthrough | 36.4423 / 36.4423 · **0.00e+00** | — | — | — | — | native active=1, self-verify defers → delegated to OCCT |

Tolerances: `cc_tapered_shank` matches OCCT within a deflection bound (vol rel 4.17e-03,
tol v=5e-2 c=1e-1; bbox maxCornerΔ 0.00e+00) and is watertight; native FACE count is a
k=3 angular tiling of the periodic-revolve oracle (9 = 3 segments × 3 spans; OCCT's 4
shared/periodic faces). The native helical / tapered thread volume (68.38) differs from the
OCCT oracle (70.28) by **chord-vs-arc** (~2.7% at spt=16, tightening to ~1.3% at spt=24) —
a deflection artifact, not a geometric mismatch: the native mesh-volume matches its own
B-rep volume to meshVolRel ≤ 1.40e-3, area already matched (415.55 vs 422.87), and every
native body meshes watertight (`boundaryEdges==0`) at every tested deflection. The FINE-PITCH
self-intersecting thread delegates transparently to OCCT with native active
(`cc_active_engine()==1`, vol rel 0.00e+00) — a fall-through proof, no native interception.
`native_thread_parity.mm` is a `.mm` already excluded by `run-sim-suite.sh`'s `*.cpp` find
(and on the explicit SKIP list), so the 221-assertion OCCT-only suite count is unperturbed;
`run-sim-suite.sh` **== 221 passed, 0 failed ==** and the sim suite (`scripts/run-sim-suite.sh`)
reports 221/221 passed.

**Pre-fix defect metric (removed):** the earlier native/OCCT volume ratio was a constant
6.405 across turns={0.1,0.25,0.5,1,4} and spt=16 (6.498 at spt=24) while area already matched
(415.55 vs 422.87), which masked an inner-band orientation inversion; the thread weld fix
resolved it and the ratio is now a clean chord-vs-arc ~1.03.

### Files (Tier D)

- `src/native/construct/thread.h` — OCCT-free `build_tapered_shank` (silhouette hand-off
  to `build_revolution_framed` about world Z) + `build_helical_thread` /
  `build_tapered_thread` (radial-V axis-aux-spine tiling with self-intersection guards)
  returning native `topology::Shape` (NULL ⇒ fall through).
- `src/native/construct/native_construct.h` — exposes the three builders; doc-comment
  marks `tapered_shank` + well-formed `helical_thread` / `tapered_thread` SUPPORTED,
  fine-pitch / self-intersecting cases DEFERRED.
- `src/native/tessellate/edge_mesher.h` / `face_mesher.h` — the canonical seam-point weld
  (`CanonicalEndpoints` / `recordEdgeAnchors` / `BoundaryAnchors`) that makes the per-turn
  ruled-band ↔ V-cap seams fuse watertight without widening the merge radius.
- `src/engine/native/native_engine.cpp` — `tapered_shank` → native builder, NULL ⇒
  fallback; `helical_thread` / `tapered_thread` → native builder guarded by a
  `robustlyWatertight` self-verify → NATIVE for well-formed threads, OCCT fall-through only
  for a fine-pitch / self-intersecting candidate.
- `tests/native/test_native_thread.cpp` — host Gate-1 (9 cases, no OCCT — including the
  multi-deflection watertight ladder for helical + tapered, and the fine-pitch guard).
- `tests/sim/native_thread_parity.mm` + `scripts/run-sim-native-thread.sh` — sim Gate-2
  native-vs-OCCT parity (own `main()`; SKIPped by `run-sim-suite.sh`).

### `#4b` geometry-completion batch — Tier 1 + Tier 2#4 residuals (`add-native-geometry-completion`)

**Change:** `add-native-geometry-completion`. The honest attempt to close the remaining
`#4b` profile / loft / sweep / thread residuals natively where the geometry is achievable,
and to keep the genuinely-hard (surface–surface-intersection / Tier-4) cases as labelled
OCCT fall-through — never faked. Built OCCT-free in `src/native/math/torus.h` (Torus
surface point/normal), `src/native/construct/residuals.h` (kind-3 spline profile edge
extrude + off-axis-arc torus revolve, emitted as exact rational-quadratic B-spline patches
so no new tessellator surface kind was needed), `src/native/construct/loft.h` (N-section
ruled chain), `src/native/construct/sweep.h` (double-reflection RMF for a non-planar spine),
`src/native/construct/thread.h` (root-clamp resolver). Engine-wired behind the same additive
`cc_set_engine(1)` toggle (default stays OCCT).

**Per-area native-vs-fallback outcome (honest):**

| Area | `cc_*` op / sub-case | Engine | Result |
|---|---|---|---|
| **(A) spline extrude** | `cc_solid_extrude_profile` kind-3 spline outer | **NATIVE** | was OCCT-fallthrough; now native |
| **(A) torus revolve** | `cc_solid_revolve_profile` off-axis arc → TORUS | **NATIVE** | was OCCT-fallthrough; now native |
| **(B) N-section loft** | `cc_solid_loft` / `_wires` 3+ equal-count planar sections | **NATIVE** | was OCCT-fallthrough (Tier B was 2-section only); now native |
| **(C) non-planar sweep** | `cc_solid_sweep` non-planar smooth spine (RMF) | **NATIVE** | was OCCT-fallthrough; now native (RMF collapses to the constant frame on a planar spine → Tier-C parity preserved) |
| **(A) self-crossing spline** | `cc_solid_extrude_profile` self-crossing spline | **DECLINE (both)** | unbuildable SSI (Tier 4) — occtId=0 natId=0, honest, never faked |
| **(A) spindle torus** | `cc_solid_revolve_profile` off-axis arc crossing the axis | **DECLINE (both)** | self-intersecting SoR (Tier 4) — occtId=0 natId=0, honest |
| **(B) mismatched-count loft** | `cc_solid_loft` unequal vertex counts | OCCT-fallthrough | delegates to OCCT `ThruSections` (native active=1, rel 0.00e+00) |
| **(C) hard curved rail** | `cc_loft_along_rail` hard rail | OCCT-fallthrough | delegates to OCCT `MakePipeShell` (needs SSI/trimming) |
| **(C) self-intersecting sweep** | `cc_solid_sweep` / `cc_guided_sweep` folding spine | OCCT-fallthrough | delegates to OCCT `MakePipe` (SSI) |
| **(C) real-twist sweep** | `cc_twisted_sweep` real twist/scale | OCCT-fallthrough | delegates to OCCT `ThruSections` (did not self-verify oracle-correct — deferred, not faked) |
| **(D) self-intersecting thread** | `cc_helical_thread` / `cc_tapered_thread` truly self-intersecting | OCCT-fallthrough | delegates to OCCT `MakePipeShell` (Tier 4; the root-clamp resolver widened only well-formed threads) |

**Honest scope note.** Tier 1 (spline extrude, torus revolve) and the two Tier-2#4 items
that self-verified (N-section ruled loft, non-planar RMF sweep) landed native. The
accumulating-twist/scale sweep (`cc_twisted_sweep`), the guided/rail cases
(`cc_guided_sweep` / `cc_loft_along_rail`), and the thread self-intersection resolver did
NOT extend the native set beyond what self-verifies watertight + oracle-correct — those
remaining fall-throughs now specifically need SSI / Tier-4 machinery (surface–surface
intersection + trimming) and stay labelled OCCT fall-through.

**Gate 1 (host, no OCCT) GREEN:** host build (`/opt/homebrew/opt/llvm/bin/clang++`,
`-DCYBERCAD_HAS_OCCT=OFF -DCYBERCAD_HAS_METAL=OFF`) configured, compiled and linked with
zero errors/warnings; CTest **100% tests passed, 0 failed out of 22** (9.57s). Every named
prior native suite passed explicitly: `test_native_tessellate` (#10), `test_native_construct`
(#11), `test_native_loft` (#14), `test_native_sweep` (#15), `test_native_thread` (#16),
`test_native_boolean` (#17), `test_native_blend` (#19), `test_native_step` (#21), plus
`test_native_math` / `_topology` / `_profile` / `_residuals` / `_curved_boolean` /
`_step_writer` / `_engine` and the 7 core/ABI tests.

**Gate 2 (sim native-vs-OCCT parity) GREEN:** `tests/sim/native_geomcompletion_parity.mm` +
`scripts/run-sim-native-geomcompletion.sh` through the `cc_*` facade under `cc_set_engine(0/1)`
(OCCT default restored in teardown). All per-area native-vs-OCCT deltas under tolerance:

| Area | Op / path | Engine | mass vol (o / n) · relVol | area rel | centroidΔ | bbox maxCornerΔ | faces (o→n) | tessellate |
|---|---|---|---|---|---|---|---|---|
| spline extrude | `cc_solid_extrude_profile` kind-3 | **NATIVE** | 45.6 / 45.5547 · **9.92e-04** | 6.60e-04 | 3.42e-04 | 9.07e-04 | 4→4 | watertight, 132 tris, meshVolRel 9.01e-03, bboxΔ 1.92e-02 |
| torus revolve | `cc_solid_revolve_profile` off-axis arc | **NATIVE** | 98.696 / 96.0542 · **2.68e-02** | 1.24e-02 | 1.46e-15 | 1.14e-02 | 2→6 | watertight, 1620 tris, meshVolRel 2.37e-02, bboxΔ 4.44e-02 |
| ruled frustum (N-section loft) | `cc_solid_loft` | **NATIVE** | — · **1.43e-14** (EXACT) | 8.58e-16 | 3.24e-14 | 1.00e-07 | 6→6 | watertight, 432 tris, meshVolRel 1.83e-14 |
| straight-rail loft (N-section) | `cc_solid_loft` | **NATIVE** | — · **5.58e-15** (EXACT) | 4.34e-15 | 2.40e-14 | 1.00e-07 | 6→6 | watertight, 432 tris, meshVolRel 5.25e-15 |
| smooth-arc sweep (RMF) | `cc_solid_sweep` | **NATIVE** | 330.299 / 330.299 · **3.44e-16** (EXACT) | 1.27e-15 | 7.11e-15 | 3.55e-15 | 98→98 | watertight, 196 tris, meshVolRel 3.44e-16 |
| self-crossing spline | `cc_solid_extrude_profile` | **DECLINE (both)** | occtId=0 natId=0 | — | — | — | — | unbuildable SSI (Tier 4), honest |
| spindle torus | `cc_solid_revolve_profile` | **DECLINE (both)** | occtId=0 natId=0 | — | — | — | — | self-intersecting SoR (Tier 4), honest |
| mismatched-count loft | `cc_solid_loft` | OCCT-fallthrough | 202.185 · **0.00e+00** | — | — | — | — | native active=1, delegated to OCCT `ThruSections` |
| hard curved rail | `cc_loft_along_rail` | OCCT-fallthrough | 258.596 · **0.00e+00** | — | — | — | — | delegated to OCCT `MakePipeShell` |
| self-intersecting sweep | `cc_solid_sweep` / `cc_guided_sweep` | OCCT-fallthrough | 17.9515 · **0.00e+00** | — | — | — | — | delegated to OCCT `MakePipe` |
| real-twist sweep | `cc_twisted_sweep` | OCCT-fallthrough | 320 · **0.00e+00** | — | — | — | — | delegated to OCCT `ThruSections` |
| self-intersecting thread | `cc_helical_thread` | OCCT-fallthrough | 1446.76 · **0.00e+00** | — | — | — | — | delegated to OCCT `MakePipeShell` |

Tolerances: N-section loft (frustum, straight-rail) and the RMF smooth-arc sweep are EXACT
vs the oracle (vol rel ≤ 1.4e-14, machine epsilon). Spline extrude (vol rel 9.92e-04) and
off-axis-arc torus revolve (vol rel 2.68e-02) match within their deflection bound (tol
v=5e-02 c=2e-01, bbox 2e-01) and are watertight. The two DECLINE cases (self-crossing spline,
spindle torus) are unbuildable self-intersecting geometry both engines refuse (occtId=0
natId=0). The five OCCT-fallback cases delegate transparently with native active
(`cc_active_engine()==1`, rel 0.00e+00) — fall-through proofs, no native interception.

**No regressions.** `scripts/run-sim-suite.sh` (OCCT full suite, booted iOS simulator)
**== 221 passed, 0 failed ==** (confirmed twice; determinism A/B serial==parallel
bit-reproducible + benchmarks green). The parity harness carries its own `main()` (on the
`run-sim-suite.sh` SKIP list), so the 221-assertion OCCT-only count is unperturbed. No source
fixes were required — both gates passed as-is.

### Files (geometry-completion batch)

- `src/native/math/torus.h` — OCCT-free `Torus` surface (point + outward normal).
- `src/native/construct/residuals.h` — `build_prism_profile_spline` (kind-3 spline edge
  extrude) + `build_revolution_profile_spline` (off-axis-arc torus revolve as exact
  rational-quadratic B-spline patches; spindle-torus / spline-revolve / self-crossing → NULL).
- `src/native/construct/loft.h` — N-section ruled chain (extends the Tier-B 2-section builder).
- `src/native/construct/sweep.h` — double-reflection RMF (`rmfFrames`) for a non-planar spine
  (collapses to the constant frame on a planar spine); real-twist/scale + guided/rail stay NULL.
- `src/native/construct/thread.h` — root-clamp near-self-intersection resolver (widens only the
  well-formed set; truly-crossing threads still NULL → OCCT).
- `src/native/construct/native_construct.h` — exposes the new builders; SUPPORTED-vs-DEFERRED
  doc-comment updated.
- `src/engine/native/native_engine.{cpp,h}` — native-else-fallback wiring + the mandatory
  `robustlyWatertight` + volume self-verify.
- `tests/native/test_native_residuals.cpp` — host Gate-1 (no OCCT).
- `tests/sim/native_geomcompletion_parity.mm` + `scripts/run-sim-native-geomcompletion.sh` —
  sim Gate-2 native-vs-OCCT parity (own `main()`; SKIPped by `run-sim-suite.sh`).

### `#4b` Tier A result table — holed / typed-profile extrude + typed-profile revolve

**Change:** `add-native-construction-profiles`. Built in `src/native/construct/profile.h`
(OCCT-free, host-buildable; unified `build_prism_with_holes` / `build_prism_profile` /
`build_revolution_profile`) + a robustified multi-hole cap triangulator in
`src/native/tessellate/uv_triangulate.h`. Engine-wired behind the SAME additive
`cc_set_engine(1)` toggle (default stays OCCT).

**Host gate (Gate 1):** `test_native_profile` (12 cases — circular / polygon / multi-hole
/ combined holes watertight with exact-or-convergent volume; full-circle extrude →
cylinder; on-axis arc revolve → sphere 36π; partial-turn revolve; typed line/arc extrude)
+ 5 new `test_native_engine` facade cases. Host CTest **13/13** green; `test_native_tessellate`
stayed green (box / sphere / cylinder / filleted-box watertight, `boundaryEdges==0`).

**Native-vs-OCCT parity gate (Gate 2)** — `tests/sim/native_construct_profiles_parity.mm`
through the `cc_*` facade under `cc_set_engine(0/1)`, OCCT default restored in teardown.
**All 22 `[NCPROF]` checks PASS.** Per-op native (n) vs OCCT (o) deltas:

| `cc_*` op / sub-case | Engine | mass vol (o / n) · relVol | area rel | centroidΔ | bbox maxCornerΔ (tol) | faces (o→n) | tessellate |
|---|---|---|---|---|---|---|---|
| `cc_solid_extrude_holes` circular | **NATIVE** | 349.735 / 351.192 · 4.17e-03 | 9.40e-04 | 2.66e-15 | 1.00e-07 (1e-1) | 7→7 | watertight, 108 tris, meshVolRel 2.38e-03 |
| `cc_solid_extrude_polyholes` square | **NATIVE** | 288 / 288 · **1.97e-16** | 1.69e-16 | 0 (EXACT) | 1.00e-07 (1e-6) | 10→10 | watertight, 32 tris, meshVolRel 0 |
| `cc_solid_extrude_profile` line+arc | **NATIVE** | 18.8496 / 18.3688 · 2.55e-02 | 1.02e-02 | 1.09e-02 | 1.00e-07 (5e-2) | 4→4 | watertight, 64 tris, meshVolRel 1.96e-02 |
| `cc_solid_revolve_profile` line-tube | **NATIVE** | 28.2743 / 27.6063 · 2.36e-02 | 1.24e-02 | 1.11e-15 | 4.37e-02 (1e-1) | 4→12 (k=3 tiling) | watertight, 168 tris, meshVolRel 1.55e-02 |
| `cc_solid_revolve_profile` arc-sphere | **NATIVE** | 113.097 / 107.473 · 4.97e-02 | 2.52e-02 | 2.28e-16 | 9.05e-02 (1e-1) | 1→3 | watertight, 780 tris, meshVolRel 3.16e-02 |
| `cc_solid_extrude_profile` kind-3 SPLINE outer | OCCT-fallthrough | 45.6 / 45.6 · **0.00e+00** | — | — | — | — | delegated to OCCT (NULL native → fallback) |
| `cc_solid_revolve_profile` off-axis arc (TORUS) | OCCT-fallthrough | 98.696 / 98.696 · **0.00e+00** | — | — | — | — | delegated to OCCT (NULL native → fallback) |

Tolerances: polygon-hole extrude is EXACT (vol/area/centroid rel = 0, identical face
tiling); curved ops match OCCT within a deflection bound (largest native mass delta
4.97e-02 on arc-sphere, within its 5e-02 tol; bbox tol 1e-1) and are all watertight.
The two deferred sub-cases (kind-3 spline extrude, off-axis-circle → torus revolve)
transparently delegate to OCCT (vol rel 0.00e+00) — fall-through proof, no native
interception. A pure spline-*revolve* takes the same NULL→fallback path as the torus
and stays OCCT-fallthrough. A kind-1 ARC extrude edge is a TRUE `Circle` cap edge + one
bounded (non-periodic) `Cylinder` patch per ≤180° span (split threshold π for the EXTRUDE
wall vs 120° for the revolve), matching OCCT's single cylindrical face — not a chord
polyline.

**No regressions.** Host CTest **13/13** (incl. `test_native_tessellate`);
`scripts/run-sim-suite.sh` **221 passed, 0 failed** against a freshly rebuilt
SIMULATORARM64 slice (determinism + IGES/STEP round-trips PASS). Zero source fixes
required during verification.

**Where OCCT is STILL required after Tiers A–D + the geometry-completion batch (reality):**
GENERAL curved booleans, curved/concave/variable blends, features, STEP IMPORT + IGES
export/import (STEP EXPORT for in-scope native solids is now NATIVE, #7), shape healing;
a FINE-PITCH / self-intersecting `cc_helical_thread` / `cc_tapered_thread` (non-manifold
regardless of weld → self-verify defers to OCCT `MakePipeShell`), wrap-emboss (Tier E);
the remaining sweep cases — a TIGHT-CURVATURE / self-intersecting spine, a REAL twist/scale
`cc_twisted_sweep`, and `cc_guided_sweep` / `cc_loft_along_rail` (Tier C pipe-shell/guide,
all needing SSI/trimming); loft with MISMATCHED counts / a NON-PLANAR / point-collapse
section / a HARD guided/rail; plus a general SPLINE surface-of-revolution and a SPINDLE
torus (off-axis arc crossing the axis — self-intersecting SoR). All of these fall through
to OCCT via `NativeEngine` (native builder returns NULL or self-verify defers → OCCT), or
DECLINE on both engines for the unbuildable SSI cases — not faked. NOW NATIVE (Tiers A–D +
geometry-completion batch): holed / typed-profile extrude, kind-3 SPLINE profile edge
extrude, typed-profile revolve, off-axis-arc TORUS revolve, 2-section AND N-section (3+)
ruled loft, sweep along a straight / smooth-planar / NON-PLANAR (RMF) spine, `cc_tapered_shank`
(silhouette revolved 360° about Z), and the WELL-FORMED `cc_helical_thread` /
`cc_tapered_thread` (radial-V helical tiling, per-turn seams weld `boundaryEdges==0` at every
deflection).

### Files

- `src/native/construct/construct.h` — OCCT-free `extrudePolygon` / `revolveSegments`
  returning native `topology::Shape` (host-buildable, no `IEngine`/OCCT).
- `src/engine/native/native_engine.{h,cpp}` — `NativeEngine : IEngine`; native
  `solid_extrude`/`solid_revolve` + native tessellate/mass/bbox/subshape on native
  bodies; forwards the rest to a held fallback `shared_ptr<IEngine>` (OCCT under
  `CYBERCAD_HAS_OCCT`, stub on host).
- `include/cybercadkernel/cc_kernel.h` + `src/facade/cc_kernel.cpp` — additive
  `cc_set_engine` / `cc_active_engine` (default OCCT; host stub no-op → reports 0).
- `src/native/tessellate/trim.h` — `isFullRectangle(..., requireCorners)`: a PLANAR
  face's fast-path now also requires the loop to hit all four box corners, so a
  convex polygon cap (triangle/hexagon) is ear-clipped instead of filled as its UV
  bbox (one real caller, `face_mesher.h`, updated; OCCT tessellation path untouched).

Tests:

- `tests/native/test_native_construct.cpp` — host construction gate (no OCCT).
- `tests/test_native_engine.cpp` — host engine delegation + toggle gate (stub fallback).
- `tests/native/checks_construct.cpp` — shared parity property-check helpers.
- `tests/sim/native_construct_parity.mm` — simulator native-vs-OCCT parity gate
  through the facade (own runner; SKIPped by `run-sim-suite.sh`).

## native-booleans result table (#5)

**Honest analytic-planar-first slice.** `cc_boolean` (fuse / cut / common) is NATIVE
for **PLANAR-faced solids** (polyhedra — axis-aligned boxes, prisms) via a BSP-tree CSG
(Naylor-Amanatides-Thibault 1990) over the solids' planar polygons, guarded by a
MANDATORY self-verify (`robustlyWatertight` + set-algebra volume `Vr ≈ Va±Vb−Vab`) that
DISCARDS any candidate that is not a valid watertight solid with the correct volume and
falls through to OCCT. Curved-face operands (cylinder/sphere/cone), near-coincident /
tangent / degenerate configurations, non-native / foreign operands, and disjoint pairs
FALL THROUGH to OCCT (`BRepAlgoAPI_Fuse`/`_Cut`/`_Common` — labelled, verified, never
faked). Built OCCT-free under `src/native/boolean/` (`polygon.h`, `bsp.h`, `assemble.h`,
`native_boolean.h`); entry point `boolean_solid(a, b, op)`. Engine-wired behind the same
`cc_set_engine(1)` toggle (default stays OCCT).

### What is native vs what falls through to OCCT

| `cc_boolean` case | Engine | Reason |
|---|---|---|
| axis-aligned box / planar-polyhedron **fuse** (overlapping OR contained) | **NATIVE** | BSP-CSG over planar polygons, self-verified watertight + exact set-algebra volume |
| axis-aligned box / planar-polyhedron **cut** (`A−B`, overlapping) | **NATIVE** | same |
| axis-aligned box / planar-polyhedron **common** (`A∩B`, overlapping OR contained) | **NATIVE** | same |
| curved-face operand (cylinder ∪/− /∩ box, sphere, cone, NURBS) | OCCT-fallthrough | `isAllPlanar` guard → NULL; no native surface-surface intersection yet |
| near-coincident / tangent / degenerate configuration | OCCT-fallthrough | preflight guard / self-verify reject → NULL |
| disjoint operands (no overlap) | OCCT-fallthrough | self-verify rejects out-of-domain result → NULL |
| foreign (OCCT-built) / non-native operand | OCCT-fallthrough | `!isNative(a) || !isNative(b)` → delegate |
| general / concave-general / mixed operands | OCCT-fallthrough | out of the verified planar domain |

**Host gate (Gate 1) GREEN:** `test_native_boolean` + `test_native_engine` (Homebrew
clang 22.1.8, `-std=c++20`, `CYBERCAD_HAS_OCCT=OFF`, `CYBERCAD_HAS_METAL=OFF`) assert
native box fuse/cut/common watertight with EXACT set-algebra volume, a prism/simple-concave
case, self-verify rejecting a deliberately open / wrong-volume candidate, and fall-through
triggers (curved operand, coincident config, foreign body → NULL). Host CTest **17/17**
green (16 pre-existing + `test_native_boolean`), including `test_native_tessellate`
(test #10) — all 13 cases with every watertight assertion green
(`box_solid_is_watertight_exact_volume`, `sphere_watertight_and_converges`,
`cylinder_solid_watertight_curved_seam`, `cylinder_solid_watertight_converges`,
`mesh_open_patch_is_manifold_not_watertight`).

**Native-vs-OCCT parity gate (Gate 2) GREEN** — `tests/sim/native_boolean_parity.mm` +
`scripts/run-sim-native-boolean.sh` through the `cc_*` facade under `cc_set_engine(0/1)`
(OCCT default restored in teardown), booted sim `2B90AEDB`. **25 assertions passed, 0
failed.** Per-case native (n) vs OCCT (o) deltas:

| Case | Op / path | Engine | vol (o / n) · relVol | bbox Δ | tessellate |
|---|---|---|---|---|---|
| overlap-fuse | `cc_boolean(0)` | **NATIVE** | 14 / 14 · **1.27e-16** | 0 | watertight, 40 tris, meshVolRel 0 |
| overlap-cut | `cc_boolean(1)` | **NATIVE** | 6 / 6 · **2.96e-16** | 0 | watertight, 24 tris, meshVolRel 0 |
| overlap-common | `cc_boolean(2)` | **NATIVE** | 2 / 2 · **2.22e-16** | 0 | watertight, 12 tris, meshVolRel 0 |
| contained-fuse | `cc_boolean(0)` | **NATIVE** | 64 / 64 · **0.00e+00** | 0 | watertight, 36 tris, meshVolRel 0 |
| contained-common | `cc_boolean(2)` | **NATIVE** | 1 / 1 · **2.22e-16** | 0 | watertight, 12 tris, meshVolRel 0 |
| self-verify-guard | native∩native disjoint | **NATIVE reject** | id=0 | — | self-verify correctly rejected out-of-domain operands (native active=1) |
| cyl-box-fuse (curved) | `cc_boolean(0)` | OCCT-fallthrough | 55.8087 / 55.8087 · **0.00e+00** | 0 | watertight=0, 164 tris, meshVolRel 5.11e-03 (curved fallback: volume-bound only) |
| near-coincident-fuse | `cc_boolean(0)` | OCCT-fallthrough | 16 / 16 · **0.00e+00** | 0 | watertight, 20 tris, meshVolRel 1.67e-10 |
| disjoint-fuse | `cc_boolean(0)` | OCCT-fallthrough | 2 / 2 · **0.00e+00** | 0 | watertight, 24 tris, meshVolRel 2.22e-16 |

**Native-vs-OCCT volume deltas:** all NATIVE cases match OCCT to ~1e-16 (machine epsilon,
EXACT); all fallback cases rel 0.00e+00 (OCCT-forwarded, identical). The box fuse / cut /
common results are EXACT (`|A|+|B|−|A∩B|` / `|A|−|A∩B|` / `|A∩B|` to machine epsilon). The
curved cyl-box fallback is the ONLY non-watertight tessellation (a curved fallback bounded
by volume only, `watertight=0` — an OCCT-mesh property, not a native-boolean defect).

**No regressions.** Host CTest **17/17** (incl. `test_native_tessellate`);
`scripts/run-sim-suite.sh` **== 221 passed, 0 failed ==** on the OCCT-only facade suite
(8 sources compiled + linked, iOS sim `2B90AEDB`) — the default engine stays OCCT and
`cc_boolean` under it is unchanged. The only `run-sim-suite.sh` change is adding
`native_boolean_parity.mm` to the SKIP list (a `.mm` already excluded by the `*.cpp` find,
so the 221 count is unperturbed). #5 changes are confined to the NativeEngine path
(`src/native/boolean/*`, `native_engine`, the facade native branch) and gated behind
`cc_set_engine(1)`; native intercepts axis-aligned box fuse/cut/common EXACT and falls
through to OCCT (self-verified, never faked) for everything else. Note: the sim boolean
parity link emits pre-existing `ld` warnings (OCCT libs built for iOS-simulator 18.0 vs
linked 14.0), unrelated to #5.

**Booleans remain the longest-lived OCCT dependency for curved/general.** Only the
verified planar-polyhedron domain is native; surface-surface intersection (curved),
robust near-tangent/coincident handling, and full shape healing are future work.

### Files (#5)

- `src/native/boolean/polygon.h` — planar polygon + plane predicates (OCCT-free).
- `src/native/boolean/bsp.h` — BSP-tree CSG (plane-clip / invert, coplanar-coincident
  face handling) over the solids' planar polygons.
- `src/native/boolean/assemble.h` — B-rep-level T-junction repair + triangulation of the
  surviving polygons (closes coplanar seams a naive per-fragment mesher would leave open).
- `src/native/boolean/native_boolean.h` — umbrella; entry point `boolean_solid(a, b, op)`
  (isAllPlanar guard, fuse/cut/common, preflight guards → NULL fall-through).
- `src/engine/native/native_engine.cpp` — `boolean_op` native branch (both operands native
  → `build_boolean`; NULL or failed `booleanSelfVerify` → OCCT fall-through).
- `tests/test_native_boolean.cpp` — host Gate-1 (no OCCT).
- `tests/sim/native_boolean_parity.mm` + `scripts/run-sim-native-boolean.sh` — sim Gate-2
  native-vs-OCCT parity (own `main()`; SKIPped by `run-sim-suite.sh`).

## native-curved-boolean result table (#5 curved analytic slice)

**Narrow analytic curved slice — AXIS-ALIGNED box ⟷ axis-parallel cylinder.** `cc_boolean`
(cut / fuse / common) is NATIVE when one operand is an axis-aligned box and the other a
cylinder whose axis ∥ a box axis (and a world axis), sitting RADIALLY INSIDE the box
cross-section — the family where plane-cylinder intersection is analytic (a ⟂ box face cuts
the cylinder in a CIRCLE). The builder RECOGNISES the pair and CONSTRUCTS the closed-form
result from TRUE `Cylinder` walls + `Circle` rim edges + `Plane` caps (nothing faceted in
the B-rep): cut → box with a round THROUGH hole (`boxVol − πr²·h`), common → the clipped
cylinder segment (`πr²·overlap`), fuse → box + protruding round BOSS
(`boxVol + πr²·protrude`). Guarded by an ANALYTIC-volume self-verify
(`curvedBooleanVerified`) that DISCARDS anything off the closed-form volume → OCCT. Built
OCCT-free in `src/native/boolean/curved.h`; wired into `native_boolean.h::boolean_solid`
(curved tried first; planar BSP-CSG path unchanged). **General curved (sphere / cone /
NURBS / non-axis-aligned / cyl-cyl / blind-hole / non-through cut / near-tangent) remains
OCCT — the longest-lived OCCT dependency.**

### What is native vs what falls through to OCCT

| `cc_boolean` case | Engine | Reason |
|---|---|---|
| axis-aligned box − axis-parallel cylinder, THROUGH hole (cut) | **NATIVE** | analytic circle intercept, closed-form `boxVol − πr²·h`, self-verified |
| axis-aligned box + axis-parallel cylinder BOSS (fuse) | **NATIVE** | analytic, closed-form `boxVol + πr²·protrude`, self-verified |
| axis-aligned box ∩ axis-parallel cylinder segment (common) | **NATIVE** | analytic, closed-form `πr²·overlap`, self-verified |
| BLIND hole / non-through cut, `cyl − box` | OCCT-fallthrough | DECLINE → NULL (only THROUGH `box − cyl` is analytic here) |
| oblique / NON-axis-aligned cylinder | OCCT-fallthrough | DECLINE → NULL (no analytic circle/line intercept) |
| sphere / cone / NURBS operand | OCCT-fallthrough | DECLINE → NULL (no native surface-surface intersection) |
| cylinder ⟷ cylinder | OCCT-fallthrough | DECLINE → NULL |
| radially-breaching cylinder (∥-face LINE-ruling slot) | OCCT-fallthrough | DECLINE → NULL (out of the round-hole/boss family) |
| near-tangent / coincident-curved | OCCT-fallthrough | DECLINE → NULL (ambiguous) |

**Host gate (Gate 1) GREEN:** `test_native_boolean` + `test_native_engine` (Homebrew
clang 22.1.8, `-std=c++20`, `CYBERCAD_HAS_OCCT=OFF`, `CYBERCAD_HAS_METAL=OFF`) assert the
box-cylinder cut / common / fuse are watertight (`boundaryEdgeCount == 0` across the
deflection ladder on all three axes) with the analytic volume within the curved deflection
bound, plus honest DECLINE cases (wrong-order cyl−box, radial breach, blind hole, cone →
NULL → OCCT). Host CTest **18/18** green.

**Native-vs-OCCT parity gate (Gate 2) GREEN** — `tests/sim/native_curved_boolean_parity.mm`
+ `scripts/run-sim-curved-boolean.sh` through the `cc_*` facade under `cc_set_engine(0/1)`
(OCCT default restored in teardown). **`[NCURVBOOL]` == 18 checks (6 cases × 3), 0 failed.**
Per-case native (n) vs OCCT (o) deltas:

| Case | Op / path | Engine | mass (o / n) · relVol | area rel | centroidΔ | bbox maxCornerΔ (tol) | tessellate |
|---|---|---|---|---|---|---|---|
| through-hole-cut | `cc_boolean(1)` | **NATIVE** | 6429.2 / 6431.25 · **3.19e-04** | 2.10e-08 | 8.88e-15 | 0 | watertight, 216 tris, meshVolRel 3.24e-04 |
| boss-fuse | `cc_boolean(0)` | **NATIVE** | 8392.7 / 8392.19 · **6.10e-05** | 2.00e-05 | 7.27e-04 | 0 | watertight, 212 tris, meshVolRel 6.20e-05 |
| common | `cc_boolean(2)` | **NATIVE** | 1099.56 / 1098.12 · **1.30e-03** | 5.84e-04 | 3.55e-15 | 4.89e-03 (1e-1) | watertight, 196 tris, meshVolRel 1.33e-03 |
| blind-hole-cut | `cc_boolean(1)` | OCCT-fallthrough | 7057.52 / 7057.52 · **0** | 0 | 0 | 0 | watertight, 140 tris, meshVolRel 8.56e-04 (volume-bound only) |
| oblique-cyl-cut | `cc_boolean(1)` | OCCT-fallthrough | 6365.73 / 6365.73 · **0** | 0 | 0 | 0 | watertight=0, 192 tris, meshVolRel 1.90e-02 (volume-bound only) |
| sphere-box-cut | `cc_boolean(1)` | OCCT-fallthrough | 7773.81 / 7773.81 · **0** | 0 | 0 | 0 | watertight=0, 376 tris, meshVolRel 3.27e-04 (volume-bound only) |

**Native vs fallback breakdown:** native = **3** cases (through-hole-cut, boss-fuse, common),
fallback = **3** cases (blind-hole-cut, oblique-cyl-cut, sphere-box-cut). All three NATIVE
cases match OCCT within the curved-face deflection bound (relVol ≤ 1.30e-3, area rel ≤
5.84e-4) and are watertight (`boundaryEdges==0`); all three fallback cases are OCCT-forwarded
(rel 0 by construction, volume-bound tessellation only — an OCCT-mesh property, not a
native-boolean defect). Full log:
`scratchpad/ncurvbool.log`.

**No regressions.** Host build (OCCT=OFF, Metal=OFF, clang++ 22.1.8) compiles cleanly and
CTest **19/19** passed, INCLUDING `test_native_boolean` (#16, planar booleans still
native+exact) and `test_native_tessellate` (#10). The full iOS-simulator OCCT suite
(`scripts/run-sim-suite.sh`) **== 221 passed, 0 failed ==**. The slice is purely additive
(a new analytic box-cylinder curved path guarded by the existing watertight+volume
self-verify; NULL falls through to the planar path), so no existing native test was
regressed and no fixes were required.

### Files (#5 curved slice)

- `src/native/boolean/curved.h` — recognisers (`recogniseBox` / `recogniseCylinder`),
  world-frame axis-aware primitive builders, dispatcher `tryBoxCylinder` (OCCT-free,
  host-buildable; worst cognitive complexity 12 🟡, no 🟠/🔴).
- `src/native/boolean/native_boolean.h` — `boolean_solid` tries the curved path first,
  then the planar BSP-CSG path (unchanged).
- `src/engine/native/native_engine.cpp` — `curvedBooleanVerified` analytic-volume oracle.
- `tests/test_native_boolean.cpp` — host Gate-1 (no OCCT), curved cases + DECLINE cases.
- `tests/sim/native_curved_boolean_parity.mm` + `scripts/run-sim-curved-boolean.sh` — sim
  Gate-2 native-vs-OCCT parity (own `main()`; SKIPped by `run-sim-suite.sh`).

## native-blend result table (#6)

**Tractable-PLANAR slice of the blend family** — `cc_chamfer_edges` (convex planar-planar
edge), `cc_fillet_edges` (CONSTANT radius, convex planar-DIHEDRAL edge), `cc_offset_face`
(planar face), `cc_shell` (uniform-thickness box-like solid) are NATIVE; every other blend
configuration is a labelled, verified OCCT fall-through. Each native builder edits the
solid's oriented-planar-polygon soup (`boolean/extractPolygons`) and re-welds via
`boolean/assembleSolid`, then the engine runs a MANDATORY `blendResultVerified` self-verify
(watertight + sane volume SIGN — chamfer/fillet/shell REDUCE, offset GROWS for +d / shrinks
for −d) and DISCARDS a bad candidate → OCCT.

### What is native vs what falls through to OCCT

| `cc_*` blend op / case | Engine | Native geometry / fall-through reason |
|---|---|---|
| `cc_chamfer_edges` — convex PLANAR-PLANAR edge | **NATIVE** | slice the convex corner off with the plane through the two setback lines; EXACT vs OCCT |
| `cc_offset_face` — PLANAR face along its normal | **NATIVE** | slide the face along its normal, drag the side faces; EXACT slab (grow +d / shrink −d) |
| `cc_shell` — uniform thickness on a PLANAR / box-like solid | **NATIVE** | inset the kept walls inward by thickness + native BSP-cut the cavity; EXACT wall |
| `cc_fillet_edges` — CONSTANT radius, convex PLANAR-DIHEDRAL edge | **NATIVE** | rolling-ball tangent cylinder (axis ∥ crease, `C = E − r/(1+n1·n2)·(n1+n2)`, tangent lines `Ti = C + r·ni`), deflection-bounded facets; blend face a `Cylinder` of radius r |
| `cc_fillet_edges` — CONSTANT radius, CONVEX CIRCULAR crease (cylinder lateral ↔ coaxial planar cap), `Rc ≥ 2r` | **NATIVE (curved, #6)** | rolling-ball canal is a TORUS (major `R=Rc−r`, minor `r`); quarter-tube `v∈[0,π/2]`, trimmed to the two tangent circles (`torus∩cyl` at `v=0` radial, `torus∩plane` at `v=π/2` axial), G1-tangent at both seams, REMOVES material; deflection-bounded facet soup welded via `assembleSolid` |
| `cc_fillet_edges` — CONSTANT radius, CONCAVE CIRCULAR crease (boss cylinder ↔ larger coaxial plane base rim) | **NATIVE (curved, #6 concave)** | material-side TORUS (major `R=Rc+r`, minor `r`), tangent circles `Rc` (wall) / `Rc+r` (plane annulus), G1-tangent, ADDS material (volume GROWS); engine self-verify `wantGrow=true` |
| `cc_fillet_edges` — VARIABLE / cyl↔cyl / cyl↔cone / NON-circular curved rim / blind-hole bottom rim / freeform / convex `Rc<2r` / seam-leaves-face / multi-edge | OCCT-fallthrough | outside the constant-radius circular cyl↔plane slice; builder NULL → forwarded to `BRepFilletAPI_MakeFillet` |
| `cc_fillet_edges` / `cc_chamfer_edges` — CONCAVE PLANAR edge (reflex dihedral) | OCCT-fallthrough | reflex-dihedral material add + neighbourhood trimming out of slice; builder DECLINEs → NULL |
| `cc_fillet_edges_variable` — CONVEX circular cyl↔cap rim, LINEAR law `r(θ)=r1+(r2−r1)·θ/2π`, `Rc≥2·rmax` | **NATIVE** | `variable_fillet_edge` swept-radius canal, G1 at both varying-radius seams; self-verify `wantGrow=false`; native vol matches closed-form swept removed volume rel ≤ 1.1e-3, distinct from OCCT evolved oracle |
| `cc_fillet_edges_variable` — NON-linear law / CONCAVE-variable / cyl↔cyl-canal / NON-circular crease / `Rc<2·rmax` / multi-edge | OCCT-fallthrough | outside the convex circular linear-law slice; builder NULL → forwarded to `BRepFilletAPI_MakeFillet` (evolved) |
| `cc_fillet_face` (blend a whole face) | OCCT-fallthrough | pure fall-through, no native builder call |
| MULTI-EDGE interference (blends overlap at a corner) | OCCT-fallthrough | setback / corner-patch handling out of the single-edge slice; DECLINE → NULL |
| edge shared by ≠ 2 faces / non-convex shell / oversized thickness / foreign body | OCCT-fallthrough | preflight guard DECLINEs or self-verify discards → forwarded, never faked |

**Host gate (Gate 1):** `test_native_blend` (10 cases) + 5 new `test_native_engine` facade
cases, Homebrew clang 22.1.8, `-std=c++20 -DCYBERCAD_HAS_OCCT=OFF -DCYBERCAD_HAS_METAL=OFF`,
clean build zero warnings/errors, host CTest **18/18** (incl. `test_native_tessellate`
13/13 watertight, unperturbed by the blend changes).

**Native-vs-OCCT parity gate (Gate 2)** — `tests/sim/native_blend_parity.mm` +
`scripts/run-sim-native-blend.sh`, booted iOS simulator, arm64, through the `cc_*` facade
under `cc_set_engine(0/1)` (OCCT default restored in teardown). **`[NBLEND]` 16 passed /
0 failed.** Per-op native (n) vs OCCT (o) deltas:

| Case | Op / edge | Engine | mass vol (o / n) · relVol | area rel | centroidΔ | bbox maxCornerΔ (tol) | tessellate |
|---|---|---|---|---|---|---|---|
| chamfer-edge | `cc_chamfer_edges` planar corner | **NATIVE** | 995 / 995 · **2.29e-16** | 1.92e-16 | 8.88e-16 | 1.11e-16 (1e-6) | watertight, 16 tris, meshVolRel 0.00e+00 |
| offset-face | `cc_offset_face` planar face | **NATIVE** | 1500 / 1500 · **4.55e-16** | 1.42e-16 | 2.66e-15 | 0.00e+00 (1e-6) | watertight, 12 tris, meshVolRel 0.00e+00 |
| shell-open-top | `cc_shell` t on box, top open | **NATIVE** | 424 / 424 · **4.02e-16** | 1.28e-16 | 8.88e-16 | 0.00e+00 (1e-6) | watertight, 52 tris, meshVolRel 0.00e+00 |
| fillet-edge | `cc_fillet_edges` const-r planar dihedral | **NATIVE** (deflection-bounded) | 997.854 / 997.765 · **8.96e-05** | 1.05e-04 | 4.16e-04 | 1.88e-16 (2e-2) | watertight, 36 tris, meshVolRel 0.00e+00 |
| fillet-curved-edge | `cc_fillet_edges` on a curved rim OUTSIDE the #6 slice (OCCT-body / segmented rim) | OCCT-fallthrough | 497.562 / 497.562 · **0.00e+00** | 0.00e+00 | 0.00e+00 | 0.00e+00 (1e-6) | forwarded to OCCT, watertight, 1010 tris, meshVolRel 3.48e-03 (curved fallback: volume-bound only). The native curved-fillet path fires only on a native single-full-circle cyl↔cap rim; it does NOT hijack this case |
| self-verify-guard | `cc_shell` t=6 on 10³ box (wall ≥ ½ span) | native REJECTS | id 0 (expect 0) | — | — | — | no verified watertight wall → OCCT-only (honest error, no fake result) |

Tolerances: chamfer / offset / box-shell are EXACT (vol/area/centroid/bbox rel ≤ 4.55e-16
vs OCCT `BRepFilletAPI_MakeChamfer` / `BRepOffsetAPI`); the constant-radius planar-dihedral
fillet matches OCCT `BRepFilletAPI_MakeFillet` within a deflection bound (vol rel 8.96e-05,
tol 2e-2) with the blend face a cylinder of radius r, watertight. **Fall-through proof:** the
curved-rim fillet runs with native active (`cc_active_engine()==1`) yet is forwarded to OCCT
(rel 0.00e+00 — delegated, no native interception); the oversized-thickness shell is rejected
by the self-verify and returns id 0 (an honest failure, not a faked solid). Runs on the sim
(OCCT linked); on `run-sim-suite.sh`'s SKIP list (own `main()`), so the 221-assertion
OCCT-only suite count is unperturbed.

**Root-cause fix that made Gate 2 pass:** the `NativeEngine` had no native
`edge_polylines`, so a native body's edges were unqueryable and `findAxisEdge` in the sim
harness resolved edge id 0 → `cc_chamfer_edges` / `cc_fillet_edges` always returned 0.
`NativeEngine::edge_polylines` now discretizes each edge (in `mapShapes(Edge)` 1-based order,
matching `subshape_ids` and the blend ops' edge lookup) via the shared `EdgeCache`, so
native-body edges are pickable exactly as OCCT-body edges are (covered by a
`test_native_engine` `cc_edge_polylines` regression case).

### Files (#6)

- `src/native/blend/blend_geom.h` — convex planar-dihedral edge classifier, in-face
  perpendicular-to-edge direction, setback lines, rolling-ball tangent-cylinder solve,
  planar-cutter builder (OCCT-free).
- `src/native/blend/chamfer_edges.h` — `chamfer_edges(shape, edgeIds, count, distance)`.
- `src/native/blend/fillet_edges.h` — `fillet_edges(shape, edgeIds, count, radius)` (constant).
- `src/native/blend/offset_face.h` — `offset_face(shape, faceId, distance)`.
- `src/native/blend/shell.h` — `shell(shape, faceIds, count, thickness)`.
- `src/native/blend/native_blend.h` — umbrella (namespace `cybercad::native::blend`).
- `src/engine/native/native_engine.cpp` — `chamfer_edges` / `fillet_edges` / `offset_face` /
  `shell` native-else-(self-verify)-else-fallback branches + `blendResultVerified` +
  native `edge_polylines`; `fillet_edges_variable` / `fillet_face` stay pure fall-throughs.
- `tests/native/test_native_blend.cpp` — host Gate-1 (no OCCT).
- `tests/sim/native_blend_parity.mm` + `scripts/run-sim-native-blend.sh` — sim Gate-2
  native-vs-OCCT parity (own `main()`; SKIPped by `run-sim-suite.sh`).

## native-curved-fillet result table (#6 — CURVED-blend slice, CONVEX + CONCAVE + VARIABLE-radius convex)

**Constant-radius rolling-ball fillet on a CIRCULAR crease** — the rim where a CYLINDER
lateral face meets a coaxial PLANE. The rolling ball's canal surface is a TORUS (minor
`r`); the native builder (`src/native/blend/curved_fillet.h`, OCCT-FREE) trims the two
faces to the analytic tangent circles, inserts the torus quarter-tube (`v ∈ [0, π/2]`),
and rebuilds the whole filleted solid as one deflection-bounded planar-facet soup welded
watertight via `boolean/assembleSolid`. G1-tangent at both seams by construction (torus
normal radial at `v=0` = cylinder normal, axial at `v=π/2` = plane normal). Dispatched in
`NativeEngine::fillet_edges` after the planar path declines; the MANDATORY
`blendResultVerified` self-verify accepts or discards → OCCT. Two ball-centre-offset signs:

- **CONVEX** cyl↔coaxial-cap rim (`curved_fillet_edge`): major `R = Rc − r`, REMOVES
  material, self-verify `0 < Vr < Vo` (`wantGrow=false`). Requires `Rc ≥ 2r` (ring torus).
- **CONCAVE** boss-on-plate base rim (`concave_fillet_edge`): ball seats on the material
  side, major `R = Rc + r`, ADDS material, self-verify `Vr > Vo` (`wantGrow=true`, the same
  grow branch offset-face uses). No ring-torus guard (`Rc+r > r` always); requires the
  plate annulus to reach `≥ Rc+r` and the wall to cover the seam, else NULL → OCCT.

Because `fillet_edges` never forwards a declined native body to OCCT (it returns an honest
error), a non-zero native result IS the native build; a convex candidate can never pass
grow and a concave never passes shrink, so the sign cannot be spoofed.

**Host gate (Gate 1):** `test_native_blend` — closed-form-volume assertions (torus-canal
REMOVED for convex, ADDED rim-band for concave) + `curved_fillet_g1_tangent_at_both_seams`
and `concave_fillet_g1_tangent_at_both_seams` (pure-math: torus canal normal exactly radial
at `v=0` vs cylinder, exactly axial/plane at `v=π/2`, seam radii `Rc` / `Rc±r` — ANALYTIC,
no mesh, no OCCT) + `concave_fillet_scope_defers` (out-of-slice incl. blind-hole → NULL).
`test_native_blend` **22 cases / 0 failed** (incl. the 5 new variable cases); host CTest
**29/29** (NUMSCI-OFF), **36/36** (NUMSCI-ON).

**Native-vs-OCCT parity gate (Gate 2)** — `tests/sim/native_curved_fillet_parity.mm` +
`scripts/run-sim-native-curved-fillet.sh`, booted iOS simulator, arm64, `cc_*` facade
under `cc_set_engine(0/1)`. **`[NCFILLET]` 23 passed / 0 failed** (15 constant convex+concave
controls + 8 variable; `activeNative=1`, REAL native — not a fallback: native `n` ≠ OCCT `o`).
Native (n) vs OCCT `BRepFilletAPI` (o), exact closed-form (x):

| Case | vol (o / n / exact) · relO / relX | area rel | tessellate | G1 (analytic) |
|---|---|---|---|---|
| CONVEX Rc=5.0 h=10.0 r=1.5 | 771.245 / 768.804 / 771.245 · 3.17e-03 / 3.17e-03 | 1.74e-03 | watertight, 896 tris, meshVolRel 0.00e+00 | cos(wall)=1.0 cos(cap)=1.0 |
| CONVEX Rc=4.0 h=8.0 r=1.0 | 397.032 / 395.539 / 397.032 · 3.76e-03 / 3.76e-03 | 2.09e-03 | watertight, 716 tris, meshVolRel 0.00e+00 | cos(wall)=1.0 cos(cap)=1.0 |
| CONVEX Rc=6.0 h=12.0 r=3.0 (Rc=2r ring-torus boundary) | 1292.49 / 1288.78 / 1292.49 · 2.86e-03 / 2.86e-03 | 1.50e-03 | watertight, 1316 tris, meshVolRel 0.00e+00 | cos(wall)=1.0 cos(cap)=1.0 |
| CONCAVE Rc=5.0 Rp=12.0 r=1.5 (`grew=1`, ADDS material) | 2296.98 / 2294.95 / 2296.98 · 8.85e-04 / 8.85e-04 | 9.00e-04 | watertight, 1690 tris, meshVolRel 0.00e+00 | cos(wall)=1.0 cos(shoulder)=1.0 |
| CONCAVE Rc=4.0 Rp=10.0 r=1.0 (`grew=1`, ADDS material) | 1199.5 / 1198.18 / 1199.5 · 1.10e-03 / 1.10e-03 | 1.07e-03 | watertight, 1416 tris, meshVolRel 1.90e-16 | cos(wall)=1.0 cos(shoulder)=1.0 |

The native-vs-OCCT gap (≤ 0.38% volume) is the deflection bound of the planar-facet
tiling against OCCT's exact torus/cylinder faces, well inside the 1% bar; nothing faked.
The concave `grew=1` flag records that native volume exceeds the sharp input (material
ADDED). **G1 is ANALYTIC** — the reported `cos = 1.0` is the closed-form tangency the
torus construction guarantees (and the host tests verify the same closed form), NOT a
mesh-sampled measurement (flagged honestly on BOTH the convex and concave paths). STILL
OCCT-fallthrough for the CONSTANT slice: blind-hole bottom rim, cyl↔cyl / cyl↔cone canals,
NON-circular creases, freeform neighbours, convex `Rc < 2r`, seam-leaves-face, multi-edge,
segmented revolve rims (VARIABLE-radius linear-law convex is now native — see below).

### VARIABLE-radius (linear-law) convex slice (`variable_fillet_edge` / `cc_fillet_edges_variable`)

The same CONVEX cylinder↔coaxial-cap rim as #6, but the rolling-ball radius varies LINEARLY
around the rim: `r(θ) = r1 + (r2−r1)·θ/2π`. The centre locus is no longer a fixed-offset circle
but a SWEPT curve, so the two trim seams are NON-circular (varying-radius) curves. The native
builder (`src/native/blend/curved_fillet.h`, `variable_fillet_edge`, OCCT-FREE) sweeps a ring of
planar facets, each station using the local `r(θ)` upright meridian arc, welded watertight; the
canal normal is radial at the wall seam (`v=0`) and axial at the plane seam (`v=π/2`) at every
station → **G1 cos=1.0 at both varying-radius seams by construction** (analytic, host-verified,
not mesh-sampled). Wired in `NativeEngine::fillet_edges_variable` behind `cc_fillet_edges_variable`,
gated by the SAME correctly-signed `blendResultVerified(wantGrow=false)` self-verify; a candidate
that is not watertight or does not shrink to a sane volume is discarded → NULL → OCCT
`BRepFilletAPI` (evolved). The HARD native gate is native volume vs the builder's OWN closed-form
SWEPT removed volume (`relX`); the native-vs-OCCT-evolved figure (`relO`) is a SEPARATE, LOOSER
line (tol 6e-2) — the upright-meridian canal differs from OCCT's tilted evolved envelope by O(r′)
in the interior, agreeing exactly at both seams and in the `r1=r2` limit; reported honestly, never
folded into the HARD bound.

| Case | vol (n / exact-swept) · relX | vs OCCT evolved (o) · relO | area rel | tessellate | G1 (analytic) |
|---|---|---|---|---|---|
| VARIABLE Rc=5.0 r1=1.0→r2=2.0 | 769.963 / 770.796 · 1.08e-03 | 778.957 · 1.15e-02 | 1.52e-02 | watertight, 2410 tris, meshVolRel 2.95e-16 | cos(wall)=1.0 cos(cap)=1.0 |
| VARIABLE Rc=6.0 r1=0.75→r2=2.25 | 1338 / 1338.72 · 5.37e-04 | 1352.74 · 1.09e-02 | 1.83e-02 | watertight | cos(wall)=1.0 cos(cap)=1.0 |

The native volume (769.963) matches its closed-form (relX 1.08e-3) and is DISTINCT from the OCCT
evolved oracle (778.957) — proof the sim exercised native geometry, not an OCCT fall-through. STILL
OCCT-fallthrough (NULL / self-verify discards, honest error, never faked): NON-LINEAR radius laws
(quadratic/spline/per-vertex), CONCAVE variable rim, cyl↔cyl / cyl↔cone canal, NON-circular
variable creases (cone/sphere/ellipse/spline rim, tilted/non-coaxial plane), curved-edge chamfer,
freeform neighbours, `Rc < 2·rmax` near-degenerate or `Rc − rmax ≤ 0`, seam-leaves-face, multi-edge.

### Files (#6 curved slice — convex + concave + variable)

- `src/native/blend/curved_fillet.h` — `curved_fillet_edge(...)` (CONVEX cyl↔cap) +
  `concave_fillet_edge(...)` (CONCAVE boss↔larger-plane base rim, offset-sign flip `Rc+r`) +
  `variable_fillet_edge(...)` (VARIABLE-radius LINEAR-law convex cyl↔cap, swept-radius canal):
  rim recognition by geometry, torus/swept-canal quarter-tube + trimmed faces (wall + cap/annulus),
  planar-triangle facet soup (OCCT-free). The convex + concave paths are byte-identical (additive-only).
- `src/engine/native/native_engine.cpp` — `fillet_edges` tries planar (verify SHRINK) →
  convex `curved_fillet_edge` (verify SHRINK) → concave `concave_fillet_edge` (verify GROW),
  each gated by its own correctly-signed `blendResultVerified`; `fillet_edges_variable` tries
  `variable_fillet_edge` (verify SHRINK, `wantGrow=false`) → NULL/error → OCCT.
- `tests/native/test_native_blend.cpp` — host Gate-1 closed-form-volume + analytic-G1 +
  scope-defer cases (convex + concave).
- `tests/sim/native_curved_fillet_parity.mm` + `scripts/run-sim-native-curved-fillet.sh` — sim
  Gate-2 native-vs-OCCT parity (own `main()`; SKIPped by `run-sim-suite.sh`).

## native-wrap-emboss result table (#7 — first NATIVE wrap-emboss slice)

**Emboss a RECTANGULAR pad onto a CYLINDER lateral face** (`boss=1`), behind the existing
Phase-3 OCCT `cc_wrap_emboss` ABI (which stays the ORACLE). The native builder
(`src/native/feature/wrap_emboss.h`, OCCT-FREE) wraps the footprint by the SAME map the
OCCT oracle uses (`u = px/R`, `v = py + vMid`), builds the raised pad (outer cylindrical
cap at `R+height` + two circumferential walls + two axial walls) and retiles the base wall
with the footprint window removed, sharing a common `u`-sample sequence so every seam welds
watertight via `boolean/assembleSolid`. Dispatched in `NativeEngine::wrap_emboss` for a
native body; the MANDATORY `wrapEmbossVerified` self-verify (watertight + volume GROWS by
≈ `footprint area × height`) accepts or discards → OCCT (a declined native body returns an
honest error — never forwarded, as OCCT would misread the native void).

**Host gate (Gate 1):** `test_native_wrap_emboss` — footprint/rectangle recovery + facet-
soup watertightness + volume growth + decline of deboss / non-rectangular / non-cylindrical.

**Native-vs-OCCT parity gate (Gate 2)** — `tests/sim/native_wrap_emboss_parity.mm` +
`scripts/run-sim-native-wrap-emboss.sh`, booted iOS simulator, arm64, `cc_*` facade under
`cc_set_engine(0/1)`. **`[NWEMB]` 6 passed / 0 failed** (`activeNative=1`, REAL native).
Native (n) vs OCCT `cc_wrap_emboss` (o), analytic expected (x = base + footprint×height):

| Case | vol (o / n / expect) · relO / relX | area rel | tessellate |
|---|---|---|---|
| Rc=10.0 h=20.0 6×8 pad ×2 | 6388.79 / 6380.46 / 6375.05 · 1.30e-03 / 8.48e-04 | 6.48e-04 | watertight, 600 tris, meshVolRel 0.00e+00 |
| Rc=8.0 h=24.0 4×5 pad ×1.5 | 4858.3 / 4850.25 / 4851.48 · 1.66e-03 / 2.54e-04 | 7.25e-04 | watertight, 528 tris, meshVolRel 1.88e-16 |
| Rc=12.0 h=16.0 10×6 pad ×3 | 7440.73 / 7432.7 / 7414.22 · 1.08e-03 / 2.49e-03 | 6.17e-04 | watertight, 672 tris, meshVolRel 0.00e+00 |

The native-vs-OCCT gap (≤ 0.17% volume) is the deflection bound of the planar-facet tiling
against OCCT's exact cylindrical faces, well inside the 1% bar; nothing faked. The Phase-3
OCCT wrap-emboss #290 oracle is unchanged (`run-sim-phase3-suite.sh` 70/70). STILL OCCT:
DEBOSS (`boss=0`), NON-rectangular / >4-corner / dense / high-curvature profiles,
NON-cylindrical (cone/sphere/planar/NURBS) base, footprints wrapping >2π / self-overlapping
/ off the axial ends, non-positive height.

### Files (#7 native wrap-emboss)

- `src/native/feature/wrap_emboss.h` — `wrap_emboss(shape, faceId, profileXY, count, height,
  boss, defl)`: cylinder-wall recovery, rectangular-footprint recovery, wrapped pad (outer
  cap + circumferential + axial walls) + windowed base wall, planar-triangle facet soup (OCCT-free).
- `src/engine/native/native_engine.cpp` — `wrap_emboss` native path + `wrapEmbossVerified`
  self-verify; native-else-honest-error, OCCT body forwards to the Phase-3 oracle.
- `tests/native/test_native_wrap_emboss.cpp` — host Gate-1 (no OCCT).
- `tests/sim/native_wrap_emboss_parity.mm` + `scripts/run-sim-native-wrap-emboss.sh` — sim
  Gate-2 native-vs-OCCT parity (own `main()`; SKIPped by `run-sim-suite.sh`).

## native-exchange result table (#7 — native STEP EXPORT slice)

**Honest scope.** ONLY `cc_step_export` is native, and only for a native solid whose
every face surface is `Plane`/`Cylinder`/`Cone`/`Sphere`/`BSpline` and every edge curve
is `Line`/`Circle`/(non-rational)`BSpline` — the `canSerialize` gate DECLINES anything
else. The writer walks the native B-rep and emits a valid ISO-10303-21 STEP **AP203**
file in true MILLIMETRES (HEADER + Part-42 DATA graph + mm `SI_UNIT` context + AP203
product spine + `ADVANCED_BREP_SHAPE_REPRESENTATION`), built OCCT-FREE under
`src/native/exchange/` (`step_writer.h/.cpp`, `native_exchange.h`). Because the native
builders emit per-face edges (edge/vertex sharing deferred, #4), the writer DEDUPLICATES
geometrically — coincident points → one `VERTEX_POINT`, the two adjacent faces of a
physical edge share ONE `EDGE_CURVE` (forward on one, reversed on the other) → a properly
sewn manifold `CLOSED_SHELL`. **`cc_step_import`, `cc_iges_export`, `cc_iges_import` stay
OCCT — intentionally out of scope** (parsing/writing arbitrary STEP/IGES is not part of
this slice; the honest end state, keeping #8 blocked).

**Native STL (`add-native-stl-exchange`) — export + mesh import, OCCT-FREE.** Extends the
capability with two additive `cc_*` entries. `cc_stl_export(body, path, deflection, binary)`
reuses the neutral tessellation path (`IEngine::tessellate` → `MeshData`, no duplicated
meshing) and writes a binary (default) or ASCII STL under `src/native/exchange/stl_writer.{h,cpp}`
— per-facet geometric normal `normalize((v1-v0)×(v2-v0))` (`(0,0,0)` for a zero-area facet,
never fails), true millimetres, deterministic byte-identical output (fixed header, no
timestamp/host/build-id), binary 80-byte header that never begins `solid`. `cc_stl_import(path)`
(`stl_reader.{h,cpp}`) auto-detects ASCII vs binary — size-identity (`84 + 50·N`) beats a
deceptive `solid` header, a non-text head byte forces binary — parses the triangle soup,
**welds** coincident vertices on a tolerance grid — searching the 3×3×3 cell neighbourhood so
sub-tolerance vertices that straddle a cell boundary still merge (no under-welding of foreign
STLs) — and tolerates degenerate/zero-area facets (skipped) → a mesh-backed native body
(import-as-mesh only, **NOT** B-rep reconstruction) so display, `cc_tessellate`, bounding box,
surface area, and volume-if-closed all work. A well-formed zero-facet ASCII solid is detected as
ASCII (a `solid` lead closed by `endsolid`, not only one containing `facet`), not misread as a
too-small binary. Malformed input fails cleanly (`cc_last_error`, `0`, no partial body). Host
`test_native_stl` (#22, 10 cases: binary round-trip, ASCII well-formed, determinism binary+ASCII,
ASCII/binary auto-detect, `solid`-headed binary trap, malformed clean-fail, measurement, mixed
valid/degenerate + leading-`+` tolerate-and-recover, grid-boundary weld straddle, and zero-facet
ASCII detection) green. `cc_step_import`, `cc_iges_export/import` still stay OCCT.

**Native-vs-fallback split (engine wiring).** `NativeEngine::step_export`: an in-scope
native body → NATIVE writer; an out-of-scope native body → clean error (never a native
void handed to OCCT, never a faked file); a foreign (OCCT-built) body → OCCT
`STEPControl_Writer`.

| Path | Body | Result |
|---|---|---|
| **NATIVE** (native ISO-10303-21 emitted) | box | 5363 bytes written |
| **NATIVE** | cylinder | 6893 bytes written |
| **NATIVE** | holed-plate | 6457 bytes written |
| **FALLBACK → OCCT `STEPControl_Writer`** | foreign-box (OCCT-built body under native engine) | 15394 bytes; re-read relV 0.00e+00 / area rel 0.00e+00 / bbox 1.00e-07, faces 6→6 |

**Gate 1 (host, no OCCT) GREEN.** `test_native_step_writer` (#19, 6 cases: `canSerialize`
scope; box AP203 header+wrapper+mm `SI_UNIT`; box 6 `PLANE` / 12 shared `EDGE_CURVE` / 8
`VERTEX_POINT`; cylinder `CYLINDRICAL_SURFACE`+`CIRCLE`; well-formed contiguous
`#n=ENTITY(...);`; coords as REALs) + `test_native_step` (#20) +
`test_native_engine::native_step_export_writes_valid_ap203_file` (#21, facade
`cc_step_export` runs native, returns 1, valid file). Host CTest **21/21** (Homebrew clang,
`-DCYBERCAD_HAS_OCCT=OFF -DCYBERCAD_HAS_METAL=OFF`, clean, no warnings), all native suites
green. All writer functions 🟢 Excellent (≤7 cognitive complexity), no systems-band fn.

**Gate 2 (sim OCCT re-read round-trip) GREEN.** The native-written file is re-read through
OCCT `STEPControl_Reader` and compared to the SOURCE native solid (source → native-written
STEP → OCCT re-read):

| Shape (native writer) | vol src / reread · relV | area rel | centroidΔ | bbox maxCornerΔ | topology faces | topology edges |
|---|---|---|---|---|---|---|
| box | 1000 / 1000 · **2.27e-16** | 1.89e-16 | 0.00e+00 | 1.00e-07 | 6 → 6 | 24 → 24 |
| cylinder | 941.282 / 942.478 · **1.27e-03** | 5.97e-04 | 8.88e-16 | 1.00e-07 | 9 → 9 | 30 → 30 |
| holed-plate | 847.149 / 846.903 · **2.90e-04** | 1.09e-04 | 1.95e-14 | 1.00e-07 | 7 → 7 | 28 → 30 (within tol) |

The holed-plate's sewn periodic wall gains one seam edge the deferred-sharing native source
omits (28 → 30) — this MATCHES OCCT's own writer and is accepted as a bounded superset.
**Writer parity (native-written vs OCCT-written, both re-read):** box relV 0.00e+00 / relA
0.00e+00 / bboxΔ 0.00e+00; cylinder relV 0.00e+00 / relA 4.64e-14 / bboxΔ 0.00e+00;
holed-plate relV 4.70e-15 / relA 6.48e-15 / bboxΔ 0.00e+00 — the native writer produces a
solid geometrically identical to OCCT's writer for the same body.

### Native STEP IMPORT (`add-native-step-import`) — FIRST import slice, OCCT-FREE, DONE at both gates

The deterministic INVERSE of the writer above: `cc_step_import` under the native engine now
parses the ISO-10303-21 AP203 DATA section and reconstructs a native B-rep for the writer's
exact entity alphabet (CARTESIAN_POINT/DIRECTION/VECTOR/AXIS2_PLACEMENT_3D, LINE/CIRCLE/
B_SPLINE_CURVE_WITH_KNOTS, PLANE/CYLINDRICAL/CONICAL/SPHERICAL/B_SPLINE_SURFACE_WITH_KNOTS,
VERTEX_POINT/EDGE_CURVE/ORIENTED_EDGE/EDGE_LOOP/FACE_OUTER_BOUND/FACE_BOUND/ADVANCED_FACE/
CLOSED_SHELL/MANIFOLD_SOLID_BREP + the mm unit/context wrapper). Two OCCT-FREE pieces:

- **Part-21 tokenizer + entity table** (`src/native/exchange/step_reader.cpp`): a recursive-
  descent scanner over `#N=ENTITY(args);` producing `map<#id, Record>` with an `Arg` variant
  (ref/int/real/str/enum/list/`$`/`*`). Handles typed reals (`1.`, `1.E2`, `-3.5E-07`),
  `''`-doubled strings, enums, nested lists, and the combined `( SUB(...) SUB(...) )` unit form.
- **Two-pass mapper** mirroring the writer: leaf geometry (memoized by `#id`) → topology,
  reusing the writer's shared `EDGE_CURVE`/`VERTEX_POINT` dedup by `#id` so adjacent faces
  reference the SAME edge node (watertight by construction). It reconstructs the analytic
  PCURVEs the tessellator needs (STEP carries none), reorders/reorients each `EDGE_LOOP` by
  vertex connectivity (the writer emits all `ORIENTED_EDGE` `.T.`, so the sense flag is not a
  reliable directed walk), drops the periodic-wall SEAM (an `EDGE_CURVE` used fwd+rev in one
  loop), and unwraps angular pcurve `u` onto one 2π branch per face.

**Honest scope / DECLINE → NULL → OCCT (never fabricates):** any unsupported entity/surface
keyword, rational/weighted B-spline, `TOROIDAL_SURFACE`/`ELLIPSE`/etc., assembly / >1
`MANIFOLD_SOLID_BREP`, non-mm unit context, malformed record, or a reconstruction that does
not self-verify. `heal::healShell` is deliberately NOT run inline (it rebuilds every face as a
best-fit PLANE → would planarize a curved solid); instead the engine self-verifies the
reconstruction robustly watertight with volume>0 and, on ANY failure, falls back to OCCT
`STEPControl_Reader`. `src/native/**` stays OCCT-free; the fallback is engine-side only.

**Gate 1 (host, no OCCT) GREEN.** `test_native_step_reader` (#25, 9 cases): box/cylinder/
holed-plate round-trip (`export_native → import_native → tessellate`) reconstructs valid
WATERTIGHT solids with volume EXACT for the box (1000, relΔ 0) and matching the source to fp
for the curved solids (both ends mesh the identical reconstruction), topology preserved
(faces/vertices/edges == source); plus DECLINE → NULL for `TOROIDAL_SURFACE`, a two-root
assembly, a non-mm unit, a malformed record, and empty input. Host CTest **29/29** green
(default) and **36/36** green under `-DCYBERCAD_HAS_NUMSCI=ON` — no regressions across the
export slice, healing, SSI S1–S4/S5, blends, marching, boolean, construct, tessellation.

**Gate 2 (sim vs OCCT + foreign STEP) GREEN.** `run-sim-native-step-import.sh` +
`tests/sim/native_step_import_parity.mm` via the `cc_*` facade — `[NIMPORT] 15 passed / 0
failed`:

| Case | vol native / OCCT · relV | area rel | bboxΔ | faces | edges (native uniq vs OCCT) |
|---|---|---|---|---|---|
| native-written box | 1000 / 1000 · **2.27e-16** | 1.89e-16 | 0.00e+00 | 6 = 6 | 12 = 12 |
| native-written cylinder | 1568.8 / 1570.8 · **1.27e-03** | 5.08e-04 | 0.00e+00 | 9 = 9 | 15 = 15 |
| native-written holed-plate | 847.149 / 846.903 · **2.90e-04** | 1.09e-04 | 0.00e+00 | 7 = 7 | 14 vs 15 (seam) |
| **FOREIGN** OCCT-written box | 1000 / 1000 · **0.00e+00** | 0.00e+00 | 0.00e+00 | 6 = 6 | 12 = 12 |
| **FOREIGN** OCCT-written cylinder | 1570.8 / 1570.8 · **0.00e+00** | 0.00e+00 | 0.00e+00 | 3 = 3 | 3 = 3 |

The native reader reconstructs the same solid OCCT's `STEPControl_Reader` reads from the
native-written files (planar EXACT, curved within the deflection bound), AND reads a FOREIGN
OCCT-`STEPControl_Writer`-produced STEP of a box/cylinder natively — proving it parses
foreign-generated AP203, not just its own writer's output.

STILL OCCT (honest, never faked): ARBITRARY / AP242 / PMI STEP import, IGES export/import,
and any body outside the writer's subset — `cc_step_import` DECLINEs to NULL and the engine
falls through to OCCT `STEPControl_Reader`. STEP EXPORT out-of-scope kinds (Ellipse/Bezier
curve, rational spline, Bezier surface) → `canSerialize` DECLINEs, clean error. #8 `drop-occt`
stays blocked (arbitrary import + IGES remain OCCT).

### Native STEP IMPORT WIDENED (`widen-native-step-import`) — multi-solid + B-spline-face + ellipse-curve

The working import slice above was widened along three independent, honestly-gated breadth
tracks (change `widen-native-step-import`, archived `2026-07-06`). Host CTest **29/29** and
sim **`[NIMPORT]` 28/28** (the prior 15 assertions preserved + 13 new). `step_writer.cpp`, the
tessellator, and the `cc_*` ABI are PRISTINE; `src/native/**` stays OCCT-free.

- **T2 — MULTI-SOLID → LANDED (genuine native Compound).** `findManifoldBreps()` collects ALL
  root `MANIFOLD_SOLID_BREP` ids (ascending #id order); `build()` maps each via the existing
  `mapManifoldBrep` and returns a `ShapeBuilder::makeCompound` when there are ≥2 (one root still
  returns a bare Solid — byte-identical prior behaviour). A `hasNestedAssembly()` guard DECLINEs
  any transform tree (`NEXT_ASSEMBLY_USAGE_OCCURRENCE`, `MAPPED_ITEM`,
  `REPRESENTATION_RELATIONSHIP*`, `ITEM_DEFINED_TRANSFORMATION`,
  `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`) → OCCT. Engine `robustlyWatertightMulti` requires
  EVERY member solid watertight/vol>0. Sim: foreign OCCT-written 2-solid file → native Compound,
  `nativeVol=1064 occtVol=1064 rel=2.14e-16`, per-solid watertight, faces/bbox match. Host
  `multi_solid_flat_file_imports_as_compound` (2 solids, exact vol) + `decline_transformed_assembly_returns_null`.
- **T3 — B-SPLINE-FACE round-trip → LANDED (exact, non-fabricated fixture).** The deferred task
  7.4 is now closed: the EXISTING native `build_prism_profile_spline` op emits a watertight
  `B_SPLINE_SURFACE`-face solid that round-trips native-export→native-import EXACT (`vol nat=304.38
  orig=304.38`, watertight, face-count match, `B_SPLINE_SURFACE` present). NO writer change and NO
  fabricated fixture. Host `spline_wall_face_round_trip_exact_volume_and_watertight`.
- **T1a — ELLIPSE curve → PARTIAL (curve kind recognised; solid still DECLINES → OCCT).** The
  reader now maps `ELLIPSE('',#pos,a,b)` to the genuine `EdgeCurve::Kind::Ellipse` (major=a along
  frame X, minor=b along Y; degenerate → decline) — verified by a host edge-mapping test. But
  there is NO watertight NATIVE ellipse-bearing-solid import: a foreign OCCT-authored ellipse-cut
  solid parses (`parsed=1`) yet the ellipse-on-quadric pcurve is out of this slice, fails the
  watertight self-verify (`watertight=0 nativeVol=0`), so the whole solid FALLS BACK to OCCT
  (`ellipse_cut vol nat=942.478 oracle=942.478` is the OCCT fallback matching the oracle). Claimed
  ONLY as "the reader recognises + maps the ELLIPSE curve entity", NOT as native ellipse-solid import.
- **T1b — TOROIDAL_SURFACE → NOT LANDED (documented DECLINE → OCCT).** No native
  `FaceSurface::Kind::Torus` (kinds are Plane/Cylinder/Cone/Sphere/BSpline/Bezier) and the
  tessellator must not be modified, so `surface()` returns `std::nullopt` for `TOROIDAL_SURFACE`
  → NULL → OCCT (`torus native parsed=0`, `fallback torus vol rel=0.00e+00`). Documented honest
  decline, not a native import.

**Residual → OCCT after the widen (before the assemblies slice below):** `TOROIDAL_SURFACE`, ellipse-on-quadric solids,
nested/transformed assemblies, AP242 / PMI, `SURFACE_OF_REVOLUTION`, `TRIMMED_CURVE`,
rational/weighted B-splines, `BEZIER`, non-mm units, all IGES. #8 `drop-occt` stays blocked (a
general STEP/AP242 reader + IGES + a general-curved kernel still block it).

### Native STEP IMPORT WIDENED (`add-native-step-assemblies`) — rigid placed assemblies + AP214/AP242 headers

The `hasNestedAssembly()` blanket decline above is replaced by a genuine assembly builder (change
`add-native-step-assemblies`, archived `2026-07-06`). Host CTest **29/29** NUMSCI OFF (**36/36**
NUMSCI ON), sim **`[NIMPORT]` 33/33** (the prior 28 preserved + 5 new). Exactly 3 files changed
(`step_reader.cpp`, `test_native_step_reader.cpp`, `native_step_import_parity.mm`);
`step_writer.cpp`, the tessellator, `src/engine/**`, and the `cc_*` ABI are PRISTINE; `src/native/**`
stays OCCT-free.

- **RIGID PLACED ASSEMBLY → LANDED (genuine native placed Compound, verified vs OCCT).** `build()`
  now routes a present transform tree to a new `assembly()` builder instead of returning NULL. It
  parses the OCCT-emitted structure — `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` →
  `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION` → `ITEM_DEFINED_TRANSFORMATION` (an
  `AXIS2_PLACEMENT_3D` from/to pair) — composes each component's placement
  `T = frameToWorld(to) ∘ frameToWorld(from)⁻¹`, gates it via `isRigid` (orthonormal `M·Mᵀ≈I` AND
  det≈+1, tol 1e-9 — non-rigid/scaled/mirrored DECLINE), resolves each component representation's
  root `MANIFOLD_SOLID_BREP`(s) STRUCTURALLY by refs (`brepsOfRep`), maps them via the UNCHANGED
  `mapManifoldBrep` in local coords, and pushes each `solid.located(Location{T})` into the Compound
  (native topology carries the placement on edges/faces; no geometry baked). Requires every root
  brep placed EXACTLY once (`placed.size()==findManifoldBreps().size()`) else NULL — never partial,
  never identity-defaulted; the flat multi-solid and single-solid paths are byte-for-byte unchanged.
  Sim vs OCCT `STEPControl_Reader` on an OCCT-authored 2-box assembly (box B carries a non-baked
  `TopLoc_Location`: rotate 0.5 rad about Z + translate(30,5,0), so the writer emits the
  CDSR/REP_REL/ITEM_DEFINED chain, not world-baked coords): 2 solids, `nativeVol=1216 occtVol=1216`
  (mass rel 3.74e-16), bbox maxCornerΔ=0.00e+00 (tol 5e-3), faces 12/12. Host
  `assembly_two_box_placed_compound` + `assembly_rotated_placement_composes`.
- **AP214 / AP242 HEADERS → PINNED (schema-independent, confirmed live).** The reader enters at
  `DATA;` and never gates on `FILE_SCHEMA`, so AP203/AP214/AP242 headers all import — confirmed on a
  real OCCT-`STEPControl_Writer`-authored AP214 (`AUTOMOTIVE_DESIGN`) file (`header=AP214(1) native
  parsed=1 solids=1`). Host `accepts_ap214_and_ap242_file_schema`.
- **DECLINES → OCCT (verified, never fabricated).** Form-B `MAPPED_ITEM`/`REPRESENTATION_MAP`
  (`decline_form_b_mapped_item_returns_null`); any non-rigid transform (the det≈+1 gate); a transform
  tree with no composable placement (`placedCount==0`, lone NAUO — `decline_assembly_without_transform_returns_null`);
  a child rep resolving to 0 or >1 brep, a brep placed twice, or >1 unplaced root; out-of-slice
  component geometry (torus → `parsed=0` → OCCT).

**Residual → OCCT after the assemblies slice (honest, narrowed):** PMI/GD&T, non-rigid/scaled/mirrored
transforms, deep-nested (multi-level) assemblies, Form-B `MAPPED_ITEM`, `TOROIDAL_SURFACE`,
ellipse-on-quadric solids, `SURFACE_OF_REVOLUTION`, `TRIMMED_CURVE`, rational/weighted B-splines,
`BEZIER`, complex/trimmed profiles, non-mm units, all IGES. #8 `drop-occt` stays blocked (a
general STEP/AP242 reader + IGES + a general-curved kernel still block it).

### Native STEP IMPORT WIDENED (`add-native-step-scaled-ap242`) — uniform-scale + mirror placements (T1) + AP242 geometry, PMI skipped (T2)

The rigid-only assembly gate above is widened to two more affine placement classes, and the
AP242 annotation graph is tolerated instead of declining the whole file (change
`add-native-step-scaled-ap242`, archived `2026-07-06`). Host CTest **29/29** NUMSCI OFF (**36/36**
NUMSCI ON), `test_native_step_reader` **20 cases** (4 new); sim **`[NIMPORT]` 41/41** (the prior 33
preserved + 8 new). Exactly 2 native/exchange files changed (`step_reader.cpp`, `step_reader.h`) +
2 tests (`test_native_step_reader.cpp`, `native_step_import_parity.mm`); `step_writer.cpp`, the
tessellator, `src/engine/**`, and the `cc_*` ABI are PRISTINE; `src/native/**` stays OCCT-free.

- **T1 UNIFORM-SCALE / MIRROR PLACEMENT → LANDED (genuine native).** The boolean `isRigid` gate is
  replaced by `classifyPlacement(const math::Transform&)` — a Gram-matrix conformality test
  `MᵀM ≈ k²·I` (scale-relative tol) with a det-sign branch → `Rigid`(k≈1,det>0) |
  `UniformScale`(k>0,det>0) | `Mirror`(det<0); a non-conformal `MᵀM` (non-uniform/shear) or
  degenerate linear part → `nullopt` (DECLINE). `Rigid` reproduces the old `isRigid==true` path
  byte-for-byte. A **UniformScale** component (parsed from a
  `CARTESIAN_TRANSFORMATION_OPERATOR_3D('',#a1,#a2,#o,scale[,#a3])`, or a frame-encoded `k·I`; a
  `_NON_UNIFORM` / unequal `scale1/2/3` triple → DECLINE) rides `solid.located(Location{T})`
  directly and self-verifies with volume `k³·V₀` — k=2 → total vol 2728 = 1000 + 216·8, component
  bbox [30,5,0]..[42,17,12], watertight (host `scaled_assembly_component_scales_by_k_cubed`; sim
  `scaled native cto k³`). A **Mirror** component is orientation-complemented using the EXISTING
  `topo::Orientation` `reversed`/`complemented` algebra (`shape.h`) BEFORE the mirror `Location`,
  so the tessellator's tangent-derived normal (`cross(place(∂u),place(∂v))`, which FLIPS under
  det<0 — see `surface_eval.h`) points OUTWARD again — the mirrored solid self-verifies watertight
  with POSITIVE volume 1216 (not −216) and reflected bbox z∈[−6,0] (host
  `mirrored_assembly_component_watertight_reflected`; sim `mirror native cto reflect`). **No
  tessellator change, no new topology primitive.** A mirrored member that still fails the
  watertight self-verify after compensation → DECLINE → OCCT (never a fabricated flip).
- **T1 HONEST CAVEAT (load-bearing).** OCCT's `STEPControl_Writer` **cannot serialize** a scaled /
  mirror assembly-component location: a 2× component re-imports at NATIVE size (vol 216, not 1728 —
  the scale is silently dropped; the IDT AXIS2 frames stay orthonormal), a `SetMirror` becomes a
  proper 180° ROTATION (det +1), and the trimmed iOS OCCT throws "Location with scaling
  transformation is forbidden" on a scaled `TopLoc_Datum3D`; a `CARTESIAN_TRANSFORMATION_OPERATOR_3D`
  in the IDT slot is schema-invalid and OCCT's reader ignores it. So an OCCT-authored
  "scaled/mirrored assembly" DEGRADES to rigid in the file — there is **no OCCT oracle for genuine
  k³/reflection**. T1 is therefore verified against the standard STEP scale/mirror operator with an
  **analytic** expectation, and separately verified native == OCCT on the degraded-to-rigid
  fixtures (sim `scaled occt cannot author`, `mirror occt cannot author`).
- **T2 AP242 GEOMETRY, PMI SKIPPED → LANDED (genuine native, verified vs OCCT).** The two GLOBAL
  record scans are relaxed so an AP242 file is not declined for carrying PMI. `validateUnitContext()`
  now answers exactly "is the LENGTH unit millimetre?" — a length `SI_UNIT` MUST be `.MILLI.` (mm
  gate UNCHANGED, no tolerance weakened) while a non-length `SI_UNIT` (`.RADIAN.`/`.STERADIAN.`,
  PMI angle/plane-angle contexts) is SKIPPED, not read as non-mm. `assemblyDisposition()` /
  `hasNestedAssembly()` take the assembly path only for a transform relationship that reaches a
  `MANIFOLD_SOLID_BREP`; a `REPRESENTATION_RELATIONSHIP`/`MAPPED_ITEM`/`CDSR` in the
  annotation/draughting graph that reaches no geometric root brep is SKIPPED, and the completeness
  gate is computed over the GEOMETRIC root breps only. An AP242 file (rewritten schema + injected
  PMI/GD&T/draughting incl. a rep-rel graph and plane/solid-angle unit contexts) imports the SOLID
  identically to the OCCT re-import (vol 1000, bbox Δ=0, faces 6/6) with PMI skipped — the
  previously-fatal rep-rel-PMI case now imports instead of declining (host
  `ap242_pmi_skipped_imports_solid`; sim `ap242 pmi-skip native` + `pmi_box mass/bbox/topology`).
- **DECLINES → OCCT (verified, never fabricated).** Non-uniform-scale / shear transforms
  (`decline_non_uniform_shear_assembly_returns_null`; sim `shear` → NULL); PMI/GD&T **semantics**
  (never turned into geometry); Form-B `MAPPED_ITEM`/`REPRESENTATION_MAP`; lone NAUO with no
  composable placement; deep-nested (multi-level) assemblies; out-of-slice component geometry
  (`TOROIDAL_SURFACE` etc.); ellipse-on-quadric solids; non-mm length units.

**Residual → OCCT after the scaled/AP242 slice (honest, narrowed):** PMI/GD&T **semantics**,
**non-uniform-scale / shear** transforms, deep-nested (multi-level) assemblies, Form-B
`MAPPED_ITEM`, `TOROIDAL_SURFACE`, ellipse-on-quadric solids, `SURFACE_OF_REVOLUTION`,
`TRIMMED_CURVE`, rational/weighted B-splines, `BEZIER`, complex/trimmed profiles, non-mm units, all
IGES. #8 `drop-occt` stays blocked (a general STEP/AP242 reader + IGES + a general-curved kernel
still block it).

### Files (#7)

- `src/native/exchange/step_writer.h` / `step_writer.cpp` — OCCT-free ISO-10303-21 text
  formatting + Part-42 emitters (with geometric dedup) + representability gate
  (`canSerialize` / `geometrySupported`) + bottom-up topology walk assigning contiguous
  `#n` ids + AP203 header/units/context wrapper. `writeStepString(solid)` /
  `step_export_native(solid, path)` / `step_can_export_native(solid)`.
- `src/native/exchange/native_exchange.h` — umbrella header (OCCT-free).
- `src/engine/native/native_engine.cpp` — `step_export`: in-scope native body → native
  writer; out-of-scope native body → clean error; foreign body → OCCT `STEPControl_Writer`.
  `step_import` / `iges_export` / `iges_import` unchanged (pure OCCT fall-through).
- `tests/native/test_native_step_writer.cpp` (#19) + `tests/native/test_native_step.cpp`
  (#20) — host Gate-1 structural unit tests (no OCCT).
- `tests/sim/native_step_parity.mm` + `scripts/run-sim-native-step.sh` — sim Gate-2 native-
  write / OCCT-read round-trip (own `main()`; SKIPped by `run-sim-suite.sh`).

## native-healing result table (FIRST shape-healing slice — internal, gates STEP import)

**Honest scope.** The healer (`cybercad::native::heal::healShell`, `src/native/heal/`) is INTERNAL
(no `cc_*` entry point, like SSI) and OCCT-FREE. It fixes EXACTLY four defect classes of a
coincident-within-tolerance face soup / malformed shell — (1) near-coincident **vertex/tolerance
unification** (VertexPool spatial hash), (2) **tolerant sewing** (share an edge only when its endpoints
unified to the same two shared vertices within tolerance — never fabricated), (3) **degenerate removal**
(zero-length edges + sliver / near-zero-area faces), (4) **orientation fix** (flood-fill outward winding
+ global enclosed-volume-sign tie-break) — then SELF-VERIFIES (watertight + `V > 0`) before claiming
success. Anything else (a real gap > tol, a genuinely open shell, missing pcurves, self-intersecting
wires, arbitrary broken industrial B-rep) is reported **UNHEALED** with the measured `maxResidualGap`
and the ORIGINAL shape unchanged → OCCT `ShapeFix`. A **measured win on the in-scope family, not a
guarantee.** This is the gating foundation for a future native STEP IMPORT (imported B-rep arrives with
exactly this coincident-within-tolerance / degenerate / orientation defect family).

**Engine wiring.** `tryNativeHeal` (`src/engine/native/native_heal_hook.h`) runs native → self-verify;
kept only if watertight + valid + `V > 0`; otherwise falls through to the OCCT oracle
`sewAndFix` (`src/engine/occt/occt_shapefix.cpp`, `BRepBuilderAPI_Sewing` → `ShapeFix_Shell`/`ShapeFix_Solid`).
No `cc_*` change; `src/native/**` stays OCCT-free.

**Gate 1 (host, no OCCT) GREEN.** `test_native_heal` — **10 cases, 0 failed** (NUMSCI OFF **and** ON):

| Case | Assertion |
|---|---|
| `heal_soup_cube_watertight` | soup cube → watertight valid solid, V=1 (tess-verified), `nMergedEdges=12` `nMergedVerts=16` `maxResidualGap==0.0` |
| `heal_degenerate_edge` | zero-length edge dropped; cube still heals to V=1 |
| `heal_sliver_face` | near-zero-area sliver dropped; cube still heals to V=1 |
| `heal_flipped_face` | one inward face re-oriented (`nFlipped≥1`) → correct +volume |
| `heal_all_inward_global_flip` | all-inward cube → global sign flip → outward |
| `heal_vertex_unify_merges_within_tol` | near-coincident vertices merged |
| `heal_vertex_unify_rejects_beyond_tol` | vertices 1.0 apart NOT merged |
| `heal_open_shell_unhealed` | missing face → `Unhealed(OpenShell)`, input unchanged |
| `heal_beyond_tolerance_gap_unhealed` | gap 1e-2 ≫ tol 1e-4 → `Unhealed(GapBeyondTolerance)`, `maxResidualGap>tol`, input unchanged |
| `heal_never_weakens_tolerance` | a 1e-2 gap is never force-closed at tol 1e-4 |

**Gate 2 (sim native-vs-OCCT parity) GREEN.** `run-sim-native-heal.sh` — native `healShell` vs OCCT
`sewAndFix` on identical soups: **`[NHEAL]` 4 passed / 0 failed**.

| Fixture | Native | OCCT oracle | Verdict |
|---|---|---|---|
| soup-cube | V=1.00000 watertight=1 | V=1.00000 valid=1 | heal parity (in-scope) |
| flipped-face | V=1.00000 watertight=1 | V=1.00000 valid=1 | heal parity (in-scope) |
| beyond-tol-gap | UNHEALED reason=1 (`GapBeyondTolerance`) residual 0.0255 | valid=0 watertight=0 | honest UNHEALED matches OCCT |
| open-shell | UNHEALED reason=2 (`OpenShell`) | valid=0 watertight=0 | honest UNHEALED matches OCCT |

On the un-healable fixtures the native UNHEALED verdict MATCHES OCCT leaving the shell open at the same
tolerance — no fabricated closure, no weakened tolerance.

### Files (native healing)

- `src/native/heal/` — `heal.cpp` + `heal.h`, `native_heal.h`, `heal_result.h`, `face_soup.h`,
  `vertex_unify.h`, `tolerant_sew.h`, `degenerate.h`, `orient.h`, `assemble_shell.h`, `self_verify.h`
  (all OCCT-free; worst function cognitive complexity 14, `findRepresentative`, Acceptable band).
- `src/engine/native/native_heal_hook.h` — engine-internal `tryNativeHeal` (native → self-verify → OCCT).
- `src/engine/occt/occt_shapefix.cpp` — the OCCT oracle `sewAndFix` (sim only; the sole OCCT TU here).
- `tests/native/test_native_heal.cpp` (#21) — host Gate-1 (no OCCT).
- `tests/sim/native_heal_parity.mm` + `scripts/run-sim-native-heal.sh` — sim Gate-2 native-vs-OCCT
  (own `main()`; SKIPped by `run-sim-suite.sh`).
- Living change `add-native-shape-healing` archived to `openspec/specs/native-healing`.

## numeric-foundations result table (native-rewrite capability #2)

**Substrate adopted, not vendored.** NumPP (`/Users/leonardoaraujo/work/NumPP`) + SciPP
(`/Users/leonardoaraujo/work/SciPP`) — the org's C++20, MIT NumPy/SciPy ports — are the
kernel's OCCT-free numeric substrate, referenced by absolute path exactly like OCCT.
CPU-only (no NumPP GPU/BLAS backend), consuming the SciPP `optimize` / `linalg`
(+ `spatial` / `integrate`) subset with **`special` + `stats` EXCLUDED** (a Homebrew-libc++
ISO-29124 gap confined to `src/special/`, unused by the kernel). The whole numerics module
is gated by a `CYBERCAD_HAS_NUMSCI` CMake option (default **OFF**): with it OFF, no
NumPP/SciPP header or source compiles and every existing native suite is byte-for-byte
unaffected. The substrate is built as a static archive `libnumsci_<target>.a` by
`scripts/build-numsci.sh {host|iossim}` and linked via `-DCYBERCAD_HAS_NUMSCI=ON`. Verdict
from `docs/EVAL-numpp-scipp.md`: **GO WITH HARDENING** — this adoption realizes the eval's
~**60–75% effort saving** on #2 (→ ~0.15–0.35 py).

**What is native.** A thin OCCT-free facade (`src/native/numerics/`) exposes:
- **Generic solvers** over SciPP — scalar root (`newton` / `brentq`), nonlinear system
  (`fsolve`), `minimize` (BFGS), `least_squares` (Levenberg-Marquardt), dense `solve` /
  `lstsq`.
- **Closest-point / projection (the `Extrema` on-ramp)** — point→curve (`Line` / `Circle` /
  B-spline / NURBS) and point→surface (`Plane` / `Cylinder` / `Cone` / `Sphere` / B-spline /
  NURBS / Torus) nearest-parameter projection over `src/native/math`, seeded by a coarse
  multi-start parameter grid and refined per-seed with SciPP `minimize` clamped to bounds,
  returning the GLOBAL-best foot (param, foot point, distance).

**Host gate (Gate 1):** `test_native_numerics` (Homebrew clang, `-std=c++20`,
`CYBERCAD_HAS_OCCT=OFF`, built under `CYBERCAD_HAS_NUMSCI=ON`) — **22 internal assertions
PASS, 0 failed**: solver known-values (`brentq(x²−2)=√2`, `newton(cos x−x)=0.739085`,
`fsolve` 2×2 residual < 1e-6, BFGS Rosenbrock → (1,1), `least_squares` line fit, `solve`
3×3 SPD, `lstsq` 4×2), closed-form closest-point on line / circle / plane / cylinder /
sphere, and a bicubic B-spline surface + curve case vs a dense brute-force grid.

**Native-vs-OCCT `Extrema` parity gate (Gate 2)** — `tests/sim/native_numerics_parity.mm`,
booted iOS simulator, arm64: native nearest-`t` vs OCCT `Extrema_ExtPC` (curves) and native
nearest-`(u,v)` vs OCCT `Extrema_ExtPS` (surfaces) at sampled 3D targets. **All 22 `[NNUM]`
cases PASS.** `dDist` = |native dist − OCCT dist|, `dPoint` = closest-point separation,
`dU`/`dV`/`dParam` = parameter deltas:

| Case | dDist | dPoint | dU / dV / dParam |
|---|---|---|---|
| plane #0–#3 | ≤ 1.776e-15 | ≤ 1.221e-10 | dU ≤ 3.783e-11, dV ≤ 1.216e-10 |
| cylinder #0–#3 | ≤ 8.882e-16 | ≤ 1.707e-10 | dU ≤ 5.689e-11, dV = 0 |
| sphere #0–#3 | ≤ 8.882e-16 | ≤ 1.209e-10 | dU ≤ 3.206e-11, dV ≤ 2.355e-11 |
| bspline_surf #0–#2 | ≤ 4.441e-16 | ≤ 3.136e-10 | dU ≤ 9.666e-12, dV ≤ 1.014e-10 |
| **bspline_surf #3** (corner u=v=0) | 0 | **3.946e-08** | dU=dV=7.595e-09 *(largest deviation)* |
| bspline_curve #0–#4 | 0 | ≤ 8.434e-11 | dParam ≤ 2.087e-11 |
| facade_eval_bsurf | 0 | 0 | dU=dV=0 |

Analytic (plane / cylinder / sphere) feet are fp-exact vs OCCT `Extrema` (dDist at machine
epsilon, dPoint ≤ 1.7e-10 — parameter-space round-off); B-spline feet are within a tight
fp64 tolerance, the single largest deviation being `bspline_surf#3` (dPoint 3.946e-08 at
the domain corner u=v=0, still ~1e-8). Build: compiled + linked clean (warnings only), ran
in the booted simulator, exit 0.

**Build status (both targets).**
- **HOST, `CYBERCAD_HAS_NUMSCI=OFF`** (clang++ 22.1.8, build `build-verify-numsci-off`):
  built clean; CTest **22/22 passed, 0 failed** (10.73s); `test_native_numerics` correctly
  ABSENT (option OFF); all prior native suites pass.
- **HOST, `CYBERCAD_HAS_NUMSCI=ON`** (build `build-verify-numsci-on`, linking
  `libnumsci_host.a` from `scripts/build-numsci.sh host` — compiled OK 77/77 TUs: 66 NumPP
  + 11 SciPP): built clean incl. `test_native_numerics`; CTest **23/23 passed, 0 failed**
  (9.48s); all prior native suites UNCHANGED + `test_native_numerics` passed.
- **arm64-iOS-simulator**: the NumPP core + SciPP `optimize`/`linalg` subset +
  `src/native/numerics` compile + link for `arm64-apple-ios16.0-simulator` (0 undefined
  symbols); the parity harness ran in the booted simulator, exit 0.
- **SIM SUITE** (`scripts/run-sim-suite.sh`, OCCT-only): **221 passed, 0 failed**;
  determinism serial==parallel bit-reproducible across all ops. `native_numerics_parity.mm`
  is on the SKIP list (own `main()`), so the 221 count is unchanged.

**Deferred (recorded, NOT blocking the bar).** This slice is **single-target
closest-point** only. (1) **Multiple-extrema enumeration** — the projector returns the
global-best foot, not the full set of stationary points OCCT `Extrema` can enumerate.
(2) **Curve-curve and surface-surface distance** (`Extrema_ExtCC` / `Extrema_ExtSS`) are
NOT implemented (only point→curve and point→surface). (3) On-sim numeric caveat: the
`bspline_surf#3` domain-corner foot (dPoint ~4e-8) is the honest ceiling of the current
multi-start grid density — well inside the harness tolerance, but the largest observed
deviation. **SSI (near-tangent surface-surface intersection) is NOT bought by this
adoption** — it remains capability #5.

### Files (numeric-foundations)

Substrate (external, absolute-path, NOT vendored): NumPP (`/Users/leonardoaraujo/work/NumPP`),
SciPP (`/Users/leonardoaraujo/work/SciPP`).

- `src/native/numerics/numerics.h` / `numerics.cpp` — the single facade boundary (the only
  TU that includes NumPP/SciPP): native↔`ndarray` marshalling + the generic-solver wrappers
  (root / `fsolve` / `minimize` / `least_squares` / `solve` / `lstsq`).
- `src/native/numerics/closest_point.h` — the typed closest-point/projection layer
  (`project_point_to_curve` / `project_point_to_surface` + elementary/B-spline/NURBS/Torus
  overloads), multi-start grid seeding + per-seed SciPP refine, global-best foot.
- `src/native/numerics/native_numerics.h` — umbrella header (OCCT-free, guarded by
  `CYBERCAD_HAS_NUMSCI`).
- `scripts/build-numsci.sh` — builds `libnumsci_{host,iossim}.a` (NumPP CPU-only full TU set
  + SciPP `optimize`/`linalg` subset, `special`/`stats` EXCLUDED).
- `tests/native/test_native_numerics.cpp` — host Gate-1 (no OCCT; registered in CTest only
  under `CYBERCAD_HAS_NUMSCI`).
- `tests/sim/native_numerics_parity.mm` + `scripts/run-sim-native-numerics.sh` — sim Gate-2
  native-vs-OCCT `Extrema` parity (own `main()`; SKIPped by `run-sim-suite.sh`).

## SSI-S1 result table (surface-surface intersection, analytic stage — SSI-ROADMAP S1)

**Stage S1 of the SSI → curved-booleans sub-roadmap** ([`openspec/SSI-ROADMAP.md`](../openspec/SSI-ROADMAP.md)).
Closed-form intersection curves for the elementary-surface family — the analytic-first
slice that unblocks elementary-pair curved booleans (S5) and is the on-ramp to the
marching kernel (S2 seeding → S3 marching → S4 robustness). OCCT-free, header-only under
`src/native/ssi/`, built over `src/native/math` only (IntAna-style closed form; NO
`GeomAPI` / NO `numsci`). SSI is **INTERNAL** — no `cc_*` entry point; parity asserted at
the `cybercad::native::ssi` C++ boundary, like native-math / native-topology.

**Host gate (Gate 1):** `test_native_ssi` (Homebrew clang, `-std=c++20`,
`CYBERCAD_HAS_OCCT=OFF`) — **11 cases, 0 failed** (plane_plane_line, plane_sphere_circle,
plane_cylinder, plane_cone, plane_torus, sphere_sphere, sphere_cylinder_coaxial,
sphere_cone_coaxial, cylinder_cylinder, cylinder_cone_coaxial, deferred_not_analytic).
Each supported pair asserts curve kind + parameters against the closed form AND every
sampled point on both surfaces within tol; the deferred case asserts `NotAnalytic` +
empty curves. CTest **23/23** with `CYBERCAD_HAS_NUMSCI=OFF` (default host build,
`build-ssi-verify-off`) and **24/24** with `CYBERCAD_HAS_NUMSCI=ON` (`build-ssi-verify-on`,
adding `test_native_numerics`). The SSI test does NOT require NUMSCI (header-only, math-only).

**Native-vs-OCCT `GeomAPI_IntSS` parity gate (Gate 2)** — `tests/sim/native_ssi_parity.mm` /
`scripts/run-sim-native-ssi.sh`, booted iOS simulator, arm64: the same operands built as
OCCT `Geom_*Surface`, run through `GeomAPI_IntSS`, native curve(s) compared on kind +
on-surface residual + coincidence + coverage + branch count. **18 pairs, 0 failed** — 17
analytic-native + 1 honest deferral. `onSurf` = max residual of native curve samples on
BOTH input surfaces; `coin` = native-vs-OCCT curve coincidence:

| Pair | native / OCCT | kind | onSurf | coin | cover | tol |
|---|---|---|---|---|---|---|
| plane ∩ plane | 1 / 1 | Line | 0 | 0 | 0 | 1e-9 |
| plane ∩ sphere | 1 / 1 | Circle | 3.79e-15 | 3.82e-15 | 8.31e-13 | 1e-9 |
| plane ⟂ cyl | 1 / 1 | Circle | 1.91e-15 | 1.91e-15 | 4.16e-13 | 1e-9 |
| plane ∠ cyl | 1 / 1 | Ellipse | 1.42e-15 | 2.57e-15 | 5.85e-13 | 1e-8 |
| plane ∥ cyl | 2 / 2 | Line/Line | 5.55e-17 | 0 | 0 | 1e-9 |
| plane ⟂ cone | 1 / 1 | Circle | 3.59e-15 | 3.59e-15 | 8.31e-13 | 1e-7 |
| plane ∠ cone | 1 / 1 | Ellipse | 2.44e-15 | 5.37e-15 | 1.01e-12 | 1e-6 |
| plane ∥ gen cone | 1 / 1 | Parabola | 2.03e-15 | 9.74e-16 | 5.66e-16 | 1e-6 |
| plane steep cone | 2 / 2 | Hyperbola×2 | 5.61e-16 | 4.45e-16 | 5.24e-16 | 1e-6 |
| plane ⟂ torus | 2 / 2 | Circle/Circle | 2.84e-15 | 2.84e-15 | 1.04e-12 | 1e-9 |
| plane ∋ axis torus | 2 / 2 | Circle/Circle | 9.93e-16 | 1.67e-15 | 2.14e-13 | 1e-8 |
| sphere ∩ sphere | 1 / 2 | Circle *(OCCT arc-splits into 2)* | 4.12e-15 | 3.82e-15 | 8.28e-13 | 1e-9 |
| coaxial sphere ∩ cyl | 2 / 2 | Circle/Circle | 1.88e-15 | 2.39e-15 | 6.41e-13 | 1e-9 |
| coaxial sphere ∩ cone | 2 / 3 | Circle/Circle *(OCCT arc-split)* | 3.14e-15 | 2.78e-15 | 7.55e-13 | 1e-7 |
| coaxial cyl ∩ cone | 2 / 3 | Circle/Circle *(OCCT arc-split)* | 1.79e-15 | 1.52e-15 | 4.27e-13 | 1e-7 |
| parallel cyl ∩ cyl | 2 / 2 | Line/Line | 1.26e-15 | 0 | 0 | 1e-9 |
| coaxial cyl ∩ cyl | 0 / 0 | coincident *(detected)* | 0 | 0 | 0 | 1e-9 |
| **skew cyl ∩ cyl** | **NotAnalytic** / 7 | *native defers; OCCT 7 Ellipse* | — | — | — | deferred |

**Curve-count deltas** (sphere∩sphere, coaxial sphere∩cone / sphere∩cyl, coaxial cyl∩cone)
arise from OCCT arc-splitting the SAME conic into multiple bounded arcs — curve TYPES match
in every analytic pair. All onSurf / coin residuals are at machine-epsilon scale (≤ ~4e-15),
well inside each pair's tolerance.

**Honest deferral (never faked).** `skew cyl∩cyl` is `NotAnalytic` by design: general skew
cylinder/cylinder is NOT a degree-≤2 closed-form reduction (it is a planar quartic; OCCT emits
7 Ellipse curves). It PASSES the parity harness as a documented deferral. By the same rule the
following also return `NotAnalytic` (route to S2/S3/OCCT): general cone∩cone, non-coaxial
cone∩cyl / sphere∩cyl / sphere∩cone, oblique plane∩torus (spiric quartic), torus∩curved, and
all freeform (NURBS/B-spline/Bézier) pairs. `NotAnalytic` + empty `curves` IS the contract with
S2/S3/OCCT — the S5 curved-boolean caller must route these to marching or OCCT.

### Files (SSI-S1)

Native library (OCCT-free, header-only, `src/native/ssi/`):

- `curve.h` — `CurveKind` (Point/Line/Circle/Ellipse/Parabola/Hyperbola), `IntersectionCurve`
  (frame `Ax3` + radius / semi-axes a,b / focal / hyperbola branch, `value(t)` evaluator
  mapping 1:1 onto OCCT `Geom_*`), `IntersectionStatus` (Ok / NoIntersection / Coincident /
  NotAnalytic) + `IntersectionResult{status, curves}`, and small conic-frame constructors.
- `tolerance.h` — scale-derived linear/angular epsilons + `parallelDirs` / `perpendicularDirs`.
- `dispatch.h` — `SurfaceKind` classify + symmetric operand canonicalization (plane→sphere→
  cylinder→cone→torus) routing to a closed-form handler or `NotAnalytic`.
- `plane_conics.h` — plane∩{plane, sphere, cylinder, cone} closed-form conics.
- `plane_torus.h` — plane∩torus for the two closed-form families (⟂ axis concentric circles,
  ∋ axis meridian circles); oblique spiric quartic → `NotAnalytic`.
- `quadric_pairs.h` — sphere∩sphere + coaxial sphere∩cyl / sphere∩cone / cyl∩cone +
  coaxial/parallel cyl∩cyl.
- `native_ssi.h` — umbrella header + namespace / contract doc (analytic == false is the S2/S3/
  OCCT hand-off).

Tests:

- `tests/native/test_native_ssi.cpp` — host Gate-1 analytic + on-surface + deferral gate (no OCCT).
- `tests/sim/native_ssi_parity.mm` + `scripts/run-sim-native-ssi.sh` — sim Gate-2 native-vs-OCCT
  `GeomAPI_IntSS` parity (own `main()`; on the `run-sim-suite.sh` SKIP list, so the 221 count is
  unchanged — it is a `.mm` file already excluded by the `*.cpp` find; the SKIP entry is
  intent-documenting).

### Deferred to S3–S4 (recorded, not blocking the S1/S2 bar)

- **S2 subdivision seeding — DONE at the bar (transversal).** See the SSI-S2 result table below.
- **S3 marching-line tracer (WLine) — DONE at the bar (transversal).** See the SSI-S3 result table below.
- **S4 tangent / degeneracy robustness (the moat) — NEXT** and **S5 curved booleans via SSI**
  follow, per `openspec/SSI-ROADMAP.md`.

## SSI-S2 result table (subdivision seeding — SSI-ROADMAP S2)

**Stage S2 of the SSI → curved-booleans sub-roadmap** ([`openspec/SSI-ROADMAP.md`](../openspec/SSI-ROADMAP.md)).
Finds ≥1 seed point per **transversal** intersection branch for the **freeform**
(NURBS / Bézier / B-spline) and **non-closed-form quadric** pairs that S1 defers as
`NotAnalytic`: recursive patch-AABB-overlap subdivision → candidate regions → refine to a
point with `least_squares(S₁(u₁,v₁) − S₂(u₂,v₂) = 0)` on the numerics substrate → 3D/param
dedup to ~one seed per branch. OCCT-free in `src/native/ssi/` (`cybercad::native::ssi`); the
refine is guarded by **`CYBERCAD_HAS_NUMSCI`**. SSI is **INTERNAL** — no `cc_*` entry point;
asserted at the C++ boundary. **Scope is TRANSVERSAL only** (`n₁ × n₂ ≠ 0`); near-tangent /
coincident / degenerate configurations are **deferred to S4** (counted in
`SeedSet.deferredTangent`, reported not faked).

**Host gate (Gate 1):** `test_native_ssi_seeding` (Homebrew clang, `-std=c++20`,
`CYBERCAD_HAS_NUMSCI=ON`) — **6 cases, 0 failed**: skew (orthogonal, unequal-radius) cylinders →
**2** transversal loops; two crossing spheres → **1** circle branch; sphere piercing a freeform
Bézier bump → **1** loop; parallel disjoint planes → **0** branches (pruned, no false seed);
externally tangent spheres → `deferredTangent ≥ 1` (an S4 gap, **NO fabricated seed**); and a
deeper-resolution case recovering a small loop that coarse subdivision misses (the recall knob).
Each seed is asserted on BOTH surfaces (`onSurfResidual ≤ tol`), transversal (`crossingSine >
0.1`), and dedup'd to the known branch count. **NUMSCI OFF** (default host build): CTest **23/23**,
`test_native_ssi_seeding` + `test_native_numerics` correctly ABSENT (NUMSCI-gated). **NUMSCI ON**:
CTest **25/25**, adding `test_native_ssi_seeding` (#25) and `test_native_numerics` (#24);
`test_native_ssi` (#19) and `test_native_tessellate` (#10) stay green.

**Native-vs-OCCT `GeomAPI_IntSS` recall parity gate (Gate 2)** — `tests/sim/native_ssi_seeding_recall.mm`
/ `native_ssi_seeding_parity.mm`, booted iOS simulator, arm64: the same operands built as OCCT
`Geom_*Surface`, run through `GeomAPI_IntSS`; native per-pair **branch recall** =
(native branches carrying ≥1 seed) ÷ (analytic transversal branch count). All pairs are the
freeform / skew-quadric cases S1 defers as `NotAnalytic`. Seed on-surface residual measured via
`GeomAPI_ProjectPointOnSurf::LowerDistance` against BOTH OCCT surfaces. **3/3 transversal branches
recalled at recall 1.00; 0 branches deferred (tangent = 0 everywhere); max seed on-surface residual
= 3.51e-16** (well under the 1e-6 `onSurfTol`):

| Pair | native / OCCT NbLines | recall | tangent | worst onSurf | note |
|---|---|---|---|---|---|
| skew cyl (unequal) | 2 / 3 | 1.00 | 0 | 3.51e-16 | TRANSVERSAL; 2 loops; OCCT NbLines=3 = arc-split of same locus |
| crossing spheres | 1 / 2 | 1.00 | 0 | 7.85e-17 | TRANSVERSAL; 1 circle; OCCT NbLines=2 = arc-split |
| sphere ∩ Bézier bump | 1 / 2 | 1.00 | 0 | 6.50e-16 | TRANSVERSAL; 1 loop; OCCT NbLines=2 = arc-split |

`OCCT NbLines = N` is OCCT's **arc-split** line count, NOT the analytic branch count the recall
denominator uses — exactly the same arc-split delta seen at S1. No regressions:
`scripts/run-sim-suite.sh` rebuilt the `CyberCadKernel.xcframework` (both slices, 27 TUs each
including the new `src/native/ssi/seeding.cpp`; no NUMSCI define so the guarded `seed_intersection`
body compiles to nothing) and ran **221 passed, 0 failed** in the booted simulator.

**Transversal-native vs near-tangent-deferred (honest).** What S2 delivers native: a seed on both
surfaces for every TRANSVERSAL branch of the tested freeform / skew-quadric pairs (recall 1.00).
What S2 defers to S4 (never faked): near-tangent / coincident / degenerate seeding — the refine
ill-conditions (`‖n₁ × n₂‖ ≈ 0`), so the region increments `SeedSet.deferredTangent` instead of
emitting a seed (the externally-tangent-spheres host case exercises exactly this). Completeness is a
**measured recall** figure, not a blind 100%: too-shallow subdivision can miss a small loop — the
acknowledged failure mode — and `minPatchFrac` (default 1/32) is the resolution/recall knob (the
deeper-resolution host case demonstrates recovery).

### Files (SSI-S2)

Native library (OCCT-free, `src/native/ssi/`; refine guarded by `CYBERCAD_HAS_NUMSCI`):

- `seed.h` — `Seed { u1,v1,u2,v2; point; onSurfResidual; crossingSine }`,
  `SeedSet { seeds; candidateRegions; deferredTangent; branchCount() }`, recall-report struct.
- `patch_bounds.h` — per-patch AABB: freeform = control-net convex hull ∩ sampled-with-Lipschitz-margin
  (both sound); elementary + torus = sampled + derivative safety margin; disjoint-AABB prune test.
- `seeding.h` — subdivision + dedup declarations + the S3 hand-off contract doc; DECLARES
  `seed_intersection` (definition behind `CYBERCAD_HAS_NUMSCI` in `seeding.cpp`).
- `seeding.cpp` — recursive patch-pair subdivision + prune, candidate regions, `least_squares`
  refine + clamp + on-surface re-check, near-tangent deferral, spatial/topological branch dedup.
- `native_ssi.h` — umbrella header now `#include`s `seed.h` / `patch_bounds.h` / `seeding.h`.

Tests:

- `tests/native/test_native_ssi_seeding.cpp` — host Gate-1 (no OCCT; NUMSCI-gated).
- `tests/sim/native_ssi_seeding_recall.mm` + `native_ssi_seeding_parity.mm` — sim Gate-2 native-vs-OCCT
  recall parity (own `main()`; `.mm` files already excluded by the `run-sim-suite.sh` `*.cpp` find, so
  the 221 count is structurally unchanged — the SKIP entries are intent-documenting).

### Deferred to S3–S4 / OCCT (recorded, not blocking the S2 bar)

- **S3 marching-line tracer (WLine) — NEXT.** From each S2 seed, walk the curve (tangent = n₁×n₂,
  adaptive step, re-project onto both surfaces via the substrate) and fit a B-spline; the `SeedSet` is
  the input contract (one WLine per seed).
- **Near-tangent** seeding (`n₁ × n₂ → 0`) → **S4**; S2 reports these as `deferredTangent`, never faked.
- **Coincident / overlapping-surface** detection and **degenerate** (cusp / singular param) seeding →
  **S4** + OCCT fallback.
- **Closing the completeness gap** (guaranteeing every small loop is found) — S2 only **measures**
  recall; hardening subdivision depth/heuristics is ongoing.

## SSI-S3 result table (marching-line tracer / WLine — SSI-ROADMAP S3)

**Stage S3 of the SSI → curved-booleans sub-roadmap** ([`openspec/SSI-ROADMAP.md`](../openspec/SSI-ROADMAP.md)).
From each S2 seed, walks the intersection curve: predictor `t = normalize(n₁×n₂)` → adaptive
step (shrink on corrector failure / deflection / slid-back step; grow on a smooth cheap step)
→ **corrector** re-projecting each node onto BOTH surfaces via the numerics substrate
(`least_squares`, m=n=4 well-posed with an along-tangent advance residual, clamped to each
range) → march both directions and stitch → close (`Closed`) / exit a boundary (`BoundaryExit`)
→ dedup retraced branches → fit a clamped-uniform B-spline through the polyline. OCCT-free in
`src/native/ssi/{marching.h,marching.cpp}` (`cybercad::native::ssi`); the corrector, adaptive
step, and B-spline fit are guarded by **`CYBERCAD_HAS_NUMSCI`** (`marching.cpp` is an EMPTY TU
with NUMSCI off — no NumPP/SciPP refs; `marching.h` types are always visible). SSI is
**INTERNAL** — no `cc_*` entry point; asserted at the C++ boundary. **Scope is TRANSVERSAL only**
(`n₁ × n₂ ≠ 0`); near-tangent branches trace *up to* the tangent (`NearTangent`, counted in
`nearTangentGaps`), coincident / branch-point / self-intersection are **deferred to S4** — never
faked.

**Host gate (Gate 1):** `test_native_ssi_marching` (Homebrew clang, `-std=c++20`,
`CYBERCAD_HAS_NUMSCI=ON`) — **7 cases, 0 failed**: crossing spheres → closed circle; plane∩sphere
→ closed circle; skew cylinders → 2 closed loops (+ seam wrap); sphere∩Bézier bump → loop on both
freeform+sphere; ramp B-spline∩plane → open segment exiting the boundary (`BoundaryExit`);
externally tangent spheres → NO curve (deferred by S2, S3 fabricates nothing); duplicate seed →
1 WLine (dedup). Every node asserted on BOTH surfaces < 1e-6; B-spline fit error < 1e-3. **NUMSCI
OFF** (default host build): CTest **23/23**, the three NUMSCI-gated tests correctly ABSENT.
**NUMSCI ON**: CTest **26/26**, adding `test_native_numerics` (#24), `test_native_ssi_seeding`
(#25), `test_native_ssi_marching` (#26); `test_native_ssi` (#19) and `test_native_tessellate`
(#10) stay green.

**Native-vs-OCCT `IntPatch` / `GeomAPI_IntSS` curve-parity gate (Gate 2)** —
`tests/sim/native_ssi_marching_parity.mm`, booted iOS simulator, arm64: the same operands built
as OCCT surfaces, run through `GeomAPI_IntSS` (`IntPatch` walker); per pair the native traced
WLine set is compared to OCCT branch-for-branch on branch count, closure, sampled point-to-OCCT-
curve distance (onCurve), on-both-surfaces residual (onSurf), and arc-length. **5 pairs, 9
branches, 0 failed — all TRANSVERSAL, all FULLY TRACED, 0 near-tangent-truncated:**

| Pair | branches nat/occt | closed nat/occt | onCurve | onSurf | lenΔ (nat / occt) | nt | seeds |
|---|---|---|---|---|---|---|---|
| bspline ∩ bspline | 1 / 1 | 1 / 1 | 1.86e-07 | 2.71e-08 | 4.35e-06 (2.8171 / 2.8171) | 0 | 1 |
| bspline ∩ plane | 4 / 4 | 0 / 0 | 5.75e-09 | 1.41e-11 | 2.28e-03 (0.6917 / 0.6933) | 0 | 4 |
| skew cyl unequal | 2 / 2 | 2 / 2 | 1.60e-06 | 6.81e-07 | 4.00e-05 (9.1521 / 9.1525) | 0 | 2 |
| sphere ∩ sphere | 1 / 1 | 1 / 1 | 1.43e-07 | 1.23e-07 | 1.58e-05 (5.4413 / 5.4414) | 0 | 1 |
| sphere ∩ bezier | 1 / 1 | 1 / 1 | 1.25e-07 | 3.37e-08 | 8.31e-05 (2.3696 / 2.3698) | 0 | 1 |

Aggregate: **9 branches / 5 pairs, all TRANSVERSAL fully-traced, 0 near-tangent-truncated**
(deferred to S4). **Closed-loop match 5/5** — every OCCT closed loop is reproduced as a `Closed`
native WLine (bspline∩plane correctly 0-closed / 4-open segments). Worst deltas: max
on-OCCT-curve **1.60e-06** (skew-cyl-unequal), max on-surface **6.81e-07** (skew-cyl-unequal),
max length delta **2.28e-03** abs / ~0.33% rel (bspline∩plane — the only sub-mm-order length gap,
still within the deflection/step tol). No regressions: `marching.cpp` is additive/guarded
(empty TU in the default build), `CMakeLists.txt` only APPENDS `test_native_ssi_marching` under
the existing `CYBERCAD_HAS_NUMSCI` block, and `run-sim-suite.sh` only adds
`native_ssi_marching_parity.mm` (own `main()`) to the SKIP list → `scripts/run-sim-suite.sh`
stays **221 passed, 0 failed**.

**Transversal-native vs near-tangent-deferred (honest).** What S3 delivers native: a full WLine
(`Closed` / `BoundaryExit`) for every TRANSVERSAL branch of the tested freeform / skew-quadric /
elementary pairs, matching OCCT branch count and closure with the deltas above. What S3 defers to
S4 (never faked): a near-tangent march is traced up to the tangent and marked `NearTangent`
(counted in `nearTangentGaps`, never a point past it); coincident / branch-point / self-
intersection marching is out of scope. Automatic densify-and-refit on a too-loose B-spline fit is
not yet wired — the polyline stays the on-surface ground truth (the fit is a convenience curve) —
a recorded follow-up.

### Files (SSI-S3)

Native library (OCCT-free, `src/native/ssi/`; corrector / step / fit guarded by `CYBERCAD_HAS_NUMSCI`):

- `marching.h` — result types (always visible, data-only): `WLinePoint { point; (u1,v1,u2,v2) }`,
  `WLine { points; curve; onSurfResidual; status; branchId }`,
  `TraceStatus { Closed, BoundaryExit, NearTangent, Failed }`, `FittedBSpline`,
  `TraceSet { lines; tracedBranches; nearTangentGaps; dedupedRetraces; … }`; tracer declarations +
  the S4 deferral / S5 hand-off contract doc.
- `marching.cpp` — predictor / corrector / adaptive step / march-both-directions + stitch / loop-
  closure + boundary-exit termination / dedup / B-spline fit (whole TU behind `CYBERCAD_HAS_NUMSCI`;
  empty in the default build).
- `native_ssi.h` — umbrella header now `#include`s `marching.h`.

Tests:

- `tests/native/test_native_ssi_marching.cpp` — host Gate-1 (no OCCT; NUMSCI-gated).
- `tests/sim/native_ssi_marching_parity.mm` — sim Gate-2 native-vs-OCCT curve parity (own `main()`;
  on the `run-sim-suite.sh` SKIP list, so the 221 count is structurally unchanged).

### Deferred to S4 / S5 / OCCT (recorded, not blocking the S3 bar)

- **Near-tangent** marching (`n₁ × n₂ → 0`: higher-order predictor / step control to cross a
  tangent) → **S4**; S3 traces up to the tangent and reports `nearTangentGaps`, never faked.
- **Branch-point splitting** (a singular crossing of two intersection branches) → **S4**; S3
  traces up to it and flags, never attempts the split.
- **Self-intersection resolution** and **coincident / overlapping-surface** curve extraction → **S4** + OCCT fallback.
- **Automatic densify-and-refit** on a too-loose B-spline fit — follow-up; the polyline is retained as ground truth.
- **S5 curved booleans** — using the traced WLines to split curved faces, classify fragments, and
  assemble the watertight shell → **S5** (this stage only produces the WLines S5 consumes).
  (S5 has since landed native slices S5-a/b/c/d — see the `native-booleans` **S5-a/b/c/d** result-table
  row — through-drill cyl∩cyl COMMON/FUSE/CUT + sphere∩sphere COMMON/FUSE/CUT (op-set COMPLETE 3/3) + branched Steinmetz COMMON/FUSE/CUT (op-set COMPLETE 3/3), native-pass=12.)

## SSI-S4 result section (coincident-region + tangent-contact CLASSIFICATION — SSI-ROADMAP S4-a/b)

**Scope bound (honest):** S4-a/b are the **detection + classification** layers of the S4
moat. They TYPE the degeneracy and emit the point/curve/region where determinable. They do
**NOT** march through a tangency and do **NOT** fabricate a curve across a degeneracy — that
is the **S4-c…f marching core** (out of scope here). A near-tangent transversal is classified
as such and handed on (still an S4-c → OCCT gap), never traced.

OCCT-free, `src/native/ssi/`; the seeded-path parts behind `CYBERCAD_HAS_NUMSCI`; INTERNAL
(no `cc_*`). The native code returns `Undecided` / `None` / empty on any non-robust
classification — the ENGINE owns the OCCT fallback + self-verify.

### S4-a — coincident / overlapping-surface detection + typed region
- Analytic: closed-form `FullSurfaceSame` predicates for ALL elementary families (plane
  same-normal±same-offset, coaxial-equal cyl/cone, same sphere, same torus), folding the
  pre-existing same-sphere / coaxial-equal-cyl `Coincident`. Returns a typed
  `CoincidentRegion{FullSurfaceSame}` (status `IntersectionStatus::Coincident` unchanged,
  now backed by the region). A shifted/rotated/resized near-miss classifies `None` (no
  false coincidence).
- Seeded (`CYBERCAD_HAS_NUMSCI`): grid-sample a candidate region — a sample AGREES iff
  on-both residual ≤ `onSurfTol` AND ‖n_A×n_B‖ ≤ `tangentSinTol`; if the whole grid agrees,
  grow the region to the agreement boundary and, if it closes robustly on both surfaces,
  emit `CoincidentRegion{OverlapSubRegion, regionA, regionB}` and SUPPRESS seeds/march
  inside it. A fuzzy/partial boundary → `Undecided` (no fabricated region).
  `SeedSet` gains `std::vector<CoincidentRegion> coincidentRegions`.

### S4-b — typed tangent-contact classification
- Replaces the blunt `SeedSet.deferredTangent` (kept as a compatibility count) with a typed
  `TangentContact { ContactType; Point3; std::optional<IntersectionCurve>; crossingSine }`.
- Analytic (`tangent_analytic.h`, closed form — never `NearTangentTransversal`/`Undecided`):
  sphere∩sphere d=R₁+R₂ (external) / d=\|R₁−R₂\| (internal) → `TangentPoint` (centre-line
  point); plane tangent sphere → `TangentPoint`; coaxial sphere∩cyl / sphere∩cone tangent
  equator (disc==0, Rc==Rs) → `TangentCurve` (the single tangent Circle); plane tangent to a
  cylinder → `TangentCurve` (the ruling Line).
- Seeded (`tangent_seeded.h`, differential geometry): at a refined solution with
  ‖n₁×n₂‖ < `tangentSinTol`, build the relative second fundamental form `H = II_A − II_B` in
  the shared tangent-plane basis (analytic 2nd derivatives for elementary operands; a
  scale-derived finite-difference jet otherwise). Sign-definite → `TangentPoint` (+point);
  rank-1 → `TangentCurve` (+locus/null-dir); indefinite → `NearTangentTransversal` (S4-c gap,
  handed on, NOT traced); within the model-scale curvature-noise band → `Undecided`. No seed
  fabricated for any. `SeedSet` gains `std::vector<TangentContact> tangentContacts`.
- Marching: `WLine` gains an additive `std::optional<TangentContact> stopReason` set at a
  `NearTangent` stop; `TraceSet.nearTangentGaps` unchanged, the tracer still stops AT the
  tangency (node count identical — purely additive).

### Classification pairs, native-vs-OCCT (sim `run-sim-native-ssi-s4.sh`)

worstOnSurf = max residual of the emitted point/curve on both surfaces.

| Pair | native | OCCT | worstOnSurf |
|---|---|---|---|
| same sphere | Same (`FullSurfaceSame`) | Same | 0.00e+00 |
| spheres d=R₁+R₂ | TangentPoint | Point | 1.22e-16 |
| spheres crossing | Transversal | proper section | 0.00e+00 |
| plane tangent sphere | TangentPoint | Point | 6.12e-17 |
| coaxial sphere∩cyl equator | TangentCurve (Circle) | tangent Circle | 1.84e-16 |
| plane tangent cyl | TangentCurve (Line) | tangent Line | 0.00e+00 |
| seeded sph∩sph (diff-geom) | TangentPoint | Point (sine 1.22e-16) | 1.22e-16 |
| seeded sph∩cyl (diff-geom) | TangentCurve | Circle (sine 0.00e+00) | 0.00e+00 |

**8 pairs, 0 failed, 0 deferred** — every sim pair was decidable and agreed with OCCT.

### Honestly deferred / undecided (asserted in the host seeded suite, never faked)
- Opposite-saddle patch pair → `NearTangentTransversal` (indefinite relative II) — the S4-c
  gap, handed on, never traced.
- Matched-curvature contact probed below the model-scale curvature-noise floor → `Undecided`
  → OCCT. The curvature band is model-scale-derived, NEVER hand-tuned to force a verdict.

### Files (SSI-S4)
- `src/native/ssi/coincidence.h` — `CoincidenceKind` + `CoincidentRegion` (region descriptor).
- `src/native/ssi/same_surface.h` — closed-form `FullSurfaceSame` predicates (all elementary families).
- `src/native/ssi/tangent_contact.h` — `ContactType` + `TangentContact` (typed result + factories).
- `src/native/ssi/tangent_analytic.h` — analytic closed-form tangent classifier.
- `src/native/ssi/tangent_seeded.h` — seeded differential-geometry classifier (relative II form).
- `src/native/ssi/{dispatch.h,seed.h,seeding.cpp,marching.{h,cpp},native_ssi.h}` — wiring (additive).
- `tests/native/test_native_ssi_s4_classification.cpp` — host Gate-1 (14 analytic + 8 seeded; NUMSCI-gated).
- `tests/sim/native_ssi_s4_classification_parity.mm` + `scripts/run-sim-native-ssi-s4.sh` — sim Gate-2.

Living change `openspec/changes/add-native-ssi-s4-classification` **archived** (`2026-07-04`).

### Deferred to S4-d(general)…f / later (recorded, not blocking the S4-a/b/c/d bar)
- **S4-d (first slice DONE)** — branch points (splitting the trace where branches meet/cross).
  The elementary transversal self-crossing (the equal-cylinder **Steinmetz** saddle) is now
  LOCALIZED + ROUTED (see the S4-d result section + capability row below). Still deferred:
  general/freeform branch points, three-plus tangent lines at one point, and cusps (double
  root of the tangent-cone quadratic) → OCCT.
- **S4-e (first slice DONE)** — chart singularities (a surface's own `(u,v)` degeneracy on the
  locus). The **sphere parametric pole** (`v=±π/2`) and the **cone apex** are now DETECTED +
  CROSSED with a point-based fixed-plane cut (see the S4-e result section + capability row
  below). Still deferred: general/freeform parametric singularities (NURBS degenerate edges,
  collapsed spline poles) and higher-order/curve cusps → OCCT.
- **S4-f** — self-intersection guards + small-loop recovery.
- **Deeper near-coincident bands / any near-tangent region not robustly crossable** — S4-c
  crosses only a `NearTangentTransversal` single-branch graze; anything below its crossable
  floor still truncates + defers to OCCT (honest gap).
- **Consuming `CoincidentRegion` / `TangentCurve` in a curved boolean** (overlap handling,
  tangent-seam trimming) → a later S5 slice.

## SSI-S4c result section (near-tangent MARCH-THROUGH — SSI-ROADMAP S4-c, FIRST SLICE)

**Scope (honest):** S4-c is the FIRST slice of the S4 marching CORE — it MARCHES THROUGH a
near-tangency **when the curve genuinely continues** (a `NearTangentTransversal` single-branch
graze) instead of truncating, verified node-by-node on both surfaces vs OCCT `GeomAPI_IntSS`.
It does NOT cross branch points (S4-d), singularities (S4-e), self-intersections (S4-f), or
deeper near-coincident bands — those still stop + classify + defer. Additive to `marching.cpp`,
gated `CYBERCAD_HAS_NUMSCI`; no `cc_*` change; tessellator byte-identical.

### The four levers
- **Fixed-plane-cut corrector** — the S3 corrector's along-`t` advance residual
  (`t = normalize(nA×nB)`) ill-conditions as `sine → 0`. Inside the crossing band `t` is
  replaced by the **last-good forward tangent `t★`** (a hyperplane the curve crosses
  transversally even where the local surface tangent degenerates), keeping the substrate
  `least_squares` solve well-posed.
- **Curvature-aware predictor** — bends `P + h·t★` by the discrete two-node curvature so the
  corrector starts in-basin across the sharp bend.
- **Fine deflection-bounded step** through the low-sine band (capped `h₀/16`, `minStep` floor,
  `crossMaxSteps` budget) — RESOLVES the region rather than leaping it.
- **Crossable gate (the honesty core)** — crosses ONLY a `NearTangentTransversal` graze; a
  **steep-sine-collapse** witness (stall sine < ¼ last-good) OR a **band-minimum-floor** fine
  look-ahead scan (min sine < `0.3·tangentSinTol`) forces a DEFER. Any node failing on-both-
  surfaces / monotone-advance verification, or budget exhaustion at `minStep`, discards the
  WHOLE crossing arc (rollback) and the march STOPS + classifies + defers → OCCT. No point is
  fabricated past a degeneracy; a crossed arc is emitted only if every node verified ≤ `onSurfTol`.

### Both gates green
- **Host** `test_native_ssi_marching` (**10 cases, 0 failed** — the 6 S3 cases + dedup +
  `march_near_tangent_crossed_s4c` (crossing) + `march_tangent_curve_not_crossed_s4c`
  (genuine-tangency defer) + tangent-spheres-no-curve). **NUMSCI OFF CTest 26/26** (S4-c TU
  empty), **NUMSCI ON CTest 31/31**.
- **Sim** `scripts/run-sim-native-ssi-s4c.sh` (booted simulator, vs OCCT `GeomAPI_IntSS`):
  **7 passed, 0 failed**. `nt-cross s4c` — sphere grazed by an offset cylinder that S3
  TRUNCATES at `tangentSinTol=0.25` now traces the FULL closed loop (`nearTangentGaps → 0`,
  **22 near-tangent nodes crossed**, onCurve ≤ 5.64e-5, onSurf ≤ 1.25e-5, crossResid ≤ 4.10e-11).
  `eq-cyl defer` — equal-radius orthogonal cylinders (branch saddle) STILL defers
  (`nearTangentCrossed = 0`, `nearTangentGaps ≥ 1`, no fabricated loop). The 5 pre-existing S3
  transversal pairs trace bit-identically (`nt=0`, branch/closed counts match OCCT). No
  regressions (S5 `native-pass=5` persists, `run-sim-suite.sh` 221/221).

### Honest caveat (measured, not hidden)
At the graze OCCT tolerance-SPLITS the single loop into separate branches (`occtBr=4`) while
the native marcher crosses it into ONE Closed loop — a legitimate CONNECTIVITY disagreement AT
the tangency. The sim gate therefore does NOT assert equal branch counts there; it asserts the
uncontested facts (native curve ON OCCT's locus + on both surfaces + a genuine crossing, not a
truncation).

### Files (SSI-S4c)
- `src/native/ssi/marching.{h,cpp}` — the fixed-plane-cut corrector, curvature predictor,
  crossable gate, and crossing accounting (`nearTangentCrossed`, `crossMaxResidual`).
- `tests/native/test_native_ssi_marching.cpp` — host Gate-1 S4-c assertions.
- `tests/sim/native_ssi_marching_parity.mm` + `scripts/run-sim-native-ssi-s4c.sh` — sim Gate-2.

Living change `openspec/changes/add-native-ssi-s4c-near-tangent-marching` **archived** (`2026-07-04`).

## SSI-S4d result section (BRANCH POINTS — self-crossing locus — SSI-ROADMAP S4-d, FIRST SLICE)

**Scope (honest):** S4-d is the hardest SSI piece: where the intersection LOCUS itself crosses
(multiple curve arms meet at one point), LOCALIZE the branch point, ENUMERATE the outgoing arms,
ROUTE each and ASSEMBLE the multi-arm curve. This first slice handles ONLY the elementary
**two-real-distinct-line transversal self-crossing** (the **Steinmetz** family — two equal-radius
orthogonal cylinders whose intersection is two ellipses crossing at two branch points). It fires
exactly where S4-c would have deferred (the steep-sine-collapse + tangent-flip witness). Additive
to `src/native/ssi/marching.{h,cpp}` + new `src/native/ssi/branch_point.h`, gated
`CYBERCAD_HAS_NUMSCI`, default-on `enableBranchPoints`; no `cc_*` change, tessellator
byte-identical, `src/native/**` OCCT-free.

**Four steps:** (1) **LOCALIZE** — `nn::minimize` the transversality sine `g(s)=‖nA×nB‖` along the
bracketed approach (each trial re-projected onto both surfaces with the S4-c fixed-plane
corrector), then a full `nn::least_squares` re-project of the minimum onto both surfaces; accepted
only if `‖A−B‖ ≤ onSurfTol` and the sine is at/near the floor, else DEFER (no fabricated B). (2)
**ENUMERATE ARMS** — build the shared tangent-plane basis at B, form the relative second
fundamental form `H = II_A − II_B`, solve the tangent-cone quadratic: discriminant `Δ>0` ⇒ TWO
distinct real tangent lines ⇒ up to four world-space rays; `Δ≤0` ⇒ EMPTY (definite ⇒ isolated
`TangentPoint`, END; double root ⇒ cusp, out of scope, DEFER). Never fabricates a ray — the same
discriminant sign as S4-b's `TangentPoint` classification enforces "an isolated tangent point
still ends". (3) **ROUTE** — step `h₀/8` off B along each real ray, S4-c-correct back onto both
surfaces, then run the normal S3 walk to termination; drop the arm if `S₀` fails on-both-surfaces
or the march makes no progress. (4) **ASSEMBLE** — dedup arms that retrace a kept arm
(`retraces`), merge their shared branch-point connectivity into the `BranchNode` (`point`,
`branchSine`, `armLineIds`), `++branchPoints`. A branch not robustly localizable/enumerable/
routable STOPS + defers exactly as S4-c (a `NearTangent` WLine in `nearTangentGaps`).

**Both gates green:**
- **Host** `test_native_ssi_marching` — **12 cases, 0 failed** (adds
  `march_steinmetz_branch_points_s4d` — Steinmetz FULLY traced, `branchPoints==2`,
  `nearTangentGaps==0`, 4 arms → 2 crossing ellipses — and `march_tangent_point_never_branches_s4d`
  — isolated `TangentPoint` ends with `branchPoints==0`, zero arms). **NUMSCI OFF CTest 26/26**
  (S4-d TU empty), **NUMSCI ON CTest 31/31**.
- **Sim** `scripts/run-sim-native-ssi-s4d.sh` (booted simulator, vs OCCT `IntPatch` /
  `GeomAPI_IntSS`): **8 passed, 0 failed**. `eq-cyl s4d` — the Steinmetz bicylinder (two equal-R=1
  orthogonal cylinders) that S3+S4-c TRUNCATE at the saddle is now FULLY traced: `branchPts=2`
  localized at `(0,±1,0)` (branch sine ≈ 5e-8 / 9e-8, re-projection residual ≈ 5e-13),
  `traced=4` `BranchArc` arms (`arms=3` routed), `NTgaps=0`, every native arc node on the OCCT
  locus onCurve ≤ 1.74e-6 and on both cylinders onSurf ≤ 1.07e-8; both branch points match the
  OCCT saddles at `(0,±1,0)` to tol. `eq-cyl defer` (flag off) STILL defers; the S4-c graze STILL
  crosses (`crossed=22`); the 5 S3 transversal pairs trace bit-identically (`nt=0`). No regressions
  (S5 `native-pass=5` persists, `run-sim-suite.sh` 221/221).

### Honest caveat (measured, not hidden)
OCCT `IntPatch` tolerance-splits the Steinmetz locus into `occtBr=7` arc segments (arbitrary
split points), while the native tracer assembles it into 4 `BranchArc` arms meeting at the 2
branch points — a legitimate arc-SEGMENTATION disagreement, not a geometry disagreement. The sim
gate therefore reconciles the branch STRUCTURE (2 branch points at `(0,±1,0)`, two crossing
ellipses) and asserts the uncontested facts (every native node on the OCCT locus + on both
surfaces + branch points matching the OCCT saddles), not equal arc-segment counts.

### Honest scope — what S4-d does NOT do
Only the elementary two-real-distinct-line **transversal self-crossing** (Steinmetz family) is
traced. General/freeform branch points, three-plus tangent lines at one point, cusps (double root
of the tangent-cone quadratic) and S4-f self-intersection completeness DEFER
→ OCCT, reported with the measured gap, never faked (**S4-e chart singularities** — sphere pole +
cone apex — are now crossed natively; see the S4-e result section below). **Steinmetz is now unblocked** natively; the
S5 curved boolean consuming this multi-arm branch `TraceSet` has since landed as **S5-d** (the
branched Steinmetz bicylinder **COMMON/FUSE/CUT** (op-set COMPLETE 3/3), native-pass=12 — see the `native-booleans` **S5-a/b/c/d**
result-table row; sphere∩sphere AND equal-R orthogonal Steinmetz FUSE/CUT/COMMON are all now native).

### Files (SSI-S4d)
- `src/native/ssi/branch_point.h` — `BranchNode` + localize/enumerate/route contract.
- `src/native/ssi/marching.{h,cpp}` — branch-point detection at the S4-c seam, localization,
  tangent-cone arm enumeration, arm routing + dedup/connectivity/assemble, `TraceSet.branchPoints`
  / `branchNodes`, `MarchOptions.enableBranchPoints`.
- `tests/native/test_native_ssi_marching.cpp` — host Gate-1 S4-d assertions.
- `tests/sim/native_ssi_marching_parity.mm` + `scripts/run-sim-native-ssi-s4d.sh` — sim Gate-2.

Living change `openspec/changes/add-native-ssi-s4d-branch-points` **archived** (`2026-07-04`).

## SSI-S4e result section (CHART SINGULARITIES — sphere pole / cone apex — SSI-ROADMAP S4-e, FIRST SLICE)

**Scope (honest):** a **chart (removable) singularity** is where ONE surface's own `(u,v)`
parametrization degenerates (`‖dU‖ → 0`) while its 3D point + surface normal stay perfectly
finite — a **sphere parametric pole** (`v = ±π/2`, where `‖dU‖ = R·cos v → 0`) or a **cone apex**
(signed radius `R₀ + v·sin α = 0`, where the tangential `‖dU‖ → 0`). The intersection curve can be
perfectly TRANSVERSAL through such a point (the pair sine `‖n_A×n_B‖` need NOT collapse), yet the
S3 marcher breaks there: its predictor `advanceParams` solves each surface's single-surface 2×2
normal equations `[dU dV]ᵀ[dU dV]·(Δu,Δv) = [dU dV]ᵀ(h·t)`, and when that surface's `dU` row
vanishes the 2×2 is rank-1, so the `(u,v)` update is ill-conditioned even though the 3D residual +
normal are fine (and the pole/apex sits on a non-periodic `v` edge → spurious `BoundaryExit` on the
sphere / node-budget step-crawl on the cone). This is DISTINCT from S4-c (the surface *pair* grazes,
`n1×n2 → 0`) and S4-d (the *locus* self-crosses). Additive to `src/native/ssi/marching.{h,cpp}` +
new OCCT-free `src/native/ssi/chart_singularity.h`, gated `CYBERCAD_HAS_NUMSCI`, default-**off**
`MarchOptions.enableChartSingularities`, no `cc_*` change, tessellator byte-identical,
`src/native/**` OCCT-free.

**Four parts:** (1) **single-surface chart witness** — `chartConditionAt` central-finite-differences
each surface's `‖dU‖` against `‖dV‖·scale`; a collapse (`‖dU‖ < collapseFrac·‖dV‖` AND `<
collapseFrac·scale`) with a **finite normal** flags a pole/apex on THAT surface — computed from ONE
surface's own Jacobian, NOT the S4-c pair sine and NOT the S4-d locus-tangent flip, and a finite cap
keeps `‖dU‖ = O(‖dV‖)` so a genuine domain boundary is NOT mistaken for a pole. (2) **point-based
fixed-plane-cut crossing** — at a collapse, `crossChartSingularity` makes a bounded sequence of fine
POINT-BASED jumps along the fixed last-good forward tangent `t★` (the branch_point.h / S4-c cut:
drive `A.point − B.point → 0` under the along-`t★` hyperplane), which NEVER touches the degenerate
single-surface `dU`, so it stays well-posed exactly where `advanceParams` failed. (3) **loose chart
map-back** — the sphere pole continues on the OPPOSITE meridian (`u_out = u_in + π mod 2π`, the
free-longitude jump, latitude reflecting) and the cone apex is a single 3D point the straight curve
passes through to the far nappe (`v → −v`); the singular point itself is never emitted. (4) **honest
guard** — a node is emitted ONLY if it verifies on BOTH surfaces ≤ `onSurfTol` with real along-`t★`
progress; on ANY failure (won't verify, no progress, or crossing budget exhausted) the whole band is
DISCARDED (roll back `out`) and the march STOPS + defers → OCCT as a `NearTangent` gap counted in
`nearTangentGaps`. No pole/apex-crossing point is ever fabricated.

**Both gates green.** Host `test_native_ssi_s4e_singularities` **5 cases, 0 failed**
(`s4e_sphere_pole_full_great_circle`, `s4e_cone_apex_both_nappes`,
`s4e_cylinder_cap_boundary_still_exits`, `s4e_switch_on_does_not_perturb_s4c_graze`,
`s4e_switch_on_does_not_perturb_s4d_steinmetz`; **NUMSCI OFF CTest 26/26** with the S4-e TU correctly
ABSENT, **NUMSCI ON CTest 32/32**) + sim native-vs-OCCT `GeomAPI_IntSS` parity
(`tests/sim/native_ssi_marching_parity.mm`, `scripts/run-sim-native-ssi-marching.sh` — **10 passed,
0 failed**): `sphere-pole s4e singX=2 NTgaps=0 closed=1` — a great circle crossing BOTH poles that
S3 TRUNCATES at the first pole (half loop, `len ≈ 3.1415`) is now FULLY traced (`len` native
**6.2829** vs OCCT **6.2832**, rel Δ **5.04e-05**, every node on the OCCT locus + both surfaces ≤
**1.51e-07**); `cone-apex s4e singX=1 NTgaps=0 nodes=159 v1=[-2.00,2.00]` — a double-cone∩plane line
through the apex that S3 STEP-COLLAPSES at (`v` stalls at ≈ −0.04) is now FULLY traced across both
nappes (bounded 159 nodes, on-locus **7.11e-16** / on-surface **6.79e-16**). The genuine finite
cylinder `v`-cap still exits as a `BoundaryExit` (`singularitiesCrossed=0` — the chart machinery does
NOT misfire); with the flag ON the S4-c graze still `crossed=22` and the S4-d Steinmetz still
`branchPts=2 traced=4`. No regressions (`run-sim-suite.sh` 221/221, the 5 transversal pairs `nt=0`
bit-identical, S5 native-pass=6 persists, `src/native/tessellate` diff EMPTY).

### Honest scope — what S4-e does NOT do
Only the two elementary chart (removable) singularities — the **sphere parametric pole** and the
**cone apex** — are crossed, each verified node-by-node on both surfaces + on the OCCT locus.
**General / freeform parametric singularities** (NURBS degenerate edges, collapsed spline poles),
**higher-order / curve cusps** (the curve's own velocity → 0 where the point-based step cannot
recover a far side) remain DEFERRED → OCCT, reported with the measured gap, never faked. (**S4-f
robust closure + a self-intersection guard + an adaptive completeness critic now landed as a
first slice — see the SSI-S4f result section; global topology repair / watertight self-intersection
resolution stay the tail.**) Any pole/apex whose point-based crossing does not verify
on both surfaces defers the same honest way (roll back → `NearTangent` → OCCT). The default is
`enableChartSingularities=false`; the S3/S4-c/S4-d behaviour is byte-identical with the flag off.

### Files (SSI-S4e)
- `src/native/ssi/chart_singularity.h` — the OCCT-free chart-singularity DETECTOR + map-back math
  (`chartConditionAt` single-surface `‖dU‖/‖dV‖` witness, `poleContinuationU` free-longitude jump).
- `src/native/ssi/marching.{h,cpp}` — `chartCondition`/`chartRecovered`/`isPoleEdge`/`chartFarUV`,
  `crossChartSingularity` (the point-based crossing + verification + rollback), `tryChartBand` (the
  band-entry that resumes on success / returns `NearTangent` on defer), `WLine.chartSingularCrossed`
  + `TraceSet.singularitiesCrossed`, `MarchOptions.enableChartSingularities` + chart knobs.
- `tests/native/test_native_ssi_s4e_singularities.cpp` — host Gate-1 (5 cases).
- `tests/sim/native_ssi_marching_parity.mm` + `scripts/run-sim-native-ssi-marching.sh` — sim Gate-2
  (`pairSpherePoleS4e` / `pairConeApexS4e`).

Living change `openspec/changes/add-native-ssi-s4e-singularities` **archived** (`2026-07-05`).

## SSI-S4f result section (COMPLETENESS + LOOP ROBUSTNESS — SSI-ROADMAP S4-f, FIRST SLICE)

The least-glamorous, most-important-for-production S4 slice: it adds NO new geometry capability —
it HARDENS the correctness/completeness of the curves S3 already traces. Additive to
`src/native/ssi/{marching.h,marching.cpp}` + new OCCT-free `src/native/ssi/completeness_critic.h`,
gated `CYBERCAD_HAS_NUMSCI`, no `cc_*` change, tessellator byte-identical, `src/native/**` OCCT-free.
Two orthogonal parts, all switches gated so the S3/S4-c/S4-d/S4-e controls stay byte-identical:

- **Robust TRUE-RETURN closure (always on).** S3 closed a loop on pure proximity
  (`distance(cur, seed) ≤ loopClose·h`), which false-closes a curve that re-approaches its seed /
  an earlier node while heading the other way. Closure is now a necessary-condition tightening:
  close only after the march has travelled a full circuit (`arcLen > 2·closeRadius`) AND the return
  heading is tangent-continuous with the seed's outgoing tangent (`dot(fwdNow, seedFwd) ≥
  closureTangentCos`, default 0.5). It can only REFUSE a close, never MAKE one.
- **Self-intersection guard (default-off `enableSelfIntersection`).** A single arm crossing ITSELF
  (a figure-eight section) is detected by a geometric segment-segment crossing test over the
  stitched polyline (two non-adjacent segments whose closest approach ≤ a tight touch radius at a
  TRANSVERSE angle, `|cos| < 0.7`), recorded as a typed `WLine.selfIntersection` (DATA); the arm
  marches THROUGH it. DISTINCT from an S4-d branch (`branchPoints == 0` — the locus does not flip).
- **Adaptive completeness critic (default-off `completenessCritic`).** After the initial fixed-
  resolution trace, LOOP-UNTIL-DRY: coverage grid over A's domain (`critic::coverageOf` /
  `uncoveredBoxes`) → re-seed FINER (`minPatchFrac *= criticRefineFactor`) at the SAME `onSurfTol`
  (discard any candidate that does not land on both surfaces) → dedup NEW branches by LOCUS vs all
  kept curves → keep the genuinely new; stop after K dry rounds or the cost cap. Reports the floor
  reached (`criticFloorFrac`, `criticStoppedDry`).

**Host gate green:** `test_native_ssi_s4f_completeness` **6 cases, 0 failed** —
(A) small loop MISSED at 1/16 (recall 0.5) → RECOVERED by the critic (recall 1.0 on that fixture,
`criticRecoveredLoops ≥ 1`, floor 1/128, stopped dry, residual acknowledged); (B) a crossing-spheres
circle traced at 10× loopCloseFrac goes from ~1.2% of the true length (S3 false-close) to ≥ 93%,
default frac byte-identical at 99.6%; (C) a figure-eight (Gerono-lemniscate section) records ≥1
typed TRANSVERSE self-crossing near the origin with `branchPoints == 0`, guard OFF byte-identical;
(D) four disjoint loops rise from recall 0.25 to 1.0 with no over-production (traced == true count).
Controls: the transversal loop still Closes; the S4-d Steinmetz still traces (`branchPoints == 2`,
`selfIntersections == 0`) with the guard ON. **NUMSCI ON CTest 33/33; NUMSCI OFF CTest 26/26 (the
S4-f TU is NUMSCI-gated and ABSENT with the substrate off).** No tolerance weakened.

**Sim gate 2 green (native-vs-OCCT, booted simulator):** `tests/sim/native_ssi_s4f_completeness_parity.mm`
via `scripts/run-sim-native-ssi-s4f.sh` — measured recall vs OCCT `GeomAPI_IntSS.NbLines`:
(A) OCCT NbLines=2, native recall **0.50 → 1.00** (traced 1→2, floor 1/128, dry, residual),
every recovered node on both OCCT surfaces ≤ 6.9e-13; (D) OCCT NbLines=4, native recall
**0.25 → 1.00** (traced 1→4, floor 1/48, dry), worst on-surface ≤ 8.9e-12; (C) OCCT returns the
self-crossing locus (NbLines=2 arc-split), native records 1 TRANSVERSE self-crossing near the
origin, `branchPoints == 0`, guard-OFF byte-identical, crossing node on both surfaces ≤ 2.0e-11.
No fabricated branch; recall stays a measured figure with the residual acknowledged.

**HONEST FRAMING:** completeness is MEASURED + ASYMPTOTIC, never a proof — below any fixed re-seed
round a smaller loop can still be missed, so `TraceSet.completenessResidual` /
`RecallReport.residualAcknowledged` are ALWAYS true; a fixture's recall→1 is scoped to that fixture
at that floor. NEVER fabricates a loop, a closure, or a seed. **S4-f DE-RISKS (does not
unblock/complete) curved blends (#6) + wrap-emboss (#7);** watertight self-intersection resolution /
global topology repair stay the S5/S6/S7 tail — S4-f DETECTS + REPORTS + traces-through.

Archived change `openspec/changes/archive/2026-07-05-add-native-ssi-s4f-completeness`.

## native-meshing result section (tetrahedral volume meshing + quality, GitHub #1)

Kernel-only slice: a TetGen-backed tetrahedral volume mesher plus always-on native
mesh-quality reporting, exposed additively on the `cc_*` ABI. TetGen is OPTIONAL,
EXTERNAL, AGPL-3.0, and default-OFF; the default MIT build links no AGPL code.

### ABI (additive PODs + entry points, `include/cybercadkernel/cc_kernel.h`)

- `CCTetMesh` — `nodes` (x,y,z triplets), `elements` (`nodesPerElement` ints per tet,
  0-based, CalculiX C3D4/C3D10 order), `nodeCount` / `elementCount` /
  `nodesPerElement` (4 or 10) / `order`. Caller-owned; freed by `cc_tet_mesh_free`.
- `CCVolumeMeshOptions` — `order` (4 linear / 10 quadratic, default 10),
  `target_element_size`, `grading` (radius-edge quality ratio), `min_scaled_jacobian`
  (reported back, not passed to TetGen). Mirrors CalculiX++ `VolumeMeshOptions`.
- `CCQualityReport` — `min/max_dihedral_angle`, `min/mean_scaled_jacobian`,
  `max_aspect_ratio`, `elements_below_threshold`, `flagged_elements` (ids below the
  threshold), `valid`. Field names align with CalculiX++ `QualityReport` so
  `map_to_model` is trivial. Freed by `cc_quality_report_free`.
- `cc_tet_mesh(body, deflection, opts)`, `cc_tet_mesh_surface(verts, vCount, tris,
  tCount, opts)`, `cc_tet_mesh_free`, `cc_mesh_quality(mesh, min_scaled_jacobian)`,
  `cc_quality_report_free`.

### Quality metrics (native, pure geometry — always on, TetGen-independent)

All from the 4 corner nodes (so `quality(C3D10) == quality(C3D4)` of its corners):

- **Signed volume** `V = (1/6)·det[e12,e13,e14]`.
- **Scaled Jacobian** (Verdict / Knupp — Stimpson et al. *Verdict* SAND2007-1751;
  Knupp, SIAM J. Sci. Comput. 23(1) 2001): `sqrt(2)·min` over corners of the triple
  product of the 3 outgoing UNIT edge vectors. Regular tet → 1.0; sliver → 0;
  inverted → < 0.
- **Dihedral angles** (6/tet, from 4 outward face normals — Shewchuk, *What Is a Good
  Linear Finite Element?* 2002). Regular tet → all six = `acos(1/3) = 70.5288°`.
  Stored in degrees.
- **Aspect ratio** = radius ratio `R·S/(9|V|)` (Verdict "aspect beta" — Liu & Joe,
  BIT 34(2) 1994). Regular → 1.0; degenerate → ∞.

### Verification (kernel-only)

- **`test_native_quality` — always-on (runs in EVERY build, no TetGen).** Regular-tet
  golden (`(1,1,1),(1,-1,-1),(-1,1,-1),(-1,-1,1)`): all six dihedrals ≈ 70.5288°,
  scaledJ ≈ 1.0, aspect ≈ 1.0, V > 0; a sliver flagged below threshold; an inverted
  tet reports scaledJ < 0; C3D10 mid-node consistency (nodes 5..10 exactly at edge
  midpoints); empty/degenerate mesh → `valid == 0`.
- **`test_native_tet` — gated `CYBERCAD_HAS_TETGEN=ON`.** A hardcoded unit cube
  (8 points + 12 triangular facets) driven through `cc_tet_mesh_surface`: the proven
  `pq1.4a…` recipe produces a watertight tetrahedralization (TetGen adds Steiner
  points); C3D4 and C3D10 outputs both have positive signed volume per element,
  C3D10 mid-nodes lie exactly at edge midpoints in CalculiX order, `Σ|V_e|` equals the
  enclosed surface volume (divergence theorem), every internal face is shared by
  exactly two tets and boundary faces match the input surface, and `cc_mesh_quality`
  reports all elements above a modest `min_scaled_jacobian`.

### License hygiene (AGPL discipline)

- No TetGen source (`tetgen.h`, `tetgen.cxx`, `predicates.cxx`, its `LICENSE`) is
  copied, vendored, or committed — referenced by absolute path only.
- `tet_mesher.cpp` is the ONLY translation unit that includes `tetgen.h`; it is
  excluded from the default source glob and compiled only under `CYBERCAD_HAS_TETGEN`,
  with `-DTETLIBRARY` + the TetGen include dir per-source-scoped to it.
- `.gitignore` covers the external archive (`/build-tet/`); no `tetgen.*` /
  `predicates.*` is ever staged.
- SPDX banners: `Apache-2.0` on `quality.*` and the aggregate header;
  `AGPL-3.0` on `tet_mesher.{h,cpp}` (honest about which pair links AGPL).

### Deferred (recorded, not part of this kernel-only PR)

- **FE patch test** and **CalculiX++ `CadMesher` wiring** (import / heal / triangulate
  / tet_mesh / quality / `map_to_model`) — the follow-up PR.
- `grading` beyond its mapping to the TetGen quality ratio (a future mtr sizing field).
- Boundary-layer / anisotropic meshing and Steiner-point control knobs.

## Phase 4 ceiling — native set vs OCCT-retained set

Phase 4 is **COMPLETE AT ITS ACHIEVABLE NATIVE CEILING**, NOT fully drop-OCCT. The tractable
native slice of every planned capability now runs native at the verification bar; what
remains is research-grade (a general robust curved kernel + general native STEP import; IGES descoped, STEP-only) and is
NOT reachable in this program's horizon.

**NATIVE at the bar (both gates green):**
math / geometry (Bézier / B-spline / NURBS curves + surfaces, elementary surfaces, Torus,
transforms); B-rep topology + traversal; watertight tessellation (curved shared-edge stitch);
construction — extrude, line-segment revolve, holed extrude (circular + polygon), typed-profile
extrude (line / arc / full-circle / kind-3 SPLINE edge), typed-profile revolve (line, on-axis-arc
→ sphere, off-axis-arc → TORUS), 2-section AND N-section (3+) ruled loft, sweep along a straight /
smooth-planar / NON-PLANAR (RMF) spine, `cc_tapered_shank`,
well-formed `cc_helical_thread` / `cc_tapered_thread`; PLANAR-polyhedron booleans (fuse / cut /
common via BSP-CSG) and the AXIS-ALIGNED box ⟷ axis-parallel-cylinder curved analytic boolean
slice (closed-form through-hole cut / boss fuse / clipped-segment common); PLANAR blends —
`cc_chamfer_edges`, constant-radius `cc_fillet_edges`, `cc_offset_face`, `cc_shell`; **STEP
EXPORT** (`cc_step_export` for in-scope native solids); and the **SHAPE-HEALING FIRST SLICE**
(internal `healShell` — tolerant sewing + vertex/tolerance unification + degenerate removal +
orientation fix of a coincident-within-tolerance face soup into a watertight solid, verified vs OCCT
`BRepBuilderAPI_Sewing`/`ShapeFix`; the gating foundation for a future native STEP import). Every native op is guarded by a
self-verify (watertight + volume / set-algebra / analytic checks) that DISCARDS a bad candidate
and falls through to OCCT — never a faked result.

**STAYS OCCT (native pending — the ceiling):**
GENERAL curved / concave booleans (surface-surface intersection: sphere / cone / NURBS /
non-axis-aligned / cyl-cyl / blind-hole / non-through cut, near-tangent / coincident-curved);
general curved blends beyond the native slices (NON-linear-law / concave-variable / cyl↔cyl-canal / non-circular-crease fillets, curved-edge chamfer — the constant convex+concave AND variable-radius linear-law convex circular cyl↔cap slices are now native), `cc_fillet_face`, general robust blend/offset over
arbitrary NURBS; the **shape-healing RESIDUAL** (beyond-tolerance gap bridging, missing-pcurve
reconstruction, self-intersecting-wire repair, arbitrary broken industrial B-rep — the
coincident-within-tolerance / degenerate / orientation first slice is now native, above); TIGHT-CURVATURE / self-intersecting / real-twist / guided / rail
sweep, MISMATCHED-count / guided / hard-rail loft, general SPLINE surface-of-revolution, a
SPINDLE torus (off-axis arc crossing the axis), fine-pitch / self-intersecting threads (all
needing SSI / Tier-4 surface-surface intersection + trimming); wrap-emboss over a general curved
target; and **STEP import + IGES export/import** (parsing / writing arbitrary exchange formats).
All of these fall through to OCCT via `NativeEngine` (native builder returns NULL, or the
self-verify / `canSerialize` gate defers), never faked.

**Why #8 `drop-occt` is BLOCKED.** Unlinking OCCT requires EVERY `cc_*` path to have a native
implementation. Two hard dependencies remain, and both are research-grade multi-year efforts,
not incremental slices: (1) a **general robust curved boolean / blend kernel** (arbitrary
surface-surface intersection with exact tolerance handling — the single hardest problem in solid
modelling, plus the healing that goes with it), and (2) **general native STEP IMPORT** (a full
AP203/AP214/AP242 parser + B-rep reconstructor — a first AP203 import slice has landed;
**IGES is DESCOPED, STEP-only**). Until both exist, OCCT stays linked. Phase 4 therefore stops HONESTLY at its native
ceiling: the tractable analytic / planar / export slices are native and verified; the
general-curved and import frontier is explicitly deferred and remains OCCT-backed.

## Regression evidence

- Host build + CTest with Homebrew clang, `-DCYBERCAD_HAS_OCCT=OFF
  -DCYBERCAD_HAS_METAL=OFF`, fresh build dir: configure OK, build OK (no
  warnings/errors), **CTest 21/21 passed, 0 failed** (through #7) — the 7 pre-existing
  targets (registry / guard / scheduler / compute_backend / parallel_policy /
  parallel_toggle / abi) + `test_native_math` / `_topology` / `_tessellate` /
  `_construct` / `_profile` / `_loft` / `_sweep` / `_thread` / `_boolean` / `_blend` /
  `_engine` + the #7 additions `test_native_step_writer` (#19) / `test_native_step` (#20),
  with `test_native_engine` (#21) now including `native_step_export_writes_valid_ap203_file`.
  `test_native_tessellate` stays 13/13 (box/cylinder/sphere/filleted-box watertight
  `boundaryEdges==0`) — unperturbed by the #7 exchange changes.
- `scripts/run-sim-suite.sh` (iphonesimulator arm64): still
  **== 221 passed, 0 failed ==** (verified twice). To confirm HONESTLY against the
  facade+NativeEngine changes (the prebuilt sim lib predated them), the
  SIMULATORARM64 slice was REBUILT from working-tree sources (24 TUs,
  `-DCYBERCAD_HAS_OCCT`, arm64 simulator — `native_engine.cpp` compiles cleanly
  under `CYBERCAD_HAS_OCCT` with `OcctEngine` as the fallthrough target) and the
  suite re-run against the fresh lib. The `.mm` parity tests (`native_math_parity.mm`,
  `native_topology_parity.mm`, `native_tessellate_parity.mm` /
  `native_tessellation_parity.mm`, and the new `native_construct_parity.mm`) are in
  the script's SKIP list and carry their own `main()`, so the OCCT-only
  221-assertion suite count is unchanged. The suite never calls `cc_set_engine`, so
  it exercises the pure OCCT path exactly as before. The #6 blend parity harness
  `native_blend_parity.mm` (own `main()`, its own `run-sim-native-blend.sh`) is
  likewise on the SKIP list — the only uncommitted change to `run-sim-suite.sh` — so
  the 221 count is preserved; `run-sim-suite.sh` **221/221** re-verified (twice).
- For #7 (native STEP export) the SIMULATORARM64 slice was REBUILT AGAIN from current
  sources (25 TUs, 0 compile failures) before re-running the suite, because the prebuilt
  `.a` predated the modified `src/engine/native/native_engine.cpp` and the new
  `src/native/exchange/step_writer.cpp` — so the **221/221** reflects the current native
  STEP-export code, not stale objects. Under the default OCCT engine `cc_step_export` is
  unchanged (routes through OCCT `STEPControl_Writer`, round-trips vol=1000/area=600/bbox
  exactly), and `cc_step_import` + `cc_iges_export`/`_import` remain OCCT (IGES 52 entities,
  round-trip preserved). `native_step_parity.mm` is on the SKIP list (own `main()`). No
  source fixes were required — the diff builds and passes as-is.
- Isolation / blast radius: capabilities #1–#3 (native math `src/native/math/`,
  native topology `src/native/topology/`, native tessellation
  `src/native/tessellate/`) remain unreachable from the `cc_*` facade by design.
  Capability #4 is the **first engine-wired** capability, but the wiring is a safe,
  ADDITIVE opt-in: `NativeEngine` (`src/engine/native/native_engine.cpp`) and the
  native construction library (`src/native/construct/`) are compiled into the
  library via `GLOB_RECURSE src/*.cpp` (OCCT excluded by regex), but they enter a
  `cc_*` call path ONLY after `cc_set_engine(1)`. The default engine is unchanged
  (`cc_set_engine(0)` restores `create_default_engine()` — OCCT where linked, stub
  on host), so every existing suite that never toggles is byte-for-byte unaffected.
  The ONE shared-code behavioural change — `isFullRectangle()` gaining a
  `requireCorners` arg in `src/native/tessellate/trim.h` — has exactly one real
  caller (`face_mesher.h`, updated) and does not touch the OCCT tessellation path;
  it is exercised by `test_native_tessellate` + `test_native_construct` (all green).

## Per-capability status

| # | Capability | Status | Notes |
|---|---|---|---|
| 1 | `native-math` | **done at the bar** | Both gates green (55 host asserts + 24 parity groups, max err 1.486e-13); no regressions; not yet engine-wired (by design). |
| 2 | `native-topology` | **done at the bar** | Both gates green (13 host cases + 3 shapes × 5 parity checks = 15/15, max accessor err 0.000e+00); no regressions (host CTest 9/9, `run-sim-suite.sh` 221/221); header-only, not engine-wired (by design). Deferred: non-manifold/degenerate + seam edges, `CompSolid`/`Internal`/`External`, holed-face parity fixture. |
| 3 | `native-tessellation` | **done at the bar** | Both gates green (host `test_native_tessellate` + sim native-vs-OCCT `BRepMesh` parity, All 20 checks PASS across 4 shapes; ALL four closed solids watertight `boundaryEdges==0`; area/volume relMesh ≤ 6.0e-3, relExact ≤ 1.24e-2, bbox maxCornerΔ ≤ 4.66e-2, on-surface residual ≤ 5.7e-15); no regressions (host CTest 10/10, `run-sim-suite.sh` 221/221); header-only `src/native/tessellate/`, not engine-wired by design. RESOLVED: curved shared-edge stitch (two-stage shared per-edge discretization) — cylinder/filleted-box now watertight. Deferred (genuinely minor, not watertightness): ear-clip trim re-triangulation quality, adaptive per-cell refinement, GPU fp32 path CPU-verified only. |
| 4 | `native-construction` | **done at the bar** | Native `cc_solid_extrude` (closed polygon → prism: bottom/top planar caps + one planar quad per profile edge) and native `cc_solid_revolve` for **LINE-SEGMENT** profiles (segments → plane / cylinder / cone faces of revolution; full 360° closes, partial adds planar caps) — full native topology + geometry under `src/native/construct/construct.h`, OCCT-free/host-buildable. Wired through a new `NativeEngine : IEngine` (`src/engine/native/`) that serves these ops + native tessellate / mass / bbox / **subshape_ids** on its own native bodies and FALLS THROUGH to the OCCT engine (or the stub on host) for every other capability. Facade toggle `cc_set_engine(int)` / `cc_active_engine()` (additive, like `cc_set_parallel`; **default stays OCCT** so existing suites are unchanged). **Both gates green.** Host: `test_native_engine` + `test_native_construct` assert native builds with NO OCCT — boxes (exact vol/area/6-faces/centroid/bbox/watertight), a **triangle prism** (now watertight, exact vol = area×depth, via the tessellator cap-fill fix below), an L-prism, a full-turn tube (9π), a quarter-turn tube (9π/4) and a cone (4π), within the deflection bound; CTest **12/12**. Sim native-vs-OCCT parity (`native_construct_parity.mm`, driven through the `cc_*` facade under `cc_set_engine(0/1)`): **17/17** across box / triangle-prism / cylinder-tube / partial-revolve — mass (vol/area/centroid), bbox, face count, watertight tessellation, plus the fallthrough boolean (native→OCCT) all match. No regressions (`run-sim-suite.sh` **221/221**, `native_tessellation_parity.mm` **20/20**). Three fixes landed here: (a) the tessellator `isFullRectangle` fast-path now, for a PLANAR face, also requires the loop to hit all four box corners, so a convex polygon cap (triangle/hexagon) is ear-clipped instead of filled as its bbox — native extrude of ANY simple polygon now meshes watertight with the exact volume (`trim.h`); (b) `NativeEngine::bounding_box` derives from the tessellated mesh (a revolved solid's B-rep vertices sit only at angular stations, so a vertex-only AABB missed the circular extremes); (c) `NativeEngine::subshape_ids` is native for native bodies (Vertex/Edge/Face counts via the native Explorer). EXPLICITLY DEFERRED to OCCT (not faked, falls through): loft, sweep, twisted/guided sweep, threads, holed/typed-profile extrude variants, revolve of ARC/SPLINE profiles. DOCUMENTED REPRESENTATIONAL DIFFERENCE (not a geometric mismatch): the native builder emits per-face edges / per-patch vertices (proper edge/vertex SHARING deferred) and tiles a full-turn surface of revolution into <π angular patches (periodic-face construction deferred), so native V/E and the full-turn face count differ from OCCT's shared/periodic representation while the SOLID is geometrically identical (volume/area/bbox/watertight all match) — the parity gate asserts face-count where the tiling matches (prisms, partial revolve) and an integer-multiple relation for the full-turn revolve. |
| 4b | `native-construction` (advanced swept solids) | ◐ Tiers A + B + C + D + geometry-completion batch done at the bar; twist/scale + guided/rail + fine-pitch/self-intersecting thread OCCT-fallthrough (SSI/Tier-4), E follow-up | **Geometry-completion batch (`add-native-geometry-completion`) done at the verification bar:** kind-3 SPLINE profile edge extrude, off-axis-arc TORUS revolve (native `Torus` in `src/native/math/torus.h`; exact rational-quadratic B-spline patches), N-section (3+) ruled loft chain, and a NON-PLANAR (double-reflection RMF) sweep are NOW NATIVE (`src/native/construct/residuals.h` / `loft.h` / `sweep.h`); both gates green (host CTest **22/22**; sim `native_geomcompletion_parity.mm` — spline extrude vol rel 9.92e-04, torus revolve rel 2.68e-02, N-section frustum + straight-rail loft rel ≤ 1.4e-14 EXACT, RMF smooth-arc sweep rel 3.44e-16 EXACT, all watertight). Honest fall-through / DECLINE (never faked): self-crossing spline + spindle torus DECLINE on both engines (Tier-4 SSI, occtId=0 natId=0); mismatched-count loft → OCCT `ThruSections`, hard curved rail → OCCT `MakePipeShell`, self-intersecting sweep → OCCT `MakePipe`, real-twist `cc_twisted_sweep` → OCCT `ThruSections`, self-intersecting thread → OCCT `MakePipeShell` (all native active=1, rel 0.00e+00 — the accumulating-twist/scale sweep, guided/rail cases, and the thread resolver did NOT self-verify oracle-correct beyond the well-formed set → remaining fall-throughs now specifically need SSI/Tier-4). No regressions (`run-sim-suite.sh` 221/221). **Tier A (`add-native-construction-profiles`) done at the verification bar:** `cc_solid_extrude_holes` (circular holes → TRUE `Circle` edge + `Cylinder` wall), `cc_solid_extrude_polyholes` (polygon holes), `cc_solid_extrude_profile` / `_profile_polyholes` (typed line/arc/full-circle outer + holes), `cc_solid_revolve_profile` (line → Plane/Cylinder/Cone, on-axis arc → Sphere) are NATIVE (`src/native/construct/profile.h`). Both gates green: host `test_native_profile` + `test_native_engine` CTest **13/13** (no OCCT); sim native-vs-OCCT parity `native_construct_profiles_parity.mm` **22/22** — 5 native families (polyhole EXACT rel 1.97e-16; curved vol rel ≤ 4.97e-2, all watertight) + 2 fall-through families (kind-3 spline extrude, off-axis-arc torus revolve, vol rel 0.00e+00). **Tier B (`add-native-loft`) done at the verification bar:** `cc_solid_loft` / `cc_solid_loft_wires` for TWO PLANAR sections with EQUAL vertex counts (≥3) are NATIVE — one BILINEAR (degree-1 Bézier) ruled side face per corresponding edge pair + two planar caps → watertight solid, mirroring ruled `BRepOffsetAPI_ThruSections` (`src/native/construct/loft.h`, all functions cognitive complexity ≤ 7). Both gates green: host `test_native_loft` (9 cases) + `test_native_engine` (2 new facade cases) CTest **14/14** (no OCCT); sim native-vs-OCCT parity `native_loft_parity.mm` **17/17** — 3 EXACT families (square-frustum rel 2.54e-16, hex-prism rel 0.00e+00, tri-prism loft_wires rel 0.00e+00) + rotated-square TWIST deflection-bounded (vol rel 5.33e-3, watertight) + a mismatched-count fall-through delegating to OCCT (vol rel 0.00e+00). No regressions (`test_native_tessellate` green — box/cylinder/sphere/filleted-box watertight `boundaryEdges==0`, 13/13 cases; `run-sim-suite.sh` 221/221). **Tier C (`add-native-sweep`) done at the verification bar:** `cc_solid_sweep` for a STRAIGHT spine (EXACT directional prism, vol = profileArea×\|d\|) and a SMOOTH CURVED but PLANAR spine (CONSTANT-frame ruled tube — the section is TRANSLATED with a fixed orientation, matching OCCT MakePipe's planar `GeomFill_CorrectedFrenet` → `Law_Constant`, NOT a perpendicular/Pappus sweep) are NATIVE (`src/native/construct/sweep.h`, reuses `loft.h` `ruledSideFace` + `construct.h` `planarFace`; `build_sweep` cognitive complexity 14). `cc_twisted_sweep` is native only when it reduces to the plain sweep (twist ≈ 0 AND scale ≈ 1). An earlier RMF/double-reflection revision was REMOVED — it produced the Pappus volume, a real oracle mismatch. Both gates green: host `test_native_sweep` (11 cases) + `test_native_engine` (3 sweep cases) CTest **15/15** (no OCCT); sim native-vs-OCCT parity `native_sweep_parity.mm` **11/11** (8 native + 3 fallback) — straight EXACT vol rel 7.11e-16 and smooth-arc EXACT vol o=330.299 n=330.299 rel 1.72e-16 (native F = OCCT F = 98, watertight), plus real-twist / guided / loft-rail fall-through delegating to OCCT (vol rel 0.00e+00, native active). STILL OCCT-fallthrough (not faked): kind-3 SPLINE edges, off-axis-arc (torus) / spline surface-of-revolution; loft with MISMATCHED counts / a NON-PLANAR section / a point-collapse section / 3+ sections / guided / rail; a NON-PLANAR / TIGHT-CURVATURE / self-intersecting sweep spine, a REAL twist/scale sweep, `cc_guided_sweep` / `cc_loft_along_rail`; and E wrap-emboss. **Tier D (`add-native-threads`): `cc_tapered_shank` + well-formed `cc_helical_thread` / `cc_tapered_thread` done at the verification bar (all NATIVE); fine-pitch / self-intersecting thread honestly OCCT-fallthrough.** `cc_tapered_shank` is a pointed-shank silhouette (cone tip → full-radius cylinder → head disk) revolved 360° about the WORLD Z axis by reusing the native `build_revolution` (`src/native/construct/thread.h`, all functions cognitive complexity ≤ 5) — reproducing the OCCT `BRepPrimAPI_MakeRevol` oracle (mass/centroid/bbox), tip a TRUE on-axis apex, robustly watertight at every deflection. `cc_helical_thread` / `cc_tapered_thread` build the full radial-V axis-aux-spine helical tiling (three bilinear ruled bands per span + two planar V caps); **the per-turn seam weld — the last blocker — is now fixed at the mesher level** (canonical seam-point snap, `edge_mesher.h` `CanonicalEndpoints` / `face_mesher.h` `BoundaryAnchors`), so a well-formed thread meshes `boundaryEdges==0` at EVERY deflection across the full parameter sweep (432/432 helical + 96/96 tapered → native), passing the engine `robustlyWatertight` self-verify and running NATIVE. Only a FINE-PITCH / self-intersecting thread (non-manifold regardless of weld) still FALLS THROUGH to OCCT `MakePipeShell` (labelled, verified, never faked). Both gates green: host `test_native_thread` (9 cases — including the multi-deflection watertight ladder for helical + tapered, and the fine-pitch guard) + `test_native_engine` (`native_thread_runs_native_watertight` + `native_fine_pitch_thread_falls_through_to_default`) CTest **18/18** (no OCCT; no fixes needed, green on first run); sim native-vs-OCCT parity `native_thread_parity.mm` — `cc_tapered_shank` NATIVE r5/fh20/th10 vol o=1837.94 n=1830.27 rel 4.17e-03 / watertight 144 tris; **`cc_helical_thread` NATIVE** mr5/p2/t4/d1 vol o=70.2841 n=68.3767 rel 2.71e-02 / area rel 1.73e-02 / bbox maxCornerΔ 1.44e-03 / F 5→194 / watertight 1286 tris meshVolRel 1.40e-03; **`cc_tapered_thread` NATIVE** top6/tip4/p2/t4 vol o=70.2677 n=68.3767 rel 2.69e-02 / watertight 1286 tris (the ~2.7% native-vs-OCCT volume gap is chord-vs-arc at spt=16, tightening to ~1.3% at spt=24; native mesh-vs-B-rep volume matches to meshVolRel ≤ 1.40e-3), plus the fine-pitch thread as OCCT fall-through (native active=1, vol rel 0.00e+00). No regressions (`run-sim-suite.sh` 221/221, `test_native_tessellate` green). |
| — | `numeric-foundations` (native-rewrite #2 — the substrate) | **done at the bar** | **NumPP + SciPP ADOPTED** as the OCCT-free numeric substrate (absolute-path, NOT vendored, CPU-only, `optimize`/`linalg`(+`spatial`/`integrate`) subset with `special`/`stats` EXCLUDED, guarded by `CYBERCAD_HAS_NUMSCI` default OFF). A thin OCCT-free facade (`src/native/numerics/`) exposes the generic solvers (root/`fsolve`/`minimize`(BFGS)/`least_squares`(LM)/`solve`/`lstsq`) and native **closest-point/projection** (the `Extrema` on-ramp — point→curve/point→surface, multi-start + SciPP refine, global-best foot). Both gates green: host `test_native_numerics` (22 assertions, no OCCT — solver known-values + closed-form + brute-force closest-point) built under `CYBERCAD_HAS_NUMSCI=ON`; sim native-vs-OCCT `Extrema` parity `native_numerics_parity.mm` **22/22 `[NNUM]`** — dDist ≤ 1.776e-15 (analytic plane/cylinder/sphere feet fp-exact, dPoint ≤ 1.707e-10; B-spline within tol, largest `bspline_surf#3` dPoint 3.946e-08 at corner). Substrate compiles+links 77/77 TUs HOST + arm64-iOS-simulator. Realizes the eval's ~60–75% #2 effort saving (→ ~0.15–0.35 py). No regressions: host `NUMSCI=OFF` CTest 22/22 (`test_native_numerics` correctly ABSENT), `NUMSCI=ON` CTest 23/23, `run-sim-suite.sh` 221/221 (determinism serial==parallel bit-reproducible). Deferred (NOT blocking): multiple-extrema enumeration, curve-curve/surface-surface distance (`Extrema_ExtCC`/`Extrema_ExtSS`), the `bspline_surf#3` corner caveat. **SSI (near-tangent) is NOT bought — it stays #5.** Living change `add-native-numerics` (archived). |
| 5 | `native-booleans` | ◐ PLANAR-polyhedron slice + AXIS-ALIGNED box-cylinder curved analytic slice done at the bar (both archived); general curved OCCT-fallthrough | Native `cc_boolean` (fuse/cut/common) for PLANAR-faced solids (axis-aligned boxes, prisms) via a BSP-tree CSG (`src/native/boolean/`), guarded by a MANDATORY self-verify (`robustlyWatertight` + set-algebra volume) that discards + falls through to OCCT otherwise. Both gates green: host `test_native_boolean` + `test_native_engine` CTest **17/17** (no OCCT); sim native-vs-OCCT parity `native_boolean_parity.mm` **25/25** — box fuse (rel 1.27e-16) / cut (2.96e-16) / common (2.22e-16), contained fuse (0.00e+00) / common (2.22e-16) all EXACT + watertight, self-verify rejects native∩native disjoint, plus curved (cyl-box, rel 0.00e+00) / near-coincident (rel 0.00e+00) / disjoint (rel 0.00e+00) OCCT-fallthrough (delegated, no interception). No regressions (`run-sim-suite.sh` 221/221, `test_native_tessellate` green). **Curved analytic slice (`add-native-curved-booleans`, archived) — AXIS-ALIGNED box ⟷ axis-parallel cylinder cut/fuse/common NOW NATIVE** (closed-form `Cylinder`+`Circle`+`Plane` B-rep, analytic-volume self-verify, `src/native/boolean/curved.h`): both gates green — host CTest **18/18**; sim `[NCURVBOOL]` **18 checks (6×3), 0 failed** — 3 NATIVE (through-hole-cut rel 3.19e-04, boss-fuse rel 6.10e-05, common rel 1.30e-03, all watertight) + 3 OCCT-fallback (blind-hole-cut / oblique-cyl-cut / sphere-box-cut, rel 0 forwarded). STILL OCCT: general curved-face booleans (surface-surface intersection: sphere/cone/NURBS/non-axis-aligned/cyl-cyl/blind-hole/non-through cut), near-tangent/coincident, general/concave-general, foreign operands, shape healing — booleans remain the longest-lived OCCT dependency for general curved. |
| 5·SSI | `native-ssi` (SSI-ROADMAP **S1** analytic) | **done at the bar** | Analytic surface-surface intersection for the elementary family, OCCT-free header-only `src/native/ssi/`, built over `src/native/math` only (IntAna-style closed form; NO GeomAPI / NO numsci). INTERNAL — no `cc_*` entry, parity at the C++ boundary. **17 analytic-native pairs** verified vs OCCT `GeomAPI_IntSS` (all curve TYPES match; onSurf/coin ≤ ~4e-15): plane∩plane (Line), plane∩sphere/⟂cyl/⟂cone (Circle), plane∠cyl/∠cone (Ellipse), plane∥cyl / ∥cyl (2 Lines), plane∥gen-cone (Parabola), plane-steep-cone (2 Hyperbola), plane∩torus (⟂-axis 1–2 + ∋-axis 2 circles), sphere∩sphere (Circle), coaxial sphere∩cyl / sphere∩cone / cyl∩cone (Circles), coaxial cyl∩cyl (coincident). Both gates green: host `test_native_ssi` **11 cases, 0 failed** (NUMSCI OFF CTest 23/23, NUMSCI ON 24/24) + sim `GeomAPI_IntSS` parity `run-sim-native-ssi.sh` **18 pairs, 0 failed**. No regressions (`run-sim-suite.sh` 221/221). **Honestly DEFERRED → `NotAnalytic` (never faked):** skew cyl∩cyl (OCCT emits 7 Ellipse — planar quartic), general cone∩cone, non-coaxial cone∩cyl / sphere∩cyl / sphere∩cone, oblique plane∩torus (spiric quartic), torus∩curved, all freeform pairs → S2 seeding / S3 marching (**both DONE**) / S4 robustness. Feeds the S5 curved-boolean payoff (`native-booleans` #5). Living change `add-native-ssi-analytic` (archived). See the SSI-S1 result table + `openspec/SSI-ROADMAP.md`. |
| 5·SSI | `native-ssi` (SSI-ROADMAP **S2** seeding) | **done at the bar (transversal)** | Subdivision seeding for the freeform / skew-quadric pairs S1 defers: recursive patch-AABB-overlap subdivision → candidate regions → `least_squares` refine on the numerics substrate → dedup to ~1 seed per **transversal** branch. OCCT-free `src/native/ssi/{seed,patch_bounds,seeding}.h + seeding.cpp` (refine behind `CYBERCAD_HAS_NUMSCI`); INTERNAL. Both gates green: host `test_native_ssi_seeding` **6 cases, 0 failed** (NUMSCI OFF CTest 23/23 with NUMSCI-gated tests ABSENT, NUMSCI ON 25/25) + sim `GeomAPI_IntSS` **recall** parity `native_ssi_seeding_recall.mm` **3/3 transversal branches, recall 1.00**, tangent 0, worst seed onSurf 3.51e-16. No regressions (`run-sim-suite.sh` 221/221). **Honest scope:** TRANSVERSAL only — near-tangent / coincident / degenerate seeding → S4 (`SeedSet.deferredTangent`, never faked); completeness is a measured recall (`minPatchFrac` 1/32 = recall/cost knob). Living change `add-native-ssi-seeding`. See the SSI-S2 result table + `openspec/SSI-ROADMAP.md`. |
| 5·SSI | `native-ssi` (SSI-ROADMAP **S3** marching) | **done at the bar (transversal)** | Marching-line tracer (WLine): from each S2 seed, predict `t=n₁×n₂` → adaptive step → **re-project** onto BOTH surfaces via the substrate (`least_squares`) → march both directions + stitch → `Closed` / `BoundaryExit` → dedup → fit B-spline. OCCT-free `src/native/ssi/{marching.h,marching.cpp}` (corrector/step/fit behind `CYBERCAD_HAS_NUMSCI`, `marching.cpp` empty TU with NUMSCI off); INTERNAL. Consumes the S2 `SeedSet`, produces a `TraceSet` (WLines with (u1,v1,u2,v2) per node) = the S5 input contract. Both gates green: host `test_native_ssi_marching` **7 cases, 0 failed** (crossing spheres/plane∩sphere/skew-cyl → Closed; ramp B-spline∩plane → BoundaryExit; tangent spheres → no curve; dup seed → 1 WLine; nodes on both surfaces < 1e-6, fit < 1e-3; NUMSCI OFF CTest 23/23 tests ABSENT, NUMSCI ON 26/26) + sim `IntPatch`/`GeomAPI_IntSS` curve parity `native_ssi_marching_parity.mm` **5 pairs / 9 branches, 0 failed — all TRANSVERSAL fully-traced, 0 near-tangent-truncated**; branch counts match OCCT, **5/5 closed loops reproduced**; worst onCurve 1.60e-06, onSurf 6.81e-07, lenΔ 2.28e-03 abs (~0.33% rel). No regressions (`run-sim-suite.sh` 221/221). **Honest scope:** TRANSVERSAL only — near-tangent traced up to the tangent (`NearTangent`, `nearTangentGaps`), coincident / branch-point / self-intersection → **S4** (its S4-a/b classification + the S4-c first near-tangent march-through slice + the S4-d first branch-point slice — the Steinmetz self-crossing localized + routed — + the S4-e first chart-singularity slice — a curve crossing a sphere parametric pole / cone apex fully traced — now done at the bar; the deeper marching core S4-d general/freeform + S4-e general/freeform + S4-f is the moat tail). Feeds the S5 curved-boolean payoff. Living change `add-native-ssi-marching` (archived). See the SSI-S3 result table + `openspec/SSI-ROADMAP.md`. |
| 5·SSI | `native-booleans` (SSI-ROADMAP **S5-a/b/c/d** curved boolean) | **slices done at the bar** | SSI-curve-driven split→classify→select→weld curved-boolean pipeline `src/native/boolean/ssi_boolean.{h,cpp}` (OCCT-free, `CYBERCAD_HAS_NUMSCI`-gated, consumes the S3 `TraceSet` — and, for S5-d, the S4-d branched re-trace). INTERNAL — invoked behind the existing `cc_boolean` op codes, no `cc_*` entry added; the native lib returns NULL when it cannot robustly handle an input and the ENGINE self-verify (watertight + correct-volume) falls back to OCCT. Produces **ten native curved-boolean sub-cases verified vs OCCT `BRepAlgoAPI_{Fuse,Cut,Common}`** (sim `run-sim-native-ssi-curved-boolean.sh` **18 passed, 0 failed, native-pass=12**): **S5-a** through-drill cyl∩cyl **COMMON** (unequal radii, transversal two-loop trace) — watertight, volN 3.1143 vs volO 3.1169, ΔV **8.14e-04**, ΔA 2.83e-04; **S5-b** through-drill cyl∩cyl **FUSE** (ΔV **8.80e-05**) + **CUT** (ΔV **4.03e-05**), assembler-only (fat wall with the two mouths cut out + planar-facet caps + reversed tunnel band / protruding end tubes, same shared `VertexPool` seams as S5-a); **S5-c** the sphere∩sphere op-set now **COMPLETE 3/3 native — COMMON + FUSE + CUT** (single closed seam; all three share the SAME decimated seam and reuse one generalised `appendSphereCap(outer,reversed)` cap builder + `VertexPool` weld, direction-slerp cap facets robust at the parametric pole): **COMMON** = two INNER (nearest-apex) caps (equal ΔV **4.13e-04** / unequal **4.66e-04**); **FUSE** (A∪B) = two OUTER (far-pole) caps, `V=V(A)+V(B)−lens` (equal ΔV **6.46e-04** volN 7.064 vs volO 7.0686 / unequal ΔV **8.34e-04** volN 8.1928 vs 8.1996); **CUT** (A−B, order-sensitive) = OUTER cap of A + INNER cap of B emitted REVERSED (inward normal bounds the scooped cavity), `V=V(A)−lens` (equal ΔV **6.98e-04** volN 2.8778 vs 2.8798 / unequal ΔV **9.29e-04** volN 6.0493 vs 6.0549) — all watertight, verified vs BOTH the analytic closed forms AND OCCT, no tolerance weakened; the COMMON path stays byte-identical (defaults `outer=false,reversed=false`); **S5-d** the **branched-trace Steinmetz bicylinder op-set now COMPLETE 3/3 native — COMMON + FUSE + CUT** (equal-R orthogonal cyl∩cyl): a `steinmetzPreGate` fires ONLY on the S4 decline edge (`nearTangentGaps>0`), RE-TRACES with `MarchOptions.enableBranchPoints=true`, `recogniseSteinmetzTrace` accepts only the canonical structure (`branchPoints==2`, four `BranchArc` arms), and the shared lune/arc split + `VertexPool` weld (`appendSteinmetzOuterWall` + the branched lune builder) drive all three ops from the SAME branched trace, differing only by which fragments survive + cap handling: **COMMON** `buildSteinmetzCommon` welds the four inside-the-other lune patches (keeps the four whose centroid is inside the other cylinder) into ONE watertight shell sharing the four arc seams + the two branch-point vertices (S5-a planar-facet + `VertexPool` weld) — verified vs **BOTH** the EXACT analytic `16 R³/3 = 5.33333` (host) **and** OCCT `BRepAlgoAPI_Common` 5.3333 (sim): volN **5.3287**, ΔV **8.75e-04** (−0.088%), ΔA **4.68e-04** (byte-identical to the S5-d baseline); **FUSE** `buildSteinmetzFuse` (A∪B) keeps the OUTSIDE wall regions of BOTH cylinders + all four original end caps, welded along the four arcs, `V=V(A)+V(B)−V(common)` — volN **32.385** vs OCCT `BRepAlgoAPI_Fuse` **32.366**, ΔV **5.82e-04**, ΔA **4.07e-03**; **CUT** `buildSteinmetzCut` (A−B) keeps A's OUTSIDE wall + A's caps + B's lune patches emitted REVERSED (inward normal, bounding the carved channel through A), `V=V(A)−V(common)` — volN **13.526** vs OCCT `BRepAlgoAPI_Cut` **13.516**, ΔV **7.22e-04**, ΔA **3.17e-03**; all watertight/closed/valid, inside the 1% curved-parity bar, no tolerance weakened. Both gates green: host `test_native_ssi_curved_boolean` + `test_native_ssi_boolean` (S5-a COMMON + S5-b FUSE/CUT vs inclusion-exclusion closed form + S5-c equal/unequal lens vs analytic lens volume + **S5-d Steinmetz COMMON vs `16 R³/3` with 2-branch/4-arm assertions** + **the new `branched_fuse_cut_watertight_matches_analytic` case asserting FUSE=V(A)+V(B)−V(common) / CUT=V(A)−V(common), watertight, dispatcher non-null**, each ≤ 1% deflection band; honest NULL fixtures: tangent/disjoint spheres, thin−fat, **disjoint Steinmetz pair → NULL for all three ops** + **the sphere∩sphere FUSE/CUT analytic case (`FUSE=V(A)+V(B)−lens`, `CUT=V(A)−lens`, equal + unequal radii, watertight, CUT order-sensitivity `A−B≠B−A` asserted)**) — **NUMSCI OFF CTest 29/29** with the NUMSCI-gated SSI tests correctly ABSENT, **NUMSCI ON CTest 36/36**. No regression across tessellator/boolean/S5 suites; tessellator (`src/native/tessellate`) diff EMPTY (the S5-a lesson — every seam-adjacent face emits planar-triangle facets, mesher untouched); all prior native passes bit-identical (the sphere∩sphere COMMON/FUSE/CUT, drill COMMON/FUSE/CUT, and Steinmetz COMMON all persist; the additions are exactly the Steinmetz FUSE + CUT). **Honestly DECLINING → OCCT (measured NULL fallbacks, never faked):** any branched pair that is NOT equal-R orthogonal Steinmetz (unequal-R / non-orthogonal / ≠ 2-branch / ≠ 4-arm), a disjoint Steinmetz pair (no seam), oblique / multi-tube cyl∩cyl, tangent/coincident pairs, and other curved-curved families (cyl∩cone, cyl∩sphere, cone∩cone, sphere∩box, cone∩box, freeform). Living changes `add-native-ssi-curved-boolean` + `add-native-ssi-curved-boolean-wider` + `add-native-ssi-branched-boolean` + `complete-sphere-sphere-fuse-cut` + `complete-steinmetz-fuse-cut` (all archived; branched `2026-07-05`, sphere fuse/cut + Steinmetz fuse/cut `2026-07-06`). See `openspec/SSI-ROADMAP.md` S5. |
| 5·SSI | `native-ssi` (SSI-ROADMAP **S4-a/b** coincident + tangent CLASSIFICATION) | **done at the bar** | The DETECTION + CLASSIFICATION layer of the S4 moat (NOT the marching core — S4-c…f). OCCT-free `src/native/ssi/{coincidence.h,same_surface.h,tangent_contact.h,tangent_analytic.h,tangent_seeded.h}`; the seeded-path parts behind `CYBERCAD_HAS_NUMSCI`; INTERNAL (no `cc_*`). **S4-a** — robust coincidence on both the analytic and seeded paths + a typed `CoincidentRegion` (`FullSurfaceSame` closed-form for ALL elementary families: plane, coaxial-equal cyl/cone, same sphere, same torus — folding the pre-existing same-sphere / coaxial-equal-cyl `Coincident`; seeded `OverlapSubRegion` with delimited param bounds via grid-agreement `on-both residual ≤ onSurfTol AND ‖n_A×n_B‖ ≤ tangentSinTol` + boundary growth, suppressing seeds/march inside; `Undecided` → OCCT when the region cannot be robustly delimited). **S4-b** — a typed `TangentContact` replacing the blunt `SeedSet.deferredTangent` counter: `TransversalOnly` / `TangentPoint` (isolated, emits the point) / `TangentCurve` (tangent along a curve, emits it) / `NearTangentTransversal` (grazes-and-crosses → S4-c gap, handed on) / `Undecided`; analytic configs decided in closed form (sphere∩sphere d=R₁+R₂ / d=\|R₁−R₂\| → point, coaxial sphere∩cyl equator → circle, plane tangent cyl → line, plane tangent sphere → point), seeded solutions by the relative second fundamental form `H = II_A − II_B` in the shared tangent basis (sign-definite → point, rank-1 → curve, indefinite → near-tangent, within the model-scale curvature-noise band → undecided — NEVER hand-tuned to force a verdict). Marching (`WLine`) additionally carries a typed `stopReason` at a `NearTangent` stop; the tracer STILL stops at the tangency, node count unchanged, never steps through (S4-c). Both gates green: host `test_native_ssi_s4_classification` **14 analytic + 8 seeded cases, 0 failed** (**NUMSCI OFF CTest 26/26** with the 8 seeded cases correctly ABSENT, **NUMSCI ON CTest 31/31**) + sim native-vs-OCCT classification parity `native_ssi_s4_classification_parity.mm` / `run-sim-native-ssi-s4.sh` **8 pairs, 0 failed, 0 deferred**: `FullSurfaceSame`↔`IntAna_Same`, `TangentPoint`↔`IntAna_Point`, `TangentCurve`↔tangent Line/Circle, `TransversalOnly`↔proper section (`IntAna_QuadQuadGeo`/`IntAna_ResultType` for analytic, `IntPatch`/`GeomAPI_IntSS`+`GeomLProp_SLProps` for seeded); emitted point/curve on both surfaces ≤ ~1.84e-16. No regressions (`run-sim-suite.sh` 221/221, all six pre-S4 parity scripts green, S5 native-pass=5 persists; additive/guarded, `src/native/tessellate` diff EMPTY). **Honestly DEFERRED / Undecided → OCCT (asserted in the seeded suite, never weakened, never faked):** opposite-saddle patch (indefinite relative II) → `NearTangentTransversal` (the S4-c gap, handed on, never traced); matched-curvature contact below the curvature-noise floor → `Undecided`. **NOT in this S4-a/b layer (→ S4-c marches these, S4-d…f the moat tail):** marching THROUGH a tangency (the S4-c `NearTangentTransversal` graze now crosses — see the S4-c row below), branch-point splitting (S4-d), singularities (S4-e), self-intersection resolution (S4-f). Living change `add-native-ssi-s4-classification` (archived `2026-07-04`). See the SSI-S4 result section + `openspec/SSI-ROADMAP.md`. |
| 5·SSI | `native-ssi` (SSI-ROADMAP **S4-c** near-tangent MARCH-THROUGH, FIRST SLICE) | **first slice done at the bar** | The FIRST slice of the S4 marching CORE: MARCHES THROUGH a near-tangency **when the curve genuinely continues** (a `NearTangentTransversal` single-branch graze) instead of truncating, verified node-by-node vs OCCT `GeomAPI_IntSS`. Additive to `src/native/ssi/marching.{h,cpp}`, gated `CYBERCAD_HAS_NUMSCI`, no `cc_*` change, tessellator byte-identical. Four levers: **fixed-plane-cut corrector** (replaces the along-`t` advance residual — ill-conditioned as `sine→0` — with a cut on the plane ⊥ the last-good forward tangent `t★`, keeping the substrate `least_squares` well-posed); **curvature-aware predictor** (bends `P+h·t★` by the discrete two-node curvature); **fine deflection-bounded step** through the band (`h₀/16` cap, `minStep` floor, `crossMaxSteps` budget); **crossable gate** (crosses ONLY `NearTangentTransversal`; a steep-sine-collapse witness OR a band-minimum-floor look-ahead scan forces a DEFER; any node failing on-both-surfaces / monotone-advance verification, or budget exhaustion, DISCARDS the whole crossing arc — no point fabricated past a degeneracy). Both gates green: host `test_native_ssi_marching` **10 cases, 0 failed** (adds `march_near_tangent_crossed_s4c` crossing + `march_tangent_curve_not_crossed_s4c` genuine-tangency defer; **NUMSCI OFF CTest 26/26** S4-c TU empty, **NUMSCI ON CTest 31/31**) + sim `run-sim-native-ssi-s4c.sh` **7 passed, 0 failed** — `nt-cross s4c`: a sphere grazed by an offset cylinder that S3 TRUNCATES at `tangentSinTol=0.25` now traces the FULL closed loop (`nearTangentGaps→0`, **22 nodes crossed**, on the OCCT locus onCurve ≤ 5.64e-5 / onSurf ≤ 1.25e-5 / crossResid ≤ 4.10e-11); `eq-cyl defer`: equal-radius orthogonal cylinders (branch saddle) STILL defers (`nearTangentCrossed=0`, `nearTangentGaps≥1`, no fabricated loop); the 5 S3 transversal pairs trace bit-identically (`nt=0`). No regressions (`run-sim-suite.sh` 221/221, S5 native-pass=5 persists). **Honest caveat:** at the graze OCCT tolerance-SPLITS into `occtBr=4` branches while the native marcher crosses into ONE Closed loop — a CONNECTIVITY disagreement AT the tangency; the sim gate asserts the uncontested facts (on OCCT locus + both surfaces + genuine crossing), not equal branch counts. **STILL DEFERRED → OCCT (honest):** at the S4-c bar the equal-cylinder branch saddle defers — that saddle is the S4-d branch-point case, now localized + routed (see the S4-d row below); singularities (S4-e), self-intersection (S4-f), deeper near-coincident bands, and any region below the crossable floor still defer. Living change `add-native-ssi-s4c-near-tangent-marching` (archived `2026-07-04`). See the SSI-S4c result section + `openspec/SSI-ROADMAP.md`. |
| 5·SSI | `native-ssi` (SSI-ROADMAP **S4-d** BRANCH POINTS, FIRST SLICE) | **first slice done at the bar** | The hardest SSI piece: where the intersection LOCUS itself crosses (multiple arms meet at one point), LOCALIZE the branch point, ENUMERATE the outgoing arms, ROUTE each and ASSEMBLE the multi-arm curve — verified vs OCCT `IntPatch`/`GeomAPI_IntSS`. Additive to `src/native/ssi/marching.{h,cpp}` + new `src/native/ssi/branch_point.h`, gated `CYBERCAD_HAS_NUMSCI`, default-on `enableBranchPoints`, no `cc_*` change, tessellator byte-identical, `src/native/**` OCCT-free. Fires exactly where S4-c would have deferred (steep-sine-collapse + tangent-flip witness). **LOCALIZE** — `nn::minimize` the transversality sine `‖nA×nB‖` along the approach (re-projected onto both surfaces via the S4-c fixed-plane corrector) + full `nn::least_squares` re-project of the minimum; accepted only if `‖A−B‖ ≤ onSurfTol` and the sine at/near the floor, else DEFER (no fabricated B). **ENUMERATE ARMS** — from the relative second fundamental form `H = II_A − II_B` tangent-cone quadratic: `Δ>0` ⇒ two real distinct tangent lines ⇒ up to four rays; `Δ≤0` ⇒ EMPTY (definite ⇒ isolated `TangentPoint`, END; double root ⇒ cusp, DEFER) — real distinct roots only, never fabricated, the same discriminant sign as S4-b `TangentPoint`. **ROUTE** — step `h₀/8` off B along each ray, S4-c-correct back on, S3-walk to termination (drop the arm if `S₀` fails on-both-surfaces or no progress). **ASSEMBLE** — retrace-dedup arms, merge shared branch-point connectivity into the `BranchNode` (`point`/`branchSine`/`armLineIds`), `++branchPoints`; a non-resolvable branch STOPS + defers exactly as S4-c. Both gates green: host `test_native_ssi_marching` **12 cases, 0 failed** (adds `march_steinmetz_branch_points_s4d` — Steinmetz FULLY traced, `branchPoints==2`, `nearTangentGaps==0`, 4 arms → 2 crossing ellipses — + `march_tangent_point_never_branches_s4d` — isolated `TangentPoint` ends, `branchPoints==0`, zero arms; **NUMSCI OFF CTest 26/26** S4-d TU empty, **NUMSCI ON CTest 31/31**) + sim `run-sim-native-ssi-s4d.sh` **8 passed, 0 failed** — `eq-cyl s4d`: the **Steinmetz** bicylinder that S3+S4-c TRUNCATE at the saddle is now FULLY traced (`branchPts=2` at `(0,±1,0)`, branch sine ≈ 5e-8, `traced=4` `BranchArc` arms, `NTgaps=0`, onCurve ≤ 1.74e-6 / onSurf ≤ 1.07e-8; both branch points match the OCCT saddles). The isolated `TangentPoint` (spheres d=R₁+R₂) STILL ENDS with zero arms; `eq-cyl defer` (flag off) STILL defers; the S4-c graze STILL crosses (`crossed=22`); the 5 transversal pairs `nt=0` bit-identical. No regressions (`run-sim-suite.sh` 221/221, S5 native-pass=5 persists). **Honest caveat:** OCCT tolerance-splits the locus into `occtBr=7` arc segments while the native tracer assembles 4 `BranchArc` arms meeting at the 2 branch points — an arc-SEGMENTATION disagreement, not a geometry one; the gate reconciles the branch STRUCTURE (2 branch points, two crossing ellipses) + the uncontested on-locus/on-surface facts, not equal arc counts. **STILL DEFERRED → OCCT (honest):** general/freeform branch points, three-plus tangent lines at one point, cusps (double root), singularities (S4-e), self-intersection completeness (S4-f). **Steinmetz is now unblocked** natively; the S5 boolean on a multi-arm branch `TraceSet` is a later slice. Living change `add-native-ssi-s4d-branch-points` (archived `2026-07-04`). See the SSI-S4d result section + `openspec/SSI-ROADMAP.md`. |
| 5·SSI | `native-ssi` (SSI-ROADMAP **S4-e** CHART SINGULARITIES, FIRST SLICE) | **first slice done at the bar** | A **chart (removable) singularity** — where ONE surface's own `(u,v)` parametrization degenerates (`‖dU‖ → 0`) while its 3D point + normal stay finite: a **sphere parametric pole** (`v=±π/2`) or a **cone apex** (signed radius `=0`). The intersection can be perfectly TRANSVERSAL through it (pair sine need NOT collapse), yet S3's single-surface predictor `advanceParams` goes rank-1 there and truncates (spurious `BoundaryExit` on the sphere / node-budget step-crawl on the cone). Distinct from S4-c (surface *pair* grazes) and S4-d (*locus* self-crosses). Additive to `src/native/ssi/marching.{h,cpp}` + new OCCT-free `src/native/ssi/chart_singularity.h`, gated `CYBERCAD_HAS_NUMSCI`, default-**off** `enableChartSingularities`, no `cc_*` change, tessellator byte-identical, `src/native/**` OCCT-free. Four parts: **single-surface chart witness** (`chartConditionAt` finite-differences each surface's `‖dU‖` vs `‖dV‖·scale`; a collapse with finite normal flags THAT surface — NOT the pair sine, NOT a locus flip; a finite cap keeps a genuine boundary from being mistaken for a pole); **point-based fixed-plane-cut crossing** (`crossChartSingularity` makes bounded fine POINT-BASED jumps along the fixed last-good tangent `t★` — the branch_point.h/S4-c cut — which never touches the degenerate `dU`); **loose chart map-back** (sphere pole → opposite meridian `u+π`; cone apex → far nappe `v→−v`; the singular point itself never emitted); **honest guard** (a node is emitted ONLY if it verifies on BOTH surfaces ≤ `onSurfTol` with real along-`t★` progress; on ANY failure the whole band is DISCARDED (roll back) and the march STOPS + defers → OCCT as a `NearTangent` gap — no pole/apex point fabricated). Both gates green: host `test_native_ssi_s4e_singularities` **5 cases, 0 failed** (NUMSCI OFF CTest **26/26** S4-e TU ABSENT, NUMSCI ON CTest **32/32**) + sim `run-sim-native-ssi-marching.sh` **10 passed, 0 failed** — `sphere-pole s4e singX=2 NTgaps=0 closed=1`: a great circle crossing BOTH poles that S3 TRUNCATES at half loop (`len ≈ 3.1415`) is now FULLY traced (`len` native 6.2829 vs OCCT 6.2832, rel Δ 5.04e-05, on locus + both surfaces ≤ 1.51e-07); `cone-apex s4e singX=1 NTgaps=0 nodes=159 v1=[-2.00,2.00]`: a double-cone∩plane line through the apex that S3 STEP-COLLAPSES at is now FULLY traced across both nappes (159 nodes, on-locus 7.11e-16 / on-surface 6.79e-16). A genuine finite cylinder `v`-cap still exits as a `BoundaryExit` (chart machinery does NOT misfire); with the flag ON the S4-c graze still `crossed=22` and the S4-d Steinmetz still `branchPts=2 traced=4`. No regressions (`run-sim-suite.sh` 221/221, 5 transversal pairs `nt=0` bit-identical, S5 native-pass=6 persists). **STILL DEFERRED → OCCT (honest):** general/freeform parametric singularities (NURBS degenerate edges, collapsed spline poles), higher-order/curve cusps, S4-f self-intersection completeness; any pole/apex that will not verify on both surfaces defers the same way. Living change `add-native-ssi-s4e-singularities` (archived `2026-07-05`). See the SSI-S4e result section + `openspec/SSI-ROADMAP.md`. |
| 6 | `native-blends` | ◐ tractable planar slice done at the bar (BOTH gates green); curved CIRCULAR cyl↔plane fillet (CONVEX + CONCAVE constant-radius, AND VARIABLE-radius LINEAR-law convex via `cc_fillet_edges_variable`) since landed native via #6/#6b (see the native-curved-fillet result table); non-linear-law/concave-variable/cyl↔cyl-canal/non-circular-crease/curved-edge-chamfer/fillet_face still OCCT-fallthrough | Native `cc_chamfer_edges` / `cc_fillet_edges` (constant radius) / `cc_offset_face` / `cc_shell` for the tractable PLANAR cases (`src/native/blend/`), each editing the solid's oriented-planar-polygon soup (`boolean/extractPolygons`) and re-welding a watertight solid via `boolean/assembleSolid`, then a MANDATORY engine self-verify (`blendResultVerified` — watertight + sane volume sign: chamfer/fillet/shell shrink, offset grows/shrinks) that DISCARDS a bad candidate (never faked). Native: **chamfer** = slice the convex corner off with the plane through the two setback lines (EXACT for a box); **fillet** = rolling-ball tangent cylinder on a convex planar dihedral (Phase-3 dihedral construction: axis ∥ crease, radius r, tangent to both planes), deflection-bounded facets, blend face a `Cylinder` of radius r, watertight; **offset_face** = slide a planar face along its normal dragging the side faces (EXACT slab); **shell** = inset kept walls inward by thickness + native BSP-cut the cavity (open-top box t=1 → wall vol 424). Both gates green: host `test_native_blend` (10 cases incl. 2-edge chamfer exact + concave-L-prism fallthrough) + 5 new `test_native_engine` facade cases (incl. a native `cc_edge_polylines` regression), host CTest **18/18** (no OCCT); sim native-vs-OCCT parity `native_blend_parity.mm` **[NBLEND] 16/16** — chamfer (vol o=995 n=995 rel 2.29e-16) / offset (1500, rel 4.55e-16) / shell (424, rel 4.02e-16) EXACT + watertight, constant-radius fillet deflection-bounded (o=997.854 n=997.765 rel 8.96e-05, watertight), a curved-rim fillet forwarded to OCCT (rel 0.00e+00), the self-verify rejecting a thickness-6 shell (id 0, honest error). No regressions (`run-sim-suite.sh` 221/221, `test_native_tessellate` 13/13 green). STILL OCCT-fallthrough (builder NULL / self-verify discards → forwarded, never faked): CURVED-face inputs beyond the #6/#6b circular cyl↔cap slices, `cc_fillet_edges_variable` beyond the convex linear-law circular slice (non-linear law / concave-variable / cyl↔cyl-canal / non-circular crease), `cc_fillet_face`, ≠2-face edges, multi-edge fillet interference, non-convex shell, oversized thickness. Blend fns 🟢 Excellent (≤10) except drivers `fillet_edges` (13) / `chamfer_edges` (11) 🟡 Acceptable. |
| 7 | `native-exchange` | ◐ native STEP EXPORT slice done at BOTH gates (host + sim OCCT re-read round-trip); STEP import + IGES stay OCCT (honest, out of scope) | Native `cc_step_export` (engine-wired behind `cc_set_engine(1)`) for a native solid whose every face surface + edge curve is in scope: walks the native B-rep and emits a valid ISO-10303-21 STEP AP203 file in true MILLIMETRES — HEADER (FILE_DESCRIPTION/FILE_NAME/FILE_SCHEMA 'CONFIG_CONTROL_DESIGN') + Part-42 DATA graph (CARTESIAN_POINT/DIRECTION/AXIS2_PLACEMENT_3D, VERTEX_POINT, LINE/CIRCLE/B_SPLINE_CURVE_WITH_KNOTS + EDGE_CURVE, ORIENTED_EDGE→EDGE_LOOP, FACE_OUTER_BOUND/FACE_BOUND, PLANE/CYLINDRICAL/CONICAL/SPHERICAL/B_SPLINE_SURFACE_WITH_KNOTS, ADVANCED_FACE→CLOSED_SHELL→MANIFOLD_SOLID_BREP, ADVANCED_BREP_SHAPE_REPRESENTATION + mm SI_UNIT context + PRODUCT/PRODUCT_DEFINITION/APPLICATION_CONTEXT). Built OCCT-FREE under `src/native/exchange/` (`step_writer.h/.cpp`, `native_exchange.h`). The native builders emit per-face edges (sharing deferred, #4), so the writer DEDUPLICATES geometrically — coincident vertices → one VERTEX_POINT, both faces of a physical edge share ONE EDGE_CURVE (forward on one, reversed on the other via ORIENTED_EDGE) → a properly-sewn manifold CLOSED_SHELL. Native-else-OCCT wiring: `NativeEngine::step_export` runs native for an in-scope native body; an out-of-scope native body → clean error (never a native void to OCCT); an OCCT body → `STEPControl_Writer`. **`cc_step_import` STAYS OCCT** (parsing arbitrary STEP out of scope) and **`cc_iges_export/import` STAY OCCT** — the honest end state (#8 stays blocked). No cc_* ABI change; default engine stays OCCT. Entity arg orders cross-checked against OCCT `RWStep*` writers (EDGE_CURVE/ADVANCED_FACE/CIRCLE/LINE/VECTOR/ORIENTED_EDGE/B_SPLINE_CURVE_WITH_KNOTS all match) so the file parses through `STEPControl_Reader`. Gate 1 (host, no OCCT) GREEN — `test_native_step_writer` (6 cases: canSerialize scope; box AP203 header+wrapper+mm SI_UNIT; box 6 PLANE / 12 shared EDGE_CURVE / 8 VERTEX_POINT; cylinder CYLINDRICAL_SURFACE+CIRCLE; well-formed contiguous `#n=ENTITY(...);`; coords as REALs) + `test_native_engine::native_step_export_writes_valid_ap203_file` (facade `cc_step_export` runs native, returns 1, valid file); host CTest **20/20**, all native suites green. All writer functions 🟢 Excellent (≤7), no systems-band fn. **Gate 2 (sim OCCT re-read round-trip) GREEN** — the native-written file re-reads through `STEPControl_Reader` to the SAME solid within volume/bbox/topology tolerance: box relV 2.27e-16 / area rel 1.89e-16 / centroidΔ 0 / bbox 1.00e-07 (faces 6→6, edges 24→24); cylinder relV 1.27e-03 / area rel 5.97e-04 (faces 9→9, edges 30→30); holed-plate relV 2.90e-04 / area rel 1.09e-04 (faces 7→7, edges 28→30 within tol). Writer parity (native-written vs OCCT-written, both re-read): box/cylinder/holed-plate relV ≤ 4.70e-15, relA ≤ 6.48e-15, bboxΔ 0. Native writer active (native ISO-10303-21 emitted): box 5363 B, cylinder 6893 B, holed-plate 6457 B; a foreign (OCCT-built) body falls through to OCCT `STEPControl_Writer` (15394 B → re-read relV 0.00e+00, faces 6→6). No regressions (host CTest 21/21 incl. `test_native_step_writer` #19 + `test_native_step` #20; `run-sim-suite.sh` 221/221 against a freshly rebuilt SIMULATORARM64 slice carrying the current native STEP sources). STILL OCCT (never faked): STEP import, IGES export/import, and an out-of-scope geometry kind (Ellipse/Bezier curve, rational spline, Bezier surface). Living change: `openspec/changes/add-native-data-exchange` (archived). |
| — | `native-meshing` (tet volume meshing + quality, GitHub #1) | ◐ kernel-only slice | New module `src/native/mesh/` (namespace `cybercad::native::mesh`): `quality.{h,cpp}` (ALWAYS-ON, pure geometry, no OCCT / no TetGen) + `tet_mesher.{h,cpp}` (the SOLE AGPL consumer). Additive `cc_*`: `cc_tet_mesh(body, deflection, opts)` (tessellate a B-rep → fill the PLC) + `cc_tet_mesh_surface(verts, tris, opts)` (raw closed TRIANGLE surface, OCCT-free), both emitting CalculiX **C3D4/C3D10** tets (corners + mid-edge nodes in `shape10tet` order, positive signed volume enforced, mid-edge nodes built NATIVELY — no TetGen `-o2`); plus always-on native `cc_mesh_quality` (signed volume, 6 dihedral angles, scaled Jacobian, aspect ratio — regular tet 70.53° / scaledJ 1; formulas cited: Verdict SAND2007-1751, Knupp 2001, Shewchuk 2002, Liu & Joe 1994). Tet mesher backed by the **OPTIONAL, EXTERNAL, AGPL-3.0 TetGen** (`/home/leonardo/work/tetgen`, absolute path, NEVER vendored/committed), gated behind a NEW option `CYBERCAD_HAS_TETGEN` (default OFF, mirrors `CYBERCAD_HAS_NUMSCI`; `-DTETLIBRARY` + include dir per-source-scoped to `tet_mesher.cpp` only). **Default MIT build links ZERO AGPL** — flag OFF → `cc_tet_mesh` returns empty `CCTetMesh` + `cc_last_error` ("tet meshing unavailable"), never crashes; `cc_mesh_quality` still works. Tests: `test_native_quality` (regular-tet golden / sliver flag / inverted / C3D10 mid-node consistency / empty-degenerate) runs in EVERY build; `test_native_tet` (unit cube → watertight C3D4/C3D10 at `pq1.4a…`, positive Jacobian, volume conservation, face manifoldness, quality gate) gated `CYBERCAD_HAS_TETGEN=ON`. Built via `scripts/build-tetgen.sh` (external sources → `build-tet/host/libtetgen_host.a`; predicates -O0, tetgen -O2). **Honest scope: kernel-only** — NO FE patch test yet; CalculiX++ `CadMesher` wiring (import / heal / triangulate / tet_mesh / quality / `map_to_model`) is a follow-up. Closed-app shipping that links TetGen needs a TetGen commercial license. See the native-meshing result section. |
| — | `native-healing` (FIRST shape-healing slice — internal, gates STEP import) | ◐ first slice done at BOTH gates; arbitrary-broken-B-rep residual OCCT-fallthrough | INTERNAL OCCT-free healer `src/native/heal/` (`healShell` — no `cc_*` entry, like SSI) that stitches a coincident-within-tolerance face soup / malformed shell into a watertight consistently-oriented solid via four sub-ops: **vertex/tolerance unification** (VertexPool hash generalized to B-rep vertices), **tolerant sewing** (share an edge only when its endpoints unify to the same two shared vertices within tol — never fabricated), **degenerate removal** (zero-length edges + sliver / near-zero-area faces), **orientation fix** (flood-fill outward winding + global enclosed-volume-sign tie-break). SELF-VERIFIED (watertight + `V > 0`) before it is kept; otherwise a typed `Unhealed` result carries the measured `maxResidualGap` + the ORIGINAL shape unchanged → OCCT `ShapeFix`. Engine hook `tryNativeHeal` (`src/engine/native/native_heal_hook.h`) wires native → self-verify → OCCT fallback (oracle `sewAndFix` in `src/engine/occt/occt_shapefix.cpp` — `BRepBuilderAPI_Sewing` → `ShapeFix_Shell`/`ShapeFix_Solid`). Both gates green: host `test_native_heal` (#21) **10 cases, 0 failed** (soup-cube V=1 `nMergedEdges=12`/`nMergedVerts=16` `maxResidualGap==0`; degenerate edge + sliver dropped; flipped face re-oriented; all-inward → global flip; near-coincident verts merged + beyond-tol rejected; missing face → `Unhealed(OpenShell)`; gap 1e-2 → `Unhealed(GapBeyondTolerance)` residual 0.0255 input-unchanged; never weakens tolerance — green NUMSCI OFF **and** ON) + sim native-vs-OCCT parity `run-sim-native-heal.sh` **`[NHEAL]` 4/4** (in-scope soup-cube + flipped-face heal to V=1 watertight matching OCCT V=1 valid; un-healable gap-1e-2 → UNHEALED reason=1 residual 0.0255 and missing-face → UNHEALED reason=2 both MATCHING OCCT valid=0 watertight=0 — no fabricated closure). No `cc_*` change; `src/native/**` OCCT-free; tessellator pristine; no regressions (host CTest 28/28, `run-sim-suite.sh` 221/221, all controls at baseline). **This is the gating foundation for a future native STEP IMPORT** (imported B-rep arrives with exactly this coincident-within-tolerance / degenerate / orientation defect family). **Honest scope (asymptotic, like SSI S4-f): a MEASURED win vs OCCT on the in-scope family, NOT a guarantee.** STILL OCCT (UNHEALED → `ShapeFix`, never faked): beyond-tolerance gap bridging, missing-pcurve reconstruction, self-intersecting-wire repair, freeform re-approximation, arbitrary broken industrial B-rep. Living change `add-native-shape-healing` (archived to `openspec/specs/native-healing`). See the native-healing result table. |
| 8 | `drop-occt` | ☐ planned (blocked) | Unlink OCCT once every capability is native — blocked while STEP import + IGES + curved/general booleans remain OCCT-backed. |
