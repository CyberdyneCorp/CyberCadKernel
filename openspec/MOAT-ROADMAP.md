# Dropping OCCT — the Moat Roadmap

The **complete remaining path** from "substantially native + OCCT fallback" to `#8
drop-occt` (unlink OCCT entirely). Everything reachable by a *bounded* native slice has
landed (see [NATIVE-REWRITE.md](NATIVE-REWRITE.md) and [SSI-ROADMAP.md](SSI-ROADMAP.md));
what remains is the **research-grade moat** — the small set of genuinely hard capabilities
that repeatedly blocked the bounded slices, plus the two *asymptotic* robustness tails that
have no finite "done" line.

Parent: [NATIVE-REWRITE.md](NATIVE-REWRITE.md) (#8 `drop-occt`). Enabler already built:
[SSI-ROADMAP.md](SSI-ROADMAP.md) (S1–S5 curve pipeline + curved booleans).

## The non-negotiable discipline (every stage, no exceptions)

**OCCT is the ORACLE throughout implementation — it is NOT removed until the capability it
backs is PROVEN native.** This is the same rule every landed tier used and it does not
relax for the moat:

1. **Two gates per capability.** (a) *Host analytic* — the native result matches a
   closed-form / independent computation with no OCCT linked. (b) *Sim native-vs-OCCT* —
   on a booted iOS simulator (OCCT linked) the native result matches the OCCT oracle
   (`BRepAlgoAPI` / `BRepFilletAPI` / `BRepMesh` / `GeomAPI_IntSS` / `IntPatch` /
   `STEPControl_Reader` …) on volume / area / watertightness / topology / continuity.
2. **Self-verify → OCCT fallback, always.** The native builder returns NULL when it cannot
   robustly build; the ENGINE runs the mandatory watertight + correct-value self-verify and
   DISCARDS a bad native result, falling through to OCCT. A wrong/leaky native result is
   never emitted.
3. **No fabrication, no dead code, no weakened tolerances.** An honest decline (with the
   measured gap + the specific blocker) is a first-class outcome. Capabilities that cannot
   be built are documented, not faked. (Session precedent: general-branched booleans =
   geometrically impossible; NURBS booleans = decline at recognition; curve cusp = the S4-c/d
   witness by the IFT; foreign rational B-spline = the M0 mesh gap below.)
4. **`src/native/**` stays OCCT-FREE; the `cc_*` ABI is additive-only.** OCCT lives only in
   `src/engine/occt` (the oracle + fallback). The final `drop-occt` step deletes that engine
   — but only once every stage below is native AND the completeness bar (M6) is met.

## The keystone finding (why the moat has the shape it does)

The bounded slices this project shipped repeatedly declined at **one recurring blocker**:
a general **foreign B-spline surface patch cannot be tessellated watertight**. Our *own*
B-spline faces mesh (they are bare-periodic `VERTEX_LOOP` faces from revolution/extrude);
a *foreign* B-spline patch has trimmed `EDGE_LOOP` bounds with pcurves the native mesher does
not handle. This single gap sits under **freeform booleans, freeform blends, freeform
wrap-emboss, AND foreign STEP import**. It is stage **M0** — the keystone — and unblocks the
most downstream work per unit effort.

## Stages (dependency order)

Effort is given as **first robust slice** → *the honest bounded target*; the two asymptotic
stages (M6, and the tail of M5) never fully close.

### M0 — Freeform surface meshing + trimming (the keystone) · ~1.5–3 py
A native tessellator path for a **general trimmed B-spline/NURBS face**: pcurve-bounded
`EDGE_LOOP` trimming, watertight edge-shared meshing of a rational/non-rational patch, and
the pole/degeneracy handling already proven for revolution surfaces generalised to arbitrary
patches. **Unblocks M1, M2, M3, and foreign-B-spline STEP import (M4).**
- *Oracle:* `BRepMesh_IncrementalMesh` watertightness + area/volume of the meshed solid;
  the foreign-rational-B-spline STEP fixture that currently declines (M0 is exactly the gap
  that decline exposed).
- *Bounded.* This is hard but finite: a trimmed-NURBS mesher is well-understood engineering.
- **Status — narrow slice LANDED (mesher keystone).** The interior-sampling gap the foreign
  patch hit is closed: a genuinely-trimmed curved free-form face (Bézier/B-spline, rational or
  not, with an `EDGE_LOOP` outer bound and inner holes) now meshes with its INTERIOR sampled —
  a constrained-Delaunay triangulation (`uv_triangulate.h::ConstrainedDelaunay`) folds a
  curvature-driven interior grid into the shared-edge boundary and refines to the deflection
  bound. Previously such a face took the boundary-only ear-clip path, leaving the curved
  interior unresolved (chord deflection ≈ the whole bump height, unbounded — the decline the
  STEP foreign-rational-B-spline slice exposed). Proven host-side: a trimmed bump-cap face
  meshes ON the surface within the deflection bound; a cylinder capped by a trimmed free-form
  patch meshes WATERTIGHT with the closed-form volume, converging as deflection → 0
  (rel-vol 6.0 %→1.4 % as deflection halves). The change is STRICTLY ADDITIVE — a new arm in
  `face_mesher.h::mesh()` reached ONLY by a curved genuinely-trimmed free-form face; every
  existing surface kind's mesh is BYTE-IDENTICAL (triangle count / watertight / area / volume
  diffed against `main`), and the full host suite (29/29) + NUMSCI-ON pass. Whitespace: no
  tolerance weakened, no `cc_*` ABI change, tessellator otherwise pristine.
- **Deferred admission NOW LANDED (via M4).** The STEP READER ADMISSION of a foreign
  `B_SPLINE_SURFACE_WITH_KNOTS` face — a faithful curved-edge B-spline-surface pcurve arm in
  `step_reader.cpp` + a `S_face(pcurve(t)) = C_edge(t)` faithful-reconstruction guard (decline →
  OCCT on any unfaithful edge) — shipped in **M4** and its rational sibling in **M4-rational**,
  verified vs OCCT `STEPControl_Reader` + `BRepMesh` on the simulator (STEP import parity 83/83).
  The guard tolerance coincides with the mesher's weld snap radius (`kSnapEps = 1e-6`), so passing
  the guard *guarantees* a watertight seam. So the M0 keystone is now complete end-to-end (mesher +
  admission); the last B-spline import residual — a foreign rational-*curve* trim/edge boundary —
  now imports native too (M4-tail-2, combined `RATIONAL_B_SPLINE_CURVE` edge geometry, verified vs
  OCCT `STEPControl_Reader`, SIM parity 90/90). The remaining import tail is AP242 PMI semantics +
  `MAPPED_ITEM` (Form-B) assemblies.

### M1 — General freeform surface–surface intersection robustness (SSI S4 general) · ~2–5 py
Extend the SSI marcher (S1–S5 + S4-a…e already native) to the **general/freeform** degeneracy
regimes still deferred: general/freeform branch points (S4-d beyond Steinmetz), general
near-tangent (S4-c breadth), coincident/overlapping freeform surfaces, and self-intersection
resolution. The curve *pipeline* exists; this is the *robustness* on adversarial freeform input
— OCCT's decades-deep `IntPatch`/`IntWalk` tuning, re-earned incrementally.
- *Oracle:* `GeomAPI_IntSS` / `IntPatch` curve match (onSurf, arc length, branch/loop counts).
- *Partly asymptotic* — ships as progressively hardened slices; whatever is not robust defers.
- **Status — first slice LANDED (freeform S4-d open-arm branch point).** The first FREEFORM
  branch point traced to completion, beyond the analytic Steinmetz case: a bicubic B-spline saddle
  tangent to a plane through its saddle point self-crosses at one branch point whose four arms
  radiate OPEN to the finite patch boundary (unlike Steinmetz's CLOSED branch-to-branch arcs).
  `reclassifyBranchArcs` recognised only the closed topology (both arc ends on a branch node);
  it is generalised with an OPEN-ARM rule (one end on a branch node, the other on the boundary).
  Verified native-vs-OCCT (`run-sim-native-ssi-marching` 12/12): `saddle s4d-g` traced=4 arms,
  onCurve 8.9e-08 / onSurf 5.1e-10, matching OCCT `GeomAPI_IntSS`'s 4 locus branches; a
  definite-contact B-spline bump honestly ends with no arms; all 11 prior controls frozen.
  STRICTLY ADDITIVE (`ssi/marching` only, +244/-11). The change proposal's own remedy —
  Richardson bias-cancellation of `relativeSecondForm` — was empirically REFUTED in diagnosis
  (central difference is already O(δ²)) and NOT shipped (no dead code). REMAINING (asymptotic
  tail): general near-tangent breadth, coincident/overlapping freeform, self-intersection.
- **Breadth attempt (M1-breadth) DECLINED — with a sharpened next-blocker.** A second freeform
  regime was attempted and honestly declined; no code shipped (`src/native` byte-identical). The
  diagnosis empirically refuted its own scoped pick (S4-f self-intersection *resolution*): on every
  Gerono-lemniscate fixture the marcher actually produces, it traces ONE lobe only (or laps the whole
  figure-eight ~2×, ~27 near-origin passes) — so no single clean transverse self-crossing exists to
  partition; a resolution pass would be defer-only dead code or forced fabrication. **The real
  blocker is upstream: marcher COMPLETENESS/dedup at the self-crossing** (trace both lobes exactly
  once, cross the origin transversally once) — that must land *before* any sub-arc partition is
  meaningful. Of the other two regimes: S4-a coincident/overlapping freeform is the only one with a
  clean *native* result (`detectOverlap` lands `OverlapSubRegion`, 0 undecided) but its decisive
  `IsDone=false`/`TheSame` OCCT oracle is only checkable on the sim (not at diagnose); S4-c general
  near-tangent stays the hard moat core.
- **M1-tail completeness attempt (M1-c) DECLINED — and CORRECTED the direction above (a theoretical
  result).** The "marcher self-crossing completeness (both lobes once + single *transverse* origin
  crossing, `branchPoints==0`)" slice was attempted and proven **geometrically self-contradictory**,
  so it is not the M1-tail's next slice after all. Two measured findings, no code shipped
  (`src/native` byte-identical, controls frozen — marching 14/14, S4-f 6/6): (1) **Analytic proof** —
  for a plane `z=0` ∩ graph `z=g(x,y)`, a self-crossing needs `g=0 ∧ ∇g=0`, and THERE the normals are
  parallel (`nB ∝ (−gₓ,−g_y,1) = (0,0,1) = nA` ⟹ `‖nA×nB‖=0`) — so **every such self-crossing is a
  tangent-degenerate S4-d BRANCH POINT, never a `branchPoints==0` transverse crossing**. (2) The
  canonical Gerono B-spline can't represent the sharp origin saddle (`z(0,0)=+9.7e-4`, rounded
  upward) — the locus is actually **two DISJOINT loops** (~0.05 gap), not a figure-eight, and the old
  `selfIntersections==1` was a false-positive proximity flag on a single pinched loop. **Corrected
  direction:** both-lobe coverage is reachable NOT via a `branchPoints==0` completeness pass but via
  either (a) the *existing* S4-d `enableBranchPoints` path on an exact/denser fixture (already yields
  both lobes, arc 6.27, `branchPoints=1`) or (b) a **seeder-dedup** fix so each disjoint spline lobe
  gets a seed → two `Closed` loops. The M1-tail's real next slice is one of those (seeding / branch-
  routing), not self-crossing completeness.

### M2 — General freeform booleans · ~2–4 py · needs M0 + M1
Lift `recogniseCurvedSolid` to accept **freeform (B-spline/NURBS) operands** (it rejects them
today — the S5 assembler is analytic-only), split freeform faces along the S3/M1-traced WLine,
classify + weld watertight (the M0 mesher). This is the general-curved-boolean payoff the
whole SSI arc was built for.
- *Oracle:* `BRepAlgoAPI_{Fuse,Cut,Common}` volume/area/watertight on NURBS↔NURBS and
  freeform↔analytic pairs.
- *Bounded per family, asymptotic in full generality* (arbitrary self-intersecting inputs).
- **Status — first-slice attempt DECLINED (honest, no code shipped).** The Wave-2 slice tried
  to lift `recogniseCurvedSolid` to admit ONE tractable freeform operand pair (freeform↔analytic,
  COMMON/CUT) and route it through split→classify→weld. It DID NOT land: the freeform boolean
  requires a from-scratch subsystem whose entry points do not yet exist, and no half-built dead
  code was left behind (working tree clean, `src/native` byte-identical, `src/native/**`
  OCCT-free). The measured blockers, confirmed in source at
  `src/native/boolean/ssi_boolean.h`:
  - **B1 — recognition rejects freeform.** `recogniseCurvedSolid` (line 200)
    `default: return std::nullopt; // BSpline / Bezier → freeform → OCCT` — only
    Cylinder/Sphere/Cone surfaces are admitted; any B-spline/NURBS face declines before any
    trace runs. The S5 assembler is analytic-only.
  - **B2 — no freeform face split.** There is no path to split a trimmed freeform face along the
    S3/M1-traced WLine into in/out sub-faces (only analytic caps/walls are assembled).
  - **B3 — no freeform point-in-solid.** `classifyPoint` (line 247) evaluates only the analytic
    `CurvedSolid` wall (cylinder/sphere/cone radial/spherical/conic half-space + planar caps);
    it has no point-in-freeform-solid classifier, so in/out labelling of split fragments is
    unavailable for a freeform operand.
  - **B4/B5 — no freeform weld/assembly.** ✅ **RESOLVED (B4 landed):** `boolean/half_space_cut.h`
    splits each analytic face the cut plane crosses, synthesises the cross-section cap from the B2
    seam spliced to the analytic-face crossing chords, welds the survivors through the M0 mesher,
    and runs the mandatory watertight self-verify — composing the first freeform↔analytic CUT. The
    **B2 smooth-trim (closed/circular wall) generalisation ✅ LANDED** (`boolean/smooth_trim_split.h`
    `splitFaceSmoothTrim`, host gate 7/7 — see the M2 row); the remaining enabler is the curved-wall
    half-space WELD that consumes it.
  Both mandatory gates remain **green with zero regression**: host-analytic native suites pass
  (incl. `freeform_bspline_face_operand_declines_before_trace`, which directly exercises B1), and
  the sim native-vs-OCCT run is `24 passed / 0 failed / 6 fell-back (native-pass=18)` — the 6
  fall-backs are exactly the `sphere×box` and `cone×box` freeform/box families across all three
  ops, each logging `recogniseCurvedSolid … gate declines → native NULL → OCCT`. No wrong or leaky
  result is ever emitted: freeform operands honestly self-decline to the OCCT oracle. This is a
  first-class HONEST DECLINE for a research-grade stage; the B1–B5 subsystem is bounded per surface
  family and remains the next critical-path target. No OpenSpec change was archived (nothing landed);
  this roadmap entry is the tracker for the measured blocker.
- **M2 substrate — B1/B2/B3 broken out as parallelizable tracks (start NOW).** The decline is not
  "M2 is hard" — it is three *nameable, bounded* subsystems the analytic S5 assembler never needed.
  Each operates on the EXISTING native topology (`shape.h` trimmed-B-spline `FaceSurface` + `PCurve`)
  plus a landed enabler, and each has its OWN standalone OCCT oracle — so **B2 and B3 can be built
  and verified concurrently right now, independent of each other and of B1**:

  | Track | Subsystem | Module | Needs (all landed) | Standalone OCCT oracle | Start now? |
  |---|---|---|---|---|---|
  | **M2b (B2)** | WLine freeform face-split — partition a trimmed freeform face's uv domain along the M1 seam pcurve into in/out sub-faces | `boolean/` (+ `ssi/`) | M1 WLine (`WLinePoint` carries `(u1,v1)`/`(u2,v2)` per node) ✓, `shape.h` faces | sub-faces tile the original + each meshes watertight via the M0 mesher | **YES — parallel** |
  | **M2c (B3)** | Freeform point-in-solid membership — ray-cast / winding against the M0-meshed trimmed faces | `boolean/` | M0 mesher ✓, `shape.h` faces | `BRepClass3d_SolidClassifier` (per-point in/out) | **YES — parallel** |
  | **M2a (B1)** | Freeform operand descriptor + recogniser — the data model B2/B3 plug into (additive sibling of `recogniseCurvedSolid`) | `boolean/` | `shape.h` | host gate: admit + round-trip + decline battery (assembly vs OCCT once a B4 weld verb exists) | ✅ **LANDED** — `freeform_operand.h` |

  Sequencing: **M2b ∥ M2c now** (two isolated tracks, disjoint algorithms, each self-verifiable);
  **M2a** is the small integration layer, best sized once B2/B3 fix their interfaces. Then
  **M2 = M2a ∘ M2b ∘ M2c** (recognise → trace [M1] → split [B2] → classify [B3] → weld [M0]) — the
  assembly is what turns three green subsystems into the first freeform boolean. The live critical
  path refines to **[B2 ∥ B3] → B1 → M2 → M3**.

  **M2c (B3) — first slice LANDED (this change).** `src/native/boolean/freeform_membership.h`
  (header-only, OCCT-free, cognitive-complexity ≤ 12 per function): isolated Möller–Trumbore and
  exact point-triangle-distance kernels, and `classifyPointInMesh(mesh, bbox, meshDeflection, p,
  tol) → {In,Out,On,Unknown}`. Odd/even crossing parity over a fixed 7-ray non-axis set with
  degenerate-ray discard + unanimity/quorum consensus; a principled ON-band
  `max(absTol, relTol·diag) + 2·meshDeflection` (the measured chord-secant mesh inset); watertight
  precondition ⇒ `Unknown` (never a fabricated verdict). Strictly ADDITIVE — `classifyPoint`/
  `recogniseCurvedSolid` and the whole tessellator are byte-identical vs `main`; no `cc_*` ABI change.
  - **Gate A (HOST ANALYTIC, no OCCT)** — `tests/native/test_native_freeform_membership.cpp`.
    Fixture: the M0 keystone `bumpCappedCylinderSolid` (Bézier paraboloid "bump cap" top wall,
    closed-form membership), meshed watertight at deflection 0.02. Away-from-band batch (3000 pts,
    clearance > 3·band): **agree 3000 / WRONG 0 / declined 0**. On-surface samples all resolve `On`.
    Full 40 000-pt random batch: **crispWRONG 0** (the load-bearing no-silent-wrong invariant).
  - **Gate B (SIM native-vs-OCCT)** — `tests/sim/native_freeform_membership_parity.mm` (runner
    `scripts/run-sim-native-freeform-membership.sh`). Bridged NURBS solids (all faces
    `Geom_BSplineSurface`) meshed with M0, classified vs `BRepClass3d_SolidClassifier`.
    `nurbs_box` (watertight): **N=3000, crispAgree 2933, crispDISAGREE 0**, in-band/declined 67 → GATE PASS.
    `nurbs_cylinder` (curved): an **honest R1 decline** — the bridged-freeform M0 mesh is non-watertight
    (measured 273 open edges: the periodic BSpline seam edge does not weld), so the classifier declines
    to `Unknown` rather than fabricate. This is the asymptotic curved-bridged tail, not a classifier
    defect (the curved case is proven crisp-correct against analytic truth in Gate A).

  **M2a (B1) — descriptor + gate LANDED; end-to-end assembly HONEST-DECLINED (this change).**
  `src/native/boolean/freeform_operand.h` (header-only, OCCT-free, backend-band): the
  `FreeformOperand`/`OperandFace`/`FaceRole` value descriptor + `recogniseFreeformSolid(Shape,
  OperandDecline*) → optional<FreeformOperand>`, a strictly ADDITIVE SIBLING to the analytic
  `recogniseCurvedSolid` (which — with `classifyPoint`, M0, M1, B2, B3 — is byte-identical vs the
  landed tree; no `cc_*` ABI change; 0 OCCT includes under `src/native/**`). The gate admits ONE
  reachable operand — a non-null single-shell `Solid` with ≥1 genuinely-trimmed BSpline/Bezier wall
  + only admissible analytic caps (plane/cylinder/sphere/cone), closed as a 2-manifold (every
  undirected edge, keyed by endpoint-vertex identity, used by exactly two face incidences) — and
  exposes exactly the M2 verbs' handles: the freeform `Face` (→ B2 `splitFace`), the operand `Shape`
  (→ M0 `SolidMesher::mesh`), the world `Aabb` (→ B3 `classifyPointInMesh`). Every other operand
  returns `nullopt` with a measured `OperandDecline` (NotSolid / MultiShell / FaceSurfaceMissing /
  UnsupportedSurfaceKind / BareFreeformFace / HoledFreeformFace / NoFreeformFace / NotWatertight) —
  a first-class honest decline, no weakened tolerance.
  - **Gate A (HOST ANALYTIC, no OCCT)** — `tests/native/test_native_freeform_operand.cpp`
    (14/14 pass). On the M0 keystone `bumpCappedCylinder` (cylinder side + planar bottom + Bézier
    bump cap): ADMITTED; 3 faces role-tagged (1 freeform + 2 analytic); the freeform surface/kind
    round-trips bit-identically; `outwardN` materially outward on every face; `bbox` tight
    (x,y ∈ [−R,R], z ∈ [0,h], diagonal usable); `watertight` true. Full decline battery green
    (each blocker measured). Exposed handles proven: the `Shape` meshes watertight under M0; the
    descriptor `Aabb` scales B3 (interior→In, exterior→Out).
  - **Assembly HONEST DECLINE (measured blocker).** The stretch — the minimal end-to-end
    freeform↔analytic CUT/COMMON composing only the four landed verbs (recognise → M1 trace → B2
    split → B3 classify → M0 weld) — is **not robustly reachable**, so B1 lands ALONE (an accepted,
    expected outcome). Two independent, measured blockers: **(i)** the sole freeform wall of the
    reachable freeform-SOLID class carries a smooth CLOSED (circular) trim, but B2 `splitFace`'s
    first slice requires a convex straight-edged outer loop (≥3 boundary segments) — it returns
    `NoOuterLoop` (measured in the gate). **(ii)** even a polygon-trimmed freeform wall does not
    close the boolean: a half-space cutting a closed solid also crosses its ANALYTIC cap/side faces
    (needing an analytic-face splitter) and synthesizes a NEW cross-section cap on the cutting plane
    — neither an analytic-face split nor a cap synthesizer is among the four landed M2 verbs (B2 is
    freeform-only). No stub/dead `freeform_boolean_solid` was written; the engine keeps its OCCT
    fall-through. Closing the gap needs a B2 convex→smooth-trim generalisation **and** a B4 analytic-
    face split + cross-section cap synthesis (the M2 weld/assembly verb), tracked next.

### M3 — General freeform blends + wrap-emboss · ~2–4 py · needs M2
The curved-curved blends and freeform-base features that sit on booleans: cyl↔cyl-canal /
elliptical-crease / variable-on-freeform fillets, general chamfers, and wrap-emboss on a
sphere/cone/freeform base (all declined this session for lack of M0/M2). General canal-surface
construction + the M0 mesher + M2 booleans.
- *Oracle:* `BRepFilletAPI` / `BRepOffsetAPI` / `cc_wrap_emboss` (volume/area/watertight/continuity).

### M4 — General STEP / AP242 import (+ IGES stays dropped) · ~1.5–3 py · needs M0
The remaining import breadth on the landed AP203+ reader: **foreign rational/general B-spline
patches** (needs M0 to mesh them), AP242 **PMI semantics** (not just skip), general **trimmed
surfaces**. **DEEP-NESTED (multi-level) rigid/conformal assemblies now LAND** (M4-tail): the
Form-A CDSR relationship graph is modelled as a parent-edge forest over shape-representations
and each leaf composes its full world placement `W = T_root ∘ … ∘ T_leaf` via a leaf→root
chain walk (single-level is the length-1 special case, byte-identical). **`MAPPED_ITEM` /
`REPRESENTATION_MAP` (Form-B) assembly INSTANCING now LANDS too** (M4-tail, `moat-m4t-assembly-import`):
the standard AP242 assembly-reuse mechanism — a `REPRESENTATION_MAP` over a shared brep, instanced by
N `MAPPED_ITEM`s each at `T = frameToWorld(target) ∘ frameToWorld(origin)⁻¹` (AXIS2 target) or a
`CARTESIAN_TRANSFORMATION_OPERATOR_3D` (scale/mirror), reusing the SAME `classifyPlacement` /
`Shape::located` substrate as Form-A; the shared brep is mapped ONCE and re-instanced through the shared
node → a placed Compound (GATE a host-analytic + GATE b SIM parity vs `STEPControl_Reader`, both green).
A mapped rep reaching ≠1 brep, a non-conformal target, a lone `REPRESENTATION_MAP`, or a MIXED Form-A+B
file still honestly DECLINE → OCCT, as do cyclic / ambiguous-shared / non-conformal Form-A graphs. **PMI
SEMANTICS remain the deferred tail** (honest DECLINE — only the read-only `pmi_scan` census recognises
GD&T/tolerance/datum entities; no tolerance-zone / feature-control-frame / datum-reference-frame model is
built). IGES is **descoped** (STEP-only; `cc_iges_*`
stays OCCT until removed/stubbed at drop-occt — never reimplemented).
- *Oracle:* `STEPControl_Reader` re-import (count/volume/watertight/topology) + foreign files.
- *Bounded* (mechanical parser breadth, once M0 meshes the surfaces).

### M5 — Shape-healing robustness · ~2–4 py + asymptotic tail · gates M4 quality
Beyond the landed first slice (tolerant sew + vertex/tolerance unify + degenerate removal +
orientation): **pcurve reconstruction, self-intersecting-wire repair, beyond-tolerance gap
bridging, arbitrary broken industrial B-rep**. Gates trustworthy foreign import (M4) and any
`drop-occt` decision.
- *Oracle:* `ShapeFix_*` / `BRepBuilderAPI_Sewing` on broken fixtures + real foreign files.
- **Asymptotic** — a first robust slice is bounded; the completeness against arbitrary broken
  input is the decades-deep `ShapeFix` moat, re-earned only incrementally.
- **Status — first slice LANDED (opt-in beyond-tolerance gap bridging).** Beyond the landed
  sew/unify/orientation healer, a near-miss seam gap slightly ABOVE the weld tolerance — which the
  healer DECLINES — now bridges to a watertight solid. `gap_bridge.h` (new, header-only, OCCT-free)
  snaps the primary weld's unpaired boundary corners onto their cross-face partner within the
  BOUNDED band `(tolerance, min(budget, ¼·shortestIncidentEdge)]` (a hard local-feature cap so it
  never collapses a real feature), then re-sews. Opt-in (`gapBridgeBudget=0` default OFF, existing
  healer BYTE-IDENTICAL); NEVER widens the weld tolerance. Verified native-vs-OCCT
  (`run-sim-native-heal` 6/6): a bridged seam heals V=1.000 (4 corners) matching OCCT `Sewing`
  (V=1.00167); an out-of-budget gap declines honestly (`GapBeyondBudget`); the 4 landed controls
  frozen.
- **Status — tail slice LANDED (opt-in single planar-hole cap).** Beyond the sew/unify/orientation +
  gap-bridging healer, a shell that sews cleanly but is simply MISSING one face — which the healer
  DECLINES as `OpenShell` — now closes to a watertight solid. `cap_hole.h` (new, header-only,
  OCCT-free) reconstructs the surviving boundary edges from the sewn shared nodes, and when they form
  EXACTLY ONE simple cycle that is coplanar within `tolerance` and a simple polygon, synthesizes ONE
  cap face on those existing shared nodes and re-sews; the UNCHANGED mandatory self-verify then proves
  watertight + positive volume. Opt-in (`capPlanarHoles=false` default OFF, existing healer
  BYTE-IDENTICAL); NEVER weakens the weld tolerance; no new `UnhealedReason`. Verified both gates:
  host analytic (capped cube V=1.0, no OCCT; two-hole and non-planar holes decline; default-off no-op)
  and native-vs-OCCT (`run-sim-native-heal` 8/8) — the cap matches an OCCT reference cap
  (`BRepBuilderAPI_MakeFace(gp_Pln, freeBoundaryWire)` + `ShapeFix`) at V=1.0, and a two-hole shell
  declines matching OCCT leaving it open. EMPIRICAL NOTE: OCCT's `MakeFace(gp_Pln, wire)` tolerates a
  mildly-non-planar wire (keeps 3D vertices) and caps it, so native declining a non-planar hole is
  native being MORE conservative and DEFERRING to OCCT — not a shared decline. REMAINING (asymptotic
  tail): pcurve reconstruction, self-intersecting-wire repair, non-planar / curved missing-face
  synthesis, arbitrary broken industrial B-rep.
- **Status — tail slice LANDED (opt-in MULTI planar-hole cap).** Strict superset of the single
  planar-hole cap: a shell MISSING two or more faces — which the healer DECLINES — now closes when
  EVERY surviving boundary ring is a disjoint simple cycle, coplanar within `tolerance`, and a
  simple polygon. `cap_hole.h` adds `traceAllLoops` (all disjoint degree-2 boundary cycles) +
  `capAllPlanarHoles`, reusing the IDENTICAL best-fit-plane / planarity / simple-polygon layers per
  ring; the landed single-hole `traceSingleLoop` / `capPlanarHole` are BYTE-IDENTICAL. ALL-OR-NOTHING:
  if any ring branches, is non-planar, or self-intersects, the WHOLE set is declined (no partial
  closure), and the UNCHANGED mandatory self-verify remains authoritative. Opt-in
  (`capMultiplePlanarHoles=false` default OFF; when false the single-hole path runs unchanged so every
  existing caller is byte-identical); NEVER weakens the weld tolerance; no new `UnhealedReason`.
  Verified both gates: host analytic (`test_native_heal` 23/23 — cube missing two OPPOSITE faces caps
  to V=1.0 with nCappedFaces=2; mixed-planarity two-hole set declines the WHOLE set; two ADJACENT
  missing faces orphan the shared corners → `GapBeyondTolerance` decline; the reused simple-polygon
  layer rejects a bowtie; default-off byte-identical) and native-vs-OCCT (`run-sim-native-heal` 10/10
  — the two caps match an OCCT reference `BRepBuilderAPI_MakeFace(gp_Pln, wire)` ×2 + `ShapeFix` at
  V=1.0, and the two-adjacent case declines matching OCCT leaving it open). REMAINING (asymptotic
  tail): pcurve reconstruction, self-intersecting-wire repair, non-planar / curved missing-face
  synthesis, arbitrary broken industrial B-rep.

### M6 — Robustness completeness bar (S4-f + coverage) · ongoing · gates drop-occt
The measured-recall / completeness discipline (SSI S4-f landed a first slice): below any fixed
resolution a smaller intersection loop can be missed. Before OCCT can be *removed* (not just
defaulted), a **measured completeness bar** across the whole native surface — booleans, blends,
import, healing — must be met and continuously guarded (loop-until-dry critics, adversarial
fuzzing vs OCCT).
- *Oracle:* differential fuzzing — random valid inputs through both native and OCCT, assert
  agreement or an honest native decline; zero silent wrong results.
- **Asymptotic** — never "done"; this is the gate that keeps `drop-occt` a decision, not a date.
- **Status — first slice LANDED (curved-boolean differential fuzzer).** A DETERMINISTIC seeded
  generator drives random VALID operands from six recognised families (sphere∩sphere, cone∩sphere,
  cone∩cyl, Steinmetz, box∩cyl, drill) × {fuse,cut,common} through BOTH native and OCCT and
  classifies each trial AGREE / honest-native-DECLINE / DISAGREE. Two seeds, 512 trials:
  432 AGREED / 80 HONESTLY-DECLINED / **0 DISAGREED** — native genuinely exercised (real volumes
  matching OCCT, dV ~1e-3..1e-4), the fixed relTol=2e-2 NEVER widened. New
  `tests/sim/native_boolean_fuzz.mm` + `scripts/run-sim-native-boolean-fuzz.sh`; `src/native`
  untouched (pure test infra). The FIRST measured completeness signal beyond the hand-picked
  native-pass=18 fixtures.
- **Breadth — FIVE native domains now under the fuzzing bar.** (2nd) STEP round-trip
  `tests/sim/native_step_import_fuzz.mm` (0 DISAGREED; *surfaced* an OCCT reader inaccuracy on
  shallow frustums, native vindicated by closed-form). (3rd, this slice) **construction
  loft/sweep** `tests/sim/native_construct_fuzz.mm` + `scripts/run-sim-native-construct-fuzz.sh`:
  a DETERMINISTIC seeded generator (splitmix64→xoshiro256**, seeded ONLY by FUZZ_SEED) drives
  random VALID inputs from four native-claimed families — equal- AND mismatched-count planar
  N-section ruled loft (frustum / prismatoid-stack) and straight constant-frame sweep (prism) —
  through BOTH the OCCT-FREE `build_loft_sections`/`build_sweep` (measured by the native
  tessellator) AND the OCCT oracle (`BRepOffsetAPI_ThruSections`/`MakePipe`, measured by
  `BRepGProp`), plus sparse non-planar loft/sweep inputs to exercise the native NULL→OCCT
  DECLINE branch. Two seeds (0x5744EE9911 N=96 → 78/18/0; 0xDEADBEEFCAFE N=128 → 110/18/0):
  **0 DISAGREED**, every AGREE OCCT-exact (dV/dA ~1e-15), fixed relTol=2e-2 NEVER widened; an
  analytic prismatoid/prism-volume arbiter is present as a ready strengthening (untriggered —
  ORACLE-INACCURATE=0). Determinism re-verified (same seed twice → byte-identical batch).
  `src/native` untouched; on run-sim-suite.sh SKIP list (own main()).
- **Breadth — 4th native domain: BLENDS (fillet/chamfer).** `tests/sim/native_blend_fuzz.mm`
  + `scripts/run-sim-native-blend-fuzz.sh`: a DETERMINISTIC seeded generator (splitmix64→
  xoshiro256**, seeded ONLY by FUZZ_SEED) drives random VALID inputs from six native-claimed
  families — planar-dihedral chamfer + fillet (box edge), constant- AND variable-linear-radius
  curved fillet, and symmetric + asymmetric cone-frustum chamfer of a convex cylinder↔cap
  circular rim — through BOTH the OCCT-FREE native blend builder called DIRECTLY (`blend::
  chamfer_edges`/`fillet_edges`/`curved_fillet_edge`/`variable_fillet_edge`/`curved_chamfer_
  edge[_asym]`, measured by the native tessellator) AND the OCCT oracle (`BRepFilletAPI_
  MakeFillet`/`MakeChamfer`, measured by `BRepGProp`), plus a sparse out-of-scope `Rc<2r`
  fillet to exercise the native NULL→OCCT DECLINE branch. Each AGREE family carries a CLOSED-
  FORM removed-volume analytic arbiter (torus-canal Pappus fillet; cone-frustum
  π·d1·d2·(Rc−d2/3); box-edge prism/groove) so a native result matching exact math while OCCT
  is the outlier is logged ORACLE-INACCURATE (native vindicated), never a bar failure. Four
  seeds (0x5744EE9911 N=96 → 91/5/0; 0xC0FFEE1234 N=160 → 154/4/0; 0xDEADBEEF99 N=160 →
  147/10/0; 0x77A9F1E3B5 N=160 → 156/3/0): **0 DISAGREED** on every seed, planar chamfer
  native==OCCT exact, curved families deflection-bounded under a FIXED never-widened tol
  (vol=2e-2 area=3e-2; max observed bias ~1.7e-2/2.6e-2); native-vs-exact-math ≤~1.6e-3 every
  family. The variable-linear curved fillet surfaced OCCT evolved-surface drift while native
  held ~2e-4 vs exact math → logged ORACLE-INACCURATE. Concave stepped-shaft fillet +
  offset/shell are an honest domain-level decline for this first blend slice. `src/native`
  untouched; on run-sim-suite.sh SKIP list (own main(), std::_Exit).
- **Breadth — 5th native domain: WRAP-EMBOSS (feature on a cylinder lateral face).**
  `tests/sim/native_wrap_emboss_fuzz.mm` + `scripts/run-sim-native-wrap-emboss-fuzz.sh`: a
  DETERMINISTIC seeded generator (splitmix64→xoshiro256**, seeded ONLY by FUZZ_SEED) drives
  random VALID inputs from four native-claimed families — a rectangular PAD emboss (material
  added), a rectangular DEBOSS pocket, and a convex N-gon emboss/deboss — all wrapped onto a
  CYLINDER lateral face, through BOTH the OCCT-FREE native builder called DIRECTLY
  (`feature::wrap_emboss`, measured by the native tessellator) AND the PRIMARY closed-form
  curvature-corrected changed-volume oracle `A·|Rout²−R²|/(2R)`, plus a SECONDARY OCCT-boolean
  reconstruction (base cylinder FUSED/CUT with a wrapped angular-sector wedge, measured by
  `BRepGProp`) that is clean for the rectangle footprints and is the only independent AREA
  oracle. Sparse out-of-scope inputs (non-cylindrical base, >2π footprint, deboss depth ≥ R,
  self-intersecting loop) exercise the native NULL→OCCT DECLINE branch. Two seeds
  (0x5745E6B055 N=120 → 107 AGREED / 0 DISAGREED / 13 BOTH-DECLINED, per-family emboss-rect 39
  / deboss-rect 27 / emboss-poly 25 / deboss-poly 16, 66 OCCT rect reconstructions; 0xC0FFEE01
  N=240 → 204 AGREED / 0 DISAGREED / 36 BOTH-DECLINED, per-family 49/58/45/52, 107 OCCT rect
  reconstructions): **0 DISAGREED** on both seeds, rectangle AGREE matches the OCCT
  reconstruction and the closed form, polygon AGREE matches the closed form (OCCT recon
  honestly skipped for arc footprints), all out-of-scope exercisers route to BOTH-DECLINED
  (DECLINE fires, never DISAGREE), max native-vs-oracle bias vol=7.641e-3 (~2.6× under the
  FIXED vol=2e-2 area=3e-2 tol, NEVER widened). Determinism re-verified (same seed → identical
  classification counts). `src/native` untouched; on run-sim-suite.sh SKIP list (own main()).
- REMAINING (asymptotic, gates M8): extend the generator across the remaining blend families
  (concave stepped-shaft, offset/shell) and healing; loop-until-dry critics; the standing
  zero-silent-wrong-results bar.

### M7 — Tier-4 construction robustness · ~1–3 py · independent
**M7a first slice LANDED (verified native-vs-OCCT at both gates):** the N-section (≥3)
planar ruled loft is now reachable through a new ADDITIVE facade entry
`cc_solid_loft_sections`, exposing the already-proven OCCT-free `build_loft_sections`
(equal- and mismatched-count sections) with a real OCCT `ThruSections` oracle, engine
self-verify → OCCT fallback, and an honest self-verify decline for candidates whose mesh is
not robustly watertight (e.g. an asymmetric expand-then-contract spool that T-junctions the
tessellator's shared-ring seam — volume exact, mesh not; discarded → OCCT rather than
weaken the tessellator). Gate 1 (host, no OCCT): `test_native_loft` 21/21. Gate 2 (sim vs
`ThruSections`): `native_loft_parity` 39/39. Additive-only (`cc_kernel.h` no deletions),
`src/native/**` 0 OCCT includes, tessellator untouched.

**M7-tail LANDED — straight-spine guided-ORIENT sweep native (overturns the M7a decline):** M7a
honest-declined a guide-aimed sweep after measuring a rigid law that was volume-correct but ~7 %
spatially wrong (bboxΔ 0.54). M7-tail reverse-engineered OCCT's actual `MakePipeShell +
SetMode(guide, CE=false, NoContact)` law from source (`GeomFill_GuideTrihedronPlan::D0`) and found
M7a *misdiagnosed* it: the law IS a rigid per-station frame, but keyed to the guide point in the
plane **perpendicular to the spine tangent**, not a guide-parameter-fraction aim. Fixing the
correspondence fixes the trap. New additive `cc_guided_orient_sweep` (distinct from the scale-splay
`cc_guided_sweep`) + `build_guided_orient_sweep` (`sweep.h`), self-verify MANDATORILY including a
bbox/Hausdorff SPATIAL check (the explicit anti-M7a-trap safeguard), NULL→OCCT for a curved spine.
Verified vs OCCT (`native_sweep_parity` 19/19) with the spatial gate: offset guide vol rel 1.8e-16 /
bboxΔ 1e-7; rotating guide (the M7a trap) vol rel 2.95e-3 / **bboxΔ 2.49e-4** (vs M7a's 0.54).

**M7t LANDED — real-twist `twisted_sweep` + curved-rail `loft_along_rail` native (both gates):**
the last two Class-B construct tails. The recorded twist decline ("a densified twisted saddle
ruled tube does not weld watertight") was empirically *false* — the real error was a missing
densification. `build_twisted_sweep` now DENSIFIES the straight spine so each ruled band's twist
stays under a per-band bound (`kMaxBandTwist`), reusing the landed Frenet section frame +
`assembleRingTube`; the tube welds watertight at every deflection and its volume converges to the
area-preserving analytic value (a pure twist preserves the cross-section area). `build_loft_along_rail`
now serves a smooth curved rail via an RMF-transported section morph densified to a per-band turn
bound (`kMaxBandTurn`), converging to the Pappus torus-sector volume. Both keep the engine
`robustlyWatertight` self-verify → OCCT. Gate 1 (host, no OCCT): `test_native_sweep` +
`test_native_engine` green (twist → area·L, rail → Pappus). Gate 2 (sim vs OCCT
`ThruSections` / `MakePipeShell`): `native_construct_tails_parity` **8/8** — real-twist vol rel
1.82e-3 / bboxΔ 1.0e-7 / χ=2, curved-rail vol rel 3.87e-4 / χ=2. Sharpened declines: a
twist COMBINED WITH a scale (twist+shrink saddle, not robustly weldable) and a tight-kink rail /
coarse section that won't weld → engine self-verify discards → OCCT (rel 0, active=1). Additive,
`src/native/**` OCCT-free, `cc_*` unchanged, tessellator untouched.

Still deferred (honest decline): a twist+scale saddle and a tight-kink / coarse-section rail
(engine self-verify discards → OCCT); **non-planar-cap loft** (needs a filling surface, no
closed-form volume); **fine-pitch self-intersecting threads** (intersecting-helicoid trimming —
needs M1/M2). Independent of the freeform-boolean chain except fine-pitch (needs M2).
- *Oracle:* `BRepOffsetAPI_MakePipeShell` / `ThruSections` / thread fixtures (volume/watertight).

### M-REF — Reference / datum geometry + topology reads · **LANDED** (bounded, was Class-B)
**Added because the CyberCad app depends on it and no numbered stage covered it** (a MUST-GO-
NATIVE Class-B bucket, 22 app sites). The seven READ-ONLY datum / reference queries that
hard-errored on a native body now dispatch NATIVE through the new OCCT-FREE, header-only
`src/native/reference/reference.h` (`cybercad::native::reference`), computed from the landed
topology graph + frame math and consuming `src/native/{math,topology}` read-only:
- **Landed native (both gates green):** `cc_face_axis` / `cc_ref_axis_from_face` (cylinder/cone
  axis), `cc_ref_plane_from_face` (planar datum plane — outward normal + on-plane origin),
  `cc_ref_axis_from_edge` (straight-edge line axis), `cc_tangent_chain` (C1 edge walk,
  line/circle/ellipse tangents), `cc_outer_rim_chain` (planar-cap outer wire), and
  `cc_offset_face_boundary` (planar-polygon in-plane miter offset).
- **Honest declines → OCCT (measured, never faked):** a non-planar face where a plane is asked;
  a circular/freeform edge axis (no `gp_Lin` oracle); a freeform edge inside a tangent walk; and
  a non-planar / arc-boundary / **growing-convex** (OCCT arc-rounds) / self-intersecting offset —
  the offset is deliberately scoped to the case that provably coincides with
  `BRepOffsetAPI_MakeOffset`. A decline returns a clean error and the facade falls through to
  OCCT; a native void is never forwarded.
- **Gates:** host `tests/native/test_native_reference.cpp` (closed-form, OCCT-free, 13/13) + sim
  `tests/sim/native_reference_parity.mm` via `scripts/run-sim-native-reference.sh` (native-vs-OCCT
  `gp_Pln`/`gp_Cylinder`/`gp_Lin`/`BRepTools::OuterWire`/`MakeOffset`/D1-tangent, 8/8). `cc_*` ABI
  unchanged; 0 OCCT includes under `src/native/**`. OpenSpec change `moat-mref-reference-topology`.

### M-DM — Direct modeling / synchronous editing · ~1.5–3 py · needs M2 + M3 + M5
**Added because the CyberCad app depends on it and no other stage covered it.** The app drives
direct-modeling operations through the `cc_*` ABI that are OCCT-only in the kernel today:
`cc_split_plane`, `cc_replace_face`, `cc_replace_face_to_plane`, plus the project tool. These are
*local B-rep re-solving* operations (the heart of synchronous/direct modeling), distinct from the
feature-tree construction the other stages cover. Substages:
- **DM1 — `cc_split_plane`** (split a solid by a plane into pieces). **FIRST SLICE LANDED** (native,
  additive, OCCT-free). `NativeEngine::split_plane` now routes native B-rep bodies through
  `native/boolean/split_plane.h::splitByPlane`, which composes the two landed verbs unchanged:
  `freeformHalfSpaceCut` (KeepSide::Below/Above → the two pieces) for a single-freeform-walled operand,
  else `boolean_solid(operand, discard-half-space-box, Cut)` for an all-planar polyhedron. Each piece
  is accepted only after the mandatory `watertightVolume` self-verify; otherwise an honest decline
  (the same clean error the pre-DM1 path returned — a native void is never handed to OCCT).
  - *Native cases (both gates green):* axis-aligned **BOX** / planar polyhedron (host partition-closure
    fp-exact; sim volume/area rel ≤ 2.3e-16 vs OCCT) and the bowl-lidded **PRISM** with one freeform
    wall (host closed-form band; sim vol rel ≤ 5.5e-3, area ≤ 6e-4, χ=2, bbox ≤ 1e-7, partition
    closure 5.4e-3 — all inside the landed curved-slice tolerances, never widened).
  - *Honest declines → OCCT (measured, not faked):* the **perpendicular cylinder slice** is `cyl − box`,
    which the landed curved slice (`curved::tryBoxCylinder`) explicitly excludes → NULL both sides;
    likewise a grazing-tangent plane, a degenerate/missing plane, a multi-freeform operand, a mesh-only
    or foreign body. The design's original case C (native cylinder ⊥ slice) was demoted to this decline
    — adding a cylinder-slice verb would violate the consume-unchanged discipline.
  - *Gates:* host `tests/native/test_native_split_plane.cpp` (partition-closure, OCCT-free, 5/5) +
    sim `tests/sim/native_split_plane_parity.mm` (native-vs-OCCT, 31/31). Remaining ~0.3–0.7 py for
    oblique-plane / multi-lump breadth. cc_* ABI unchanged; OCCT bodies byte-identical.
  - *Zero-regression (substrate-gated):* `split_plane.h` reaches the freeform seam trace
    `ssi::trace_intersection`, defined only under `CYBERCAD_HAS_NUMSCI`, so the always-compiled
    `native_engine.cpp` gates both the include and the native-split body on `#ifdef CYBERCAD_HAS_NUMSCI`
    (native split honestly declines when OFF, as pre-DM1). Verified: NUMSCI-OFF host links
    `test_native_engine`/`test_native_boolean` (pass) with 0 `trace_intersection` refs (matches `main`),
    and `run-sim-native-boolean.sh` links + 25/25. NUMSCI-ON host suite + split-plane sim (31/31) green.
- **DM2 — `cc_replace_face_to_plane`** ✅ *(landed — native slice)* (flatten/retarget a planar face to a
  target plane, re-solving adjacent planar faces — extend neighbours to the new plane, retrim, heal).
  Native `directmodel::replaceFaceToPlane` (`src/native/directmodel/replace_face.h`, OCCT-FREE, header-
  only) composes the LANDED DM1 `splitByPlane` + `boolean_solid(Fuse)` + `build_prism`: a parallel PULL is
  one DM1 cut, a parallel PUSH is a face-loop slab fused on, a tilted/mixed move is GROW-then-TRIM (one
  Fuse + one tilted cut — NOT an N-cut half-space chain, which the DIAGNOSE probe showed breaks the
  watertight self-verify at ~4 cuts). δ is affine over the planar face, so the swept volume is the exact
  closed form `ΔV = A_F·d̄`. Mandatory re-solve self-verify (watertight closed 2-manifold, single lump
  χ=2, distinct-plane count preserved, moved face on the target plane, volume == `V₀+A_F·d̄`) → else NULL →
  honest decline. `native_engine.cpp::replace_face_to_plane` gates the native path on `#ifdef
  CYBERCAD_HAS_NUMSCI` (an OCCT body forwards to the OCCT half-space-cut oracle unchanged; a native body
  the slice can't re-solve declines honestly, never handed to OCCT). Verified BOTH gates: host closed-form
  (`test_native_replace_face`, 8/8 — parallel push/pull fp-exact, tilted trim + tilted grow-then-trim,
  and declines: curved neighbour / degenerate normal / no-op / topology-change) and sim native-vs-OCCT
  plane-cut-and-extend oracle (`run-sim-native-replace-face.sh`, 32/32, volume/area/watertight/χ/bbox
  fp-exact). Honest declines (→ OCCT): curved neighbour, non-planar picked face, degenerate/topology-
  changing/non-convex move. Bounded. ~0.5–1.5 py.
- **DM3 — `cc_replace_face` (general push-pull / move-face)** ✅ *(pure-offset slice landed — native)*
  — retarget a planar face by offsetting it along its own outward normal (and, in general, tilting it),
  trimming the solid to the new plane. Native `directmodel::replaceFaceOffsetTilt`
  (`src/native/directmodel/replace_face_general.h`, OCCT-FREE, header-only) DERIVES the target plane
  `(o + n̂_F·offset, n̂_F)` for the pure-offset case (`tiltDeg ≈ 0`) and re-solves via the byte-frozen DM2
  `replaceFaceToPlane` (grow-then-trim = 1 Fuse + 1 Cut, watertight self-verified at `V₀ + A_F·offset`).
  `native_engine.cpp::replace_face` now serves a native body (was a hard `CC_NATIVE_BODY_UNSUPPORTED`);
  an OCCT body forwards. GATE (a) HOST 6/6 (`test_native_replace_face_general` — pure push/pull fp-exact,
  off-axis face offset, + declines). GATE (b) SIM (`native_dm3_dm4_parity.mm`, booted sim, 36/36 combined with DM4) vs
  the OCCT move-face oracle: volume/area rel ≤ 3.5e-16, off-axis bbox 1.8e-15, watertight, Euler χ=2.
  HONEST DECLINE: a non-zero tilt (OCCT face-parametrization X-axis is a foreign convention we don't
  reproduce for a native body — the sharpened next blocker), a non-planar picked face, a curved neighbour
  → OCCT. Residual (general tilt / arbitrary target surface) needs M2/M3/M5 breadth. ~1–2 py total.
- **DM4 — project tool** ✅ *(analytic slice landed — native)* — additive `cc_project_point_on_face`:
  drop a point onto a face's analytic surface, returning the closed-form foot-of-perpendicular + distance.
  Native `directmodel::projectPointOnFace` (`src/native/directmodel/project.h`, OCCT-FREE, header-only) is
  the closed-form normal projection for plane / cylinder / sphere; `OcctEngine::project_point_on_face` =
  the `GeomAPI_ProjectPointOnSurf` oracle. GATE (a) HOST 9/9 (`test_native_project` — plane/cylinder/sphere
  feet + declines cone/axis/centre/foreign). GATE (b) SIM vs `GeomAPI_ProjectPointOnSurf` — foot coords +
  distance to machine precision (0). HONEST DECLINE: cone / torus / freeform, and ambiguous poses (point on
  a cylinder axis / at a sphere centre) → OCCT. ~0.5–1 py.
- *Oracle:* `BRepFeat` / `BRepAlgoAPI` / `ShapeUpgrade` on the re-solved solid (volume/watertight/
  topology); DM1 also has a closed-form partition oracle (the two pieces sum to the whole).
- *Bounded* (well-understood synchronous-modeling engineering), not asymptotic. **Gates a
  fully-OCCT-free *app*, though not the kernel's geometry primitives.**

### M-GS — Kernel geometry services for drafting & analysis · ~2.5–5 py · needs tessellate + M2
**Added from the app audit: the CyberCad app's 2D-drawing and measurement features depend on OCCT
geometry *services* the native kernel does not provide** (distinct from solid-modeling primitives).
Surfaced by the app's `DrawingProjector` / `ManufacturingDrawing` / `Measure` / `SectionGeometry`
tests. Substages:
- **GS1 — HLR (hidden-line removal) + drawing projection** — project a B-rep's edges onto a drawing
  plane and classify visible vs hidden segments against the occluding faces (OCCT `HLRBRep_Algo`).
  The 2D-drawings feature (`DrawingProjector`, `ProjectEdges`, `ProjectBody`) cannot go OCCT-free
  without it. Consumes the native tessellator + topology; the algorithm itself is net-new. ~1–2 py.
  - *Status (POLYHEDRAL slice — TWO-GATE COMPLETE, change `moat-hlr-hidden-line-removal`):*
    OCCT-free header-only orthographic-HLR core at `src/native/drafting/` (`orthographic_hlr.h`):
    orthographic edge projection → Möller–Trumbore occlusion against the M0 triangle mesh →
    midpoint-interior visibility classification → bisection edge-splitting → disjoint
    visible/hidden 2D sets. Wired end to end behind an additive `cc_hlr_project` accessor
    (facade → `IEngine::hlr_project` → `NativeEngine::hlr_project` over the M0 occluder + the
    deduplicated topology edges; `OcctEngine::hlr_project` = the `HLRBRep_Algo` oracle in
    `src/engine/occt/occt_drafting.cpp`, the only HLR-linked piece).
    **Gate (a) HOST ANALYTIC: PASS** — box from an isometric corner → exactly 9 visible + 3 hidden
    segments (hidden triple meeting at the occluded far corner), plus empty-occluder and
    edge-split invariants; wired into the always-on native suite.
    **Gate (b) SIM native-vs-OCCT `HLRBRep_Algo` parity: PASS** — `tests/sim/native_hlr_parity.mm`
    (via `scripts/run-sim-native-hlr.sh`) drives `cc_hlr_project` under both engines for box (iso +
    oblique), triangle prism, and non-convex L-prism and matches visible/hidden COUNT, total
    projected LENGTH, and endpoint PARTITION to machine epsilon (13/13 PASS; length rel ≤ 2.2e-16,
    partition tol 1e-5).
  - *Status (CURVED slice — TWO-GATE COMPLETE, changes `moat-hlrc-curved-silhouette` +
    `moat-gs1c-curved-hlr`):* the closed-form `n·viewDir=0` silhouette is traced in
    `src/native/drafting/silhouette.h` (OCCT-free, header-only) and fed through the SAME
    occlusion+split path for **cylinder** (two axial generators), **sphere** (great circle),
    **cone / cone-frustum** (two straight contour rulings), and **torus** (`Kind::Torus`; two
    closed turning-point contours). Gate (a) host analytic asserts each silhouette's on-surface
    residual and the `n·viewDir=0` tangency to machine ε (contour point counts, straight-ruling
    vs closed-loop shape), plus the honest declines (axis-parallel cylinder/cone, cone end-on,
    torus axis-view). Gate (b) `native_hlr_parity.mm` compares native vs `HLRBRep_Algo` for
    cylinder/sphere/cone/frustum solids (count + length band + visible⊆oracle-visible
    classification). **Sharpened decline:** a native `cc_solid_revolve` builds a torus as
    rational-B-spline surface-of-revolution BANDS (`Kind::BSpline`), NOT a `Kind::Torus` face, so a
    revolve-built torus DECLINES via the freeform path (a `Kind::Torus` STEP-imported face is
    traced). Freeform (B-spline/Bézier) faces remain declined outright; no wrong visible/hidden
    classification is emitted for any accepted case.
- **GS2 — Section curves** — extract the section *curves* (not just the solid) where a plane cuts a
  solid, incl. capped section geometry (OCCT `BRepAlgoAPI_Section`). Largely reachable via the landed
  M2 half-space / SSI seam machinery. ~0.5–1 py.
  **✅ NATIVE (landed).** Header-only, OCCT-free `src/native/section/section.h` behind the additive
  `cc_section_plane` facade: plane∩{plane,cylinder,cone,sphere} closed-form conics assembled into
  closed section loops (+ capped area). **Oblique-cylinder relaxation (this pass):** now that the
  landed ssi `intersectPlaneCylinder` returns the correct oblique ellipse (semi-major `R/|cosθ|`,
  regression `test_native_ssi::plane_cylinder`), GS2's conservative oblique-cylinder DECLINE guard is
  removed and the oblique cut COMPUTES the section ellipse (`a=R/|cosθ|`, `b=R`, area `π·R²/|cosθ|`),
  assembled into a closed `Ellipse` loop like the other conic cases. **Verified both gates:** GATE A
  host closed-form (`test_native_section` 11/0, incl. `cylinder_oblique_section_is_ellipse`;
  `test_native_ssi` 11/0) and GATE B sim native-vs-OCCT `BRepAlgoAPI_Section` (`native_section_parity`
  all-pass, oblique case edge-length 22.921187 = OCCT, capped area 39.985946 = OCCT, 1 closed loop).
  **Honest declines preserved (not faked):** plane tangent to a curved face, a non-closing section,
  an arc-trimmed curved-face conic, and freeform/torus faces still return a typed decline; the final
  closed-loop self-verify still guards against emitting any wrong/open section.
- **GS3 — Exact measurement / distance queries** — minimum entity-to-entity distance (point/edge/
  face pairs) + angle, analytically/NURBS-exact rather than mesh-approximate (OCCT
  `BRepExtrema_DistShapeShape`). Consumes the native NURBS eval. ~0.5–1.5 py.
  **✅ NATIVE (landed).** Header-only, OCCT-free `src/native/analysis/{distance,angle}.h` behind the
  additive `cc_measure_distance` / `cc_measure_angle` facade. Closed-form analytic·analytic cells
  (point/line/segment/circle/plane/cylinder/sphere, line·line parallel+skew) + deterministic
  seed-and-refine (degree×span grid → Newton polish) for simple-NURBS pairs; angle for
  line·line / plane·plane / line·plane. **Verified both gates:** GATE A host closed-form
  (`test_native_analysis` 47/0, `test_native_analysis_facade` 5/0) and GATE B sim native-vs-OCCT
  `BRepExtrema_DistShapeShape` (`native_analysis_parity` 21/0, deltas ≤ 1.1e-16).
  **Honest declines (expected, not faked):** genuinely-trimmed freeform patches whose global optimum
  is not certifiable (multiple comparably-deep basins / non-converging constrained-boundary restart)
  return a clean decline via `cc_last_error`, never a guessed minimum.
- **GS4 — Curvature analysis** — Gaussian/mean/principal surface + edge curvature for analysis
  (zebra, draft). Consumes native NURBS derivatives. ~0.3–0.8 py.
  **✅ NATIVE (landed).** Header-only, OCCT-free `src/native/analysis/curvature.h` behind the additive
  `cc_surface_curvature` (→ `[K,H,k1,k2]`) / `cc_edge_curvature` (→ κ) facade. Analytic arm
  (plane 0; sphere `1/R²`,`1/R`; cylinder `0`,`1/(2R)`; cone; torus `cos v/(r(R+r cos v))`) + NURBS
  arm via first/second fundamental forms from native surface derivatives (maxDeriv=2), with the
  outward-normal sign convention (curvature flipped for a `Reversed` face) and `k1≥k2` ordering.
  **Verified both gates:** GATE A host closed-form and GATE B sim vs OCCT
  `GeomLProp_SLProps`/`GeomLProp_CLProps`. **Honest declines:** parametric singularity
  (`EG−F² ≤ ε·max(E,G)²`), cone apex, and degenerate edge tangent (`‖C′‖≤ε`) return a clean decline.
- **GS5 — Inertia / principal moments** — a mesh-based inertia tensor (signed-tetra second moments
  over the M0 triangulation about the centroid, then symmetric-3×3 cyclic-Jacobi eigen).
  **✅ NATIVE (landed).** Header-only, OCCT-free `src/native/analysis/inertia.h::principalInertia`;
  `NativeEngine::principal_moments` rewired from the `CC_NATIVE_BODY_UNSUPPORTED → OCCT` stub to the
  native path, **guarded by a watertight precondition** (open / non-watertight body → honest decline →
  OCCT, never a wrong tensor). **Verified both gates:** GATE A host closed-form — box `(V/12){b²+c²…}`
  exact (relmax 1.2e-15), cylinder ≤3e-3, sphere `2/5 V r²` O(1/n²)→1.8e-3; GATE B sim vs OCCT
  `GProp_PrincipalProps` (box relmax 1.24e-15, cylinder 2.0e-4, sphere 1.8e-3, principal axes matched).
  **Honest decline:** open-body inertia is non-certifiable (returns `std::nullopt` / declines to OCCT).
  Re-ran the M6 mass-properties differential fuzzer: 0 DISAGREED, zero silent wrong masses.
- **GS6 — B-rep validity checking** — a native `cc_check_solid` (closed 2-manifold, consistent outward
  orientation, no self-intersection, no degenerate/zero-area face or zero-length edge, finite coords),
  reusing the tessellator + topology + GS3 distance for the self-intersection test.
  **✅ NATIVE (landed).** Header-only, OCCT-free `src/native/analysis/validity.h::checkSolidMesh` +
  `ValidityReport`, behind the additive `int cc_check_solid(body, CCValidityReport*)` /
  `CCValidityReport` / `CCValidityCheck` (ABI additive-only; `cc_principal_moments` unchanged) with
  `NativeEngine::check_solid`, an `IEngine` default, and `OcctEngine::check_solid` (BRepCheck oracle).
  **Verified both gates:** GATE A host fixtures of KNOWN state — valid box/cyl/sphere → valid;
  non-closed shell → `closed=0`; flipped face → `oriented=0`; zero-area/zero-length → `nondegenerate=0`;
  interpenetrating polyhedron → `noSelfIntersection=0`; each `first_failure` names the specific
  invalidity; GATE B sim vs OCCT `BRepCheck_Analyzer::IsValid` on the SAME valid AND deliberately-broken
  fixtures — matched on every one. **Honest decline:** the self-intersection check returns UNDECIDABLE
  (report `decided=0`, never a false `valid`) where no-self-intersection is not robustly certifiable
  (e.g. a general freeform patch). Gates trustworthy import (with M5) and any healing UX.
- *Oracle:* `HLRBRep_Algo` / `BRepAlgoAPI_Section` / `BRepExtrema_DistShapeShape` / `BRepLProp` on
  visible-segment sets / section-curve length+topology / min-distance / curvature values.
- *Bounded.* GS2/GS3/GS4 reuse landed native machinery; **GS1 (HLR) is the substantial one** and the
  hard blocker for OCCT-free 2D drawings. **Gates a fully-OCCT-free *app* with drawings, not the
  kernel's solid-modeling primitives.**

### M-TX — Native affine transforms for native bodies · ✅ LANDED
The rigid/affine transform layer behind the app's translate / rotate / mirror / scale / place
tools — `cc_translate_shape`, `cc_rotate_shape_about`, `cc_mirror_shape`, `cc_scale_shape`,
`cc_scale_shape_about`, `cc_place_on_frame` — used to hard-error (`CC_NATIVE_BODY_UNSUPPORTED`)
on a NATIVE body; the legacy mesh extrude `cc_extrude` forwarded unconditionally to OCCT.
- **✅ NATIVE (landed).** These six ops now apply a `math::Transform` to a native body via the
  proven `topology::Shape::located(math::Transform)` + `SolidMesher` placement path (the same
  path `native_transform_fuzz.mm` certifies vs OCCT `BRepBuilderAPI_Transform` + closed-form).
  A MIRROR flips handedness (signed-vol sign) while staying a valid watertight positive-|vol|
  solid; a zero/degenerate scale (and zero axis / normal / degenerate frame) declines honestly;
  a NON-native body forwards to OCCT unchanged. `cc_extrude` now attempts the native prism first.
- *Two-gate proven.* Gate (a) host-analytic ABI invariants (`tests/test_native_engine.cpp`, 7 new
  cases, OCCT-free); gate (b) SIM native-vs-OCCT ABI parity driving `cc_*` under BOTH engines
  (`tests/sim/native_transform_parity.mm`, box + cylinder, 88/88) plus the pre-existing
  transform-chain fuzzer. `src/native/**` untouched (OCCT-free); `cc_*` ABI unchanged.
- *Bounded, cheap.* The largest app-facing OCCT fall-through by call-site count (27 transform +
  10 extrude sites) closed with no new geometry algorithms — pure placement composition.

### M8 — `drop-occt` — unlink OCCT · gated on M0–M7 + **M-DM** + **M-GS** + M6 bar
> **Itemized unlink checklist:** [DROP-OCCT-READINESS.md](DROP-OCCT-READINESS.md) — every OCCT fall-through site classified A (now-native) / B (must-go-native, ~6.5–15 py) / C (IGES decline), with the concrete unlink sequence and readiness verdict.

Delete `src/engine/occt`, drop the OCCT link, remove/stub `cc_iges_*`. **Only** once every
stage above (including **M-DM** for direct modeling and **M-GS** for the app's drafting/measurement
geometry services) is native at the acceptance bar AND the M6 completeness bar holds (differential
fuzzing shows zero silent wrong results — every non-native input honestly declines with a clear error
rather than a fabricated shape). This is the terminal step; it does not begin until the fallback is
provably unnecessary for the supported domain.
**IGES note (app-relevant):** `cc_iges_import/export` is **descoped** from native but the app *uses*
it — so a fully-OCCT-free *app* additionally requires an IGES decision: drop IGES from the app, keep a
thin OCCT-linked IGES shim (app not 100 % OCCT-free), or reimplement IGES natively (~1.5–3 py, out of
current scope).

## Sequencing

```
M0 freeform mesher/trimmer (KEYSTONE) ──┬──► M2 freeform booleans ──► M3 freeform blends/wrap
                                        │            ▲
M1 SSI S4 general robustness ───────────┴────────────┘
                                        └──► M4 general STEP/AP242 import ──► (needs M5 quality)
M5 shape-healing robustness ────────────────────────► gates M4 + M8
M6 completeness bar (S4-f + fuzzing) ───────────────► gates M8   [asymptotic]
M7 Tier-4 construction (guided sweep / hard loft / fine-pitch) ──► (fine-pitch needs M2)
                                                                        │
                          ALL of M0–M7 native at the bar + M6 holds ──► M8 drop-occt
```

## Parallelism — what can be researched concurrently

These are multi-py research arcs, so "parallel" means **separate efforts / tracks**, not the
short-lived workflow-parallelism used for bounded slices. Two things make a stage
parallelizable: (a) its **upstream dependency** is met, and (b) it touches a **disjoint
`src/native` module** (so tracks integrate cleanly — the same isolation the parallel bounded
slices used). Both are captured below.

> **Wave-1 first slices LANDED (all verified native-vs-OCCT).** The four launched Wave-1 tracks
> each banked a first verified slice: **M0** (keystone) trimmed free-form interior mesher,
> **M1** freeform S4-d open-arm branch point (marching 12/12), **M5** opt-in gap bridging
> (heal 6/6), **M6** curved-boolean differential fuzzer (512 trials, 0 DISAGREED). **M7a**
> (construct sweep/loft) has now banked its first verified slice too: the N-section planar
> ruled loft wired to `cc_solid_loft_sections` (Gate 1 21/21, Gate 2 39/39). **This opens Wave
> 2**: with M0's mesher ready and M0+M1 both past a first slice, **M2 (freeform booleans) and
> M4 (general import) can now start in parallel** — alongside the still-running asymptotic
> tracks (M1 breadth, M5, M6) and M7a's remaining residuals (guided sweep, non-planar-cap loft).

> **Wave-2 + M2-substrate batch LANDED (all verified native-vs-OCCT, integrated).** Seven further
> slices banked: **M4** foreign trimmed `B_SPLINE_SURFACE_WITH_KNOTS` admission (STEP parity 79/79 —
> completes the M0 keystone's deferred half) + **M4-rational** `RATIONAL_B_SPLINE_SURFACE` admission
> (83/83); **M7a** N-section loft (39/39, guided-orient sweep honest-declined with a measured self-
> verify-trap); **M5-tail** opt-in planar-hole cap (heal 8/8); **M6-breadth** STEP round-trip
> differential fuzzer (0 DISAGREED, *surfaced* OCCT reader inaccuracy on shallow frustums, native
> vindicated by closed-form). **M2 substrate:** **B2** freeform face-split (12/12) + **B3** point-in-
> freeform-solid membership (crispDISAGREE=0) — the two hard subsystems the M2 decline named. The
> single freeform-boolean **M2** was an honest DECLINE that mapped its blockers; two of its three
> substrate pieces (B2, B3) are now native, leaving only **B1** (the small operand-descriptor join
> point) before the M2 assembly (recognise[B1] → trace[M1] → split[B2] → classify[B3] → weld[M0]).

| Stage | Module (disjoint unit) | Needs | Status / when it can run |
|---|---|---|---|
| **M0** freeform mesher/trimmer | `tessellate/` | — | ✅ **Wave-1 slice LANDED** — mesher ready; unblocks M2/M4. ✅ **Weld robustness LANDED (deflection-fragility RESOLVED):** shared-curved-edge single-sampling (`edge_mesher.h`+`face_mesher.h`, additive, OCCT-free) welds the freeform boolean seam + bowl-lid quad edges watertight at ANY deflection — the freeform CUT/COMMON no longer oscillate watertight↔decline across `{0.03…0.002}`. Zero regression PROVEN byte-identical (FNV of verts+tris+wt+area+vol) for every existing surface kind; host `41/41`, sim parity `20/20` at d=0.01 AND 0.004 |
| **M1** SSI S4 general robustness | `ssi/marching` | — | ✅ **Wave-1 slice LANDED** — breadth continues (asymptotic) |
| **M5** shape-healing robustness | `heal/` | — | ✅ **Wave-1 slice LANDED** — tail continues (asymptotic) |
| **M6** completeness / fuzzing harness | test infra + `ssi/` | — | ✅ **Wave-1 + breadth×8 LANDED** — curved-boolean + STEP round-trip + construction loft/sweep + blend fillet/chamfer + wrap-emboss + mass-properties + **geometry-services (GS3 distance / GS4 curvature / GS2 section incl. OBLIQUE / GS5 inertia / GS6 validity / GS1 HLR vs OCCT, incl. tilted regimes; 0 DISAGREED across 2 seeds, 480 trials)** + **transform-chains (three-way native/OCCT/closed-form similarity; translate/rotate/uscale/mirror, N=160×2 seeds, 0 DISAGREED, mirror handedness-flip confirmed; found + gated an OCCT zero-scale hang)** fuzzers (0 DISAGREED, **8 native domains**); concave-shaft blends + healing remain (gates M8) |
| **M7a** guided sweep · hard loft | `construct/` | — | ✅ **Wave-1 slice LANDED** — N-section loft ABI (`cc_solid_loft_sections`); guided sweep (measured trap) + non-planar-cap loft remain OCCT |
| **M4** general STEP/AP242 import | `exchange/` | M0 | ✅ **Wave-2 LANDED** — non-rational + `RATIONAL_B_SPLINE_SURFACE` + `RATIONAL_B_SPLINE_CURVE` (edge/trim) admission native (parity 90/90). ✅ **M4-tail `MAPPED_ITEM` / `REPRESENTATION_MAP` (Form-B) assembly INSTANCING LANDED** (`moat-m4t-assembly-import`): a `REPRESENTATION_MAP` over a shared brep instanced by N `MAPPED_ITEM`s (AXIS2 or CARTESIAN_TRANSFORMATION_OPERATOR_3D target), reusing the Form-A `classifyPlacement`/`Shape::located` substrate → placed Compound; the shared brep mapped ONCE, re-instanced through the shared node. GATE (a) HOST-analytic (`test_native_step_reader.cpp` +5, 67/67): N shared-box instances at known translations/rotation match closed-form vol/bbox; ≠1-brep / lone-REP_MAP / mixed-Form-A+B DECLINE. GATE (b) SIM vs `STEPControl_Reader` (`native_step_mapped_item_parity.mm`, booted sim, 5/5): 3 instances, vol/area/centroid rel ~1e-16, bbox Δ=0, faces 18=18; no-brep mapped rep declines→OCCT. Structural: `git diff src/native` OCCT-free & additive, writer + `mapManifoldBrep` byte-frozen, 0 `cc_*` change. **PMI SEMANTICS remain OCCT** (census-only `pmi_scan`; no GD&T semantic model) |
| **M2b (B2)** freeform face-split | `boolean/` · `ssi/` | M0 ✅ + M1 ✅ | ✅ **Wave-2 slice LANDED** — `boolean/face_split.h` `splitFace` (tiles vs OCCT 12/12); non-convex/multi-crossing tail declines. **SMOOTH-TRIM ✅ LANDED** — `boolean/smooth_trim_split.h` `splitFaceSmoothTrim` (additive sibling; closed/circular interior seam → disk + annulus-hole; host gate 7/7, tiling ε, closed-form `π·ρ²`); B2 convex path byte-frozen |
| **M2c (B3)** freeform point-in-solid | `boolean/` | M0 ✅ | ✅ **Wave-2 slice LANDED** — `boolean/freeform_membership.h` `classifyPointInMesh` (crispDISAGREE=0 vs `BRepClass3d`); near-tangent band → On/Unknown |
| **M2a (B1)** freeform operand descriptor | `boolean/` | `shape.h` | ✅ **Wave-2 slice LANDED** — `boolean/freeform_operand.h` `recogniseFreeformSolid` (host gate 14/14; admit + round-trip + 8-way decline battery); **completes the M2 substrate**. End-to-end assembly honest-declined (B2 needs smooth-trim split + a B4 analytic-face-split/cap-synth verb) |
| **M2** general freeform booleans (assembly) | `boolean/` | **B1 ✅ + B2 ✅ + B3 ✅ + B4 ✅** | ✅ **FIRST freeform boolean LANDED — BOTH gates (HOST-analytic (a) + SIM native-vs-OCCT (b)), native-vs-OCCT verified** — `boolean/half_space_cut.h` (B4 analytic-face-split + cross-section-cap-synthesis weld verb) composes recognise[B1] → trace[M1] → split[B2] → B4 → classify[B3] → weld[M0] → mandatory watertight self-verify into the first end-to-end freeform↔analytic half-space **CUT** (bowl-lidded convex-quad prism). GATE (a) HOST ANALYTIC GREEN (OCCT-free): watertight + enclosed volume = closed-form `∫∫_{Q∩{x≤0}}(H0+a(x²+y²))dA` to 0.7% (deflection band), oracle unit-checked vs `H0/2+a/6`; host suite `40/40`. GATE (b) SIM PARITY GREEN (`tests/sim/native_first_freeform_boolean_parity.mm`, booted iOS-17 sim): operand reconstructed in OCCT (sewn 6-face `Geom_BezierSurface`-topped solid), cut by `BRepAlgoAPI_Cut`, compared vs native `freeformHalfSpaceCut` on **volume (rel 7.13e-03 ≤ 2e-2), area (rel 4.43e-04), watertightness (closed 2-manifold), topology (Euler χ=2, single closed solid), bbox (worst 1.00e-07) + one-sided Hausdorff (1.57e-07 ≤ 1.5e-2)** — **12/12 PASS**, fixed curved-tessellation tolerances never widened, no native-vs-OCCT discrepancy surfaced (the CUT is correct vs OCCT). Harness has own `main()` + `run-sim-suite.sh` SKIP entry. Blocker (i) SIDESTEPPED by the bowl-lidded-prism operand (its one freeform face has the convex straight-edged loop B2 splits); the **B2 smooth-trim (closed/circular wall) generalisation is DEFERRED** as the next enabler. Every stage NULL-on-decline; OCCT stays the fall-through. Deferred sub-gates: explicit in-harness `BRepClass3d` query-point batch (already `crispDISAGREE=0` in M2c) and a perturbed-candidate discard case (host-driver `NotWatertight` path). **COMMON also LANDED (host gate, M2-breadth):** the complementary keep-side `freeformHalfSpaceCut(A,P,KeepSide::Above) = A∩{x≥0}` (no new geometry — `KeepSide::Above` consumed byte-identical), host gate 4/4 — partition-closure oracle `V(x≤0)+V(x≥0)=V(full)` exact, COMMON watertight at its complementary closed-form volume. **DEFLECTION FRAGILITY — ✅ RESOLVED (M0 weld robustness, shared-curved-edge single-sampling):** the CUT/COMMON deflection oscillation is GONE. Root cause (pinned by measurement, refining the original hypothesis): the per-face `d.points` already agreed to machine ε — the actual leaks were (1) a straight-edge canonical anchor placing its FAR endpoint at the interpolated `ce.a + dir·1` (≈1 ULP off the shared vertex `ce.b`, e.g. `−0.5+0.585 ≠ 0.085`), which competes for the shared corner where a curved seam meets the walls, and (2) the freeform boolean seam carried by the cap and the trimmed freeform sub-face as SEPARATE edge nodes whose independent discretizations diverge ~1 ULP; both split when the point lands on a weld-cell boundary. The additive fix (`tessellate/edge_mesher.h` + `face_mesher.h`, OCCT-free): (a) pin the straight-edge anchor ENDPOINTS to the exact `ce.a`/`ce.b` (the shared vertices), and (b) give a genuinely-curved edge reached via a separate node ONE canonical discretization keyed by its endpoint pair + a **distance-matched** 3-D midpoint (boundary-free, unlike a quantized midpoint), consumed BIT-IDENTICALLY by both incident faces. **GATE (a) HOST:** operand + CUT + COMMON now weld watertight at EVERY deflection of `{0.03,0.02,0.01,0.008,0.004,0.002}` at the closed-form volume (`test_native_freeform_boolean_breadth.cpp::weld_robust_across_full_deflection_sweep`). **GATE (b) SIM:** native-vs-OCCT parity now asserted at 0.01 AND 0.004 (a deflection the pre-fix mesher declined) — **20/20 PASS**. **ZERO REGRESSION PROVEN:** every existing surface kind's mesh (Plane/Cylinder/Sphere/Bézier/BSpline, curved seams, box/holed/loft/sweep/thread/step) is BYTE-IDENTICAL (FNV of all vertices + tris + wt + area + vol) — the only mesh that changes is the freeform operand at the two deflections where it was PREVIOUSLY LEAKING, and there it only fuses the un-welded duplicate vertex (verts 18→17, 42→41) and flips wt 0→1; full host suite `41/41`. **FUSE ✅ LANDED — FIRST two-operand freeform boolean, BOTH gates (M2-FUSE):** a freeform bowl-lidded prism `A` FUSED with a FINITE all-planar box `B` for the single-curved-cut pose. Two additive OCCT-free header-only verbs: `boolean/inter_solid_seam.h` (`buildInterSolidSeam` — unique-`Pcut` straddle + containment guard → landed `traceWallSeam`/B2 split/`cutAnalyticFace`/`orderLoop` → ONE closed inter-solid seam loop) + `boolean/two_operand.h` (`freeformBooleanTwoOperand` — FUSE = A-outer keeps ∪ B whole faces ∪ the `Pcut` rectangle-minus-D ANNULUS whose curved hole IS the shared seam; B3 `classifyPointInMesh` CONFIRMS survivor membership; mandatory watertight + union-volume self-verify → NULL→OCCT on any decline). CUT/COMMON reduce (theorem of the containment guard) to the landed `freeformHalfSpaceCut` Below/Above. **GATE (a) HOST ANALYTIC (OCCT-free), 6/6:** seam closes; FUSE watertight at the closed-form union `V(B)+V(A∩{x≤0})` (rel ≤ 2e-2, MONOTONE-converging 0.02/0.01/0.005); CUT/COMMON at their closed forms; non-cutting box + non-freeform operand DECLINE to NULL. **GATE (b) SIM native-vs-OCCT `BRepAlgoAPI_Fuse` (booted sim), 23/23** (`tests/sim/native_two_operand_freeform_boolean_parity.mm`): d=0.01 AND 0.005 — VOLUME rel 9.4e-4/5.7e-4, AREA rel 5.8e-5, WATERTIGHT, Euler χ=2, BBOX 1e-7, HAUSDORFF 7e-10, + a 2197-pt CLASSIFY batch vs `BRepClass3d_SolidClassifier` **ZERO crisp IN↔OUT disagreements** (native via landed B3 multi-ray), fallback declines to NULL. **ZERO-REGRESSION:** `git diff src/native` empty, 0 OCCT includes added, landed CUT (5/5) + COMMON (5/5) host suites unchanged, B1/B2/B3/M0 + `freeformHalfSpaceCut` byte-identical, M1 untouched. NEXT BLOCKER: the GENERAL multi-curved-seam box pose (>1 face slicing the wall) needs the seam-GRAPH assembly (design §10); a non-containment pose needs the two-operand CUT/COMMON survivor weld; B2 smooth-trim; freeform↔freeform. **M2-multiseam SEAM-GRAPH BUILDER ✅ LANDED (host gate, design §9 level 3) — the general multi-seam NEXT BLOCKER now has a proven enabler + a sharpened decline:** two additive OCCT-free header-only verbs `boolean/seam_graph.h` (`buildSeamGraph` — generalises `findCuttingFace` `nCut==1` to a cutting-face SET of EXACTLY two adjacent straddling faces + containment guard; traces BOTH arcs via byte-unchanged `traceWallSeam`; computes the junction `J` ANALYTICALLY as the two arcs' orthogonal iso-parameters and VERIFIES it lies on BOTH cutting planes inside the trimmed wall — `junctionPlaneResidual = 0`; clips + joins the arcs into one bent boundary→J→boundary seam) + `boolean/multi_seam.h` (`freeformBooleanMultiSeam` — builds+proves the graph, then honestly DECLINES the weld → NULL → OCCT). **GATE (a) HOST ANALYTIC (OCCT-free), 6/6** (`test_native_multi_seam.cpp`): graph closes (J on both planes, two arcs joined, EACH arc individually B2-splits the wall); closed-form CORNER oracle identities `V(A∩B)+V(A−B)=V(A)` and `V(A∪B)=V(A)+V(B)−V(A∩B)` to machine precision (`V(A∩B)=0.051275`, `V(A−B)=0.145035`, `V(A∪B)=0.529035`); a single-cut box + a non-freeform operand DECLINE to NULL. **REFUTED (measured) the proposal's "one bent WLine keeps B2 byte-identical" reduction:** `splitFace(jointSeam)` reaches `crossings==2`, `tilingGap≈0` (geometrically exact partition) but declines `RebuildMismatch` — B2's fixed global-density rebuild self-verify cannot resolve the interior valence-3 kink at `J`, and the strict rebuild tolerance MUST NOT be weakened. **THE NAMED NEXT ENABLER:** a junction-AWARE split introducing `J` as an EXACT shared valence-3 vertex (additive; does NOT touch byte-frozen B2), then the per-op corner+L weld + self-verify + the sim `BRepAlgoAPI_*` parity gate. **ZERO-REGRESSION:** `git diff src/native` empty (only NEW `seam_graph.h`/`multi_seam.h`), 0 OCCT includes, B1/B2/B3/M0/M1 + landed single-seam `inter_solid_seam.h`/`two_operand.h` byte-identical, native-booleans + native-ssi host suites unchanged, new verbs `-fsyntax-only` clean under the iossim toolchain. **M2-multiseam JUNCTION-AWARE WALL SPLIT ✅ LANDED (host gate, WAVE 2) — the prior wave's named enabler is now a self-verified exact valence-3 partition:** one additive OCCT-free header-only verb `boolean/junction_split.h` (`splitFaceJunction`) resolves the `RebuildMismatch` the seam-graph wave declined at. Where byte-frozen B2 `splitFace(jointSeam)` corner-cuts the sharp interior kink at `J` (its single degree-1 seam edge reflattened by `buildRegion` at 8 samples/edge shortcuts the bend → ~1e-5·parentArea loss), the junction-aware verb builds the seam as TWO edges (arc0-half E→J + arc1-half J→X) meeting at `J`; because the two arcs are ORTHOGONAL iso-parametric curves (u-const / v-const → straight lines in UV, the ONLY bend at `J`), making `J` an edge endpoint lets each half reflatten to MACHINE PRECISION and PASS the SAME strict rebuild tolerance (`rebuildTolFrac = 1e-6`, NEVER weakened). It consumes B2's `detail::` primitives (`flattenOuter`/`seamCross`/`buildSeamEdge`/`restrictEdge`/`shoelace`) + the SAME `tess::buildRegion` reflatten + the SAME `SplitOptions` — B2 UNCHANGED. **GATE (a) HOST ANALYTIC (OCCT-free), 7/7** (`test_native_multi_seam.cpp`): the wall partitions into corner (`A∩{x≥0,y≥0}`) + L-survivor with `tilingGap`/`rebuildResidual` ~3e-16; the corner sub-face UV area equals the closed-form `Q∩{u≥½,v≥½}` projection to 7e-17 (`msx::uvCornerArea()`); `J` bit-identical in both sub-loops; byte-frozen B2 still declines `RebuildMismatch` on the same seam (the contrast asserted). `freeformBooleanMultiSeam` now composes recognise[B1] → `buildSeamGraph` → arc-B2-consistency → `splitFaceJunction` (LANDED) → honest decline → OCCT. **SHARPENED NEXT BLOCKER (MEASURED):** the corner box straddles `A`'s footprint quad `Q`, so the `x=0`/`y=0` planes ALSO corner-clip `A`'s flat BOTTOM quad + the TWO side walls over the `Q` edges they cross (`msx::footprintStraddlesBothPlanes()` = true) — the full result needs those faces split, two box CAP faces synthesized inside `A`, and a MULTI-FACE shell welded across multiple junctions (`J` on the wall, `J'` on the bottom, wall/plane pierce points) + self-verified; the sim `BRepAlgoAPI_*` parity lands WITH that multi-face corner-clip weld, then the general `≥3`-seam / branch-point graph. `freeformBooleanMultiSeam` returns NULL → OCCT with `MultiFaceWeldUnreachable` (never a wrong/leaky/partial solid). **ZERO-REGRESSION (WAVE 2):** only NEW `junction_split.h` + additive edits to (untracked) `multi_seam.h`; byte-frozen B2 `face_split.h` + single-seam `inter_solid_seam.h`/`two_operand.h`/`seam_graph.h`/M0/M1 UNCHANGED; 0 OCCT includes; host suites `test_native_face_split` 5/5, `first_freeform_boolean` 5/5, `freeform_boolean_breadth` 5/5, `two_operand` 6/6 GREEN; new verb `-fsyntax-only` clean under the iossim toolchain. **M2-multiseam MULTI-FACE CORNER-CLIP WELD ✅ LANDED — BOTH gates, all THREE ops native-vs-OCCT verified (WAVE 3):** the seam-graph + junction-split enablers now compose into a WATERTIGHT result solid via one additive OCCT-free header `boolean/multi_face_weld.h` (`multiFaceCornerClip`), wired into `freeformBooleanMultiSeam`. The corner box straddles A's footprint quad, so `multiFaceCornerClip` clips A's flat BOTTOM quad (an L-survivor rerouted through the bottom junction `J'`, or a convex two-plane corner) + the TWO side walls (byte-frozen `hscdetail::cutAnalyticFace`), and synthesizes the two box CAP faces inside A (CUT/COMMON — sharing the bowl seam arc with the wall sub-face + the vertical `J→J'` edge between caps) or NOTCHES B's two cutting faces (FUSE — rectangle-minus-notch, curved boundary = the shared arc); B's four non-cutting faces weld WHOLE. Mandatory self-verify: M0 watertight + a consistent op-volume bound → NULL→OCCT on any decline (never a leaky/partial/wrong solid; NO tolerance weakened). **GATE (a) HOST ANALYTIC (OCCT-free), `test_native_multi_seam.cpp` 8/8:** CUT/COMMON/FUSE each weld watertight at the closed-form corner oracle (`V(A−B)=0.145035`, `V(A∩B)=0.051275`, `V(A∪B)=0.529035`) — rel 9.4e-3/4.0e-3/2.4e-3 at d=0.01, MONOTONE-converging to ≤0.5% (CUT 4.6e-3 at d=0.005); Euler χ=2; the single-cut-box + non-freeform declines to NULL preserved. **GATE (b) SIM native-vs-OCCT `BRepAlgoAPI_Cut/Common/Fuse` (booted iOS-17 sim), `native_multi_seam_freeform_boolean_parity.mm` 59/59:** at d=0.01 AND 0.005 — VOLUME rel CUT 9.4e-3/4.6e-3, COMMON 4.0e-3/3.1e-3, FUSE 2.4e-3/1.3e-3 (each also vs the closed form to ~1e-11); AREA rel ≤6e-4; WATERTIGHT + Euler χ=2 all three; BBOX 1.0e-7; HAUSDORFF ≤1.6e-7; a 2197-pt CLASSIFY batch per op vs `BRepClass3d_SolidClassifier` **ZERO crisp IN↔OUT disagreements**; single-cut-box fallback declines `SeamGraphDeclined`. **ZERO-REGRESSION (WAVE 3):** only NEW `multi_face_weld.h` + additive edits to `multi_seam.h` (report fields + weld call) & `test_native_multi_seam.cpp`; byte-frozen B2 `face_split.h` + `inter_solid_seam.h`/`two_operand.h`/`seam_graph.h`/`junction_split.h`/M0/M1 UNCHANGED; 0 OCCT includes in `src/native`; 0 `cc_*` change; full host `ctest` **52/52** GREEN. NEXT BLOCKER: the general `≥3`-seam / branch-point seam graph, non-planar bottom/side clips, and freeform↔freeform **M2b B2 SMOOTH-TRIM ✅ LANDED (host gate) — the deferred "closed/circular wall" enabler is now a self-verified closed-seam partition:** one additive OCCT-free header-only verb `boolean/smooth_trim_split.h` (`splitFaceSmoothTrim`) resolves the case byte-frozen B2 `splitFace` DECLINES (`CrossingsNot2`, `crossings==0`): a CLOSED SMOOTH seam INTERIOR to a trimmed freeform/analytic face (a horizontal plane slicing a bowl/dome cap → a circle; a ring on a curved wall). It partitions the face into `faceInside` (the disk the seam encloses, outer wire = the seam loop) + `faceOutside` (the parent's outer wire REUSED verbatim + the seam loop as a single HOLE wire); the M0 mesher's existing inside-outer-AND-outside-holes keep rule (`trim.h` UVRegion) meshes BOTH with NO tessellator change. The seam is built as one short STRAIGHT edge per traced polyline segment (the FAITHFUL representation — a curvature-driven edge discretizer UNDER-samples a single degree-1 B-spline standing in for the whole curved arc, the MEASURED failure that halved the meshed disk area 0.126→0.065), each edge built ONCE and laid on both sub-faces with OPPOSITE orientation ⇒ bit-exact shared boundary, watertight weld. Consumes B2's `detail::` primitives (`flattenOuter`/`seamCross`/`buildSeamEdge`/`shoelace`/`segmentsCross`) + the SAME `tess::buildRegion` reflatten + the SAME `SplitOptions` tolerances — B2 UNCHANGED. **GATE (a) HOST ANALYTIC (OCCT-free), 7/7** (`test_native_smooth_trim_split.cpp`): a quad-trimmed Bézier bowl sliced by the horizontal plane `z=c` gives a CLOSED CIRCLE (real S3 trace, 241 nodes, radius ρ=0.20 interior to the quad); the split tiles to machine ε (`tilingGap`/`rebuildResidual` ~3e-16), the disk area equals the closed-form `π·ρ²=0.125664` to rel 1.3e-4, and the two sub-faces mesh watertight with areas CONVERGING MONOTONICALLY to the true curved area (rel 6.2e-3→4.2e-3→1.07e-3 at d={0.02,0.01,0.005}); byte-frozen B2 `splitFace` still DECLINES the same seam (`crossings==0`, contrast asserted); honest-decline battery (open chord `SeamNotInterior`, self-intersecting `SelfIntersecting`, too-short `SeamTooShort`). **GATE (b) SIM — HONEST SCOPE:** a closed interior trim has no clean `BRepAlgoAPI` face-split oracle (needs a splitter/section-wire reconstruction, not a solid boolean); the closed-form partition grounds Gate A and the sim `BRepAlgoAPI_Cut` parity lands WITH the downstream weld verb that consumes the smooth-trim split. **SHARPENED NEXT BLOCKER:** the curved-wall freeform half-space CUT that CONSUMES `splitFaceSmoothTrim` (dome cut by a horizontal plane: split the freeform cap by the closed seam, split the analytic walls, synthesize the flat circular cross-section cap, weld + watertight self-verify) + its sim parity gate. **ZERO-REGRESSION:** `git diff src/native` = only NEW `smooth_trim_split.h` (0 existing files touched), 0 OCCT includes, no `cc_*` change; byte-frozen B2 `face_split.h` + `junction_split.h` + single-seam/multi-seam paths + M0 tessellator + M1 UNCHANGED; full host `ctest` 53/53 GREEN (incl. `test_native_face_split` 5/5); new verb `-fsyntax-only` clean under the iossim toolchain; verb cognitive complexity 14 (within the backend band) |
| **M3** freeform blends + wrap-emboss | `blend/` · `feature/` | M2 | **Wave 3** — after M2 |
| **M7b** fine-pitch self-intersecting thread | `construct/` | M2 | **Wave 3** — after M2 |
| **M8** `drop-occt` — unlink | `engine/occt` (delete) | ALL + M6 bar | **Terminal** |

```
WAVE 1 — first slices ✅ LANDED (M0·M1·M5·M6·M7a)
  M0 tessellate ✅  │  M1 ssi/marching ✅  │  M5 heal ✅  │  M6 fuzz-harness ✅  │  M7a construct ✅ (loft-sections)
        │  └──────────────┐                        (asymptotic tracks continue)        │
        ├──► WAVE 2 ▶ OPEN NOW                                                          │ (M7a independent)
        │    M4 exchange (needs M0 ✅)                                                  │
        └──► M2 boolean (needs M0 ✅ + M1 ✅) ◄────────────────────────────────┘
                  │
                  └──► WAVE 3:  M3 blend/feature   +   M7b construct(fine-pitch)   (both need M2)

  ALL of M0–M7 native at the bar  +  M6 completeness bar holds  ──►  M8 drop-occt
```

**Reading it (current front):** Wave 1's four launched tracks each banked a first verified slice,
so the concurrency front has advanced. **Runnable in parallel RIGHT NOW:** ▶ **M2** (freeform
booleans — `boolean/`) and ▶ **M4** (general import — `exchange/`), both unblocked by the M0
mesher; plus **M7a** (construct sweep/loft — independent, still unstarted); plus the continuing
asymptotic tracks **M1** breadth, **M5**, and **M6** — six disjoint modules, no shared code. The
freeform payoff chain remains the critical path: **M0 + M1 → M2 → M3** (booleans gate blends).
M5 (healing) and M6 (fuzzing) run the whole time and *gate the finish* (M4 quality, M8) rather
than the middle. M8 is terminal — only when every track is native at the bar and the M6 bar holds.

**Critical path (longest serial chain):** `M0 → M1↘ M2 → M3` (≈ 6–13 py end-to-end) — its first
link (M0 ↘ M1) is now past a first slice, so the live critical work is **M2 → M3**. Minimum
wall-clock to drop-OCCT is set by that chain + the asymptotic M5/M6 tails, not by total py,
because M4/M5/M6/M7 overlap it. Staffing M2 + M4 + M7a concurrently now is what compresses the
calendar next.

## Effort rollup (honest)

| | Person-years |
|---|---|
| **Delivered + verified vs OCCT (this project)** | ≈ **3.5–4.5 py** — planar/analytic breadth, SSI S1–S5 + S4-a…e, five curved-boolean families 3/3, curved fillet/chamfer (const/variable/asym), STEP export + broad import (all quadric+torus+general revolution, trimmed, assemblies, AP242-skip), shape-healing + STEP-import first slices, mismatched loft, deboss/polygon wrap-emboss |
| **Moat slices landed (this campaign)** | **M0** keystone mesher **+ M4/M4-rational/M4-tail-2/M4-tail-4** foreign B-spline STEP admission — surfaces + rational + curves + rectangular-trims + N-level nested assemblies + PMI census (**M4 import complete**) · **M0-weld** shared-curved-edge canonical placement (**freeform boolean now deflection-robust**) · **M1** freeform S4-d open-arm branch · **M2** substrate B1/B2/B3 + B4 -> **first freeform<->analytic boolean CUT+COMMON at BOTH gates** (sim vs BRepAlgoAPI_Cut, Hausdorff 1.6e-7) · **M5** gap-bridge + planar-cap · **M6** **SEVEN** fuzz domains (curved-boolean, STEP round-trip, construction, blend, wrap-emboss, mass-properties, **geometry-services** — GS3 distance / GS4 curvature / GS2 section incl. OBLIQUE / GS5 inertia / GS6 validity / GS1 HLR vs OCCT), all 0 DISAGREED · **M7a** N-section loft **+ M7-tail** guided-orient sweep (overturned M7a's decline) · **M-DM DM1** cc_split_plane · **M-GS GS3+GS4** measurement + curvature (both gates) **+ GS1** polyhedral HLR (both gates, cc_hlr_project) **+ GS5+GS6** native inertia tensor / principal moments (vs GProp_PrincipalProps) + B-rep validity checker cc_check_solid (vs BRepCheck_Analyzer::IsValid on valid + deliberately-broken fixtures, both gates) — each verified native-vs-OCCT, additive, src/native OCCT-free. Honest declines that sharpened the map: M2 first-attempt, guided-orient sweep (later overturned), M1-tail self-crossing (analytic proof), shared-sub-assembly, HLR curved silhouettes · **M2-FUSE first TWO-OPERAND freeform boolean (FUSE/CUT/COMMON, both gates, Hausdorff 7e-10 vs BRepAlgoAPI)** · **M-GS GS2** section curves + **GS5** inertia + **GS6** validity + **GS1-curved** cylinder/sphere HLR silhouettes (all vs OCCT) · fixed a real SSI oblique plane-cylinder ellipse bug (+regression) surfaced by GS2 |
| **Remaining — kernel geometry moat (M2/M3 breadth + tails + M8)** | ≈ **3–8 py** — **M2/M3 breadth** (freeform booleans/blends, bounded *per family*, the parallelizable bulk) + the **M5 + M6** asymptotic gates. M4 import is now essentially complete (B-spline surfaces + curves + rational + trims + nested assemblies + PMI census); M7 residuals small |
| **+ M-DM direct modeling (app-required, newly added)** | ≈ **1.5–3 py** — `cc_split_plane` (near-term, reuses the landed M2 half-space verb) + `cc_replace_face(_to_plane)` + project (push-pull / local B-rep re-solve; needs M2 + M3 + M5). NOT covered by any prior stage |
| **+ M-GS drafting/analysis geometry services (app-required, newly added)** | ≈ **2.5–5 py** — **GS1 HLR** (hidden-line removal for 2D drawings — the substantial one, hard blocker for OCCT-free drawings) + GS2 section curves + GS3 exact measurement + GS4 curvature. GS2/3/4 reuse landed native machinery; NOT covered by any prior stage |
| **➤ App fully OCCT-free — recomputed (this is what "the app uses our library without OCCT" costs)** | **Adopt native kernel (native + kernel's OCCT fallback — NOT OCCT-free):** ≈ **0.25–0.5 py** (swap the app's OCCT `KernelBridge.mm` for the CyberCadKernel lib). **Scoped OCCT-free** (app declares its supported domain, natively covers must-have booleans/blends/direct-modeling/**drawings+measurement** + declines exotic freeform, IGES dropped or thin OCCT shim): ≈ **8–15 py**. **Full-native OCCT-free** (every input OCCT handles today — arbitrary freeform booleans/blends, robust arbitrary-broken healing, native IGES): ≈ **14–27 py + the asymptotic M5/M6 tails** (never fully "closes") |
| **Note — the CAD *app* layer is separate** | The above is the **geometry-kernel** cost. CyberCad's "SolidWorks on iPad" value — sketcher/constraint solver (planegcs), parametric feature history, multi-body/mates/IK, drawing/annotation system, datums, patterns, document/persistence, Metal rendering + Pencil UX — is an **app-layer body of work OCCT never provided**, mostly already built/in-flight, and **orthogonal to drop-occt** |
| IGES (app uses `cc_iges_import/export`) | **DECISION: DROPPED** (product decision). IGES is *not* reimplemented natively; `cc_iges_*` is removed/stubbed at `drop-occt`. The app is STEP-first; IGES interop, if ever needed, is an optional OCCT-linked plugin outside the OCCT-free core. **0 native py; removes IGES from the drop-occt critical items.** |

## The remaining path to `drop-occt` (M8) — projection once the M2 substrate closes

*Projection as of the M2-substrate round (B1 in flight; B2 + B3 landed). Once B1 lands, the
freeform-boolean **assembly** is reachable and the hard **research** is largely behind — what
remains to actually unlink OCCT is **breadth** (parallelizable) plus **two asymptotic gates** that
must HOLD, not new keystones.*

What the campaign has closed: the **keystone** (M0 mesher), its **import surface** (M4 non-rational
+ M4-rational B-spline admission — foreign trimmed B-spline patches now import native end-to-end),
and the **M2 substrate** (B1 recognise · B2 split · B3 classify, over M1 trace + M0 weld). The
freeform-boolean chain's *enablers* are done; the *payoff breadth* is what's ahead.

| Remaining stage | What's left to be native at the bar | Bounded / Asymptotic | Needs | Parallel? |
|---|---|---|---|---|
| **M2 breadth** | freeform booleans across families: NURBS↔NURBS, all 3 ops, **multi-face / holed / multi-branch-seam** splits (first slice reachable once B1 lands; breadth ahead) | bounded *per family*, asymptotic in full generality | substrate (B1+B2+B3) | assembly serial; **families parallelize** |
| **M3** blends + wrap-emboss | curved-curved fillets/chamfers + wrap-emboss on **freeform bases** | bounded per family | **M2** | after M2; families parallel |
| **M4 tail** | ~~rational-*curve* trims~~ **LANDED** (M4-tail-2, combined `RATIONAL_B_SPLINE_CURVE` edge/trim admission, verified vs OCCT, SIM parity 90/90) · AP242 **PMI semantics** · ~~deep-nested rigid/conformal assemblies~~ **LANDED** (Form-A CDSR chain walk, verified vs OCCT) · `MAPPED_ITEM` (Form-B) still deferred | bounded (parser breadth) | M0 ✅ | ✅ **now** (`exchange/`) |
| **M5 tail** | self-intersecting-wire repair · pcurve reconstruction · **arbitrary broken industrial B-rep** | **ASYMPTOTIC** (`ShapeFix` moat) | — | ✅ **now** (`heal/`) |
| **M6 bar** | fuzz the remaining domains (blends, healing) + **HOLD** zero-silent-wrong across the whole surface | **ASYMPTOTIC** — the *gate*, never "done" | tracks it guards | ✅ **now** (test infra) |
| **M7 tail** | ~~guided-sweep orientation law~~ ✅ (M7-tail) · ~~curved-rail morph~~ ✅ + ~~real-twist sweep~~ ✅ (M7t) · fine-pitch thread (M7b) | bounded-ish | fine-pitch needs M2 | mostly LANDED (`construct/`) |
| **M8** `drop-occt` | delete `src/engine/occt` · drop the OCCT link · stub `cc_iges_*` | terminal | **ALL native at the bar + M6 holds** | terminal, single |

```
[B1] → M2 assembly → M2 breadth ──► M3 blends/wrap        (freeform payoff — bounded per family)
                          M4-tail ┐
                          M5-tail ├── overlap the whole time  (the asymptotic gates)
                          M6 bar  ┘
   ALL native at the bar  +  M6 completeness bar HOLDS  ──►  M8 drop-occt
```

**Why M8 is a decision, not a date.** Even once every capability is native, OCCT is not unlinked on
the day the last feature lands. **M5 (healing robustness) and M6 (the completeness fuzzing bar) are
the gate.** OCCT stays the labelled oracle + fallback until the M6 differential fuzzers demonstrate
*zero silent wrong results across the entire native surface* — the moment the fallback is provably
unnecessary for the supported domain. M6-breadth already proved the bar's worth by catching OCCT
itself mis-importing shallow frustums (native vindicated by closed-form). The research walls are
mostly behind; what remains is **breadth you can parallelize** and **two gates that must hold**.

## Honest framing

- **M0 was the highest-leverage single target** — the recurring blocker under freeform booleans,
  blends, wrap-emboss, and foreign STEP import. It is now **complete end-to-end**: the mesher plus
  the STEP-reader admission (landed as M4 / M4-rational, verified vs OCCT `BRepMesh`). Foreign
  trimmed B-spline patches import native.
- **M2/M3 are the payoff.** The M2 *substrate* is (all but) done — **B2** (split) and **B3**
  (classify) landed, **B1** (recognise) in flight — so the freeform-boolean **assembly** is the
  next critical-path target, then breadth per family. They are bounded per surface family and
  asymptotic only in full arbitrary generality.
- **M5 and M6 are why `drop-occt` (#8) is a long-horizon direction, not a date.** They are
  asymptotic by nature (arbitrary broken input, sub-resolution completeness); a first robust
  slice is bankable, the guarantee is re-earned continuously. OCCT stays the labelled oracle
  and fallback until the M6 bar demonstrates the fallback is unnecessary for the supported
  domain — never removed on faith.
- Every stage ships the same way the landed tiers did: **a narrow verified slice + an explicit
  OCCT-oracle gate + an honest fallback**, one capability at a time.
