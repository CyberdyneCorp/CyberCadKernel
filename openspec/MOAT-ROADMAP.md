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
  admission); the only import residual is a rational-*curve* trim boundary (still OCCT, see M4).

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
  - **B4/B5 — no freeform weld/assembly.** With B1–B3 open there is no consumer wiring the split
    fragments back through the M0 trimmed-free-form mesher into a watertight welded solid, nor a
    self-verify tuned for freeform output.
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
  | **M2a (B1)** | Freeform operand descriptor + recogniser — the data model B2/B3 plug into (extends `recogniseCurvedSolid`) | `boolean/` | `shape.h` | (integration; validated once B2 + B3 exist) | scaffold now, **join point** |

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

### M3 — General freeform blends + wrap-emboss · ~2–4 py · needs M2
The curved-curved blends and freeform-base features that sit on booleans: cyl↔cyl-canal /
elliptical-crease / variable-on-freeform fillets, general chamfers, and wrap-emboss on a
sphere/cone/freeform base (all declined this session for lack of M0/M2). General canal-surface
construction + the M0 mesher + M2 booleans.
- *Oracle:* `BRepFilletAPI` / `BRepOffsetAPI` / `cc_wrap_emboss` (volume/area/watertight/continuity).

### M4 — General STEP / AP242 import (+ IGES stays dropped) · ~1.5–3 py · needs M0
The remaining import breadth on the landed AP203+ reader: **foreign rational/general B-spline
patches** (needs M0 to mesh them), AP242 **PMI semantics** (not just skip), general **trimmed
surfaces**, deep-nested/`MAPPED_ITEM` assemblies. IGES is **descoped** (STEP-only; `cc_iges_*`
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
  tail): pcurve reconstruction, self-intersecting-wire repair, ≥ 2-hole / non-planar / curved
  missing-face synthesis, arbitrary broken industrial B-rep.

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
  native-pass=18 fixtures. REMAINING (asymptotic, gates M8): extend the generator across blends,
  import, and healing; loop-until-dry critics; the standing zero-silent-wrong-results bar.

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

Still deferred (honest decline, this change did NOT land them): **orientation guided/rail
sweep** (`cc_guided_orient_sweep`) — needs a NEW OCCT `MakePipeShell(guideWire)` oracle + a
guide-aimed native frame (the ABI-additive path is open, next M7 slice); **non-planar-cap
loft** (needs a filling surface, no closed-form volume); **fine-pitch self-intersecting
threads** (intersecting-helicoid trimming — needs M1/M2). Independent of the
freeform-boolean chain except fine-pitch (needs M2).
- *Oracle:* `BRepOffsetAPI_MakePipeShell` / `ThruSections` / thread fixtures (volume/watertight).

### M8 — `drop-occt` — unlink OCCT · gated on M0–M7 + M6 bar
Delete `src/engine/occt`, drop the OCCT link, remove/stub `cc_iges_*`. **Only** once every
stage above is native at the acceptance bar AND the M6 completeness bar holds (differential
fuzzing shows zero silent wrong results — every non-native input honestly declines with a
clear error rather than a fabricated shape). This is the terminal step; it does not begin
until the fallback is provably unnecessary for the supported domain.

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
| **M0** freeform mesher/trimmer | `tessellate/` | — | ✅ **Wave-1 slice LANDED** — mesher ready; unblocks M2/M4 |
| **M1** SSI S4 general robustness | `ssi/marching` | — | ✅ **Wave-1 slice LANDED** — breadth continues (asymptotic) |
| **M5** shape-healing robustness | `heal/` | — | ✅ **Wave-1 slice LANDED** — tail continues (asymptotic) |
| **M6** completeness / fuzzing harness | test infra + `ssi/` | — | ✅ **Wave-1 + breadth LANDED** — curved-boolean fuzzer + STEP round-trip fuzzer (0 DISAGREED); bar continues (gates M8) |
| **M7a** guided sweep · hard loft | `construct/` | — | ✅ **Wave-1 slice LANDED** — N-section loft ABI (`cc_solid_loft_sections`); guided sweep (measured trap) + non-planar-cap loft remain OCCT |
| **M4** general STEP/AP242 import | `exchange/` | M0 | ✅ **Wave-2 LANDED** — non-rational + `RATIONAL_B_SPLINE_SURFACE` admission native (parity 83/83); rational-*curve* trims, PMI, deep assemblies remain OCCT |
| **M2b (B2)** freeform face-split | `boolean/` · `ssi/` | M0 ✅ + M1 ✅ | ✅ **Wave-2 slice LANDED** — `boolean/face_split.h` `splitFace` (tiles vs OCCT 12/12); non-convex/multi-crossing tail declines |
| **M2c (B3)** freeform point-in-solid | `boolean/` | M0 ✅ | ✅ **Wave-2 slice LANDED** — `boolean/freeform_membership.h` `classifyPointInMesh` (crispDISAGREE=0 vs `BRepClass3d`); near-tangent band → On/Unknown |
| **M2a (B1)** freeform operand descriptor | `boolean/` | `shape.h` | ▶ **OPEN NOW — the next step** (last M2 substrate piece; join point for B2 + B3) |
| **M2** general freeform booleans (assembly) | `boolean/` | **B1 + B2 ✅ + B3 ✅** | ▶ **reachable once B1 lands** — recognise[B1] → trace[M1] → split[B2] → classify[B3] → weld[M0] |
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
| **Moat first slices landed (this campaign)** | M0 keystone trimmed-free-form interior mesher · M1 freeform S4-d open-arm branch · M5 opt-in gap bridging · M6 curved-boolean differential fuzzer (512 trials, 0 DISAGREED) — each verified native-vs-OCCT, additive, `src/native` OCCT-free |
| **Remaining to drop OCCT (M2–M8 + tails)** | ≈ **5–11 py**, now dominated by M2/M3 (freeform booleans/blends — Wave 2 open) + M4 import, with M5 + M6 the asymptotic tails; the M0 keystone mesher is past its first slice (STEP admission = M4) |
| ~~IGES~~ | descoped (STEP-only) — saved ~1.5–3 py |

## Honest framing

- **M0 was the highest-leverage single target** — the recurring blocker under freeform booleans,
  blends, wrap-emboss, and foreign STEP import. Its mesher first slice is now **landed**, so the
  most-downstream work (M2/M4) is unblocked; the remaining M0 piece is the STEP-reader admission,
  which is M4's job (gated by the sim `BRepMesh` oracle).
- **M2/M3 are the payoff** and are now **reachable** (M0 + M1 first slices landed); they are
  bounded per surface family and asymptotic only in full arbitrary generality. **M2 is the next
  critical-path target.**
- **M5 and M6 are why `drop-occt` (#8) is a long-horizon direction, not a date.** They are
  asymptotic by nature (arbitrary broken input, sub-resolution completeness); a first robust
  slice is bankable, the guarantee is re-earned continuously. OCCT stays the labelled oracle
  and fallback until the M6 bar demonstrates the fallback is unnecessary for the supported
  domain — never removed on faith.
- Every stage ships the same way the landed tiers did: **a narrow verified slice + an explicit
  OCCT-oracle gate + an honest fallback**, one capability at a time.
