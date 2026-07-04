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
- ✅ **STEP export** (native AP203; import stays OCCT).
- ✅ **Numeric foundations (#2)** — adopted **NumPP + SciPP** (MIT C++20 NumPy/SciPy
  ports) as the OCCT-free numeric substrate + native closest-point (Extrema).
- ✅ **SSI S1** — analytic surface-surface intersection (elementary pairs, closed-form
  curves) vs OCCT `GeomAPI_IntSS`; **SSI S2** — subdivision seeding (transversal
  seeds for freeform/skew-quadric pairs, 100% recall vs OCCT); **SSI S3** —
  marching-line tracer (WLine): full transversal intersection curves traced from the
  S2 seeds vs OCCT `IntPatch` (5 pairs / 9 branches, all fully-traced, 0 near-tangent-
  truncated; onSurf ≤ 6.81e-07, length within the step tol).
- ◐ **SSI S5-a/b/c (curved-boolean slices)** — the SSI-curve-driven
  split→classify→weld pipeline (`src/native/boolean/ssi_boolean.{h,cpp}`, consumes the
  S3 `TraceSet`) produces **five native curved-boolean sub-cases verified vs OCCT
  `BRepAlgoAPI_{Fuse,Cut,Common}`**: the through-drill cylinder∩cylinder COMMON (S5-a) +
  FUSE + CUT (S5-b) and the sphere∩sphere COMMON lens (S5-c, equal + unequal radii) — all
  watertight, ΔV ≤ 8e-04. Sphere fuse/cut, other curved-curved families, and near-tangent
  pairs (incl. Steinmetz) still decline to OCCT — honest, measured fallbacks.

**Still OCCT-backed (the tail that keeps OCCT linked):**
- ☐ SSI **S4 near-tangent robustness** (the moat) → **wider S5 curved booleans**
  (fuse/cut caps, more families, lifting the near-tangent gate — consuming the S3 WLines).
- ☐ General curved **booleans** & **blends** (sit on SSI); curved **wrap-emboss**.
- ☐ Non-planar/guided/rail sweep robustness; general loft; fine-pitch threads.
- ☐ **Shape healing**; **STEP/IGES import**.
- ☐ **`drop-occt`** — BLOCKED until the above are native (research-grade, multi-year).

**Effort:** ≈ 0.9–1.3 py delivered (planar/analytic breadth); ≈ **9–18 py remaining**
to genuinely drop OCCT, concentrated in SSI-S4 robustness + healing + import.

---

**Guiding rules**

- The `cc_*` ABI never breaks; additive-only. The app is insulated from every phase.
- Native and OCCT-backed implementations coexist behind the adapter so each
  migration is measured (correctness + performance) before OCCT is retired.
- Changes are proposed (`/opsx:propose`) when a phase is about to start, and their
  delta specs are synced into `openspec/specs/` when validated.
