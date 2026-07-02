# Roadmap

Phase-level view of the **wrap → accelerate → rewrite** trajectory. The
**canonical, change-level roadmap** (with every OpenSpec change, capability,
contract reference, and GitHub issue) is [openspec/ROADMAP.md](../openspec/ROADMAP.md).

```mermaid
flowchart LR
    P0["Phase 0 · Foundation<br/>wrap OCCT ✅"] --> P1["Phase 1 · Multi-core CPU ✅"]
    P1 --> P2["Phase 2 · GPU / Metal ◐"]
    P2 --> P3["Phase 3 · Features OCCT lacks ◐"]
    P3 --> P4["Phase 4 · Native rewrite<br/>drop OCCT ☐"]

    style P0 fill:#1b5e20,color:#fff,stroke:#333
    style P1 fill:#1b5e20,color:#fff,stroke:#333
    style P2 fill:#e65100,color:#fff,stroke:#333
    style P3 fill:#e65100,color:#fff,stroke:#333
    style P4 fill:#37474f,color:#fff,stroke:#333
```

Legend: ✅ complete (at the simulator acceptance bar) · ◐ in progress · ☐ planned.

## Phase 0 — Foundation ✅
Stand up the library and move CyberCad's OCCT bridge behind the `cc_*` facade,
unchanged in behaviour. Seams: `kernel-facade`, `engine-adapter`,
`operation-scheduler`, `compute-backend`. All 57 `cc_*` verified on the simulator.

## Phase 1 — Multi-core acceleration ✅
Turn on OCCT's existing parallel paths behind the facade — parallel booleans
(`SetRunParallel` + tuned fuzzy), parallel meshing (`InParallel`), bounded worker
pool, fine-thread boolean gate. Determinism audit: serial == parallel, bit-identical.

## Phase 2 — GPU acceleration (Metal) ◐
fp32-tolerant, data-parallel work through the compute backend; CPU stays the
source of truth.
- ✅ **Metal compute backend** — device, unified-memory buffers, runtime MSL, fp32 guard.
- ✅ **GPU tessellation** — surface-eval wired into `cc_tessellate` (per-face, OCCT fallback).
- ◐ **Spatial acceleration** — GPU LBVH + ray-pick verified; frustum-pick leg +
  `cc_*` cull wiring remain.

## Phase 3 — Missing features OCCT lacks ◐
Native geometry OCCT can't do, each behind the same facade.
- ✅ **Reference geometry** — datum planes/axes.
- ✅ **Robust wrap-emboss** (#290) — cap-and-side + healed sew.
- ✅ **Robust thread↔shaft boolean** (#286) — feature-based, no minutes-long hang.
- ✅ **G2 blend fillet** (#284) — measured curvature continuity at straight seams.
- ◐ **Full-round fillet** (#285) — rolling-ball for parallel walls; else valid fallback.

## Phase 4 — Native rewrite → drop OCCT ☐
Replace the OCCT adapter with native C++20, one capability at a time, each
validated against the OCCT path behind the same facade call, then OCCT unlinked
for that capability. Rough order: math/geometry → topology → tessellation →
swept-solids → **booleans** (the hardest, longest-lived OCCT dependency) →
fillets/offsets → data-exchange → **drop OCCT** (fully native C++20, cleanly MIT).

---

**Guiding rules**

- The `cc_*` ABI never breaks; additive-only. The app is insulated from every phase.
- Native and OCCT-backed implementations coexist behind the adapter so each
  migration is measured (correctness + performance) before OCCT is retired.
- Changes are proposed (`/opsx:propose`) when a phase is about to start, and their
  delta specs are synced into `openspec/specs/` when validated.
