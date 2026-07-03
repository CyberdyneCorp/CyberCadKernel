# add-native-booleans

Phase 4 capability **#5 — `native-booleans`**, the **RESEARCH-GRADE** capability and
the hardest, longest-lived OCCT dependency (`openspec/NATIVE-REWRITE.md`). This change
introduces the FIRST native slice of `cc_boolean` (`op`: `0` fuse, `1` cut `a−b`,
`2` common) in `NativeEngine` — an **analytic PLANAR-FACED** boolean (polyhedra: boxes,
prisms, convex or simple-concave solids) — guarded by a **mandatory self-verify** that
DISCARDS any bad native result and falls through to OCCT.

It does NOT change the `cc_*` ABI, does NOT change the default engine (stays OCCT), and
does NOT fake any case: curved-face solids, near-tangent / coincident / degenerate
configurations, and anything the planar algorithm cannot robustly handle fall through to
OCCT — labelled, verified, never faked. It is fully acceptable (and reported honestly)
that only axis-aligned / planar-polyhedron booleans land native and everything else is
OCCT-fallback.

## What a boolean is (the `cc_boolean` contract)

```c
/* Booleans: op = 0 fuse, 1 cut (a-b), 2 common. */
CCShapeId cc_boolean(CCShapeId a, CCShapeId b, int op);
```

`cc_boolean(a, b, op)` combines two solids by regularised set algebra:

- **fuse** (`op == 0`) — the union `A ∪ B`.
- **cut** (`op == 1`) — the difference `A − B`.
- **common** (`op == 2`) — the intersection `A ∩ B`.

The result is a new watertight `Solid` body. This is the `cc_*` mirror of OCCT
`BRepAlgoAPI_Fuse` / `_Cut` / `_Common` (BOPAlgo). The general robust B-rep boolean over
arbitrary NURBS solids is research-grade and out of reach in one pass; this change lands
the **analytic-planar-first** slice and verifies it EXACTLY against the OCCT oracle on
axis-aligned box cases.

## Scope (#5, analytic-planar-first) — `cc_boolean` native for PLANAR-faced solids

| Boolean case | Native in this change | Falls through to OCCT (honest, labelled) |
|---|---|---|
| **Two PLANAR-faced solids** (polyhedra — boxes, prisms, convex or simple-concave), any `op` (fuse / cut / common) | YES — face–face intersection segments → face splitting → fragment inside/outside/on classification → surviving-shell assembly → sew/heal to a watertight `Solid`. EXACT vs OCCT on axis-aligned box cases | — |
| Result that FAILS the self-verify guard (not closed 2-manifold, or wrong set-algebra volume sign/magnitude) | — | YES — the engine **DISCARDS** the native result and falls through to OCCT (never emits a leaky/wrong solid) |
| **Curved-face solids** (cylinder / sphere / cone / any non-planar surface on either operand) | — | YES — surface–surface intersection curves are out of scope for this slice; labelled OCCT fall-through |
| **Near-tangent / coincident / degenerate** configurations (coplanar overlapping faces, touching-only contact, shared boundary, slivers) | — | YES — the classic BOPAlgo robustness wall; the builder DECLINES and falls through, rather than emit a wrong classification |
| One or both operands **not a native body** (foreign / OCCT-built shape id) | — | YES — a native boolean requires the native B-rep of both operands; a foreign body falls through unchanged |

### Why the hard cases fall through (not faked)

- **Curved-face solids.** A robust boolean over cylinders / spheres / NURBS needs
  surface–surface intersection (the intersection CURVES between arbitrary analytic +
  free-form surfaces) and curved section-edge classification — a strictly harder problem
  than planar face–face segments. This slice does planar faces only; any curved face on
  either operand is a **labelled, verified** OCCT fall-through.
- **Near-tangent / coincident / degenerate configurations.** Coplanar overlapping faces,
  touching-only (measure-zero) contact, shared boundaries, and slivers are the
  fp64-robustness wall BOPAlgo is engineered around. Rather than emit a wrong
  classification, the native builder **declines** (returns a NULL `Shape`) and the engine
  falls through to OCCT.
- **The self-verify guard is mandatory.** Even for a nominally planar case, the native
  result is accepted ONLY if it is a closed watertight 2-manifold AND has the correct
  set-algebra volume (sign + magnitude for the op); otherwise it is DISCARDED and the
  call falls through to OCCT. The engine NEVER ships an unverified boolean solid.

## Method (locked, per NATIVE-REWRITE.md)

CLEAN-ROOM from computational-geometry references (polyhedral boolean / mesh-boolean
literature — face-splitting along intersection segments, point-in-polyhedron
classification by ray parity / winding, BSP/half-space reasoning for convex parts) and
the `cc_boolean` contract (the doc-comment in `include/cybercadkernel/cc_kernel.h`), with
OCCT source (`/Users/leonardoaraujo/work/OCCT/src`: `BRepAlgoAPI_BooleanOperation`,
`BRepAlgoAPI_Fuse` / `_Cut` / `_Common`, `BOPAlgo_Builder` / `BOPAlgo_PaveFiller`,
`BOPTools`) consulted as a **reference ORACLE only** — to confirm the pipeline stages
(intersection → split → classify → build) and the fuse/cut/common face-survival rules,
and to compare numerics — never copied.

## Architecture / OCCT boundary (unchanged from #1–#4)

- A new native builder lives under `src/native/boolean/` (new subtree) and stays
  **OCCT-FREE and host-buildable** (`/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`, no
  OCCT, no simulator); it includes only `src/native/math` + `src/native/topology` +
  `src/native/tessellate` (for the watertight / volume self-verify) and returns a
  `topology::Shape` (NULL ⇒ the engine falls through).
- `src/engine/native/native_engine.{h,cpp}` — `boolean_op` (currently a pure
  `CC_NATIVE_BODY_UNSUPPORTED` + `fallback().boolean_op(...)`) becomes
  native-else-fallback: when BOTH operands are native bodies, it runs the native planar
  boolean and applies the mandatory self-verify guard; a NULL native result, a foreign
  operand, or a failed self-verify falls through to the held fallback engine with **no
  native interception**. OCCT stays behind `CYBERCAD_HAS_OCCT`; the native builder never
  sees OCCT.
- **No `cc_*` ABI change**; the default engine stays OCCT (opt-in via `cc_set_engine(1)`),
  so every existing suite is unchanged unless it opts in.

## Verification (two gates, per NATIVE-REWRITE.md)

1. **Host analytic unit tests** (`clang++ -std=c++20`, no OCCT): the native boolean of
   two planar solids + its native tessellation.
   - **Axis-aligned box fuse / cut / common EXACT** — two overlapping axis-aligned boxes
     produce a watertight result whose volume equals the exact set-algebra value
     (`|A| + |B| − |A∩B|` for fuse, `|A| − |A∩B|` for cut, `|A∩B|` for common), the mesh
     is watertight (`boundaryEdgeCount == 0`), and every edge is shared by exactly two
     faces (closed 2-manifold).
   - **Prism / simple-concave** — an L-prism cut by a box, a convex fuse, etc., watertight
     with the exact set-algebra volume.
   - **Self-verify guard** — a deliberately mis-assembled shell (open, or wrong volume)
     is REJECTED by the guard.
   - **Fall-through triggers** — a curved-face operand, a coincident/tangent
     configuration, and a foreign body each cause the builder to return NULL (or the guard
     to reject), so the engine falls through.
2. **Simulator native-vs-OCCT parity through the facade** (`cc_set_engine(1)` vs default):
   the SAME `cc_boolean` calls issued native vs OCCT, compared on mass properties / bbox /
   sub-shape counts / watertight tessellation against the OCCT
   `BRepAlgoAPI_Fuse`/`_Cut`/`_Common` oracle. Axis-aligned box booleans match the oracle
   **EXACTLY** (volume / bbox relative error ~0). The fall-through cases (curved-face,
   near-tangent / coincident, foreign body) asserted **identical** under both engines
   (fall-through proof). Default restored in teardown; own `main()` (on the
   `run-sim-suite.sh` SKIP list) so the 221-assertion suite count is unchanged.

A requirement is done only when BOTH gates are green AND every existing suite
(`scripts/run-sim-suite.sh` 221/221, host CTest, GPU / Phase-3) stays green at the OCCT
default. This is honestly reported as an **analytic-planar-first slice** of the
research-grade boolean; general robustness (curved surfaces, coincident/tangent handling,
full sew/heal) is future work.
