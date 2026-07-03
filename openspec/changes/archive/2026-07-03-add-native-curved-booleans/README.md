# add-native-curved-booleans

A **NARROW curved slice** of Phase 4 capability **#5 `native-booleans`** (deferred
residual #2 of that research-grade capability, `openspec/NATIVE-REWRITE.md`). The planar
slice (`add-native-booleans`, archived) made `cc_boolean` native for PLANAR-faced solids
via BSP-CSG. This change extends that native boolean to ONE analytic curved family:
**AXIS-ALIGNED box ↔ cylinder** where the cylinder axis is PARALLEL to a box axis. It
does NOT change the `cc_*` ABI, does NOT change the default engine (stays OCCT), and does
NOT fake any case: sphere, cone, non-axis-aligned cylinders, cylinder-cylinder, NURBS,
near-tangent / coincident-curved, and ALL general curved cases fall through to OCCT —
labelled, verified, never faked.

## Why this case is analytic (and the rest is research-grade)

For a cylinder whose axis is parallel to a box axis, plane-cylinder intersection is
CLOSED-FORM — no surface-surface numerical intersection is needed:

- A box face **perpendicular** to the cylinder axis cuts the cylinder in a **CIRCLE**
  (the cap trace).
- A box face **parallel** to the cylinder axis cuts the cylinder in **LINE segments**
  (rulings of the lateral surface at fixed angle).
- Point-in-cylinder is a trivial **half-space / radial** test:
  `dist_to_axis ≤ r` AND `axialMin ≤ axialCoord ≤ axialMax`.

So the whole boolean reduces to: split the cylindrical lateral face and the box planar
faces along these analytic curves, classify each fragment inside/outside the other solid
by the point-in-solid test, keep the surviving shell for the op, and heal the shared
circle / line seams watertight. The **analytic volume is exact** (`π·r²·h` cylinder
contributions), so the mandatory self-verify can check it to fp64.

A **general** curved boolean (arbitrary cylinder orientation, cylinder-cylinder,
sphere, cone, NURBS) needs true surface-surface intersection curves and robust
near-tangent handling — the classic BOPAlgo research wall — and stays OCCT-backed.

## What a boolean is (the `cc_boolean` contract — unchanged)

```c
/* Booleans: op = 0 fuse, 1 cut (a-b), 2 common. */
CCShapeId cc_boolean(CCShapeId a, CCShapeId b, int op);
```

- **fuse** (`op == 0`) — union `A ∪ B` (round boss: box + cylinder).
- **cut** (`op == 1`) — difference `A − B` (round hole: box − cylinder).
- **common** (`op == 2`) — intersection `A ∩ B`.

The result is a new watertight `Solid`. This is the `cc_*` mirror of OCCT
`BRepAlgoAPI_Fuse` / `_Cut` / `_Common` (BOPAlgo). No ABI change.

## Scope (NARROW + HONEST) — `cc_boolean` native for axis-aligned box ↔ cylinder

| Boolean case | Native in this change | Falls through to OCCT (honest, labelled) |
|---|---|---|
| **Axis-aligned box ↔ cylinder** (cylinder axis ∥ a box axis), any `op` (fuse / cut / common) — round hole, round boss, intersection | YES — analytic plane-cylinder split (circle on ⟂ faces, line rulings on ∥ faces) + point-in-solid classification + healed watertight curved shell. Analytic volume EXACT vs OCCT; curved faces deflection-bounded | — |
| Result that FAILS the self-verify guard (not watertight, or wrong analytic volume) | — | YES — the engine **DISCARDS** the native result (never emits a leaky/wrong curved boolean) |
| **Sphere / cone** on either operand | — | YES — no native analytic split for these surfaces; labelled OCCT fall-through |
| **NON-axis-aligned cylinder** (axis not parallel to a box axis) | — | YES — the plane-cylinder trace is a general conic/ellipse, not a circle/line; OCCT fall-through |
| **Cylinder ↔ cylinder** (any orientation) | — | YES — surface-surface intersection curve; OCCT fall-through |
| **NURBS / free-form** faces | — | YES — general surface-surface intersection; OCCT fall-through |
| **Near-tangent / coincident-curved** (cylinder tangent to a box face, coincident radius/axis, sliver) | — | YES — the curved-robustness wall; the builder DECLINES; OCCT fall-through |
| PLANAR-faced booleans (boxes / prisms) — the archived planar slice | Unchanged (still native via BSP-CSG) | — |

### Why the hard cases fall through (not faked)

- **Non-axis-aligned cylinders / cylinder-cylinder / sphere / cone / NURBS.** These need
  a true surface-surface intersection curve (an ellipse, a Viviani-type quartic, a
  general spatial curve) rather than the closed-form circle / line the axis-aligned case
  gives. This slice implements ONLY the analytic axis-aligned box-cylinder trace; every
  other curved case is a **labelled, verified** OCCT fall-through.
- **Near-tangent / coincident-curved configurations.** A cylinder exactly tangent to a
  box face, a coincident axis/radius, or a sliver trace is the fp64-robustness wall.
  Rather than emit a wrong classification, the native builder **declines** and the engine
  falls through to OCCT.
- **The self-verify guard is mandatory.** Even for a nominally axis-aligned case, the
  native result is accepted ONLY if it is a closed watertight solid AND has the correct
  **analytic** volume for the op; otherwise it is DISCARDED. The engine NEVER ships an
  unverified curved boolean.

## Method (locked, per NATIVE-REWRITE.md)

CLEAN-ROOM from computational-geometry references (analytic plane-quadric intersection;
the cylinder as `dist_to_axis = r`; circle / line traces) and the `cc_boolean` contract
(`include/cybercadkernel/cc_kernel.h`), with OCCT source
(`/Users/leonardoaraujo/work/OCCT/src`: `BRepAlgoAPI_Fuse` / `_Cut` / `_Common`,
`IntTools_FaceFace`, `GeomInt_IntSS`, `IntAna_QuadQuadGeo` — the analytic quadric-quadric
intersector OCCT itself uses for the plane-cylinder circle/line case) consulted as a
**reference ORACLE only** — to confirm the analytic trace kinds and the fuse/cut/common
face-survival rules, and to compare numerics — never copied.

## Architecture / OCCT boundary (unchanged from #1–#5)

- The native builder extends the existing `src/native/boolean/` subtree (stays
  **OCCT-FREE and host-buildable**, `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`, no
  OCCT, no simulator); it includes only `src/native/math`, `src/native/topology`, and
  `src/native/tessellate` (for the watertight / volume self-verify) and returns a
  `topology::Shape` (NULL ⇒ the engine falls through / errors honestly).
- `src/engine/native/native_engine.cpp` — `boolean_op` gains a curved analytic path
  alongside the planar BSP-CSG path (both guarded by the SAME mandatory self-verify,
  extended with the analytic-volume oracle). A NULL native result or a failed self-verify
  is reported honestly (native voids OCCT cannot read are never handed to OCCT — see the
  design note on the native-void constraint).
- **No `cc_*` ABI change**; the default engine stays OCCT (opt-in via `cc_set_engine(1)`),
  so every existing suite is unchanged unless it opts in. The archived PLANAR slice keeps
  working.

## Verification (two gates, per NATIVE-REWRITE.md)

1. **Host analytic unit tests** (`clang++ -std=c++20`, no OCCT): the native curved boolean
   of a box + an axis-aligned cylinder + its native tessellation.
   - **Round hole (cut) analytic volume EXACT** — `boxVol − π·r²·h` (through hole) within
     the deflection bound of the curved faces; the mesh is watertight
     (`boundaryEdgeCount == 0`).
   - **Round boss (fuse)** — `boxVol + π·r²·h_boss − overlap`, watertight.
   - **Common** — `π·r²·h_overlap`, watertight.
   - **Self-verify guard** — a deliberately wrong-volume / open curved candidate is
     REJECTED.
   - **Fall-through triggers** — a sphere, a cone, a non-axis-aligned cylinder, a
     cylinder-cylinder pair, and a near-tangent config each cause the builder to return
     NULL (or the guard to reject) — all with no OCCT.
2. **Simulator native-vs-OCCT parity through the facade** (`cc_set_engine(1)` vs default):
   the SAME `cc_boolean` calls native vs OCCT, compared on mass properties / bbox /
   watertight tessellation against the OCCT `BRepAlgoAPI` oracle. Axis-aligned
   box-cylinder cut / fuse / common match the oracle within the curved-face deflection
   bound (analytic volume ~exact); the fall-through cases (sphere / cone / non-aligned /
   cyl-cyl / NURBS / near-tangent) asserted **identical** under both engines
   (fall-through proof). Default restored in teardown; own `main()` (on the
   `run-sim-suite.sh` SKIP list) so the 221-assertion suite count is unchanged.

A requirement is done only when BOTH gates are green AND every existing suite
(`scripts/run-sim-suite.sh` 221/221, host CTest, GPU / Phase-3, and the archived planar
`test_native_boolean`) stays green at the OCCT default. This is honestly reported as a
**narrow analytic slice** (axis-aligned box-cylinder) of the research-grade curved
boolean; general curved booleans remain OCCT-backed.
