# Proposal — add-native-curved-booleans

## Why

Phase 4 replaces OCCT capability by capability behind the unchanged `cc_*` facade
(`openspec/NATIVE-REWRITE.md`). Capability **#5 `native-booleans`** is the research-grade
one; its FIRST slice (`add-native-booleans`, archived) made `cc_boolean` native for
PLANAR-faced solids (polyhedra) via a BSP-tree CSG, guarded by a mandatory watertight +
set-algebra-volume self-verify, with every curved / near-tangent / foreign case falling
through to OCCT. That slice explicitly deferred **curved-face booleans** as future work
(deferred residual #2).

A **general** curved boolean over arbitrary NURBS / analytic solids is genuinely
research-grade: it needs robust surface-surface intersection curves (the intersection of
two arbitrary surfaces is a general spatial curve) and fp64-robust near-tangent handling
— the classic BOPAlgo wall. Doing it all at once would risk faking coverage on the hard
part. This proposal scopes the ONE curved family where the intersection is **analytic /
closed-form** and therefore tractable and exactly verifiable: an **axis-aligned box ↔
cylinder**, where the cylinder axis is parallel to a box axis.

For that case, plane-cylinder intersection needs no numerical surface-surface solver:

- a box face **perpendicular** to the cylinder axis cuts the cylinder in a **CIRCLE**;
- a box face **parallel** to the axis cuts it in **LINE segments** (rulings);
- point-in-cylinder is a **radial half-space** test (`dist_to_axis ≤ r` within the axial
  band).

The analytic volume (`π·r²·h`) is exact, so the mandatory self-verify can check the
result to fp64. This is explicitly reported as a NARROW analytic slice; sphere, cone,
non-axis-aligned cylinders, cylinder-cylinder, NURBS, and all general curved cases remain
honest, labelled OCCT fall-through.

## What changes

1. **Curved analytic extension of the native boolean subtree**
   (`src/native/boolean/`, OCCT-free, host-buildable). New headers extending the existing
   `polygon.h` / `bsp.h` / `assemble.h` / `native_boolean.h`, returning a
   `topology::Shape` (NULL ⇒ fall through). They include only `src/native/math`,
   `src/native/topology`, and `src/native/tessellate` (for the self-verify), never OCCT.
   The pipeline stages, mirroring the planar slice but for the one analytic curved family:
   - **Domain gate.** Recognise the axis-aligned box ↔ cylinder configuration: one
     operand is an all-planar axis-aligned box; the other is a `Cylinder`-lateral +
     two-planar-cap solid whose axis is parallel to a box axis (and to a world axis).
     Anything else → NULL (OCCT fall-through). Reuse the existing `Cylinder`
     `FaceSurface` and `Circle` `EdgeCurve` (they already exist in
     `src/native/topology`).
   - **Analytic plane-cylinder split.** For each box face: a face ⟂ the cylinder axis
     traces a **CIRCLE** on the cylinder (split the box face by that circle → a disk hole
     / annulus, and cap the cylinder at that plane with a Circle-bounded planar disk); a
     face ∥ the axis traces **LINE rulings** (split the cylindrical lateral face along
     those axial lines, and split the box face along the vertical chord segment). Keep the
     circle / line as TRUE `Circle` / `Line` edges (deflection-bounded facets only at
     tessellation, never a chord polyline in the B-rep).
   - **Fragment classification.** Classify each planar fragment and each cylindrical
     lateral fragment INSIDE / OUTSIDE / ON the other solid by the point-in-solid test at
     the fragment's interior point — point-in-box (6 half-spaces) and point-in-cylinder
     (radial + axial half-space).
   - **Surviving-shell assembly + heal.** Select the fragments that survive for the op
     (fuse: outside-outside; cut: box-outside-cyl + cyl-inside-box reversed → round hole;
     common: box-inside-cyl + cyl-inside-box), orient outward, and sew/heal the shared
     circle / line seams into one closed watertight `Solid`. Reuse the planar
     `assemble.h` vertex-weld + T-junction repair, extended to weld along a shared
     **circle** edge (curved seam) so the cap disk and the lateral fragment share the
     circle discretization watertight (the same shared-1D-discretization contract the
     tessellator's two-stage edge/face mesher already enforces).
2. **Analytic point-in-cylinder + plane-cylinder trace predicates**
   (`src/native/boolean/`, OCCT-free). `dist_to_axis ≤ r` within `[axialMin, axialMax]`;
   the circle trace of a ⟂ plane; the line-ruling trace of a ∥ plane; near-tolerance
   ambiguity (tangent / coincident) ⇒ the builder DECLINES.
3. **Analytic-volume self-verify** (`src/engine/native/native_engine.cpp`, extending the
   existing `booleanResultVerified`). The current guard checks watertight + set-algebra
   volume from the operands' native volumes; for the curved case the operand volumes are
   themselves analytic (`π·r²·h` for the cylinder), so the SAME check applies — the
   candidate volume `Vr` must satisfy `Vr ≈ boxVol − π·r²·h` (cut through-hole),
   `≈ boxVol + π·r²·h_boss − overlap` (fuse boss), or `≈ π·r²·h_overlap` (common), within a
   relative tolerance that accounts for the curved-face tessellation deflection. If the
   watertight OR volume check fails, the candidate is **DISCARDED**.
4. **`NativeEngine` glue** (`src/engine/native/native_engine.cpp`). `boolean_op` — today
   native-planar-else-honest-error for two native operands — gains the curved analytic
   attempt: when both operands are native and the planar builder declines, try the curved
   analytic builder; apply the SAME self-verify. A NULL curved result (a sphere / cone /
   non-aligned / cyl-cyl / near-tangent DECLINE) is reported honestly. OCCT stays behind
   `CYBERCAD_HAS_OCCT`; the native builder never sees OCCT.

## Non-goals (DEFERRED — fall through to OCCT, not implemented, not faked)

- **Sphere / cone** on either operand — no native analytic split for these surfaces.
  Labelled, verified OCCT fall-through.
- **NON-axis-aligned cylinders** (axis not parallel to a box axis) — the plane-cylinder
  trace is a general ellipse / conic, not a circle / line. OCCT fall-through.
- **Cylinder ↔ cylinder** (any orientation) — a true surface-surface intersection curve.
  OCCT fall-through.
- **NURBS / free-form faces** — general surface-surface intersection. OCCT fall-through.
- **Near-tangent / coincident-curved / degenerate** configurations — cylinder tangent to
  a box face, coincident axis/radius, sliver trace. The builder DECLINES rather than emit
  a wrong classification; OCCT fall-through.
- **General robust curved B-rep boolean** — full surface-surface intersection, robust
  near-tangent handling, and shape healing over arbitrary solids remain future work (the
  rest of the research-grade capability). This slice does NOT advance them.
- The archived PLANAR-polyhedron boolean and every non-boolean op — unchanged.

## Impact

- New curved headers in the `src/native/boolean/` subtree (analytic plane-cylinder trace
  / point-in-solid / curved split / curved-seam heal) added to the `native_boolean.h`
  umbrella — all OCCT-free, host-buildable. The planar `polygon.h` / `bsp.h` /
  `assemble.h` are extended (curved-seam weld) but the planar path is unchanged. New host
  CTest cases in `tests/test_native_boolean.cpp` (+ facade cases in
  `tests/test_native_engine.cpp`).
- `src/engine/native/native_engine.cpp` — `boolean_op` gains the curved analytic attempt;
  `booleanResultVerified` gains the analytic-volume oracle for the box-cylinder case.
  `native_engine.h` unchanged (`boolean_op` signature already present).
- **No** `include/cybercadkernel/cc_kernel.h` signature change; **no**
  `src/facade/cc_kernel.cpp` change (the `cc_boolean` entry already routes through the
  active engine). The `cc_boolean` doc-comment (op: 0 fuse, 1 cut a−b, 2 common) is the
  contract this change extends natively for the axis-aligned box-cylinder case.
- Behaviour unchanged by default (engine stays OCCT); only callers that call
  `cc_set_engine(1)` see the new native curved path. All existing suites (including the
  archived planar `test_native_boolean`) stay green at the OCCT default.

## Verification

Two independent gates from `NATIVE-REWRITE.md`: (a) **host** analytic-value unit tests on
the built native curved boolean B-rep + native tessellation — an axis-aligned box-cylinder
cut (round hole) is a watertight solid with volume `boxVol − π·r²·h` within the curved
deflection bound; a fuse (round boss) and a common are watertight with their analytic
volumes; a wrong-volume / open candidate is rejected by the self-verify; and a sphere /
cone / non-axis-aligned cylinder / cylinder-cylinder / near-tangent case each cause
fall-through — all with no OCCT; (b) **sim parity** through the facade (`cc_set_engine(1)`
vs default) comparing native vs OCCT `BRepAlgoAPI_Fuse`/`_Cut`/`_Common` for axis-aligned
box-cylinder booleans (analytic volume ~exact, curved faces deflection-bounded, watertight)
and asserting the fall-through cases (sphere / cone / non-aligned / cyl-cyl / NURBS /
near-tangent) identical under both engines. Done only when both gates pass and every
existing suite stays green at the OCCT default. Reported honestly as a narrow analytic
slice; general curved booleans remain research-grade OCCT-backed.
