# Tasks — moat-m0-freeform-mesher (MOAT M0, the keystone)

Order: baseline capture → additive constrained triangulator → additive trimmed-
freeform mesh branch → reader admission + faithful-pcurve guard → host analytic gate
→ sim parity gate → zero-regression proof → docs, or HONEST DECLINE. All new native
code stays OCCT-free and host-buildable (`clang++ -std=c++20`), namespace
`cybercad::native::tessellate` / `cybercad::native::exchange`. No `cc_*` ABI change.
The tessellator change is **strictly additive**: the new branch is reachable ONLY by
a curved genuinely-trimmed freeform face (which TODAY declines), and every existing
mesh MUST stay byte-identical — PROVEN, not assumed. No tolerance is weakened; a
correct decline (foreign patch stays OCCT) is a first-class outcome.

## STATUS (narrow slice landed — mesher keystone)

DONE and validated host-side (OCCT-free): the additive constrained-Delaunay interior
triangulator (§1), the guarded trimmed-free-form mesh arm (§2), and the host analytic
gate (§4 — on-surface, within deflection, watertight solid, closed-form volume, rational
+ hole variants). Zero-regression PROVEN (§6.2 byte-identical per-kind snapshot vs `main`;
host 29/29 and NUMSCI-ON 36/36 green). Cognitive complexity stays 🟢 (CDT `legalize` 7,
`build` 6, `refine` 5; new mesh arm helpers ≤ 5).

DEFERRED — needs the OCCT-linked simulator, so intentionally NOT done in this OCCT-OFF
worktree: the STEP reader admission + faithful-pcurve guard (§3) and the sim `BRepMesh`
parity gate (§5). The named sim suites in §6.1 (`run-sim-suite`, STEP 77/77, curved-fillet,
…) are sim-side and were not run here; the equivalent host regression (byte-identical mesh
of every surface kind) IS proven. The mesher those steps depend on is ready and validated.

## 0. Baseline (capture BEFORE touching the tessellator)

- [ ] 0.1 Build host + NUMSCI and record the GREEN baseline: `run-sim-suite`
      (221/221), STEP import (77/77), curved-fillet (23/23), curved-chamfer (18/18),
      curved-boolean (native-pass=18), wrap-emboss (14/14), loft, phase3 (70/70).
- [ ] 0.2 Snapshot the per-face mesh signature (triangle count, watertight flag,
      enclosed volume) for a face of EVERY existing surface kind (`Plane`, `Cylinder`,
      `Cone`, `Sphere`, `Torus`, bare-periodic `BSpline`, `Bezier`) — the
      byte-identical reference for §7.
- [ ] 0.3 Obtain / confirm the foreign trimmed-B-spline STEP fixture (a pcurve-bounded
      `EDGE_LOOP` over a B-spline surface) near `src/native/exchange`; author its OCCT
      `BRepMesh` oracle (volume / area / watertight / triangle envelope).

## 1. Constrained interior triangulator (`src/native/tessellate/uv_triangulate.h`)

- [ ] 1.1 Add `triangulateConstrained(pts, loops, interior)` that triangulates the
      region so EVERY boundary segment of every loop is an output edge and the
      interior Steiner points are incorporated (constrained Delaunay in UV, or
      grid-clip + boundary-stitch — whichever proves robust on the fixture). Cover +
      no-gap + no-overlap; drop triangles whose centroid is outside the region.
- [ ] 1.2 Leave `triangulatePolygon` (the planar ear-clip) UNCHANGED — a host test
      asserts its output is byte-identical before/after this change.
- [ ] 1.3 Keep the triangulator complexity in the systems band; isolate the
      irreducible geometry behind the one entry point and document it (like the
      existing ear-clip note).

## 2. Additive trimmed-freeform mesh branch (`src/native/tessellate/face_mesher.h`)

- [ ] 2.1 Add a guarded arm in `mesh()` AFTER the `isFullRectangle` test: dispatch to
      `trimmedFreeformMesh` ONLY when `freeForm && region.hasOuter() &&
      !isFullRectangle`. Planar trimmed faces keep `earClipMesh`; full-rectangle and
      no-boundary faces keep `structuredGrid`. NO existing arm edited.
- [ ] 2.2 `trimmedFreeformMesh`: (a) boundary loops from `flattenWireShared` (shared
      per-edge fractions, anchors recorded); (b) interior UV samples on a
      curvature-driven grid (`worstCurvature` foldTwist + `divisionsFor`), kept only
      `UVRegion::inside` and clear of the boundary by a distance guard; (c)
      `triangulateConstrained`; (d) evaluate each UV vertex `S(u,v)` via `eval.d1`
      (rational-aware), snap boundary vertices to anchors, flip normals if Reversed.
- [ ] 2.3 Keep the driver's cognitive complexity ≤ ~12 by delegating boundary /
      interior-sample / triangulate / evaluate to helpers.

## 3. Reader admission of ONE foreign trimmed B-spline face (`step_reader.cpp`)

- [ ] 3.1 Admit a `B_SPLINE_SURFACE_WITH_KNOTS` (rational `weights` included) bounded
      by a real `EDGE_LOOP` through `buildFaceWithPCurves` (today it effectively
      declines: no faithful B-spline-surface pcurve arm).
- [ ] 3.2 Add the B-spline-surface arm to `pcurveFor`: straight-in-`(u,v)` edge →
      `projectBSplineUV` endpoints + UV Line; curved edge → densified projected
      samples → UV B-spline pcurve (degree/knots preserved).
- [ ] 3.3 Faithful-reconstruction guard: re-evaluate `S_face(pcurve(t)) = C_edge(t)`
      at several `t` within a SCALE-RELATIVE tolerance (never weakened). Any edge
      failing ⇒ `decline()` → OCCT. Rational eval unchanged (`bsplineSurface` already
      reads the weights shape).

## 4. Host analytic gate (`tests/native/…`)

- [ ] 4.1 Build a native trimmed `Kind::BSpline` face (a known trimmed patch, no
      OCCT) and mesh it: every triangle's chord deviation ≤ deflection, the solid is
      watertight, and its enclosed volume matches the independent value within tol.
- [ ] 4.2 Rational (`weights`) variant of the same, evaluated via
      `nurbsSurfacePoint` / `nurbsSurfaceDerivs`.
- [ ] 4.3 A hole (inner `EDGE_LOOP`) variant: triangles inside the hole omitted,
      hole-boundary vertices on the hole pcurve, still watertight.

## 5. Sim parity gate (booted simulator, OCCT linked)

- [ ] 5.1 Import the foreign trimmed-B-spline STEP fixture; the native solid's volume
      / area / watertight / triangle envelope matches OCCT `BRepMesh` within tol — OR
      the reader declines and the file round-trips through OCCT unchanged (both PASS).
- [ ] 5.2 Confirm the engine's watertight + volume self-verify DISCARDS any
      non-watertight native result → OCCT (a wrong/leaky mesh is never emitted).

## 6. Zero-regression proof (MANDATORY — the tessellator was touched)

- [ ] 6.1 Re-run the FULL tessellation-sensitive suite from §0.1; every count MUST be
      unchanged (`run-sim-suite` 221/221, STEP 77/77, curved-fillet 23/23,
      curved-chamfer 18/18, curved-boolean native-pass=18, wrap-emboss 14/14, loft,
      phase3 70/70).
- [ ] 6.2 Re-snapshot §0.2 per-kind mesh signatures; triangle counts, watertight
      status, enclosed volumes MUST be BYTE-IDENTICAL to the baseline. If ANY differs
      → revert the mesher change (see §8).

## 7. Docs / spec

- [ ] 7.1 Update `openspec/MOAT-ROADMAP.md` M0 status (landed narrow slice, or the
      honest decline with the measured gap + specific blocker). Note the M1/M2/M3/M4
      unblock.
- [ ] 7.2 `openspec validate moat-m0-freeform-mesher --strict`; archive on completion.

## 8. Honest-out (a first-class outcome, not a fallback failure)

- [ ] 8.1 If a clean additive path that keeps every existing mesh byte-identical AND
      meshes the foreign patch watertight is NOT achievable: REVERT the tessellator +
      reader changes, keep the OCCT decline for the foreign patch, and DOCUMENT the
      specific blocker + measured gap. No fabrication, no dead code, no weakened
      tolerance — the tessellator stays pristine and the decline is reported honestly.
