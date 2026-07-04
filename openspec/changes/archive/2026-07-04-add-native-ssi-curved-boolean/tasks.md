# Tasks — add-native-ssi-curved-boolean (SSI Stage S5-a)

Verification levels: **host** = OCCT-free host CTest (Steinmetz `16 r³/3` analytic
oracle for equal-cyl common; watertight `boundaryEdgeCount == 0` + every edge shared by
exactly two faces; correct set-algebra volume sign; every WLine seam node on both
surfaces ≤ tol; near-tangent / coincident fixtures return NULL — deferred, no native
solid); **sim** = native-vs-OCCT `BRepAlgoAPI_{Fuse,Cut,Common}` parity on a booted iOS
simulator (volume, surface area, watertight closed shell, shape validity) via
`native_ssi_curved_boolean_parity.mm`. The path is invoked behind the existing
`cc_boolean` op codes — **no `cc_*` entry point is added or changed**; the builder is
asserted at the `cybercad::native::boolean` C++ boundary. It consumes the S3
`ssi::TraceSet`, so the SSI-driven path is compiled under **`CYBERCAD_HAS_NUMSCI`**.
`src/native/**` stays OCCT-free.

## 1. SSI-driven curved-boolean module + engine wiring
- [ ] 1.1 Add `src/native/boolean/ssi_curved.h` (+ `.cpp`): `ssi_curved_boolean(a, b,
  op)` — the split → classify → weld driver consuming the S3 `ssi::TraceSet`. OCCT-free;
  reuses `native-ssi`, `bsp.h` (classification idea), `assemble.h`, `native-math`,
  `native-numerics`. (**host**)
- [ ] 1.2 Wire it into `boolean_solid` (`native_boolean.h`) as a SIBLING path: try
  analytic `curved::tryBoxCylinder` → SSI-driven `ssi_curved_boolean` (when an operand
  has an elementary curved face outside the analytic family) → planar BSP-CSG. NULL from
  each falls through to the next; a fully-NULL result → OCCT. No change to the planar or
  analytic paths. (**host**)
- [ ] 1.3 Compile the SSI-driven path under `CYBERCAD_HAS_NUMSCI` (declarations visible,
  definitions gated — like `marching.h`); wire the host test target the way
  `test_native_ssi_marching` is wired in `CMakeLists.txt`. (**host** NUMSCI on/off)

## 2. Gate on a fully-transversal trace  [CYBERCAD_HAS_NUMSCI]
- [ ] 2.1 For each intersecting curved face pair, build the two `SurfaceAdapter`s and call
  `ssi::trace_intersection` / `trace_from_seeds`; proceed ONLY when `nearTangentGaps == 0`
  and every consumed `WLine.status` is `Closed` or `BoundaryExit`. (**host**)
- [ ] 2.2 A `TraceSet` with `nearTangentGaps > 0`, or any `NearTangent` / `Failed`
  WLine, returns NULL (→ OCCT) — the honest S4 hand-off boundary, never consumed.
  (**host** ✓ near-tangent fixture returns NULL)

## 3. Split — cut each curved face along its WLine  [CYBERCAD_HAS_NUMSCI]
- [ ] 3.1 `splitFaceAlongWLine()` — use the WLine's per-node `(u,v)` track on that face
  as the split polyline in the face's UV domain (curved analogue of `bsp.h`
  `splitPolygon`), partitioning the trimmed face into fragments. Closed WLine → enclosed
  UV loop (inner/outer); BoundaryExit WLine → edge-to-edge (two side fragments).
  (**host**)
- [ ] 3.2 Build the shared **seam edge**: 3D geometry = the WLine's fitted B-spline;
  pcurve on each face = that face's `(u,v)` track. Each fragment keeps its parent face's
  exact surface kind (Cylinder / Sphere / Cone / Plane) — nothing faceted. (**host** ✓
  seam nodes on both surfaces ≤ tol)

## 4. Classify — curved point-in-solid  [CYBERCAD_HAS_NUMSCI]
- [ ] 4.1 `pointInSolid()` — curved containment test: signed inside/outside per the other
  operand's exact faces (cylinder `r(P)<R` in the axial slab; sphere `‖P−c‖<R`; box six
  half-spaces; cone apex-angle in the height slab), composed by the solid's boolean
  structure (reuse the `bsp.h` clip/invert idea), ON-band measured via `native-numerics`
  closest-point. (**host**)
- [ ] 4.2 `classifyFragment()` — sample a UV point strictly interior to the fragment
  (centroid, reject-and-reseed off seam/trim/ON-band), evaluate `P*`, tag INSIDE /
  OUTSIDE / ON. A robustly-ON sample (coincident / tangent) → abort native → NULL (OCCT),
  never a guessed side. (**host**)
- [ ] 4.3 Select survivors per the op face-survival rule — the SAME set algebra as
  `booleanPolygons`: fuse = outside(A)∪outside(B); cut = outside(A)∪inside(B)-reversed;
  common = inside(A)∩inside(B); orient outward. (**host**)

## 5. Weld — watertight curved shell  [CYBERCAD_HAS_NUMSCI]
- [ ] 5.1 Extend `assemble.h` (or an `ssi_curved` weld) to sew surviving curved + planar
  fragments: weld coincident corners → shared `Vertex`; the WLine seam edge is shared by
  exactly the two fragments it split (one per operand) → the two faces meet watertight
  along the curved seam (the tessellator's curved-seam weld). Wrap faces → `Shell` →
  `Solid`. (**host** ✓ watertight)

## 6. Self-verify → OCCT fallback (ENGINE)
- [ ] 6.1 `ssi_curved_boolean` returns the assembled `Solid` or NULL — it does NOT decide
  shippability. The engine (`native_engine.cpp`) runs the mandatory guard: (a) watertight
  closed 2-manifold at every deflection in the mesher ladder (positive enclosed volume,
  every edge shared by exactly two faces) AND (b) correct set-algebra volume sign +
  magnitude for the op within the deflection-sized relative tolerance. (**host**)
- [ ] 6.2 EITHER check failing → engine DISCARDS the candidate → OCCT `BRepAlgoAPI` (OCCT
  operand) or an honest error (both native). The engine NEVER emits an unverified / leaky
  / wrong curved boolean. (**host** ✓ a mis-welded / wrong-volume candidate discarded)

## 7. Honest scope — deferrals (never faked)
- [ ] 7.1 Near-tangent (`nearTangentGaps > 0`), coincident / overlapping curved faces,
  branch-point / self-intersecting seams, and freeform (NURBS / Bézier) operand faces →
  NULL (OCCT), documented in `ssi_curved.h` header + the `native-booleans` /
  `native-ssi` namespace docs; `nearTangentGaps > 0` documented as the S4 fallback
  boundary. (**host** ✓ docs + NULL fixtures)

## 8. Verification (two gates)
- [ ] 8.1 Host suite `test_native_ssi_curved_boolean`: equal-radius right-angle cyl∩cyl
  `common` → Steinmetz `16 r³/3` within the deflection band; through-`cut` complement;
  sphere∩box, cone∩box cut/fuse/common → watertight, correct volume sign, seam nodes on
  both surfaces ≤ tol; near-tangent / coincident fixtures → NULL (deferred). No OCCT.
  Full CTest green NUMSCI on AND off (S5-a tests correctly absent with NUMSCI off).
  (**host**)
- [ ] 8.2 Sim native-vs-OCCT `BRepAlgoAPI_{Fuse,Cut,Common}` parity
  (`native_ssi_curved_boolean_parity.mm`, modelled on
  `scripts/run-sim-native-ssi-marching.sh` + `tests/sim/native_ssi_marching_parity.mm`):
  volume, surface area, watertight closed shell, shape validity on cyl∩cyl (Steinmetz +
  cut + fuse), sphere∩box, cone∩box; run via `xcrun simctl spawn <booted udid>`. Report
  per-pair deltas + the near-tangent gap count (pairs deferred to OCCT). (**sim**)
- [ ] 8.3 `openspec validate add-native-ssi-curved-boolean --strict` green; mark S5-a
  done-at-bar with measured deltas in `SSI-ROADMAP.md` / `ROADMAP.md` /
  `NATIVE-REWRITE.md` / `README.md`; S4 robustness (moat) + freeform/multi-branch curved
  booleans remain the tail.

## Deferred to S4 / OCCT (NOT in S5-a scope — honest)

- [ ] **Near-tangent / coincident-curved** pairs (`nearTangentGaps > 0`, tangent faces,
  coincident/overlapping surfaces) → **S4 + OCCT fallback**; not consumed, reported.
- [ ] **Branch-point / self-intersecting** seams (singular multi-branch crossings) →
  **S4 + OCCT fallback**.
- [ ] **Freeform** (NURBS / Bézier) operand faces → deferred (S5-a is elementary only);
  OCCT fallback until a later S5 stage.
- [ ] **Curved blends (#6)** and **curved wrap-emboss (#7)** compose on the S5 curved
  boolean — out of this change.
