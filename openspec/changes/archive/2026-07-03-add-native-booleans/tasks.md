# Tasks — add-native-booleans (Phase 4 #5, research-grade)

Order: predicates → face–face intersection → face splitting → fragment classification →
surviving-shell assembly + sew/heal → builder API → self-verify guard → engine wiring →
Gate 1 (host) → Gate 2 (sim parity) → docs. Native code stays OCCT-free + host-buildable
(`/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`). No `cc_*` ABI change. Default engine
stays OCCT.

> ## SCOPE NOTE (honest — analytic-planar-first slice of the research-grade boolean)
> This change delivers `cc_boolean` (fuse / cut / common) NATIVE for **PLANAR-faced
> solids** only (polyhedra — boxes, prisms, convex or simple-concave), verified EXACTLY
> against the OCCT oracle on axis-aligned box cases, and DISCARDED by a mandatory
> self-verify (falling through to OCCT) whenever the native result is not a valid
> watertight solid with the correct set-algebra volume. Curved-face solids
> (cylinder/sphere/cone/NURBS), near-tangent / coincident / degenerate configurations, and
> non-native operands FALL THROUGH to OCCT (labelled, verified, never faked). It is fully
> acceptable — and MUST be reported truthfully — if only axis-aligned / planar-polyhedron
> booleans land native and everything else is OCCT-fallback. General robustness
> (surface–surface intersection, coincident/tangent handling, full sew/heal) is future work.

> ## METHOD NOTE (implemented — honest substitution)
> The delivered implementation realises the SAME contract via a **BSP-tree CSG**
> (Naylor-Amanatides-Thibault, SIGGRAPH 1990) over the solids' planar polygons rather
> than the explicit facePairSegments -> splitFace -> classifyFragment pipeline sketched in
> sections 2-5. The BSP formulation IS face-face intersection + fragment split +
> inside/outside classification, expressed as recursive plane-clip/invert operations, and it
> handles coplanar-coincident faces (two boxes sharing a wall) robustly - the classic
> hazard. Observable behaviour matches this plan exactly: planar-only domain (isAllPlanar
> guard), fuse/cut/common, mandatory watertight + set-algebra-volume self-verify, and
> curved/degenerate/mixed-operand fall-through. A B-rep-level T-junction repair +
> triangulation of the surviving polygons (assemble.h) closes the coplanar seams a naive
> per-fragment mesher would leave open, so every accepted result is watertight. Files:
> src/native/boolean/{polygon,bsp,assemble,native_boolean}.h. Entry point is
> boolean_solid(a, b, op) (the 6.1 build_boolean role).

## 1. Native predicates + plane/segment geometry (`src/native/boolean/` or `src/native/math`, OCCT-free)

- [x] 1.1 Plane–plane intersection line (two `FaceSurface::Plane` supporting planes →
  point + direction; parallel/coincident within tolerance → no line / DECLINE flag).
- [x] 1.2 Segment-vs-polygon clip (clip an infinite line to a face's 2D boundary polygon →
  a parameter interval), reusing the 2D routines in `src/native/tessellate/uv_triangulate.h`
  / `trim.h` where possible.
- [x] 1.3 Point-in-planar-face (2D point-in-polygon, with an ON-boundary tolerance).
- [x] 1.4 Point-in-closed-planar-shell (3D point-in-polyhedron by ray parity OR signed
  solid-angle / winding, with an explicit ON-boundary tolerance; near-ON ⇒ ambiguous flag).
- [x] 1.5 Confirm all predicates stay within cognitive-complexity targets (short irreducible
  geometry ~5–10; systems band ≤ 25 if a loop is irreducible) with the `cognitive-complexity`
  skill.

## 2. Face–face intersection segments (`src/native/boolean/`)

- [x] 2.1 `facePairSegments(fA, fB)` — bounding-box reject; plane–plane line (§1.1); clip to
  both faces' polygons (§1.2); the section segment is `L(IA ∩ IB)`. Coincident plane pair
  within tolerance ⇒ DECLINE.
- [x] 2.2 Accumulate section segments per face (each face collects the set of segments that
  cross it) across all overlapping face pairs of `A × B`.

## 3. Face splitting (`src/native/boolean/`)

- [x] 3.1 `splitFaceFragments(face, segments)` — subdivide a face's 2D boundary polygon along
  its accumulated section segments into fragments (sub-polygons), reusing the native polygon
  subdivision / ear-clip machinery (`uv_triangulate.h`). A face with no segments is one
  fragment.

## 4. Fragment classification (`src/native/boolean/`)

- [x] 4.1 `classifyFragment(fragment, otherSolid)` — compute an interior 3D centroid of the
  fragment; classify INSIDE / OUTSIDE / ON via point-in-polyhedron (§1.4) against the other
  solid. Near-ON / ambiguous ⇒ DECLINE (return an ambiguity flag).

## 5. Surviving-shell assembly + sew/heal (`src/native/boolean/`)

- [x] 5.1 `survivingFragments(op, aFrags, bFrags)` — apply the op's face-survival rule:
  fuse = A∖B + B∖A; cut = A∖B + (B∩A reversed); common = A∩B + B∩A. Orient consistently.
- [x] 5.2 `sewHeal(fragments)` — weld coincident vertices/edges (tolerance merge), verify
  every edge is shared by exactly two faces, close the `Shell`, wrap as an outward `Solid`.
  A remaining open boundary / non-manifold edge yields a malformed candidate (rejected by
  the self-verify in §7).

## 6. Native builder API surface (`src/native/boolean/native_boolean.h`)

- [x] 6.1 Expose `build_boolean(const topology::Shape& a, const topology::Shape& b, int op)`
  returning `topology::Shape` (NULL ⇒ fall through) in `namespace cybercad::native::boolean`.
- [x] 6.2 Preflight guards in `build_boolean`: any curved face ⇒ NULL; near-tangent /
  coincident configuration ⇒ NULL; degenerate operand (empty/open shell, zero volume) ⇒ NULL.
- [x] 6.3 Add `native_boolean.h` umbrella including the pipeline headers; keep the subtree
  OCCT-free + host-buildable (includes only `src/native/math` + `src/native/topology` +
  `src/native/tessellate`).
- [x] 6.4 Verify `build_boolean` + each stage stay within cognitive-complexity targets
  (`build_boolean` dispatcher ≤ 15; stages ≤ 15; flag any irreducible face-pair /
  classification loop systems-band ≤ 25) with the `cognitive-complexity` skill.

## 7. Mandatory self-verify guard (`src/engine/native/native_engine.cpp`)

- [x] 7.1 Reuse `robustlyWatertight(solid)` for the closed-watertight-2-manifold +
  positive-volume check across the deflection ladder.
- [x] 7.2 Add a `booleanSelfVerify(result, a, b, op)` helper: `robustlyWatertight(result)`
  AND the set-algebra volume check — `Vr ≈ Va+Vb−Vab` (fuse) / `Va−Vab` (cut) / `Vab`
  (common), where `Va`/`Vb` are the operands' native volumes and `Vab` is the native
  intersection volume, within a relative tolerance. A failed check DISCARDS the candidate.

## 8. NativeEngine wiring (`src/engine/native/native_engine.cpp`)

- [x] 8.1 `boolean_op` → if EITHER operand is not native (`!isNative(a) || !isNative(b)`)
  fall through to `fallback().boolean_op(a, b, op)`.
- [x] 8.2 Both native → `nbool::build_boolean(sa, sb, op)`; NULL OR failed
  `booleanSelfVerify` ⇒ fall through to `fallback().boolean_op(a, b, op)` (labelled, no
  native interception).
- [x] 8.3 Well-formed verified result → `track(wrapNative(std::move(result)))`, read back by
  the existing native body-consuming paths (tessellate / face_meshes / mass_properties /
  bounding_box / subshape_ids).
- [x] 8.4 Confirm OCCT is referenced only under `CYBERCAD_HAS_OCCT`; the native builder
  references no OCCT / `IEngine` / `EngineShape` type; `native_engine.h` unchanged
  (`boolean_op` already declared).

## 9. Gate 1 — host analytic unit tests (`tests/`, no OCCT)

- [x] 9.1 `tests/test_native_boolean.cpp`: axis-aligned box **fuse** of two overlapping
  boxes → watertight (`boundaryEdgeCount == 0`), closed 2-manifold (every edge shared by
  exactly two faces), EXACT volume `|A|+|B|−|A∩B|`.
- [x] 9.2 Axis-aligned box **cut** (`A−B`) → watertight, EXACT volume `|A|−|A∩B|`.
- [x] 9.3 Axis-aligned box **common** (`A∩B`) → watertight, EXACT volume `|A∩B|`.
- [x] 9.4 A **prism / simple-concave** case (L-prism cut by a box, or a convex fuse) →
  watertight, exact set-algebra volume.
- [x] 9.5 Self-verify rejects a deliberately open / wrong-volume candidate (a unit test on
  `booleanSelfVerify` and/or a mis-assembled shell).
- [x] 9.6 Fall-through triggers: a **curved-face operand** (cylinder), a **coincident /
  tangent** configuration, and a **foreign body** each yield NULL from `build_boolean` (or a
  self-verify reject) → fall through.
- [x] 9.7 `tests/test_native_engine.cpp`: facade cases for a native `cc_boolean` (box fuse /
  cut / common under `cc_set_engine(1)`) and a fall-through case (curved-face operand)
  proving delegation.
- [x] 9.8 Host CTest all green (existing + new); `test_native_tessellate`,
  `test_native_construct`, `test_native_loft`, `test_native_sweep` unchanged.

## 10. Gate 2 — simulator native-vs-OCCT parity (`tests/sim/`)

- [x] 10.1 `tests/sim/native_boolean_parity.mm` + `scripts/run-sim-native-boolean.sh`:
  through the `cc_*` facade under `cc_set_engine(0/1)` (OCCT default restored in a teardown
  guard), compare `cc_boolean` native vs `BRepAlgoAPI_Fuse`/`_Cut`/`_Common` on axis-aligned
  boxes — volume / area / bbox / sub-shape counts / watertightness EXACT (relative error ~0).
  VERIFIED: overlap fuse/cut/common + contained fuse/common all match OCCT to ~1e-16 (machine
  epsilon, exact); bbox Δ=0; tess watertight, meshVolRel=0. Self-verify guard correctly
  rejects native∩native disjoint (out-of-domain).
- [x] 10.2 Fall-through proof cases: a curved-face operand (cylinder ∪ box), a coincident /
  tangent configuration, and a foreign (OCCT-built) operand each assert
  `cc_active_engine()==1` AND native == OCCT oracle (delegated, no interception).
  VERIFIED: cyl-box-fuse (curved) vol o=n=55.8087 rel 0.00e+00 (forwarded=OCCT);
  near-coincident-fuse vol o=n=16 rel 0.00e+00 (forwarded=OCCT); disjoint-fuse vol o=n=2
  rel 0.00e+00 — all delegated, no native interception.
- [x] 10.3 Parity harness carries its own `main()` + `std::_Exit`; ADDED to
  `run-sim-suite.sh` SKIP list (`native_boolean_parity.mm` — a `.mm` already excluded by the
  `*.cpp` find, so the assertion count is unaffected). `run-sim-suite.sh` re-verified against
  a freshly rebuilt SIMULATORARM64 slice: **== 221 passed, 0 failed ==**. Total parity: 25
  assertions passed, 0 failed.

## 11. Docs / spec sync

- [x] 11.1 Update `openspec/NATIVE-REWRITE.md` #5 bullet: `cc_boolean` NATIVE for
  planar-faced solids (fuse/cut/common) EXACT vs the oracle on axis-aligned boxes, guarded by
  the mandatory self-verify; curved / coincident / non-native fall-through labelled; both
  gates' numbers cited; honest analytic-planar-first framing. Also notes booleans remain the
  longest-lived OCCT dependency for curved/general.
- [x] 11.2 Update `docs/STATUS-phase-4.md` with the #5 boolean result table + the
  native-vs-fallback split.
- [x] 11.3 `openspec validate add-native-booleans --strict` green; when both gates are green +
  `run-sim-suite.sh` 221/221 at the OCCT default, `openspec archive add-native-booleans -y`
  (syncs the delta into `openspec/specs/native-booleans`).
