# Tasks — moat-m5t-healing-tail (MOAT M5 healing tail)

Order: substrate → additive option → cap_hole.h multi-loop generalization →
heal.cpp opt-in branch → host analytic gate (A) → sim native-vs-OCCT gate (B) →
quality + docs, or HONEST DECLINE. All new native code is header-only / additive,
OCCT-free, host-buildable (`clang++ -std=c++20`), namespace
`cybercad::native::heal`. `cc_*` additive-only (in fact untouched); no existing
signature changes; the tessellator is not modified. No tolerance weakened; a
declined multi-hole set (branching / non-planar / self-intersecting) is a
first-class `Unhealed{OpenShell}` with the input UNCHANGED — never a partial closure.

## 0. Substrate

- [x] 0.1 `bash scripts/build-numsci.sh iossim && bash scripts/build-numsci.sh host`;
      export `CYBERCAD_NUMSCI_DIR` to `build-numsci/host` (host gate) /
      `build-numsci/iossim` (sim gate).
- [x] 0.2 Confirm the landed heal gate is green (18/18 host cases) and the landed
      single-hole cap path (`capPlanarHole`, `traceSingleLoop`, the four detail
      layers) is the byte-identical baseline the new pass must not disturb.

## 1. Additive option (`src/native/heal/heal_result.h`)

- [x] 1.1 Add `HealOptions.capMultiplePlanarHoles` (default `false`) with the bounded,
      superset-of-single-hole contract documented inline. No other field changes;
      `nCappedFaces` / `maxCapPlanarityDev` already exist and are reused.

## 2. Multi-loop cap generalization (`src/native/heal/cap_hole.h`, additive, OCCT-free)

- [x] 2.1 `traceAllLoops(BoundaryGraph)`: require EVERY boundary vertex degree == 2
      (a branching degree-4 corner from two ADJACENT missing faces ⇒ empty ⇒ decline
      whole); walk each unvisited component into a closed cycle; a non-closing
      component ⇒ empty. Return the vector of disjoint simple cycles.
- [x] 2.2 `struct MultiCapResult { std::vector<FaceLoop> caps; bool declined; double
      planarityDev; }` and `capPlanarHoles(sr, tol)`: for each cycle run the UNCHANGED
      `bestFitPlane` / `maxPlaneDeviation ≤ tol` / `isSimplePolygon` layers; if ALL
      pass emit one cap `FaceLoop` per loop on its EXISTING shared nodes; if ANY fails
      set `declined = true` and emit no caps (decline the whole set).
- [x] 2.3 Leave the landed `capPlanarHole` (single) and all four detail helpers
      BYTE-IDENTICAL; `capPlanarHoles` is new code that reuses them.

## 3. Opt-in branch (`src/native/heal/heal.cpp`, additive, guarded)

- [x] 3.1 Before the landed single-hole `capPlanarHoles`-guarded branch, add a
      `capMultiplePlanarHoles`-guarded branch that appends every emitted cap to `work`,
      re-sews, and updates `nMergedVerts` / `nMergedEdges` / `nCappedFaces` /
      `maxCapPlanarityDev`. Route so the landed single-hole branch runs ONLY when
      `capMultiplePlanarHoles == false` (byte-identical for every existing caller).
- [x] 3.2 Leave the surviving-boundary honest-out, orientation flood-fill, global
      outward sign flip, assemble, and mandatory self-verify UNCHANGED — self-verify
      stays authoritative. No new `UnhealedReason` (declined set stays `OpenShell`).

## 4. GATE A — host analytic (`tests/native/test_native_heal.cpp`, no OCCT)

- [x] 4.1 ACCEPT: unit cube missing two OPPOSITE faces, `capMultiplePlanarHoles == true`
      → `Healed`, `watertight`, `valid`, `nCappedFaces == 2`, `maxCapPlanarityDev ≤ tol`,
      `maxResidualGap == 0`, tessellated `enclosedVolume == 1.0` (analytic, no OCCT).
- [x] 4.2 DECLINE (adjacent): unit cube missing two ADJACENT faces → decline,
      `nCappedFaces == 0`, input UNCHANGED. IMPLEMENTATION NOTE (measured, honest): on a
      cube two adjacent missing faces do NOT create a degree-4 branch — they ORPHAN the
      two exclusively-shared corners (each left on one face), so the sew measures a
      residual ≈ 1 and the honest-out reports `GapBeyondTolerance` (not `OpenShell`); the
      wrap-around boundary is also non-planar. Either way no cap, input unchanged. The
      pure branching (degree != 2) guard is the SAME invariant `traceSingleLoop` already
      enforces and `traceAllLoops` reuses; a genuine degree != 2 fixture from simple cube
      removal is not constructible, so it is covered by the shared invariant, not a
      contrived non-manifold fin.
- [x] 4.3 DECLINE (mixed planarity): a two-hole set with one loop lifted out of plane
      → `Unhealed{OpenShell}`, `nCappedFaces == 0`, input UNCHANGED.
- [x] 4.4 DECLINE (self-intersecting): the multi-cap pass reuses the UNCHANGED
      `isSimplePolygon` layer per loop. IMPLEMENTATION NOTE: a self-crossing boundary is
      not cleanly constructible from cube-face removal without breaking corner coincidence
      (a cube hole is always a convex square), so this is proven by a DIRECT unit test of
      the reused layer — `heal_cap_self_intersecting_layer_rejects` asserts a planar bowtie
      is refused and a convex square accepted. Any refused loop declines the whole set.
- [x] 4.5 DEFAULT-OFF byte-identical: the same two-opposite-hole soup with
      `capMultiplePlanarHoles == false` declines exactly as `heal_cap_two_holes_declines`;
      re-run the landed 18-case suite unchanged.

## 5. GATE B — sim native-vs-OCCT (`tests/sim/native_heal_parity.mm`, booted sim)

- [x] 5.1 Build the two-opposite-hole cube as an OCCT open shell; reference-cap each
      hole via `BRepBuilderAPI_MakeFace(gp_Pln, freeBoundaryWire)` + `ShapeFix_Shell` /
      `ShapeFix_Solid`. Assert native (`capMultiplePlanarHoles == true`) matches OCCT:
      same watertight closed solid, same `enclosedVolume` vs `BRepGProp` within tol.
- [x] 5.2 Decline parity: the branching two-adjacent-hole fixture → native `Unhealed`
      AND the engine falls through to OCCT `ShapeFix` (documented asymmetry note where
      OCCT tolerates a near-planar wire, mirroring the landed single-hole note).
- [x] 5.3 Fixed tolerances, never widened; run via `scripts/run-sim-native-heal.sh`.

## 6. Quality + docs

- [x] 6.1 cognitive-complexity pass over the new `cap_hole.h` functions (backend band
      ≤ 15); flag any genuinely irreducible function instead of mangling it.
- [x] 6.2 Update `openspec/MOAT-ROADMAP.md`: mark M5-tail multi-hole planar cap landed
      with the three documented declines (branching / non-planar / self-intersecting),
      and the two out-of-scope tail defects (curved cap, wire untangling) still OCCT.
- [x] 6.3 `openspec validate moat-m5t-healing-tail --strict` passes.
