# Tasks — moat-gs2-section-curves (MOAT M-GS, GS2)

Order: substrate build → per-face section edges → loop assembly → service + cap →
additive facade → host analytic gate (A) → sim native-vs-OCCT gate (B) → docs, or
HONEST DECLINE. All new native code is header-only, OCCT-free, host-buildable
(`clang++ -std=c++20`), namespace `cybercad::native::section`. GS2 CONSUMES M1 SSI
+ topology + the B4 cap synthesis READ-ONLY; it does NOT modify `boolean/` or
`ssi/` (the sibling M2-FUSE workflow owns those). `cc_*` additive-only; no existing
signature changes. No tolerance weakened; a measured decline (tangent plane /
non-closing / hard-freeform) is a first-class outcome — a wrong or open section is
NEVER emitted.

## 0. Substrate

- [ ] 0.1 `bash scripts/build-numsci.sh iossim && bash scripts/build-numsci.sh host`;
      export `CYBERCAD_NUMSCI_DIR` to `build-numsci/host` (host gate) /
      `build-numsci/iossim` (sim gate).
- [ ] 0.2 Confirm the consumed parts build and are on the verified path:
      `ssi::intersect_surfaces` (plane∩{plane,cyl,cone,sphere} closed-form),
      `ssi::marching` WLine (plane∩freeform), `topology::surfaceOf`/`pcurveOf`/
      face-loop iteration, and the B4 `boolean/half_space_cut.h` cross-section-cap
      path — all READ-ONLY from `section/`.

## 1. Per-face section edges (`src/native/section/section_edges.h`, OCCT-free)

- [ ] 1.1 For an ANALYTIC face route `(cutPlane, surfaceOf(F))` through
      `ssi::intersect_surfaces`; take the native conic branch(es).
- [ ] 1.2 CLIP each conic to `F`'s trim via `pcurveOf` (inside outer loop, outside
      holes) → bounded on-face arcs; verify EVERY sample on-plane AND on-face.
- [ ] 1.3 For a FREEFORM face route through the M1 marcher; clip the WLine polyline
      to trim the same way; `NearTangent`/non-converged → DECLINE.
- [ ] 1.4 Tangent / degenerate analytic result (Point / double-line) → DECLINE.
- [ ] 1.5 Emit `SectionEdge{curveKind, samples, face, endpointA, endpointB}`.

## 2. Loop assembly (`src/native/section/loop_assembly.h`, OCCT-free)

- [ ] 2.1 Dedup endpoints within `kLinearEps`, preferring topology shared-vertex
      identity; build the endpoint→edges adjacency.
- [ ] 2.2 Walk edges by shared free endpoint into CLOSED cycles; stuck / ambiguous
      branch → typed `NotClosed` / `AmbiguousBranch` DECLINE.
- [ ] 2.3 Orient each loop CCW about the plane normal (outer vs holes by signed
      area).

## 3. Service + optional cap (`src/native/section/section.h`, OCCT-free)

- [ ] 3.1 `sectionPlane(solid, plane, wantCap)`: section_edges → loop_assembly →
      per-loop VERIFY (on-plane, on-face, closed) → optional cap + area.
- [ ] 3.2 Cap via the B4 cross-section-cap path (outer minus holes); area by the
      in-plane shoelace. Nested-beyond-one-level / self-intersecting → DECLINE.
- [ ] 3.3 Return `Section{loops, loopCount, allClosed, cappedArea, totalEdgeLength}`
      or typed `Decline{reason}`.

## 4. Additive facade (`include/cybercadkernel/cc_kernel.h`, `src/facade/cc_kernel.cpp`)

- [ ] 4.1 Add `CCSectionLoop` / `CCSection` PODs + `cc_section_plane(...)` +
      `cc_section_free(...)` — APPENDED, no existing signature changed.
- [ ] 4.2 Native build → `section::sectionPlane`; engine build → `BRepAlgoAPI_Section`
      (+ `BRepGProp` cap) behind the SAME accessor (the sim oracle path).
- [ ] 4.3 DECLINE → empty `CCSection` (loops null, count 0) + `cc_last_error`; never
      a partial section.

## 5. GATE A — host analytic (`tests/native/test_native_section.cpp`, no OCCT)

- [ ] 5.1 Per-loop invariants: every point on the cut plane (`|n·(P−O)|≤tol`) AND
      on some solid face (residual ≤ tol) AND loop closed.
- [ ] 5.2 Enclosed area vs closed form: box planar section = `w·h`; cylinder axial
      section = `2R·H`; cylinder cross-section ⟂ axis = `πR²`; sphere great circle
      = `πR²`.
- [ ] 5.3 Declines asserted: plane tangent to a cylinder/sphere; a non-closing
      section; a hard-freeform face the marcher stops on — each returns a typed
      decline, NOT a wrong/open section.
- [ ] 5.4 Freeform ACCEPT fixture: the landed bowl-lidded convex-quad prism —
      section loop closed + on both surfaces.

## 6. GATE B — sim native-vs-OCCT (`tests/sim/native_section_parity.mm`, booted sim)

- [ ] 6.1 Reconstruct each fixture in OCCT; assert native TOTAL section-edge length,
      LOOP COUNT and CLOSED-NESS match `BRepAlgoAPI_Section`.
- [ ] 6.2 Assert native `cappedArea` matches `BRepGProp` on the section face.
- [ ] 6.3 Fixed tolerances, never widened; harness `main()` + `run-sim-suite.sh`
      entry.

## 7. Quality + docs

- [ ] 7.1 cognitive-complexity pass over `src/native/section/` (backend band ≤ 15);
      flag any genuinely irreducible function.
- [ ] 7.2 Update `openspec/MOAT-ROADMAP.md`: mark GS2 NATIVE landed with the three
      documented declines (tangent / non-closing / hard-freeform).
- [ ] 7.3 `openspec validate moat-gs2-section-curves --strict` passes.
