# Tasks — moat-hlrc-curved-silhouette (quadric silhouette HLR: cylinder + sphere)

Order: baseline capture → OCCT-free silhouette helper (cylinder + sphere, with
decline guards) → engine per-face router → host analytic gate → sim parity gate
→ zero-regression proof → docs, or HONEST DECLINE. All new native code stays
OCCT-free and host-buildable (`clang++ -std=c++20`), namespace
`cybercad::native::drafting`. **No `cc_hlr_project` signature or POD change** —
COVERAGE extension only. `orthographic_hlr.h` stays BYTE-IDENTICAL for
polyhedral inputs (13/13 polyhedral parity unchanged). Cone/torus/partial-
quadric/freeform stay DECLINED; a decline is a first-class outcome and no
fabricated outline or wrong classification is ever emitted.

## 0. Baseline (capture BEFORE touching HLR)

- [ ] 0.1 Build numsci (`scripts/build-numsci.sh iossim` + `host`), export
      `CYBERCAD_NUMSCI_DIR`, build host + sim; record GREEN baseline: polyhedral
      HLR host analytic (box 9v+3h, no-occluder 12v, half-occluded split,
      length conservation) and the 13/13 sim polyhedral parity.
- [ ] 0.2 Snapshot the polyhedral `projectOrthographic` output (visible/hidden
      counts + total length) for box iso, box oblique, tri-prism, L-prism — the
      BYTE-IDENTICAL reference asserted unchanged in §6.
- [ ] 0.3 Author the OCCT `HLRBRep_Algo` oracle for a cylinder (side-on +
      oblique), a sphere, a cone and a torus in the sim harness (counts, total
      length, endpoint samples of the visible/hidden compounds).

## 1. OCCT-free silhouette helper (`src/native/drafting/silhouette.h`)

- [ ] 1.1 Add `silhouette.h` (namespace `cybercad::native::drafting`,
      header-only, includes ONLY `native/math`): `SilhouetteParams`,
      `SilhouetteResult{traced, polylines}`, `FaceSurfaceView` adapter, and
      `traceSilhouette(face, view, prm)`.
- [ ] 1.2 `sphereGreatCircle`: emit the great circle `C + r(cos t·right +
      sin t·trueUp)` discretized to `chordDeflection` as a closed world
      polyline; drawing-plane projection is an exact circle radius `r`.
- [ ] 1.3 `cylinderGenerators`: `θ* = atan2(−(X·d),(Y·d))` and `θ*+π`, each
      swept `s∈[0,h]` (the face `v`-range) → two straight world segments.
- [ ] 1.4 Degenerate/axis-parallel guard: if `‖d − (d·A)A‖ < axisParallelTol`
      emit NO generators (cap edge is the outline — correct, not a decline); in
      the near-threshold band return `traced=false` (decline, never a
      near-degenerate generator).
- [ ] 1.5 Trim-window guard: a generator whose `θ*`/axial span leaves the face's
      `(u,v)` trim window (partial/trimmed quadric) → return `traced=false`
      (decline, no clipped-generator guess).
- [ ] 1.6 Explicit `traced=false` for `Cone`, `Torus`, `BSpline`, `Bezier`.
- [ ] 1.7 Register `silhouette.h` in `native_drafting.h` umbrella; confirm
      `src/native/drafting/**` has ZERO OCCT includes; compiles host + arm64
      iOS simulator.

## 2. Engine per-face router (`src/engine/native/native_engine.cpp::hlr_project`)

- [ ] 2.1 Replace the blanket non-planar decline (~L1498) with a per-face pass:
      `Plane` → unchanged; `Cylinder`/`Sphere` → build `FaceSurfaceView`, call
      `traceSilhouette`, `return make_error(...)` if `!traced`, else append
      polylines to the world-edge set; any other curved kind → decline.
- [ ] 2.2 Feed silhouette polylines through the SAME edge de-dup + occluder +
      `projectOrthographic` path (no new occlusion code); cap circles flow
      through the existing `cache.discretize` loop.
- [ ] 2.3 Confirm `cc_hlr_project` + `CCDrawing`/`CCDrawingSegment`/`CCHlrOptions`
      are UNCHANGED (additive COVERAGE only); a cone/torus/freeform body still
      returns an empty `CCDrawing` with `cc_last_error` set.

## 3. GATE (a) — HOST ANALYTIC (`tests/native/test_native_drafting.cpp`, no OCCT)

- [ ] 3.1 Sphere r at C, arbitrary view → outline is a closed circle radius `r`
      at `project(C)` within `1e-9`; fully visible; total length `2πr`; 0 hidden.
- [ ] 3.2 Cylinder r,h side-on (axis ⟂ view) → exactly two VISIBLE generator
      segments length `h` separated by `2r`; the two cap circles split into a
      visible + a hidden arc; total projected length conserved (`1e-6`).
- [ ] 3.3 Axis-parallel cylinder view → no generators, cap edge is the outline
      (NOT a decline); near-threshold band declines.
- [ ] 3.4 Cone / torus / freeform / partial-quadric-out-of-window → declined
      with the documented error; NO mesh-edge outline emitted.

## 4. GATE (b) — SIM native-vs-OCCT (`tests/sim/native_hlr_parity.mm`)

- [ ] 4.1 Drive `cc_hlr_project` under `cc_set_engine(1)` (native) vs
      `cc_set_engine(0)` (OCCT `HLRBRep_Algo`) for cylinder (side-on + oblique)
      and sphere built identically under each engine.
- [ ] 4.2 Assert match on visible COUNT, hidden COUNT, total projected LENGTH
      (rel ≤ `1e-4`), endpoint PARTITION (each native segment covered by an OCCT
      segment of the same class within `1e-5`, and vice versa). NO tolerance
      widening.
- [ ] 4.3 Assert a cone AND a torus body are DECLINED (empty `CCDrawing` +
      `cc_last_error`), not compared.

## 5. Zero-regression proof (polyhedral path untouched)

- [ ] 5.1 Diff `orthographic_hlr.h` vs `main` → byte-identical (the file is NOT
      edited; only `silhouette.h` is added and the engine router changed).
- [ ] 5.2 Re-run §0.2 polyhedral snapshots → identical counts + lengths; the
      13/13 polyhedral sim parity is unchanged.
- [ ] 5.3 Cognitive complexity: `traceSilhouette` + emitters flagged in the
      geometry band (≤ 25); the engine router `switch` ≤ 15.

## 6. Docs

- [ ] 6.1 Update `cc_kernel.h` `cc_hlr_project` doc block: scope now includes
      cylinder + sphere quadric silhouettes; cone/torus/partial-quadric/freeform
      still DECLINED. Signature UNCHANGED.
- [ ] 6.2 Note the new `silhouette.h` in `native_drafting.h` and the drafting
      slice status.

## HONEST-DECLINE fallback

If cylinder/sphere silhouettes cannot be traced to the closed-form host oracle
AND matched to OCCT `HLRBRep_Algo` within the stated tolerances at a
caller-chosen deflection, the silhouette branch is reverted and the affected
quadric keeps the honest decline (the body stays on OCCT). No fabricated
outline, no weakened tolerance, no wrong visible/hidden classification is ever
emitted.
