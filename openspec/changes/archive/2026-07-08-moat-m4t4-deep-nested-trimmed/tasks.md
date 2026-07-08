# Tasks — moat-m4t4-deep-nested-trimmed

Order: diagnosis lock (3-level guard) → `RECTANGULAR_TRIMMED_SURFACE` unwrap → host
gate → sim gate → decline coverage. All new reader code stays OCCT-free
(`src/native/**`, `clang++ -std=c++20`, namespace `cybercad::native::exchange`), the
`cc_*` ABI is unchanged (additive reader behaviour only), and the landed single/2-level
assembly + `EDGE_LOOP`-trim import paths stay BYTE-IDENTICAL. Keep per-function
cognitive complexity green (factor the rect-box validation into a small helper).

## 1. Substrate

- [x] 1.1 `bash scripts/build-numsci.sh iossim && bash scripts/build-numsci.sh host`,
      then `export CYBERCAD_NUMSCI_DIR=…/build-numsci/{iossim|host}`.
- [x] 1.2 Baseline: build + run the landed host reader suite and sim parity gate GREEN
      before any edit (diff against this to prove byte-identity of landed paths).

## 2. Diagnosis lock — deep-nested N-level (already handled by `composeChain`)

- [x] 2.1 Confirm by reading `composeChain` (step_reader.cpp:711) that the leaf→root
      walk is depth-unbounded and that no constant caps chain length; record the
      finding in `design.md` (done).
- [x] 2.2 HOST ANALYTIC 3-level fixture in `tests/native/test_native_step_reader.cpp`:
      three rigid frames `A,B,C`; CDSRs leaf→sub2 (`T₃=A`), sub2→sub1 (`T₂=B`),
      sub1→root (`T₁=C`). Assert the imported leaf's world box equals the leaf geometry
      mapped by the INDEPENDENT matrix product `C·B·A` (computed in the test, not via
      `composeChain`). Also assert a 4-level chain composes (guard against a latent
      depth cap).
- [x] 2.3 SIM parity in `tests/sim/native_step_import_parity.mm`: the same 3-level
      buffer through `STEPControl_Reader` + `BRepMesh` matches native on count / volume
      / bbox / centroid / topology.

## 3. `RECTANGULAR_TRIMMED_SURFACE` unwrap (the chosen gap)

- [x] 3.1 Add `rectangularTrimmedSurface(const Record& r)` to `step_reader.cpp`:
      validate arg shape `('',#basis,u1,u2,v1,v2,usense,vsense)`; DECLINE (`nullopt`)
      if the basis arg is not a ref, if any of `u1,u2,v1,v2` is non-finite, or if
      `u2 ≤ u1` or `v2 ≤ v1`; otherwise `return surface(#basis)` (recurse — inherits the
      basis's own supported/decline verdict). Factor the rect-box check into a small
      static helper.
- [x] 3.2 Wire one dispatch line in `surface(id)`:
      `if (r->keyword == "RECTANGULAR_TRIMMED_SURFACE") return rectangularTrimmedSurface(*r);`
      placed with the other keyword arms (before the final `return nullopt`). No other
      arm changes.
- [x] 3.3 Confirm the `ADVANCED_FACE` path is untouched: the reduced basis surface flows
      through `advancedFace` / `buildFaceWithPCurves` exactly as a directly-referenced
      basis; the `EDGE_LOOP` remains the trim; the torus-with-real-trim and childless-
      bound guards fire unchanged (no boundary synthesis added).

## 4. HOST gate — analytic, no OCCT

- [x] 4.1 Round-trip equivalence: a solid whose ONE face surface is a
      `RECTANGULAR_TRIMMED_SURFACE` over a `PLANE` imports to the SAME native `Shape`
      (volume, area, topology) as the identical file referencing the basis `PLANE`
      directly. Independent equivalence — no OCCT.
- [x] 4.2 Cylinder basis: a `RECTANGULAR_TRIMMED_SURFACE` over a `CYLINDRICAL_SURFACE`
      imports watertight; the imported face's sampled corner points `S(uᵢ,vⱼ)` match the
      closed-form analytic cylinder within scale-relative tolerance (never widened).

## 5. SIM gate — native vs OCCT oracle

- [x] 5.1 In `tests/sim/native_step_import_parity.mm`: the rect-trimmed-plane and
      rect-trimmed-cylinder solids each match `STEPControl_Reader` + `BRepMesh` on
      triangle-envelope count, enclosed volume, bbox, centroid, and watertight topology.

## 6. Honest-decline coverage (first-class outcomes)

- [x] 6.1 A `RECTANGULAR_TRIMMED_SURFACE` over an UNSUPPORTED basis (e.g. an offset /
      swept surface keyword `surface()` does not map) → import returns NULL (DECLINE),
      round-trips through OCCT unchanged, tolerance NOT widened.
- [x] 6.2 A `RECTANGULAR_TRIMMED_SURFACE` with an INVERTED / degenerate rect box
      (`u2 ≤ u1` or a non-finite bound) → DECLINE (NULL), no fabricated face.
- [x] 6.3 Shared-sub-assembly (one child SR, two distinct parents) STILL declines
      (unchanged `parentEdges` guard) — assert the landed decline test stays green.

## 7. Regression + docs

- [x] 7.1 Full host reader suite + sim parity GREEN; diff the landed-path results
      against the §1.2 baseline to prove BYTE-IDENTITY (single/2-level assembly +
      `EDGE_LOOP`-trim unchanged).
- [x] 7.2 Verify `src/native/**` has 0 OCCT includes (`grep -R` for OCCT headers) and
      the `cc_*` ABI is unchanged (no header signature diff).
- [x] 7.3 `openspec validate moat-m4t4-deep-nested-trimmed --strict` passes; archive on
      completion.
