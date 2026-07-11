# Tasks — nurbs-gordon-surface

## 1. Foundation — module + curve network + consistency check
- [x] 1.1 Create `src/native/math/bspline_gordon.h` — `CurveNetwork`, `NetworkCheck`,
      `GordonResult`, and the `verifyNetwork` / `gordonSurface` declarations. Reuses the Layer-1
      `BsplineCurveData` / `BsplineSurfaceData` types. Add to `native_math.h`.
- [x] 1.2 `verifyNetwork` — non-rational + well-formed guard for both families; strictly-increasing
      station params (a proper monotone grid); grid consistency `C_k(u_l) == D_l(v_k)` within `tol`
      for every `(k,l)`; report `maxGridError` + the averaged `K×L` grid; decline honestly with a
      reason on any violation (`ok=false`).

## 2. Boolean sum (§10.5)
- [x] 2.1 `interpFamilyAcross` — interpolate ONE compatible family across the transversal direction
      at PRESCRIBED params (not chord-length): a shared `M×M` collocation matrix, one `lin_solve` per
      coordinate per along-index; assemble the tensor surface (curve shape along, interpolation
      across), transposing when the curve shape maps to V. Singular → decline.
- [x] 2.2 `interpGrid` — tensor-product interpolation of the `K×L` grid at `(uParams, vParams)`:
      interpolate each v-station row across u, then each u-control column across v (two `lin_solve`
      stages). Row-major U-outer output. Singular → decline.
- [x] 2.3 `unifyDirection` (per direction) — raise two surfaces to the common max degree
      (`elevateDegreeSurface`) then merge to the union knot vector (`refineKnotSurface`), both exact
      Layer-1 ops (no geometry drift). Fold `S_u`/`S_v`/`T` pairwise until all three share one basis.
- [x] 2.4 `gordonSurface` — verify → make each family compatible (`makeSectionsCompatible`) → build
      `S_u`, `S_v`, `T` at the prescribed params → unify to a common basis → form the Gordon net
      `poles(S_u) + poles(S_v) − poles(T)`. Non-rational; `K,L ≥ 2`; decline honestly on
      inconsistent/degenerate/rational input or a failed common-basis merge.

## 3. HOST-analytic gate (no OCCT — the airtight-oracle primary gate)
- [x] 3.1 `tests/native/test_native_nurbs_gordon.cpp` + CMake wiring (numsci-gated, mirroring
      `test_native_nurbs_skin`: `_SRC` + `CYBERCAD_TESTS` under `CYBERCAD_HAS_NUMSCI`, plus the
      per-target `target_compile_definitions(... CYBERCAD_HAS_NUMSCI=1)`).
- [x] 3.2 Network containment (the core oracle): `S(·, v_k) == C_k` and `S(u_l, ·) == D_l` pointwise
      on a dense sample to ~1e-8 (achieved ~5e-15) — for both a cubic×cubic `6×6` network and a
      mixed-degree `K≠L` `6×5` network.
- [x] 3.3 Grid intersection: the `K×L` grid points lie on the surface to ~1e-8 (achieved ~1e-15).
- [x] 3.4 Known-surface round-trip: extract a Greville iso-curve network from a KNOWN (uniform-knot)
      tensor surface → Gordon → recover the source closely (~1e-6, avg-knot confound documented);
      IDEMPOTENCE (rebuild at the same station params) is the machine-exact full-surface identity
      (~4e-15).
- [x] 3.5 Honest declines: inconsistent network (curve lifted off the grid), <2 curves in a
      direction, rational curve, mismatched param sizes, non-monotone stations — all `ok=false` with
      a reason, no crash.

## 4. SIM native-vs-OCCT parity gate — OPTIONAL FOLLOW-UP (not this pass)
- [ ] 4.1 `tests/sim/native_nurbs_gordon_parity.mm` cross-checking OCCT network surfacing for a
      couple of networks. HOST is primary and sufficient; this is a separate track (simulator shared
      with concurrent tracks).

## 5. Docs & close-out
- [x] 5.1 Update `docs/NURBS-SCOPE.md` §4 Layer-6 row: skinning+sweep+Gordon now partial; rational +
      irregular/N-sided networks + exact BRepFill residual.
- [x] 5.2 Run `openspec validate --all --strict` (pass), full host ctest (87/87 pass, zero
      regression). `cc_*` ABI byte-unchanged (no ABI file touched); `src/native` stays OCCT-free;
      `bspline_ops.h` / `bspline_skin.h` only `#include`d (not modified); `ssi/blend/boolean/topology`
      untouched.
