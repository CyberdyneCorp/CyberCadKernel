# Tasks — nurbs-swept-surface

## 1. Foundation — module + declarations
- [x] 1.1 Create `src/native/math/bspline_sweep.h` — `SweepResult`, and the `sweepTranslational` /
      `sweepAlongTrajectory` declarations. Reuses the Layer-1 `BsplineCurveData` /
      `BsplineSurfaceData` and `vec.h` `Vec3`/`Dir3`. Document the frame choice (rotation-minimizing,
      double-reflection) and residuals in the header. Add to `native_math.h`.

## 2. Translational sweep (§10.4, exact)
- [x] 2.1 `sweepTranslational(section, sweep)` — the EXACT tensor product: U = section (degree `p`,
      section knots, `N` poles), V = degree-1 two-pole path `{0,0,1,1}` with poles `0` and `sweep`;
      net `pole(i,0)=section.poles[i]`, `pole(i,1)=section.poles[i]+sweep`. `vParams={0,1}`.
      Non-rational. Guards: rational section, malformed section, null sweep → `ok=false`.

## 3. General sweep (§10.4, transform-then-skin)
- [x] 3.1 Spine sampling — `stations` params across the clamped domain; `curveDerivs` for point +
      unit tangent; stationary-point and coincident-trajectory guards.
- [x] 3.2 Rotation-minimizing frame — double-reflection propagation (Wang et al. 2008), seeded from
      the section plane normal projected orthogonal to `T₀`; re-orthogonalize + normalize per step.
- [x] 3.3 Station placement — build the section reference basis and each station basis; the rigid
      placement `v' = Rₖ·v + Pₖ` transforms a copy of the section.
- [x] 3.4 `sweepAlongTrajectory` — skin the `stations` placed sections via `skinSurface`; return the
      surface + station `vParams`. Guards: `<2` stations, rational section/trajectory, malformed
      inputs, degenerate domain, skin failure → `ok=false`.

## 4. HOST-analytic gate (no OCCT — the airtight-oracle primary gate)
- [x] 4.1 `tests/native/test_native_nurbs_sweep.cpp` + CMake wiring (numsci-gated, mirroring
      `test_native_nurbs_skin`: `_SRC` + `CYBERCAD_TESTS` under `CYBERCAD_HAS_NUMSCI`, plus the
      per-target `target_compile_definitions(... CYBERCAD_HAS_NUMSCI=1)`).
- [x] 4.2 Translational exactness: `S(·,v)` == section translated by `v·sweep` pointwise on a dense
      station+u sample to ~1e-12 (achieved ~1e-15) — the base-case oracle.
- [x] 4.3 Station containment (general): the swept surface contains each transformed section at its
      station parameter to ~1e-8 (achieved ~1e-15).
- [x] 4.4 Known-surface checks: straight section → closed-form bilinear ruled patch (~1e-12);
      circle-approx section → cylinder-like patch, each height iso == section profile (~1e-12).
- [x] 4.5 Anti-twist frame sanity: straight general sweep == translational sweep pointwise (~1e-8,
      achieved ~1e-15); curved-spine station arc-length invariant.
- [x] 4.6 Degenerate guards: `<2` stations, coincident-trajectory / null sweep, rational
      section/trajectory, malformed sections handled honestly (no crash).

## 5. SIM native-vs-OCCT parity gate — OPTIONAL FOLLOW-UP (not this pass)
- [ ] 5.1 `tests/sim/native_nurbs_sweep_parity.mm` cross-checking OCCT `BRepOffsetAPI_MakePipe` /
      `GeomFill` for a couple of sweeps. HOST is primary and sufficient; this is a separate track
      (simulator shared with concurrent tracks).

## 6. Docs & close-out
- [x] 6.1 Update `docs/NURBS-SCOPE.md` §4 Layer-6 row: skinning + sweep now partial; Gordon / network
      / rational / exact-BRepFill / variable-section residual.
- [x] 6.2 Run `openspec validate --all --strict` (pass), full host ctest (zero regression). `cc_*`
      ABI byte-unchanged (no ABI file touched); `src/native` stays OCCT-free; `bspline_ops.h` /
      `bspline_skin.h` / `bspline.h` / `transform.h` only `#include`d (not modified);
      `ssi/blend/boolean/topology` untouched.
