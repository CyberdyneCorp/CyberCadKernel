# Tasks — moat-clash-interference (MOAT M-GS, GS7)

## 1. Native mesh-level classifier (OCCT-free)
- [x] 1.1 New header `src/native/analysis/interference.h` (namespace
      `cybercad::native::analysis`), consuming `boolean/freeform_membership.h` (B3)
      + `tessellate/mesh.h` READ-ONLY; zero OCCT includes.
- [x] 1.2 `ClashState` enum (Clear / Touching / Clash / Unknown) + `InterferenceResult`
      (state, minDistance, overlapVolume, witness AABB + point).
- [x] 1.3 `meshInterference(a, b, deflection)`: watertight precondition → Unknown;
      penetration via B3 vertex + triangle-centroid `In`; contact/clearance via the
      min triangle–triangle distance; coplanar-safe (shared face reads `On`).
- [x] 1.4 Honest-decline gate: an `Unknown` vetoes only when the point is strictly
      inside the target AABB AND beyond the contact band (masks a possible overlap);
      a spurious coplanar-grazing / seam `Unknown` does not force a decline.

## 2. Engine dispatch (native + OCCT oracle)
- [x] 2.1 `IEngine::interference(a, b)` virtual + `InterferenceData` POD (state,
      overlapVolume, minDistance, witness) with an `engine_unsupported` default.
- [x] 2.2 `NativeEngine::interference`: reject mixed native/OCCT, forward all-OCCT;
      mesh both bodies; run `meshInterference`; on CLASH fill the overlap volume from
      `boolean_solid(A,B,Common)` via `watertightVolume`.
- [x] 2.3 TWO-SIDED volume self-verify: COMMON watertight AND `vc <= min(V(A),V(B))`;
      a null / non-watertight / out-of-band COMMON DECLINES to OCCT (never a wrong volume).
- [x] 2.4 Sharpen the CLASH witness to the COMMON solid's AABB + signed-tetra centroid.
- [x] 2.5 `OcctEngine::interference` ORACLE: `BRepAlgoAPI_Common` + `BRepGProp` volume,
      `BRepExtrema_DistShapeShape` clearance, COMMON bbox + centroid witness.

## 3. Facade (additive-only cc_* ABI)
- [x] 3.1 `CCInterference` POD + `CCClashState` enum + `cc_interference(a, b, out)`
      in `include/cybercadkernel/cc_kernel.h` (signature-styled like `cc_check_solid`).
- [x] 3.2 `cc_interference` in `src/facade/cc_kernel.cpp` (guarded; marshals the
      engine result; `decided = 0` + `cc_last_error` on decline).

## 4. GATE A — host analytic (no OCCT)
- [x] 4.1 `tests/native/test_native_interference.cpp` + CMake registration.
- [x] 4.2 Overlapping boxes → CLASH, exact intersection-box volume via the native
      COMMON + witness point inside the true overlap.
- [x] 4.3 Disjoint boxes → CLEAR, exact gap; face-touching boxes → TOUCHING, volume 0;
      nested box → CLASH.
- [x] 4.4 Non-watertight operand → UNKNOWN honest decline (never a guessed clash).

## 5. GATE B — sim native-vs-OCCT parity
- [x] 5.1 `tests/sim/native_interference_parity.mm` (own `main()`): CLASH volume vs
      `BRepAlgoAPI_Common` + `BRepGProp`; CLASH/TOUCHING/CLEAR + clearance vs
      `BRepExtrema_DistShapeShape`, on identical geometry at fixed tolerances.
- [x] 5.2 `scripts/run-sim-native-interference.sh` runner + `run-sim-suite.sh` SKIP entry.

## 6. Finalize
- [x] 6.1 `openspec/MOAT-ROADMAP.md` — new "analysis — interference/clash (GS7)" entry.
- [x] 6.2 `openspec validate --strict` on the change.
- [x] 6.3 Structural discipline: `src/native/**` OCCT-free + `cc_*` additive; tessellator
      + boolean + blend + exchange UNTOUCHED (analysis-only + engine/facade).
