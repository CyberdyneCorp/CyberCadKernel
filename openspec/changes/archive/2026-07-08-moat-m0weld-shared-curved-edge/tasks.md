# Tasks — moat-m0weld-shared-curved-edge (M0 weld robustness)

Order: baseline + regression witness → canonical curved‑edge single‑sampling
(additive) → host analytic sweep gate → sim OCCT parity gate → zero‑regression proof →
docs, or HONEST DECLINE. All new native code stays OCCT‑free and host‑buildable
(`clang++ -std=c++20`), namespace `cybercad::native::tessellate`. No `cc_*` ABI change.
The change is **strictly additive**: the canonical curved path is reachable ONLY by a
genuinely‑curved, non‑`TShape`‑shared edge that today welds coincidentally, and every
existing mesh MUST stay byte‑identical — PROVEN, not assumed. No tolerance is weakened;
a correct decline (freeform boolean keeps the OCCT path) is a first‑class outcome.

## 0. Baseline + regression witness (capture BEFORE touching the mesher)

- [x] 0.1 Build host + NUMSCI (`bash scripts/build-numsci.sh iossim && … host`; export
      `CYBERCAD_NUMSCI_DIR`). Record the GREEN baseline: `run-sim-suite`, STEP import,
      curved‑fillet, curved‑chamfer, curved‑boolean (native‑pass), wrap‑emboss, loft,
      phase3.
- [x] 0.2 Snapshot the per‑face mesh signature (triangle count, watertight flag,
      enclosed volume) for a face of EVERY existing surface kind (`Plane`, `Cylinder`,
      `Cone`, `Sphere`, `Torus`, bare‑periodic `BSpline`, `Bezier`), a planar trim, and
      a loft side wall — the byte‑identical reference for §5.
- [x] 0.3 Add the **regression witness**: run the freeform CUT + COMMON (and the full
      bowl operand) through the sweep `{0.03,0.02,0.01,0.008,0.004,0.002}` on `main` and
      record the watertight flag at each deflection — this captures the current
      watertight↔NotWatertight oscillation the fix must eliminate.

## 1. Canonical curved‑edge single‑sampling (`src/native/tessellate/edge_mesher.h`)

- [x] 1.1 Add a `CurveKey` (order‑independent quantized endpoint pair + quantized 3‑D
      **midpoint** discriminator `C_edge((first+last)/2)`) and `CurveKeyHash`, mirroring
      the existing `EndpointKey`/`EndpointKeyHash`. The midpoint discriminator guarantees
      two DIFFERENT arcs between the same endpoints never share a key.
- [x] 1.2 Add `curvedByEndpoints_` (`CurveKey → EdgeDiscretization`) alongside `cache_`
      and `segsByEndpoints_`. Extend `discretize(edge)`: (a) `TShape` hit → return
      existing record UNCHANGED; (b) build the record; (c) if straight in 3‑D
      (`edgeCurvature ≈ 0`, same test as `edgeSegments`) → store in `cache_` and return
      UNCHANGED; (d) genuinely curved + reached via a separate node → look up/insert in
      `curvedByEndpoints_`, return the CANONICAL record, and memoise it under the
      `TShape` in `cache_`.
- [x] 1.3 Keep `edgeCurveLocal`, `edgeCurvature`, `edgeSegments`, `build`,
      `canonicalLineEndpoints`, and `segsByEndpoints_` BYTE‑IDENTICAL — the new store is
      strictly additive and inert for straight and `TShape`‑shared edges.
- [x] 1.4 Keep `discretize` cognitive complexity in the backend band (≤ 15; target ≤ ~8):
      isolate the curved‑key build and lookup behind one helper each.

## 2. Both faces consume the ONE canonical polyline (`face_mesher.h`)

- [x] 2.1 Confirm (test) that `recordEdgeAnchors`'s curved arm now snaps BOTH incident
      faces' seam vertices to the SAME canonical `d.points` (from §1) — no new cross‑face
      anchor state; `BoundaryAnchors` stays local. If a small gate/guard helper is needed
      for clarity, add it WITHOUT editing the three mesh arms or `flattenWireShared`.
- [x] 2.2 Assert `structuredGrid`, `earClipMesh`, `trimmedFreeformMesh`,
      `appendEdgeSamplesAtFracs` are untouched (byte‑identical output on §0.2 fixtures).

## 3. Host analytic gate — deflection sweep (`tests/native/…`, no OCCT)

- [x] 3.1 Extend `test_native_first_freeform_boolean.cpp` (or a sibling) to sweep
      `d ∈ {0.03,0.02,0.01,0.008,0.004,0.002}`: the full bowl operand meshes WATERTIGHT
      at every `d`, enclosed volume within the `d`‑band of the closed‑form
      `∫∫_Q (H0 + a(x²+y²))`.
- [x] 3.2 `freeformHalfSpaceCut` (KeepSide::Below) meshes WATERTIGHT at every `d` in the
      sweep, enclosed volume within the `d`‑band of the closed‑form CUT value
      `∫∫_{Q∩{x≤0}} (H0 + a(x²+y²))` — replacing the single‑deflection `0.01` check.
- [x] 3.3 The COMMON meshes WATERTIGHT at every `d`, volume within the `d`‑band of its
      closed‑form value. No watertight↔open oscillation across the sweep.

## 4. Sim OCCT parity gate (booted simulator, OCCT linked)

- [x] 4.1 The freeform CUT parity test (`BRepAlgoAPI_Cut` oracle) still passes AND now at
      MULTIPLE deflections (≥ `0.01` and one finer, e.g. `0.004`): native volume / area /
      watertight / triangle envelope match OCCT within tolerance at each — OR the reader
      declines and OCCT handles it (both PASS).
- [x] 4.2 Confirm the engine's watertight + volume self‑verify still DISCARDS any
      non‑watertight native result → OCCT (a leaky mesh is never emitted).

## 5. Zero‑regression proof (MANDATORY — the mesher was touched)

- [x] 5.1 Re‑run the FULL suite from §0.1; every count MUST be unchanged.
- [x] 5.2 Re‑snapshot §0.2 per‑kind signatures; triangle counts, watertight status,
      enclosed volumes MUST be BYTE‑IDENTICAL to the `main` baseline. If ANY differs →
      revert (see §7).
- [x] 5.3 Re‑run §0.3: the sweep is now WATERTIGHT at all six deflections (the witness
      flips from oscillating to all‑watertight).

## 6. Docs / spec

- [x] 6.1 Update `openspec/MOAT-ROADMAP.md` M0 weld‑robustness status (curved‑edge
      single‑sampling landed, sweep now watertight; or the honest decline with the
      measured sweep + specific obstacle).
- [x] 6.2 `openspec validate moat-m0weld-shared-curved-edge --strict`; archive on
      completion.

## 7. Honest‑out (a first‑class outcome, not a failure) — NOT TRIGGERED

> Resolution: the additive path WAS achieved and PROVEN byte‑identical (every existing
> surface kind's mesh — FNV of all vertices + tris + wt + area + vol — unchanged; only the
> previously‑leaking freeform operand deflections change, to become watertight). The
> honest‑out below therefore did not apply.

- [ ] 7.1 If no additive path keeps every existing mesh byte‑identical AND welds the
      curved seam at every deflection (e.g. the two incident faces carry geometrically
      DIFFERENT seam curves beyond weld tolerance — a healing problem — or the boolean
      does not expose recoverable per‑edge endpoints): REVERT `edge_mesher.h` /
      `face_mesher.h`, keep the OCCT decline for the freeform boolean, and DOCUMENT the
      specific obstacle + the measured sweep. No fabrication, no dead code, no weakened
      tolerance — the mesher stays pristine and the decline is reported honestly.
