# Tasks — moat-dm1-split-plane (M-DM DM1, native `cc_split_plane`)

Order: substrate build → baseline capture → keep-side mapping + domain dispatch →
consume the two landed verbs → per-piece self-verify + labelled decline → host
partition-closure gate → sim native-vs-OCCT parity gate → zero-regression proof →
docs, or HONEST DECLINE. All new native-facing code stays OCCT-free and host-buildable
(`clang++ -std=c++20`); the assembly is additive engine glue in `src/engine/native/`.
No `cc_*` ABI change. The two landed verbs (`freeformHalfSpaceCut`, `boolean_solid`)
are CONSUMED byte-identical. No tolerance is weakened; a correct decline (piece stays
OCCT) is a first-class outcome.

## 0. Substrate + baseline (capture BEFORE touching the engine)

- [x] 0.1 Build the numeric substrate for both targets: `bash scripts/build-numsci.sh
      iossim && bash scripts/build-numsci.sh host`; export `CYBERCAD_NUMSCI_DIR` to the
      matching `build-numsci/{iossim,host}` per target.
- [x] 0.2 Build host + NUMSCI and record the GREEN baseline for the split/boolean-
      adjacent suites (native-boolean host pass, curved-boolean native-pass, the M2
      `freeformHalfSpaceCut` host gate 40/40, the sim first-freeform-boolean parity
      12/12) — the reference for the §7 zero-regression proof.
- [x] 0.3 Confirm `NativeEngine::split_plane` today is the unconditional
      `CC_NATIVE_BODY_UNSUPPORTED` → `fallback().split_plane` fall-through (every
      `cc_split_plane` call routes to OCCT), so the native branch is reachable by NO
      currently-passing native path — the additive precondition.
- [x] 0.4 Author the OCCT oracles for the reachable fixtures: for each of {axis-aligned
      box, axis-aligned cylinder cut ⊥ axis, bowl-lidded convex-quad prism} split by a
      plane, the OCCT per-piece volume / area / watertight / Euler-χ / bbox for BOTH
      keep sides (`BRepAlgoAPI_Section` / the OCCT `split_plane` two-sided `Cut`).

## 1. Keep-side mapping + plane frame

- [x] 1.1 Build the cut plane `P` from the ABI args: `P.pos.origin = (ox,oy,oz)`,
      `P.pos.z = normalize(nx,ny,nz)`; reject a degenerate normal (‖n‖ < 1e-9) → decline.
- [x] 1.2 Map the ABI keep flag: `keepPositive != 0` → `KeepSide::Above` (keep `+n`,
      signed dist ≥ 0); `0` → `KeepSide::Below`. Unit-assert the mapping against
      `freeformHalfSpaceCut`'s documented `Below ≤ 0 / Above ≥ 0` semantics.
- [x] 1.3 Implement `halfSpaceBox(P, discardSide)`: a bbox-scaled (`L = 2·(diag+1)`)
      native planar box covering the DISCARD half, built through the existing native
      construction path (no OCCT), for the analytic BSP cut.

## 2. Native branch in `NativeEngine::split_plane` (additive, before the fall-through)

- [x] 2.1 Guard: proceed to the native branch ONLY when `body` resolves to a native
      solid; otherwise keep `fallback().split_plane(...)` BYTE-IDENTICAL.
- [x] 2.2 Dispatch A — freeform-walled operand (exactly one `Kind::Bezier`/`BSpline`
      wall): `piece = freeformHalfSpaceCut(operand, P, side, defl, &why)`; NULL on any
      `HalfSpaceCutDecline` → decline (log `declineName(why)`).
- [x] 2.3 Dispatch B — all-planar polyhedron (every face `Kind::Plane`):
      `piece = boolean_solid(operand, halfSpaceBox(P, discard), Op::Cut)`.
- [ ] 2.4 Dispatch C — axis-aligned cylinder with `n ∥ axis` (perpendicular cut):
      `piece = boolean_solid(operand, halfSpaceBox(P, discard), Op::Cut)` (the
      axis-aligned box⟷cylinder analytic slice). Any oblique / non-axis-aligned cut of a
      curved face → decline (dispatch falls to the else arm).
- [x] 2.5 Keep the driver's cognitive complexity in the systems band: delegate keep-side
      mapping, domain classification, and self-verify to helpers; the geometry stays in
      the two already-landed verbs (no re-derivation).

## 3. Per-piece self-verify + labelled decline → OCCT

- [x] 3.1 After a native `piece`, run the engine's mandatory audit
      `watertightVolume(piece)`: accept native ONLY when the piece is a closed
      watertight 2-manifold with positive enclosed volume.
- [x] 3.2 On `piece.isNull()` OR `watertightVolume(piece) ≤ 0`, DISCARD and return
      `fallback().split_plane(...)` — the exact OCCT result. Never emit an unverified /
      leaky piece; never hand a native void to OCCT.
- [ ] 3.3 Label each decline path (freeform decline, non-planar/oblique curved,
      multi-freeform, degenerate, foreign) and prove it returns EXACTLY the OCCT result
      of the same call (`cc_set_engine(1)` vs `cc_set_engine(0)` identical).

## 4. Host gate (a) — analytic partition-closure (OCCT-free)

- [x] 4.1 For each reachable fixture, split by a plane and compute BOTH pieces
      (`cc_split_plane` with `keepPositive` 0 then 1) under the native engine.
- [x] 4.2 Assert each piece is watertight (`watertightVolume > 0`) and
      `V(below) + V(above) = V(whole)` within the deflection tolerance
      (partition-closure).
- [x] 4.3 Assert each piece matches its closed-form volume where known — axis-aligned
      box: fp-exact half-volumes; axis-aligned cylinder ⊥ cut: `π·r²·h_i`; bowl-lidded
      prism: the landed closed-form band per half.

## 5. Sim gate (b) — native-vs-OCCT parity (booted iOS simulator)

- [x] 5.1 For each reachable fixture and each keep side, reconstruct the operand in OCCT,
      cut it by the same plane, and compare the OCCT piece vs the native piece on
      per-piece volume, area, watertightness, topology (Euler χ = 2, single closed
      solid), and bbox.
- [x] 5.2 Match within the landed curved-slice tolerances (volume rel ≤ 2e-2, area/bbox
      tight) — NEVER widened. A native-vs-OCCT discrepancy on a reachable case is a
      BLOCKER (fix the native path or shrink the domain), not a tolerance relaxation.
- [x] 5.3 Register the harness in `run-sim-suite.sh` (own `main()` + SKIP entry, mirror
      the M2 first-freeform-boolean parity harness).

## 6. Decline coverage (verify the honest-out, not faked)

- [x] 6.1 Oblique plane grazing / tangent to a curved face → native returns NULL, call
      matches OCCT.
- [ ] 6.2 Multi-lump split (plane severing the solid into > 2 connected halves) →
      decline → OCCT.
- [x] 6.3 Degenerate (plane misses the solid / coincident with a boundary face /
      zero-volume sliver) → decline → OCCT.
- [x] 6.4 Multi-freeform operand and foreign / OCCT body → decline / fall-through →
      identical to `cc_set_engine(0)`.

## 7. Zero-regression proof + docs

- [x] 7.1 Prove BYTE-IDENTICAL: the two consumed verbs (`freeformHalfSpaceCut`,
      `boolean_solid`) and every existing analytic path are unchanged; re-run the §0.2
      baseline suites and confirm no diff (the branch is reachable only by a case that
      today declines to OCCT).
- [x] 7.2 Confirm `cc_*` ABI unchanged: `cc_split_plane`, `IEngine::split_plane`, the
      facade, and `OcctEngine::split_plane` are byte-identical; only
      `NativeEngine::split_plane` gained the native branch.
- [x] 7.3 Measure cognitive complexity of the changed `NativeEngine::split_plane` +
      helpers (cognitive-complexity skill); keep within the systems band, flag any
      genuinely irreducible function. RESULT: `splitByPlane` = 7 (🟢 Excellent), the
      `NativeEngine::split_plane` driver ~3 — both well inside the systems band.
- [x] 7.5 NUMSCI-OFF LINK REGRESSION: `native/boolean/split_plane.h` reaches
      `half_space_cut.h::freeformHalfSpaceCut`, whose seam trace `ssi::trace_intersection`
      is DEFINED only under `CYBERCAD_HAS_NUMSCI`. The always-compiled `native_engine.cpp`
      therefore MUST gate both the `#include` and the native-split body on
      `#ifdef CYBERCAD_HAS_NUMSCI` (native split honestly declines when OFF, exactly as
      pre-DM1). Verified: NUMSCI-OFF host build links `test_native_engine` +
      `test_native_boolean` (both pass) and `native_engine.o` has 0 `trace_intersection`
      undefined refs (matching `main`); `scripts/run-sim-native-boolean.sh` links + 25/25.
- [x] 7.4 Update `openspec/MOAT-ROADMAP.md` §M-DM DM1 with the landed status (which
      cases native, which decline, both gate results), or record the HONEST DECLINE with
      the measured gap if a gate cannot be met.
