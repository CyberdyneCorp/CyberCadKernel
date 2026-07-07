# Tasks — moat-m6d-blend-differential-fuzz

## 1. Deterministic seeded generator (native-claimed blend families)
- [x] 1.1 splitmix64 → xoshiro256** RNG keyed ONLY by an explicit uint64 seed (argv/env),
      no clock / `rand()`; fixed deterministic default seed + N.
- [x] 1.2 Generate blend inputs from planar chamfer (box edge) / planar fillet (box edge) /
      curved constant-r fillet / curved variable-linear fillet (r1→r2) / curved symmetric
      chamfer / curved asymmetric chamfer (d1,d2), plus ONE sparse out-of-scope DECLINE-
      exerciser (fillet `Rc/2 < r < Rc`, outside the native ring-torus scope).
- [x] 1.3 Build every body through the SAME construct entry points the facade uses
      (`build_prism_profile` full-circle → capped cylinder; `build_prism` → box); constrain
      every param valid / non-degenerate (`Rc ≥ 2·max(r)`, `Rc − d2 > 0`, setback well
      inside the box faces).
- [x] 1.4 Compute the closed-form REMOVED volume per AGREE family (the analytic arbiter
      input): box-edge prism/groove, torus-canal Pappus fillet (constant + linear law),
      cone-frustum chamfer `π·d1·d2·(Rc − d2/3)`.

## 2. Dual build on the SAME geometric edge/rim (native builder vs OCCT oracle)
- [x] 2.1 Call the OCCT-free native blend builder DIRECTLY (`blend::chamfer_edges` /
      `fillet_edges` / `curved_fillet_edge` / `variable_fillet_edge` / `curved_chamfer_edge`
      / `curved_chamfer_edge_asym`) so a NULL / non-watertight result is an UNAMBIGUOUS
      native DECLINE, not a silent facade forward; measure by the native tessellator (mesh
      vol/area, watertight, solid count).
- [x] 2.2 Build the SAME body + blend the SAME geometric edge/rim via OCCT
      (`BRepPrimAPI_MakeBox` / `MakeCylinder` → `BRepFilletAPI_MakeFillet` / `MakeChamfer`,
      including `Add(d1,d2,edge,face)` for the asymmetric chamfer); measure exactly by
      `BRepGProp` + a `BRepCheck` validity check. Match the box edge by vertex coincidence;
      pick the cylinder rim by geometry (Circle edge at `z = h`).
- [x] 2.3 ORACLE validity gate: for a CORE family the OCCT build MUST be a valid closed
      solid, else ORACLE_UNRELIABLE (excluded, fails the bar — never laundered).

## 3. Analytic-arbitrated classifier (exact math is the primary correctness oracle)
- [x] 3.1 AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE / BOTH-DECLINED at a
      FIXED tolerance (`kVolRelTol = 2e-2`, `kAreaRelTol = 3e-2`), never widened per-trial.
- [x] 3.2 Clean AGREE = native-vs-OCCT within tol. When native-vs-OCCT EXCEEDS tol,
      arbitrate with exact math: native matches analytic AND OCCT matches analytic → AGREED
      (native vindicated, counted as analytic-AGREE); native matches analytic, OCCT does not
      → ORACLE-INACCURATE; native fails analytic → DISAGREED. Volume is exact-math-gated so
      an area-only excursion can never produce a false DISAGREE.
- [x] 3.3 Print seed + case index + family/param tuple + all measurements (native, OCCT,
      analytic) on any DISAGREE / ORACLE-INACCURATE (a reproducible regression / limitation
      record).

## 4. Coverage summary + zero-silent-wrong-blend bar
- [x] 4.1 Per-family summary [agreed/declined/DISAGREED/oracle-inaccurate/both-declined] +
      the analytic-AGREE count + the measured max native-vs-OCCT bias vs the fixed tol;
      exit 0 IFF `DISAGREED == 0` AND core-family `ORACLE_UNRELIABLE == 0`.
- [x] 4.2 Log the domain-level honest exclusions (concave stepped-shaft fillet;
      offset_face / shell — left to curated parity for this first slice) in the harness
      header + spec.

## 5. Build + wiring (additive test/sim only)
- [x] 5.1 `tests/sim/native_blend_fuzz.mm` (own `main()`, `std::_Exit`, OCCT-free native
      TUs + OCCT oracle, no numsci).
- [x] 5.2 `scripts/run-sim-native-blend-fuzz.sh` (compile + run in booted simulator, seed +
      N argv/env).
- [x] 5.3 Add `native_blend_fuzz.mm` to `scripts/run-sim-suite.sh`'s SKIP list.
- [x] 5.4 Confirm `src/native/**` UNTOUCHED and OCCT-free; `cc_*` ABI unchanged.

## 6. Proof
- [x] 6.1 `DISAGREED == 0` across ≥2 seeds (0x5744EE9911 N=96; 0xC0FFEE1234, 0xDEADBEEF99
      N=160) with real per-family coverage (all six core families ≥15 AGREE each) and the
      DECLINE branch exercised.
- [x] 6.2 native-vs-exact-math error ≤ ~1.6e-3 for every family (a ~12× margin under the
      fixed 2e-2 tolerance); byte-identical determinism across two runs of the same seed.
- [x] 6.3 Record the ORACLE-INACCURATE finds (seeds 0xC0FFEE1234 index 37/134, 0xDEADBEEF99
      index 38/57/76: OCCT's variable-radius fillet ~2–2.6% off exact math, native matches
      it to ~2–3e-4 → native vindicated, bar NOT failed).

## 7. Archive (post-merge)
- [x] 7.1 `openspec archive moat-m6d-blend-differential-fuzz` after the change lands.
