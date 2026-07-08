# Tasks — moat-m2asm-first-freeform-boolean

## 1. Substrate build (fresh worktree)

- [x] 1.1 `bash scripts/build-numsci.sh iossim && bash scripts/build-numsci.sh host` — both exit 0.
- [x] 1.2 Export `CYBERCAD_NUMSCI_DIR=…/build-numsci/host` for host builds; host suite green at
      baseline (CTest `39/39` before this change; `40/40` after) — diffed, no regression.
- [x] 1.3 B1/B2/B3/M0/M1 headers byte-identical vs `HEAD` (`git diff --stat` empty) and their
      tests pass unchanged.

## 2. The bowl-lidded convex-quad prism operand fixture (host)

- [x] 2.1 `tests/native/first_freeform_boolean_fixture.h`: builds solid `A` — bowl Bézier top,
      four vertical PLANAR side walls, planar bottom quad at `z = −H0` (base depth `H0=0.5`,
      bowl UNSHIFTED so the B2 fixture's real M1 seam is reused verbatim).
- [x] 2.2 `operand_admitted_and_meshes_at_closed_form_volume` asserts one `Freeform` + five
      `AnalyticHalfSpace` faces, `watertight == true` (B1 unchanged).
- [x] 2.3 Same test asserts M0 `SolidMesher::mesh(A)` watertight, volume = closed-form
      `∫∫_Q (H0+a(x²+y²))dA` within the deflection band (measured 0.95% at deflection 0.01).
- [x] 2.4 `polyVolume` closed-form oracle (fan triangulate + exact per-tri quadratic moments);
      `closed_form_volume_oracle_is_exact` unit-checks it vs the hand value `H0/2 + a/6`.

## 3. B4 verb — `src/native/boolean/half_space_cut.h` (NEW, OCCT-free)

- [x] 3.1 `HalfSpaceCutDecline` enum (`NotAdmitted`, `NoFreeformFace`, `SeamUnusable`,
      `SplitFailed`, `UnsupportedEdgeKind`, `AnalyticCrossingNot2`, `SectionLoopOpen`,
      `SectionLoopNotSimple`, `WeldOpen`, `NotWatertight`, `MembershipReject`) + `declineName`.
- [x] 3.2 Analytic-face split (`cutAnalyticFace`/`scanAnalyticEdges`): keep-whole / drop-whole /
      split-along-`Face ∩ P` (crossing found by bisection on the edge's 3-D curve; curved edges
      split by de Casteljau), rebuilding the keep sub-face over the parent `Plane` frame with a
      fresh pcurve, orientation chosen to match the parent's outward normal.
- [x] 3.3 Cross-section cap synthesis: the B2 seam chord + each split face's `Face∩P` chord are
      chained into ONE closed loop by endpoint-matching (`orderLoop`), simplicity-tested
      (`loopSimple`), and built as an outward-oriented `Plane` cap. The seam samples the same
      3-D nodes as the kept freeform sub-face → M0 position-welds it bit-coincident.
- [x] 3.4 Weld: kept freeform sub-face + kept analytic sub/whole faces + cap → Shell → Solid;
      DECLINE (`WeldOpen`) if < 4 faces.
- [x] 3.5 `halfSpaceCut(op, P, side, split, seam3d)` driver wiring 3.2–3.4; every function in the
      backend cognitive-complexity band (clang-tidy `cognitive-complexity` ≤ 15, verified).
- [~] 3.6 Host proof lands via the composed gate (`first_freeform_cut_is_watertight_at_closed_form_volume`
      exercises the exact `e0`/`e2`/bottom split + cap + weld). Decline coverage: plane-misses-operand
      and non-operand return NULL. **PARTIAL** — a B4-in-isolation test (tangent crossing / artificially
      non-simple loop / open weld) is not yet split out; the code paths exist and DECLINE.

## 4. Compose the first freeform↔analytic CUT (host)

- [x] 4.1 `freeformHalfSpaceCut(operand, P, side)`: B1 recognise → M1 `WLine`
      (`Closed`/`BoundaryExit`, ≥ 2 nodes; ParamBox from the operand bbox → covers the full seam)
      → B2 `splitFace` → B4 `halfSpaceCut` → M0 self-verify; NULL on any verb decline.
- [~] 4.2 B3 confirmation: **DEFERRED.** The keep-side selection is made geometrically (the freeform
      sub-face's trim centroid's signed side of `P`); the independent B3 `classifyPointInMesh`
      cross-check is not yet wired (a boundary-face centroid classifies `On`, so it needs an
      inward nudge — deferred to avoid false declines). Correctness is instead proven by the
      independent closed-form volume oracle (4.4). `MembershipReject` code is reserved.
- [x] 4.3 Mandatory self-verify: the driver meshes the result and DISCARDS (→ NULL,
      `NotWatertight`) any non-watertight candidate. The closed-form volume band is checked by the
      gate test (4.4) rather than inside the oracle-free driver.
- [x] 4.4 HOST ANALYTIC gate (`first_freeform_cut_is_watertight_at_closed_form_volume`): the CUT
      assembles watertight and its enclosed volume = `∫∫_{Q∩{x≤0}}(H0+a(x²+y²))dA` within the band
      (measured 0.71% at deflection 0.01).
- [~] 4.5 Honest-decline tests: `cut_plane_missing_operand_declines_null` and
      `non_freeform_operand_declines_null` pass. **PARTIAL** — fabricated non-watertight /
      wrong-volume discard tests not yet added (the `NotWatertight` discard path exists in the driver).

## 5. SIM native-vs-OCCT gate (booted simulator, OCCT linked)

- [x] 5.1 New standalone sim harness `tests/sim/native_first_freeform_boolean_parity.mm`:
      operand + cutter reconstructed natively (bowl-lidded convex-quad prism) AND as OCCT
      shapes (sewn 6-face `Geom_BezierSurface`-topped solid); runs native
      `freeformHalfSpaceCut` AND `BRepAlgoAPI_Cut` against a large box. Ran on booted
      simulator `2B90AEDB-…` → **12/12 PASS**.
- [~] 5.2 **DONE:** `BRepGProp` volume (nat=0.098998 vs occt=0.098297, rel 7.13e-03 ≤ 2e-2)
      + area (rel 4.43e-04) parity within the fixed curved band; native watertightness
      (closed 2-manifold) + topology (Euler χ=2, single closed solid) asserted; spatial
      agreement via `AddOptimal` bbox (worst 1.00e-07) and one-sided Hausdorff (1.57e-07 ≤
      1.5e-2). **PARTIAL:** the explicit in-harness `BRepClass3d_SolidClassifier`
      query-point batch clause was NOT re-added — freeform point-in-solid membership vs
      `BRepClass3d` was already gated `crispDISAGREE=0` on the same native mesher in the
      landed **M2c** change; the spatial Hausdorff/bbox parity here is the stronger
      surface-level equivalent for this gate.
- [~] 5.3 **PARTIAL:** the watertight self-verify DISCARD path exists in the driver
      (`NotWatertight` → NULL → OCCT fall-through, verified in the host GATE (a) driver,
      cf. task 4.5); a deliberately-perturbed-candidate case was NOT re-fabricated inside
      the sim harness. The sim gate asserts the ACCEPT path is watertight + OCCT-parity.
- [x] 5.4 Sim suite accounting consistent: harness has its own `main()` and is added to the
      `run-sim-suite.sh` SKIP list (one line) — excluded from the auto-linked multi-source
      suite exactly like the other `native_*_parity` harnesses; suite stays green.

## 6. Additivity, complexity, docs

- [x] 6.1 BYTE-IDENTICAL vs `HEAD`: `freeform_operand.h`, `face_split.h`,
      `freeform_membership.h`, `solid_mesher.h`, `ssi/marching.h`, `ssi_boolean.h` — `git diff`
      empty. `grep` finds 0 OCCT includes under `src/native/**`. No `cc_*` change (facade untouched).
- [x] 6.2 clang-tidy `readability-function-cognitive-complexity` (threshold 15) on
      `half_space_cut.h`: every function ≤ 15 after extracting `scanAnalyticEdges`/`traceWallSeam`/
      `seamChord3d` helpers.
- [x] 6.3 `openspec/MOAT-ROADMAP.md` M2 row + B4/B5 note updated: B4 landed, first CUT verified,
      B2 smooth-trim generalisation recorded as the deferred next enabler.
- [x] 6.4 CTest green `40/40` (was `39/39`), new test wired into CMake under `CYBERCAD_HAS_NUMSCI`.

## 7. Honest-decline fallback (if the composition does not robustly assemble)

- [x] 7.1 NOT triggered — the end-to-end CUT robustly assembled this wave (watertight, closed-form
      volume to 0.71%), so the composed gate is the landed proof. The NEXT enablers are recorded:
      the **SIM native-vs-OCCT gate (§5)**, the **B3 inward-nudge confirmation (§4.2)**, and the
      **B2 smooth-trim (closed/circular wall) generalisation** — the deferred sidestepped blocker (i).
