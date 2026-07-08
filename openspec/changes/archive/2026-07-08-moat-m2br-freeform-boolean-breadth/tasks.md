# Tasks — moat-m2br-freeform-boolean-breadth

## 1. Substrate build (fresh worktree)

- [x] 1.1 `bash scripts/build-numsci.sh iossim && bash scripts/build-numsci.sh host` — both exit 0.
- [x] 1.2 Configure host build (`CYBERCAD_HAS_NUMSCI=ON`, `CYBERCAD_NUMSCI_DIR=…/build-numsci/host`);
      M2-assembly CUT gate green at baseline (`test_native_first_freeform_boolean` 5/5) — no regression.
- [x] 1.3 `half_space_cut.h` + B1/B2/B3/M0/M1 byte-identical vs the M2-assembly baseline (consumed
      unchanged; the `KeepSide::Above` path already exists).

## 2. Diagnosis: is COMMON reachable from the landed CUT verb? (host, no OCCT)

- [x] 2.1 Confirm `freeformHalfSpaceCut(A, P, KeepSide::Above)` produces the complementary keep-side
      solid (measured mesh volume ≈ `V_above`).
- [x] 2.2 Confirm the closed-form partition identity `V_below + V_above = V_full` holds mesh-free to
      machine precision (measured residual `0`).
- [x] 2.3 Deflection sweep `{0.03 … 0.002}` for BOTH keep-sides — record the WT/NULL pattern
      (design §3 table): both keep-sides are deflection-fragile; a leak is NEVER emitted; CUT is not
      special (declines at 0.02, 0.004; COMMON at 0.02, 0.01, 0.004, 0.002).
- [x] 2.4 Localise the fragility: the non-watertight boundary edges lie ON the bowl surface along a
      kept quad edge (`z = a(x²+y²)` verified at the crack coordinates) ⇒ a shared CURVED-edge weld
      mismatch from per-face independent tessellation. Root cause pinned; M0 unchanged.

## 3. Complementary oracle + fixture extension (host, OCCT-free)

- [x] 3.1 `tests/native/first_freeform_boolean_breadth_fixture.h`: `clipXge0` (Sutherland–Hodgman
      complement of the M2-assembly `clipXle0`) and `cutVolumeAbove() = polyVolume(clipXge0(quadXY()))`,
      reusing the M2-assembly fixture (`buildOperand`, `polyVolume`, `quadXY`, `cutPlane`) UNCHANGED.
- [x] 3.2 `complement_partition_oracle_is_exact`: `|V_below + V_above − V_full| ≤ 1e-12`,
      `0 < V_above < V_full`, and `area(clipXle0) + area(clipXge0) = area(Q)` (complement check).

## 4. COMMON host gate (host, OCCT-free)

- [x] 4.1 `tests/native/test_native_freeform_boolean_breadth.cpp` — new host suite, wired into CMake
      additively under `CYBERCAD_HAS_NUMSCI` (does NOT modify the M2-assembly CUT test).
- [x] 4.2 `common_keep_side_watertight_at_complementary_volume`: at a both-sides-weld deflection `d*`,
      `freeformHalfSpaceCut(A, P, KeepSide::Above)` returns a watertight `Solid` with volume = `V_above`
      within the 2% deflection band (tolerance NOT weakened).
- [x] 4.3 `cut_and_common_partition_the_operand`: at `d*`, both keep-sides watertight and
      `vol(CUT) + vol(COMMON) = V_full` within the (doubled) deflection band — mesh-level partition.
- [x] 4.4 `self_verify_never_emits_a_leak_across_deflections`: sweep `{0.03,0.02,0.01,0.008,0.005,0.004}`,
      both keep-sides, assert every result is `isNull()` OR watertight — the honest-decline discipline
      holds at 100% of the sweep; the shared-curved-edge weld fragility is documented in code.
- [x] 4.5 CTest green (new suite added; M2-assembly CUT suite still 5/5, unchanged).

## 5. FUSE breadth-blocker diagnosis (documentation, no code)

- [x] 5.1 Diagnose FUSE-with-a-finite-cutter as a TWO-operand boolean requiring inter-solid SSI,
      multi-seam splitting, and two-operand face classification/merge — NOT supplied by the landed
      single-operand/single-plane/single-seam/single-cap verbs (design §6). DECLINED as the next
      breadth blocker; NO partial FUSE path is written or stubbed.

## 6. SIM native-vs-OCCT gate (documented follow-up)

- [ ] 6.1 Extend the curved-boolean sim harness with a `freeform_common_parity` case:
      build operand + half-space natively AND as OCCT shapes; compare native
      `freeformHalfSpaceCut(KeepSide::Above)` against `BRepAlgoAPI_Common` (`BRepGProp`
      volume/area, topology counts, `BRepClass3d_SolidClassifier` point batch). DEFERRED to a
      separate simulator harness (matching the CUT slice's host-first landing) — NOT faked here.

## 7. Additivity, complexity, docs

- [x] 7.1 BYTE-IDENTICAL vs the M2-assembly baseline: `half_space_cut.h`, `freeform_operand.h`,
      `face_split.h`, `freeform_membership.h`, `solid_mesher.h`, `ssi/marching.h`, `ssi_boolean.h`
      (`git diff` empty). `grep` finds 0 OCCT includes under `src/native/**`. No `cc_*` change.
- [x] 7.2 No new production function ⇒ backend cognitive-complexity band unaffected.
- [x] 7.3 `openspec/MOAT-ROADMAP.md`: M2-breadth row — COMMON landed at the host gate (partition
      identity + watertight-at-complementary-volume), the shared-curved-edge single-sampling M0 fix
      recorded as the robust-watertightness enabler, and FUSE recorded as the next breadth blocker.

## 8. Honest-decline fallback (if COMMON had not reached the host gate)

- [x] 8.1 NOT triggered — COMMON reached the host gate: the partition identity closes to machine
      precision and the complementary solid welds watertight at its closed-form volume at a
      both-sides-weld deflection. The measured NEXT enablers are recorded: the **shared-curved-edge
      single-sampling M0 fix** (robust deflection-independent watertightness for CUT and COMMON), the
      **SIM `BRepAlgoAPI_Common` parity harness** (§6), and **FUSE's two-operand inter-solid
      intersection verb** (§5) — the next breadth operator.
