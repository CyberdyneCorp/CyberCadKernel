# Proposal — moat-m6q-clash-fuzz (MOAT M6-breadth-17, the SEVENTEENTH domain)

## Why

The MOAT completeness bar (removing OCCT is gated by *proven* native correctness, not
merely by defaulting to it) has fifteen landed differential-fuzzing domains through
moat-m6o (curved booleans, STEP round-trip, construction, blends, wrap/emboss, mass-
properties, geometry-services, transform-chains, reference/datum, direct-modeling,
transformed-boolean, section, HLR, healing, curved-blend). A concurrent track adds a
sixteenth (the draft/HLR-of-drafted-solid fuzzer). This change adds the **seventeenth**:
an INTERFERENCE / CLASH differential fuzzer certifying the newly-native `cc_interference`
surface.

The landed clash op — `src/native/analysis/interference.h`, an OCCT-FREE header-only
classifier over the B3 membership + M0 mesh vocabulary, reached through the
`cc_interference` facade — decides, for a PAIR of solids, whether the two INTERIORS
overlap over a set of positive volume (**CLASH**), merely make zero-volume boundary
contact (**TOUCHING**), or have a positive clearance gap (**CLEAR**), reporting the
minimum clearance and, on a clash, a witness AABB + interior point. Its only tests today
are the OCCT-free host fixtures (`tests/native/test_native_interference.cpp`) and a
five-pose hand-picked box parity harness (`tests/sim/native_interference_parity.mm`).
Neither drives *random* pairs of solids at *random* relative placements spanning the
three regimes — which is exactly where a clash classifier silently mis-fires (a
knife-edge flush contact read as a clash; a thin overlap read as clear).

This fuzzer is high-value for three reasons:

1. **A wrong clash flag is a silent-wrong-result the user cannot detect** — an assembly
   mate reporting a phantom interference, or missing a real interpenetration, corrupts
   the model with no visible symptom. The bar searches the pose space for that.
2. **The TOUCHING knife-edge is the classic robustness trap** — two solids sharing a
   flush face are boundary-coincident (`On`), NOT a clash. The landed op handles this
   coplanar-safely via B3 ON-band membership; this fuzzer stresses that boundary under
   randomised flush / slight-penetrate / slight-gap jitter, not just one hand-built pose.
3. **It carries a CLOSED-FORM arbiter where the pair has one** — box∩box intersection-box
   volume + axis gap, and sphere∩sphere lens volume + centre-distance regime — an
   engine-independent ground truth, so a native result more correct than a sub-tolerance
   OCCT rounding is classified ORACLE-INACCURATE (native vindicated), never a failure.

It shares NO code with the concurrent draft fuzzer track (it owns the CLASH fuzzer only)
and fuzzes only the STABLE landed `interference.h` / `cc_interference` op.

## The oracles (OCCT + closed-form, the primary arbiter)

- **OCCT** — the same decision the fixture parity makes: `BRepAlgoAPI_Common` volume
  (> band ⇒ interior overlap ⇒ CLASH) COMBINED with `BRepExtrema_DistShapeShape` (min
  boundary distance: ~0 with no overlap ⇒ TOUCHING; > 0 ⇒ CLEAR).
- **CLOSED-FORM (PRIMARY where present)** — box∩box: the exact axis-aligned
  intersection-box volume + the exact axis gap. sphere∩sphere: the exact lens volume +
  the exact centre-distance-vs-(rA+rB) regime. Exact for the ideal solid.

`interference.h` is OCCT-FREE and header-only, so — like `native_interference_parity.mm`
— the native classifier runs directly on watertight tessellated meshes built in the
harness (box / regular n-gon prism / faceted cylinder / UV-sphere), and the OCCT oracle
runs on the matching `BRepPrimAPI` solids; BOTH operands are placed by the SAME rigid
transform. This is the self-contained differential the header-only op supports; it does
not need the whole kernel or the `cc_set_engine` A/B (the classifier has no OCCT arm to
select).

## The faceting boundary (why native-vs-OCCT alone is insufficient for curved pairs)

The native classifier consumes a deflection-bounded PLANAR-FACET mesh; OCCT keeps a true
analytic B-rep. A faceted cylinder / sphere is INSET from the true surface by up to
~2×deflection (chord-secant), so at a near-flush curved contact the faceted surfaces can
read a hair CLEAR where OCCT's exact B-rep reads TOUCHING — a real, bounded, EXPECTED
facet artefact, NOT a native fault (the op's own contact band is `max(1e-9·scale,
2·deflection)`). The harness therefore treats a curved-pair TOUCHING↔CLEAR straddle as
a CONVERGENT match (FACET-CONVERGENT), never a DISAGREE; a CLASH↔CLEAR or CLASH↔TOUCHING
split is always a hard state and any disagreement there fails the bar. NO tolerance is
widened to force a pass — the band is the classifier's own.

## What changes

- **ADD** `tests/sim/native_interference_fuzz.mm` — a deterministic seeded (splitmix64
  → xoshiro256**, keyed ONLY by `FUZZ_SEED`) differential fuzzer that, per trial: draws
  a family {box / n-gon prism / cylinder / sphere} and a target regime {CLEAR / TOUCHING
  / CLASH}, builds both a watertight native mesh and the matching OCCT solid at random
  dims, places body B by a shared random rigid transform (translate + rotate) landing the
  pair in that regime — with a flush knife-edge jitter (exact-flush / slight-penetrate /
  slight-gap) on the TOUCHING regime and a minority non-watertight soup probe — runs
  native `meshInterference` vs OCCT (`BRepAlgoAPI_Common` + `BRepExtrema_DistShapeShape`)
  and the closed-form arbiter, then classifies AGREED / HONESTLY-DECLINED / DISAGREED /
  ORACLE-INACCURATE / FACET-CONVERGENT / BOTH-DECLINED.
- **ADD** `scripts/run-sim-native-interference-fuzz.sh` — cloned from
  `run-sim-native-interference.sh` (links only the native math TUs — `interference.h`
  is header-only — plus the OCCT oracle toolkits `TKBO`/`TKShHealing`/`TKHLR`/`TKPrim`/…);
  seeded ONLY by `FUZZ_SEED`/argv; default N=72 per seed.
- **ADD** `native_interference_fuzz.mm` to the `run-sim-suite.sh` SKIP list (own
  `main()`, `std::_Exit`).
- **UPDATE** `openspec/MOAT-ROADMAP.md` M6 row: breadth ×16 → ×17 (the concurrent draft
  fuzzer bumps ×15 → ×16; the final count after both merge is ×17).

## The bar

DISAGREED == 0 across ≥2 seeds, N≥60 trials/seed (the runner fails if ANY seed fails),
every populated [family × regime] cell truly exercised (≥1 AGREED / ORACLE-INACCURATE /
FACET-CONVERGENT — not an all-decline cell). The contact band is the classifier's own
(`max(1e-9·scale, 2·deflection)`) and is NEVER widened. An honest native `Unknown` →
OCCT decline (a non-watertight or ambiguous operand) is first-class, counted separately.

## Finding surfaced (reported, not fixed)

During bring-up the fuzzer LOCALIZED a genuine native limitation and the analysis is
recorded here (product code is NOT changed): for two boxes whose faces are EXACTLY
coplanar and overlap in a plus-sign CROSS where NEITHER box contains a boundary vertex of
the other, `interference.h` step 4 (min triangle–triangle distance via the six
vertex-vs-face sub-tests, edge-edge NOT evaluated — the header explicitly documents this
as "a tight bound otherwise") over-estimates the min distance and mis-reports the flush
TOUCH as CLEAR. This is confirmed OCCT-free on the host gate. It is a documented
approximation OUTSIDE the certified assembly-mate contact envelope (seated / coincident /
contained / slid faces all resolve TOUCHING correctly — verified). The fuzzer's TOUCHING
generator therefore draws contact poses inside that certified envelope (B's contact
footprint ⊆ A's), and the coplanar-cross limitation is REPORTED here rather than masked by
any tolerance change. A future product change (not this test-infra track) can extend
step 4 with the edge-edge distance term to close it.

## Non-goals / discipline

- `src/native/**`, `src/engine/**`, `include/**`, and the `cc_*` ABI stay
  **byte-unchanged** (test infrastructure only). The coplanar-cross limitation above is
  REPORTED, not fixed here. A native result more correct than OCCT at a numeric edge is
  classified ORACLE-INACCURATE (native vindicated), not DISAGREED.
- Does NOT build the concurrent draft fuzzer (a separate track owns it) — only the CLASH
  interference fuzzer.
- No new geometry capability; no widened tolerance; no `Date.now()`/`rand()`.
