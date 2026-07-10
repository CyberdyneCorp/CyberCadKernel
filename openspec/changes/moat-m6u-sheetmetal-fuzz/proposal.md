# Proposal — moat-m6u-sheetmetal-fuzz (MOAT M6-breadth, the SHEET-METAL domain)

## Why

The MOAT completeness bar (removing OCCT is gated by *proven* native correctness, not
merely by defaulting to it) has a growing set of landed differential-fuzzing domains —
curved booleans, STEP round-trip, construction/loft/sweep, blends, wrap/emboss, mesh
mass-properties, geometry-services, transform chains, reference/datum geometry,
direct-modeling, transformed-boolean, section curves, orthographic HLR, shape-healing,
curved-blend, draft-angle, interference, freeform-boolean, variable-section sweep, and
N-sided fill.

A NEW native surface landed: the constant-thickness **SHEET-METAL first slice**
(`src/native/sheetmetal/{base_flange,edge_flange,unfold,common}.h`) — a flat BASE flange
(the 2D profile extruded by the sheet gauge), a single EDGE flange with a cylindrical BEND
off one straight rim, and the flat-pattern UNFOLD of that single-bend part — reached
through the shipping `cc_sheet_base_flange` / `cc_sheet_edge_flange` / `cc_sheet_unfold`
facade under the NativeEngine. That path has a *curated* self-test
(`native_sheetmetal_selftest.mm`, five hand-picked fixtures) but **no seeded differential
fuzzer** that drives *random* profiles / thicknesses / edge selections / bend radii /
angles / k-factors through the facade and classifies every trial. This change closes
exactly that gap and certifies the freshly-landed sheet-metal surface at the facade level.

## OCCT is NOT the oracle — closed form + a round-trip invariant + validity are the arbiters

**OCCT core has NO sheet-metal module.** UNLIKE every other native fuzzer in this campaign,
this harness therefore does NOT compare against OCCT and does NOT drive `cc_set_engine(0)`.
The sheet-metal ops are NATIVE-ONLY; a case the native builder cannot robustly build
HONEST-DECLINES (returns 0, `cc_last_error` set), and is NEVER forwarded to OCCT nor faked.
The ARBITER is CLOSED FORM + INVARIANTS, ground truth by construction:

- **BASE FLANGE** — native volume == `|profileArea|·thickness` (EXACT; a planar prism meshes
  exactly, a hard equality at the fp floor).
- **EDGE FLANGE (fold)** — native volume == base + bend + wall =
  `L·W·t + ½·θ·((r+t)²−r²)·W + height·t·W`. The bend is a TRUE cylinder meshed to a
  deflection, so the meshed volume CONVERGES FROM BELOW to the closed form; the AGREE band is
  the SAME 1.5% the product's own `common.h::verifySolid` already gates at (a fixed deflection
  bound, NEVER widened), and the native volume must not EXCEED the closed form.
- **fold→unfold AREA INVARIANT** (the load-bearing round-trip check) — the developed
  flat-blank area == `baseArea + BA·W + flangeArea` with the bend allowance
  `BA = θ·(r + k·t)` (the k-factor formula). The unfold blank is a planar prism, so its
  volume == `devArea·t` EXACTLY; the invariant is asserted both as the native blank volume
  vs the closed-form developed area AND as the additive decomposition
  (`baseArea + bendDevelopedLen·W + flangeArea`) matching the direct `devLength·W` — i.e.
  the developed footprint area is invariant under fold→unfold.
- **VALIDITY** — every built part is a valid CLOSED 2-MANIFOLD: watertight, consistently
  oriented, non-degenerate, Euler χ=2 (a single genus-0 lump), positive volume — read through
  the facade from `cc_check_solid`'s per-check breakdown + `cc_mass_properties`, the SAME
  contract the product's own `verifySolid` and the host Gate (a) enforce.

## What changes

- **ADD** `tests/sim/native_sheetmetal_fuzz.mm` — a deterministic seeded (splitmix64 →
  xoshiro256**, keyed ONLY by `FUZZ_SEED`) differential fuzzer that, per trial: builds a
  random base flange (rectangle / regular n-gon / convex-jittered n-gon × random thickness);
  builds a random edge flange off the +X straight rim of a random `L×W×t` base (random bend
  radius, angle, wall height); and unfolds that folded part at a random k-factor ∈[0,1].
  Each op is measured through the shipping `cc_sheet_*` facade under the NATIVE engine and
  arbitrated by the closed form + the fold→unfold area invariant + closed-2-manifold validity,
  classified AGREED / HONESTLY-DECLINED / DISAGREED / NATIVE-CHECK-INACCURATE. Out-of-slice
  decline probes (wrong/non-straight edge id, degenerate profile, bend angle outside
  (0°,180°), unfold of a non-fold body) exercise the native NULL honest-decline branch.
- **ADD** `scripts/run-sim-native-sheetmetal-fuzz.sh` — cloned from
  `run-sim-native-sheetmetal.sh` + the two-seed loop of `run-sim-native-ngon-fill-fuzz.sh`
  (links the WHOLE kernel — facade + core + engine[native+occt adapter] + native math — plus
  the OCCT toolkit set, `TKHLR`/`TKShHealing` retained, so the facade's
  `create_default_engine` resolves under `-DCYBERCAD_HAS_OCCT`; the harness never enters an
  OCCT path). The sheet-metal path is OCCT-FREE and NOT `CYBERCAD_HAS_NUMSCI`-gated, so it
  needs NO numsci substrate. Seeded ONLY by `FUZZ_SEED`/argv; default N=72 per seed; runs
  TWO default seeds and fails if either fails.
- **ADD** `native_sheetmetal_fuzz.mm` to the `run-sim-suite.sh` SKIP list (own `main()`,
  `std::_Exit`).
- **UPDATE** `openspec/MOAT-ROADMAP.md` M6 row: breadth ×20 → ×21 (this domain; the
  concurrent wrap-emboss fuzzer also bumps it — reconcile at merge).

## The bar

DISAGREED == 0 across ≥2 seeds, N≥60 trials/seed (the runner fails if ANY seed fails), each
of the three ops (base-flange / edge-flange-fold / unfold) with ≥1 geometrically-correct
trial. The FIXED tolerances (planar-prism volume vs closed form ≤ 1e-6 — base + unfold blank
areas are exact; curved-bend meshed volume vs closed form ≤ 1.5% — the same band
`verifySolid` uses, converging from below; fold→unfold area-invariant residual ≤ 1e-6) are
NEVER widened to force a pass. A "DISAGREE" = native volume ≠ closed form OR the fold→unfold
area invariant violated OR a built part that is NOT a valid closed 2-manifold. An honest
native NULL on an out-of-slice pose is first-class, counted separately, never a bar failure.

## Localized finding — a PRE-EXISTING GS6 self-intersection false positive on the bend

The fuzzer surfaced (and localized) a genuine, PRE-EXISTING product interaction: on the
edge-flange FOLD, `cc_check_solid`'s GS6 `no_self_intersection` sub-check FALSE-POSITIVES —
it reports the otherwise-valid folded part as self-intersecting (`first_failure` code 5)
because the bend is a fan of near-coplanar planar strips approximating a true cylinder and
adjacent tight-bend facets read as self-intersecting to the M0-mesh detector. This
reproduces on the BASE commit with the LANDED code — `native_sheetmetal_selftest.mm`'s
`edge_flange cc_check_solid valid` line FAILs there too (10 passed / 1 failed) — so it is a
GS6-vs-tessellated-cylinder interaction, NOT a sheet-metal fold-geometry fault and NOT
introduced by this test-infra change. The folded part is CORRECT by every geometric arbiter
this fuzzer owns (watertight / consistently-oriented / χ=2 / volume==closed-form). Per the
track discipline this is REPORTED (a future product GS6 fix), NOT fixed here, and classified
NATIVE-CHECK-INACCURATE (the no-OCCT-oracle analogue of the sibling fuzzers'
ORACLE-INACCURATE) — logged, measured, never a bar DISAGREE.

## Non-goals / discipline

- `src/native/**`, `src/engine/**`, `include/**`, and the `cc_*` ABI stay
  **byte-unchanged** (test infrastructure only). The GS6 self-X false positive is REPORTED,
  not fixed here.
- Does NOT compare against OCCT (there is no OCCT sheet-metal oracle); does NOT fuzz the
  concurrent wrap-emboss / sweep / interference tracks nor any other op — only the STABLE
  landed `cc_sheet_*` ops.
- No new geometry capability; no widened tolerance; no `Date.now()`/`rand()`.
