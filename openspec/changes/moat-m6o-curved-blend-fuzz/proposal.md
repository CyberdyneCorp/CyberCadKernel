# Proposal — moat-m6o-curved-blend-fuzz (MOAT M6-breadth-15, the FIFTEENTH domain)

## Why

The MOAT completeness bar (removing OCCT is gated by *proven* native correctness, not
merely by defaulting to it) currently has **fourteen** landed differential-fuzzing
domains — curved booleans, STEP round-trip, construction/loft/sweep, blends,
wrap/emboss, mesh mass-properties, geometry-services, transform chains,
reference/datum geometry, direct-modeling, transformed-boolean, section curves,
orthographic HLR, and shape-healing.

This session landed a large NEW native surface: the analytic **CURVED-BLEND** paths —
`src/native/blend/{curved_fillet.h, curved_shell.h, curved_offset.h, canal_fillet.h}` —
reached through the shipping `cc_fillet_edges` / `cc_shell` / `cc_offset_face` facade
under the NativeEngine. Those paths have *curated* per-op parity harnesses
(`native_curved_{fillet,shell,offset}_parity.mm`, hand-picked fixtures) but **no seeded
differential fuzzer** that drives *random* valid analytic-revolve solids through the
facade under BOTH engines and classifies every trial. The 4th M6 domain
(`native_blend_fuzz.mm`) fuzzed the native blend BUILDERS directly on a box edge / one
cylinder rim and **explicitly declared the curved SHELL and curved OFFSET an "honest
domain-level decline for this first blend slice"**. This change closes exactly that gap
and certifies the freshly-landed curved-blend surface at the facade level.

The curved-blend fuzzer is high-value for three reasons:

1. **It certifies a large surface that just landed** — three ops (fillet/shell/offset)
   over three curved walls (cylinder/cone/sphere), nine families, none of which was
   previously under a *randomised* facade-level bar.
2. **A wrong curved blend is a silent-wrong-result the user cannot detect** — a
   watertight-but-wrong filleted/shelled/offset solid hands the user a corrupted model
   with a wrong volume presented as a valid feature. The bar searches the parameter
   space for that.
3. **It is the SHIPPING path, both engines.** It calls the exact public `cc_*` facade
   the app calls, once under OCCT (`cc_set_engine(0)`) and once under the NativeEngine
   (`cc_set_engine(1)`) — an A/B the per-op parity fixtures do not randomise. It shares
   NO code with the concurrent tracks: it does NOT touch `cc_variable_sweep`
   (src/native/construct, a separate track) nor the in-progress `src/native/sheetmetal`
   module; it fuzzes only the STABLE landed curved-blend ops.

## The oracles (OCCT + closed-form, the third arbiter)

Every family carries a **CLOSED-FORM** analytic volume for the ideal solid — the
PRIMARY arbiter, exact independent of either engine — so a native result matching the
closed form while OCCT is the outlier is logged ORACLE-INACCURATE (native vindicated),
never a bar failure. The OCCT oracle is:

- **FILLET / SHELL** — OCCT through the facade (`cc_set_engine(0)` → `BRepFilletAPI` /
  `BRepOffsetAPI_MakeThickSolid`), measured by `cc_mass_properties`.
- **OFFSET** — built DIRECTLY with OCCT (`BRepPrimAPI_MakeCylinder/MakeCone/MakeSphere`
  at the offset radius, `BRepGProp`), because the SHIPPED OCCT `cc_offset_face` is
  PLANAR-ONLY and honestly declines a curved wall — precisely the M3 residual the
  native arm fills. The cone wall offsets by the true perpendicular distance so both cap
  radii shift by `d/cosσ` (NOT by `d`); the closed-form and the direct-OCCT oracle both
  use `d/cosσ`, matching `curved_offset.h`.

## What changes

- **ADD** `tests/sim/native_curved_blend_fuzz.mm` — a deterministic seeded (splitmix64
  → xoshiro256**, keyed ONLY by `FUZZ_SEED`) differential fuzzer that, per trial: builds
  a random VALID analytic-revolve base solid (capped cylinder / cone frustum / sphere-cap
  dome) at random valid parameters through the ACTIVE engine's public facade; applies one
  of the nine curved-blend families {FILLET, SHELL, OFFSET} × {cyl, cone, sphere} through
  `cc_fillet_edges` / `cc_shell` / `cc_offset_face`; compares native vs the OCCT oracle
  AND vs the closed-form volume; asserts watertight + Euler χ=2 + correct grow/shrink
  direction; and classifies AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE /
  ORACLE_UNRELIABLE / BOTH-DECLINED.
- **ADD** `scripts/run-sim-native-curved-blend-fuzz.sh` — cloned from
  `run-sim-native-curved-offset.sh` (links the WHOLE kernel — facade + core +
  engine[native+occt] + native math — for the facade path, plus `TKHLR`/`TKShHealing`);
  seeded ONLY by `FUZZ_SEED`/argv; default N=72 per seed.
- **ADD** `native_curved_blend_fuzz.mm` to the `run-sim-suite.sh` SKIP list (own
  `main()`, `std::_Exit`).
- **UPDATE** `openspec/MOAT-ROADMAP.md` M6 row: breadth ×14 → ×15.

## The bar

DISAGREED == 0 and ORACLE_UNRELIABLE == 0 across ≥2 seeds, N≥60 trials/seed (the runner
fails if ANY seed fails), each of the nine core families with ≥1 AGREED trial. The FIXED
deflection-convergence tolerances (native-vs-OCCT volume ≤ 2e-2, native-vs-closed-form
volume ≤ 2e-2, area ≤ 4e-2 — the same bands the per-op curved parity harnesses validated)
are NEVER widened to force a pass. An honest native NULL → OCCT decline is first-class.

## Non-goals / discipline

- `src/native/**`, `src/engine/**`, `include/**`, and the `cc_*` ABI stay
  **byte-unchanged** (test infrastructure only). If the fuzzer surfaces a real native
  curved-blend bug it is REPORTED, not fixed here; a native result more correct than
  OCCT at a numeric edge is classified ORACLE-INACCURATE (native vindicated), not
  DISAGREED.
- Does NOT fuzz `cc_variable_sweep` (src/native/construct — a separate concurrent track)
  nor the in-progress `src/native/sheetmetal` module — only the STABLE landed
  curved-blend ops.
- No new geometry capability; no widened tolerance; no `Date.now()`/`rand()`.
