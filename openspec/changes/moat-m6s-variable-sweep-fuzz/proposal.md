# Proposal — moat-m6s-variable-sweep-fuzz (MOAT M6, the VARIABLE-SECTION SWEEP domain)

## Why

The MOAT completeness bar (removing OCCT is gated by *proven* native correctness, not
merely by defaulting to it) has a growing set of landed differential-fuzzing domains —
curved booleans, STEP round-trip, construction/loft/sweep, blends, wrap/emboss, mesh
mass-properties, geometry-services, transform chains, reference/datum geometry,
direct-modeling, transformed-boolean, section curves, orthographic HLR, shape-healing,
curved-blend, draft-angle, and clash/interference.

A NEW native surface has landed: the **VARIABLE-SECTION / guide+spine sweep**
(`src/native/construct/sweep.h` `build_variable_sweep` / `build_variable_sweep_tube`) —
sweep a section that MORPHS from profile A (spine start) to profile B (spine end, SAME
vertex count) along a 3D polyline spine, each station the vertex-k→vertex-k linear blend
placed in the frame perpendicular to the spine, OPTIONALLY scaled uniformly by a guide
rail's splay (`scale(f) = dist(spine(f),guide(f)) / dist(spine(0),guide(0))`), reached
through the shipping `cc_variable_sweep` facade under the NativeEngine. This is a superset
of `cc_loft_along_rail` (adds the guide scale) and `cc_guided_sweep` (adds the A→B morph).
That path has a *curated* per-op parity harness (`native_vsweep_parity.mm` — four
hand-picked fixtures [circle→circle straight, constant-square straight, guide-scaled
square, circle→circle arc] plus one deferred non-planar case) but **no seeded differential
fuzzer** that drives *random* morph profiles × *random* spine × *random* optional guide
through the facade under BOTH engines and classifies every trial. This change closes
exactly that gap and certifies the freshly-landed variable-sweep surface at the facade
level.

The variable-sweep fuzzer is high-value for three reasons:

1. **It certifies a surface that just landed** — random circle→circle / polygon→polygon /
   section-A→section-B morphs × straight or smooth-curved-arc spine × optional +X-splaying
   guide rail, none previously under a *randomised* facade-level bar.
2. **A wrong sweep is a silent-wrong-result the user cannot detect** — a watertight-but-
   wrong morph tube hands the user a corrupted shaping boss with a wrong volume presented
   as a valid sweep. The bar searches the parameter space for that.
3. **It is the SHIPPING path, both engines.** It calls the exact public `cc_variable_sweep`
   facade the app calls, once under OCCT (`cc_set_engine(0)` →
   `BRepOffsetAPI_MakePipeShell` multi-section) and once under the NativeEngine
   (`cc_set_engine(1)` → `build_variable_sweep` with an honest OCCT fallback) — an A/B the
   per-op parity fixture does not randomise. It shares NO code with the concurrent
   freeform-boolean / N-fill / interference tracks; it fuzzes only the STABLE landed
   variable-sweep op.

## The oracles (closed-form the PRIMARY arbiter where it exists, OCCT the cross-check)

A **straight**-spine variable sweep's cross-section at fraction f∈[0,1] is a planar polygon
P(f) whose vertex k is `(A_k + (B_k−A_k)·f)·s(f)`, where `s(f)` is the guide-splay scale
(≡1 with no guide). Its signed area A(f) is a polynomial in f of degree ≤ 4 (the vertex
blend contributes ≤2, the guide-scale² contributes ≤2), so the exact volume is
`∫₀¹ A(f)·H df` where H is the straight spine length, evaluated by a **composite Simpson
over 4 sub-intervals (5 samples), analytically EXACT for a quartic** — the PRIMARY arbiter
for the four straight-spine families. For the circle-morph straight family the SMOOTH
truncated-cone volume `πH/3(r0²+r0r1+r1²)` is used; the native 64-gon profile inscribes it,
so the band absorbs the fixed inscription bias. Because the closed form is exact, a native
result matching it while OCCT is the outlier is logged ORACLE-INACCURATE (native
vindicated), never a bar failure.

The **OCCT oracle** is `cc_set_engine(0)` → OCCT `BRepOffsetAPI_MakePipeShell` multi-section
through the SAME `cc_variable_sweep` facade (same profiles, spine, guide), measured by
`cc_mass_properties`. The **circle-morph CURVED-arc** family has no simple closed form, so
it is arbitrated against OCCT ONLY (a deflection-bounded band) plus the engine-independent
watertight + Euler χ=2 invariants.

## What changes

- **ADD** `tests/sim/native_vsweep_fuzz.mm` — a deterministic seeded (splitmix64 →
  xoshiro256**, keyed ONLY by `FUZZ_SEED`) differential fuzzer that, per trial: generates
  random morph profiles + spine + optional guide for one of five families; sweeps them
  through the public `cc_variable_sweep` facade under BOTH engines; compares native vs the
  OCCT oracle AND (straight families) vs the closed-form volume; asserts watertight +
  Euler χ=2 + positive volume; reads the native BUILDER's decline signal
  (`build_variable_sweep` NULL) to keep the HONESTLY-DECLINED bucket meaningful; and
  classifies AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE /
  ORACLE_UNRELIABLE / BOTH-DECLINED. Interleaves a fixed cadence of out-of-envelope DECLINE
  probes (mismatched vertex counts, coincident guide start, collapsing guide scale,
  non-planar guided helix) so the honest-decline branch is exercised every run.
- **ADD** `scripts/run-sim-native-vsweep-fuzz.sh` — cloned from `run-sim-native-vsweep.sh`
  (links the WHOLE kernel — facade + core + engine[native+occt] + native math — plus the
  broad OCCT toolkit set; `TKHLR`/`TKShHealing` retained; NO numsci, as the native
  variable-sweep path is OCCT-free AND numsci-free); seeded ONLY by `FUZZ_SEED`/argv;
  default N=72 per seed; runs TWO default seeds and fails if either fails.
- **ADD** `native_vsweep_fuzz.mm` to the `run-sim-suite.sh` SKIP list (own `main()`,
  `std::_Exit`).
- **UPDATE** `openspec/MOAT-ROADMAP.md` M6 row: breadth ×17 → ×19 (this domain plus the
  concurrent freeform-boolean / N-fill tracks bump the same row; reconciled at merge to the
  final ×20).

## The bar

DISAGREED == 0 and ORACLE_UNRELIABLE == 0 across ≥2 seeds, N≥60 trials/seed (the runner
fails if ANY seed fails), each of the five families (circle-morph / polygon-morph /
section-A→B straight; guided straight; circle-morph curved) with ≥1 AGREED trial. The
FIXED tolerances (native-vs-closed-form volume ≤ 2e-3 for the polygon families — planar
straight sweep volume is exact; ≤ 1.2e-2 for the circle families to absorb the 64-gon
inscription of the smooth cone; native-vs-OCCT volume ≤ 5e-2, area ≤ 8e-2 to absorb OCCT
pipe-shell discretisation) are NEVER widened to force a pass. An honest native decline →
OCCT is first-class.

## Non-goals / discipline

- `src/native/**`, `src/engine/**`, `include/**`, and the `cc_*` ABI stay
  **byte-unchanged** (test infrastructure only). If the fuzzer surfaces a real native
  sweep bug it is REPORTED, not fixed here; a native result more correct than OCCT at a
  numeric edge is classified ORACLE-INACCURATE (native vindicated), not DISAGREED.
- Does NOT fuzz the concurrent freeform-boolean / N-fill / interference tracks nor any
  other op — only the STABLE landed `cc_variable_sweep`.
- No new geometry capability; no widened tolerance; no `Date.now()`/`rand()`.
