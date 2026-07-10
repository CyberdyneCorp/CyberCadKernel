# Proposal — moat-m6p-draft-fuzz (MOAT M6-breadth-16, the SIXTEENTH domain)

## Why

The MOAT completeness bar (removing OCCT is gated by *proven* native correctness, not
merely by defaulting to it) currently has **fifteen** landed differential-fuzzing
domains — curved booleans, STEP round-trip, construction/loft/sweep, blends,
wrap/emboss, mesh mass-properties, geometry-services, transform chains,
reference/datum geometry, direct-modeling, transformed-boolean, section curves,
orthographic HLR, shape-healing, and curved-blend.

This session landed a NEW native surface: the molding/manufacturing **DRAFT ANGLE**
(`src/native/feature/draft_faces.h`) — taper one or more PLANAR side faces of a
prismatic solid about a planar NEUTRAL plane by a draft angle θ along a PULL direction,
reached through the shipping `cc_draft_faces` facade under the NativeEngine. That path
has a *curated* per-op parity harness (`native_draft_faces_parity.mm`, three hand-picked
fixtures — box +x wedge, box 4-side frustum, off-axis wedge — plus one cap-face decline)
but **no seeded differential fuzzer** that drives *random* valid prismatic solids and
*random* draft poses through the facade under BOTH engines and classifies every trial.
This change closes exactly that gap and certifies the freshly-landed draft surface at the
facade level.

The draft fuzzer is high-value for three reasons:

1. **It certifies a surface that just landed** — random prismatic bases (box / n-gon
   prism) × random drafted-face subsets × random neutral-plane pose × random draft angle,
   none previously under a *randomised* facade-level bar.
2. **A wrong draft is a silent-wrong-result the user cannot detect** — a watertight-but-
   wrong drafted solid hands the user a corrupted mold-release feature with a wrong
   volume presented as a valid taper. The bar searches the parameter space for that.
3. **It is the SHIPPING path, both engines.** It calls the exact public `cc_draft_faces`
   facade the app calls, once under OCCT (`cc_set_engine(0)` →
   `BRepOffsetAPI_DraftAngle`) and once under the NativeEngine (`cc_set_engine(1)` →
   `feature::draftFaces` with an honest OCCT fallback) — an A/B the per-op parity fixture
   does not randomise. It shares NO code with the concurrent CLASH fuzzer track (which
   owns `cc_interference`); it fuzzes only the STABLE landed draft op.

## The oracles (closed-form the PRIMARY arbiter, OCCT the cross-check)

The neutral plane is ALWAYS the base plane (origin (0,0,0), pull +Z) and every drafted
face is a vertical prism wall, so the drafted solid's cross-section at height z is the
footprint polygon with EACH drafted edge's supporting line pushed INWARD by z·tanθ
(non-drafted edges fixed). The exact drafted VOLUME is therefore

    V = ∫₀^h A(z) dz,   A(z) = area( footprint clipped by the inward-shifted drafted
                                     half-planes ).

A(z) is a polynomial of degree ≤ 2 in z, so a 3-point Simpson quadrature over [0,h] is
**analytically exact** — the PRIMARY arbiter, exact for the ideal prismatic draft,
handling adjacent-face corner interactions exactly via polygon clipping (validated
against the parity harness's box wedge `V = 1000 − 500·tan8°` and 4-side frustum closed
forms). Because it is exact, a native result matching the closed form while OCCT is the
outlier is logged ORACLE-INACCURATE (native vindicated), never a bar failure.

The **OCCT oracle** is `cc_set_engine(0)` → OCCT `BRepOffsetAPI_DraftAngle` through the
SAME `cc_draft_faces` facade (same face ids, same neutral/pull/angle), measured by
`cc_mass_properties`. The drafted AREA has no simple closed form (tapered walls + shrunk
top cap), so AREA is cross-checked against OCCT only, never against a fabricated analytic
area.

## What changes

- **ADD** `tests/sim/native_draft_faces_fuzz.mm` — a deterministic seeded (splitmix64 →
  xoshiro256**, keyed ONLY by `FUZZ_SEED`) differential fuzzer that, per trial: builds a
  random VALID prismatic solid (box / regular n-gon prism, n∈[3,8]) at random dimensions
  through the ACTIVE engine's public `cc_solid_extrude`; selects a random subset of
  planar side faces (SINGLE-face and MULTI-face families) resolved per engine; drafts
  them by a random valid angle about the base plane through `cc_draft_faces`; compares
  native vs the OCCT oracle AND vs the closed-form drafted volume; asserts watertight +
  Euler χ=2 + volume STRICTLY smaller than the base (a draft removes stock); and
  classifies AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE /
  ORACLE_UNRELIABLE / BOTH-DECLINED.
- **ADD** `scripts/run-sim-native-draft-faces-fuzz.sh` — cloned from
  `run-sim-native-directmodel-fuzz.sh` (links the WHOLE kernel — facade + core +
  engine[native+occt] + native math — plus the numsci substrate the native draft's
  inward split-plane cut needs, and `TKHLR`/`TKShHealing`); seeded ONLY by
  `FUZZ_SEED`/argv; default N=72 per seed; runs TWO default seeds and fails if either
  fails.
- **ADD** `native_draft_faces_fuzz.mm` to the `run-sim-suite.sh` SKIP list (own
  `main()`, `std::_Exit`).
- **UPDATE** `openspec/MOAT-ROADMAP.md` M6 row: breadth ×15 → ×16.

## The bar

DISAGREED == 0 and ORACLE_UNRELIABLE == 0 across ≥2 seeds, N≥60 trials/seed (the runner
fails if ANY seed fails), each of the four families (box/ngon × single/multi-face) with
≥1 AGREED trial. The FIXED tolerances (native-vs-closed-form volume ≤ 1e-3 — planar draft
volume is exact; native-vs-OCCT volume ≤ 2e-2, area ≤ 3e-2 to absorb OCCT draft-build
discretisation) are NEVER widened to force a pass. An honest native NULL → OCCT decline
is first-class.

## Non-goals / discipline

- `src/native/**`, `src/engine/**`, `include/**`, and the `cc_*` ABI stay
  **byte-unchanged** (test infrastructure only). If the fuzzer surfaces a real native
  draft bug it is REPORTED, not fixed here; a native result more correct than OCCT at a
  numeric edge is classified ORACLE-INACCURATE (native vindicated), not DISAGREED.
- Does NOT fuzz `cc_interference` (the concurrent CLASH fuzzer track) nor any other op —
  only the STABLE landed `cc_draft_faces`.
- No new geometry capability; no widened tolerance; no `Date.now()`/`rand()`.
