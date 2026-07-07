# Design — moat-m6c-construction-differential-fuzz

## Context

The M6 curved-boolean fuzzer (`native_boolean_fuzz.mm`) established the pattern: a
splitmix64/xoshiro256** seeded generator emits random-valid inputs; each input is run
through BOTH the native path and an OCCT oracle built from the SAME parameters; a
classifier bins each trial AGREE / honestly-declined / DISAGREE; the process exits
non-zero if any DISAGREE. M6b (`native_step_import_fuzz.mm`) reused that skeleton for the
STEP reader and added the closed-form analytic arbiter. This change reuses BOTH for a
third, independent domain — the native swept-solid construction library (loft + sweep).

The native construction library (`src/native/construct` — `loft.h`, `sweep.h`) depends
only on `src/native/{construct,tessellate,topology,math}` — all OCCT-free and header-only
(math `bezier.cpp` / `bspline.cpp` are the only compiled native TUs) — so the harness
needs **no numsci**. The OCCT dependency is only the oracle (`BRepOffsetAPI_ThruSections`
+ `BRepOffsetAPI_MakePipe` + `BRepGProp` + `BRepCheck`).

## Decision: call the native builders DIRECTLY, not through the cc_* facade

The curated parity harnesses (`native_loft_parity` / `native_sweep_parity`) drive the
`cc_*` facade with `cc_set_engine(1)` and rely on the NativeEngine's internal self-verify
to forward a declined build to OCCT. That is the right shape for a curated fixture, but it
is a POOR differential substrate: when the native path declines it SILENTLY forwards to
OCCT, so engine-0 and engine-1 return the SAME OCCT solid and trivially "agree" — you
cannot tell a native-handled build from a fall-through, so a fuzzer over the facade cannot
measure real native coverage.

This harness therefore calls the OCCT-free native builders DIRECTLY
(`ncst::build_loft_sections`, `ncst::build_sweep`) so a NULL / non-watertight result is an
UNAMBIGUOUS native DECLINE, and invokes OCCT SEPARATELY as the reference oracle
(replicating the exact idiom `src/engine/occt/occt_construct.cpp` uses:
`ThruSections(solid, ruled)` over the section polygons; `MakePipe` of the centroid-centred
profile face along the spine polyline). This mirrors the M6b design (native reader called
directly, OCCT `STEPControl_Reader` as the separate oracle).

## The differential: native builder vs OCCT builder on the SAME inputs

```
input ──┬─ native builder (OCCT-free) ─▶ natShape ─native mesh─▶ (volN, areaN, watertight, solidsN)
        └─ OCCT ThruSections / MakePipe ─▶ occShape ─BRepGProp──▶ (volO, areaO, valid, closed, solidsO)
```

The native result is measured by the native tessellator; the OCCT build is measured
exactly by `BRepGProp`. This asymmetry (native-mesh vs OCCT-exact) is the SAME as the two
siblings, so the mesh is tessellated fine (deflection 0.001). Crucially, the fuzzer's
arbitrated AGREE families are chosen so that asymmetry is ZERO: a planar parallel-plane
frustum and a straight prism have planar faces, so the native inscribed mesh reproduces
the EXACT solid — every AGREE lands within ~1e-15 of OCCT, far under the fixed
`relTol = 2e-2` (never widened per-trial), while a genuine wrong build would be off by a
large margin.

### Native self-verify, reproduced honestly

The native engine wraps every construction candidate in a MANDATORY self-verify
(watertight + sane enclosed volume) and DISCARDS a non-watertight candidate → OCCT. Since
this harness calls the raw builder, it reproduces that self-verify itself: a native result
that is NULL **or** whose native mesh is not watertight is classified HONESTLY-DECLINED
(the engine would forward it to OCCT). This is exactly the discipline
`native_loft_parity`'s header documents for the T-junctioning N-section spool.

## Decision: a closed-form analytic ground-truth ARBITER

Every arbitrated AGREE family has a closed-form volume + area:

- A **PRISMATOID loft** between coaxial regular n-gons is a stack of pyramidal frustums —
  band volume `(Δz/3)·(A(R_k) + √(A(R_k)·A(R_{k+1})) + A(R_{k+1}))`, and because each side
  face is a planar trapezoid the lateral area is closed-form too (edge/apothem of the
  regular n-gon). The T1 mismatched-count case carries the analytic of its regular-n-gon
  OUTLINE — the collinear resampled vertices change neither volume nor area.
- A **PRISM sweep** along a straight path is `profileArea·pathLength` with surface area
  `2·profileArea + profilePerimeter·pathLength`.

The harness computes the exact truth per case and uses it to ATTRIBUTE a native-vs-OCCT
disagreement:

- native watertight AND matches OCCT within tol → **AGREED**.
- native NULL / non-watertight → **HONESTLY-DECLINED** (OCCT fallback ships; oracle valid).
- native watertight but DIFFERS from OCCT:
  - native matches analytic AND OCCT does NOT → **ORACLE-INACCURATE** (native VINDICATED by
    exact math; oracle-side limitation; logged; NOT a bar failure).
  - otherwise → **DISAGREED** (a genuine silent wrong native build; the bar failure).

**Why this is a strengthening, not a weakening.** A native result is exonerated ONLY when
it POSITIVELY matches independent exact math while OCCT does not; a genuine native error
(native ≠ analytic) still fails the bar; the `relTol` is never widened. Unlike the M6b
STEP reader (where a shallow-cone re-import made OCCT the outlier), the construction AGREE
families are planar/prism and both sides are exact, so no ORACLE-INACCURATE was observed
in practice — the arbiter is a present-and-ready safety net that holds the native builder
to a HIGHER standard than the OCCT differential alone, and would fire the moment OCCT
mis-measured a case the native builder got right.

### Guards (attributability)

- **ORACLE_UNRELIABLE** — for a CORE family the OCCT build must be a valid closed solid; if
  it is not, the input is not a trustworthy oracle → excluded from the verdict and FAILS
  the bar (investigate; never launder).
- **BOTH-DECLINED** — a DECLINE-exerciser (out-of-scope input) where native returned NULL
  AND OCCT also built no valid solid: neither engine produced a wrong result, so there is
  nothing to compare. Logged, NOT a bar failure. (This split keeps the oracle-validity bar
  STRICT for the in-scope core families while allowing the deliberately out-of-scope
  exercisers to be refused by both engines.)

The BAR: `DISAGREED == 0` AND core-family `ORACLE_UNRELIABLE == 0`.

## Scope boundaries (honest, logged exclusions)

- **Twisted / rotated-section loft** (bilinear hyperbolic-paraboloid side faces): the
  native mesh is inscribed while OCCT measures the exact ruled surface, so native-vs-OCCT
  is only deflection-bounded — excluded from the seeded comparison, covered by
  `native_loft_parity::buildRotatedSquareTwist`.
- **Smooth-curved planar sweep** (constant-frame ruled tube): native constant-frame vs OCCT
  `MakePipe` agree only deflection-bounded — excluded, covered by
  `native_sweep_parity::buildSmoothArcSweep`.
- **N-section stacks with per-band mismatched taper ratios**: geometrically exact but the
  native tessellator T-junctions the shared ring (documented in `native_loft_parity`'s
  header), so the engine self-verify discards them → OCCT. The generator draws the N-section
  family mostly from the welding sub-families (prisms; symmetric 3-spools) for real AGREE
  coverage and leaves a fraction free to naturally exercise that self-verify DECLINE.

## Alternatives considered

- **Fuzz through the `cc_*` facade (like the parity harnesses).** Rejected: a native
  decline silently forwards to OCCT, so a fuzzer cannot distinguish native-handled from
  fall-through — no real native coverage is measured. Direct native calls give an
  unambiguous DECLINE signal.
- **Include twisted lofts / curved sweeps in the seeded AGREE set.** Rejected: their
  native-mesh-vs-OCCT-exact match is only deflection-bounded, which at a fixed 2e-2 relTol
  blurs AGREE vs DISAGREE. Kept in the curated parity harnesses (deflection-scaled tol)
  and documented as an honest domain-level exclusion.
- **Widening `relTol` to absorb the inscribed-mesh bias of curved families.** Rejected
  outright — it would weaken the tolerance and hide genuine wrong results. The fuzzer
  instead restricts its AGREE families to the exact (planar/prism) ones and keeps the
  arbiter at a FIXED tolerance.
- **Comparing the native build against its own analytic only (no OCCT).** Rejected as the
  primary bar: OCCT is the required oracle and a native-only check could agree with a
  shared native bug. The analytic is the independent ARBITER layered on top of the OCCT
  differential, not a replacement for it.

## Verification

- Built + run by `scripts/run-sim-native-construct-fuzz.sh` on a booted iOS simulator.
- `DISAGREED == 0` across seeds 0x5744EE9911 (N=96) and 0xABCDEF12345 / 0x99 /
  0xDEADBEEFCAFE (N=128), with per-family coverage of loft2-frustum / loftN-prismatoid-
  stack / loft2-mismatched-count / sweep-straight-prism (AGREED), the two non-planar
  exercisers and free N-section stacks (HONESTLY-DECLINED); zero ORACLE_UNRELIABLE.
- Determinism: two runs of seed 0x99 (N=64) produce byte-identical trial output.
- On `run-sim-suite.sh`'s SKIP list; `src/native/**` untouched and OCCT-free.
