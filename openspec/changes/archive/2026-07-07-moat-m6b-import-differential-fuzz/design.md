# Design — moat-m6b-import-differential-fuzz

## Context

The M6 curved-boolean fuzzer (`native_boolean_fuzz.mm`) established the pattern: a
splitmix64/xoshiro256** seeded generator emits random-valid inputs; each input is run
through BOTH the native path and an OCCT oracle built from the SAME parameters; a
three-way classifier bins each trial AGREE / honestly-declined / DISAGREE; the process
exits non-zero if any DISAGREE (a silent wrong result). This change reuses that skeleton
verbatim for a second, independent domain — the native STEP reader — and adds one
principled refinement (a closed-form analytic arbiter) demanded by that domain's oracle.

The native STEP reader (`src/native/exchange/step_reader`) is the OCCT-free inverse of
the native STEP writer (`step_writer`). Both depend only on `src/native/{exchange,math,
topology,tessellate,construct}` — all OCCT-free — so the harness needs **no numsci**
(unlike the boolean fuzzer, whose SSI path does). The OCCT dependency is only the oracle
(`STEPControl_Reader` + `BRepGProp` + `BRepCheck`).

## The differential: native reader vs OCCT reader on the SAME bytes

Pipeline per generated native solid `S`:

```
S ──native writer──▶ file F ──┬─native reader (OCCT-free)─▶ natShape ─native mesh─▶ (volN, areaN, wt, solidsN)
                              └─OCCT STEPControl_Reader────▶ occShape ─BRepGProp──▶ (volO, areaO, valid, solidsO)
```

Both readers consume **one on-disk file** (identical bytes), mirroring the landed
`native_step_import_parity::runNativeWritten` discipline (import BOTH ways, compare).
The native reconstruction is measured by the native tessellator; the OCCT re-import is
measured exactly by `BRepGProp`. This asymmetry (native-mesh vs OCCT-exact) is the SAME
as the sibling boolean fuzzer, so the mesh is tessellated fine (deflection 0.001) to keep
a correct native import's inscribed-facet bias (≈2–4e-3) comfortably under the fixed
`relTol = 2e-2`, which is NEVER widened per-trial.

### Why generate through the cc_* facade's construct entry points

Curved solids are built with `build_revolution` on a **raw polygon** (the exact path
`cc_solid_revolve` takes: `{0,0, r,0, r,h, 0,h}` for a cylinder, `{0,0, r0,0, r1,h, 0,h}`
for a frustum), NOT `build_revolution_profile` with hand-authored `ProfileSegment`s. An
early prototype used explicit profile segments and produced revolution solids whose
native mesh tripped the tessellator's watertight-seam heuristic AND whose STEP the OCCT
oracle re-imported as an INVALID shape — i.e. the writer→OCCT round-trip was not a clean
oracle. Building through the proven facade path (`build_prism`, `build_revolution`
raw-polygon, `build_prism_with_holes`) makes the write→OCCT-read round-trip valid, so a
DISAGREE is attributable to the READER, not to a bad source or a marginal file.

## Decision: a closed-form analytic ground-truth ARBITER

**Problem.** The task frames OCCT as the oracle. But a pure native-vs-OCCT comparison
reflexively blames the native reader on any disagreement — even when OCCT is the one that
is wrong. Empirically, OCCT's `STEPControl_Reader` re-imports a **shallow native cone**
slightly inaccurately: seed 0x1234 index=10 (frustum r0=1.4211 r1=1.3651 H=2.0612) gives
native vol/area within 9e-4/5e-4 of the exact frustum (12.5688 / 30.2478) while OCCT is
2.7e-2 / 1.2e-2 HIGH. Blindly trusting OCCT would flag the CORRECT native import as a
silent wrong result — a false positive that papers over an OCCT limitation.

**Decision.** Every family here has a **closed-form volume + area** (box, regular prism,
cylinder, frustum, holed box). The harness computes the exact truth per case and uses it
to ATTRIBUTE a native-vs-OCCT disagreement:

- native watertight AND matches OCCT within tol → **AGREED**.
- native NULL / non-watertight → **HONESTLY-DECLINED** (OCCT fallback ships; oracle valid).
- native watertight but DIFFERS from OCCT:
  - native matches analytic ground truth AND OCCT does NOT → **ORACLE-INACCURATE**
    (native VINDICATED by exact math; oracle-side limitation; logged; NOT a bar failure).
  - otherwise (native fails analytic, or ambiguous) → **DISAGREED** (a genuine silent
    wrong native import; the bar failure; printed with seed + repro tuple).

**Why this is a strengthening, not a weakening.** A native result is exonerated ONLY when
it POSITIVELY matches independent exact math while OCCT does not; a genuine native error
(native ≠ analytic) still fails the bar. The `relTol` is never widened. The analytic
arbiter holds the native reader to a HIGHER standard (exact truth) than the OCCT
differential alone, while refusing to launder OCCT's own inaccuracy into a native fault.

### Guards (attributability)

- **ORACLE_UNRELIABLE** — the OCCT re-import is not a valid closed solid. For the scoped
  families this must never happen; if it does the file is not a trustworthy oracle →
  excluded from the reader verdict and FAILS the bar (investigate; never launder).
- **WRITER_DECLINE** — `canSerialize` false / empty emit. The generator reached past the
  writer's scope; the trial exercises no reader. Logged as a coverage drop (a silently-
  uncovered family cannot hide), NOT a bar failure — an honest writer decline is allowed.

The BAR: `DISAGREED == 0` AND `ORACLE_UNRELIABLE == 0`.

## Scope boundaries (honest, logged exclusions)

- **Sphere** (bare-periodic `SPHERICAL_SURFACE` / `VERTEX_LOOP`) is excluded: OCCT's
  re-import of the native VERTEX_LOOP sphere is inconsistent (spurious sub-solids /
  degenerate area), so OCCT is not a clean oracle for it in THIS write→read direction.
  Sphere import is covered by the curated foreign-authored `runRevolvedSphere` fixture.
- **Ruled loft** (bilinear `B_SPLINE_SURFACE`) is excluded: the native writer honestly
  declines to serialise it (`canSerialize` = false), so no reader trial exists.
- **Full-cone apex** (r1 = 0) is avoided in the frustum generator (both radii strictly
  positive) to keep the write→OCCT-read round-trip a clean oracle.

## Alternatives considered

- **Pure native-vs-OCCT with OCCT as sole oracle (no arbiter).** Rejected: it reports a
  false DISAGREE on the shallow-cone case where the native reader is provably correct and
  OCCT is the outlier. The differential is still computed and printed; the arbiter only
  changes ATTRIBUTION, never hides a real native error.
- **Comparing the native reader against the SOURCE solid's native mesh (writer self-
  check).** Rejected as the primary bar: the source, native writer, and native reader are
  all native, so a shared bug could agree spuriously. The independent oracles are OCCT and
  the closed-form analytic truth. (A source/OCCT faithfulness check remains implicit in
  the ORACLE validity gate.)
- **Widening `relTol` to absorb the shallow-cone OCCT error.** Rejected outright — it
  would weaken the tolerance and hide genuine wrong results. The arbiter attributes the
  error at a FIXED tolerance instead.

## Verification

- Built + run by `scripts/run-sim-native-step-import-fuzz.sh` on a booted iOS simulator.
- `DISAGREED == 0` across seeds 0x5744EE9911, 0xB16B00B5, 0x1234 (N=96 and N=128), with
  per-family coverage of box / prism / cylinder / frustum / holed-box, honest frustum
  declines, and one arbitrated ORACLE-INACCURATE on seed 0x1234.
- Determinism: two runs of seed 0x5744EE9911 produce byte-identical trial output.
- On `run-sim-suite.sh`'s SKIP list; `src/native/**` untouched and OCCT-free.
