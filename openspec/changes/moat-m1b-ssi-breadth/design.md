# Design — moat-m1b-ssi-breadth

## Context

M1's "skew cyl∩cyl = NotAnalytic" is an **S1-layer** decision: S1 is the closed-form conic
dispatch (`src/native/ssi/quadric_pairs.h` → `parallelDirs(a1,a2) ? … : notAnalytic()`), and a
skew cyl∩cyl quartic genuinely has no closed form, so `NotAnalytic` is the honest, correct S1
answer. The intersection curve for these pairs is instead produced by the **S2 seeder** +
**S3 predictor-corrector marching tracer** (`seeding.cpp` / `marching.cpp`), which are surface-
agnostic (they consume only `point/dU/dV/normal` adapters). Those already handle general poses.

## Empirical finding (why no tracer change)

A host probe drove `seed_intersection` + `trace_intersection` on five general non-coaxial /
skew analytic quadric poses. All traced cleanly with `nearTangentGaps == 0` and a maximum
on-both-surfaces residual ≈ 1e-11:

| pose                                   | native result                    | max on-surf |
|----------------------------------------|----------------------------------|-------------|
| skew cyl∩cyl, gap 0.4, tilt 60°, R1/0.7| 1 closed loop (single quartic)   | 2.4e-11     |
| skew cyl∩cyl, gap 0.2, tilt ~45°, R1/0.5| 2 closed loops                  | 2.2e-11     |
| cyl∩cone off-axis oblique              | 1 boundary-exit branch           | 4.7e-11     |
| cone∩cone general                      | 1 closed loop                    | 2.9e-11     |
| sphere∩cyl off-axis                    | 1 closed loop                    | 2.5e-11     |
| sphere∩cone off-axis                   | 1 closed loop                    | 4.5e-11     |

The tracer is already robust on this family. The missing piece is a **regression lock** and a
**native-vs-OCCT oracle gate** — without them the roadmap decline is overstated and a future
edit could silently regress the poses. So this change is test + spec, not tracer code.

## Verification design (two gates, OCCT = oracle)

### Gate A — host analytic (OCCT-free), `test_native_ssi_marching.cpp`
Self-consistency + closure, no OCCT: every traced node lies on BOTH native surfaces
(closed-form point/axis distance) to ≤ 1e-9; `nearTangentGaps == 0`; the branch closes (a
`Closed` WLine) or exits the patch (`BoundaryExit`) as the pose dictates; the traced connected-
component count equals the analytically-known count for the pose (one connected loop when the
smaller quadric fully penetrates the larger with an oblique-but-single crossing region; two when
it enters and exits as two disjoint caps). This is the closed-form-where-available half of the
gate — for these quartics no closed intersection curve exists, so the host gate asserts the
strong self-consistency + closure facts (per the M1 gate rule: "otherwise assert self-consistency
+ closure").

### Gate B — sim native-vs-OCCT parity, `native_ssi_marching_parity.mm`
The decisive gate. Each pose is built identically as native adapters AND as OCCT `Geom_*`
surfaces, and routed through the **already-shipped `reportPair`** helper, which:
1. runs `GeomAPI_IntSS(sa, sb, 1e-7)`, classifies each OCCT line transversal/tangential via the
   oracle-side normal cross-product, and **welds** the transversal arcs into connected components
   (OCCT tolerance-splits a smooth loop into arcs; the weld re-joins them);
2. asserts native `tracedBranches == occtTransversalComponents == expectTransversal`
   (analytic truth), `nativeClosed == occtClosed`, densely-sampled native points on the OCCT
   locus ≤ onCurveTol and on both OCCT surfaces ≤ onSurfTol, and native arc-length ≈ OCCT
   arc-length within a relative `lengthTol`.

`expectTransversal` and the closed-count are cross-checked against BOTH the analytic truth and
the OCCT oracle inside `reportPair`, so a wrong native trace, a wrong oracle read, or a wrong
hand-set expectation all fail honestly.

### Cone / OCCT convention
Native `math::Cone{pos, radius=refRadius@v0, semiAngle}` maps to
`Geom_ConicalSurface(toOcctAx3(pos), semiAngle, refRadius)` — the same convention already used
by the shipped `pairConeApexS4e` case, so the two cone LOCI coincide (the v-parametrisation
differs but `GeomAPI_IntSS` compares the surfaces, not the charts).

## Honest-decline boundary (unchanged)

- S1 `intersect_surfaces` STILL returns `NotAnalytic` for skew cyl∩cyl / general cone∩cone /
  non-coaxial quadrics — correct (no closed form). `test_native_ssi.cpp`'s `NotAnalytic`
  assertions stay green; this change does not fake a closed form.
- The near-tangent / branch-saddle / coincident regimes remain the honest S4 gap
  (`nearTangentGaps > 0` → deferred to OCCT) — this change only covers the **transversal**
  general poses the marcher already resolves, and does not weaken any tolerance to force a pass.
- Any general pose that turns out to graze near-tangent on the sim (OCCT still yields a curve but
  native truncates) is reported as the honest decline, not padded.

## Alternatives considered

- **Add a closed-form skew-cyl handler to S1** — rejected: a skew cyl∩cyl is a genuine quartic
  space curve with no elementary closed form; forcing one would fabricate. The marcher is the
  correct path and already works.
- **A brand-new parity `.mm` + runner** — rejected in favour of extending the existing
  `native_ssi_marching_parity.mm` + `run-sim-native-ssi-marching.sh` (already on the
  `run-sim-suite.sh` SKIP list), which additively reuses `reportPair` and the OCCT oracle slice.
