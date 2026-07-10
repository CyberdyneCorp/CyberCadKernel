# Design — moat-m6n-healing-fuzz

## Context

The M6 completeness bar drives random valid inputs through BOTH the native path and the
OCCT oracle and classifies every trial, requiring `DISAGREED == 0`. Thirteen domains
have landed; HEALING (`heal::healShell`) is the last-named remaining one. The curated
`native_heal_parity.mm` already proves both engines on hand-picked fixtures; this change
turns those fixtures into a *seeded, randomized batch* with a per-trial classifier.

## The third-oracle problem (why healing was deferred, and how this solves it)

`moat-m6h` deferred healing because a heal's "correct" output is a heuristic agreement
with no closed-form ground truth, so a native-vs-OCCT disagreement could not be
attributed. This design removes that objection with a single principle:

> **Inject defects into a solid whose exact geometry is already known.**

Every base solid is built by the harness itself (unit cube, axis-aligned box, convex
N-gon prism), so its enclosed volume and surface area are exact closed forms. Every
defect family is *shape-preserving* — it perturbs only the B-rep representation
(vertex jitter, face orientation, a redundant vertex/edge, a near-miss gap, a missing
face the cap pass restores) and never the intended solid. Therefore a *correct* heal
reconstructs the ORIGINAL solid, and the closed-form volume+area is a THIRD,
engine-independent arbiter. A native `Healed` solid is AGREED only when it matches that
closed form (and OCCT concurs); a watertight native solid that misses the closed form
is a genuine DISAGREE regardless of what OCCT does.

## Reusing the proven fixture generators

The defect builders already exist and are proven in `native_heal_parity.mm`:
`cubeQuads(jitter)`, `cubeTopSeamQuads(g)`, `cubeMissingTop`, `cubeMissingTwoOpposite`,
`cubeMissingTwoAdjacent`, `cubeTopShortEdgeSoup(seg)`, `cubeTopCollinearVertSoup(t)`,
plus `nativeSoup`/`occtSoup`/`occtSoupWithPlanarCap`/`occtSoupWithTwoPlanarCaps` and the
native `nativeQuadFace`/`nativePolyFace` builders. The fuzzer reuses these verbatim,
generalized from the unit cube to a randomly sized/positioned axis-aligned box and a
random convex N-gon prism (the box/prism generalization is a scale+translate of the
same quad topology, keeping the closed form exact). The RNG only chooses: base family +
dimensions, defect family, and the defect magnitude (jitter/gap/seg/t) drawn inside the
family's in-scope or out-of-scope band.

## Defect families and their in/out-of-scope split

| Defect | Native flag | In-scope magnitude → AGREED | Out-of-scope → DECLINE |
|---|---|---|---|
| SEW-JITTER | (default) | jitter ≤ tol | jitter ≫ tol (BEYOND-TOL-GAP) |
| FLIPPED-FACE | (default) | any single flipped face | — |
| SEAM-GAP | `gapBridgeBudget` | tol < g ≤ budget | g > budget (out-of-budget) |
| SHORT-EDGE | `shortEdgeMergeLen` | tol < seg ≤ mergeLen | flag OFF → more-conservative |
| COLLINEAR-VERT | `removeCollinearVerts` | t interior on the span | flag OFF → more-conservative |
| MISSING-1-HOLE | `capPlanarHoles` | one planar hole | — |
| MISSING-2-OPP | `capMultiplePlanarHoles` | two disjoint planar holes | — |
| MISSING-2-ADJ | `capMultiplePlanarHoles` | — | non-planar wrap → DECLINE (both) |
| BEYOND-TOL-GAP | (default) | — | residual ≫ tol → DECLINE (both) |

The two `flag OFF` rows are the *equal-or-more-conservative* probes: native honestly
declines while OCCT aggressively repairs; the trial is AGREED because OCCT's closure is
the honest ground-truth solid and native's deferral is safe (the opt-in flag separately
recovers the win, proven by the ON rows). This is the SAME contract the curated harness
asserts, made into a fuzzed batch.

## Classifier (exactly one bucket per trial)

```
native Healed watertight?
├─ yes → |Vnat − Vtruth| ≤ band  AND  |Anat − Atruth| ≤ band ?
│        ├─ yes → OCCT valid & matches truth?  → AGREED
│        │        OCCT valid but MISSES truth? → ORACLE-INACCURATE (native vindicated)
│        └─ no  → DISAGREED                     ← the bar failure
└─ no  (Unhealed) → OCCT valid & matches truth? → AGREED-by-more-conservative
                    OCCT also declined?          → AGREED-by-parity / BOTH-DECLINED
                    otherwise                     → HONESTLY-DECLINED
```

`Vtruth`/`Atruth` are the closed-form volume/area of the undamaged base solid. `band`
is the fixed mesh-deflection tolerance (`1e-3` for exact-collinear closures that land
the solid exactly; the sew/bridge tolerance `~g` for bridged closures where the merged
vertex sits anywhere within the gap — matching the curated harness's `< budget` bands).
No band is ever widened to force a pass.

## Determinism

`Rng` is the shared splitmix64 → xoshiro256** generator (copied from
`native_boolean_fuzz.mm`), seeded ONLY by an explicit `FUZZ_SEED` (argv[1] / env).
`main(argc, argv)` reads seed + N, runs the batch, prints the coverage table, and
`std::_Exit(bar_holds ? 0 : 1)` (OCCT static teardown is not exit-clean in the trimmed
static build — same rationale as every sibling sim harness). The runner loops ≥2 seeds
and fails if any seed fails.

## Build & link

Cloned from `run-sim-native-heal.sh` (the proven healing link line) plus the ≥2-seed
loop from `run-sim-native-reference-geometry-fuzz.sh`. Links the OCCT-free native TUs
(`src/native/heal/*.cpp` + `src/native/math/*.cpp`; topology/tessellate header-only) and
the SINGLE OCCT oracle TU `src/engine/occt/occt_shapefix.cpp`, with `TKShHealing
TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel`. No numsci, no `cc_*`
facade, no OcctEngine spine (healing is internal, exactly like the curated harness).

## Discipline / non-goals

`src/native/**`, `src/engine/**`, `include/**`, and the `cc_*` ABI are byte-unchanged
(test infra only). If a real native heal bug surfaces it is REPORTED (not fixed here). A
native result more correct than OCCT at a numeric edge is ORACLE-INACCURATE, not
DISAGREED. No tolerance widened, no clock, no `rand()`.
