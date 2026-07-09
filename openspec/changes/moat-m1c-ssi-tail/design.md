# Design — moat-m1c-ssi-tail

## Root cause (measured, host-probed)

The M1b decline was attributed to two effects. Both were confirmed empirically here.

### 1. Infinite oracle vs finite native patch
OCCT `Geom_ConicalSurface` / `Geom_CylindricalSurface` / `Geom_SphericalSurface` are analytic
INFINITE surfaces; the native adapters (`makeConeAdapter` etc.) are FINITE patches over a
`ParamBox`. `GeomAPI_IntSS` intersects the INFINITE surfaces, so when the unbounded quadric can
pierce the other operand more than once it returns the full multi-loop locus — which the finite
native trace, by construction, does not cover. This is an oracle-setup artefact, not a native gap.

**Fix:** wrap each oracle surface in `Geom_RectangularTrimmedSurface(s, u0,u1, v0,v1)` trimmed to
the native `ParamBox`. The native adapters and the OCCT quadrics share the parameterisation
(u = angle ∈ [0,2π]; v = height for cyl/cone, latitude for sphere), so the box maps 1:1 and the
trimmed-surface intersection is exactly the finite locus the native trace covers. Harness-only.

### 2. Topological-cluster over-merge → second-loop seeding-recall miss
For the twice-piercing off-axis sphere∩cyl pose there are genuinely TWO disjoint closed loops
(dense-grid ground truth: near-side z≈−0.85, far-side z≈+0.65). At coarse `initialGrid` the S2
seeder's pre-refine clustering (union-find over param-box adjacency on both surfaces) MERGES the
two loops into ONE cluster — the leaf boxes bridging the two penetration regions are param-adjacent.
`refineClusters` keeps one representative seed per cluster, so `refinedAccepted == 1` and only ONE
loop is traced (measured: `numClusters == 1` at grids 4/6/8; `== 2` only at grid ≥ 10).

Crucially, this is NOT a subdivision-recall hole (the far-loop candidate regions exist) nor a
refine failure (LM converges to both loops from coarse starts, gap ≈ 1e-15). A seeder-only 3D-
proximity clustering guard was tried and REJECTED: the two loops' large coarse leaf boxes still
3D-bridge along the cylinder wall, and a fixed 3D radius would over-split a single large loop (e.g.
the unit-radius sphere∩sphere circle). The correct disambiguator needs the TRACED curve — which is
exactly what the shipped S4-f completeness critic uses (locus point-to-polyline dedup).

**Fix:** a TARGETED variant of the S4-f critic. The critic already computes the uncovered param
cells (`critic::uncoveredBoxes`) but currently re-seeds the WHOLE domain at a finer `minPatchFrac`.
`criticTargetedReseed` re-seeds only the uncovered A-cells — each seeded as A clamped to the cell vs
B's full domain — so the second loop's region (which the traced first loop does not cover) gets a
dedicated finer re-seed and is recovered. Cheaper (never re-subdivides covered region) and equal or
higher recall. Host-measured: recovers the second loop at grids 4/6/8 (`criticRecoveredLoops == 1`,
`criticStoppedDry` after 1 round).

## Why additive + non-regressing
- Both new `SeedOptions` fields default off. `criticTargetedReseed` is consumed only inside the
  critic re-seed loop, which itself only runs when `completenessCritic` is on. With the defaults,
  the seeder and the critic are byte-identical to the shipped behaviour → every already-passing case
  (all 6 host SSI suites, all 14 prior sim cases) is unchanged, proven by re-running them green.
- No tolerance is widened. The targeted re-seed uses the SAME `onSurfTol` — a cell candidate that
  does not land on both surfaces is discarded by the seeder, never fabricated. The domain-clipped
  oracle is a stricter (finite) comparison, not a looser one.

## Alternatives rejected
- **Seeder-level 3D-proximity clustering guard** — rejected (over-splits large single loops; the
  coarse leaf boxes 3D-bridge the two loops anyway). Documented above.
- **Enabling the shipped whole-domain critic unchanged** — works, but re-scans the whole domain per
  round; the targeted variant is the honest, cheaper improvement the roadmap asked for and it lives
  in `src/native/ssi/`.
- **Adding a closed form for these quadric pairs** — impossible/incorrect (no elementary closed
  form); the S1 `NotAnalytic` decline is preserved.

## Two-gate results (numbers)
- **Gate A (host, OCCT-free), `test_native_ssi_marching` 19/19:** cone∩cone general 1 closed loop;
  cyl∩cone off-axis 1 open arc; sphere∩cyl twice-piercing baseline 1 loop → bump 2 closed loops;
  every traced node on both surfaces ≤ 1e-9.
- **Gate B (sim, native-vs-OCCT via clipped `GeomAPI_IntSS`), `run-sim-native-ssi-marching` 17/17:**
  cone cone general 1/1, closed 1/1, onCurve 3.9e-6, onSurf 3.4e-6, lenDelta 3.3e-6 (nat 3.3630 /
  occt 3.3630); cyl cone off-axis 1/1, closed 0/0, onCurve 4.7e-7, onSurf 4.3e-7, lenDelta 7.9e-5
  (nat 2.7501 / occt 2.7503); sphere cyl twice 2/2, closed 2/2, onCurve 7.6e-7, onSurf 4.4e-7,
  lenDelta 7.0e-6 (nat 4.2356 / occt 4.2356). All 14 prior cases frozen and passing.
