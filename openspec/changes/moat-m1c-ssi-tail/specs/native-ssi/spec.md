# native-ssi

## ADDED Requirements

### Requirement: Declined off-axis quadric-intersection tail is promoted to verified via a domain-clipped oracle and a targeted seeding-recall re-seed

The native SSI module SHALL trace, and verify against OCCT, the off-axis quadric-intersection
families that M1b honestly declined — general cone∩cone, off-axis cylinder∩cone, and off-axis
sphere∩cylinder (including the twice-piercing pose that yields two disjoint loops) — via its S2
seeder + S3 marching tracer, WITHOUT weakening any tolerance and WITHOUT fabricating any curve.

Two mechanisms SHALL make this a decisive, apples-to-apples verification:

- **Domain-clipped oracle (verification harness).** Because an OCCT
  `Geom_Conical`/`Geom_Cylindrical`/`Geom_SphericalSurface` is INFINITE while the native adapter is
  a FINITE patch over a parameter box, the sim parity gate SHALL wrap each oracle surface in a
  `Geom_RectangularTrimmedSurface` trimmed to the SAME parameter box the native adapter uses, so
  `GeomAPI_IntSS` produces the SAME finite locus the native trace covers. This is a test-harness
  construction only; it SHALL NOT change `src/native` and SHALL NOT widen any tolerance.

- **Targeted seeding-recall re-seed (S2/S4-f, additive, default-off).** A `SeedOptions` flag
  (`criticTargetedReseed`, bounded by `criticMaxCells`) SHALL, when set together with the S4-f
  completeness critic, re-seed ONLY the parameter cells no traced curve covers — each uncovered
  cell seeded as a restricted sub-domain against the other operand's full domain — so the SECOND
  loop of a twice-piercing off-axis pose (which the coarse fixed grid merges into one topological
  cluster and therefore seeds once) is recovered at practical grid densities. This flag and its cost
  bound SHALL DEFAULT OFF, so the fixed-resolution seeder and the existing whole-domain critic re-
  seed are byte-identical for every already-passing case. The targeted re-seed SHALL NEVER fabricate
  a seed (each cell candidate must still lie on BOTH surfaces within the SAME on-surface tolerance)
  and SHALL NEVER widen a tolerance.

The S1 analytic dispatch SHALL still return NOT-ANALYTIC for these non-closed-form pairs (no
elementary closed form exists); this requirement verifies the S2/S3 marching path and SHALL NOT add
any closed form. Any pose that still cannot be robustly traced SHALL remain an honest decline with a
measured reason, never forced to pass.

The behaviour SHALL be verified by BOTH gates of the two-gate model:
- **Gate A (host, OCCT-free)** — self-consistency + closure: every traced node lies on both native
  surfaces within tolerance, each branch is classified `Closed` or `BoundaryExit` as the pose
  dictates, `nearTangentGaps == 0`, and the traced connected-component count equals the
  analytically-known count (including two closed loops for the twice-piercing pose after the recall
  bump, and one loop before it — the recall bump's effect is asserted explicitly).
- **Gate B (sim, native-vs-OCCT parity)** — the same pair built as native adapters AND as
  domain-clipped OCCT `Geom_*` surfaces, compared via `GeomAPI_IntSS`: the native traced-branch and
  closed-loop counts SHALL equal the OCCT transversal-component counts, densely-sampled native
  points SHALL lie on the OCCT locus and on both OCCT surfaces within tolerance, and the native
  arc-length SHALL match the OCCT arc-length within a deflection/step relative tolerance.

#### Scenario: general cone∩cone traces one closed loop, verified vs a domain-clipped oracle
- GIVEN two finite cones with offset apexes and tilted axes, posed so their intersection is a single
  closed loop inside both finite patches
- WHEN the S3 tracer marches from the S2 seed
- THEN it SHALL trace ONE `Closed` WLine with `nearTangentGaps == 0` and every node on both cones
  within tolerance
- AND against `GeomAPI_IntSS` on the domain-clipped oracle surfaces the native branch count,
  closed-loop count, on-locus / on-both-surfaces deltas and arc-length SHALL all match within
  tolerance

#### Scenario: off-axis cylinder∩cone traces one open arc, verified vs a domain-clipped oracle
- GIVEN a cylinder and a cone with offset, tilted axes posed so the intersection arc runs off the
  finite patch boundaries
- WHEN the S3 tracer marches from the S2 seed
- THEN it SHALL trace ONE non-closed (`BoundaryExit`) WLine with `nearTangentGaps == 0` and every
  node on both surfaces within tolerance
- AND against `GeomAPI_IntSS` on the domain-clipped oracle surfaces the native branch count matches,
  the closed-loop count is zero on both sides, and the on-locus / on-surface / arc-length deltas
  match within tolerance

#### Scenario: twice-piercing off-axis sphere∩cylinder recovers the second loop with the targeted re-seed
- GIVEN a thin cylinder offset from the sphere centre so it pierces the finite sphere on both sides
  (two disjoint closed loops)
- WHEN the fixed-resolution seeder + tracer run at a practical grid density
- THEN they SHALL trace only ONE loop (the coarse grid merges the two loops into one topological
  cluster → one seed) — the honest recall miss
- AND WHEN the targeted seeding-recall re-seed (`completenessCritic` + `criticTargetedReseed`) is
  enabled
- THEN the tracer SHALL recover the SECOND loop (`criticRecoveredLoops ≥ 1`) → TWO `Closed` WLines,
  `nearTangentGaps == 0`, every node on both surfaces within tolerance
- AND against `GeomAPI_IntSS` on the domain-clipped oracle surfaces the native branch count (2) and
  closed-loop count (2) SHALL match, with on-locus / on-surface / arc-length deltas within tolerance

#### Scenario: the recall-bump flags default off leave every prior case unchanged
- GIVEN any already-verified SSI pair traced without the M1c flags
- WHEN `criticTargetedReseed` and `criticMaxCells` are left at their defaults
- THEN the seeder and the S4-f completeness critic SHALL behave byte-identically to the shipped
  behaviour (no new seed, no changed branch count, no changed tolerance), so all prior host and sim
  cases stay green

#### Scenario: the S1 closed-form decline for these pairs is preserved
- GIVEN a general cone∩cone / off-axis cylinder∩cone / off-axis sphere∩cylinder pair handed to the
  S1 analytic dispatch `intersect_surfaces`
- WHEN S1 classifies the pair
- THEN it SHALL still return `NotAnalytic` with an empty curve list (no elementary closed form
  exists), deferring to the S2/S3 marching path, and SHALL NOT fabricate a closed-form curve to
  satisfy the new breadth coverage
