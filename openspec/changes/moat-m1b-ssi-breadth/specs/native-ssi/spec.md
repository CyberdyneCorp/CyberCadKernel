# native-ssi

## ADDED Requirements

### Requirement: General non-coaxial / skew analytic quadric intersections are traced by S3 and verified against OCCT

The native SSI module SHALL trace, and verify against OCCT, general non-coaxial / skew
analytic quadric intersections (at least: general skew cylinder∩cylinder that resolves to a
single connected quartic loop, and off-axis sphere∩cone) via its S2 seeder + S3 marching
tracer.

The S1 analytic dispatch correctly returns NOT-ANALYTIC for the non-closed-form analytic
quadric pairs (skew cylinder∩cylinder, general cone∩cone, non-coaxial cone∩cylinder,
off-axis sphere∩cylinder / sphere∩cone) because no elementary closed-form intersection
curve exists for them. For every such **transversal** pair the native S2 seeder + S3
predictor-corrector marching tracer (`cybercad::native::ssi`, OCCT-free, under
`CYBERCAD_HAS_NUMSCI`) SHALL trace the full intersection curve as one WLine per distinct
transversal connected component, whose sampled points provably lie on BOTH native surfaces
within a scale-derived tolerance, with `nearTangentGaps == 0`. No tracer or seeder code
change and no `cc_*` signature or POD struct change is required to serve this — the
capability is the already-shipped S3 output; this requirement locks it as a verified,
regression-covered breadth family and does NOT introduce any new closed form.

The DECISIVE native-vs-OCCT parity (Gate B) SHALL cover the poses whose intersection is a
SINGLE loop bounded by a FINITE operand (the smaller cylinder's single penetration region;
the finite sphere). Poses where an UNBOUNDED OCCT quadric pierces the other operand more than
once along its infinite extent (general cone∩cone and off-axis cyl∩cone with both operands
unbounded; off-axis sphere∩cylinder where the infinite cylinder pierces twice) SHALL be an
HONEST DECLINE with a measured reason (the finite native trace cannot match the infinite-locus
oracle without domain-clipped oracle surfaces; and/or a seeding-recall miss of the second
loop) — NEVER faked, and NEVER forced by weakening a tolerance.

The behaviour SHALL be verified by BOTH gates of the two-gate model:
- **Gate A (host, OCCT-free)** — self-consistency + closure: every traced node lies on both
  native surfaces within tolerance, the branch is classified `Closed` or `BoundaryExit` as
  the pose dictates, `nearTangentGaps == 0`, and the traced connected-component count equals
  the analytically-known count for the pose.
- **Gate B (sim, native-vs-OCCT parity)** — the same pair built as native adapters AND as
  OCCT `Geom_*` surfaces, compared via `GeomAPI_IntSS`: the native traced-branch count and
  closed-loop count SHALL equal the OCCT transversal-component count (arc-split loci welded
  into connected components), densely-sampled native points SHALL lie on the OCCT
  intersection locus and on both OCCT surfaces within tolerance, and the native arc-length
  SHALL match the OCCT arc-length within a deflection/step relative tolerance.

The S1 NOT-ANALYTIC decline for these pairs SHALL be preserved unchanged; no tolerance SHALL
be weakened to force a Gate-B pass, and any pose that grazes near-tangent on the oracle (OCCT
still yields a curve but native must truncate) SHALL be reported as the honest decline, never
padded with fabricated points.

#### Scenario: general skew cylinder∩cylinder traces one connected quartic loop, verified vs OCCT
- GIVEN two cylinders whose axes are neither parallel nor intersecting (a nonzero minimum
  axis-to-axis distance) AND oblique (a non-right relative angle), posed so the smaller
  cylinder fully penetrates the larger in a single crossing region
- WHEN the S3 tracer marches from the S2 seed(s)
- THEN it SHALL trace ONE connected `Closed` WLine (the single quartic loop) with
  `nearTangentGaps == 0` and every node on both cylinders within tolerance
- AND against `GeomAPI_IntSS` for the same pair the native branch count, closed-loop count,
  on-locus / on-both-surfaces deltas and arc-length SHALL all match within tolerance

#### Scenario: off-axis sphere∩cone traces a single loop and matches OCCT
- GIVEN a sphere and a cone whose axis does not pass through the sphere centre (off-axis),
  posed transversally so the finite sphere admits the near cone nappe in a single loop
- WHEN the S3 tracer marches from the S2 seed
- THEN it SHALL trace the intersection loop as one `Closed` WLine with `nearTangentGaps == 0`
  and every node on both surfaces within tolerance
- AND the native result SHALL match `GeomAPI_IntSS` for the same pair (count/type/on-locus/
  on-surface/arc-length) within tolerance

#### Scenario: unbounded-quadric multi-loop poses are honestly declined
- GIVEN a pose in which an unbounded OCCT quadric pierces the other operand more than once
  along its infinite extent (general cone∩cone, off-axis cyl∩cone, or off-axis sphere∩cylinder
  where the infinite cylinder pierces both sides)
- WHEN the pair is compared native-vs-OCCT
- THEN the mismatch SHALL be recorded as an honest decline with a measured reason (finite
  native trace vs infinite-locus oracle, and/or seeding-recall miss of the second loop), and
  the native side SHALL NOT fabricate the missing loop nor weaken a tolerance to force a pass

#### Scenario: the S1 closed-form decline for these pairs is preserved
- GIVEN a skew cylinder∩cylinder / general cone∩cone / non-coaxial quadric pair handed to
  the S1 analytic dispatch `intersect_surfaces`
- WHEN S1 classifies the pair
- THEN it SHALL still return `NotAnalytic` with an empty curve list (no elementary closed
  form exists), deferring to the S2/S3 marching path — and SHALL NOT fabricate a closed-form
  curve to satisfy the new breadth coverage
