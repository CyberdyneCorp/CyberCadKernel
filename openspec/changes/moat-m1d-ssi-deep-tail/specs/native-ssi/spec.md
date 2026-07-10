# native-ssi

## ADDED Requirements

### Requirement: S4-c near-tangent crossing breadth is extended into the grazing regime via adaptive re-anchoring, with a measured honest floor

The native SSI marching tracer SHALL cross near-tangent GRAZING intersections deeper into the
transversality-sine regime than the shipped fixed-plane crossing corrector, WITHOUT weakening any
tolerance and WITHOUT fabricating any curve, gated behind a default-off `MarchOptions` flag.

The shipped S4-c crossing corrector freezes one reference tangent as both the crossability anchor
and the fixed-plane advance direction for the whole crossing; its robust-crossing floor is a
minimum transversality sine (the norm of the cross product of the two unit surface normals along
the intersection) of approximately 0.17 — below it the two-surface corrector fails to converge
because the turning curve leaves the frozen advance plane.

- **Adaptive crossing re-anchoring (S3/S4-c, additive, default-off).** A `MarchOptions` flag
  (`adaptiveCrossReanchor`, with a `reanchorBlend` weight) SHALL, when set, let the crossing
  corrector re-anchor its fixed-plane advance direction toward the LOCAL intersection tangent
  (the continuity-oriented normalized cross product of the two surface normals) as it steps, so the
  advance plane follows the curve's turn through a tighter graze. On this path per-step progress
  SHALL be measured along the actual step direction (bounded by an anti-orbit total-arc cap so a
  non-traversing orbit still terminates and defers), and hand-back to the normal transversal march
  SHALL occur once transversality recovers above the band-enter threshold with a two-consecutive-
  node stability requirement. This flag SHALL DEFAULT OFF, so the crossing corrector, its per-node
  crossability guard, its step control, and its hand-back threshold are byte-identical to the
  shipped S4-c crossing for every already-passing case.

- **Honesty anchors preserved.** With the flag on, the crossability decision (the band-minimum
  sine floor, the steep-sine-collapse witness, and the per-step ≥60° branch-flip guard) SHALL
  remain, every crossing node SHALL be verified on BOTH surfaces within the SAME on-surface
  tolerance (never fabricated), and a genuine tangency or branch point (transversality sine → 0)
  SHALL still defer. No closed form SHALL be added; no tolerance SHALL be widened.

- **Measured breadth extension and honest floor.** With re-anchoring on, the robust-crossing floor
  SHALL drop from a minimum transversality sine of approximately 0.17 to approximately 0.14: a
  grazing pose whose sine dips to ≈ 0.14 (which the shipped corrector declines) SHALL be traced to
  a full closed loop with every node on both surfaces within tolerance. A grazing pose whose sine
  dips below approximately 0.12 SHALL remain an HONEST DECLINE even with re-anchoring on (the
  near-tangent band is then too wide to recover to a transversal stretch within budget) — reported
  as a near-tangent gap, never forced to pass.

The behaviour SHALL be verified by BOTH gates of the two-gate model:
- **Gate A (host, OCCT-free)** — self-consistency + closure: for the newly-crossable grazing pose,
  the default path (flag off) defers with a near-tangent gap and no curve, while the re-anchoring
  path (flag on) traces ONE closed loop with `nearTangentGaps == 0`, `nearTangentCrossed ≥ 1`,
  every node on both surfaces within tolerance, and an arc length within a step-bounded window of
  the tolerance-below-dip ground truth; and for the pose below the extended floor, the re-anchoring
  path still declines (no crossing, no closed loop, a reported near-tangent gap) while a
  ground-truth loop still exists.
- **Gate B (sim, native-vs-OCCT via `GeomAPI_IntSS`)** — the re-anchoring crossing SHALL trace a
  single closed loop whose densely-sampled nodes all lie on the OCCT intersection locus AND on both
  surfaces within tolerance (the crossed curve IS the true intersection, not a fabricated path),
  the default path SHALL still decline the same pose, and below the extended floor the native
  marcher SHALL decline while OCCT reports a locus (the honest, measured boundary).

#### Scenario: a tighter graze the shipped corrector declines is crossed with adaptive re-anchoring
- GIVEN an offset cylinder grazing a sphere posed so the intersection is a single closed loop whose
  minimum transversality sine dips to ≈ 0.14 (below the ≈ 0.17 shipped fixed-plane floor)
- WHEN the crossing is attempted with `adaptiveCrossReanchor` OFF (the default)
- THEN the tracer SHALL HONESTLY DEFER — `nearTangentGaps ≥ 1`, `nearTangentCrossed == 0`, no closed
  loop
- AND WHEN the crossing is attempted with `adaptiveCrossReanchor` ON
- THEN the tracer SHALL trace ONE `Closed` WLine with `nearTangentGaps == 0`, `nearTangentCrossed ≥ 1`,
  every node on both surfaces within tolerance, and an arc length within a step-bounded window of the
  tolerance-below-dip ground truth
- AND against `GeomAPI_IntSS` every densely-sampled node SHALL lie on the OCCT locus and on both
  surfaces within tolerance

#### Scenario: below the extended floor the marcher honestly declines while OCCT reports
- GIVEN an offset cylinder grazing a sphere posed so the minimum transversality sine dips below
  ≈ 0.12 (below the extended re-anchoring floor), with a ground-truth loop that still exists when
  traced with the tolerance below the dip
- WHEN the crossing is attempted with `adaptiveCrossReanchor` ON
- THEN the tracer SHALL still HONESTLY DECLINE — `nearTangentCrossed == 0`, no closed loop, a
  reported near-tangent gap — never fabricating a curve across the knife-edge
- AND `GeomAPI_IntSS` SHALL report a locus for the same pose (the honest, measured boundary between
  native and OCCT)

#### Scenario: the re-anchor flag defaults off leave every prior case unchanged
- GIVEN any already-passing SSI case (transversal march, shipped S4-c graze, S4-d branch, S4-e
  chart singularity, or a genuine tangency that must defer)
- WHEN it is traced with `adaptiveCrossReanchor` at its default (off)
- THEN the crossing corrector, its per-node crossability guard, its step control, and its hand-back
  threshold SHALL be byte-identical to the shipped S4-c behaviour, so the result is unchanged
