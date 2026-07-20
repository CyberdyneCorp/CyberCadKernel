# native-ssi

## ADDED Requirements

### Requirement: The S4-c re-anchored crossing orients by continuity, so a graze whose tangent turns past 90° is traversed rather than trapped

The native SSI marching tracer SHALL resolve the ORIENTATION of the re-anchored advance direction
against the previously accepted step direction rather than against the frozen band-entry tangent,
gated behind a default-off `MarchOptions` flag, so that a crossable graze whose intersection
tangent accumulates a turn of 90° or more across the near-tangent band is traversed instead of
being trapped in a non-advancing cycle.

The shipped re-anchor path resolves BOTH the local tangent's sign AND its adoption gate against
the frozen band-entry tangent. Both are half-spaces of a single stale direction and both degenerate
at the same point: once the intersection curve's tangent has turned 90° from that direction, the
sign test inverts the true forward tangent into the backward one. The march then retreats, re-enters
the sub-90° half-space, and advances again — a self-sustaining cycle that consumes the whole node
budget without net transport. The resulting decline is honestly reported, but it is a TRAP rather
than a geometric limit: the poses are crossable.

- **Incremental orientation (S4-c, additive, default-off).** A `MarchOptions` flag
  (`reanchorIncrementalOrientation`) SHALL, when set, re-reference BOTH the sign test and the
  adoption gate of the re-anchored tangent to the previously accepted step direction, seeded to the
  frozen band-entry tangent for the first step. Orientation by continuity is monotone across an
  arbitrarily large accumulated turn. The flag SHALL be consulted ONLY when the adaptive re-anchor
  path is itself enabled, and SHALL DEFAULT OFF, so every already-passing case is byte-identical.

- **Both tests SHALL move together.** Re-referencing only the sign test SHALL NOT be considered
  conformant: the adoption gate then rejects the local tangent, the advance direction falls back to
  the frozen tangent, the step lands perpendicular to the curve, and the step size collapses to the
  floor — relocating the decline rather than removing it.

- **Net-transport termination guard.** On this path the tracer SHALL bound the ratio of arc spent to
  net displacement from the band entry, and discard-and-defer when it is exceeded. The existing
  per-step advance test cannot detect a non-advancing cycle, because the corrector pins the measured
  advance to the requested step along the very direction the step was taken in; and the existing
  anti-orbit arc cap is derived from the step budget, so it scales with the budget and cannot bind.
  This guard is TERMINATION SAFETY only: it converts a residual cycle into an immediate honest
  defer, and SHALL NOT be able to cause a curve to be emitted.

- **Honesty anchors preserved.** The frozen band-entry tangent SHALL remain the anchor for the
  crossability decision — the band-minimum sine floor, the steep-sine-collapse witness, and the
  per-step branch-flip guard are UNCHANGED. Every node SHALL still be verified on BOTH surfaces at
  the SAME on-surface tolerance. No tolerance SHALL be widened and no point SHALL be fabricated.

- **The floor SHALL rest on a declared tolerance.** With the trap removed, the limiting condition
  SHALL be the configured minimum crossing sine, reached at the band-minimum gate with ZERO crossing
  nodes emitted — a principled refusal, distinguishable from a budget-exhaustion decline that emits
  and discards a full budget of non-advancing nodes.

#### Scenario: A wide-band graze whose tangent turns past 90° is crossed

- **GIVEN** a unit sphere and a cylinder of radius 0.4 offset along +x by 0.593, 0.595, or 0.597,
  whose intersection loop has a minimum transversality sine of approximately 0.118, 0.100, or 0.077
- **AND** the adaptive re-anchor path enabled
- **WHEN** the intersection is traced with incremental orientation OFF
- **THEN** the tracer declines, reporting a near-tangent gap and no closed curve
- **WHEN** the same trace is run with incremental orientation ON
- **THEN** the tracer SHALL produce exactly ONE closed loop with no near-tangent gap
- **AND** every corrected node SHALL lie on BOTH surfaces within the on-surface tolerance
- **AND** the traced length SHALL be within a step-bounded window of a ground-truth trace taken
  with a tolerance below the dip

#### Scenario: Below the minimum crossing sine the tracer still declines

- **GIVEN** the same family offset by 0.5975 or 0.598, whose band-minimum sine falls below the
  configured minimum crossing sine
- **WHEN** the intersection is traced with incremental orientation ON
- **THEN** the tracer SHALL decline, reporting a near-tangent gap and no closed curve
- **AND** SHALL emit no crossing nodes at all

#### Scenario: A genuine tangency still defers

- **GIVEN** the same family offset so the surfaces are tangent
- **WHEN** the intersection is traced with incremental orientation ON
- **THEN** the tracer SHALL NOT report a crossing and SHALL NOT close a loop

#### Scenario: The flag defaults off and is inert without the re-anchor path

- **GIVEN** default march options
- **THEN** incremental orientation SHALL be off
- **AND** a pose that the shipped re-anchor path declines SHALL still decline
- **WHEN** incremental orientation is set WITHOUT the adaptive re-anchor path
- **THEN** the trace SHALL be identical to the plain frozen-tangent path
