# native-ssi

## ADDED Requirements

### Requirement: Subdivision SHALL prune a box pair proven separated along an oriented direction

The native SSI seeder's recursive subdivision SHALL be able to discard a candidate box pair when the
two surface pieces are PROVEN to come no closer than the subdivision gap, using a witness direction
that need not be axis-aligned.

The existing disjointness test is axis-aligned. On a near-parallel pair separated along no
coordinate axis it can never fire, so the descent enumerates the entire 4D box product and hands
every leaf to the region refiner even though the pair does not intersect anywhere.

- **The witness SHALL be a containment argument.** By the convex-hull property each surface piece
  lies inside the hull of its exact sub-net over that param sub-box. Projection onto a direction is
  linear, so it maps each hull into a closed interval. Two intervals separated by more than the gap
  therefore PROVE no crossing exists in that box pair. Because a descendant's param boxes are
  contained in its parent's, a parent-level proof discards only crossing-free subtrees.

- **Soundness SHALL NOT depend on the direction chosen.** A direction that fails to separate simply
  leaves the descent unchanged, so the direction is a heuristic for REACH and never for
  correctness. The predicate SHALL normalize the direction itself rather than relying on its caller,
  since a non-unit direction would scale the projections and could overstate the separation. A
  degenerate direction SHALL cause the predicate to refuse.

- **Scope SHALL match what the proof requires and nothing looser.** The predicate SHALL apply only
  where both operands expose an exact single-span non-rational control net. It SHALL NOT require the
  two nets to share a degree, because each hull bounds its own surface independently and no
  correspondence between the nets is involved. Operands outside that scope SHALL take the unchanged
  path with byte-identical results.

- **The predicate SHALL NOT be applied where box locality does not hold.** Region refinement clamps
  into the full parameter domain and is effectively a global solve, so a converged solution is not
  confined to the candidate box that produced it. Applying this test there would discard real seeds.
  The justification recorded in the code SHALL be the containment argument, NOT an argument from
  precedent, because a precedent-shaped justification would equally license that unsound use.

#### Scenario: A near-parallel pair with no axis of separation is pruned

- **GIVEN** two freeform surfaces offset along their common normal by more than the subdivision gap,
  positioned so that their axis-aligned bounds overlap on every coordinate axis at every depth
- **WHEN** the seeder subdivides the pair
- **THEN** the descent SHALL be pruned rather than enumerating the full box product
- **AND** the seeder SHALL report no seeds, because the pair does not intersect

#### Scenario: A transversal crossing is never lost

- **GIVEN** two freeform surfaces that genuinely intersect
- **WHEN** the seeder subdivides the pair with the prune active
- **THEN** the seed count, the branch count and any coincidence verdict SHALL be unchanged from the
  same seeder with the prune inactive
- **AND** only the candidate-region count, which is pure cost, may differ

#### Scenario: An operand outside the proof's scope is unaffected

- **GIVEN** a pair in which either operand is elementary, rational, or a multi-span B-spline
- **WHEN** the seeder subdivides the pair
- **THEN** the prune SHALL NOT fire and the emitted candidate regions SHALL be byte-identical to the
  behaviour without it

#### Scenario: A caller-supplied direction cannot overstate the separation

- **GIVEN** a box pair whose surfaces are closer than the gap along every direction
- **WHEN** the predicate is asked with a direction of large magnitude
- **THEN** it SHALL NOT report a separation, having normalized the direction before comparing
