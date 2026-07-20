# native-ssi

## ADDED Requirements

### Requirement: The densify refit SHALL stay clear of interpolation and SHALL be judged between nodes

The native SSI marching tracer's convenience-curve densify-and-refit SHALL choose a pole count that
remains a fraction of the node count, and SHALL judge a candidate refit at parameters BETWEEN the
nodes as well as at the nodes, so that a fit which interpolates its data while oscillating between
data points is never accepted.

At a pole count equal to the node count the least-squares system is square and interpolating, and a
clamped-uniform knot vector over a chord-length parametrization degenerates: the curve passes
through every node exactly while deviating from the true curve between them by orders of magnitude
more than the fit it replaced. The at-node error metric is structurally blind to this, because it is
sampled at precisely the parameters such a fit interpolates.

- **Conditioning guard.** The refit pole target SHALL be bounded by a FRACTION of the node count,
  not by the node count itself, keeping the ratio clear of the measured degeneration onset. The
  bound SHALL be RELATIVE: a flat pole ceiling low enough to avoid degeneration on a moderate-node
  loop starves a dense high-curvature loop that legitimately requires the full pole cap.

- **Between-node accept test.** A candidate refit SHALL be accepted only if it does not worsen the
  deviation measured at node-midpoint parameters, in addition to the existing at-node test. The
  polyline remains the ground truth for that comparison.

- **Trigger proportionate to the verification budget.** The densify trigger SHALL be set tighter
  than the on-curve tolerance the native-vs-OCCT parity verification enforces, so a loop whose fit
  misses the true locus by more than that budget actually trips the refit. It SHALL remain loose
  enough that ordinary well-resolved loops do not refit. The multiplier SHALL be exposed as a
  caller option so the previous behaviour can be restored; changing it SHALL affect cost and curve
  quality only, and SHALL NOT move a node, widen a tolerance, or alter the polyline.

#### Scenario: A moderate-node high-curvature loop refits without interpolating

- **GIVEN** a near-tangent intersection loop of at most a few hundred nodes whose initial fit
  exceeds the densify trigger
- **WHEN** the curve is fitted
- **THEN** the refit SHALL fire, raising the pole count above the initial value
- **AND** the resulting pole count SHALL remain strictly below the node count
- **AND** the fitted curve sampled BETWEEN nodes SHALL lie on both surfaces within the on-curve
  budget the parity verification enforces

#### Scenario: A dense high-curvature loop still receives the full pole budget

- **GIVEN** an intersection loop with many hundreds of nodes that genuinely requires the pole cap
- **WHEN** the curve is fitted
- **THEN** the refit SHALL still reach the full pole cap, unchanged by the conditioning guard

#### Scenario: Ordinary loops are unaffected

- **GIVEN** a well-resolved intersection loop whose initial fit already rides its nodes
- **WHEN** the curve is fitted
- **THEN** no refit SHALL fire and the result SHALL be unchanged
