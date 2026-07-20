# native-blend

## ADDED Requirements

### Requirement: Native `cc_fillet_edges` rounds the crossing creases of two UNEQUAL-radius NON-ORTHOGONAL cylinders

The engine SHALL provide a NATIVE, OCCT-free path for `cc_fillet_edges(body, edgeIds, 1,
radius)` on the crossing creases of an unequal-radius OBLIQUE bicylinder COMMON — two cylinders
whose axes CROSS at an angle α that is clearly NON-orthogonal and NON-parallel, with DISTINCT
radii `Ra ≠ Rb` (the thin cylinder passing through the thick one at a slant). On such a native
body the engine SHALL build the blend as TWO closed CANAL STRIPS — one per DISJOINT crease loop
(distinct radii keep the top and bottom intersection loops disjoint and non-degenerate at ANY
crossing angle, so there is no pole and no corner patch) — each G1-tangent to both cylinder walls
at its two seam curves, welded watertight to the rebuilt (faceted, trimmed) thin-wall waist tube
and the two thick-wall cap patches through the existing planar-facet assembly (the tessellator is
NOT modified). A produced candidate SHALL be accepted ONLY under the engine's SHRINK self-verify
(watertight, consistently oriented, enclosed volume strictly less than the sharp bicylinder). This
path SHALL remain OCCT-free and SHALL NOT change the `cc_*` ABI. ORTHOGONAL axes SHALL route to
the orthogonal unequal canal path instead; EQUAL radii (which pinch at poles at any angle) and
near-parallel axes SHALL decline.

The native builder SHALL recognise the body WHOLESALE FROM ITS PLANAR-FACET SOUP: it SHALL
recover the two cylinder axes as the directions perpendicular to the facet-normal families and TWO
radii classified PER FACET BY RADIUS, declining unless every facet lies on one of the two
cylinders, the radii are DISTINCT, and the axes cross at |cosα| ∈ (0.05, 0.97). In a canonical
frame (thin axis `ez`, thick axis `b̂ = sinα·ex + cosα·ez`) the rolling-ball centre along each
crease loop SHALL satisfy `cz±(u) = [R0·cosu·cosα ± √(R0b²−R0²sin²u)]/sinα` with `R0 = Ra−radius`
and `R0b = Rb−radius`, lying at distance `Ra−radius` from the thin axis and `Rb−radius` from the
thick axis (the exact canal spine, never reaching a pole because `Rb > Ra`); the ring-torus guard
(`Ra ≥ 2·radius`) and strict separation (`R0b > R0`) SHALL hold or the builder SHALL decline. A
MANDATORY internal self-verify (consistently oriented AND a removed-volume bound scaled by 1/sinα)
SHALL reject any large-radius fold → NULL → OCCT.

#### Scenario: Oblique-bicylinder creases fillet natively and converge (host)

- GIVEN a native unequal OBLIQUE bicylinder COMMON (`Ra ≠ Rb`, axes crossing at a clearly non-orthogonal, non-parallel angle, long enough that the disc caps do not touch the fillet band) whose crossing crease is the picked edge, with the native engine active and no OCCT
- WHEN `cc_fillet_edges(B, {crease}, 1, r)` is invoked with `Ra ≥ 2r`
- THEN the native op SHALL return a watertight, consistently-oriented solid (χ = 2) whose enclosed volume is strictly less than the sharp bicylinder AND keeps the large majority of the body, converging monotonically as the deflection is refined

#### Scenario: The oblique canal strip is G1-tangent to both cylinder walls (host, analytic)

- GIVEN the oblique canal builder's closed-form strip at any crossing angle α
- WHEN the strip surface normal `(P−C)/radius` is compared to the incident wall radials at both seam curves
- THEN it SHALL equal the thin-wall radial at the `t=0` seam and the thick-wall radial at the `t=1` seam, and each seam point SHALL lie exactly on its wall (`|xy| = Ra` on the thin wall, perpendicular distance `Rb` from the oblique thick axis), with no reliance on OCCT or a mesh

#### Scenario: Non-oblique, equal-radius, and degenerate inputs decline to OCCT

- GIVEN a body that is NOT an unequal oblique bicylinder — an ORTHOGONAL bicylinder (routes to the orthogonal unequal path), a near-parallel or non-crossing pair, an EQUAL-radius pair (pole pinch), a box, a thin cylinder with `Ra < 2·radius`, `radius ≤ 0`, or a multi-edge pick
- WHEN `cc_fillet_edges` is invoked with the oblique candidate
- THEN the oblique native builder SHALL return NULL (decline) and the engine SHALL fall through to OCCT, NEVER a wrong or leaky solid
