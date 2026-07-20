# native-blend

## ADDED Requirements

### Requirement: Native `cc_fillet_edges` rounds the crossing creases of two UNEQUAL-radius orthogonal cylinders

The engine SHALL provide a NATIVE, OCCT-free path for `cc_fillet_edges(body, edgeIds, 1,
radius)` on the crossing creases of an unequal-radius orthogonal bicylinder COMMON — two
cylinders whose axes are ORTHOGONAL and CROSSING with DISTINCT radii `Ra ≠ Rb` (the thin
cylinder poking fully through the thick one). On such a native body the engine SHALL build the
blend as TWO closed CANAL STRIPS — one per DISJOINT crease loop (the distinct radii make the
top `cz>0` and bottom `cz<0` intersection loops disjoint and non-degenerate, so there is no
pole and no corner patch) — each G1-tangent to both cylinder walls at its two seam curves,
welded watertight to the rebuilt (faceted, trimmed) thin-wall waist tube and the two thick-wall
cap patches through the existing planar-facet assembly (the tessellator is NOT modified). A
produced candidate SHALL be accepted ONLY under the engine's SHRINK self-verify (watertight,
consistently oriented, enclosed volume strictly less than the sharp bicylinder). This path SHALL
remain OCCT-free and SHALL NOT change the `cc_*` ABI. EQUAL radii SHALL route to the Steinmetz
canal path instead.

The native builder SHALL recognise the body WHOLESALE FROM ITS PLANAR-FACET SOUP: it SHALL
recover the two orthogonal cylinder axes as the directions perpendicular to the facet-normal
families and TWO radii classified PER FACET BY RADIUS (a facet on the `Ra` cylinder has
perpendicular distance `Ra` exactly, regardless of its planar-facet normal tilt), declining
unless every facet lies on one of the two cylinders and the radii are DISTINCT. The rolling-ball
centre along each crease loop SHALL lie at distance `Ra − radius` from the thin axis and
`Rb − radius` from the thick axis (the exact canal spine, never reaching a pole because
`Rb > Ra`); the ring-torus guard (`Ra ≥ 2·radius`) and strict separation (`Rb − radius >
Ra − radius`) SHALL hold or the builder SHALL decline. A MANDATORY internal self-verify
(consistently oriented AND a removed-volume bound) SHALL reject any large-radius fold → NULL →
OCCT.

#### Scenario: Unequal-bicylinder creases fillet natively and converge (host)

- GIVEN a native unequal orthogonal bicylinder COMMON (`Ra ≠ Rb`, axes orthogonal and crossing, long enough that the disc caps do not touch the fillet band) whose crossing crease is the picked edge, with the native engine active and no OCCT
- WHEN `cc_fillet_edges(B, {crease}, 1, r)` is invoked with `Ra ≥ 2r`
- THEN the native op SHALL return a watertight, consistently-oriented solid (χ = 2) whose enclosed volume is strictly less than the sharp bicylinder AND keeps the large majority of the body, converging monotonically as the deflection is refined

#### Scenario: The canal strip is G1-tangent to both cylinder walls (host, analytic)

- GIVEN the unequal-canal strip geometry at a canonical frame (`Ra=1, Rb=1.5, r=0.2`)
- WHEN the strip surface normal `(P − C)/r` is evaluated at the thin-wall seam (`t=0`) and the thick-wall seam (`t=1`) for every spine sample on both crease loops
- THEN the normal SHALL equal the thin cylinder's outward radial at the `t=0` seam and the thick cylinder's outward radial at the `t=1` seam (to 1e-9), AND each seam point SHALL lie exactly on its wall (`|xy|=Ra`, `|yz|=Rb`) — the tangency the fillet claims, with no OCCT and no mesh

#### Scenario: Out-of-envelope unequal creases are honestly declined (host)

- GIVEN a native body that is a planar box, an EQUAL-radius bicylinder (routes to the Steinmetz canal), a body with `Ra < 2r`, a zero/negative radius, or a multi-edge pick, with the native engine active and no OCCT
- WHEN `cc_fillet_edges` (unequal-canal arm) is invoked
- THEN the unequal-canal builder SHALL return a NULL result (everything else falls through to OCCT `BRepFilletAPI_MakeFillet`) AND SHALL NEVER emit an unverified or wrong-shaped solid

#### Scenario: OCCT oracle confirms the case on the simulator; native fillet is host-gated (sim)

- GIVEN the cc_* facade on a booted iOS simulator and an unequal-bicylinder COMMON built via OCCT (`BRepPrimAPI_MakeCylinder` ×2 + `Common`)
- WHEN the case runs under the OCCT engine (oracle, `BRepFilletAPI_MakeFillet` + `BRepGProp`) and the native engine
- THEN the OCCT oracle SHALL produce a filleted unequal bicylinder whose volume is strictly below the sharp body (confirming the case is real), AND — because the native unequal COMMON is not practically constructible through the `cc_boolean` facade (a boolean-track breadth gap, NOT a fillet gap) — the harness SHALL record that honestly without fabricating a native body; the native canal FILLET itself SHALL be gated by the host suite
