# native-blend

## ADDED Requirements

### Requirement: Native `cc_fillet_edges` rounds the crossing crease of two equal-radius orthogonal cylinders

The engine SHALL provide a NATIVE, OCCT-free path for `cc_fillet_edges(body, edgeIds, 1,
radius)` on the crossing crease of a Steinmetz bicylinder — two EQUAL-radius cylinders whose
axes are ORTHOGONAL and CROSSING. On a native bicylinder COMMON (intersection) whose picked
edge is such a crease, the engine SHALL build the blend as TWO coaxial TORUS strips (one per
crease plane, major radius `Rc − radius`, minor radius `radius`), each G1-tangent to both
cylinder walls at its two seam curves and each TAPERING to zero cross-section width at the two
shared poles, welded watertight to the rebuilt (faceted, trimmed) cylinder lune walls and disc
caps through the existing planar-facet assembly (the tessellator is NOT modified). A produced
candidate SHALL be accepted ONLY under the engine's SHRINK self-verify (watertight,
consistently oriented, enclosed volume strictly less than the sharp bicylinder). This path
SHALL remain OCCT-free and SHALL NOT change the `cc_*` ABI.

The native builder SHALL recognise the body WHOLESALE FROM ITS PLANAR-FACET SOUP (the native
SSI boolean does not preserve analytic cylinder faces): it SHALL recover the two orthogonal
cylinder axes as the directions perpendicular to the facet-normal families, a single common
radius `Rc`, and the axis-crossing point — declining unless every facet lies on one of the two
walls at that common `Rc` (or a disc cap ⟂ an axis) — so a rebuilt solid of a DIFFERENT shape
can never pass. The rolling-ball centre along each crease SHALL lie at CONSTANT distance
`Rc − radius` from both axes (the exact canal spine); the ring-torus guard (`Rc ≥ 2·radius`)
SHALL hold or the builder SHALL decline. The two canal strips SHALL share the two canonical
pole vertices (a degenerate pinch), so no finite corner patch is synthesized. The op SHALL
round the WHOLE crossing crease (all four arcs) — the only watertight resolution for a crease
whose arcs meet at the poles. A MANDATORY internal self-verify (consistently oriented AND a
removed-volume bound) SHALL reject any large-radius pole-region fold → NULL → OCCT.

#### Scenario: Steinmetz crease fillets natively and converges to the closed form (host)

- GIVEN a native Steinmetz bicylinder COMMON (two equal-radius cylinders, axes orthogonal and crossing, long enough that the disc caps do not touch the fillet band) whose crossing crease is the picked edge, with the native engine active and no OCCT
- WHEN `cc_fillet_edges(B, {crease}, 1, r)` is invoked with `Rc ≥ 2r`
- THEN the native op SHALL return a watertight, consistently-oriented solid (χ = 2) whose enclosed volume is strictly less than the sharp bicylinder (`16/3·Rc³` for the full lens) AND matches the closed-form removed volume (the canal-tube integral over the two crease spines) to the deflection bound, converging monotonically as the deflection is refined

#### Scenario: Out-of-envelope canal creases are honestly declined (host)

- GIVEN a native body that is a single cylinder / cone / sphere cap rim (served by their own arms), two cylinders of UNEQUAL radius, non-orthogonal or non-crossing axes, `Rc < 2r`, a planar box, a zero/negative radius, or a multi-edge pick, with the native engine active and no OCCT
- WHEN `cc_fillet_edges` (canal arm) is invoked
- THEN the canal builder SHALL return a NULL result (everything else falls through to OCCT `BRepFilletAPI_MakeFillet`) AND SHALL NEVER emit an unverified or wrong-shaped solid, and a native void SHALL NEVER be handed to OCCT

#### Scenario: OCCT oracle confirms the case on the simulator; native fillet is host-gated (sim)

- GIVEN the cc_* facade on a booted iOS simulator and a Steinmetz bicylinder COMMON built via `cc_boolean(cylZ, cylX, common)`
- WHEN the case runs under the OCCT engine (oracle, `BRepFilletAPI_MakeFillet` + `BRepGProp`) and the native engine
- THEN the OCCT oracle SHALL produce a filleted bicylinder whose volume is strictly below the sharp bicylinder (confirming the case is real), AND — because the native COMMON of two full cylinders is not currently constructible through the `cc_boolean` facade (a boolean-track breadth gap, NOT a fillet gap) — the harness SHALL record that honestly without fabricating a native body; the native canal FILLET itself SHALL be gated by the host suite (a watertight, consistently-oriented, material-removing fillet on a genuine native Steinmetz body)
