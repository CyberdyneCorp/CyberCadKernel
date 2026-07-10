# native-booleans

## ADDED Requirements

### Requirement: Native `cc_boolean` COMMON builds a Steinmetz bicylinder from facade operands

The engine SHALL produce a VERIFIED watertight native bicylinder COMMON for `cc_boolean(a, b,
2)` when `a` and `b` are two EQUAL-radius cylinders whose axes are ORTHOGONAL and CROSSING,
regardless of whether the operands were built directly (`buildCommonSegment`) or through the
shipping facade (a `cc_solid_extrude_profile` full-circle cylinder plus a
`cc_rotate_shape_about`-located cylinder). A located/rotated cylinder operand SHALL be
recognised identically to an axis-aligned one (the recognizer SHALL fold the operand's
location into its axis/frame before the equal-radius / orthogonality / axis-crossing tests).
The result SHALL be accepted only under the existing curved-boolean self-verify (watertight,
consistently oriented, enclosed volume → the analytic 16·Rc³/3 to the deflection bound). This
path SHALL remain OCCT-free and SHALL NOT change the `cc_*` ABI. A cyl-cyl pair that is NOT a
canonical Steinmetz (unequal radius, non-orthogonal, non-crossing, or near-tangent) SHALL
still decline honestly → OCCT.

#### Scenario: Facade-built equal-radius orthogonal cylinders intersect to a native Steinmetz (host)

- GIVEN two equal-radius cylinders whose axes cross orthogonally, one built by extruding a full-circle profile and the other by rotating such a cylinder 90° (the shipping facade construction), with the native engine active and no OCCT
- WHEN `cc_boolean(a, b, 2)` (common) is invoked
- THEN the native engine SHALL return a watertight, consistently-oriented solid whose enclosed volume matches the analytic bicylinder 16·Rc³/3 to the deflection bound — identical to the result obtained when the same two cylinders are built directly via `buildCommonSegment`

#### Scenario: The facade builds the native Steinmetz body on the simulator (sim)

- GIVEN the cc_* facade on a booted iOS simulator with the native engine, and a Steinmetz bicylinder built via `cc_boolean(cylZ, cylX, 2)` (a `cc_solid_extrude_profile` full-circle cylinder plus a `cc_rotate_shape_about`-located crossing cylinder)
- WHEN the native body is built and, separately, the OCCT oracle builds and mass-measures the same bicylinder
- THEN the native `cc_boolean` COMMON SHALL now return a non-zero body whose `cc_mass_properties` volume matches the analytic bicylinder 16·Rc³/3 (and the OCCT oracle) to the facade deflection bound — the sim canal cases SHALL report a `native-body` PASS, closing the body-build gap that previously held the whole path at a `native-note`

#### Scenario: Composing the landed canal fillet through the facade is the sharpened next blocker (honest decline)

- GIVEN the native Steinmetz body built through the facade (above), whose native SSI assembler emits the crossing lens as a FACETED planar-facet shell (welded triangulated lune patches) rather than a smooth B-rep with two crossing-crease arc edges
- WHEN the sim (and the host `native_facade_steinmetz_common` gate) enumerate the body's edges to seat `cc_fillet_edges` on the crossing crease
- THEN NO single smooth crossing-crease ARC is exposed (only 2-point facet edges), so composing the LANDED `moat-canal-cyl-cyl-fillet` fillet through the facade is honestly DECLINED at this cyl↔cyl COMMON recogniser stage and recorded as a measured `native-note`; the remaining work is a smooth-crease native boolean emit (an assembler-representation change), NOT a change to the recogniser, the fillet builder (host-gated in `test_native_blend`), or the tessellator
