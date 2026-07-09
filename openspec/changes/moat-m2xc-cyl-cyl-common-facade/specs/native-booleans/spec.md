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

#### Scenario: The landed native canal fillet becomes reachable via the facade (sim)

- GIVEN the cc_* facade on a booted iOS simulator and a Steinmetz bicylinder built via `cc_boolean(cylZ, cylX, 2)` under the native engine
- WHEN `cc_fillet_edges` rounds its crossing crease under the native engine and, separately, under OCCT
- THEN the native canal fillet (already landed in `moat-canal-cyl-cyl-fillet`) SHALL now run on the native body — watertight + consistently oriented + material-removing — and the sim canal cases SHALL report a full native `mass` + `tessellate` + `occt-parity` PASS instead of the body-build `native-note`
