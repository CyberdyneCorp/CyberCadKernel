# native-topology

This change adds a new analytic surface kind — **`Torus`** — to the `FaceSurface` variant, **additively**,
so that the STEP reader (`native-exchange`) can attach a torus surface to a face and the tessellator
(`native-tessellation`) can mesh it. `math::Torus` already exists in `native-math`; this delta only widens
the topology's surface-kind attachment. The addition is byte-neutral for every existing kind: the new
enumerator is appended (no existing ordinal changes) and the new `minorRadius` field defaults to `0.0`.

## ADDED Requirements

### Requirement: FaceSurface carries an analytic Torus kind additively

The `FaceSurface` surface descriptor SHALL support an analytic **`Torus`** kind alongside the existing
`Plane`, `Cylinder`, `Cone`, `Sphere`, `BSpline`, and `Bezier` kinds, referencing the `native-math`
`Torus` descriptor without duplicating it. A torus `FaceSurface` SHALL carry its placement frame (`Ax3`),
a **major** radius (axis → tube-centre distance, reusing the existing `radius` field), and a **minor**
radius (the tube cross-section radius, a new `minorRadius` field), such that the attached surface equals
`math::Torus{frame, majorRadius = radius, minorRadius}`. The addition SHALL be **additive and
byte-neutral** for every existing kind: the new `Torus` enumerator SHALL be appended so no existing
enumerator's value changes, and the new `minorRadius` field SHALL default such that every existing kind's
in-memory meaning of `radius` / `semiAngle` is unchanged and no existing construction, accessor, or
serialization keys on the enum ordinal. All attached values SHALL be fp64. The topology library SHALL
remain OCCT-free and host-buildable.

#### Scenario: A face reads back an attached torus surface (host)
- GIVEN a `Face` built with an attached `Torus` `FaceSurface` (known frame, major radius `R`, minor radius `r`) on the host with no OCCT
- WHEN its surface is read back
- THEN the face SHALL return a surface of kind `Torus` whose frame, major radius, and minor radius equal the attached values within the documented fp64 tolerance, corresponding to `math::Torus{frame, R, r}`

#### Scenario: Adding the Torus kind leaves every existing kind byte-identical (host)
- GIVEN faces built with each existing surface kind (`Plane`, `Cylinder`, `Cone`, `Sphere`, `BSpline`, `Bezier`) on the host with no OCCT, before and after the `Torus` kind and `minorRadius` field are added
- WHEN each face's surface descriptor is read back and its in-memory layout is inspected
- THEN every existing kind SHALL read back IDENTICAL values (the `radius` / `semiAngle` semantics are unchanged, `minorRadius` defaults so it does not affect any existing kind, and no existing enumerator's ordinal changed) — the addition is purely additive
