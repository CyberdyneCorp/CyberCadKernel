# kernel-facade

## ADDED Requirements

### Requirement: Additive loft-variant ABI entries (`cc_loft_circles`, `cc_loft_circle_wire`, `cc_loft_typed`, `cc_loft_along_rails`)

The plain-C facade SHALL expose four additive loft entry points, signature-matched to the
app's `KernelBridgeAPI.h`, so the CyberCad app links the kernel product with its existing
call-sites unchanged:

- `CCShapeId cc_loft_circles(const double *c1, const double *n1, double r1, const double *c2, const double *n2, double r2)` — a loft between two TRUE circles (centre, unit normal, radius), producing a smooth conical/cylindrical B-rep (one side face, two circular edges), NOT a faceted polygon.
- `CCShapeId cc_loft_circle_wire(const double *cc, const double *cn, double cr, const double *wXYZ, int wCount)` — a loft from a true circle section to an arbitrary polygon wire (x,y,z triplets), keeping the circle rim a single edge (no faceting).
- `CCShapeId cc_loft_typed(const CCProfileSeg *segsA, int countA, const double *splineA, int splineACount, const double *frameA, const CCProfileSeg *segsB, int countB, const double *splineB, int splineBCount, const double *frameB)` — a loft between two TYPED section profiles (line/arc/circle/spline loops with spline side-channels), each placed on its own plane frame (origin(3)+u(3)+v(3)), so curved boundaries stay true B-rep curve edges.
- `CCShapeId cc_loft_along_rails(const double *railXYZ, int railCount, const double *guideXYZ, int guideCount, const double *profileA_XY, int aCount, const double *profileB_XY, int bCount)` — a two-rail sweep: `railXYZ` is the spine and `guideXYZ` steers as an auxiliary spine; it SHALL fall back to the single-rail sweep (guide dropped) if the guided build fails, and return 0 only if even that fails.

Each SHALL route through the active engine (the standard guarded `cc_*` seam), return 0 with
`cc_last_error` set on degenerate/invalid input, and add NO new public type (`CCProfileSeg`
already exists). These declarations SHALL be purely additive — no existing `cc_*` signature
changes.

#### Scenario: Circle→circle loft converges to the closed-form frustum (host/sim, OCCT oracle)

- GIVEN two coaxial circles — radius r at z=0 and radius R at z=h, both with the +Z normal — and a B-rep engine
- WHEN `cc_loft_circles` is invoked
- THEN it SHALL return a non-zero `CCShapeId` whose `cc_mass_properties` volume matches the closed-form conical frustum `π·h·(r² + r·R + R²)/3` (the cylinder `π·r²·h` when r==R) to the deflection bound

#### Scenario: Degenerate loft input is an honest decline

- GIVEN a null section pointer, a non-positive radius, or a wire with fewer than 3 points
- WHEN any of the four loft entries is invoked
- THEN it SHALL return 0 AND set `cc_last_error`, never a fabricated or wrong-topology solid

#### Scenario: Native engine declines the loft variants to the OCCT oracle (transition)

- GIVEN the native engine is active
- WHEN a loft-variant entry is invoked (which requires true analytic-circle / spline rim topology outside the native ruled-loft envelope)
- THEN the native engine SHALL DECLINE and the operation SHALL fall through to the OCCT oracle, and SHALL NEVER emit a faceted approximation that silently changes the promised topology

### Requirement: Additive solid-enumeration ABI entries (`cc_shape_solid_count`, `cc_shape_solid_at`)

The plain-C facade SHALL expose two additive solid-enumeration entry points, signature-matched
to the app's `KernelBridgeAPI.h`:

- `int cc_shape_solid_count(CCShapeId body)` — the number of connected solids in a body (a body may be a compound of several disjoint lumps); 0 if the body has no solids or is unknown.
- `CCShapeId cc_shape_solid_at(CCShapeId body, int index)` — the `index`-th connected solid (index is 0-based), registered as its own independent shape; 0 on an out-of-range index or failure.

The count SHALL equal an `TopExp_Explorer(TopAbs_SOLID)`-style enumeration (each disjoint lump
once). The native engine SHALL serve these NATIVELY for a native body (the native `Explorer`
over `ShapeType::Solid`, matching the OCCT explorer for disjoint lumps) and forward an OCCT body
to the OCCT engine. These declarations SHALL be purely additive.

#### Scenario: Enumerate a multi-lump compound (host analytic + sim parity)

- GIVEN a body that is a compound of exactly two disjoint solids of known volumes V1 and V2
- WHEN `cc_shape_solid_count` is invoked
- THEN it SHALL return 2
- AND `cc_shape_solid_at(body, 0)` and `cc_shape_solid_at(body, 1)` SHALL each return a non-zero shape that itself enumerates to `cc_shape_solid_count == 1`, with volumes V1 and V2 (summing to the whole)

#### Scenario: Single solid enumerates to one

- GIVEN a body that is a single connected solid
- WHEN `cc_shape_solid_count` is invoked
- THEN it SHALL return 1 AND `cc_shape_solid_at(body, 0)` SHALL return an equivalent single solid

#### Scenario: Out-of-range index is an honest zero

- GIVEN a body with N solids
- WHEN `cc_shape_solid_at(body, index)` is invoked with `index < 0` or `index >= N`
- THEN it SHALL return 0 without crashing
