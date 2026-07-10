# sheet-metal

## ADDED Requirements

### Requirement: Native base flange

`cc_sheet_base_flange(profileXY, pointCount, thickness)` SHALL build the flat sheet
solid = the closed 2D polygon `profileXY` (x,y pairs on z=0, `pointCount` points)
extruded by `thickness` along +Z, and return a new native body id, or `0` with
`cc_last_error` set on a degenerate profile (fewer than 3 points, or zero area) or a
non-positive thickness. The result SHALL be a watertight, consistently-oriented, single
genus-0 (χ=2) solid whose enclosed volume equals the closed form `|profileArea|·thickness`
(exact for a planar sheet). Sheet metal is native-only; the op SHALL NOT be forwarded to
OCCT (OCCT core has no sheet-metal module) and SHALL NOT fabricate a wrong solid.

#### Scenario: A rectangle base flange has the closed-form volume (host)
- GIVEN a closed 40×20 rectangle profile and thickness 2, on the host with no OCCT
- WHEN `cc_sheet_base_flange` builds the base flange
- THEN the result SHALL be watertight AND consistently oriented AND χ=2 AND its enclosed volume SHALL equal `40·20·2 = 1600` within `1e-6`

#### Scenario: A degenerate base flange honest-declines (host)
- GIVEN a profile with fewer than 3 points, or a thickness ≤ 0
- WHEN `cc_sheet_base_flange` is called
- THEN it SHALL return `0` with `cc_last_error` set AND SHALL NOT forward to OCCT

### Requirement: Native edge flange with a single cylindrical bend

`cc_sheet_edge_flange(body, edgeId, height, bendRadius, angleDeg)` SHALL add one flange
off the STRAIGHT edge `edgeId` (1-based, subshape order) of a recognised rectangular base
flange: a cylindrical BEND of inner radius `bendRadius` swept through `angleDeg` (in the
open interval (0,180)), welded to a planar FLANGE WALL of length `height`, at the base's
constant thickness. The whole part (base + bend + wall) SHALL be emitted as ONE
watertight, consistently-oriented, single genus-0 (χ=2) solid whose enclosed volume
equals the closed form `base + ½·θ·((r+t)² − r²)·W + height·t·W` within a
deflection-set band (the cylindrical bend meshes to the deflection, so the volume
converges to the closed form from below). The op SHALL be NATIVE-ONLY and SHALL NOT be
forwarded to OCCT.

The op SHALL HONEST-DECLINE (return `0` with `cc_last_error` set, a measured reason) and
SHALL NOT fabricate a wrong or self-intersecting solid when: a parameter is degenerate
(height < 0, bendRadius < 0, or angle outside (0,180)); the picked bend line is not a
straight edge; the body is not a recognised single-bend base; or the fold would
self-collide (the flange re-enters the base). Multi-bend interference, the miter between
adjacent flanges, and corner-relief cuts are OUT of this first slice and SHALL decline.

#### Scenario: A 90° edge flange has the closed-form volume and is watertight (host)
- GIVEN a 40×20×2 rectangular base flange and its straight +X rim edge id
- WHEN `cc_sheet_edge_flange` folds a flange with height 15, bendRadius 3, angle 90°
- THEN the result SHALL be watertight AND consistently oriented AND χ=2 AND its enclosed volume SHALL be ≤ the closed form `1600 + ½·(π/2)·(5²−3²)·20 + 15·2·20` and within 1% of it

#### Scenario: A non-straight bend line honest-declines (host)
- GIVEN a picked `edgeId` whose 3D curve is not a straight line
- WHEN `cc_sheet_edge_flange` is called
- THEN it SHALL return `0` with `cc_last_error` set (a non-straight bend line) AND SHALL NOT forward to OCCT

#### Scenario: A degenerate parameter honest-declines (host)
- GIVEN a bend angle of 0, or a negative bendRadius
- WHEN `cc_sheet_edge_flange` is called
- THEN it SHALL return `0` with `cc_last_error` set AND SHALL NOT fabricate a solid

### Requirement: Native flat-pattern unfold

`cc_sheet_unfold(body, kFactor)` SHALL produce the FLAT-PATTERN developed blank of a
single-bend part by unrolling its bend about the neutral fibre, with bend allowance
`BA = angle·(bendRadius + kFactor·thickness)` and `kFactor` in `[0,1]`. The developed
blank SHALL be a planar sheet solid of the same thickness whose footprint is
`(baseRun + BA + flangeHeight) × width`; its developed area SHALL equal
`baseArea + BA·width + flangeArea` and SHALL be INVARIANT under fold→unfold. The blank
SHALL be watertight, consistently oriented, χ=2, with enclosed volume `developedArea ·
thickness` (exact for a planar blank). The op SHALL serve only a body produced by
`cc_sheet_edge_flange`; any other body SHALL HONEST-DECLINE (return `0` with
`cc_last_error` set), and the op SHALL NOT be forwarded to OCCT.

#### Scenario: The unfold develops to the k-factor bend allowance and is area-invariant (host)
- GIVEN a folded part (baseRun 40, width 20, thickness 2, bendRadius 3, angle 90°, flange 15) and kFactor 0.44
- WHEN `cc_sheet_unfold` develops the flat blank
- THEN the developed length SHALL equal `40 + (π/2)·(3 + 0.44·2) + 15` AND the developed area SHALL equal `40·20 + BA·20 + 15·20` AND the blank volume SHALL equal that area × 2 within `1e-6`

#### Scenario: Unfolding an unrecognised body honest-declines (host)
- GIVEN a body that is not a recognised single-bend part (no valid fold record), or a kFactor outside [0,1]
- WHEN `cc_sheet_unfold` is called
- THEN it SHALL return `0` with `cc_last_error` set AND SHALL NOT forward to OCCT

### Requirement: Closed-form arbiter (no OCCT sheet-metal oracle)

Because OCCT core has no sheet-metal module, sheet-metal verification SHALL use CLOSED
FORM as the arbiter rather than an OCCT comparison. Every built sheet-metal solid SHALL
self-verify watertight + consistently-oriented + single genus-0 (χ=2) + positive enclosed
volume within a deflection-set band of the closed form, and SHALL be DISCARDED (honest
decline) if any check fails. Tolerances SHALL NOT be widened to force a marginal case
through, and a native void SHALL NEVER be handed to OCCT.

#### Scenario: Sim self-test verifies validity + closed-form volume under the native engine
- GIVEN the native engine (`cc_set_engine(1)`) on a booted simulator, with NO OCCT sheet-metal comparison
- WHEN the base flange, edge flange, and unfold are built through the cc_* facade
- THEN each SHALL pass `cc_check_solid` (a valid closed 2-manifold) AND its `cc_mass_properties` volume SHALL match the host closed form AND the build SHALL be deterministic
