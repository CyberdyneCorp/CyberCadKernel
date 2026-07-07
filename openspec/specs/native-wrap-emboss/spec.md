# native-wrap-emboss Specification

## Purpose
TBD - created by archiving change add-native-wrap-emboss. Update Purpose after archive.
## Requirements
### Requirement: Native rectangular-pad-on-cylinder emboss

The kernel SHALL provide a native, **OCCT-free** wrap-emboss builder that computes
`cc_wrap_emboss(body, faceId, profileXY, count, height, boss)` NATIVELY when ALL of
the following hold: `boss != 0` (emboss / add material), `body` is a native solid,
`faceId` resolves to a `FaceSurface`-kind-`Cylinder` LATERAL face (radius `R`, axis
`A`), and the profile is a simple closed **RECTANGLE** (4 points, positive area) that
fits within the face's angular (`< 2π`) and axial extent without self-overlap.

For such an input the builder SHALL:
- **project the rectangular footprint onto the cylinder** by the wrapping map
  arc-length `px → u = px / R` (angle), axial `v = py + vMid` (centred on the face's
  V-mid) — so the two constant-angle footprint edges become straight axial segments on
  the cylinder and the two constant-axial edges become circular arcs on the cylinder;
- **build the raised pad** as an explicit surface set: an OUTER CAP that is a
  `Cylinder` patch at radius `R + height` over the footprint's angular × axial box, two
  AXIAL side walls that FOLLOW the cylinder (radial-plane strips from `R` to
  `R + height` along the two constant-angle edges), and two CIRCUMFERENTIAL side walls
  (planar strips at the two constant-axial edges spanning `ρ ∈ [R, R + height]`), all
  tiled into deflection-bounded facets; and
- **WELD the pad into the base cylinder** by rebuilding the WHOLE embossed solid as one
  deflection-bounded planar-facet soup and assembling it through the native
  `src/native/boolean` `assembleSolid`: the base cylinder lateral wall is retiled over
  the full turn with the footprint WINDOW removed, and the pad's four walls close that
  window against the base along the SHARED footprint-boundary vertices (the wall window
  and the pad's inner edges reuse the same angular sample sequence). The pad-side-wall ∩
  base-cylinder seam is therefore expressed by coincident welded vertices — the curved-
  boolean intersection of the wrapped footprint curve — with no doubled/coincident
  faces and no fragile solid-solid boolean. (This mirrors the `native-blends` curved-
  fillet slice, which welds a torus quarter-tube into the cylinder+cap the same way.)

The result SHALL be a native `topology::Shape` of type `Solid`, watertight, whose
enclosed volume is strictly GREATER than the base body's by a magnitude consistent
with the wrapped footprint area × `height`, and it SHALL match the OCCT
`cc_wrap_emboss` oracle for the same input within a curved-parity tolerance
(volume / area / watertight). The builder SHALL remain OCCT-free and SHALL reference
no OCCT / `IEngine` / `EngineShape` type. Because the weld is expressed entirely in the
`src/native/boolean` `assembleSolid` (no S3 SSI substrate), the native path SHALL build
and run in BOTH the default and `CYBERCAD_HAS_NUMSCI` configurations, and a build
without the substrate SHALL still gain the native rectangular-pad-on-cylinder path
(everything outside the slice keeps the OCCT-only behaviour). No `cc_*` signature or
POD struct SHALL change.

#### Scenario: Rectangular pad embossed on a cylinder is a watertight solid with the added volume (host)
- GIVEN a native cylinder solid of radius `R` and height `H` (built on the host, in either the default or `CYBERCAD_HAS_NUMSCI` configuration) and a rectangular profile of wrapped width `w` and axial height `t` centred on the lateral face's V-mid, with `boss = 1` and pad `height`
- WHEN `cc_wrap_emboss(cyl, lateralFaceId, rect, 4, height, 1)` is computed and the result is tessellated by `src/native/tessellate`
- THEN the result SHALL be a watertight `Solid` (`boundaryEdgeCount == 0`) AND its enclosed volume SHALL equal `|cyl| + (wrappedFootprintArea × height)` within the tessellation deflection bound, where `wrappedFootprintArea` is the wrapped rectangle's area on the cylinder

#### Scenario: Every projected footprint corner lies on the cylinder (host)
- GIVEN the same cylinder and rectangular profile, built on the host
- WHEN the native builder projects the footprint onto the cylinder
- THEN every projected footprint corner `(u,v)` SHALL lie on the cylinder of radius `R` within a scale-derived tolerance, BEFORE the pad is built

#### Scenario: Native emboss matches the OCCT cc_wrap_emboss oracle (parity)
- GIVEN the same cylinder body, lateral face, rectangular profile, `height`, and `boss = 1` on a booted iOS simulator
- WHEN `cc_wrap_emboss` is called with the native engine active (`cc_set_engine(1)`) and with the OCCT default (`cc_set_engine(0)`)
- THEN the two results' exact/measured volume, surface area, and watertightness (`BRepCheck_Analyzer::IsValid`) SHALL agree within the curved-parity tolerance, and the reported native-vs-OCCT deltas SHALL be recorded

### Requirement: Mandatory wrap-emboss self-verify (discard and fall through to OCCT)

The engine SHALL accept a native wrap-emboss result as native ONLY when it PASSES a
mandatory self-verify: the candidate SHALL be a **closed watertight 2-manifold** with
positive enclosed volume, AND its volume SHALL change with the SIGN DERIVED FROM
`boss` — for an emboss (`boss != 0`, ADDS material) strictly GREATER than the base
body's, for a deboss (`boss == 0`, REMOVES material) strictly LESS than the base
body's — by a magnitude within a documented plausible band of the wrapped footprint
area × `height`/`depth` (not merely "changed"). The wrapped footprint area SHALL be
the TRUE profile area: the SHOELACE area of the `(px, py)` polygon (exact for a
rectangle, correct for any polygon). If ANY
check fails (not watertight, or the volume does not change by the correct sign and a
plausible amount), OR the native builder returns a NULL Shape (out-of-slice input),
the engine SHALL DISCARD the native result and fall through to the OCCT
`cc_wrap_emboss` oracle. The engine SHALL NEVER emit an unverified, leaky, or
wrong-signed embossed solid, and SHALL NEVER weaken a tolerance to pass.

#### Scenario: A bad native wrap-emboss result is discarded and the call falls through (host)
- GIVEN a native wrap-emboss candidate that is open / non-manifold OR whose volume does not change by the correct sign and a plausible footprint-area × height/depth amount, built on the host
- WHEN the self-verify guard is applied
- THEN the guard SHALL reject the candidate AND `NativeEngine` SHALL fall through to the OCCT `cc_wrap_emboss` oracle for that call (no leaky or wrong solid is emitted)

#### Scenario: The self-verify uses the true polygon (shoelace) footprint area (host)
- GIVEN a native NON-RECTANGULAR polygon emboss whose bounding-box area exceeds its true shoelace area
- WHEN the self-verify computes the expected added volume
- THEN it SHALL use the SHOELACE polygon area (not the bounding box) so the volume band gates on the ACTUAL added material, and a candidate whose volume matches the bbox area but not the shoelace area SHALL be rejected → OCCT

#### Scenario: Out-of-slice inputs defer to OCCT (never faked)
- GIVEN a wrap-emboss request that is NOT a supported track — a self-intersecting / dense profile, a sphere / general freeform base face, a deboss with `depth ≥ R`, or a footprint that wraps `> 2π` / self-overlaps / exceeds the face's axial extent
- WHEN `cc_wrap_emboss` is invoked with the native engine active
- THEN the native builder SHALL return a NULL Shape (or the self-verify SHALL discard the candidate) AND the engine SHALL fall through to the OCCT `cc_wrap_emboss` oracle — it SHALL NOT emit an approximate, hand-tuned, or fabricated embossed body

### Requirement: Native wrap-emboss adds no ABI and keeps the OCCT oracle authoritative

The native wrap-emboss slice SHALL NOT add or change any `cc_*` entry point,
signature, or POD struct: it is a NATIVE path behind the existing `cc_wrap_emboss`.
The Phase-3 OCCT cap-and-side + healed-sew builder (`occt_wrap_emboss.cpp`, #290)
SHALL remain the verification ORACLE and the fallback for every case the native slice
does not cover, and SHALL NOT be modified or regressed by this change.

#### Scenario: ABI unchanged and the OCCT oracle is preserved
- GIVEN the host app previously linked the kernel and the Phase-3 OCCT wrap-emboss passes its existing tests
- WHEN it links the version with the native wrap-emboss slice
- THEN `cc_wrap_emboss` SHALL keep its exact signature, the ABI contract test (`tests/test_abi.cpp`) SHALL still pass, AND the existing OCCT wrap-emboss behaviour and tests SHALL be unchanged (the native slice only adds a path in front of the same oracle)

### Requirement: Native rectangular deboss pocket on a cylinder (T1)

The kernel SHALL compute `cc_wrap_emboss(body, faceId, profileXY, count, depth,
boss)` NATIVELY, OCCT-free, for a RECESSED rectangular DEBOSS when ALL of the
following hold: `boss == 0` (remove material), `body` is a native solid, `faceId`
resolves to a `FaceSurface`-kind-`Cylinder` LATERAL face (radius `R`, axis `A`),
the profile is a simple closed RECTANGLE (4 points, positive area) fitting within
the face's angular (`< 2π`) and axial extent without self-overlap, and the pocket
`depth` satisfies `0 < depth < R` (the floor stays strictly off the axis).

For such an input the builder SHALL mirror the raised-pad build inward: project the
footprint onto the cylinder (`u = px/R`, `v = py + vMid`); build the pocket FLOOR as
a `Cylinder` patch at radius `R − depth`; build the two AXIAL and two
CIRCUMFERENTIAL side walls spanning `ρ ∈ [R − depth, R]` with outward normals
facing INTO the pocket; retile the base lateral wall over the full turn with the
footprint WINDOW removed (sharing the window's angular samples) and close the pocket
against the base along the SHARED footprint-boundary vertices; and weld the whole
recessed solid as one deflection-bounded planar-facet soup via the native
`assembleSolid`. The result SHALL be a watertight `Solid` whose enclosed volume is
strictly LESS than the base body's by a magnitude consistent with the wrapped
footprint area × `depth`. The builder SHALL remain OCCT-free and SHALL build and run
in BOTH the default and `CYBERCAD_HAS_NUMSCI` configurations. For `depth ≥ R`, a
non-rectangular profile, or a non-cylindrical base, the builder SHALL return a NULL
Shape → OCCT. No `cc_*` signature or POD struct SHALL change.

#### Scenario: Rectangular deboss is a watertight solid with the removed volume (host)
- GIVEN a native cylinder solid of radius `R` and height `H` (built on the host, in either the default or `CYBERCAD_HAS_NUMSCI` configuration) and a rectangular profile of wrapped width `w` and axial height `t` centred on the lateral face's V-mid, with `boss = 0` and pocket `depth` (`0 < depth < R`)
- WHEN `cc_wrap_emboss(cyl, lateralFaceId, rect, 4, depth, 0)` is computed and tessellated by `src/native/tessellate`
- THEN the result SHALL be a watertight `Solid` (`boundaryEdgeCount == 0`) AND its enclosed volume SHALL equal `|cyl| − (wrappedFootprintArea × depth)` within the tessellation deflection bound

#### Scenario: A too-deep pocket declines to OCCT (host)
- GIVEN the same cylinder and a rectangular deboss whose `depth ≥ R` (the floor would reach or cross the axis)
- WHEN `cc_wrap_emboss` is invoked with the native engine active
- THEN the native builder SHALL return a NULL Shape AND the engine SHALL fall through to the OCCT `cc_wrap_emboss` oracle — no fabricated pocket is emitted

#### Scenario: Native deboss matches the OCCT cc_wrap_emboss oracle (parity)
- GIVEN the same cylinder body, lateral face, rectangular profile, `depth`, and `boss = 0` on a booted iOS simulator
- WHEN `cc_wrap_emboss` is called with the native engine active (`cc_set_engine(1)`) and with the OCCT default (`cc_set_engine(0)`)
- THEN the two results' measured volume, surface area, and watertightness (`BRepCheck_Analyzer::IsValid`) SHALL agree within the curved-parity tolerance, and the reported native-vs-OCCT deltas SHALL be recorded

### Requirement: Native non-rectangular polygon emboss on a cylinder (T2)

The kernel SHALL compute `cc_wrap_emboss` NATIVELY, OCCT-free, for a RAISED emboss
(`boss != 0`) whose footprint is an arbitrary CLOSED SIMPLE POLYGON — `count ≥ 3`
points in `(px, py)` with positive shoelace area, non-self-intersecting, fitting
within the cylinder lateral face's angular (`< 2π`) and axial extent — projected
onto the cylinder by the SAME map (`u = px/R`, `v = py + vMid`).

For such an input the builder SHALL: triangulate the polygon in `(u, v)` (a fan for
a convex polygon, ear-clipping for a simple non-convex polygon) and map it to the
cap at radius `R + height`, subdividing each triangle's angular extent so the
sagitta `R(1 − cos Δu/2) ≤ deflection`; emit one ruled SIDE WALL per polygon edge
from `R` to `R + height` (a helical edge tiled into deflection-bounded chords);
retile the base lateral wall over the full turn with the POLYGON WINDOW removed via
a constrained triangulation that SHARES every polygon-boundary vertex with the cap
inner edge and the side-wall R-side edge; and weld the whole embossed solid via the
native `assembleSolid`. The result SHALL be a watertight `Solid` whose enclosed
volume is strictly GREATER than the base body's by a magnitude consistent with the
SHOELACE polygon area × `height`. Convex polygons SHALL build robustly; a simple
non-convex polygon SHALL be accepted ONLY if the mandatory self-verify passes. A
self-intersecting / non-simple polygon, a footprint that wraps `> 2π`, or a
constrained triangulation that cannot close SHALL return a NULL Shape → OCCT. The
builder SHALL remain OCCT-free and SHALL build in BOTH the default and
`CYBERCAD_HAS_NUMSCI` configurations. No `cc_*` signature or POD struct SHALL change.

#### Scenario: Convex polygon (hexagon) emboss is watertight with the shoelace-area volume (host)
- GIVEN a native cylinder solid and a convex HEXAGON profile (`count = 6`, positive shoelace area) centred on the lateral face's V-mid, with `boss = 1` and pad `height`
- WHEN `cc_wrap_emboss(cyl, lateralFaceId, hexagon, 6, height, 1)` is computed and tessellated
- THEN the result SHALL be a watertight `Solid` AND its enclosed volume SHALL equal `|cyl| + (shoelacePolygonArea × height)` within the deflection bound

#### Scenario: Simple non-convex polygon (L-shape) is accepted only under the self-verify (host)
- GIVEN a native cylinder solid and a simple non-convex L-SHAPED profile centred on the lateral face's V-mid, with `boss = 1`
- WHEN the native builder ear-clips and welds the emboss and the mandatory self-verify is applied
- THEN a watertight result whose volume grows by the shoelace area × height SHALL be accepted natively, OTHERWISE the builder result SHALL be discarded and the call SHALL fall through to the OCCT `cc_wrap_emboss` oracle

#### Scenario: A self-intersecting polygon declines to OCCT (host)
- GIVEN a native cylinder solid and a self-intersecting (non-simple) polygon footprint
- WHEN `cc_wrap_emboss` is invoked with the native engine active
- THEN the native builder SHALL return a NULL Shape AND the engine SHALL fall through to the OCCT `cc_wrap_emboss` oracle — no fabricated emboss is emitted

### Requirement: Freeform (cone / sphere) base declines to OCCT (T3)

The native wrap-emboss builder SHALL return a NULL Shape for ANY non-cylindrical
base — a `FaceSurface`-kind-`Cone`, `Sphere`, `Torus`, `BSpline`, `Bezier`, or
planar face — so the engine falls through to the OCCT `cc_wrap_emboss` oracle. NO
native cone / sphere / freeform builder is provided by this change: the native path
recovers ONLY a `Cylinder` lateral face, and a non-cylinder face is rejected at that
gate. There is therefore NO dead, never-accepted cone or sphere code path.

The recorded blocker is that the OCCT `cc_wrap_emboss` ORACLE is itself
CYLINDER-ONLY — it rejects a `GeomAbs_Sphere` / `GeomAbs_Cone` lateral face ("face
is not cylindrical") — so a native cone / sphere path would have NEITHER a working
OCCT fallback NOR a parity oracle to certify against. Teaching the OCCT oracle the
cone / sphere case is scope beyond this change and is deferred to a future change
that widens the oracle first. The native builder SHALL remain OCCT-free. No `cc_*`
signature or POD struct SHALL change.

#### Scenario: A cone / sphere / freeform base declines to OCCT (host)
- GIVEN a native body whose picked face is a Cone, Sphere, Torus, BSpline, Bezier, or planar face
- WHEN `cc_wrap_emboss` is invoked with the native engine active
- THEN the native builder SHALL return a NULL Shape AND the engine SHALL fall through to the OCCT `cc_wrap_emboss` oracle — no fabricated freeform-base emboss is emitted, and no never-accepted native cone / sphere path exists

