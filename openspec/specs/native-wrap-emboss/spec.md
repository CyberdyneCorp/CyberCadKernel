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
positive enclosed volume, AND — because an emboss ADDS material — its volume SHALL be
strictly GREATER than the base body's by a magnitude within a documented plausible
band of the wrapped footprint area × `height` (not merely "changed"). If EITHER check
fails (not watertight, or the volume does not increase by a plausible amount), OR the
native builder returns a NULL Shape (out-of-slice input), the engine SHALL DISCARD the
native result and fall through to the OCCT `cc_wrap_emboss` oracle. The engine SHALL
NEVER emit an unverified, leaky, or wrong-volume embossed solid, and SHALL NEVER weaken
a tolerance to pass.

#### Scenario: A bad native wrap-emboss result is discarded and the call falls through (host)
- GIVEN a native wrap-emboss candidate that is open / non-manifold OR whose volume does not increase by a plausible footprint-area × height amount, built on the host
- WHEN the self-verify guard is applied
- THEN the guard SHALL reject the candidate AND `NativeEngine` SHALL fall through to the OCCT `cc_wrap_emboss` oracle for that call (no leaky or wrong solid is emitted)

#### Scenario: Out-of-slice inputs defer to OCCT (never faked)
- GIVEN a wrap-emboss request that is NOT the supported slice — `boss = 0` (deboss), a non-rectangular / complex / dense profile, a freeform or non-cylindrical base face, or a footprint that wraps `> 2π` / self-overlaps / exceeds the face's axial extent
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

