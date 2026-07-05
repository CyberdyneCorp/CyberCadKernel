# wrap-emboss Specification

## Purpose
TBD - created by archiving change add-robust-wrap-emboss. Update Purpose after archive.
## Requirements
### Requirement: Signature-stable robust wrap-emboss
The wrap-emboss SHALL keep the exact existing `cc_wrap_emboss(CCShapeId
body, int faceId, const double *profileXY, int count, double depth, int boss)`
signature and semantics (wrap a 2D profile onto a cylindrical face and add
material when `boss != 0` or remove material when `boss == 0`). The pad MAY be built
by EITHER the Phase-3 OCCT cap-and-side + healed-sew builder (the ORACLE and the
default for the general case) OR, for the supported first-slice input (an emboss of a
RECTANGULAR profile onto a CYLINDER lateral face), a NATIVE OCCT-free builder selected
behind the same signature. In all cases no `cc_*` signature or POD struct layout SHALL
change, the operation SHALL be at least as capable as the previous
`BRepOffsetAPI_ThruSections` implementation (never regress), and any native result
that is not a valid watertight solid with the correct volume-change sign SHALL be
discarded in favour of the OCCT path.

#### Scenario: ABI unchanged
- GIVEN the host app previously linked the kernel
- WHEN it links the version with the robust wrap-emboss
- THEN `cc_wrap_emboss` SHALL keep its signature AND the ABI contract test
  (`tests/test_abi.cpp`) SHALL still pass

#### Scenario: Host stub is a safe no-op
- GIVEN a build with no B-rep engine (the host stub)
- WHEN `cc_wrap_emboss` is called
- THEN it SHALL return `0` without crashing

#### Scenario: Native rectangular-pad-on-cylinder emboss is selected behind the same signature
- GIVEN a native cylinder body, a rectangular profile on its lateral face, `boss = 1`, and a build with the native engine active
- WHEN `cc_wrap_emboss` is called with its unchanged signature
- THEN the native rectangular-pad-on-cylinder builder MAY produce the result (subject to the mandatory watertight + volume-increasing self-verify), OTHERWISE the call SHALL fall through to the OCCT `cc_wrap_emboss` oracle — with no change to the signature, POD structs, or observable semantics

### Requirement: Dense high-curvature profile yields a valid watertight solid
The robust pad builder SHALL, for a dense high-curvature profile wrapped onto a
cylindrical face, build the pad as a cap-and-side surface set and sew + heal it
(`BRepBuilderAPI_Sewing` + `ShapeFix`) into a closed solid, and the final
embossed/debossed body SHALL be `BRepCheck_Analyzer::IsValid` and watertight (a
closed shell with no free boundary).

#### Scenario: Embossed dense profile is valid and watertight
- GIVEN a cylinder body and a dense, high-curvature profile, on a booted iOS
  simulator, with `boss = 1`
- WHEN `cc_wrap_emboss` runs
- THEN it SHALL return a non-zero body whose shape is
  `BRepCheck_Analyzer::IsValid` AND watertight (no free/naked edges)

#### Scenario: Debossed dense profile is valid and watertight
- GIVEN the same cylinder body and dense profile, on a booted iOS simulator, with
  `boss = 0`
- WHEN `cc_wrap_emboss` runs
- THEN it SHALL return a non-zero body whose shape is
  `BRepCheck_Analyzer::IsValid` AND watertight

### Requirement: Correct volume-change sign
The wrap-emboss result SHALL change the body's exact B-rep volume in the correct
direction: embossing (`boss != 0`) SHALL make the resulting volume GREATER than
the base body's volume, and debossing (`boss == 0`) SHALL make it LESS than the
base body's volume, by a magnitude consistent with the wrapped profile area times
the emboss depth (not merely "changed").

#### Scenario: Emboss increases volume
- GIVEN the base cylinder volume `V_base` from `cc_mass_properties`
- WHEN `cc_wrap_emboss` with `boss = 1` produces a body with volume `V_after`
- THEN `V_after` SHALL be strictly greater than `V_base`, AND `V_after - V_base`
  SHALL lie within a documented plausible range of the padded profile
  area × depth

#### Scenario: Deboss decreases volume
- GIVEN the base cylinder volume `V_base` from `cc_mass_properties`
- WHEN `cc_wrap_emboss` with `boss = 0` produces a body with volume `V_after`
- THEN `V_after` SHALL be strictly less than `V_base`, AND `V_base - V_after`
  SHALL lie within a documented plausible range of the removed profile
  area × depth

### Requirement: Robust path beats ThruSections, with honest fallback
The robust sewn builder SHALL produce a valid watertight solid for at least one
profile on which the previous `BRepOffsetAPI_ThruSections` pad was NOT
`BRepCheck_Analyzer::IsValid`. If a particular profile still cannot be sewn into a valid
solid, the operation SHALL fall back to the coarse `BRepOffsetAPI_ThruSections`
pad and return a VALID (if lower-fidelity) result, and that case SHALL be recorded
as deferred with a note — it SHALL NOT be reported as a full robust success and
SHALL NOT be faked as passing.

#### Scenario: A previously invalid case now succeeds
- GIVEN a profile for which building the pad with `BRepOffsetAPI_ThruSections`
  yields a shape that is NOT `BRepCheck_Analyzer::IsValid` (captured directly), on
  a booted iOS simulator
- WHEN `cc_wrap_emboss` runs the robust sewn builder for the same profile
- THEN it SHALL return a body that IS `BRepCheck_Analyzer::IsValid` and watertight

#### Scenario: Unsewable case falls back to a valid coarse result and is deferred
- GIVEN a profile that the robust sewn builder cannot heal into a valid solid
- WHEN `cc_wrap_emboss` runs
- THEN it SHALL fall back to the coarse `BRepOffsetAPI_ThruSections` pad and
  return a body that is `BRepCheck_Analyzer::IsValid` (lower fidelity)
- AND the change SHALL record this profile as a deferred case with the measured
  reason, NOT claim a full robust pass for it

### Requirement: Deterministic and OCCT-guarded
The robust pad SHALL be built from a fixed face decomposition and a fixed sew
order so repeated runs on the same input are reproducible, and the robust path
SHALL be compiled only under `#ifdef CYBERCAD_HAS_OCCT` (sewing/`ShapeFix` are
OCCT), leaving the host stub as a safe no-op.

#### Scenario: Repeated wrap-emboss is reproducible
- GIVEN the same body, face, profile, depth, and boss flag on a booted iOS
  simulator
- WHEN `cc_wrap_emboss` runs twice
- THEN both results SHALL have the same exact volume and bounding box within a
  tight tolerance

