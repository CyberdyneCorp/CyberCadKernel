# wrap-emboss

This change (Feature #7 first slice) records that `cc_wrap_emboss` now has a NATIVE
path for the rectangular-pad-on-cylinder emboss (behind the unchanged signature),
while the Phase-3 OCCT cap-and-side + healed-sew builder remains the oracle and the
fallback for every other case. The observable ABI contract and the volume-change-sign
semantics are unchanged.

## MODIFIED Requirements

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
