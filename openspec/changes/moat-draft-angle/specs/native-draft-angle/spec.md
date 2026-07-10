# native-draft-angle

## ADDED Requirements

### Requirement: Native prismatic draft angle (taper planar side faces about a neutral plane)

The kernel SHALL provide a native, **OCCT-free** draft-angle op that computes
`cc_draft_faces(body, faceIds, faceCount, neutralOrigin, pullDir, angleDeg)` NATIVELY when
ALL of the following hold: `body` is a native solid every one of whose faces is planar
(a prismatic polyhedron), `pullDir` is a non-degenerate direction (the neutral plane's
normal), and each `faceIds[i]` (1-based, `mapShapes` Face order — the same ids
`cc_subshape_ids` / `cc_fillet_edges` use) resolves to a planar face that is NOT
perpendicular to `pullDir` (so it has a trace line on the neutral plane).

For such an input the builder SHALL, per drafted face `F` with outward unit normal `n̂_F`:
- compute the trace direction `t̂ = normalize(n̂_F × p̂)` (with `p̂` the unit pull
  direction) and the tilted outward normal `n̂' = Rot(t̂, φ)·n̂_F`, where
  `φ = sign((centroid − neutralOrigin)·p̂) · angleRad` so a POSITIVE angle tapers the wall
  INWARD as it recedes from the neutral plane along `+p̂`;
- derive the tilted target plane through `footN = centroid − ((centroid − neutralOrigin)·p̂)·p̂`
  (a point of the pivot line `L = F ∩ N`), computed from the ORIGINAL solid's face
  geometry (before any earlier cut fragments a neighbour's mesh); and
- apply the plane as an inward half-space TRIM via the landed DM1 `nb::splitByPlane`,
  keeping the material (bulk) side.

A multi-face draft SHALL be the SEQUENCE of these inward trims. The result SHALL be
accepted ONLY when the composite passes the self-verify: watertight closed 2-manifold,
positive enclosed volume, single genus-0 lump (Euler χ = 2), consistently oriented
(`tess::isConsistentlyOriented`), and volume STRICTLY SMALLER than the original (a draft
only removes stock). Any candidate that fails SHALL be DISCARDED. The native path SHALL
consume `nb::splitByPlane` (boolean/split_plane.h), `rfdetail::readPickedFace` and the
mesh-audit primitives (directmodel/replace_face.h), and `math::Mat3::rotation`
BYTE-IDENTICALLY, adding no change to those landed headers, and SHALL keep the
tessellator, boolean, analysis, and exchange modules UNTOUCHED.

Anything outside this arm SHALL return a NULL Shape (→ OCCT) with a measured
`DraftFacesDecline`: a curved base (not all-planar → `NonPrismaticOrForeign`), a
degenerate pull direction (`NonPlanarNeutral`), a face perpendicular to the pull axis
(a cap, no trace line → `FaceParallelToPull`), a `|angleDeg|` below the numeric floor or
`≥ 90°` (`DegenerateAngle`), or a per-face trim / composite audit that cannot be verified
(a self-intersecting or topology-changing draft → `ResolveFailed`). The engine SHALL
report the decline and SHALL NEVER hand a native void to OCCT.

#### Scenario: Box side face drafted about the base plane (closed-form wedge)

- **WHEN** `cc_draft_faces` drafts one planar side face of a box by θ degrees about the
  base plane (pull along the base normal) under the native engine
- **THEN** the drafted face becomes a trapezoid at the exact taper, the result is a
  watertight, consistently-oriented single lump (χ = 2), and its enclosed volume equals
  the closed-form `V₀ − ½·H·(H·tanθ)·D` (the removed wedge)

#### Scenario: All four side faces drafted (frustum)

- **WHEN** `cc_draft_faces` drafts all four side faces of a box by θ degrees about the
  base plane under the native engine
- **THEN** the result is a watertight truncated pyramid whose volume equals the frustum
  closed form `(H/3)(A_bot + A_top + √(A_bot·A_top))`

#### Scenario: Native-vs-OCCT parity on the simulator

- **WHEN** the same drafts run under the native engine and against the OCCT oracle
  `BRepOffsetAPI_DraftAngle` + `BRepGProp` on a booted iOS simulator
- **THEN** the native and OCCT results agree on volume, area, watertightness, Euler χ,
  bounding box, and one-sided Hausdorff within fixed, tessellation-bounded tolerances

#### Scenario: Honest decline outside the prismatic slice

- **WHEN** the picked base is curved, the pull direction is degenerate, a drafted face is
  perpendicular to the pull axis (a cap), the angle is degenerate or `≥ 90°`, or the draft
  would self-intersect
- **THEN** the native builder returns a NULL Shape with the matching `DraftFacesDecline`
  and the engine falls back to OCCT — never a wrong, leaky, or grown solid

The `cc_*` ABI change SHALL be ADDITIVE (a new `cc_draft_faces` entry only); no existing
signature changes.
