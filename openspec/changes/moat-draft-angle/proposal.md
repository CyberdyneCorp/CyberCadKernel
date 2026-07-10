# Proposal — moat-draft-angle (MOAT feature — draft angle)

## Why

DRAFT ANGLE — tapering faces about a neutral plane along a pull direction — is the
molding / manufacturing feature every professional CAD (SolidWorks, Fusion, Onshape)
ships and CyberCad's app will need for any part destined for injection molding, casting,
or die work. It is not expressible through the landed push/pull tools: `cc_offset_face`
moves a face parallel to itself, `cc_replace_face` / `cc_replace_face_to_plane` retarget
a single face, but neither TILTS a face about its trace with a neutral plane so a wall
tapers for mold release. No `cc_*` entry exists yet, so the app's draft path can only
reach OCCT `BRepOffsetAPI_DraftAngle`.

Draft on the tractable PRISMATIC case is a bounded **assembly-of-landed-parts** slice,
not new geometry: each drafted planar side face pivots on its trace line with the neutral
plane (the trace stays fixed, the face tilts by the draft angle about the pull axis), and
because a draft only REMOVES stock, each drafted face is a pure inward TRIM — one already
gated DM1 `splitByPlane` half-space cut. A multi-face draft is a sequence of such cuts,
the planes all derived UP FRONT from the original face geometry, then one composite
self-verify.

## What changes

- **New OCCT-free header `src/native/feature/draft_faces.h`** —
  `feature::draftFaces(solid, faceIds, faceCount, angleRad, neutralOrigin, pull)` derives
  each drafted target plane (pivot on the trace `L = F ∩ N`, normal `Rot(t̂, ±θ)·n̂_F`)
  from the ORIGINAL face geometry and applies it as an inward `nb::splitByPlane` trim,
  then re-audits the composite (watertight closed 2-manifold, single lump χ=2,
  consistently oriented, positive volume strictly SMALLER than the original — a draft only
  removes stock). `DraftFacesDecline` (NonPrismaticOrForeign / NonPlanarNeutral /
  FaceParallelToPull / DegenerateAngle / ResolveFailed).
- **Additive `cc_draft_faces` in the C ABI** — signature-styled like the landed
  `cc_replace_face` / `cc_fillet_edges` feature ops. `NativeEngine::draft_faces` serves a
  native all-planar body via `draftFaces` (self-verified, else honest decline);
  `OcctEngine::draft_faces` is the `BRepOffsetAPI_DraftAngle` oracle. No existing `cc_*`
  signature changes.

## Impact

- DM1/DM2 (`split_plane.h`, `replace_face.h`) and M0/M1/M2 remain **byte-frozen**; the new
  header is an additive `feature/` sibling next to `wrap_emboss.h`.
- `src/native/**` stays OCCT-FREE (descriptive comments only).
- Tessellator, boolean, analysis, and exchange tracks are UNTOUCHED.
- Curved base / non-planar neutral / a cap face perpendicular to the pull axis / a
  degenerate or ≥90° angle / a self-intersecting draft are honest-declined (measured
  reason) → OCCT; never a wrong / leaky / grown solid.
