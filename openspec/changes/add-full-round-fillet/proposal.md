## Why

A **full-round (rolling-ball) fillet** replaces a narrow face — or the thin strip
between two side faces — with a blend surface tangent to both neighbouring faces,
CONSUMING the middle face (`ROADMAP.md` Phase 3; GitHub #285). This is the classic
"round off the end of a rib/boss" operation: the top face of a rib disappears and
a rolling-ball surface tangent to both side walls takes its place. OCCT's stock
`BRepFilletAPI_MakeFillet` rounds an EDGE by a fixed radius; it does not offer a
"remove this face and blend its two neighbours tangentially" operation, and the
radius of a full round is determined by the geometry (the gap between the
neighbours), not given. So this is a feature the app needs that OCCT does not
provide directly, even though it can be built on OCCT surface/fillet primitives.

## What Changes

- Add additive `cc_*` full-round entry points:
  - `CCShapeId cc_full_round_fillet(CCShapeId body, int faceId)` — replace the
    single narrow `faceId` with a rolling-ball blend tangent to its two opposite
    neighbour faces (the middle face is consumed).
  - `CCShapeId cc_full_round_fillet_faces(CCShapeId body, int leftFaceId, int
    middleFaceId, int rightFaceId)` — the explicit three-face form: consume
    `middleFaceId`, blend tangent to `leftFaceId` and `rightFaceId`.
- Implement in the OCCT adapter, building on OCCT primitives:
  - Identify the middle face and its two neighbour (side) faces and the two shared
    edges (middle↔left, middle↔right).
  - Build a **rolling-ball blend**: the largest surface tangent to both side faces
    across the strip the middle face occupied (a variable-radius/`BRepFill`
    tangent blend, or OCCT's `ChFi3d`/`BRepOffsetAPI` blend machinery driven to a
    face-consuming full round), producing a face G1-tangent to both neighbours at
    the two seams.
  - **Rebuild the solid** with the middle face removed and the blend face sewn in
    (`BRepBuilderAPI_Sewing` + `ShapeFix`), gated on `BRepCheck_Analyzer::IsValid`.
- **Honest fallback**: if a true face-consuming full round cannot be built for a
  given case, fall back to a standard valid edge fillet (e.g. fillet the two
  middle-face edges at a radius derived from the strip width) that is still
  `BRepCheck_Analyzer::IsValid`, and mark the case **deferred** with a note — never
  fake the tangency/consumption result.

C ABI change is **ADDITIVE only**: two new entry points; no existing `cc_*`
signature or POD layout changes. OCCT-only; the host stub returns `0`.

## Capabilities

### New Capabilities
- `full-round-fillet`: a rolling-ball / full-round fillet
  (`cc_full_round_fillet` / `cc_full_round_fillet_faces`) that consumes a narrow
  middle face and replaces it with a blend surface G1-tangent to both neighbour
  faces, rebuilding a `BRepCheck_Analyzer::IsValid`, watertight solid in which the
  target face is gone and the blend is tangent to both neighbours at the seam
  (verified by sampled surface normals). When a true full round is not achievable
  for a case, it falls back to a valid standard edge fillet and is marked deferred.
  Builds on `engine-adapter` (OCCT surface/fillet/sew primitives).

## Impact

- **Contract**: delivers the #285 full-round fillet the app needs, augmenting the
  stock `cc_fillet_edges` (which cannot consume a face); `occt-usage` §Fillets &
  chamfers limitation.
- **App**: gains a face-consuming rolling-ball round for ribs/bosses/tabs.
- **Build**: OCCT-only; guarded by `CYBERCAD_HAS_OCCT`; host stub returns `0`.
- **Determinism**: fixed neighbour/seam identification and blend construction;
  reproducible.
- **Risk / honesty**: full-round blends are hard; where a true tangent
  face-consuming round is not achievable, the op returns a VALID standard fillet
  and the case is marked deferred with the measured tangency gap — never a faked
  G1 claim.
