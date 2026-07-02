## Context

A full-round (rolling-ball) fillet rolls a ball of the largest fitting radius
along the strip between two side faces, replacing the narrow middle face (the top
of a rib, the end of a boss) with a blend surface tangent to both sides. Unlike a
standard edge fillet — where OCCT's `BRepFilletAPI_MakeFillet` rounds one edge at
a GIVEN radius and keeps the adjacent faces — a full round CONSUMES the middle
face and its radius is DERIVED from the local gap between the neighbours. OCCT has
no stock "consume this face and blend its neighbours" call, so this is a native
feature composed on OCCT surface/fillet/sew primitives.

The construction: from the middle face, find its two neighbour side faces and the
two shared edges; build a blend surface tangent to both side faces across the
strip (a `BRepFill`/`GeomFill` tangent blend, or OCCT's `ChFi3d` blend machinery
driven to a full round); rebuild the solid with the middle face removed and the
blend sewn in; gate on `BRepCheck_Analyzer::IsValid`.

Constraints:
- **Real bars**: result `BRepCheck_Analyzer::IsValid` + watertight; the target
  face is GONE (not in the rebuilt shell's face set); and the blend is G1-tangent
  to both neighbours at the two seams — asserted by sampling surface normals on
  both sides of each seam (`BRepLProp_SLProps`) and requiring they agree within a
  tangency tolerance. Not a trivially-true check.
- **ABI**: two additive entry points; no existing signature changes.
- **Honesty**: hard cases fall back to a valid standard edge fillet and are marked
  deferred with the measured tangency gap — no faked G1/consumption.

## Goals / Non-Goals

Goals:
- `cc_full_round_fillet(body, faceId)` and the explicit
  `cc_full_round_fillet_faces(body, left, middle, right)`.
- A blend surface G1-tangent to both neighbours, with the middle face consumed,
  rebuilt into a valid watertight solid.
- Sampled-normal G1 tangency check at both seams; target-face-gone check.
- Honest fallback to a valid standard edge fillet + deferred note when a true full
  round is not achievable.

Non-Goals:
- G2 curvature continuity (that is `add-g2-blend-fillet`); G1 tangency is the bar
  here.
- Full-round across more than two neighbours / a chain of faces (single strip,
  two neighbours, per call).
- A native (non-OCCT) blend kernel (Phase 4).

## Decisions

- **Two additive entry points.** `cc_full_round_fillet(body, faceId)` auto-finds
  the two opposite neighbours of `faceId`; `cc_full_round_fillet_faces(body, left,
  middle, right)` takes them explicitly for cases where auto-detection is
  ambiguous. Both route through new `IEngine::full_round_fillet` /
  `full_round_fillet_faces` virtuals (default `engine_unsupported`); stub → `0`.
- **Neighbour + seam identification on the CPU/OCCT.** From the middle face, use
  the edge→face adjacency (`TopExp` maps) to find the two side faces sharing the
  middle face's two long edges, and record the two seam edges (middle↔left,
  middle↔right).
- **Blend surface tangent to both sides.** Build the rolling-ball blend as a
  surface tangent to both side faces across the strip — preferring OCCT's blend
  machinery (`ChFi3d`/`BRepOffsetAPI`) driven to consume the middle face, or a
  `BRepFill`/`GeomFill_ConstrainedFilling` tangent-constrained fill between the two
  seam edges with tangency to the side faces. The radius is derived from the local
  strip width (the rolling ball that touches both sides).
- **Rebuild + sew.** Remove the middle face, insert the blend face, and sew
  (`BRepBuilderAPI_Sewing` + `ShapeFix_Shell`/`ShapeFix_Solid`) into a closed
  solid; gate on `BRepCheck_Analyzer::IsValid`.
- **G1 tangency is verified, not assumed.** At sample points along each seam,
  evaluate the blend-face normal and the neighbour-face normal
  (`BRepLProp_SLProps`) and require the angle between them below a tangency
  tolerance (e.g. normals' dot ≥ cos(tol)). This is the real acceptance property.
- **Target-face-gone is verified.** After rebuild, the middle face id SHALL not
  resolve to a face in the new body's face set (its geometry is consumed by the
  blend).
- **Honest fallback.** If the tangent face-consuming blend cannot be built or is
  invalid, fall back to filleting the two middle-face edges with
  `BRepFilletAPI_MakeFillet` at a radius from the strip width — a VALID standard
  fillet — and mark the case deferred with the measured tangency gap. Never report
  a faked full round.

## Risks / Trade-offs

- **Blend construction is hard.** OCCT's blend APIs are finicky about tangency and
  face removal; some geometries will not yield a valid face-consuming round. Those
  fall back to a standard fillet and are deferred — honest, not faked.
- **Neighbour auto-detection ambiguity.** A face with more than two "long"
  neighbours is ambiguous for the single-arg form; the three-face form disambiguates.
  Ambiguous single-arg cases fall back / return `0`.
- **Tangency tolerance choice.** Too loose passes a non-tangent seam; too tight
  fails a numerically-tangent one. Pinned so a clean rib round passes and a plain
  (non-blended) face-swap would fail; documented with the constant.
- **Radius derivation.** The rolling-ball radius from local strip width may vary
  along the seam (variable full round); the blend must remain tangent as the
  radius varies. If a constant-radius approximation is used, it is labelled as an
  approximation and the tangency check still applies.

## Migration Plan

1. Add `IEngine::full_round_fillet(body, faceId)` +
   `full_round_fillet_faces(body, left, middle, right)` (default
   `engine_unsupported`); stub → `0`.
2. OCCT adapter: neighbour + seam identification from `TopExp` adjacency.
3. Build the tangent rolling-ball blend; rebuild the solid with the middle face
   removed and the blend sewn in; gate on `BRepCheck_Analyzer::IsValid`.
4. Add `cc_full_round_fillet` + `cc_full_round_fillet_faces` to the header +
   facade (additive); `test_abi` unchanged.
5. iOS-sim checks: on a rib/boss body, result IsValid + watertight; the target
   middle face id no longer resolves to a face; blend-face normal vs each
   neighbour-face normal at seam samples agree within the tangency tolerance (G1).
6. Fallback path: force a case where the full round fails, assert it returns a
   VALID standard edge fillet and is recorded deferred with the measured gap.
7. Host stub `0`; `run-sim-suite.sh` unchanged (additive).
8. `openspec validate --all --strict`; update `ROADMAP.md` Phase 3 status.

## Open Questions

- Which OCCT blend path (`ChFi3d` vs `GeomFill`/`BRepFill` tangent fill) most
  reliably yields a valid face-consuming tangent round on the test fixtures —
  decided empirically; documented.
- Whether the rolling-ball radius should be constant or variable along the seam;
  if constant is used it is labelled an approximation and the tangency check still
  gates it.
- The G1 tangency tolerance (max normal-angle deviation) at the seam — pinned so a
  clean round passes and a non-tangent swap fails; recorded with the result.
- Which real geometries fall back to a standard fillet (deferred) — recorded
  honestly from the sim results with the measured tangency gap.
