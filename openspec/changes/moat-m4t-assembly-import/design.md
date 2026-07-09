# Design — moat-m4t-assembly-import

## STEP entity structure (ISO 10303-42, confirmed against OCCT RWStep readers)

```
#top = SHAPE_REPRESENTATION('', (#mi1, #mi2, ...), #ctx);   -- items list holds MAPPED_ITEMs
#mi1 = MAPPED_ITEM('', #repMap, #target1);   -- (name, mapping_source, mapping_target)
#mi2 = MAPPED_ITEM('', #repMap, #target2);   -- SAME repMap → the reuse mechanism
#repMap = REPRESENTATION_MAP(#origin, #mappedRep);  -- (mapping_origin, mapped_representation)
#origin = AXIS2_PLACEMENT_3D('', ...);       -- the source frame of the shared rep
#mappedRep = SHAPE_REPRESENTATION('', (#brep, #frame), #ctx);  -- the SHARED geometry
#brep = MANIFOLD_SOLID_BREP('', #shell);
#target1 = AXIS2_PLACEMENT_3D('', ...);      -- OR CARTESIAN_TRANSFORMATION_OPERATOR_3D
```

- `MAPPED_ITEM(name, mapping_source→REPRESENTATION_MAP, mapping_target→placement item)`
  (confirmed: `RWStepRepr_RWMappedItem.cxx`, 3 params, source is a `RepresentationMap`,
  target is a `RepresentationItem`).
- `REPRESENTATION_MAP(mapping_origin→placement item, mapped_representation→Representation)`
  (confirmed: `RWStepRepr_RWRepresentationMap.cxx`, 2 params).

## Placement math — reuses the landed Form-A substrate exactly

OCCT `STEPControl_ActorRead::TransferMappedItem` computes, for an `AXIS2_PLACEMENT_3D`
target, `Trsf.SetTransformation(ax3Target, ax3Origin)` — i.e. the map from the origin frame
to the target frame. In this reader that is precisely:

```
T = frameToWorld(target) ∘ frameToWorld(origin)⁻¹
```

which is the SAME expression as `itemDefinedTransform` (Form-A). When the target is a
`CARTESIAN_TRANSFORMATION_OPERATOR_3D`, OCCT applies it directly — matched by the existing
`cartesianOperator(#target)`. Conformality is gated by the existing `classifyPlacement`
(rigid / uniform-scale / mirror else decline); a mirror complements the instance's faces
via `reversedShape()` so tangent-derived world normals stay outward — identical to
`assembly()`.

## New members (all additive, in `class Mapper`)

- `AsmKind` gains `MappedItem`. `assemblyDisposition()` returns `MappedItem` when a
  `MAPPED_ITEM` whose `REPRESENTATION_MAP` reaches exactly one brep exists AND no
  brep-reaching CDSR exists (a mixed file declines — bounded slice). A `MAPPED_ITEM` /
  `REPRESENTATION_MAP` that reaches no brep keeps today's `Decline`.
- `long representationMapBrep(long repMapId)` — resolve `REPRESENTATION_MAP` arg[1]
  (`mapped_representation`) → `brepOfRepresentation(mappedRep)`.
- `std::optional<std::pair<math::Transform,bool>> mappedItemPlacement(const Record& mi)` —
  read `MAPPED_ITEM` arg[1] (repMap → origin) + arg[2] (target); build `T`; classify.
- `topo::Shape mappedAssembly()` — iterate MAPPED_ITEMs in ascending #id, memoize each
  distinct shared brep's mapped solid, locate + (mirror) complement per instance, return a
  Compound (a single MAPPED_ITEM yields a single located Solid).

## Why a bounded slice

- ONE shared brep per `REPRESENTATION_MAP` (a mapped rep listing ≠1 brep declines): keeps
  the instance→solid resolution unambiguous. A mapped rep that is itself an assembly (nested
  MAPPED_ITEM) is out of scope → decline.
- Form-A and Form-B are not composed in one file (mixed → decline): each mechanism is
  self-contained; a real file uses one or the other. This avoids inventing a placement the
  file did not describe.
- PMI *semantics* remain declined: `pmi_scan` already gives the census; a GD&T semantic
  model (tolerance zones, feature-control-frame evaluation, datum reference frames) is a
  large, separate effort with no bounded win here — recorded as the sharpened decline.

## Verification

- **Gate (a) host-analytic** — a hand-authored REPRESENTATION_MAP over a native-written box,
  instanced by N MAPPED_ITEMs at known target frames: each admitted instance's volume =
  closed-form box volume, world bbox = box translated/rotated by the target frame. Decline
  cases assert NULL (no brep in mapped rep; lone REPRESENTATION_MAP; mixed Form-A+B).
- **Gate (b) SIM parity** — the same file read natively AND through OCCT
  `STEPControl_Reader`; the set of solids matches (count + per-solid vol/area/centroid/bbox/
  face+edge counts) within tolerance. OCCT is the oracle.
