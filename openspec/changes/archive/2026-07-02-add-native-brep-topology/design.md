## Context

Phase 4 (`openspec/NATIVE-REWRITE.md`) replaces the OCCT adapter with native
C++20, one capability at a time, behind the unchanged `cc_*` facade. Capability #1
(`native-math`, `src/native/math/`) landed the OCCT-free fp64 value types
(`Vec3`/`Point3`/`Dir3`/`Transform`) and curve/surface evaluators. The locked
dependency order puts `native-topology` second: construction (swept solids),
booleans, blends, and tessellation all operate on a **B-rep topology model** — the
boundary hierarchy that binds evaluated geometry into oriented, connected vertices,
edges, wires, faces, shells, and solids, with the adjacency those algorithms walk.
So topology must land next, and it must be **OCCT-free and host-buildable** — that
property is what enables the two independent verification gates the roadmap
requires:

1. **Host unit tests** — the library compiles and unit-tests with
   `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`, no OCCT, no simulator, against
   topological invariants (counts, orientation round-trips, ancestry/explorer
   symmetry, id stability).
2. **Simulator native-vs-OCCT parity** — on the simulator (OCCT linked) an OCCT
   `TopoDS_Shape` is imported into the native model and the two are compared for
   identical structure, ids, orientation, ancestry, and attached-geometry values.

Constraints:
- **OCCT-free library.** No file under `src/native/topology/` may include any OCCT
  header. It MAY include `src/native/math`. The library links no OCCT. OCCT appears
  ONLY in the simulator parity test, which imports an OCCT shape and compares.
- **Host-buildable.** The library + host tests build with `clang++ -std=c++20` (and
  the `native-math` headers) and nothing else.
- **Clean-room.** Implemented from first principles, the `cc_*` contract, and
  standard B-rep references. OCCT source (`TopoDS`, `TopAbs`, `TopExp`, `TopTools`,
  `BRep_Tool`, `TopLoc_Location`) is a convention oracle — consulted to match
  orientation semantics, sub-shape ordering, and ancestry — never copied verbatim.
- **No ABI change, no engine wiring.** The `cc_*` facade and the active engine are
  untouched; this change ships the library + its tests only.
- **Determinism.** Fixed enumeration order, stable integer ids, fp64 geometry;
  reproducible run to run.
- **Maintainability first.** Clear, well-named C++20 (`std::span`/`std::variant`/
  concepts where natural), documented conventions with their OCCT-oracle citation,
  cognitive complexity in the systems band; the explorer/enumeration recursion is
  the one place that may approach the band and is isolated and flagged.

## Goals / Non-Goals

Goals:
- B-rep shape hierarchy: `Vertex`/`Edge`/`Wire`/`Face`/`Shell`/`Solid`/`Compound`
  as an oriented, located *use* over a shared immutable *underlying* entity.
- Orientation `Forward`/`Reversed` (with `Internal`/`External` reserved) that
  composes correctly through nesting.
- `TopLoc`-style location/placement (a `native-math` `Transform`) composing down the
  hierarchy.
- Geometry attachment: vertex → `Point3` + tol; edge → 3D curve + `[first,last]` +
  optional pcurve-on-face; face → surface + outer/inner wires + tol.
- Stable integer sub-shape ids under a deterministic enumeration matching
  `TopExp::MapShapes` conventions.
- Type-filtered explorer (`TopExp_Explorer` analogue) and sub-shape → parents
  ancestry (`TopExp::MapShapesAndAncestors` analogue).
- `BRep_Tool`-style accessors reading geometry off a shape through its location.
- Two-gate verification: host topological-invariant tests + simulator
  native-vs-OCCT structural parity (import an OCCT shape, compare).

Non-Goals:
- Any construction / modelling operation (make a box, extrude, sweep, revolve),
  booleans, fillets, or healing (later capabilities) — this is the passive data
  model + traversal only, plus a test-only importer to feed the parity gate.
- Any `cc_*` ABI change or facade exposure, and any engine wiring.
- Tessellation / triangulation attached to faces (that is `native-tessellation`).
- Curve/surface intersection, projection, or classification (point-in-solid) — no
  geometric queries beyond reading attached geometry.
- A persistent on-disk shape format (STEP/BREP files) — that is `native-exchange`.

## Decisions

- **Directory + build seam.** All native topology lives under
  `src/native/topology/` (headers + sources), OCCT-free, may include
  `src/native/math`. A host CTest target compiles it with `clang++ -std=c++20` and
  links only the invariant tests. A separate simulator parity target links OCCT and
  includes both the native headers and OCCT to import a shape and compare — OCCT
  never enters the library translation units. A grep gate asserts no OCCT include
  under `src/native/topology/`.
- **Underlying / use split (TShape / TopoDS_Shape).** Following OCCT's proven model,
  each shape is a *use* = `(underlying, orientation, location)` triple, where the
  *underlying* entity is a shared, reference-counted, immutable node holding the
  child list and any geometry. This makes sub-shape *sharing* first-class (the same
  edge referenced by two faces is one underlying node, two uses) — which is exactly
  what stable ids and ancestry need — and keeps uses cheap value-like handles.
- **Shape kinds mirror `TopAbs_ShapeEnum`.** `Compound`, `CompSolid`(reserved),
  `Solid`, `Shell`, `Face`, `Wire`, `Edge`, `Vertex` — same set and same top-down
  ordering so enumeration matches OCCT. A `ShapeType` enum names them.
- **Orientation is `Forward`/`Reversed` and composes.** `Orientation` mirrors
  `TopAbs_Orientation` (`Forward`, `Reversed`, `Internal`, `External`; only the
  first two are exercised now). The effective orientation of a nested sub-shape is
  the composition of the orientations along the path (Forward is identity, Reversed
  flips), matching `TopAbs::Compose`. The explorer yields each use with its
  *resolved* orientation. Semantics (a Reversed face flips its surface normal; a
  Reversed edge traverses its curve backwards) are documented, matched to the OCCT
  oracle, and pinned by parity.
- **Location is a composing `Transform`.** A `Location` wraps a `native-math`
  `Transform` (rigid placement). A sub-shape's effective placement is the product of
  the locations along the path from the root (`TopLoc_Location` semantics). Geometry
  read through an accessor is returned in the resolved (world) placement. Identity
  locations are represented cheaply so the common unplaced case costs nothing.
- **Geometry attachment references `native-math`, never duplicates it.** A `Vertex`
  underlying holds a `Point3` + `double` tolerance. An `Edge` underlying holds a
  handle to a 3D curve (a `native-math` curve descriptor), a parameter range
  `[first,last]`, and a small list of **pcurves**, each a `(Face-underlying, 2D
  curve)` pair so the same edge can carry a different parameter-space curve per
  adjacent face (`BRep_Tool::CurveOnSurface` semantics). A `Face` underlying holds a
  handle to a surface, an ordered wire list with the **first wire designated
  outer** and the rest inner/holes (documented convention matched to the oracle),
  and a `double` tolerance.
- **Deterministic enumeration + stable ids (`TopExp::MapShapes`).** A single
  depth-first walk visits a shape *before* its sub-shapes, recursing children in
  fixed per-type order; the first time an underlying entity is seen it is assigned
  the next integer id and recorded, and a re-visit of a shared sub-shape reuses its
  id (no duplicate node, matching `MapShapes` cumulative-map behaviour). Ids are
  therefore stable and share correctly. The same walk backs the explorer and the
  ancestry builder so all three agree by construction.
- **Explorer is a filtered view of the enumeration.** `Explorer(shape, ShapeType)`
  yields, in enumeration order, each *use* of the requested type with its resolved
  orientation and location — the `TopExp_Explorer` analogue. It is a lazy forward
  iterator over the same deterministic walk, so it never allocates a full list
  unless the caller collects it.
- **Ancestry is one pass over the walk.** `Ancestors(shape, childType, parentType)`
  (the `MapShapesAndAncestors` analogue) records, for each child-type underlying,
  the list of encountered parent-type uses in first-seen order, de-duplicated per
  parent. Symmetry with the explorer (every edge the explorer yields under a face
  appears in that edge's ancestor list, and vice-versa) is a host invariant.
- **`BRep_Tool`-style accessors are free functions.** `pnt(Vertex)`,
  `tolerance(shape)`, `curve(Edge) -> (curve, first, last)`,
  `curve_on_surface(Edge, Face) -> pcurve`, `surface(Face)` read the underlying
  geometry and apply the resolved location, mirroring `BRep_Tool`. They are the only
  read path into attached geometry, keeping the underlying nodes opaque.
- **Complexity is isolated and flagged.** The one recursive enumeration/walk that
  backs enumeration, explorer, and ancestry is the sole routine that may approach
  the systems band; it is a single documented function with its `TopExp` reference,
  and everything else (value-like uses, accessors, orientation/location composition)
  stays well under 15.

## Verification

Each requirement is verifiable two ways (the roadmap's two gates):

- **(a) Host topological-invariant unit test** — `clang++ -std=c++20`, no OCCT, no
  simulator. Builds shapes directly through the model (e.g. a single triangular
  face from three vertices and three edges; a two-face shell sharing an edge) and
  asserts invariants: sub-shape counts per type, orientation round-trip
  (`reversed(reversed(s)) == s`; a Reversed use resolves to the flipped
  orientation), id stability across repeated enumeration and id sharing for a shared
  edge, explorer order equals the enumeration order, ancestry symmetry (shared edge
  lists both faces; each listed face's explorer yields that edge), and accessor
  values (a placed vertex reads back its world point; an edge reads back its curve +
  range; a face reads back its surface + outer/inner wires).
- **(b) Simulator native-vs-OCCT parity** — links the native headers and OCCT side
  by side, imports representative OCCT `TopoDS_Shape`s (a box, a cylinder, a shape
  with a placed/located sub-shape and a face with a hole) into the native model via
  a test-only importer, and asserts: identical sub-shape counts per type, identical
  enumeration order and ids (native id order matches `TopExp::MapShapes` visitation),
  identical resolved orientations, identical ancestry (native edge→faces equals
  `MapShapesAndAncestors`), and attached-geometry values (vertex points, edge curve
  samples + ranges, face surface samples) matching within the tight fp64 tolerance
  from `native-math`.

The host gate pins the structure to constructed ground truth so a convention drift
cannot pass; the parity gate pins conventions (orientation composition, enumeration
order, id sharing, ancestry ordering, outer/inner wire designation, pcurve-per-face)
to OCCT so the later engine wiring is a drop-in.

## Risks / Trade-offs

- **Convention mismatch with OCCT** (orientation composition through nesting,
  sub-shape enumeration order, ancestry ordering, shared-sub-shape id sharing,
  outer-vs-inner wire designation, pcurve-per-face storage). Mitigation: the parity
  gate imports real OCCT shapes and compares directly; the host gate independently
  pins structural invariants, so a mismatch surfaces in at least one gate.
- **Sharing / lifetime of the underlying nodes.** Mitigation: underlying entities
  are shared, reference-counted, and immutable after construction; uses are cheap
  value handles — no aliasing surprises, and id assignment keys on the underlying
  identity so sharing is exact.
- **Enumeration/walk complexity.** Mitigation: one documented recursive walk backs
  enumeration, explorer, and ancestry (single source of truth); it is flagged in
  the systems band and everything around it stays simple.
- **Geometry-attachment coupling to `native-math`.** Mitigation: topology only
  *references* `native-math` curve/surface descriptors and reads them through
  accessors; it neither re-implements evaluation nor duplicates geometry, so the
  math gate already covers the numerics and topology tests cover only structure +
  read-back.
- **Test-only importer scope creep.** Mitigation: the OCCT→native importer lives in
  the parity test only, is not part of the library, and does the minimum needed to
  feed comparison (it is not the eventual engine construction path).

## Migration Plan

1. Create `src/native/topology/` with the shape-kind enum, orientation, location,
   the underlying/use split, and a host CTest target building with
   `clang++ -std=c++20`, no OCCT (may include `src/native/math`). Add a grep gate
   for no-OCCT-include. (**host**)
2. Add orientation composition and location composition; host tests (orientation
   round-trip and compose table; located sub-shape resolves to the product
   placement). (**host**)
3. Add geometry attachment (vertex point + tol, edge curve + range + pcurve-on-face,
   face surface + outer/inner wires + tol) and the `BRep_Tool`-style accessors; host
   tests (read-back of a placed vertex/edge/face). (**host**)
4. Add the deterministic enumeration + stable id assignment (`TopExp::MapShapes`
   conventions, shared-sub-shape id sharing); host tests (counts, id stability, id
   sharing). (**host**)
5. Add the type-filtered explorer and the ancestry map; host tests (explorer order
   equals enumeration; ancestry symmetry with the explorer). (**host**)
6. Add the simulator native-vs-OCCT parity target (OCCT linked only there, test-only
   importer) covering counts, order, ids, orientation, ancestry, and attached
   geometry within the tight fp64 tolerance; keep every existing suite green.
   (**sim-parity**)
7. `openspec validate add-native-brep-topology --strict` green; reflect the
   `native-topology` in-progress status in `openspec/NATIVE-REWRITE.md` /
   `ROADMAP.md` Phase 4.

## Open Questions

- Whether to expose `Internal`/`External` orientation now (they exist in
  `TopAbs_Orientation` for embedded sub-shapes) or defer until a consumer needs
  them — default to reserving the enum values but exercising only `Forward`/
  `Reversed`, since construction/booleans introduce the internal cases.
- Whether ancestry should be built eagerly for all (child,parent) type pairs or lazily
  per query — default lazy per query (matching `MapShapesAndAncestors`' per-call
  build), with an option to cache if a consumer shows it is hot.
- The exact handle representation for attached `native-math` curves/surfaces (owning
  variant vs shared pointer to a descriptor) — resolve at implementation to keep uses
  cheap and geometry shared; parity/host tests do not depend on the choice.
- Whether pcurves are stored on the edge (per-face list, OCCT style) or on the
  face — default OCCT style (per-edge, keyed by face) so `curve_on_surface(Edge,
  Face)` maps directly to `BRep_Tool::CurveOnSurface`.
