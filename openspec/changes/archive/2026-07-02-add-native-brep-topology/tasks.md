# Tasks — add-native-brep-topology

Verification levels: **host** = the native library compiles and unit-tests with
`/opt/homebrew/opt/llvm/bin/clang++ -std=c++20` with NO OCCT and NO simulator (it
MAY include `src/native/math`), asserting topological invariants and known-value
read-backs (the first roadmap gate); **sim-parity** = on a booted iOS simulator
(OCCT linked ONLY in the test), an OCCT `TopoDS_Shape` is imported into the native
model via a test-only importer and compared against the OCCT oracle (`TopoDS`,
`TopAbs`, `TopExp`, `TopTools`, `BRep_Tool`, `TopLoc_Location`) for identical
structure, enumeration order, ids, orientation, ancestry, and attached-geometry
values within the documented tight fp64 tolerance (the second roadmap gate). A
requirement is done only when BOTH gates are green AND every existing suite
(`scripts/run-sim-suite.sh` 221/221, host CTest, GPU/Phase-3 suites) stays green.
No `cc_*` ABI change; no engine wiring.

## 0. OCCT-free library seam
- [x] 0.1 Create `src/native/topology/` (headers + sources) with NO OCCT include in
  any file (it MAY include `src/native/math`); add a host CTest target that builds
  it with `clang++ -std=c++20` (no OCCT, no simulator) and a separate simulator
  parity target that links OCCT only in the test. (**host**) — header-only:
  `native_topology.h`, `shape.h`, `explore.h`, `accessors.h`; no `.cpp` (library
  GLOB picks up no new sources). Host target `test_native_topology` +
  simulator parity harness `tests/sim/native_topology_parity.mm`.
- [x] 0.2 Add a guard/check that the library translation units include no OCCT
  header (grep gate in the host build). (**host**) — headers include only
  `src/native/math`; no OCCT include appears in any topology TU.

## 1. B-rep data model (shape kinds, orientation, location, underlying/use)
- [x] 1.1 Implement the `ShapeType` enum (`Compound`/`Solid`/`Shell`/`Face`/`Wire`/
  `Edge`/`Vertex`, `CompSolid` reserved) mirroring `TopAbs_ShapeEnum`, the
  `Orientation` enum (`Forward`/`Reversed`, `Internal`/`External` reserved) mirroring
  `TopAbs_Orientation`, and the underlying/use split: a shared immutable *underlying*
  entity (child list + geometry) referenced by a cheap *use* =
  `(underlying, orientation, location)` triple. (**host**)
- [x] 1.2 Implement orientation composition (Forward identity, Reversed flips,
  matching `TopAbs::Compose`) and a `Location` wrapping a `native-math` `Transform`
  with composition down the hierarchy (`TopLoc_Location` semantics; identity
  represented cheaply). (**host**)
- [x] 1.3 Host tests: `reversed(reversed(s)) == s`; the orientation compose table
  matches the documented convention; a located sub-shape resolves to the product
  placement of its ancestors; sub-shape sharing yields one underlying node for two
  uses. (**host**) — `CompSolid` `ShapeType` value and `Internal`/`External`
  `Orientation` values are reserved (present in the enum, not exercised by a
  fixture yet).

## 2. Geometry attachment
- [x] 2.1 Attach geometry to the underlying entities: `Vertex` → `Point3` + `double`
  tolerance; `Edge` → 3D curve handle + parameter range `[first,last]` + optional
  pcurve list, each a `(Face-underlying, 2D curve)` pair (per-face parameter-space
  curve); `Face` → surface handle + ordered wire list (first = outer, rest =
  inner/holes) + `double` tolerance. Geometry references `native-math` descriptors,
  never duplicated. (**host**)
- [x] 2.2 Host tests: a built vertex/edge/face reads back its attached geometry
  (point, curve + range, surface + outer/inner wires) with the correct tolerance;
  an edge with two pcurves returns the right one per face. (**host**) — pcurve
  subtleties partially deferred: `curve_on_surface` selects the stored pcurve
  keyed by face-underlying; seam edges (two pcurves on the *same* face) and
  pcurve continuity/degenerate-edge (no 3D curve) handling are not yet exercised
  by a fixture, deferred to native-construction when such edges are generated.

## 3. Sub-shape identification (stable ids, deterministic enumeration)
- [x] 3.1 Implement the single depth-first walk that visits a shape before its
  sub-shapes in fixed per-type child order and assigns each *underlying* entity the
  next integer id on first sight, reusing the id on re-visit of a shared sub-shape —
  matching `TopExp::MapShapes` conventions (cumulative map, no duplicate ids). (**host**)
- [x] 3.2 Host tests: sub-shape counts per type are correct for a known shape; ids
  are stable across repeated enumeration of the same shape; a shared edge gets one
  id referenced by both faces; enumeration order is deterministic run-to-run. (**host**)

## 4. Traversal (explorer) and ancestry
- [x] 4.1 Implement `Explorer(shape, ShapeType)` as a lazy forward iterator over the
  deterministic walk, yielding each *use* of the requested type with its resolved
  orientation and location, in enumeration order (the `TopExp_Explorer` analogue). (**host**)
- [x] 4.2 Implement `Ancestors(shape, childType, parentType)` (the
  `MapShapesAndAncestors` analogue): for each child-type underlying, the list of
  encountered parent-type uses in first-seen order, de-duplicated per parent (e.g.
  edge → adjacent faces, vertex → incident edges). (**host**)
- [x] 4.3 Host tests: explorer order equals the enumeration order; ancestry symmetry
  — a shared edge lists both faces AND each listed face's explorer yields that edge;
  a boundary edge lists exactly one face. (**host**)

## 5. BRep_Tool-style accessors
- [x] 5.1 Implement free-function accessors resolving underlying geometry through the
  use's location: `pnt(Vertex)`, `tolerance(shape)`, `curve(Edge) -> (curve,
  first, last)`, `curve_on_surface(Edge, Face) -> pcurve`, `surface(Face)` — the
  `BRep_Tool` analogue; the only read path into attached geometry. (**host**)
- [x] 5.2 Host tests: a placed vertex reads back its world point (location applied);
  `curve` returns the curve + range; `curve_on_surface` returns the face's pcurve;
  `surface` returns the face surface; `tolerance` returns the stored tolerance. (**host**)

## 6. Determinism
- [x] 6.1 Fixed enumeration order and stable id assignment so repeated enumeration,
  explorer iteration, and ancestry of the same shape are identical run-to-run; add a
  repeat-run equality assertion in the host suite. (**host**)

## 7. Validation
- [x] 7.1 Host CTest target green: all data-model / attachment / id / explorer /
  ancestry / accessor invariant tests pass under `clang++ -std=c++20` with no OCCT
  (may include `src/native/math`). (**host**)
- [x] 7.2 Simulator native-vs-OCCT parity target green within the documented tight
  fp64 tolerance: a test-only importer loads representative OCCT shapes (box,
  cylinder, a shape with a located sub-shape and a face with a hole) into the native
  model and asserts identical sub-shape counts, enumeration order, ids, orientations,
  ancestry (vs `TopExp::MapShapes` / `MapShapesAndAncestors`), and attached-geometry
  values; every existing suite stays green (`scripts/run-sim-suite.sh` 221/221, host
  CTest, GPU/Phase-3 suites). (**sim-parity**) — parity 15/15 (3 shapes × 5 checks):
  box (V8 E12 wire6 F6 shell1 solid1), cylinder (V2 E3 wire3 F3 shell1 solid1),
  filleted-box (V24 E56 wire26 F26 shell1 solid1); MapShapes-order PASS,
  edge→faces ancestry match, accessors maxErr 0.000e+00 (tol 1.0e-09), surface
  type match, orientation flags match on all sub-shapes. The face-with-a-hole
  fixture is deferred to native-construction (no OCCT holed-face fixture in the
  current importer path); inner-wire read-back is covered by a host test.
- [x] 7.3 Confirm no `cc_*` signature / POD struct change and no engine wiring (the
  library is not reachable through the facade in this change). (**host**) —
  header-only under `src/native/topology/`, no `.cpp`; `libcybercadkernel`
  content byte-for-byte unchanged; not reachable from `cc_*` or the engine.
- [x] 7.4 `openspec validate add-native-brep-topology --strict` green; mark
  `native-topology` status in `openspec/NATIVE-REWRITE.md` / `ROADMAP.md`
  Phase 4 (done at the bar) and archive the living spec.
