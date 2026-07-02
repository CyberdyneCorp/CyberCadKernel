## Why

Phase 4 drops OCCT one capability at a time until it can be unlinked entirely
(`openspec/NATIVE-REWRITE.md`). Capability #1 (`native-math`) delivered the
OCCT-free fp64 math + geometry-evaluation foundation. The next thing every higher
capability needs — construction (swept solids), booleans, blends, tessellation,
data exchange — is a **B-rep topology model**: the boundary-representation shape
hierarchy that binds evaluated geometry (points, curves, surfaces) into
oriented, connected vertices, edges, wires, faces, shells, and solids, with the
adjacency and traversal those algorithms walk. This is capability `#2`
(`native-topology`) in the locked dependency order — it builds directly on
`native-math` and is depended on by everything after it.

Making the topology layer OCCT-free is what unlocks the two-gate verification
model: because it links no OCCT and needs no simulator, it can be unit-tested on
the host with `clang++ -std=c++20` against topological invariants (Euler-style
counts, orientation round-trips, ancestry symmetry), and it can *also* be
compared against the OCCT oracle on the simulator by importing an OCCT shape into
the native model and asserting identical structure, ids, orientation, and
ancestry. Two independent gates over the same code give high confidence in the
data model and its conventions before anything is wired into the engine.

This change delivers the data model + traversal + its verification ONLY. It does
NOT touch the `cc_*` ABI and does NOT wire native code into the active engine —
that begins with `add-native-swept-solids` (construction) and `add-native-booleans`,
which produce and consume these topology types behind the facade. The app keeps
running unchanged on OCCT throughout.

## What Changes

- Add a new **OCCT-free C++20 topology library** under `src/native/topology/` (no
  OCCT include anywhere; it may include `src/native/math`; compiles with
  `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20` with no OCCT), implemented
  **clean-room** from first principles, the `cc_*` contract, and standard B-rep
  references. OCCT source (`TopoDS`, `TopAbs`, `TopExp`, `TopTools`, `BRep_Tool`,
  `TopLoc_Location`) is consulted only as a **convention oracle** (orientation
  semantics, sub-shape ordering, ancestry), never copied.
- **B-rep data model**: the shape kinds `Vertex`, `Edge`, `Wire`, `Face`, `Shell`,
  `Solid`, `Compound` (mirroring `TopAbs_ShapeEnum`), each shape being a shared,
  immutable *underlying* entity (TShape-style) referenced through an oriented,
  located *use* (mirroring `TopoDS_Shape`): a `(underlying, orientation, location)`
  triple. **Orientation** is `Forward` / `Reversed` (with `Internal` / `External`
  reserved to match `TopAbs_Orientation`) and composes correctly through nesting.
  **Location/placement** is a `TopLoc`-style rigid placement (a `Transform` from
  `native-math`) that composes down the hierarchy so a sub-shape's effective
  placement is the product of its ancestors'.
- **Geometry attachment**: a `Vertex` carries a `Point3` + a tolerance; an `Edge`
  carries a 3D curve handle + a parameter range `[first, last]` + optional
  **pcurve(s)** (2D parameter-space curve on a specific `Face`); a `Face` carries a
  surface handle + an ordered set of boundary `Wire`s (one outer, zero or more
  inner/hole) + a tolerance. Geometry is referenced through the `native-math`
  evaluators, never duplicated.
- **Sub-shape identification**: every distinct underlying sub-shape gets a **stable
  integer id** under a deterministic enumeration whose order matches
  `TopExp::MapShapes` conventions (shape visited before its sub-shapes, fixed
  per-type child order, no duplicate ids for shared sub-shapes). Ids are stable for
  a given shape so tests and parity comparisons can key on them.
- **Traversal**: an **explorer** that iterates the sub-shapes of a shape filtered
  by type (e.g. all `Edge`s of a `Solid`), yielding each *use* with its resolved
  orientation, in the deterministic enumeration order — the `TopExp_Explorer`
  analogue.
- **Ancestry**: a sub-shape → list-of-parents map (e.g. `Edge` → adjacent `Face`s,
  `Vertex` → incident `Edge`s), built once over a shape — the
  `TopExp::MapShapesAndAncestors` analogue — with a documented ordering.
- **`BRep_Tool`-style accessors**: free functions to read the geometry off a shape
  — `pnt(Vertex)`, `tolerance(shape)`, `curve(Edge) → (curve, first, last)`,
  `curve_on_surface(Edge, Face) → pcurve`, `surface(Face)` — resolving the
  underlying entity through its location, the `BRep_Tool` analogue.
- **Verification harness**: for every requirement, (a) a host topological-invariant
  unit test (built shape has the expected vertex/edge/face counts, orientation
  round-trips, ancestry is symmetric with the explorer, ids are stable across
  repeated enumeration) built with `clang++ -std=c++20` and no OCCT, AND (b) a
  native-vs-OCCT parity test on the iOS simulator that imports an OCCT `TopoDS_Shape`
  into the native model and asserts identical sub-shape counts, enumeration order,
  ids, orientations, ancestry, and attached-geometry values within tolerance.

No public C ABI change. No engine wiring. Determinism: fixed enumeration order,
stable ids, reproducible run to run.

## Capabilities

### New Capabilities
- `native-topology`: OCCT-free, host-buildable C++20 B-rep topology model +
  traversal — the shape hierarchy (`Vertex`/`Edge`/`Wire`/`Face`/`Shell`/`Solid`/
  `Compound`) with forward/reversed orientation and a `TopLoc`-style
  location/placement; geometry attachment (vertex → point + tol, edge → curve +
  range + optional pcurve-on-face, face → surface + outer/inner wires + tol);
  stable integer sub-shape ids under a deterministic `TopExp::MapShapes`-matching
  enumeration; a type-filtered explorer and a sub-shape → parents ancestry map; and
  `BRep_Tool`-style geometry accessors. Clean-room from first principles + the
  `cc_*` contract + standard B-rep references, with OCCT (`TopoDS`/`TopAbs`/
  `TopExp`/`TopTools`/`BRep_Tool`/`TopLoc_Location`) as a convention oracle only.
  Verified by host topological-invariant unit tests AND native-vs-OCCT structural
  parity on the simulator (import an OCCT shape, compare). Depends on `native-math`
  (value types + geometry evaluators); it is the foundation `native-construction`,
  `native-booleans`, `native-tessellation`, and `native-blends` build on.

### Modified Capabilities
<!-- none — native-topology is purely additive: a new OCCT-free library plus its
     tests. No cc_* signature or POD struct changes, and it is NOT wired into the
     active engine in this change (engine wiring begins with native-construction /
     native-booleans). -->

## Impact

- **Contract / ABI**: none. No `cc_*` signature or POD struct layout changes; the
  library is not yet reachable through the facade.
- **Engine**: none. `NativeEngine` does not consume this yet; the OCCT engine
  remains the sole active implementation. Wiring begins in `native-construction` /
  `native-booleans`, which produce/consume these topology types.
- **Build**: new OCCT-free `src/native/topology/` sources (may include
  `src/native/math`); a host CTest target (`clang++ -std=c++20`, no OCCT, no
  simulator) and a simulator native-vs-OCCT parity target (OCCT linked only in the
  parity test, never in the library, to import a shape and compare).
- **Determinism / precision**: fixed enumeration order, stable integer ids, fp64
  geometry from `native-math`; reproducible run to run and matching the OCCT
  `TopExp` conventions.
- **Risk**: convention mismatch with OCCT (orientation composition through nesting,
  sub-shape enumeration order, ancestry ordering, shared-sub-shape id sharing,
  pcurve-per-face storage, outer-vs-inner wire designation). Bounded by the
  native-vs-OCCT parity gate; the host invariant tests independently pin the model
  to structural ground truth so a convention mismatch cannot pass silently in one
  gate alone.
