# add-native-brep-topology

Phase 4 capability #2 (`native-topology`): an OCCT-free, host-buildable C++20 B-rep
**topology data model + traversal** under `src/native/topology/`, built on the
verified `native-math` foundation (`src/native/math/`: `Point3`/`Dir3`/`Transform`,
curve + surface eval). Provides the boundary-representation shape hierarchy
(`Vertex`/`Edge`/`Wire`/`Face`/`Shell`/`Solid`/`Compound`) with per-use
**orientation** (forward/reversed) and a `TopLoc`-style **location/placement**;
**geometry attachment** (vertex → point + tolerance; edge → 3D curve + parameter
range + optional pcurve-on-face; face → surface + outer/inner wires + tolerance);
**stable sub-shape identification** (deterministic integer ids, enumeration order
matching `TopExp::MapShapes` conventions); **traversal** (an explorer over a shape
by sub-type) and **ancestry** (sub-shape → parents, e.g. edge → adjacent faces);
and **`BRep_Tool`-style accessors**. Clean-room from first principles + the `cc_*`
contract + standard B-rep references, with OCCT (`TopoDS`, `TopAbs`, `TopExp`,
`TopTools`, `BRep_Tool`, `TopLoc_Location`) as a **convention oracle only** — for
orientation semantics, sub-shape ordering, and ancestry — never copied verbatim.
Every requirement is verified by (a) a host topological-invariant unit test
compiled with `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20` (no OCCT) AND (b) a
native-vs-OCCT parity test on the iOS simulator (import an OCCT shape into the
native model and compare structure, ids, orientation, and ancestry). This change
delivers the topology data model + traversal + its verification ONLY — no engine
wiring and no `cc_*` ABI change (engine wiring arrives with `native-construction`
and `native-booleans`).
