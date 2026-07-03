# Proposal — add-native-booleans

## Why

Phase 4 replaces OCCT capability by capability behind the unchanged `cc_*` facade
(`openspec/NATIVE-REWRITE.md`). Capabilities #1–#4 landed the native math, B-rep
topology, tessellation, and construction (extrude / revolve / loft / sweep / shank)
foundations — all OCCT-free and host-buildable, with `NativeEngine` serving native bodies
and falling through to OCCT for the rest. Capability **#5 `native-booleans`** is the
sequenced next step and the **research-grade** one: `cc_boolean` (fuse / cut / common) is
the hardest and longest-lived OCCT dependency, because a robust B-rep boolean must reason
about the intersection of two arbitrary existing solids with fp64 robustness at
near-tangent / coincident configurations (the classic BOPAlgo wall), not just build clean
topology from parameters.

A general robust boolean over arbitrary NURBS solids is out of reach in one pass. Doing it
all at once would risk faking coverage on the genuinely hard surface–surface intersection
and coincident-face robustness. This proposal scopes an **analytic PLANAR-FACED** slice —
`cc_boolean` native for polyhedra (boxes, prisms, convex or simple-concave solids) for all
three ops — verified EXACTLY against the OCCT oracle on axis-aligned box cases, and guarded
by a **mandatory self-verify** that discards any bad native result and falls through to
OCCT. Curved-face solids, near-tangent / coincident / degenerate configurations, and any
non-native operand remain honest, labelled OCCT fall-through. This is explicitly reported
as the analytic-planar-first slice of the research-grade boolean; general robustness is
future work.

## What changes

1. **Native boolean subtree** (`src/native/boolean/`, OCCT-free, host-buildable). A new
   subtree with the planar-boolean pipeline, returning a `topology::Shape` (NULL ⇒ fall
   through). It includes only `src/native/math`, `src/native/topology`, and
   `src/native/tessellate` (for the self-verify), never OCCT. The pipeline stages:
   - **Face–face intersection segments.** For each planar face of `A` against each planar
     face of `B`, compute the intersection segment of the two supporting planes clipped to
     both face polygons (a plane–plane line intersected with each face's 2D boundary). Collect
     the section segments per face.
   - **Face splitting.** Split each face along the section segments that cross it, producing
     face fragments (2D polygon subdivision in each face's plane, using the native math /
     `uv_triangulate` polygon machinery).
   - **Fragment classification.** Classify each fragment's interior as INSIDE / OUTSIDE / ON
     the other solid, by point-in-polyhedron testing (ray-parity / signed-solid-angle) at the
     fragment centroid against the other operand's shell.
   - **Surviving-shell assembly.** Select the fragments that survive for the op — fuse keeps
     `A`-outside-`B` + `B`-outside-`A`; cut keeps `A`-outside-`B` + `B`-inside-`A` (reversed);
     common keeps `A`-inside-`B` + `B`-inside-`A` — and orient them consistently.
   - **Sew / heal.** Sew the surviving fragments into a shell (weld coincident vertices/edges),
     close it, and wrap as a `Solid`.
2. **Point-in-polyhedron + plane/segment predicates** (`src/native/boolean/` or a small
   `src/native/math` addition, OCCT-free). Robust-enough fp64 predicates: point-in-planar-face
   (2D), point-inside-closed-planar-shell (ray parity / winding), plane–plane line, and
   segment–polygon clip. Near-tolerance ambiguity (coincident / tangent) ⇒ the builder DECLINES.
3. **Mandatory self-verify guard** (`src/engine/native/native_engine.cpp`, reusing the
   existing `robustlyWatertight` + `enclosedVolume` from `src/native/tessellate`). After the
   native builder returns a candidate solid, the engine verifies it is (a) a closed watertight
   2-manifold across a deflection ladder and (b) has the correct **set-algebra volume sign and
   magnitude** for the op (checked against the operands' own native volumes). If EITHER check
   fails, the candidate is **DISCARDED** and the call falls through to OCCT. This mirrors the
   Tier-D thread `robustlyWatertight` self-verify, extended with the set-algebra volume check.
4. **`NativeEngine` glue** (`src/engine/native/native_engine.{h,cpp}`). `boolean_op` — currently
   `CC_NATIVE_BODY_UNSUPPORTED(a) / (b)` + `fallback().boolean_op(...)` — becomes
   native-else-fallback: when BOTH operands are native bodies, it runs the native planar builder
   and the self-verify guard; a foreign operand, a NULL native result (an unsupported / declined
   case), or a failed self-verify falls through to the fallback with no interception. OCCT stays
   behind `CYBERCAD_HAS_OCCT`; the native builder never sees OCCT.

## Non-goals (DEFERRED — fall through to OCCT, not implemented, not faked)

- **Curved-face solids** — any cylinder / sphere / cone / NURBS face on either operand needs
  surface–surface intersection curves and curved section-edge classification (out of scope for
  the planar slice). Labelled, verified OCCT fall-through.
- **Near-tangent / coincident / degenerate configurations** — coplanar overlapping faces,
  touching-only (measure-zero) contact, shared boundaries, slivers. The builder DECLINES rather
  than emit a wrong classification; OCCT fall-through.
- **Non-native / foreign operands** — a native boolean requires the native B-rep of BOTH
  operands; a foreign (OCCT-built) body falls through unchanged.
- **General robust B-rep boolean** — full sew/heal, tolerance reconciliation, and robust
  coincident/tangent handling over arbitrary solids remain future work (the rest of the
  research-grade capability).
- Every feature / query / transform / exchange op and the already-native construction ops —
  unchanged.

## Impact

- New `src/native/boolean/` subtree (planar-boolean pipeline: intersection / split / classify /
  assemble / sew) + any small `src/native/math` predicate additions — all OCCT-free,
  host-buildable, added to a `native_boolean.h` umbrella. New host CTest cases in
  `tests/test_native_boolean.cpp` (+ facade cases in `tests/test_native_engine.cpp`).
- `src/engine/native/native_engine.cpp` — `boolean_op` changes from pure fall-through to
  "native-else-(self-verify)-else-fallback"; the self-verify guard extends `robustlyWatertight`
  with a set-algebra volume check. `native_engine.h` unchanged (`boolean_op` signature already
  present).
- **No** `include/cybercadkernel/cc_kernel.h` signature change; **no** `src/facade/cc_kernel.cpp`
  change (the `cc_boolean` entry point already routes through the active engine). The
  `cc_boolean` doc-comment (op: 0 fuse, 1 cut a−b, 2 common) is the contract this change
  implements natively for planar-faced solids.
- Behaviour unchanged by default (engine stays OCCT); only callers that call `cc_set_engine(1)`
  see the new native path. All existing suites stay green at the OCCT default.

## Verification

Two independent gates from `NATIVE-REWRITE.md`: (a) **host** exact-value / analytic unit tests
on the built native boolean B-rep + native tessellation — axis-aligned box fuse / cut / common
are watertight closed 2-manifolds with the EXACT set-algebra volume; a prism / simple-concave
case is watertight with the exact volume; a mis-assembled shell is rejected by the self-verify;
and a curved-face operand / coincident configuration / foreign body cause fall-through — all with
no OCCT; (b) **sim parity** through the facade (`cc_set_engine(1)` vs default) comparing native vs
OCCT `BRepAlgoAPI_Fuse`/`_Cut`/`_Common` for axis-aligned box booleans (EXACT) and asserting the
fall-through cases (curved, coincident, foreign) identical under both engines. Done only when both
gates pass and every existing suite stays green at the OCCT default. Reported honestly as the
analytic-planar-first slice; general robustness is future work.
