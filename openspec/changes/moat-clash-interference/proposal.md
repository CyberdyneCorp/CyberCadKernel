# Proposal — moat-clash-interference (MOAT M-GS, GS7)

## Why

An assembly / mates workflow needs to answer a single, high-value question about
two placed solids: **do they interfere?** — and if so, *how much* (the overlap
volume) and *where* (a witness region). This is the interference/clash check every
mechanical CAD assembly tool exposes; today the only source of the answer in this
kernel is OCCT (`BRepAlgoAPI_Common` + `BRepGProp` for the overlap volume,
`BRepExtrema_DistShapeShape` for touching/clearance), which keeps the LGPL engine on
the critical path of a read-only analysis feature and blocks the native-only build.

This is a bounded **assembly-of-landed-parts** slice, not new geometry. Every
primitive the clash check needs is already landed and OCCT-verified in this worktree:

- `src/native/boolean/freeform_membership.h` (B3) — the multi-ray parity
  `classifyPointInMesh` point-in-solid classifier with an honest `Unknown` decline,
  and `minDistanceToMesh` / `pointTriangleDistance` (exact point-to-triangle
  distance), all verified vs OCCT on the simulator;
- `src/native/boolean/native_boolean.h` — `boolean_solid(A, B, Op::Common)`, the
  native BSP-CSG intersection, consumed READ-ONLY for the overlap VOLUME;
- `src/native/tessellate/mesh.h` — `isWatertight` / `enclosedVolume`, the M0
  mesh-property vocabulary;
- the engine's landed `watertightVolume` self-verify helper (native_engine.cpp),
  the same watertight+volume guard `boolean_op` already uses.

GS7 therefore does NOT touch the tessellator, the boolean layer (it CONSUMES
`boolean_solid` COMMON read-only), or blend/exchange. Its new code lives in a NEW
header `src/native/analysis/interference.h` (namespace `cybercad::native::analysis`,
mesh-math only, OCCT-FREE), the engine dispatch, and the facade.

The hard cases are an **HONEST DECLINE**, never a wrong clash flag or overlap volume:
a non-watertight operand mesh; a boundary point the B3 ray-parity classifier cannot
decide (grazing / disagreeing rays) that lies in the enclosable interior region; a
CLASH whose overlap volume the native COMMON cannot robustly produce (curved /
near-tangent operands); or a mesh-soup (imported STL) operand that has no B-rep to
intersect for the volume. A measured decline is a first-class outcome — the native
engine returns a clean error and the facade falls through to the OCCT oracle; a
wrong overlap is NEVER emitted, and no tolerance is weakened.

## What Changes

1. **A new header-only, OCCT-free module `src/native/analysis/interference.h`** in
   namespace `cybercad::native::analysis`, consuming the B3 membership classifier +
   the M0 mesh vocabulary read-only:
   - `meshInterference(meshA, meshB, deflection)` classifies the pair as **CLASH**
     (interiors overlap over a set of positive volume), **TOUCHING** (boundary
     contact, no interior overlap), **CLEAR** (positive clearance gap), or
     **UNKNOWN** (honest decline).
   - CLASH is detected **coplanar-safely** via the B3 classifier: a boundary VERTEX
     or a boundary TRIANGLE CENTROID of one solid that classifies strictly INSIDE the
     other. A shared face reads `On` (the B3 ON-band absorbs the seam), never `In`, so
     a flush TOUCH never fires as a clash; a face poking through the other's wall has
     an interior centroid → CLASH. (A raw Möller tri–tri crossing was evaluated and
     rejected — it over-reports at a shared seam where edge-adjacent triangles of two
     flush solids "cross" though nothing interpenetrates.)
   - When no penetration signature fires, the minimum triangle–triangle distance
     (via the landed `pointTriangleDistance`, AABB-pruned) decides TOUCHING (within
     the mesh-fidelity contact band) vs CLEAR (the min distance is the clearance).
   - An `Unknown` ray verdict only VETOES (→ decline) when the point lies strictly
     inside the target's AABB AND beyond the contact band — the only region a masked
     interior overlap could occupy. A spurious coplanar-grazing `Unknown` on a far
     face, or an `Unknown` at a contact seam, does not force a decline.
   - The overlap **VOLUME** is NOT computed in the header (it stays boolean-free /
     OCCT-free); the ENGINE fills it from the native COMMON with a two-sided
     self-verify.

2. **Engine dispatch** (`IEngine::interference`, `NativeEngine::interference`,
   `OcctEngine::interference`):
   - NativeEngine meshes both native bodies, runs `meshInterference`, and on CLASH
     computes the overlap volume as `watertightVolume(boolean_solid(A,B,Common))`
     with a **TWO-SIDED self-verify**: the COMMON must be watertight AND its volume
     must not exceed `min(V(A), V(B))` (an overlap cannot be larger than either
     operand). A null / non-watertight / out-of-band COMMON DECLINES to OCCT. The
     witness on CLASH is sharpened to the COMMON solid's AABB + its signed-tetra
     centroid (a guaranteed interior point). Mixed native/OCCT operands are rejected;
     an all-OCCT pair forwards to the oracle.
   - OcctEngine is the ORACLE: overlap volume via `BRepAlgoAPI_Common` + `BRepGProp`,
     clearance via `BRepExtrema_DistShapeShape`, witness = the COMMON bbox + centroid.

3. **One additive facade op `cc_interference(a, b, CCInterference* out)`** (+ the
   `CCInterference` POD and `CCClashState` enum) in
   `include/cybercadkernel/cc_kernel.h` / `src/facade/cc_kernel.cpp`, signature-styled
   like the landed `cc_check_solid(body, CCValidityReport*)`. Returns 1 for a definite
   verdict, 0 on an honest decline (`out->decided == 0`, `cc_last_error` set) or an
   unknown body (out zeroed). ADDITIVE-ONLY: no existing `cc_*` signature changes;
   `src/native/**` stays OCCT-FREE (zero OCCT includes).

4. **Two verification gates** (the non-negotiable discipline):
   - **GATE A — HOST ANALYTIC (no OCCT):** two overlapping axis-aligned boxes →
     CLASH with overlap volume = the exact intersection-box volume + a witness AABB
     that contains the true overlap box; disjoint boxes → CLEAR with min distance =
     the exact gap; face-touching boxes → TOUCHING with overlap volume 0; a nested box
     → CLASH; a non-watertight operand → the UNKNOWN honest decline
     (`tests/native/test_native_interference.cpp`).
   - **GATE B — SIM native-vs-OCCT:** on a booted iOS simulator, match the OCCT
     oracle — overlap volume vs `BRepAlgoAPI_Common` + `BRepGProp`, the CLASH /
     TOUCHING / CLEAR state + clearance vs `BRepExtrema_DistShapeShape` — on identical
     geometry at fixed (never-widened) tolerances
     (`tests/sim/native_interference_parity.mm`).

Out of scope (honest declines, tracked): a CLASH whose overlap volume needs a
curved / near-tangent COMMON the native boolean cannot robustly build; a freeform
operand overlap; a mesh-soup (STL) operand's overlap volume. These DECLINE at the
service boundary and stay on OCCT.
