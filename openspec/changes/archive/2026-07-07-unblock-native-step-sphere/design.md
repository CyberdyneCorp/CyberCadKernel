# Design — unblock-native-step-sphere

Close the ONE remaining blocker on native STEP import of a full SPHERE: the reader already maps the
surface (`SPHERICAL_SURFACE` keyword AND on-axis-circle `SURFACE_OF_REVOLUTION` → native
`FaceSurface::Kind::Sphere`, host-verified), but an OCCT full sphere is a **single spherical FACE that
is both u-PERIODIC (the longitude seam) and DOUBLE-POLE-DEGENERATE (a whole parametric u-edge collapses
to a point at each of `v = ±π/2`)**, and the reader handles only the seam, not the poles — so the face
wire never closes and the watertight self-verify DECLINES → OCCT. This is a **reader / topology-only**
fix. Clean-room from ISO 10303-42/-43 + the existing reader/tessellator; OCCT
(`STEPControl_Reader` / `BRepPrimAPI_MakeSphere` / `STEPControl_Writer`) is the ORACLE + fixture-author
+ fallback only. NOT a general-surface reader.

## 0. What the reader + tessellator already do (the substrate this leans on)

Three EXACT facts, verified in the source, make this a topology-only fix:

- **The surface is already built.** `surface(#id)` maps `SPHERICAL_SURFACE` →
  `placedSurface(K::Sphere, 1)` (`step_reader.cpp` L920) and an on-axis-circle `SURFACE_OF_REVOLUTION`
  → `revolvedCircle` → `Kind::Sphere` (L1048–1065), both guarded by `circleOnSurface` /
  `placedSurface` faithful-reduction checks and asserted host-side
  (`test_native_step_reader.cpp` `surface_of_revolution_on_axis_circle_maps_to_sphere`). Nothing about
  the surface mapping changes.

- **The seam is already dropped.** `edgeLoop` (L1318) counts each `EDGE_CURVE`'s forward/reversed uses
  in the loop (`senseMask`) and drops the pair used BOTH ways (`senseMask == 3`, L1344–1345) — the
  writer/OCCT periodic-wall longitude seam. `chainLoop` (L1268) already tolerates all-closed loops.

- **The analytic sphere already meshes watertight over its NATURAL bounds.** `SurfaceEvaluator::bounds()`
  returns, for `Kind::Sphere`, `u ∈ [0, 2π], v ∈ [−π/2, π/2]` — the COMPLETE sphere including both
  poles (`surface_eval.h` L237). `FaceMesher::mesh` (`face_mesher.h` L209): when a face has **no usable
  outer boundary** (`!region.hasOuter()`, L235) it calls `structuredGrid(hasBoundary=false)` (L377),
  which samples the full `[0,2π]×[−π/2,π/2]` grid; the pole rows (`v = ±π/2`) all evaluate to the SAME
  pole point → the solid-mesher's vertex-merge fans them and drops the degenerate pole quads, and the
  `u = 0` / `u = 2π` columns evaluate to identical 3D points → they weld. This is EXACTLY how a
  native-constructed analytic sphere already tessellates watertight. And `ShapeBuilder::makeFace`
  ACCEPTS a NULL outer wire (`native_topology.h` L134: `if (!outer.isNull()) push_back`), i.e. a face
  can be a **bare periodic surface** with no trimming wire.

So the missing piece is purely: **recognise the degenerate pole edges** (so the reader stops building a
broken wire out of them) and, for a genuine FULL sphere, **emit the `Sphere` face as a bare periodic
surface** so it routes to the already-watertight natural-bounds mesh.

## 1. The OCCT full-sphere B-rep (ISO 10303-42) — what arrives

`BRepPrimAPI_MakeSphere(R)` → one `MANIFOLD_SOLID_BREP` → one `CLOSED_SHELL` → ONE `ADVANCED_FACE` over
a `SPHERICAL_SURFACE`. That face's `FACE_OUTER_BOUND` → `EDGE_LOOP` is the parametric-rectangle border:

```
v = −π/2 (south pole)  : DEGENERATE — the whole u-edge collapses to the south pole vertex
u = 0                  : the longitude SEAM meridian (a half-great-circle pole→pole)
v = +π/2 (north pole)  : DEGENERATE — the whole u-edge collapses to the north pole vertex
u = 2π                 : the SAME seam meridian, traversed REVERSED
```

- The two `u = 0 / 2π` meridians are ONE `EDGE_CURVE` referenced forward + reversed →
  `senseMask == 3` → **already dropped** by the landed seam logic.
- The two `v = ±π/2` pole edges are **DEGENERATE**: each is an `EDGE_CURVE` whose two endpoint
  `VERTEX_POINT`s coincide at the pole (a zero-length / on-axis edge — OCCT's "degenerated" edge, its
  3D sweep collapsed to a point). **These are the unhandled part.** (Grounding task 1 confirms the exact
  OCCT emission — a degenerate `EDGE_CURVE` with coincident vertices, vs a `VERTEX_LOOP` single-vertex
  bound; the design handles BOTH forms.)

Today, after the seam drop, the two pole edges survive as two `LoopEntry`s (each `v0 == v1`);
`chainLoop`'s `allClosed` branch keeps them; `buildFaceWithPCurves` tries to synthesise a pcurve for a
zero-length edge → a degenerate boundary loop → the assembled shell is not watertight →
`robustlyWatertightImport` DECLINES → OCCT. Honest, but deferred.

## 2. The topology fix (reader-only)

### 2a. Recognise + drop the degenerate pole edges (`edgeLoop`)

A **degenerate pole edge** is distinguished from a legitimate full-circle rim edge (both have
`v0 == v1`) by its 3D geometry: a pole edge has **zero arc length** — its `EDGE_CURVE`, evaluated over
its whole parameter range, stays within a scale-relative tolerance of a single point (equivalently:
both endpoint `VERTEX_POINT`s coincide AND the curve lies on / collapses to the revolution axis point).
A rim circle, by contrast, sweeps a real non-zero circle.

`edgeLoop` gains a `degeneratePoleEdge(ecId) → bool` test and drops such entries alongside the seam:

```cpp
std::vector<LoopEntry> entries;
for (const LoopEntry& e : raw)
  if (senseMask[e.ecId] != 3 &&        // not the periodic seam (landed)
      !degeneratePoleEdge(e.ecId))      // not a collapsed pole singularity (NEW)
    entries.push_back(e);
// entries may now be EMPTY (a full sphere: seam + both poles were all dropped).
```

The current `if (entries.empty()) { decline(); return {}; }` guard (L1346) is REPLACED by returning an
**empty edge list** (not a decline) so the caller can build a bare-surface face. A `VERTEX_LOOP`
face-bound (a single-vertex pole loop) is likewise mapped to an empty edge list rather than a decline.
A genuine full-circle rim (`v0 == v1`, non-zero sweep) is UNTOUCHED — the cylinder/cone cap-rim and
seam-wall paths are byte-unchanged.

### 2b. Emit the FULL sphere as a bare periodic surface (`advancedFace`)

`advancedFace` (L1383) resolves the surface then its bounds. When the surface is a native `Sphere` AND
every face-bound reduced to an empty edge list (the loop covered ONLY the seam + the degenerate poles →
a genuine FULL sphere, no surviving real trim), it builds the face with a **NULL outer wire**:

```cpp
if (srf->kind == K::Sphere && allBoundsEmpty(wires))     // full periodic double-pole sphere
  return topo::ShapeBuilder::makeFace(*srf, topo::Shape{}, {}, orient);   // bare surface
```

`makeFace` accepts the null outer (L134). At tessellation the face has no pcurve boundary →
`buildBoundaryLoops` is empty → `region.hasOuter()` is false → `structuredGrid(hasBoundary=false)`
meshes the natural `[0,2π]×[−π/2,π/2]` sphere → **watertight** (§0). If ANY real latitude-trim edge
survived the drop (a partial spherical zone), the existing wire path is kept unchanged — this fires
ONLY for the genuine full sphere.

### 2c. The gate — never a mis-fit or non-watertight sphere

The bare-surface `Sphere` face is emitted ONLY when BOTH hold:
1. **Faithful surface reduction** — the surface passed the existing `placedSurface(K::Sphere)` /
   `circleOnSurface` guard (centre, radius, plane-contains-axis all verified). Unchanged.
2. **Genuine full coverage** — every bound of the face reduced to an empty edge list (seam + poles
   only); no real trim edge survived. If a real edge survives, it is NOT a full sphere → keep the wire
   path.

Then the engine `robustlyWatertightImport` (per-member, volume > 0) is the FINAL arbiter — the SAME
self-verify every native import already runs. A sphere solid that does not mesh closed → DECLINE →
OCCT. **No tolerance is widened; no engine gate is added.**

## 3. Honest-out (the discipline the GOAL mandates)

The reader **keeps the current OCCT deferral** — returns NULL → OCCT, exactly as today — whenever the
pole-degenerate sphere face cannot be built robustly watertight:

- the surface does not reduce to `Sphere` (a torus / hyperboloid / general revolution — the landed
  declines, unchanged);
- a real latitude-trim edge survives the seam+pole drop (a **partial** spherical zone / pole-capped
  hemisphere) and the resulting face does not self-verify watertight — DECLINE (not forced onto the
  bare-surface path);
- the pole edges are not genuinely degenerate (a non-zero rim survives) — treated as a normal wire;
- the assembled sphere solid fails `robustlyWatertightImport`.

A correct still-deferred outcome is ACCEPTABLE and reported plainly (the sphere already imports
correctly via OCCT). A faked watertight claim, a non-watertight sphere, or a widened tolerance is NOT.

## 4. Architecture / OCCT boundary (unchanged)

```
cc_step_import (facade, unchanged)
   └─ active engine
        ├─ NativeEngine::step_import(path)
        │     ├─ step_import_native(path)  (src/native/exchange, OCCT-FREE) → topo::Shape
        │     │     • surface(): SPHERICAL_SURFACE | on-axis-circle SURFACE_OF_REVOLUTION → Sphere (landed)
        │     │     • edgeLoop(): drop the periodic SEAM (landed) + the DEGENERATE POLE edges (NEW)
        │     │     • advancedFace(): full sphere (seam+poles only) → BARE-surface Sphere face (NEW)
        │     │     • single / flat / placed / AP242 / trimmed-curve / partial-zone paths unchanged
        │     ├─ NULL → OCCT fall-through (honest-out: non-full / non-watertight sphere)
        │     └─ Solid/Compound → robustlyWatertightImport (per-member, volume>0) → wrap native
        └─ OCCT STEPControl_Reader (fallback + oracle + fixture author)
   cc_iges_* / cc_step_export → unchanged
```

`src/native/**` stays OCCT-free (grep-gated). The tessellator and `step_writer.cpp` are NOT modified —
the natural-bounds structured grid already meshes an analytic sphere watertight; the writer's own
full-sphere serialisation limitation is a separate, out-of-scope concern. No `cc_*` ABI change; default
engine stays OCCT (opt-in `cc_set_engine(1)`).

## 5. Why NOT the alternatives (rejected)

- **Teach the tessellator's structured grid to pole-collapse a bounded periodic face** — forbidden by
  the discipline ("Do NOT modify the tessellator unless genuinely required … sphere already tessellates
  elsewhere — prefer a reader/topology fix") and riskier (could regress every periodic/pole surface).
  Unnecessary: the natural-bounds path ALREADY meshes the sphere; the reader just has to route to it.
- **Re-tile the sphere into native multi-lune sub-patches** (as `build_revolution` does) — heavier,
  invents intermediate topology, and changes the imported face count for no watertightness benefit over
  the bare-surface route.
- **Modify the STEP writer** — out of scope and not needed for FOREIGN (OCCT) sphere import.

## 6. Cognitive complexity

`degeneratePoleEdge` is a small point-collapse test (≤ ~8). The `edgeLoop` drop gains one guard clause
(≤ +2 on an existing ≤ ~12 function). `advancedFace` gains one full-sphere branch (≤ +3). All measured
with the `cognitive-complexity` skill before archive; nothing pushed to a higher band (parser/systems
25–35).

## 7. Verification plan

- **Gate 1 (host, OCCT-free)** — `tests/native/test_native_step_reader.cpp`:
  - **Full sphere** — a hand-authored ISO-10303-21 buffer with a SINGLE `SPHERICAL_SURFACE`
    `ADVANCED_FACE` whose `EDGE_LOOP` is the seam meridian (forward+reversed) + two DEGENERATE pole
    edges → the reader returns a `Sphere` solid that is valid + watertight, volume = `4/3·π·R³`, bbox =
    `[−R,R]³`, IDENTICAL to the `SPHERICAL_SURFACE`-keyword multi-lune / on-axis-circle-revolution
    sphere. (Also assert the on-axis-circle `SURFACE_OF_REVOLUTION` form of the same face imports
    watertight, since both reduce to `Sphere`.)
  - **Honest-out** — a spherical face that still carries a real latitude-trim edge and cannot close, and
    a non-sphere revolution, DECLINE (NULL).
  - **No regression** — the parallel/oblique/perpendicular revolution reductions, the partial spherical
    zone, the single-solid, flat multi-solid, placed rigid / uniform-scale / mirror assembly, AP242,
    trimmed-curve, quadric, and bspline-face round-trips STILL pass.
- **Gate 2 (sim vs OCCT, OCCT linked)** — `tests/sim/native_step_import_parity.mm` +
  `scripts/run-sim-native-step-import.sh` via `cc_*` under `cc_set_engine(1)`;
  `xcrun simctl list devices booted` first:
  - **Sphere parity (native)** — `runRevolvedSphere` (OCCT `BRepPrimAPI_MakeSphere`, the exact
    single-periodic-pole-face B-rep) now imports **NATIVELY** (native raw `parsed = 1`, watertight);
    OCCT `STEPControl_Reader` re-imports; assert same solid **count / volume / area / watertight / bbox**
    within tolerance — flipping the currently-deferred case (native parsed=0 → OCCT) to native.
  - **Decline parity (unchanged)** — the torus / general-revolution fixtures still DECLINE natively and
    import via OCCT identical to `cc_set_engine(0)`.
  - Own `main()`, on the `run-sim-suite.sh` SKIP list; default engine restored in teardown so the suite
    assertion count is unchanged.
- **Done** only when the relevant gates are green and every existing suite stays green at the OCCT
  default (prior import slices host round-trip + sim `[NIMPORT]` 65/65 incl. revolution
  cylinder/cone/plane + sphere-reduction + honest torus/general declines, STEP export, healing,
  SSI S1–S5, native blends + #6/#7, curved-boolean native-pass=13, marching, boolean, construct,
  tessellation, phase3 do NOT regress). Honestly reported: this adds **native watertight import of a
  full periodic double-pole SPHERE face**; a non-full / pole-capped spherical zone that cannot close, a
  torus / hyperboloid / general revolution, and arbitrary surfaces stay OCCT; if the pole-degenerate
  face cannot be built robustly watertight the honest OCCT deferral is KEPT; #8 `drop-occt` stays
  blocked.
