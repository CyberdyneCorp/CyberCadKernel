# Design — add-native-booleans (Phase 4 #5, research-grade)

Clean-room native `cc_boolean` (fuse / cut / common) for **PLANAR-faced solids**, on the
#1–#3 foundations (`src/native/math`, `src/native/topology`, `src/native/tessellate`).
OCCT-free, host-buildable. OCCT (`BRepAlgoAPI_Fuse`/`_Cut`/`_Common`, `BOPAlgo_Builder` /
`BOPAlgo_PaveFiller`, `BOPTools`) is a **reference ORACLE only** — consulted to confirm the
four-stage pipeline (intersect → split → classify → build) and the fuse/cut/common
face-survival rules, and to compare numerics; nothing copied.

> **HONEST SCOPE.** A general robust B-rep boolean over arbitrary NURBS solids is
> research-grade and out of reach in one pass. This change lands the **analytic-planar-first**
> slice: `cc_boolean` native for polyhedra (boxes, prisms, convex or simple-concave), verified
> EXACTLY against the OCCT oracle on axis-aligned box cases, and DISCARDED by a mandatory
> self-verify (falling through to OCCT) whenever the native result is not a valid watertight
> solid with the correct set-algebra volume. Curved-face solids, near-tangent / coincident /
> degenerate configurations, and non-native operands fall through to OCCT (labelled, verified,
> never faked). It is fully acceptable — and reported truthfully — if only axis-aligned /
> planar-polyhedron booleans land native and everything else is OCCT-fallback.

## 1. The contract (`cc_boolean`)

```c
/* Booleans: op = 0 fuse, 1 cut (a-b), 2 common. */
CCShapeId cc_boolean(CCShapeId a, CCShapeId b, int op);
```

- `a`, `b` — two solid body ids.
- `op` — `0` fuse (`A ∪ B`), `1` cut (`A − B`), `2` common (`A ∩ B`).
- Result — a new watertight `Solid` body (the `cc_*` mirror of OCCT
  `BRepAlgoAPI_Fuse`/`_Cut`/`_Common`).

## 2. Case split (what is native vs fall-through)

```
cc_boolean(a, b, op)
   │
   ├─ a AND b are native bodies, both PLANAR-faced (every FaceSurface is a Plane),
   │  no near-tangent/coincident configuration detected
   │      → planar-boolean pipeline (§3) → candidate Solid
   │            │
   │            ├─ candidate PASSES self-verify (§4): closed watertight 2-manifold
   │            │  AND correct set-algebra volume sign+magnitude for op
   │            │      → NATIVE result
   │            │
   │            └─ candidate FAILS self-verify (open / non-manifold / wrong volume)
   │                   → DISCARD → OCCT FALL-THROUGH (labelled)
   │
   ├─ a OR b is NOT a native body (foreign / OCCT-built)
   │      → OCCT FALL-THROUGH (labelled)
   │
   └─ a curved face on either operand, OR a near-tangent / coincident /
      degenerate configuration
          → builder DECLINES (returns NULL Shape) → OCCT FALL-THROUGH (labelled)
```

The self-verify guard is **mandatory** and applies even to a nominally-planar case: the
engine ships a native boolean ONLY when it is verified valid. No unverified or wrong solid
is ever emitted.

## 3. The planar-boolean pipeline (`src/native/boolean/`)

Four stages, mirroring BOPAlgo's structure (intersect → split → classify → build), each a
small OCCT-free step over the native `topology::Shape` model.

### 3.1 Preflight — planar + non-degenerate check

Walk both solids' faces (via `topology::explore`). If ANY face's `FaceSurface::kind` is not
`Plane`, the builder DECLINES (NULL) — curved-face booleans are out of scope. Reject
degenerate operands (empty shell, zero volume). Detect obvious near-coincidence early
(bounding-boxes that only touch, coplanar overlapping faces within tolerance) and DECLINE —
the classic robustness wall is left to OCCT.

### 3.2 Face–face intersection segments

For each planar face `fA` of `A` and each planar face `fB` of `B` whose bounding boxes
overlap:

- Intersect the two supporting planes → an infinite line `L` (skip parallel/coincident
  planes; a coincident pair within tolerance ⇒ DECLINE).
- Clip `L` to `fA`'s 2D polygon boundary → a parameter interval `IA`; clip to `fB`'s
  boundary → `IB`. The section segment on this face pair is `L(IA ∩ IB)`.
- Record the segment against BOTH `fA` and `fB` (each face accumulates the set of section
  segments that cross it).

Native primitives used: `math` plane representation (`Ax3` / normal + point), a
plane–plane line solve, and a segment-vs-convex-and-simple-polygon clip built from the 2D
routines already in `src/native/tessellate/uv_triangulate.h` / `trim.h`.

### 3.3 Face splitting

For each face, subdivide its 2D boundary polygon along its accumulated section segments →
a set of face **fragments** (sub-polygons), each lying entirely on one side of every section
segment. Reuse the native polygon subdivision / ear-clip machinery
(`uv_triangulate.h`) so fragments are consistent with the tessellator. A face with no
section segments is a single (unsplit) fragment.

### 3.4 Fragment classification

Classify each fragment by testing its **centroid** (a point strictly interior to the
fragment, in 3D) against the OTHER solid:

- INSIDE — centroid strictly inside the other closed shell.
- OUTSIDE — centroid strictly outside.
- ON — centroid on a face of the other solid (a coincident face → this is the ambiguity the
  preflight should have caught; if it reaches here, DECLINE).

Point-in-polyhedron test: cast a ray from the centroid and count crossings of the other
solid's faces (ray-parity), OR sum signed solid angles (winding) — a robust-enough fp64
predicate with an explicit ON-boundary tolerance. Ambiguous (near-ON) ⇒ DECLINE.

### 3.5 Surviving-shell assembly + sew/heal

Select surviving fragments by the op (BOPAlgo face-survival rules):

| op | keep `A` fragments | keep `B` fragments | orientation |
|---|---|---|---|
| fuse (`A ∪ B`) | OUTSIDE `B` | OUTSIDE `A` | as-is |
| cut (`A − B`) | OUTSIDE `B` | INSIDE `A` | `B` fragments REVERSED |
| common (`A ∩ B`) | INSIDE `B` | INSIDE `A` | as-is |

Sew the surviving fragments into a `Shell`: weld coincident vertices/edges (a tolerance
merge), verify every edge is shared by exactly two faces, and wrap the closed shell as a
`Solid`, oriented outward. If the sew leaves an open boundary or a non-manifold edge, the
candidate is malformed — the self-verify (§4) will reject it.

## 4. Mandatory self-verify guard (`src/engine/native/native_engine.cpp`)

After `nbool::build_boolean(...)` returns a candidate `topology::Shape`, the engine accepts
it as native ONLY if BOTH hold (else DISCARD → fall through to OCCT):

1. **Valid watertight 2-manifold** — reuse the existing `robustlyWatertight(solid)` helper
   (closed at every deflection in the `{0.05, 0.02, 0.01, 0.005}` ladder via
   `ntess::SolidMesher` + `ntess::isWatertight`, and positive `ntess::enclosedVolume`).
2. **Correct set-algebra volume** — compute the candidate's native volume `Vr`
   (`|enclosedVolume|`) and compare it to the expected set-algebra value derived from the
   operands' own native volumes `Va`, `Vb` and their intersection volume `Vab` (itself the
   native `common` volume, or a direct native `A∩B` measure):
   - fuse: `Vr ≈ Va + Vb − Vab`
   - cut: `Vr ≈ Va − Vab`
   - common: `Vr ≈ Vab`
   within a relative tolerance. This is the set-algebra sign+magnitude check the task
   mandates — it catches a result that is watertight but geometrically wrong (e.g. kept the
   wrong fragments).

This mirrors the Tier-D thread `robustlyWatertight` self-verify pattern already in the
engine, extended with the volume-algebra check specific to booleans.

## 5. `NativeEngine` wiring

```cpp
ShapeResult NativeEngine::boolean_op(EngineShape a, EngineShape b, int op) {
    // A native boolean needs the native B-rep of BOTH operands.
    if (!isNative(a) || !isNative(b)) return fallback().boolean_op(a, b, op);

    const ntopo::Shape& sa = nativeShapeOf(a);
    const ntopo::Shape& sb = nativeShapeOf(b);
    ntopo::Shape result = nbool::build_boolean(sa, sb, op);   // NULL ⇒ declined
    if (result.isNull() || !booleanSelfVerify(result, sa, sb, op))
        return fallback().boolean_op(a, b, op);               // labelled fall-through
    return track(wrapNative(std::move(result)));
}
```

`booleanSelfVerify` = `robustlyWatertight(result)` AND the set-algebra volume check (§4).
A foreign operand, a NULL native result (planar/coincidence/curved DECLINE), or a failed
self-verify all fall through to OCCT with no interception — identical to `cc_set_engine(0)`.
OCCT appears only under `CYBERCAD_HAS_OCCT` in the fallback wiring; the native builder
references no OCCT / `IEngine` / `EngineShape` type. (`native_engine.h` unchanged;
`boolean_op` is already declared.)

## 6. Guards (honesty — DECLINE → OCCT fall-through)

The native builder returns a **NULL `Shape`** (never a wrong solid) when:

- **Curved face** on either operand (any `FaceSurface::kind != Plane`).
- **Near-tangent / coincident** — coplanar overlapping faces within tolerance, a
  plane-pair coincident within tolerance, a fragment centroid near-ON the other solid, or
  bounding boxes that only touch (measure-zero contact).
- **Degenerate input** — empty / open operand shell, zero volume, or a self-intersecting
  operand.

Even past the builder, the **self-verify** (§4) is the final backstop: any candidate that
is not a valid watertight solid with the correct set-algebra volume is DISCARDED.

## 7. Cognitive complexity

- Each pipeline stage is its own function (`facePairSegments`, `splitFaceFragments`,
  `classifyFragment`, `assembleShell`, `sewHeal`), target ≤ 15 (systems band); the
  point-in-polyhedron predicate and plane–plane solve are short irreducible geometry (~5–10).
- `build_boolean` is a dispatcher (preflight guards + the four stage calls), target ≤ 15.
- Flag any genuinely irreducible face-pair / classification loop as systems-band (≤ 25) per
  NATIVE-REWRITE.md; split above ~35.

## 8. Verification detail

- **Gate 1 (host, no OCCT):**
  - Axis-aligned box **fuse / cut / common** — two overlapping unit-ish boxes; result
    watertight (`boundaryEdgeCount == 0`), closed 2-manifold (every edge shared by exactly
    two faces), and EXACT set-algebra volume (`|A|+|B|−|A∩B|` / `|A|−|A∩B|` / `|A∩B|`).
  - A **prism / simple-concave** case (L-prism cut by a box; a convex fuse) — watertight,
    exact volume.
  - **Self-verify rejects** a deliberately open / wrong-volume candidate.
  - **Fall-through triggers** — a curved-face operand, a coincident/tangent configuration,
    and a foreign body each yield NULL (or a self-verify reject) → fall through.
- **Gate 2 (sim, OCCT oracle):** `cc_boolean` native vs `BRepAlgoAPI_Fuse`/`_Cut`/`_Common`
  on axis-aligned boxes — volume / bbox / sub-shape counts / watertightness EXACT (relative
  error ~0); the fall-through cases (curved, coincident, foreign) asserted identical under
  both engines (fall-through proof, `cc_active_engine()==1`). OCCT default restored in
  teardown; the parity test carries its own `main()` (on the suite's SKIP list) so
  `run-sim-suite.sh` 221/221 is unchanged.

## References (oracle only — not copied)

- OCCT `/Users/leonardoaraujo/work/OCCT/src`: `BRepAlgoAPI_BooleanOperation`,
  `BRepAlgoAPI_Fuse` / `_Cut` / `_Common`, `BOPAlgo_Builder`, `BOPAlgo_PaveFiller`,
  `BOPTools_AlgoTools` (the intersect → pave → split → classify → build pipeline and the
  fuse/cut/common face-survival rules).
- Computational-geometry / mesh-boolean literature — polyhedral boolean via face splitting
  along intersection segments + point-in-polyhedron classification (ray parity / winding),
  BSP / half-space reasoning for convex parts; consistent with the native
  `src/native/math` + `src/native/tessellate` polygon and predicate routines.
