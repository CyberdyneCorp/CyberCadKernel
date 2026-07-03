# Design — add-native-loft

## Method (locked, per NATIVE-REWRITE.md)

Clean-room from the `cc_*` contract semantics — the `cc_solid_loft` and
`cc_solid_loft_wires` doc-comments in `include/cybercadkernel/cc_kernel.h`
(`cc_solid_loft`: `bottomXY` at `z=0` + `topXY` at `z=depth`, same point count →
ruled solid; `cc_solid_loft_wires`: two arbitrary 3D wires, same point count) — and
computational-geometry first principles, with OCCT source
(`/Users/leonardoaraujo/work/OCCT/src`) as a **reference oracle only** (consulted to
confirm the ruled-skin decomposition and outward-orientation conventions; never
copied):

- `BRepOffsetAPI_ThruSections` constructed with `ruled = true` — "the faces
  generated between the edges of two consecutive wires are ruled surfaces"
  (`BRepOffsetAPI_ThruSections.hxx`). With two sections of equal edge count this is
  exactly the pair-and-rule construction Tier B builds; it is the parity oracle.
- `BRepFill` / `BRepFill_Filling` ruled-surface between two edges — confirms the
  degree-1 skin between corresponding edges and its parameterisation.

The delivered construction is **exact analytic / low-degree B-rep** (a degree-1
`Bezier`/`BSpline` skin per side face, or an exact `Plane` when the four corners are
coplanar; `Plane` caps for planar sections; `Line` connecting edges); no
higher-degree NURBS approximation is introduced. Balance maintainability /
readability / performance: a short linear assembler over `ShapeBuilder`, with the
per-edge-pair ruled-face loop and the planar-vs-ruled surface classification
isolated as the only functions near the systems band.

## What already exists (built on, not rebuilt)

- `src/native/construct/construct.h` — #4 assemblers: `detail::planarFace(loop,
  normal, orient)`, `detail::lineEdge(v0,v1)`, `build_prism(profileXY,n,depth)`,
  `build_revolution(...)`, and the `ShapeBuilder` usage patterns Tier B mirrors.
- `src/native/construct/profile.h` — Tier-A `build_prism_with_holes` /
  `build_prism_profile` / `build_revolution_profile` and the multi-loop cap /
  side-shell assembly idioms.
- `src/native/topology` — `ShapeBuilder::makeVertex / makeEdge / makeWire / makeFace
  / makeShell / makeSolid`, `EdgeCurve{Line,Circle,Ellipse,BSpline,Bezier}`,
  `FaceSurface{Plane,Cylinder,Cone,Sphere,BSpline,Bezier}` (note: a `Bezier`/
  `BSpline` face surface can hold the degree-1 ruled skin; `Plane` holds the
  coplanar limit and the caps).
- `src/native/math` — `Vec3`/`Point3`/`Dir3`, `Ax3`, `Plane`, Bézier/B-spline/NURBS
  surface eval. A **ruled/skin helper** is added here iff the existing surface
  primitives cannot express a degree-1 skin between two edges directly.
- `src/native/tessellate` — the mesher that reads back the native `Solid` for the
  host watertightness / volume assertions and the sim parity comparison.
- `NativeEngine` — `solid_loft` and `solid_loft_wires` overrides already exist and
  currently `return fallback().<same>(...)`. Only their bodies change; the
  `wrapNative` / `track` native-body machinery from #4 / Tier A is reused unchanged.

## Capability constraints that fix the honest scope

| Configuration | Native surface / topology needed | Available? | Decision |
|---|---|---|---|
| two planar sections, equal count, ruled side per edge pair | degree-1 `Bezier`/`BSpline` skin (or `Plane` when coplanar) + `Plane` caps | yes | **native** |
| a side quad whose 4 corners are coplanar | `Plane` | yes | **native** (exact) |
| a side quad whose 4 corners are NOT coplanar | degree-1 skin (`Bezier`/`BSpline`) | yes | **native** |
| mismatched vertex counts | canonical 1:1 correspondence / point insertion | **no** (ambiguous) | **fall through** |
| a section degenerating to a point (punctual) | triangular-fan apex cap (cone-like) | different topology | **fall through** |
| a non-planar section wire (cap not a single plane) | general non-planar cap face | **no** (separate hard piece) | **fall through** |
| a self-intersecting section / ruling | — (no valid watertight solid) | — | **reject → fall through** |
| 3+ sections / guided / rail loft | multi-section skinning / rail sweep | out of Tier B | **fall through** (Tier C) |

The "equal-count only", "planar-cap only", and "no punctual section" facts are the
reason Tier B defers exactly those configurations — the native builder returns NULL
and the glue falls through, never faking a surface or an invalid solid.

## Native builder (`src/native/construct/`, OCCT-free)

### Ruled/skin surface (native-math)

For a corresponding edge pair — bottom edge `A: a[i]→a[i+1]`, top edge
`B: b[i]→b[i+1]` — the ruled side surface is
`S(u,v) = (1−v)·A(u) + v·B(u)`, `u,v ∈ [0,1]`. With straight section edges this is
the **bilinear patch** over the four corners `a[i], a[i+1], b[i+1], b[i]` — a
degree-1 surface in both directions, expressed as a degree-1 `Bezier` (or `BSpline`)
face surface. When the four corners are **coplanar** within tolerance the patch is
an exact `Plane`; the classifier emits `Plane` in that case (matching OCCT, which
also degrades a coplanar ruled face to a plane) and the degree-1 skin otherwise.

The native-math helper (added iff not already expressible) evaluates `S(u,v)` and
its normal so the tessellator and the parity gate agree with the surface stored on
the face; it is pinned by host analytic tests (a known bilinear point; a coplanar
patch equals its plane).

### Two-section ruled loft assembler

```
build_loft_ruled(sectionA_XYZ, sectionB_XYZ, n) -> topo::Shape (Solid)
```

Algorithm:
1. **Validate** (else return NULL → fall through): equal count `n ≥ 3`; each section
   has ≥ 3 distinct points; neither section is punctual (not all points coincident);
   each section is planar within `kProfileTol`; sections do not self-intersect.
2. Make the `2n` section vertices, the `n` **connecting edges** `a[i]→b[i]` (`Line`,
   built once and shared by the two adjacent side faces), and the `2n` **section
   edges** (`a[i]→a[i+1]`, `b[i]→b[i+1]`).
3. For each `i ∈ [0,n)`: build ONE **ruled side face** bounded by the four-edge wire
   `a[i]→a[i+1]`, `a[i+1]→b[i+1]`, `b[i+1]→b[i]`, `b[i]→a[i]`, carrying the degree-1
   skin surface (or `Plane` when coplanar), oriented outward.
4. Build the **bottom cap** (section A, normal away from B) and **top cap**
   (section B, normal away from A) as `Plane` faces with the section wire.
5. Assemble one closed outward `Shell` (`n` side faces + 2 caps) → `Solid`. If the
   shell is not watertight / manifold, return NULL (fall through).

Winding is normalised so the caps point away from each other and the side faces
point outward (consistent with the #4 prism convention).

### `cc_solid_loft` vs `cc_solid_loft_wires`

- `build_loft(bottomXY, topXY, n, depth)` — lift `bottomXY[i]=(x,y)` to
  `(x,y,0)` and `topXY[i]=(x,y)` to `(x,y,depth)`, then call `build_loft_ruled`.
  This is the `cc_solid_loft` path.
- `build_loft_wires(aXYZ, bXYZ, n)` — pass the two 3D point lists straight to
  `build_loft_ruled`. This is the `cc_solid_loft_wires` path.

(Final header/function names may be unified — e.g. one `build_loft_ruled` core with
two thin adapters — and are documented in `tasks.md` if they differ from the intent
above, exactly as Tier A documented its naming.)

### Cognitive complexity

`build_loft_ruled` is a linear assembler (target ≤ 15; the per-edge-pair side-face
loop and the validation are extracted). The planar-vs-ruled surface classifier
(coplanar-corner test → `Plane` else degree-1 skin) is the one function that may
approach the systems band (target ≤ 25, isolated); split + flag if it exceeds the
band, per the repo complexity policy.

## Engine glue (`src/engine/native/native_engine.cpp`)

Each of the two overrides changes from pure fall-through to native-else-fallback:

```cpp
ShapeResult NativeEngine::solid_loft(const double* b, int bc, const double* t,
                                     int tc, double d) {
  ntopo::Shape solid = ncst::build_loft(b, bc, t, tc, d);  // NULL on any deferred config
  if (solid.isNull()) return fallback().solid_loft(b, bc, t, tc, d);
  return track(wrapNative(std::move(solid)));
}

ShapeResult NativeEngine::solid_loft_wires(const double* a, int ac, const double* b,
                                           int bc) {
  ntopo::Shape solid = ncst::build_loft_wires(a, ac, b, bc);  // NULL on any deferred config
  if (solid.isNull()) return fallback().solid_loft_wires(a, ac, b, bc);
  return track(wrapNative(std::move(solid)));
}
```

A NULL native `Shape` (mismatched count, punctual/non-planar section, non-closeable
ruling) triggers the existing fall-through with **no interception** — identical to
the current #4 behaviour for that input. `native_engine.h` needs no change (both
signatures already exist). OCCT stays behind `CYBERCAD_HAS_OCCT` (only in the
fallback wiring); the native builder references no OCCT type.

## Facade

**No change.** `cc_solid_loft` and `cc_solid_loft_wires` already route through the
active engine in `src/facade/cc_kernel.cpp`; the `cc_set_engine` toggle from #4
selects native vs OCCT. No `cc_*` signature or POD layout changes.

## Alternatives considered

- **Handling mismatched point counts by re-parameterising / inserting points** —
  rejected for Tier B: OCCT's compatibility step (uniform re-parameterisation and
  vertex insertion so sections share an edge count) is a strictly harder, separately
  verifiable piece; emitting it now risks a mismatched/faked correspondence.
  Deferred; Tier B falls through honestly on unequal counts.
- **Punctual (point) sections as triangular fans** — rejected: a cone-like apex cap
  is a genuinely different topology from the quad-ruled band; smuggling it into Tier
  B risks degenerate near-zero-area quad faces. Deferred to a later tier.
- **General non-planar caps for `cc_solid_loft_wires`** — rejected: a non-planar
  section cap needs a triangulated/curved cap face and its own watertightness proof;
  Tier B accepts only planar-cappable sections and falls through otherwise.
- **Sampling the ruled skin into flat triangle-strip faces** — rejected: violates
  the clean-room "real native-math surfaces" mandate and would not match the OCCT
  ruled-surface oracle; the side faces carry a true degree-1 skin (or exact `Plane`).
