# Design — add-native-construction-profiles

## Method (locked, per NATIVE-REWRITE.md)

Clean-room from the `cc_*` contract semantics — the `CCProfileSeg` struct and the
doc-comments on `cc_solid_extrude_holes` / `_polyholes` / `_profile` /
`_profile_polyholes` / `cc_solid_revolve_profile` in
`include/cybercadkernel/cc_kernel.h` — and computational-geometry first principles,
with OCCT source (`/Users/leonardoaraujo/work/OCCT/src`) as a **reference oracle
only** (consulted to confirm inner-wire orientation, revolve-surface classification,
and numerics; never copied):

- `BRepBuilderAPI_MakeFace(outerWire)` + `Add(innerWire)` — hole faces are a face with
  a reversed inner wire; confirms the orientation convention Tier A uses.
- `BRepPrimAPI_MakeRevol` / `BRepSweep_Revol` + `ElSLib` — surface-of-revolution
  classification (Plane / Cylinder / Cone / Sphere / Torus) and the analytic frames.

The delivered constructions are **exact analytic B-rep** (Plane / Cylinder / Cone /
Sphere surfaces, Line / `Circle` edges); no NURBS approximation is introduced. Balance
maintainability / readability / performance: short linear assemblers over
`ShapeBuilder`, with the surface-of-revolution classification and the hole-wire sweep
isolated as the only systems-band functions.

## What already exists (built on, not rebuilt)

- `src/native/construct/construct.h` — #4 assemblers: `planarFace(loop, normal, …)`,
  `lineEdge`, `arcEdge`, `build_prism(profileXY, n, depth)`,
  `build_revolution(segments, axis, angle)` with the `RevPoint` / `AxisFrame` /
  `segmentSurface` helpers (Plane / Cylinder / Cone classification for line segments,
  full-turn close + partial-angle meridian caps). Tier A **extends** these.
- `src/native/topology` — `ShapeBuilder::makeVertex/makeEdge/makeWire/makeFace/
  makeShell/makeSolid`, `EdgeCurve{Line,Circle,Ellipse,BSpline,Bezier}`,
  `FaceSurface{Plane,Cylinder,Cone,Sphere,BSpline,Bezier}` (note: **no Torus**).
- `src/native/math` — `Ax3`, `Plane/Cylinder/Cone/Sphere`, Bézier/B-spline/NURBS eval,
  `Circle`/arc parameterisation for the analytic edge frames.
- `NativeEngine` — the five method overrides already exist and currently `return
  fallback().<same>(...)`. Only their bodies change.

## Capability constraints that fix the honest scope

| Sub-case | Native surface needed | Available? | Decision |
|---|---|---|---|
| circular hole side | `Cylinder` | yes | **native** |
| polygon hole side | `Plane` | yes | **native** |
| arc / circle outer profile edge, extruded | `Cylinder` side + `Circle` edge | yes | **native** |
| line segment revolved | Plane / Cylinder / Cone | yes | **native** |
| arc/circle revolved, meridian **centred on axis** | `Sphere` | yes | **native** |
| arc revolved, **offset from axis** | `Torus` | **no** | **fall through** |
| spline (kind 3) profile edge | `BSpline` surface of revolution / swept `BSpline` side | modelled, but hard | **fall through** |
| any spline-revolve | `BSpline` surface of revolution | modelled, but hard | **fall through** |

The "no Torus" and "spline is a separate hard piece" facts are the reason Tier A
defers exactly those sub-cases — the native builder returns NULL and the glue falls
through, never faking a surface.

## Native builders (`src/native/construct/`, OCCT-free)

### 1. Holed prisms

Generalise the #4 prism to inner wires. A prism is: a **bottom cap face**, a **top cap
face**, an **outer side shell**, and — new — **one inner side shell per hole**. The cap
faces gain reversed inner wires (one per hole) so the material is the annulus.

```
build_prism_holes(outerXY, outerN, circleHoles{cx,cy,r}, polyHoles{ptsXY,counts},
                  depth) -> topo::Shape (Solid)
```

Algorithm:
1. Build the outer prism rings/caps/side shell exactly as #4 `build_prism`.
2. For each **circular hole** `(cx,cy,r)`: build a bottom `Circle` edge and a top
   `Circle` edge (TRUE circle, `EdgeCurve::Kind::Circle`, one edge each), a shared
   seam line edge bottom→top, and one **`Cylinder`** inner side face bounded by that
   wire (oriented inward — normal toward the axis so material is outside). Add the
   reversed bottom/top circle edges as inner wires of the bottom/top caps.
3. For each **polygon hole** (a point list): build bottom/top inner rings (N line
   edges each) + N vertical line edges + N **`Plane`** inner side faces (inward), and
   add the reversed bottom/top inner rings as inner wires of the caps.
4. Assemble one closed `Shell` (outer caps-with-holes + outer side + all inner sides)
   → `Solid`.

Reject: hole outside/overlapping the outer polygon, hole radius/area ≤ 0, holes that
overlap each other (documented tolerance) → NULL (fall through). Winding of each hole
wire is normalised opposite to the outer wire (reversed inner wire).

### 2. Typed-profile prisms

Build the OUTER wire from `CCProfileSeg`s, then reuse the holed-prism machinery.

```
build_prism_profile(segs, segCount, splineXY, circleHoles, polyHoles, depth) -> Shape
```

- **kind 0 line** `(x0,y0)->(x1,y1)` → a `Line` edge; sweeps to a `Plane` side face.
- **kind 1 arc** `(cx,cy,r, a0->a1, endpoints)` → one **`Circle`** `EdgeCurve` over
  `[a0,a1]`; sweeps to a **`Cylinder`** side face; endpoints shared with neighbours.
- **kind 2 full circle** `(cx,cy,r)` → a single closed **`Circle`** edge forming a
  one-edge outer wire (a disc profile); sweeps to a full `Cylinder` side and a disc
  cap. (A full-circle profile means the whole outer boundary is that one circle.)
- **kind 3 spline** → **return NULL** (fall through; not faked).

Segments must chain end-to-end into ONE closed outer wire (endpoint continuity within
tolerance) else NULL. Then extrude + add circular / polygon holes as in (1).

### 3. Typed-profile revolve

```
build_revolution_profile(segs, segCount, splineXY, axis{ax,ay,adx,ady}, angle) -> Shape
```

Generalise #4 `build_revolution` from the fixed axis to the caller's in-plane axis
(`AxisFrame` built from `(ax,ay)` + normalised `(adx,ady)`). Per segment, classify vs
the axis and build the exact analytic face of revolution:

- **kind 0 line**: parallel→`Cylinder`, perpendicular→`Plane`, oblique→`Cone`,
  on-axis→none (the #4 `segmentSurface`, reused via the general axis).
- **kind 1 arc / kind 2 circle**, meridian circle **centred ON the axis** (centre lies
  on the axis line within tolerance): the revolved surface is a **`Sphere`** of that
  radius about the centre; the arc's angular span sets the sphere's v-range. A degenerate
  arc (zero curvature / centre at infinity) reduces to the line cases.
- **kind 1 arc**, centre **off the axis**: swept surface is a **torus** →
  **return NULL** (fall through).
- **kind 3 spline** → **return NULL** (fall through).

Circular edges of revolution are TRUE `Circle` edges about the axis; meridian edges are
the original line/arc edges. Full 360° closes the shell; partial angle adds the two
planar meridian cap faces (as #4).

### Cognitive complexity

`build_prism_holes` and `build_prism_profile` are linear assemblers (target ≤ 15;
inner-wire loop extracted). The revolve segment classification (`segmentSurface`
extended with the sphere/torus branch) is the one systems-band function (target ≤ 25,
isolated switch on segment-kind × axis-relation); the torus/spline branches are early
`return NULL` guards, keeping the classification flat. Split + flag if any exceeds the
systems band, per the repo complexity policy.

## Engine glue (`src/engine/native/native_engine.cpp`)

Each of the five overrides changes from pure fall-through to native-else-fallback:

```cpp
ShapeResult NativeEngine::solid_extrude_holes(const double* o, int oc, const double* h,
                                              int hc, double d) {
  topo::Shape s = construct::build_prism_holes(o, oc, /*circles=*/h, hc, /*polys=*/{}, d);
  if (!s.isNull()) return track(wrapNative(std::move(s)));   // built natively
  return fallback().solid_extrude_holes(o, oc, h, hc, d);    // unsupported sub-case
}
```

Same pattern for `solid_extrude_polyholes`, `solid_extrude_profile`,
`solid_extrude_profile_polyholes`, `solid_revolve_profile`. A NULL native `Shape`
(degenerate input, or a deferred sub-case — SPLINE edge / torus-revolve / spline-
revolve) triggers the existing fall-through with **no interception** — identical to the
current #4 behaviour for that input. The native shape holder (`wrapNative` / `track`)
and native tessellate / mass / bbox / subshape paths are the #4 machinery, reused
unchanged, so a native holed/profile body is read back by the same native query paths.

OCCT stays behind `CYBERCAD_HAS_OCCT` (only in the fallback wiring); the native
builders in `src/native/construct/` reference no OCCT type. `native_engine.h` needs no
change — the five signatures already exist.

## Facade

**No change.** `cc_solid_extrude_holes` / `_polyholes` / `_profile` /
`_profile_polyholes` / `cc_solid_revolve_profile` already route through the active
engine in `src/facade/cc_kernel.cpp`; the `cc_set_engine` toggle from #4 selects
native vs OCCT. No `cc_*` signature, POD layout, or `CCProfileSeg` field changes.

## Alternatives considered

- **BSpline surface of revolution for the torus / spline sub-cases** — rejected for
  Tier A: correct watertight seam-welded B-spline surfaces of revolution are a
  strictly harder, separately verifiable piece; emitting them now risks a faked/
  approximate surface. Deferred to a later #4b tier; Tier A falls through honestly.
- **Sampling arcs/circles into polygon edges** — rejected: violates the contract's
  "TRUE circle edges / a whole arc = ONE selectable edge" requirement and the
  clean-room "real native-math curved edges" mandate.
- **A `Torus` `FaceSurface` kind added now** — out of scope: adding an analytic torus
  surface + its tessellation + native parity is its own increment; not smuggled into
  Tier A.
