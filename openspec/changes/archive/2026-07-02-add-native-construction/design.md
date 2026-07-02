# Design — add-native-construction

## Method (locked, per NATIVE-REWRITE.md)

Clean-room from first principles + the `cc_*` contract semantics, using OCCT
source (`/Users/leonardoaraujo/work/OCCT/src`:
`BRepPrimAPI`/`BRepBuilderAPI`/`BRepOffsetAPI`) as a **reference oracle only** —
consulted to confirm the topology/orientation conventions match, never copied.
The two constructions here are exact analytic B-rep; no NURBS approximation is
needed. Balance maintainability / readability / performance: short linear
assemblers over `ShapeBuilder`, systems-band cognitive complexity only where the
face-of-revolution classification is genuinely irreducible.

## Layering

```
cc_solid_extrude / cc_solid_revolve   (facade, unchanged signatures)
        │
        ▼  active_engine()  ←── cc_set_engine / cc_active_engine toggle
   NativeEngine : IEngine   (src/engine/native/)
        ├─ solid_extrude (polygon)      → construct::extrudePolygon  ─┐
        ├─ solid_revolve (line-seg only)→ construct::revolveSegments ─┤ OCCT-free
        │                                                             │  native
        └─ everything else ─────────────────────────────────────────┘
                     │ delegate to fallback_
                     ▼
              OcctEngine (CYBERCAD_HAS_OCCT) | StubEngine (host)
```

`src/native/construct/` (OCCT-free) knows nothing about `IEngine`. It exposes
plain functions returning `topology::Shape`. The engine glue in
`src/engine/native/` adapts those to `EngineShape` and owns the fallback.

## Native constructor (`src/native/construct/`, OCCT-free)

Includes only `native/math` + `native/topology`. Header-only where practical, to
match the existing native foundations.

### extrudePolygon(profileXY, pointCount, depth) → Solid

Contract (from `cc_solid_extrude` doc): a closed 2D polygon in the XY plane,
extruded along `+Z` by `depth`.

Algorithm (bottom-up `ShapeBuilder`, from Mäntylä):
1. Normalise the profile: drop a duplicate closing point; require ≥ 3 distinct
   points; compute the signed area to fix CCW winding so the bottom-face normal
   points `−Z` (outward) and the top `+Z`.
2. Build `2·N` vertices: bottom ring at `z = 0`, top ring at `z = depth`
   (shared, one `Vertex` node each).
3. Build the bottom wire (N line edges on the bottom ring), the top wire (N line
   edges on the top ring), and N vertical edges (bottom→top) — each edge a
   `Line` `EdgeCurve` with a bounded parameter range and its two shared
   vertices.
4. Bottom face = `Plane(z=0, normal −Z)` bounded by the bottom wire (reversed so
   material is above). Top face = `Plane(z=depth, normal +Z)`.
5. For each profile edge `i`: a **planar quad side face** on the plane through
   the four corner vertices (bottom_i, bottom_{i+1}, top_{i+1}, top_i), its wire
   reusing the SHARED bottom edge, vertical edge, top edge, and the neighbour's
   vertical edge — so every edge is shared by exactly two faces (manifold).
6. Assemble one closed `Shell` of `N+2` faces, oriented outward; wrap in a
   `Solid`.

Degenerate depth (≈ 0) or a non-planar / self-intersecting profile → failure
(the engine returns unsupported and the facade yields 0). Convexity is NOT
required for extrude (planar side faces are valid for any simple polygon); a
self-intersecting profile is rejected.

### revolveSegments(profileXY, pointCount, angle) → Solid  (LINE-SEGMENT only)

Contract (from `cc_solid_revolve` doc): a profile revolved about an axis (the
native path fixes the axis as the Y axis in the profile plane, matching the
oracle's default and the existing OCCT behaviour; documented in the spec).

Per segment `(p0, p1)`, classify against the axis and build the exact analytic
face of revolution:
- **both endpoints equidistant from axis** but at different heights → the segment
  is parallel to the axis → **Cylinder** face (radius = that distance).
- **endpoints at the same height** (segment ⟂ axis) → **Plane** annulus/disc
  face (a washer perpendicular to the axis).
- **otherwise** (oblique) → **Cone** face (half-angle from the segment slope,
  reference radius at a reference height).
- a segment lying ON the axis contributes no face (degenerate radius 0).

Circular edges of revolution are `Circle` `EdgeCurve`s about the axis; the
meridian edges are the original line segments. Full **360°**: the revolved shell
closes on itself (start meridian == end meridian, shared), producing a closed
solid with no cap faces. **Partial angle** (`0 < θ < 2π`): add the two **planar
cap faces** (the profile swept nowhere — the flat start and end meridian planes)
so the solid is still closed and watertight.

Arc / spline segments are OUT OF SCOPE here → `NativeEngine::solid_revolve`
detects them and returns unsupported so the facade falls through to OCCT
(the native engine's fallback). Detection: the native `solid_revolve` takes only
a raw `profileXY` point list (all straight segments by construction of that ABI),
so the curved case arrives via `cc_solid_revolve_profile` (typed segments), which
is NOT overridden and falls through unconditionally.

### Cognitive complexity

`extrudePolygon` is a linear assembler (target ≤ 12). `revolveSegments`'
per-segment surface classification is the one systems-band function (target ≤ 25,
isolated switch on the segment/axis relation); if it exceeds that it will be
split by surface kind and flagged, per the repo complexity policy.

## Engine glue (`src/engine/native/native_engine.{h,cpp}`)

```cpp
class NativeEngine final : public IEngine {
 public:
  explicit NativeEngine(std::shared_ptr<IEngine> fallback);
  std::string name() const override { return "native"; }
  bool available() const override { return fallback_ && fallback_->available(); }

  ShapeResult solid_extrude(const double* xy, int n, double depth) override;   // native
  ShapeResult solid_revolve(const double* xy, int n, double angle) override;   // native line-seg;
                                                                               // else fallback_
  // EVERY other override forwards to fallback_->same(...).
 private:
  std::shared_ptr<IEngine> fallback_;
};
```

- The native shape is type-erased into `EngineShape` via a small holder that
  wraps a `topology::Shape` (a new native shape-holder analogous to the OCCT
  adapter's `TopoDS_Shape` holder). Mixing native-built and OCCT-built shapes in
  one operation is NOT supported in this change (feature/boolean stay OCCT and
  receive OCCT shapes); a native shape is currently a terminal build result read
  back by tessellation/query — those query/tessellate paths are DEFERRED for the
  native holder and fall through, so in practice a native shape is compared via a
  native-side host test and via the sim-parity harness, not fed back through an
  OCCT-only op. This limitation is stated in the spec.
- `solid_revolve` line-segment gate: the raw `profileXY` ABI carries only
  straight segments, so `solid_revolve` is always eligible; the curved case only
  exists on `solid_revolve_profile`, which is NOT overridden → falls through.
- **OCCT reference only under the guard.** The fallback wiring
  (`create_default_engine()` returning a `NativeEngine` wrapping the OCCT engine)
  lives behind `#ifdef CYBERCAD_HAS_OCCT`. `src/native/construct/` contains no
  OCCT reference at all.

## Facade toggle (`cc_set_engine` / `cc_active_engine`)

Modelled on `cc_set_parallel` / `cc_parallel_enabled`:

```c
/* ADDITIVE (not part of the mirrored ABI): swap the active geometry engine so a
 * native-vs-OCCT A/B is possible behind one cc_* call. Default OCCT. On a build
 * with no native engine wired this is a no-op and cc_active_engine() reports 0. */
void cc_set_engine(int native);   /* 1 = native, 0 = OCCT/default */
int  cc_active_engine(void);      /* 1 iff the native engine is active */
```

Definitions in `src/facade/cc_kernel.cpp`, `guard`/`guard_void` wrapped:
`cc_set_engine(1)` builds/installs the `NativeEngine` via `set_active_engine`;
`cc_set_engine(0)` restores `create_default_engine()`. `cc_active_engine()`
checks `active_engine()->name() == "native"`. The default engine is untouched at
startup, so every existing suite runs on OCCT exactly as before.

Because `active_engine()` is process-wide and mutex-guarded (see
`engine_registry.cpp`), the toggle is a global A/B switch; tests must restore the
default (`cc_set_engine(0)`) in teardown. This is documented in the spec so the
existing 221/221 suite stays deterministic.

## Alternatives considered

- **Per-call engine parameter** instead of a global toggle — rejected: it would
  change every `cc_*` signature (breaks the mirrored ABI). The global toggle
  matches the established `cc_set_parallel` pattern and keeps signatures fixed.
- **Native shapes convertible into OCCT** for downstream OCCT ops — deferred:
  requires a native→OCCT bridge in production code, out of scope; parity is done
  via the test-only bridge pattern already used by the tessellation harness.
- **Implement convex-decomposition side faces** for extrude — unnecessary: a
  planar quad per edge is valid for any simple polygon and matches the oracle.
