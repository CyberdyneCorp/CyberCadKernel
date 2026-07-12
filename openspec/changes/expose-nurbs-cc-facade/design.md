# Design — expose-nurbs-cc-facade

The one genuinely-reviewable decision here is **how a NURBS curve/surface crosses
the plain-C `cc_*` boundary**. Everything else (the per-feature wrappers, the
Python object model, the gallery) follows mechanically once that is fixed. This
document pins that decision, the accessor layout, the lifetime/error model, the
tessellation boundary, and the additive/app-compat guarantee.

## 1. Geometry-crossing model: opaque handles + POD accessors

`cc_*` today is handle-based: an integer `CCShapeId` indexes a registry of
solids/faces; POD structs (`CCMesh`, `CCMassProps`, …) carry value data across the
boundary. NURBS **curves** and **surfaces** have no handle type — that is the gap.

**Decision: add `cc_curve` and `cc_surface` as new opaque integer handle types,**
registry-backed exactly like `CCShapeId`, and read their data back through POD
accessor structs + buffer-fill calls. Rejected alternative: serialize every
curve/surface as raw pole/knot/weight arrays on every call (Option B). Handles win
because:

- **Identity enables chaining.** The Wave D–I value is in composition —
  `fit → fair → offset → tessellate`, `skin → join → trim-boolean`. Handles let a
  result feed the next op with no marshalling; serialization would copy the full
  control net across the boundary at every step.
- **Consistency.** It mirrors the model the ABI (and the Python RAII `Shape`)
  already use; the same registry, release, and use-after-release guards apply.
- **The boundary stays POD.** No C++/engine type crosses; accessors return plain
  ints/doubles into caller buffers.

Handles are additive: the app never references them, so nothing app-facing breaks.

## 2. Accessor layout (POD, read-back)

A curve/surface is opaque; callers read its data through a small header struct
plus buffer-fill calls sized from the header. Illustrative (final field order +
`sizeof` guards fixed in implementation, cross-checked against a C `sizeof`, as
`add-python-binding` §8.2 did):

```c
typedef struct { int32_t id; } cc_curve;      /* 4-byte opaque handle */
typedef struct { int32_t id; } cc_surface;

typedef struct {                 /* cc_curve_info(h, &info) */
  int32_t degree;
  int32_t n_ctrl;                /* control-point count           */
  int32_t n_knots;               /* = n_ctrl + degree + 1         */
  int32_t rational;              /* 0 = weights all 1, 1 = rational */
} CCCurveInfo;

typedef struct {                 /* cc_surface_info(h, &info) */
  int32_t degree_u, degree_v;
  int32_t n_ctrl_u, n_ctrl_v;
  int32_t n_knots_u, n_knots_v;
  int32_t rational;
} CCSurfaceInfo;
```

Buffer-fill calls copy C-owned data into caller-owned storage sized from the info
struct, returning the count written (or `< 0` on a too-small buffer):

```c
int cc_curve_knots (cc_curve h, double* out, int cap);   /* n_knots doubles      */
int cc_curve_poles (cc_curve h, double* out, int cap);   /* 4*n_ctrl: x,y,z,w    */
int cc_surface_knots_u(cc_surface h, double* out, int cap);
int cc_surface_knots_v(cc_surface h, double* out, int cap);
int cc_surface_poles (cc_surface h, double* out, int cap); /* 4*nu*nv, row-major */
```

Poles are always emitted in **homogeneous (x, y, z, w)** so the rational case is
uniform; a non-rational curve reports `rational = 0` with all `w = 1`. This is the
same homogeneous convention the native modules use internally, so no conversion
risk is introduced at the boundary.

Evaluation is a direct call (no handle churn):

```c
int cc_curve_eval  (cc_curve h, double t, double* xyz);          /* point         */
int cc_surface_eval(cc_surface h, double u, double v, double* xyz);
```

## 3. Lifetime & the honest-decline error model

- `cc_curve_release` / `cc_surface_release` free the registry slot, idempotent and
  crash-free on double release, exactly like `cc_shape_release`. The Python
  `Curve`/`Surface` wrap this in RAII (context manager + GC backstop + stale-handle
  guard), reusing the `Shape` machinery.
- **Every constructor/op returns a `0` (invalid) handle and sets `cc_last_error`
  on an honest decline** — the over-radius freeform fillet where the ball won't
  fit, the over-constrained fit, the G2-infeasible creased vertex blend, the
  coincident-loop trim boolean. This is the ABI-level expression of the kernel's
  `DISAGREED = 0` discipline: a decline is a typed error, never a
  plausible-but-wrong handle. Python raises `KernelError` carrying
  `cc_last_error()`, matching the existing pattern. No tolerance is widened at the
  boundary to turn a decline into a success.

## 4. Tessellation bridge — display only, NOT the mesher wall

To *show* results, a NURBS surface must reach the mesh/render pipeline:

```c
int cc_surface_tessellate(cc_surface h, const CCTessOptions* opt, CCMesh* out);
int cc_curve_polyline    (cc_curve  h, int n_samples, CCEdgePolyline* out);
```

This is **single-surface display tessellation** — a triangle mesh for
visualization — and is explicitly distinct from the **frozen-mesher curved-seam
weld** that gates a watertight multi-face B-rep solid (the architectural wall
documented for Layer 3/8). This change tessellates one surface for rendering; it
does **not** claim to weld multiple curved trimmed faces into a watertight solid,
and the gallery pieces that are single surfaces (fills, sweeps, blends) render
directly, while multi-face assemblies are shown as face sets, not as a sewn solid.
An optional `cc_surface_to_face` that admits a NURBS surface into the existing
face/body model is included only where the target engine already accepts a
tessellated face, and carries the same non-watertight caveat.

## 5. Feature wrappers (`cc_nurbs_*`) — thin, grouped

Each wrapper validates inputs, calls the native module, and registers the result
handle (or declines). Groups and representative symbols:

| Group | Symbols (representative) | Native module |
|---|---|---|
| Fitting / reverse-eng | `cc_nurbs_fit_curve`, `cc_nurbs_fit_surface`, `cc_nurbs_interp_curve`, `cc_nurbs_estimate_weights_curve/surface`, `cc_nurbs_fit_curve_constrained`, `cc_nurbs_fit_surface_constrained`, `cc_nurbs_fair_curve/surface`, `cc_nurbs_simplify_curve`, `cc_nurbs_detect_primitive`, `cc_nurbs_recognize_curve/surface` | `bspline_fit`, `bspline_fair`, `bspline_simplify`, `primitive_fit`, `analytic_nurbs` |
| Analytic → NURBS | `cc_nurbs_circle`, `cc_nurbs_arc`, `cc_nurbs_ellipse`, `cc_nurbs_cylinder`, `cc_nurbs_cone`, `cc_nurbs_sphere`, `cc_nurbs_torus`, `cc_nurbs_plane` | `analytic_nurbs` |
| Intersection | `cc_nurbs_intersect_cc`, `cc_nurbs_intersect_cs` | `bspline_intersect` |
| Surfacing | `cc_nurbs_skin`, `cc_nurbs_gordon`, `cc_nurbs_coons`, `cc_nurbs_nsided_fill` (mode C0/G1/G2/rational), `cc_nurbs_sweep_variable`, `cc_nurbs_sweep_two_rail`, `cc_nurbs_revolve`, `cc_nurbs_join` (mode G1/G2) | `bspline_skin/gordon/coons/nsided*/sweep/join` |
| Blend | `cc_nurbs_fillet_freeform_g2`, `cc_nurbs_vertex_blend`, `cc_nurbs_chamfer_variable`, `cc_nurbs_chamfer_freeform` | `blend/*` |
| Offset / thicken | `cc_nurbs_offset_rational`, `cc_nurbs_offset_trimmed`, `cc_nurbs_thicken_trimmed`, `cc_nurbs_shell_trimmed` | `bspline_offset/thicken/shell` |
| Trimmed B-rep | `cc_nurbs_trim_region_boolean` | `topology/trim_boolean` |

Naming is `cc_nurbs_<verb>` to namespace the additions clearly away from the
existing 154 symbols; all are declared in a new `cc_kernel_nurbs.h` included by
`cc_kernel.h`, so the app's header set is a superset, never a change.

## 6. Additive / app-compat guarantee

- The existing 154 `cc_*` symbols and every existing POD struct are byte-unchanged
  (guarded by `test_abi`). New symbols only.
- New handle types (`cc_curve`, `cc_surface`) and structs (`CCCurveInfo`,
  `CCSurfaceInfo`, `CCTessOptions`) are new declarations the app does not reference,
  so the `ios-app-compat` "kernel ABI is a verified superset of the app's bridge
  header" requirement continues to hold.
- `src/native/**` stays OCCT-free; the bridging lives in `src/facade`.

## 7. Delivery phasing (maps to the wave-loop)

The scope is large; implementation lands as isolated waves so each is reviewable
and independently verifiable:

1. **J1 — ABI foundation**: `cc_curve`/`cc_surface` handles, accessors,
   evaluators, tessellation bridge, lifetime, `sizeof` guards. (The data-model core
   of this change; everything else depends on it.)
2. **J2–J5 — feature wrappers** by group (fitting+conversion; surfacing;
   blend+offset; intersection+trim-boolean), file-disjoint.
3. **J6 — Python** `Curve`/`Surface`/`nurbs` object model + pytest.
4. **J7 — example gallery** + PNG renders.

Each wave carries the standard invariants (src/native OCCT-free, `cc_*` existing
symbols byte-unchanged, honest-decline over silent-wrong, a regression test per
feature).
