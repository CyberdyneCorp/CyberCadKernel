# Design — widen-native-step-import

Three **independent, honestly-gated** breadth slices on the WORKING native STEP reader
(`src/native/exchange/step_reader.{h,cpp}`, the archived `add-native-step-import` first
slice): (T1) map `ELLIPSE` onto the native ellipse edge kind + keep `TOROIDAL_SURFACE` a
documented DECLINE (no native surface kind); (T2) lift the blanket `>1 MANIFOLD_SOLID_BREP`
assembly-decline into a multi-solid **Compound** import; (T3) close the deferred bspline-face
round-trip IF a non-fabricated native fixture is constructible, else skip honestly. Each track
maps ONLY onto native geometry that genuinely exists and DECLINEs otherwise (NULL → OCCT).
Clean-room from ISO 10303-21/-42 + the existing reader; OCCT (`STEPControl_Reader` /
`STEPControl_Writer`) is the ORACLE + fixture-author + fallback only. NOT a general AP242
parser; arbitrary import stays OCCT.

## 0. What the reader already does (the substrate this widens)

The reader tokenizes the DATA section to `map<#id, Record>`, then two passes: Pass A resolves
leaf geometry (`point` / `direction` / `axis2placement` / `curve` / `surface`), Pass B builds
topology (`VERTEX_POINT` → vertex, `EDGE_CURVE` → one shared edge per `#id` with a projected
param range, `ORIENTED_EDGE` → oriented, `EDGE_LOOP` → wire with the periodic seam dropped,
`ADVANCED_FACE` → face, `CLOSED_SHELL` → shell, `MANIFOLD_SOLID_BREP` → solid). `build()`
gates on `validateUnitContext` (mm only) then `findSingleManifoldBrep` (exactly one root).
The engine (`native_engine.cpp::step_import`) keeps the result native ONLY when
`robustlyWatertight` passes across a deflection ladder; else it falls to OCCT. This change
touches Pass A `curve()` (T1a), `build()` + root discovery (T2), the engine self-verify
(T2 compound), and the DECLINE documentation (T1b) — nothing else.

## 1. T1 — `ELLIPSE` (map) + `TOROIDAL_SURFACE` (documented DECLINE)

### 1.1 Native kinds that genuinely exist (checked)

| STEP entity | Native kind | Exists? | Evidence |
|---|---|---|---|
| `ELLIPSE` | `EdgeCurve::Kind::Ellipse` (`radius`=major, `minorRadius`=minor) | **YES** | `shape.h` enum + fields; `edge_mesher.h::edgeCurveLocal` case `K::Ellipse` evaluates `origin + X·(major·cos t) + Y·(minor·sin t)`; `trim.h` case `K::Ellipse` trims it |
| `TOROIDAL_SURFACE` | `FaceSurface::Kind::Torus` | **NO** | `shape.h` `FaceSurface::Kind` = `{Plane,Cylinder,Cone,Sphere,BSpline,Bezier}`; `surface_eval.h` has NO torus arm; `math::Torus` is only emitted (by construction) as a *rational* B-spline patch, which the reader DECLINEs |

### 1.2 T1a — `ELLIPSE` → `EdgeCurve::Kind::Ellipse`

STEP form (ISO 10303-42): `ELLIPSE('', #position, semiAxis1, semiAxis2)` where `#position` is
an `AXIS2_PLACEMENT_3D` (centre + Z normal + X = major-axis direction), `semiAxis1` the
semi-major length (along X), `semiAxis2` the semi-minor length (along Y = Z×X). This is the
exact analogue of `CIRCLE('', #position, radius)`, so the new `curve()` arm mirrors the
existing `CIRCLE` arm:

```cpp
if (r->keyword == "ELLIPSE") {
  if (r->args.size() != 4 || !r->args[1].isRef() ||
      !r->args[2].isNumber() || !r->args[3].isNumber()) return std::nullopt;
  const auto f = axis2placement(r->args[1].ref);
  if (!f) return std::nullopt;
  const double a = r->args[2].asReal();   // semi-major (frame X)
  const double b = r->args[3].asReal();   // semi-minor (frame Y)
  if (!(a > 0.0) || !(b > 0.0)) return std::nullopt;  // degenerate → DECLINE
  topo::EdgeCurve c;
  c.kind = topo::EdgeCurve::Kind::Ellipse;
  c.frame = *f;
  c.radius = a;         // native: radius = major
  c.minorRadius = b;    // native: minorRadius = minor
  return c;
}
```

The `EDGE_CURVE` param range comes from the existing vertex-projection path (the ellipse is
evaluated by the same `edgeCurveLocal`/trim machinery a circle uses, parameterised by angle
`t`), so no new range logic is needed. The frame's X axis is the major axis exactly as the
tessellator's `X·(major·cos t)` expects — the placement convention already matches.

**Why FOREIGN OCCT authoring is the honest test.** The native writer's `curve()` switch has
NO `ELLIPSE` arm (`case K::Ellipse` is absent; it falls to `default: return 0`, guarded by
`canSerialize`), so a native ellipse-edge solid is not even natively exportable — there is no
native→native round-trip to lean on. The honest verification is therefore: OCCT
`STEPControl_Writer` authors a solid carrying an `ELLIPSE` edge (e.g. an elliptical hole /
elliptical rim), the native reader imports it, and it is compared vs the OCCT re-import. This
matches the task's stated method (foreign-import vs OCCT re-import).

### 1.3 T1b — `TOROIDAL_SURFACE` stays a documented DECLINE

There is no native surface kind to map a torus onto, and the two paths that could fake one are
both prohibited:
- **Adding `FaceSurface::Kind::Torus` + a `surface_eval.h` torus arm** would modify the
  tessellator — forbidden by the task's hard discipline. Without a tessellator arm a torus
  face cannot mesh, so the engine's `robustlyWatertight` self-verify could never pass anyway.
- **Mapping to a rational-quadratic B-spline patch** (the form `math::Torus` construction
  emits) is a *weighted* B-spline, which the reader already DECLINEs (non-rational-only gate
  in `bsplineSurface`); forcing it would violate the no-weakening / no-wrong-map rule.

So `surface()` keeps returning `std::nullopt` for `TOROIDAL_SURFACE`; the only change is a
comment naming the reason so it reads as a deliberate, honest DECLINE — the file falls through
to OCCT `STEPControl_Reader`, which meshes the torus. This is the correct outcome under the
"map only to native geometry that genuinely exists" rule.

## 2. T2 — multi-solid import (Compound)

### 2.1 Root discovery

Today `findSingleManifoldBrep` DECLINEs on the second `MANIFOLD_SOLID_BREP`. Replace with
`findManifoldBreps` returning the sorted list of root brep ids:

```cpp
std::vector<int> findManifoldBreps() {
  if (hasNestedAssembly()) { decline(); return {}; }   // §2.3
  std::vector<int> ids;
  for (const auto& [id, r] : recs_)
    if (!r.combined && r.keyword == "MANIFOLD_SOLID_BREP") ids.push_back(id);
  std::sort(ids.begin(), ids.end());   // stable, #id order == author order
  return ids;
}
```

Root = a `MANIFOLD_SOLID_BREP` in the representation's item list. In the multi-solid files OCCT
`STEPControl_Writer` emits for several co-equal solids, each `MANIFOLD_SOLID_BREP` is a direct
item of one `ADVANCED_BREP_SHAPE_REPRESENTATION` (or a small set thereof) with no assembly
usage — that is the case T2 accepts. A transform tree (§2.3) is NOT.

### 2.2 Assembly in `build()`

```cpp
topo::Shape build() {
  if (!validateUnitContext()) return {};
  const std::vector<int> brepIds = findManifoldBreps();
  if (brepIds.empty() || fail_) return {};
  std::vector<topo::Shape> solids;
  for (int id : brepIds) {
    topo::Shape s = mapManifoldBrep(id);
    if (fail_ || s.isNull()) return {};       // any member fails → whole import DECLINEs
    solids.push_back(std::move(s));
  }
  if (solids.size() == 1) return solids.front();          // unchanged single-solid path
  return topo::ShapeBuilder::makeCompound(std::move(solids));  // ≥2 → compound
}
```

`mapManifoldBrep` is reused per solid unchanged — each solid's shell is built and shared-node
watertight by construction exactly as the single-solid slice. The **single-solid path is
byte-for-byte the prior behaviour** (no regression). Only ≥2 roots produce a Compound.

### 2.3 Nested-assembly DECLINE (out of scope, honest)

A product-structure assembly (a transform tree, mapped items, external part refs) is NOT a
flat multi-solid representation and is out of scope. `hasNestedAssembly()` DECLINEs if any of
`NEXT_ASSEMBLY_USAGE_OCCURRENCE`, `MAPPED_ITEM`, `ITEM_DEFINED_TRANSFORMATION`,
`REPRESENTATION_RELATIONSHIP`, `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION`,
`CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` appears among the records — those are the entities
that place instances by a transform, which the flat-compound path would silently ignore
(mis-placing solids). Declining them → OCCT is the honest outcome (no wrong placement, no
fabricated tree).

### 2.4 Engine self-verify for a Compound

`step_import` self-verifies each member robustly watertight, not the merged mesh (two disjoint
solids share no faces, but per-member verification is the honest guarantee and localises a bad
member):

```cpp
bool robustlyWatertightMulti(const ntopo::Shape& s) {
  if (s.type() == ntopo::ShapeType::Compound) {
    bool any = false;
    for (ntopo::Explorer ex(s, ntopo::ShapeType::Solid); ex.more(); ex.next()) {
      any = true;
      if (!robustlyWatertight(ex.current())) return false;   // any leaky member → OCCT
    }
    return any;                       // empty compound → not accepted
  }
  return robustlyWatertight(s);       // Solid: unchanged path
}
```

`step_import` calls `robustlyWatertightMulti` instead of `robustlyWatertight`; a single Solid
takes the identical branch as before. Any failure (or a NULL parse) → OCCT `STEPControl_Reader`
re-reads the SAME file. No native void is handed to OCCT.

### 2.5 Verification (foreign OCCT-authored)

OCCT `STEPControl_Writer` authors a STEP file from a `TopoDS_Compound` of two disjoint OCCT
solids (e.g. two boxes, or a box + a cylinder). The native reader imports it as a Compound of
two native solids. The harness explores both the native compound and the OCCT re-import
(`STEPControl_Reader`) for their solids and asserts: same solid count (2), and per-solid
matched by centroid, volume / bbox within tolerance, each watertight + valid.

## 3. T3 — B-spline-FACE round-trip (deferred task 7.4), IF constructible

The reader's `B_SPLINE_SURFACE_WITH_KNOTS` (degreeU/V, row-major poles, RLE-expanded knotsU/V,
non-rational gate) and `B_SPLINE_CURVE_WITH_KNOTS` mapping are already implemented; the writer
emits them. 7.4 was deferred because no known native op produced a **watertight** solid whose
faces are `FaceSurface::Kind::BSpline` (a lone spline patch is an open shell — no enclosed
volume to round-trip, and the tessellator needs a closed 2-manifold to self-verify).

**T3 investigates, then chooses honestly:**

1. **Search existing native construct ops** (`src/native/construct/**` — loft / sweep / revolve
   and their residual builders) for a path that emits a CLOSED solid carrying at least one
   `FaceSurface::Kind::BSpline` face and, for its rims, `EdgeCurve::Kind::BSpline` edges — and
   that `step_can_export_native` accepts (non-rational; every face/edge kind in scope).
2. **IF such a solid is constructible** (native-built, not fabricated): add a host round-trip
   in `test_native_step_reader.cpp` — build it → `step_export_native` → `step_import_native` →
   tessellate → assert valid + watertight AND the B-spline face reconstructs the same degrees /
   row-major control grid / RLE-expanded knot vectors AND the volume matches within analytic
   tolerance (deflection-bounded, since a spline face meshes approximately). Mark 7.4 `[x]`.
3. **IF NO non-fabricated fixture exists**: record the honest skip in `tasks.md` and the spec
   note — the B_SPLINE mapping stays reviewed-but-unexercised-by-round-trip. **Do NOT** hand-
   build an artificial watertight bspline-face solid, and **do NOT** modify `step_writer.cpp`
   to synthesize one. A round-trip we cannot honestly source is not fabricated to hit a box.

The writer change is permitted by the task ONLY if T3 genuinely needs the writer to emit a
constructible **native-built** bspline-face solid; the preferred path is a foreign-OCCT fixture
or an existing native op, and any writer touch (if unavoidable) is additive + noted. This
design's default is: no writer change; report constructibility truthfully.

## 4. Architecture / OCCT boundary (unchanged)

```
cc_step_import (facade, unchanged)
   └─ active engine
        ├─ NativeEngine::step_import(path)
        │     ├─ step_import_native(path)  (src/native/exchange, OCCT-FREE) → topo::Shape
        │     │     • ELLIPSE edge → EdgeCurve::Kind::Ellipse (T1a)
        │     │     • ≥2 MANIFOLD_SOLID_BREP → Compound of solids (T2)
        │     │     • TOROIDAL_SURFACE / nested assembly / rational / non-mm → NULL (DECLINE)
        │     ├─ NULL → OCCT fall-through
        │     └─ Solid/Compound → robustlyWatertightMulti (per-member) → wrap native
        │             └─ any member fails → OCCT fall-through (labelled)
        └─ OCCT STEPControl_Reader (fallback + oracle + fixture author)
   cc_iges_* / cc_step_export → unchanged
```

`src/native/**` stays OCCT-free (grep-gated). `step_writer.cpp` (default) and the tessellator
are NOT modified. No `cc_*` ABI change; default engine stays OCCT (opt-in `cc_set_engine(1)`).

## 5. Honest scope (what still DECLINEs → OCCT)

- `TOROIDAL_SURFACE` (no native surface kind + tessellator immutable), `SURFACE_OF_REVOLUTION`,
  `TRIMMED_CURVE`, rational/weighted B-splines, `BEZIER` — DECLINE → OCCT.
- Nested product-structure assemblies (transform trees, mapped items, external refs) — DECLINE
  → OCCT (T2 handles only flat multiple co-equal root solids as a compound).
- AP242 / PMI / colours / names, non-mm units, non-manifold / unhealable B-reps — DECLINE.
- IGES import/export — stay OCCT `IGESControl_*`.
- #8 `drop-occt` stays blocked.

## 6. Cognitive complexity

T1a is one small `curve()` arm (mirror of `CIRCLE`, ≤ ~6). T2's `findManifoldBreps` +
`hasNestedAssembly` are guard/loop functions (≤ ~10); `build()`'s compound branch adds one
loop + a size check (≤ ~10 total). `robustlyWatertightMulti` is a small type-dispatch (≤ ~6).
All measured with the `cognitive-complexity` skill before archive; no function pushed into a
higher band.

## 7. Verification plan

- **Gate 1 (host, OCCT-free)** — `tests/native/test_native_step_reader.cpp`:
  - **T1a** an in-scope buffer with an `ELLIPSE('',#plc,a,b)` edge maps to
    `EdgeCurve::Kind::Ellipse` with `radius==a`, `minorRadius==b`, frame from `#plc`; a
    degenerate `ELLIPSE` (semi-axis ≤ 0) DECLINEs (NULL).
  - **T1b** a buffer with a `TOROIDAL_SURFACE` face returns NULL (DECLINE) — documented.
  - **T2** a two-root buffer imports as a `Compound` of two `Solid`s (count 2, each valid);
    a buffer containing a nested-assembly entity (e.g. `NEXT_ASSEMBLY_USAGE_OCCURRENCE`)
    DECLINEs (NULL); the existing SINGLE-solid buffers still return a Solid (no regression).
  - **T3** IF a native bspline-face solid is constructible: round-trip EXACT (degrees /
    row-major poles / RLE knots / volume within analytic tol); ELSE the skip is documented.
- **Gate 2 (sim vs OCCT, OCCT linked)** — `tests/sim/native_step_import_parity.mm` +
  `scripts/run-sim-native-step-import.sh` via `cc_*` under `cc_set_engine(1)`;
  `xcrun simctl list devices booted` first:
  - **T1** OCCT authors a foreign solid with an `ELLIPSE` edge; native import vs OCCT
    re-import agree (volume / watertight / valid within tol). `TOROIDAL_SURFACE` foreign file
    DECLINEs natively and imports via OCCT identical to `cc_set_engine(0)`.
  - **T2** OCCT authors a foreign 2-solid `TopoDS_Compound`; native import returns a compound
    whose per-solid centroid / volume / bbox / count match the OCCT re-import.
  - Own `main()`, on the `run-sim-suite.sh` SKIP list; default engine restored in teardown so
    the suite assertion count is unchanged.
- **Done** only when the relevant gates are green and every existing suite stays green at the
  OCCT default (prior import slice host round-trip + sim `[NIMPORT]` parity, STEP export,
  healing, SSI S1–S4, S5 native-pass, blends + #6/#7, marching, boolean, construct,
  tessellation, phase3 do NOT regress). Honestly reported: T1 adds ELLIPSE (torus stays OCCT),
  T2 adds multi-solid compounds (nested assemblies stay OCCT), T3 closes the bspline-face
  round-trip only if a non-fabricated fixture exists.
