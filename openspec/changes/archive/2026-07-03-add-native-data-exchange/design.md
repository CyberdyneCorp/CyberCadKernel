# Design — add-native-data-exchange

Narrow, honest slice of Phase 4 #7 (`native-exchange`): make **`cc_step_export` native**
for a native-built solid — serialize a `topology::Shape` solid to a valid ISO-10303-21
(STEP AP203) file, Part-42 B-rep entity graph, true mm — verified by an OCCT-read
round-trip. STEP import + IGES export/import stay OCCT. Clean-room from the STEP standard
(ISO 10303-21 / -42) + the `cc_step_export` contract; OCCT (`STEPControl_Writer`,
`STEPCAFControl`, `GeomToStep_*`, `TopoDSToStep_*`, `RWStepShape_*`) as reference ORACLE
only.

## 1. Why export is tractable and exactly verifiable (and import + IGES are not)

STEP export is a **projection of an owned B-rep onto a fixed, known entity schema**. The
native kernel already owns everything the file needs:

- the topology (vertices / edges / loops / faces / shell / solid) from #2
  `native-topology` (`src/native/topology`), enumerable and shared-node;
- the geometry attached to it — `Plane` / `Cylinder` / `Cone` / `Sphere` / `BSpline`
  surfaces, `Line` / `Circle` / `BSpline` curves — from #1 `native-math`
  (`src/native/math/elementary.h`, `bspline.*`), each already parametrized to match OCCT.

There is a **1:1 map** from these native geometry / topology kinds to Part-42 entities
(table §3), so export is a deterministic walk-and-emit with no numerical solving. And it
is **exactly verifiable**: OCCT `STEPControl_Reader` re-reads the file and the resulting
`TopoDS_Shape`'s volume / bbox / topology is compared to the source native solid — a
mechanical honesty bar.

**Import is the opposite.** A STEP file in the wild can carry hundreds of the Part-42 +
AP203/AP214 entity types, external references, and non-conforming constructs from dozens
of foreign CAD systems; a reader is a full ISO-10303-21 tokenizer + a whole-schema
entity-graph resolver + B-rep reconstruction (pcurves, sewing) + shape healing. That is a
large, long-lived effort with no closed form — it stays OCCT `STEPControl_Reader`.
**IGES** is a second, older, entity-numbered format with its own writer and parser and its
own quirks — a separate large effort — and stays OCCT `IGESControl_*`. Attempting either
now would be high-effort and high-faking-risk; export-only is the honest achievable
ceiling.

## 2. ISO-10303-21 file skeleton (what the writer emits)

A Part-21 exchange file is text:

```
ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('CyberCadKernel native STEP AP203'),'2;1');
FILE_NAME('<path>','<utc-timestamp>',(''),(''),'CyberCadKernel','','');
FILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));          /* AP203 */
ENDSEC;
DATA;
#1 = APPLICATION_CONTEXT('configuration controlled 3D designs ...');
... product / context wrapper ...
#k = MANIFOLD_SOLID_BREP('', #shell);
#k+1 = ADVANCED_BREP_SHAPE_REPRESENTATION('', (#k), #context);
ENDSEC;
END-ISO-10303-21;
```

Each `#n = ENTITY(args);` is one instance record; args reference other records by `#m`.
The writer emits records **bottom-up** so every reference is already defined when used
(no forward reference), which also matches how the entity ids are assigned during the
single topology walk.

## 3. Native → Part-42 entity map (the core of the change)

| Native (`src/native/topology`, `src/native/math`) | Part-42 / AP203 entity emitted | Notes |
|---|---|---|
| `math::Point3` (vertex position, curve/surface origin, B-spline pole) in mm | `CARTESIAN_POINT((x,y,z))` | deduplicated by value+id |
| `math::Dir3` (axis / ref direction) | `DIRECTION((dx,dy,dz))` | unit vector |
| `math::Ax3` frame (origin, X, Z) | `AXIS2_PLACEMENT_3D(#origin, #axisZ, #refX)` | placement for every analytic geom |
| `topology` `Vertex` | `VERTEX_POINT('', #cartesianPoint)` | shared across incident edges |
| `EdgeCurve::Kind::Line` | `LINE(#pnt, #vector)` where `VECTOR(#direction, 1.0)` | direction from the edge frame |
| `EdgeCurve::Kind::Circle` | `CIRCLE(#axis2placement3d, radius)` | radius in mm |
| `EdgeCurve::Kind::BSpline` | `B_SPLINE_CURVE_WITH_KNOTS(deg, (#poles...), .UNSPECIFIED., F, F, (mults), (knots), .UNSPECIFIED.)` | rational ⇒ wrap with weights (`(...RATIONAL_B_SPLINE...)`) |
| `EdgeCurve::Kind::Ellipse` / `Bezier` (not down-converted) | — (DECLINE) | outside the emitted subset → empty buffer |
| topology `Edge` (curve + two vertices) | `EDGE_CURVE('', #v1, #v2, #curve, .T.)` | `.T.` same-sense; one per shared edge node |
| edge use in a wire (with orientation) | `ORIENTED_EDGE('', *, *, #edgeCurve, .T./.F.)` | orientation from native `Orientation` |
| topology `Wire` (edge loop) | `EDGE_LOOP('', (#orientedEdge...))` | |
| outer wire of a face / hole wire | `FACE_OUTER_BOUND('', #edgeLoop, .T.)` / `FACE_BOUND(...)` | index 0 child = outer (see `shape.h`) |
| `FaceSurface::Kind::Plane` | `PLANE(#axis2placement3d)` | |
| `FaceSurface::Kind::Cylinder` | `CYLINDRICAL_SURFACE(#axis2placement3d, radius)` | |
| `FaceSurface::Kind::Cone` | `CONICAL_SURFACE(#axis2placement3d, radius, semiAngle)` | |
| `FaceSurface::Kind::Sphere` | `SPHERICAL_SURFACE(#axis2placement3d, radius)` | |
| `FaceSurface::Kind::BSpline` | `B_SPLINE_SURFACE_WITH_KNOTS(...)` | rational ⇒ weight-wrapped |
| topology `Face` (surface + bounds + sense) | `ADVANCED_FACE('', (#bound...), #surface, .T./.F.)` | sense from native face orientation |
| topology `Shell` (closed) | `CLOSED_SHELL('', (#advancedFace...))` | |
| topology `Solid` | `MANIFOLD_SOLID_BREP('', #closedShell)` | exactly one |
| the solid, bound to a mm context | `ADVANCED_BREP_SHAPE_REPRESENTATION('', (#brep), #geomRepContext)` | + product/context wrapper §4 |

The frame / parametrization conventions already match OCCT (`elementary.h` mirrors
`ElSLib`; B-splines mirror `BSplCLib`/`BSplSLib`), so the emitted placements and knot
vectors read back to the same geometry — the round-trip gate proves it.

## 4. Units + product/context wrapper (true millimetres)

`cc_*` is a millimetre kernel. The writer emits the AP203 unit context with an explicit
`SI_UNIT(.MILLI.,.METRE.)` length unit (plus `.RADIAN.` plane-angle and steradian
solid-angle units), an `UNCERTAINTY_MEASURE_WITH_UNIT` (linear tolerance), a
`GEOMETRIC_REPRESENTATION_CONTEXT(3)` combined with those units
(`(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNIT_ASSIGNED_CONTEXT((...)) ...)`), and the
minimal product spine `APPLICATION_CONTEXT` → `APPLICATION_PROTOCOL_DEFINITION` →
`PRODUCT` / `PRODUCT_DEFINITION_FORMATION` / `PRODUCT_DEFINITION` →
`PRODUCT_DEFINITION_SHAPE` so the shape representation is a valid AP203 product shape.
Coordinates are written directly in mm (no scaling), so an OCCT read (which honours the mm
context) reconstructs the same-size solid.

## 5. Representability gate (what the native writer accepts)

The native writer accepts ONLY:

- a native `Solid` containing at least one `Shell` that is **closed / manifold** (each
  edge used by exactly two faces of the shell — the native watertight invariant already
  enforced by the tessellate/boolean self-verify);
- faces whose `FaceSurface::kind` ∈ {`Plane`, `Cylinder`, `Cone`, `Sphere`, `BSpline`};
- edges whose `EdgeCurve::kind` ∈ {`Line`, `Circle`, `BSpline`};
- finite, non-degenerate geometry.

Anything else — an `Ellipse` edge, a `Bezier` surface not down-converted to a
`B_SPLINE_SURFACE_WITH_KNOTS`, an open / non-manifold shell, a degenerate face, a
zero-length edge — makes the writer **return an empty buffer** (DECLINE), and the engine
falls through to OCCT `STEPControl_Writer`. The writer never emits a partial / lossy file.

## 6. The mandatory OCCT-read round-trip self-check (the honesty gate)

This is the load-bearing correctness rule. After the native writer produces a buffer and
`step_export` writes it to `path`, the engine (behind `CYBERCAD_HAS_OCCT`) **re-reads the
file with OCCT `STEPControl_Reader`** and compares the reconstructed `TopoDS_Shape` to the
source native solid:

- the read must succeed and yield a **valid** shape (`BRepCheck_Analyzer` OK, one solid);
- **volume** within a relative tolerance (curved faces are analytic, so this is ~fp-exact
  for planar / analytic solids; a small band absorbs OCCT's own tessellation of the
  volume integral);
- **bounding box** corners within a linear tolerance;
- **sub-shape counts / topology** consistent (same face / edge / vertex cardinality up to
  OCCT's shared / periodic representation, as already characterised in #4's "representational
  difference" note — the SOLID must be geometrically identical, not bit-identical topology).

If the read fails, the shape is invalid, or any comparison exceeds tolerance, the engine
**DISCARDS** the native file and re-exports with OCCT `STEPControl_Writer`. The kernel
NEVER leaves a native STEP file on disk that reads back as a different or invalid solid.
On the host (no OCCT) the round-trip check cannot run, so host tests assert the buffer
STRUCTURALLY (§ gate 1) and the round-trip is the simulator gate.

## 7. Architecture / OCCT boundary

```
cc_step_export (facade, unchanged)
   └─ active engine
        ├─ NativeEngine::step_export
        │     ├─ isNative(body)?  ── no ─┐
        │     ├─ native writer (src/native/exchange, OCCT-FREE) → buffer
        │     │     └─ empty (DECLINE) ──┤
        │     ├─ write buffer to path
        │     ├─ OCCT round-trip self-check (STEPControl_Reader)
        │     │     └─ fail ──────────────┤
        │     └─ ok → return 1            │
        │                                 ▼
        └─ OCCT STEPControl_Writer fall-through (labelled)
   cc_step_import / cc_iges_export / cc_iges_import → unchanged (OCCT)
```

- `src/native/exchange/` — OCCT-FREE, host-buildable
  (`/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`); includes only `src/native/math` +
  `src/native/topology`; emits a `std::string` buffer; references no OCCT / `IEngine` /
  `EngineShape`. Suggested files: `step_text.h` (real-number / list / record formatting to
  ISO-10303-21 lexical rules), `step_entities.h` (Part-42 entity emitters + the `#n` id
  allocator / dedup map), `step_writer.h` (the topology walk + representability gate +
  header/context assembly), umbrella `native_exchange.h`.
- `src/engine/native/native_engine.cpp` — `step_export` native-else-fallback + the OCCT
  round-trip self-check (behind `CYBERCAD_HAS_OCCT`). `step_import` / `iges_export` /
  `iges_import` untouched.
- **No ABI change**; default engine stays OCCT (opt-in via `cc_set_engine(1)`).

## 8. Native-void constraint (consistency with #4–#6)

As in the boolean / blend slices, a native body's B-rep is a `topology::Shape` that OCCT
cannot read directly. That is fine here: the native writer consumes the NATIVE shape (not
an OCCT handle) and produces a FILE, and the round-trip self-check reads that FILE back
through OCCT (a legitimate OCCT input) — it never hands a native void to OCCT. The
fall-through path (foreign body, DECLINE, or failed self-check) is the only route that
calls OCCT `STEPControl_Writer`, and it does so on an OCCT body (foreign) or by having
OCCT re-derive from... — for a native body that DECLINEs, the engine has no OCCT B-rep to
give `STEPControl_Writer`, so that specific case reports an honest failure (`0`) rather
than a faked file, exactly as the boolean slice reports an honest error for a
native-native discard. (A native solid that is representable is the success path; a native
solid that is NOT representable is by construction outside what a native serializer can
honestly emit — the honest outcome is `0`, not a lossy file.)

## 9. Cognitive complexity

The entity emitters are flat formatters (each ≤ ~8). The topology walk with dedup is the
one loop-nested function; it is kept in the systems band (≤ ~20) by pushing each entity
kind into its own emitter and using a single `#n` id map. The round-trip self-check is a
guard sequence (≤ ~12). All flagged and measured with the `cognitive-complexity` skill
before archive.

## 10. Verification plan

- **Gate 1 (host, no OCCT)** — `tests/test_native_exchange.cpp`: emit the buffer for a
  native box / cylinder / (down-convertible) B-spline solid and assert structurally — file
  framing (`ISO-10303-21;` … `END-ISO-10303-21;`), AP203 `FILE_SCHEMA`, mm `SI_UNIT`,
  exactly one `MANIFOLD_SOLID_BREP` → one `CLOSED_SHELL`, an `ADVANCED_FACE` per native
  face with the correct surface entity, `EDGE_CURVE`/`ORIENTED_EDGE`/`EDGE_LOOP` with the
  right curve entities, every `#n` reference resolvable (no dangling), coordinates in mm;
  plus DECLINE cases (`Ellipse` edge, non-manifold shell, unrepresentable surface → empty
  buffer). Facade cases in `tests/test_native_engine.cpp` for the native success + the
  fall-through.
- **Gate 2 (sim native-write / OCCT-read round-trip)** —
  `tests/sim/native_exchange_parity.mm` + `scripts/run-sim-native-exchange.sh` through the
  `cc_*` facade under `cc_set_engine(1)`: export a native solid, re-read with OCCT
  `STEPControl_Reader`, assert volume / bbox / topology within tolerance vs the source and
  a valid `BRepCheck`, with OCCT `STEPControl_Writer` (`cc_set_engine(0)`) as oracle;
  assert the fall-through cases (foreign / unrepresentable) and `cc_step_import` /
  `cc_iges_*` identical under both engines. Own `main()` on the `run-sim-suite.sh` SKIP
  list; default restored in teardown so the 221-assertion count is unchanged.
- **Done** only when both gates are green and every existing suite stays green at the OCCT
  default. Honestly reported as the export-only native ceiling; STEP import + IGES remain
  OCCT and #8 `drop-occt` stays blocked.
