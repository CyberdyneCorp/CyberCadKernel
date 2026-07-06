# Design — add-native-step-import

Narrow, honest slice of Phase 4 #7 (`native-exchange`): make **`cc_step_import` native**
for the AP203 `MANIFOLD_SOLID_BREP` subset the native writer (and OCCT) emit for planar +
elementary-quadric + basic B-spline solids — parse an ISO-10303-21 (STEP Part 21) file and
reconstruct a native `topology::Shape` Solid, closing the file's sub-tolerance gaps with
`heal::healShell`, verified by a native export→import round-trip and by parity vs OCCT
`STEPControl_Reader`. Arbitrary / AP242 / assembly / IGES import stay OCCT. Clean-room from
the STEP standard (ISO 10303-21 / -42) + `src/native/exchange/step_writer.cpp` (the exact
entity set to invert); OCCT (`STEPControl_Reader`, `RWStep*`, `StepToTopoDS_*`) as reference
ORACLE + engine-side fallback only.

## 1. Why THIS subset is tractable (and arbitrary import is not)

The native writer emits a **fixed, known entity graph** (`step_writer.cpp`): a single
`MANIFOLD_SOLID_BREP` → one `CLOSED_SHELL` → N `ADVANCED_FACE`, each over one of five
surface kinds, each bounded by `EDGE_LOOP`s of `ORIENTED_EDGE`→`EDGE_CURVE` over three
curve kinds, with `VERTEX_POINT`/`CARTESIAN_POINT`/`DIRECTION`/`AXIS2_PLACEMENT_3D` leaves
and the AP203 product/context wrapper. Reading **that** subset back is the deterministic
inverse of the writer's §3 map — no whole-schema resolver, no foreign healing beyond the
sub-tolerance sew the file inherently needs. OCCT `STEPControl_Writer` emits the *same*
subset for the same solids (the writer was built to match it), so a foreign OCCT-written
STEP of a box/cylinder is in scope too.

**Arbitrary import is the opposite** — hundreds of Part-42 + AP203/AP214 + AP242 entity
types, external references, PMI, assemblies, non-conforming files from many CAD systems,
pcurve/sewing reconstruction, freeform re-approximation. That stays OCCT. This slice reads
the emitted subset and DECLINEs (NULL → OCCT) on anything outside it.

## 2. ISO-10303-21 file skeleton (what the parser consumes)

A Part-21 file is text (see `step_writer.cpp::writeHeader`/`writeStepString`):

```
ISO-10303-21;
HEADER;  FILE_DESCRIPTION(...); FILE_NAME(...); FILE_SCHEMA(('CONFIG_CONTROL_DESIGN')); ENDSEC;
DATA;
#1 = CARTESIAN_POINT('',(0.,0.,0.));
...
#k = MANIFOLD_SOLID_BREP('CyberCadKernel_part',#shell);
#k+1 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );   /* combined instance */
...
#m = ADVANCED_BREP_SHAPE_REPRESENTATION('',(#k),#ctx);
ENDSEC;
END-ISO-10303-21;
```

The parser reads ONLY the DATA section into a `map<int, Record>`; the HEADER is skipped
(schema is not enforced — a `FILE_SCHEMA` that names a config-control / AP203 / AP214
schema is accepted; a schema that clearly is not is a soft DECLINE signal but the entity
scope check is the real gate). Coordinates are read as-is in **millimetres** (the writer's
unit context is `SI_UNIT(.MILLI.,.METRE.)`; we do not currently apply a non-mm scale — a
non-mm length unit context ⇒ DECLINE so we never silently mis-scale).

## 3. Part 21 tokenizer + entity table (`step_parser.h/.cpp`)

A record is `#N = KEYWORD ( arg , arg , ... ) ;`. The tokenizer produces, per record, a
`Record{ int id; std::string keyword; std::vector<Arg> args; }` where:

```cpp
struct Arg {                       // one STEP parameter value
  enum class Kind { Ref, Int, Real, Str, Enum, List, Null, Derived } kind;
  int ref = 0;                     // Kind::Ref  → #M
  long long i = 0;                 // Kind::Int
  double r = 0.0;                  // Kind::Real
  std::string s;                   // Kind::Str (unquoted) / Kind::Enum (.T. etc.)
  std::vector<Arg> list;           // Kind::List
};
```

Lexical rules handled (from ISO 10303-21, cross-checked against `step_writer.cpp`'s output):

| Token | Rule | Writer example |
|---|---|---|
| Ref | `#` digits | `#42` |
| Real | optional sign, digits, `.`, optional frac, optional `E±exp`; **typed forms** `1.`, `1.E2`, `-3.5E-07` | `1.E-07`, `0.` |
| Int | digits with NO `.`/`E` | degree `3`, mult `2` |
| Str | `'...'`, embedded quote doubled `''` → `'` | `'CyberCadKernel_part'` |
| Enum | `.` name `.` | `.T.`, `.F.`, `.MILLI.`, `.UNSPECIFIED.` |
| List | `( ... )` nested | poles, knots, `(#a,#b)` |
| Null | `$` | optional AXIS2 fields |
| Derived | `*` | `ORIENTED_EDGE('',*,*,#ec,.T.)` |
| Combined instance | `( SUB(...) SUB(...) )` at record top level | the unit/context entities |

The tokenizer is a flat char-scanner (skip whitespace/comments `/* */`); each value form is
one small sub-parser (systems band, isolated). A **combined-instance** record (top-level
`( ... )` with multiple `KEYWORD(...)` groups) is stored with keyword `""` and a list of
sub-records — the mapper only needs its `SI_UNIT` for the unit check, so it scans the
sub-records for `SI_UNIT`. Unknown keywords are stored verbatim; a record that is never
referenced by an in-scope entity is simply ignored, and a *referenced* unknown ⇒ DECLINE.

Cognitive-complexity note: the value-form dispatch is an irreducible tokenizer switch
(systems band ≤ ~25, documented, Visitor-free); each sub-parser stays ≤ ~10.

## 4. STEP → native entity map (the inverse of the writer §3)

Two passes over the entity table, memoising by `#id`.

**Pass A — leaf geometry** (`resolvePoint/Dir/Placement/Curve/Surface`):

| STEP entity (as `step_writer.cpp` emits) | Native (`src/native/{math,topology}`) | Notes |
|---|---|---|
| `CARTESIAN_POINT('',(x,y,z))` | `math::Point3{x,y,z}` (mm) | memoised by `#id` |
| `DIRECTION('',(dx,dy,dz))` | `math::Dir3` | |
| `AXIS2_PLACEMENT_3D('',#o,#z,#x)` | `math::Ax3{origin, x, y=z×x, z}` | Y re-derived from Z×X, mirroring `worldFrame`; `$` axis/ref ⇒ default frame |
| `LINE('',#p,#v)` where `VECTOR('',#d,mag)` | `EdgeCurve{Line, frame{origin=p, X=d}}` | inverse of `lineCurve` |
| `CIRCLE('',#plc,r)` | `EdgeCurve{Circle, frame=plc, radius=r}` | inverse of `circleCurve` |
| `B_SPLINE_CURVE_WITH_KNOTS(deg,(#poles),.UNSPECIFIED.,.F.,.F.,(mults),(knots),.UNSPECIFIED.)` | `EdgeCurve{BSpline, degree, poles, knots=RLE-expand(mults,knots)}` | non-rational only; a `RATIONAL_*` wrap ⇒ DECLINE |
| `PLANE('',#plc)` | `FaceSurface{Plane, frame=plc}` | |
| `CYLINDRICAL_SURFACE('',#plc,r)` | `FaceSurface{Cylinder, frame, radius=r}` | |
| `CONICAL_SURFACE('',#plc,r,semiAngle)` | `FaceSurface{Cone, frame, radius, semiAngle}` | inverse of `coneSurface` |
| `SPHERICAL_SURFACE('',#plc,r)` | `FaceSurface{Sphere, frame, radius}` | |
| `B_SPLINE_SURFACE_WITH_KNOTS(dU,dV,(( #pts )),...,(mU),(mV),(kU),(kV),...)` | `FaceSurface{BSpline, degreeU/V, nPolesU/V, poles row-major, knotsU/V=RLE-expand}` | inverse of `bsplineSurface`; non-rational only |
| any other surface/curve keyword (`TOROIDAL_SURFACE`, `ELLIPSE`, `SURFACE_OF_REVOLUTION`, rational wrap) | — (DECLINE → NULL) | outside the writer subset |

Knot RLE expansion inverts `compressKnots`: `(mults)=(2,1,1,2)`, `(knots)=(0.,.33,.66,1.)`
→ flat `[0,0,.33,.66,1,1]`. Poles for a surface come as a list-of-U-rows each a list of V
`#CARTESIAN_POINT` refs (row-major, U outer) — exactly `poleNet`'s layout.

**Pass B — topology** (follow refs, dedup by `#id`):

| STEP entity | Native builder | Notes |
|---|---|---|
| `VERTEX_POINT('',#cp)` | `ShapeBuilder::makeVertex(point[#cp])` | memoised by `#id` |
| `EDGE_CURVE('',#v0,#v1,#crv,.T.)` | `makeEdgeWithVertices(curve[#crv], first,last, {vtx[#v0], vtx[#v1]})` | ONE edge per `#id` (writer already shares); param range from projecting the two vertices onto the curve |
| `ORIENTED_EDGE('',*,*,#ec,.T./.F.)` | `edge[#ec].oriented(Forward/Reversed)` | the shared edge, oriented |
| `EDGE_LOOP('',(#oe...))` | `makeWire({oriented edges})` | **seam edges dropped** (see §5) |
| `FACE_OUTER_BOUND('',#loop,.T.)` | outer wire (child 0) | orientation flag folded into the wire |
| `FACE_BOUND('',#loop,.T.)` | hole wire | |
| `ADVANCED_FACE('',(#bounds),#srf,.T./.F.)` | `makeFace(surface[#srf], outer, holes, sense)` | sense `.F.` ⇒ `Orientation::Reversed` |
| `CLOSED_SHELL('',(#faces))` | collected faces | fed to `makeShell` after heal-sew |
| `MANIFOLD_SOLID_BREP('',#shell)` | the root solid candidate | |
| `ADVANCED_BREP_SHAPE_REPRESENTATION('',(#brep),#ctx)` | the ROOT | exactly one; else DECLINE |

The frame / parametrisation conventions already match (the writer used `elementary.h` /
`bspline`), so a reconstructed placement + knot vector re-creates the same geometry — the
round-trip gate proves it.

## 5. The periodic-wall SEAM edge (a writer artefact, not a physical edge)

For a full-turn cylinder/cone/sphere wall the writer synthesises a straight `LINE`
`EDGE_CURVE` used **once forward at u=period and once reversed at u=0** to close the
periodic parametric region (`wallLoopWithSeam` / `seamEdgeCurve`). This seam is a
parametric closure artefact, not a physical edge of the solid. The reader detects it — an
`EDGE_LOOP` whose edges include one `EDGE_CURVE` referenced by two `ORIENTED_EDGE`s of
OPPOSITE sense (forward + reversed) within the same loop — and DROPs the seam pair from the
reconstructed wire, so the native face is bounded only by its real rim circles, exactly as
the native writer's *source* solid was (native solids defer edge-node sharing and carry no
seam). This keeps the round-trip edge count exact against the native original. (When OCCT
re-reads the same file it *keeps* the seam as an edge — the +2-edge "bounded superset" the
export slice's Gate 2 already documented; the sim-vs-OCCT gate compares volume/watertight,
not raw edge count, so both are consistent.)

## 6. Post-import heal (`heal::healShell`) — closing the sub-tolerance gaps

STEP writes each face's boundary independently at fp precision, so two faces meeting at a
physical edge carry two `EDGE_CURVE`s whose endpoints are coincident only within a small
tolerance (even the native writer's geometric edge-dedup keys on rounded coordinates, and a
foreign OCCT file shares nothing). The reconstructed shell is therefore a **face soup / open
shell** exactly of the family `add-native-shape-healing` handles. After Pass B the reader
runs `heal::healShell(candidateSolid, HealOptions{tolerance})`:

- **`Healed`** — `healShell` unified the near-coincident vertices, tolerant-sewed the
  coincident edge sides into shared nodes, fixed orientation, assembled a shell/solid, and
  self-verified watertight + enclosed volume > 0. The reader returns `result.shape` (the
  watertight solid).
- **`Unhealed`** — a gap beyond tolerance, a genuinely open shell, a non-manifold input, or
  a self-verify failure. The reader returns a **NULL Shape** (DECLINE → OCCT). The tolerance
  is NEVER widened to force a pass; the honest residual is reported (logged), not hidden.

This is the same deferral seam healing already defines: an `Unhealed` import is routed to
OCCT `STEPControl_Reader`, which runs its own `ShapeFix`.

## 7. Root selection + DECLINE (honest scope gate)

The reader DECLINEs (returns NULL Shape) — and the engine falls through to OCCT — when ANY
of:

- no `ADVANCED_BREP_SHAPE_REPRESENTATION` / `MANIFOLD_SOLID_BREP`, or **more than one** root
  solid (assembly / multi-root out of scope);
- a referenced entity has an unknown/unsupported **keyword** (a Part-42 entity we do not
  model), or a **surface/curve kind** outside {Plane/Cylinder/Cone/Sphere/BSpline} /
  {Line/Circle/BSpline}, or a **rational (weighted)** B-spline wrap;
- a non-mm length unit context (we do not silently rescale);
- a malformed record (dangling `#ref`, arity mismatch, parse error);
- `healShell` returns `Unhealed`.

No partial / invented solid is ever returned. **We parse only what is in the file and map
only what we genuinely support.**

## 8. Architecture / OCCT boundary

```
cc_step_import (facade, unchanged)
   └─ active engine
        ├─ NativeEngine::step_import(path)
        │     ├─ step_import_native(path)  (src/native/exchange, OCCT-FREE) → topo::Shape
        │     │     ├─ NULL (DECLINE) ───────────────────┐
        │     │     └─ solid → self-verify (valid watertight, vol>0)
        │     │             └─ fail ───────────────────────┤
        │     └─ wrap native EngineShape → return          │
        │                                                  ▼
        └─ OCCT STEPControl_Reader fall-through (labelled)
   cc_iges_import / cc_iges_export / cc_step_export → unchanged
```

- `src/native/exchange/` — OCCT-FREE, host-buildable
  (`/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`); includes only `src/native/{math,
  topology,heal}`; consumes a path/text and returns a `topo::Shape`; references no OCCT /
  `IEngine` / `EngineShape`. New files: `step_parser.h/.cpp` (tokenizer + entity table),
  `step_reader.h/.cpp` (the two-pass map + seam drop + `healShell` + root/DECLINE gate),
  extending `native_exchange.h`.
- `src/engine/native/native_engine.cpp` — `step_import` native-else-fallback + the
  self-verify. `iges_export` / `iges_import` / `step_export` untouched. The STEP **writer**
  (`step_writer.cpp`) and the tessellator are NOT modified.
- **No ABI change**; default engine stays OCCT (opt-in via `cc_set_engine(1)`).

## 9. Native-void constraint (consistency with #4–#7 + the export slice)

The native reader consumes a FILE (a legitimate input) and produces a NATIVE
`topology::Shape` — it never hands a native void to OCCT and never reads an OCCT handle. The
fall-through path (DECLINE / failed self-verify) re-reads the SAME file with OCCT
`STEPControl_Reader` from scratch (a file, not a native void), so the boundary is clean. A
foreign OCCT-written STEP is likewise just a file to the native reader.

## 10. Cognitive complexity

The tokenizer's value-form dispatch is the one systems-band function (irreducible lexer
switch, ≤ ~25, documented). The two-pass mapper is split into small per-entity resolvers
(`resolvePoint/Dir/Placement/Curve/Surface`, `buildVertex/Edge/Loop/Face`), each ≤ ~12,
with a single `#id`→native memo map — mirroring how the writer's walk split into small
methods. The DECLINE gate is a guard sequence (≤ ~12). All flagged and measured with the
`cognitive-complexity` skill before archive.

## 11. Verification plan

- **Gate 1 (host round-trip, no OCCT)** — `tests/native/test_native_step_reader.cpp`:
  (a) **tokenizer** unit cases — typed reals (`1.`, `1.E2`, `-3.5E-07`), strings with `''`,
  enums, `$`, `*`, nested lists, combined instances; a malformed record → parse error.
  (b) **round-trip EXACT** — a native box (planar), a native cylinder + capped cylinder
  (elementary quadric), a holed/typed-profile solid, and (if the writer's B-spline export is
  exercised) a spline-face solid: `step_export_native` → `step_import_native` → tessellate →
  the imported solid is valid + watertight with volume / bbox / face+edge+vertex counts /
  topology matching the original EXACTLY (both ends native, seam dropped so edge count is
  exact). (c) **DECLINE** — an unsupported surface keyword (`TOROIDAL_SURFACE`), a
  multi-root file (two `MANIFOLD_SOLID_BREP`), a rational-spline wrap, and a deliberately
  gapped B-rep `healShell` cannot close → each returns a NULL Shape. Facade case in
  `tests/test_native_engine.cpp` (`native_step_import_reads_native_file`): native
  `cc_step_import` of a native-written file returns a valid body; an unsupported file falls
  through.
- **Gate 2 (sim vs OCCT)** — `tests/sim/native_step_import_parity.mm` +
  `scripts/run-sim-native-step-import.sh` through the `cc_*` facade under `cc_set_engine(1)`:
  (a) import a file the native writer produced with the native reader and with OCCT
  `STEPControl_Reader`; assert both are valid watertight solids with volume / bbox within
  tolerance. (b) FOREIGN STEP: `STEPControl_Writer` writes a box/cylinder STEP from an OCCT
  solid; import it NATIVELY and compare vs the OCCT re-import (volume / watertight / valid
  within tol). (c) fall-through: an assembly / unsupported-surface file is DECLINEd natively
  and imported by OCCT identical to `cc_set_engine(0)`. `xcrun simctl list devices booted`
  first; own `main()` on the `run-sim-suite.sh` SKIP list; default restored in teardown so
  the suite assertion count is unchanged.
- **Done** only when both gates are green and every existing suite stays green at the OCCT
  default (export slice, healing, SSI S1–S4, S5 native-pass=6, native blends + #6/#7,
  marching, boolean, construct, tessellation, phase3 do NOT regress). Honestly reported as
  the first native import slice (the writer-emitted AP203 manifold-solid-brep subset);
  arbitrary / AP242 / IGES import remain OCCT and #8 `drop-occt` stays blocked.
