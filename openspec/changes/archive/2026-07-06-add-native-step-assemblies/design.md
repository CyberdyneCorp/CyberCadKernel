# Design — add-native-step-assemblies

Widen the WORKING native STEP reader (`src/native/exchange/step_reader.{h,cpp}`, the archived
`add-native-step-import` → `widen-native-step-import` slices) from a **flat** multi-solid
`Compound` to a **placed** `Compound`: parse the STEP assembly transform structure, compose
each component's **rigid** placement into a native `topology::Location`, apply it per component
`MANIFOLD_SOLID_BREP` via `Shape::located()`, and return the placed compound. Also confirm +
regression-pin **AP214/AP242 `FILE_SCHEMA`** acceptance (the reader was already
schema-independent). Map ONLY onto geometry + placements the file genuinely carries and that
the native topology genuinely models; DECLINE (NULL → OCCT) otherwise. Clean-room from
ISO 10303-21/-42/-43 + the existing reader; OCCT (`STEPControl_Reader` / `STEPControl_Writer` /
`STEPCAFControl_Writer`) is the ORACLE + fixture-author + fallback only. NOT a general
product-structure importer; deep-nested / non-rigid / external-ref assemblies stay OCCT.

## 0. What the reader already does (the substrate this widens)

Two passes over a `map<#id, Record>`: Pass A resolves leaf geometry (`point` / `direction` /
`axis2placement` → `math::Ax3` / `curve` / `surface`), Pass B builds topology
(`VERTEX_POINT`→vertex, `EDGE_CURVE`→one shared edge, `ORIENTED_EDGE`→oriented, `EDGE_LOOP`→
wire, `ADVANCED_FACE`→face, `CLOSED_SHELL`→shell, `MANIFOLD_SOLID_BREP`→solid).
`build()` gates on `validateUnitContext` (mm only), then **`if (hasNestedAssembly()) return
{};`** (the blanket transform-tree decline this change lifts), then `findManifoldBreps()`
(all roots) → one `Solid` or a flat `Compound`. The engine (`native_engine.cpp::step_import`)
keeps the result native ONLY when `robustlyWatertightImport` passes (per-member for a
Compound); else it falls to OCCT. **The reader NEVER reads or gates on the `FILE_SCHEMA`
header** — it enters at `DATA;`. This change adds an assembly pass, has `build()` call it
instead of the blanket decline, and pins the (already-true) schema-independence — nothing
else in the reader changes.

## 1. The native capability this maps onto (checked)

| STEP concept | Native construct | Exists? | Evidence |
|---|---|---|---|
| A rigid instance placement (rotation + translation) | `topology::Location{math::Transform{Mat3 rot, Vec3 t}}` applied by `Shape::located(loc)` | **YES** | `shape.h` `Location` wraps `math::Transform`; `located()` composes it onto a shared node; `accessors.h` (`pointOf`/`curveOf`/`surfaceOf`) resolves WORLD coords via the composed Location — full `TopLoc_Location` semantics |
| A placed component solid that must self-verify watertight | `Explorer(Solid)` over the compound → `robustlyWatertight` per member | **YES** | `native_engine.cpp::robustlyWatertightImport` already explores a `Compound`'s solids; a rigid transform preserves the watertight 2-manifold, so a placed solid meshes + verifies unchanged (no tessellator arm needed) |
| A composed placement chain | `Location::composedWith` / `Transform::composedWith` | **YES** | `shape.h` / `transform.h` — exact fp64 affine composition |
| A rigid-only guard | orthonormal `Mat3` + det ≈ +1 | **derived** | `transform.h` `Mat3` has `determinant()`; orthonormality is `M·Mᵀ ≈ I` — a tiny predicate |

The topology is a placed-instance kernel already; the ONLY new work is **parsing the STEP
transform tree and composing each component's rigid `Transform`** — the placement then rides
the existing located-node + per-member-verify machinery.

## 2. STEP assembly transform structure (ISO 10303-21/-43/AP203-AP242)

An assembly places a **component representation** into a **parent representation** by a rigid
transform. OCCT's `STEPControl_Writer`/`STEPCAFControl_Writer` emit one of two equivalent
forms; the reader handles both. The **exact keyword strings + arg orders are confirmed against
an OCCT-authored fixture Diagnose (task 1.1)** before the arms are finalised; the shapes below
are the ISO forms.

### 2.1 Form A — `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION` + `ITEM_DEFINED_TRANSFORMATION`

```
#a = NEXT_ASSEMBLY_USAGE_OCCURRENCE('','','',#parentPD,#childPD,$);
#c = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#rel,#pds);
#rel = ( REPRESENTATION_RELATIONSHIP('','',#childRep,#parentRep)
         REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#idt)
         SHAPE_REPRESENTATION_RELATIONSHIP() );
#idt = ITEM_DEFINED_TRANSFORMATION('','',#fromAx2,#toAx2);
#fromAx2 = AXIS2_PLACEMENT_3D('',#p,#z,#x);   // datum in the CHILD rep frame
#toAx2   = AXIS2_PLACEMENT_3D('',#p',#z',#x'); // datum in the PARENT rep frame
```

The rigid placement that takes a point expressed in the **child** representation into the
**parent** (world) representation is the map carrying the `#from` frame onto the `#to` frame:

```
T = frameToWorld(toAx2) ∘ frameToWorld(fromAx2)⁻¹
```

where `frameToWorld(ax3)` is the affine `{rotation = [X | Y | Z] columns, translation =
origin}` built from the orthonormal `math::Ax3` (the reader already resolves an
`AXIS2_PLACEMENT_3D` → `Ax3` via `axis2placement`). Composition + inverse are `Transform`
methods (exact fp64). In the common OCCT case `#from` is the identity placement at the origin,
so `T = frameToWorld(toAx2)` — a plain rotation+translation.

### 2.2 Form B — `MAPPED_ITEM` + `REPRESENTATION_MAP`

```
#mi  = MAPPED_ITEM('',#map,#targetAx2);
#map = REPRESENTATION_MAP(#originAx2,#childRep);
```

The placement is `T = frameToWorld(targetAx2) ∘ frameToWorld(originAx2)⁻¹`, composed with the
child representation's own placement (identity for a plain mapped solid). Same rigid gate,
same `frameToWorld` helper.

### 2.3 Associating a placement with the component's solids

`nextAssemblyPlacements()` builds `map<componentRepId, Transform>`:
1. For each `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` / `MAPPED_ITEM`, resolve `(childRep, T)`
   via §2.1/§2.2.
2. For each `childRep` (a `SHAPE_REPRESENTATION` / `ADVANCED_BREP_SHAPE_REPRESENTATION`),
   collect its `MANIFOLD_SOLID_BREP` items (its representation `items` list, or — if the rep
   wraps `SHAPE_REPRESENTATION_RELATIONSHIP` down to a brep rep — follow that link).
3. Every root `MANIFOLD_SOLID_BREP` must belong to exactly one placed component; a root not
   reached by any placement, or a component with no resolvable placement, → DECLINE (we do not
   default a solid to identity — that would mis-place it).

The exact linkage (which product-definition ties a `NEXT_ASSEMBLY_USAGE_OCCURRENCE` to a
`CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` to a child rep) is confirmed from the fixture Diagnose
(task 1.1); the design keeps the resolution structural (follow refs, no name matching) so it is
deterministic and does not depend on string labels.

## 3. `isRigid` — the honest placement gate

```cpp
// A rigid placement: orthonormal linear part with det ≈ +1 (a rotation). A scale,
// shear, or mirror (det ≈ −1, or M·Mᵀ ≠ I) is NOT rigid → DECLINE → OCCT. We decline
// (rather than apply) a non-rigid instance: the native Location would apply the linear
// part faithfully, but a scaled/mirrored component is out of this honest slice (it can
// change a solid's volume/handedness), so it falls to OCCT.
static bool isRigid(const math::Transform& t) {
  const math::Mat3& m = t.linear();
  const math::Mat3 mmt = m * m.transposed();          // must be ≈ identity
  if (!approxIdentity(mmt, 1e-9)) return false;
  const double det = t.determinant();
  return std::fabs(det - 1.0) < 1e-9;                  // proper rotation (not a mirror)
}
```

Every composed component placement passes through `isRigid` before it is applied; any failure
DECLINES the whole assembly → OCCT. (If `Mat3` lacks `transposed`, the predicate expands the
six orthonormality dot-products inline — no math change beyond a read-only helper.)

## 4. `build()` — placed compound (the one behavioural change)

```cpp
topo::Shape build() {
  if (!validateUnitContext()) return {};
  if (hasNestedAssembly()) return assembly();          // was: return {};  (blanket decline)

  const std::vector<int> brepIds = findManifoldBreps(); // FLAT path — unchanged
  ...                                                    // single Solid / flat Compound
}

// Parse the assembly transform tree → a placed Compound, or NULL (DECLINE).
topo::Shape assembly() {
  const auto placements = nextAssemblyPlacements();      // map<childRepId, Transform>
  if (placements.empty() || fail_) return {};            // could not compose → OCCT
  std::vector<topo::Shape> placed;
  for (const auto& [repId, xf] : placements) {           // deterministic: sorted repIds
    if (!isRigid(xf)) return {};                         // non-rigid → OCCT
    for (const int brepId : brepsOfRep(repId)) {         // component's root breps
      const topo::Shape solid = mapManifoldBrep(brepId); // build at LOCAL coords (unchanged)
      if (fail_ || solid.isNull()) return {};            // out-of-slice component → OCCT
      placed.push_back(solid.located(topo::Location{xf}));
    }
  }
  // Every root brep must have been placed exactly once (no unplaced/duplicated root).
  if (placed.empty() || placed.size() != findManifoldBreps().size()) return {};
  if (placed.size() == 1) return placed.front();         // a single placed component
  return topo::ShapeBuilder::makeCompound(std::move(placed));
}
```

`mapManifoldBrep` is reused **unchanged** — it builds each component solid at its own
(component-local) coordinates exactly as the single-solid slice; the placement is applied by
`located()` afterward. The FLAT multi-solid path and the single-solid path are **byte-for-byte
the prior behaviour** (only a present transform tree takes the new `assembly()` branch, which
previously returned NULL — so no accepting path regresses; a file that previously imported
flat still has no transform tree and still imports flat).

## 5. AP214 / AP242 schema-header acceptance (confirm + pin)

The reader's `Parser::parse` finds `DATA;` and never inspects the `FILE_SCHEMA` line, so an
`AUTOMOTIVE_DESIGN {...}` (AP214) or `AP242_MANAGED_MODEL_BASED_3D_ENGINEERING {...}` header is
already accepted as long as the DATA entities + unit context are in slice. This change:
- adds a host test that a buffer with an AP214 / AP242 `FILE_SCHEMA` header and in-slice
  entities imports (proving schema-independence),
- adds a sim foreign fixture (OCCT writes AP214 by default via `STEPControl_Writer` with the
  `write.step.schema` parameter, or `STEPCAFControl_Writer`),
- updates `step_reader.h`'s doc-comment to state schema-independence explicitly (so a future
  reader does not "helpfully" add a schema gate).

No reader logic changes for the schema; it is confirmed + pinned, not newly added — reported
honestly as such.

## 6. healShell per placed solid (policy unchanged)

`readStepString` deliberately does NOT run `heal::healShell` (it planarizes, destroying curved
faces); the shared-node reconstruction is watertight by construction and the engine
self-verifies. For a placed solid the same holds: `mapManifoldBrep` builds the watertight solid
in local coords; `located()` rigidly re-places it (a rotation/translation preserves
watertightness); the engine's per-member `robustlyWatertight` verifies the WORLD-placed solid
(accessors resolve the Location). A placed solid that still leaves a gap fails the self-verify →
OCCT. The `healShell` deferral described by `add-native-step-import` is unchanged; the spec
restates it for the placed case. No healing-policy change.

## 7. Architecture / OCCT boundary (unchanged)

```
cc_step_import (facade, unchanged)
   └─ active engine
        ├─ NativeEngine::step_import(path)
        │     ├─ step_import_native(path)  (src/native/exchange, OCCT-FREE) → topo::Shape
        │     │     • transform tree present → assembly(): compose rigid Location per
        │     │       component → placed solids → Compound (this change)
        │     │     • flat multi-root → Compound (unchanged) ; one root → Solid (unchanged)
        │     │     • any FILE_SCHEMA header accepted (schema-independent — confirmed)
        │     │     • non-rigid transform / out-of-slice component / uncomposable
        │     │       structure / non-mm → NULL (DECLINE)
        │     ├─ NULL → OCCT fall-through
        │     └─ Solid/Compound → robustlyWatertightImport (per-member) → wrap native
        │             └─ any member fails → OCCT fall-through (labelled)
        └─ OCCT STEPControl_Reader (fallback + oracle + fixture author)
   cc_iges_* / cc_step_export → unchanged
```

`src/native/**` stays OCCT-free (grep-gated). `step_writer.cpp` and the tessellator are NOT
modified. No `cc_*` ABI change; default engine stays OCCT (opt-in `cc_set_engine(1)`).

## 8. Honest scope (what still DECLINEs → OCCT)

- **Non-rigid / scaled / mirrored** placement transforms — DECLINE (only rotation+translation
  composed).
- **Deep multi-level nesting, external part refs, shared sub-assembly transform sub-trees** —
  single-level rigid placements only; a structure the reader cannot fully compose → DECLINE.
- **Out-of-slice component geometry** (`TOROIDAL_SURFACE`, `SURFACE_OF_REVOLUTION`,
  `TRIMMED_CURVE`, rational B-splines, `BEZIER`) inside any component → whole assembly DECLINES.
- **PMI / GD&T / colours / names**, non-mm units, non-manifold / unhealable component B-reps —
  DECLINE.
- **IGES** import/export — stay OCCT `IGESControl_*`. **#8 `drop-occt`** stays blocked.

## 9. Cognitive complexity

`itemDefinedTransform` / `representationMap` / `mappedItem` are small ref-resolving builders
(≤ ~8 each, mirror `axis2placement`). `nextAssemblyPlacements` is a ref-walk building a map
(≤ ~12; a `parser/systems` band function, documented). `assembly()` is a compose-and-place
loop with guard clauses (≤ ~12). `isRigid` / `frameToWorld` are ≤ ~6. `build()` gains one
branch. All measured with the `cognitive-complexity` skill before archive; nothing pushed to a
higher band (systems targets 25–35).

## 10. Verification plan

- **Gate 1 (host, OCCT-free)** — `tests/native/test_native_step_reader.cpp`:
  - **Placement composition** — an `ITEM_DEFINED_TRANSFORMATION('',$,#from,#to)` with a known
    frame delta composes to the expected `Location` (rotation `Mat3` + translation `Vec3`
    checked against the delta); `frameToWorld(from)⁻¹` composed with `frameToWorld(to)`.
  - **Placed compound** — a two-component transform-tree buffer imports as a `Compound` of two
    `Solid`s whose per-solid centroids sit at the composed world placements (not at the origin).
  - **Rigid gate** — a `MAPPED_ITEM`/`ITEM_DEFINED_TRANSFORMATION` with a scaled or mirrored
    frame delta DECLINES (NULL).
  - **Out-of-slice component** — an assembly whose one component carries a `TOROIDAL_SURFACE`
    DECLINES (NULL) — no partial import.
  - **Uncomposable structure** — a transform tree with a root brep reached by no placement
    DECLINES (NULL).
  - **Schema** — an AP214 / AP242 `FILE_SCHEMA` header with in-slice entities imports.
  - **No regression** — the FLAT multi-solid, single-solid, quadric, and bspline-face
    round-trip cases STILL pass; `decline_transformed_assembly_returns_null` is repurposed to
    the non-rigid/out-of-slice case (a rigid assembly now IMPORTS, so that exact fixture, if
    rigid, is replaced by a placed-import assertion).
- **Gate 2 (sim vs OCCT, OCCT linked)** — `tests/sim/native_step_import_parity.mm` +
  `scripts/run-sim-native-step-import.sh` via `cc_*` under `cc_set_engine(1)`;
  `xcrun simctl list devices booted` first:
  - **(A) assembly parity** — OCCT authors a 2-component assembly (two boxes at distinct rigid
    placements via the transform tree, `STEPControl_AsIs` on a compound of transformed solids,
    or `STEPCAFControl_Writer` on an XCAF assembly doc); native `cc_step_import` returns a
    placed `Compound`; OCCT `STEPControl_Reader` re-imports; assert same solid **count**, same
    **total volume** (within rel tol), per-solid **bbox + centroid/placement** matched within
    tol.
  - **(B) 3+-component / nested** — if the authoring path yields it tractably, extend the
    parity assertion; else document that (B) reduces to (A) repeated (honest).
  - **(C) unsupported assembly** — OCCT authors an assembly with a component out of slice (or a
    scaled instance); native `cc_step_import` DECLINES → OCCT and matches `cc_set_engine(0)`.
  - **AP214 header** — OCCT authors an AP214 file with in-slice entities; native import matches
    OCCT re-import.
  - Own `main()`, on the `run-sim-suite.sh` SKIP list; default engine restored in teardown so
    the suite assertion count is unchanged.
- **Done** only when the relevant gates are green and every existing suite stays green at the
  OCCT default (prior import slices host round-trip + sim `[NIMPORT]` 28/28, STEP export,
  healing, SSI S1–S5, native blends + #6/#7, marching, boolean, construct, tessellation,
  phase3 do NOT regress). Honestly reported: this adds rigid single-level assembly import
  (placed compound) + AP214/AP242 header acceptance (confirmed schema-independent); non-rigid /
  deep-nested / out-of-slice / PMI assemblies stay OCCT; #8 `drop-occt` stays blocked.
