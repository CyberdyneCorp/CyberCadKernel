# Design — add-native-step-scaled-ap242

Widen the WORKING native STEP reader (`src/native/exchange/step_reader.{h,cpp}`, the archived
`add-native-step-import` → `widen-native-step-import` → `add-native-step-assemblies` slices)
along two bounded axes: **(T1)** accept **uniform-scale** and **mirror (reflection)** component
placements in a single-level assembly (in addition to the rigid placements it composes today),
and **(T2)** import the **geometry** of an **AP242** file while **skipping** its PMI / GD&T /
annotation entities. Map ONLY onto placements + geometry the file genuinely carries and that the
native topology + un-modified tessellator genuinely render; DECLINE (NULL → OCCT) otherwise.
Clean-room from ISO 10303-21/-42/-43 + AP242 (ISO 10303-242) + the existing reader; OCCT
(`STEPControl_Reader` / `STEPControl_Writer` / `STEPCAFControl_Writer`) is the ORACLE +
fixture-author + fallback only. NOT a general product-structure importer; NOT a PMI importer.

## 0. What the reader already does (the substrate this widens)

Two passes over a `map<#id, Record>`: Pass A resolves leaf geometry; Pass B builds topology
(`MANIFOLD_SOLID_BREP`→solid). `build()` gates on `validateUnitContext` (mm only), then routes
a present transform tree to `assembly()`:

```
if (hasNestedAssembly()) return assembly();   // rigid placements, else DECLINE
```

`assembly()` (landed by `add-native-step-assemblies`) walks each
`CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` → `(REPRESENTATION_RELATIONSHIP +
REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION)` → `ITEM_DEFINED_TRANSFORMATION` (an
`AXIS2_PLACEMENT_3D` FROM/TO pair), composes `T = frameToWorld(to) ∘ frameToWorld(from)⁻¹`,
**gates it with `isRigid(T)`** (orthonormal, det ≈ +1), maps each root brep at local coords and
applies `Shape::located(Location{T})`, requiring every geometric root placed exactly once. The
engine (`native_engine.cpp::step_import`) keeps the result native ONLY when
`robustlyWatertightImport` passes (per-member for a Compound); else OCCT.

**Two exact facts this design leans on (verified in the source):**

- `math::Transform` (`transform.h`) is a **full affine** map `v' = L·v + t` with `L` a general
  `Mat3` — it ALREADY models uniform scale (`Mat3::diagonal(s,s,s)`), non-uniform scale, and
  mirror (`determinant()<0`, `isMirrored()`), and composes/inverts in closed form. So a scaled /
  mirrored placement is representable with **no math change** — `isRigid()` is the only thing
  refusing it.
- The tessellator (`surface_eval.h::SurfaceEvaluator::d1`) computes the world normal as
  `normal = cross(place(∂u), place(∂v))` where `place` applies the located `Transform`'s linear
  part to the tangents. This is the load-bearing behaviour for T1 (§3).

## 1. The native capability this maps onto (checked)

| STEP concept | Native construct | Exists? | Evidence |
|---|---|---|---|
| A uniform-scale instance placement (`R·kI` + t, k>0) | `Location{Transform{Mat3, Vec3}}` via `Shape::located` | **YES** | `transform.h` `Mat3::diagonal` + full-affine compose; `surface_eval.h` `cross(place(∂u),place(∂v))` scales magnitude, keeps direction+winding under k>0 → correct normal + `k³` volume, tessellator UNMODIFIED |
| A mirror (reflection, det<0) instance placement | same `Location`, PLUS a face-orientation complement | **YES (with §3 compensation)** | `transform.h` applies the reflection faithfully; but `cross(place(∂u),place(∂v))` FLIPS under det<0 → inward normals unless the topology orientation is complemented (`shape.h` `reversed`/`complemented`) |
| Skipping an AP242 annotation/PMI entity | ref-driven brep traversal never visits an unreferenced entity | **YES (already) + explicit skip (§5)** | `mapManifoldBrep` walks refs down from a `MANIFOLD_SOLID_BREP`; an unreferenced `DRAUGHTING_MODEL`/`ANNOTATION_*`/`*_GEOMETRIC_TOLERANCE` is never touched — the only trip hazards are the two GLOBAL scans (§4/§5) |
| A rigid-vs-scale-vs-mirror-vs-shear classifier | `MᵀM` scalar-multiple test + det sign | **derived** | `Mat3::transposed`/`determinant` exist; conformality is `MᵀM ≈ k²·I` — a tiny predicate |

The topology + un-modified tessellator are a **conformal-placed-instance kernel already** for
`k>0`; the ONLY new work is (a) a classifier that admits the conformal (uniform-scale/mirror)
classes and rejects shear, (b) a topology-level orientation complement for the mirror class, and
(c) scoping the two global scans to skip AP242 PMI.

## 2. T1 — scale / mirror transform structure (ISO 10303-42/-43, AP242)

A uniform scale or mirror can arrive **two ways**; the reader composes whichever the file
carries and classifies the RESULT `Transform` — it never trusts a keyword, it measures the map.

### 2.1 Via the `ITEM_DEFINED_TRANSFORMATION` frame pair (no new parse)

`add-native-step-assemblies` already composes `T = frameToWorld(to) ∘ frameToWorld(from)⁻¹`. If
the `#to` (or `#from`) `AXIS2_PLACEMENT_3D` triad is left-handed (a mirror) or the frames encode a
scale, the composed `T` naturally carries it. `classifyPlacement(T)` (§3) reads it off the result.
No new entity parse — the same `itemDefinedTransform` builder, with `isRigid` replaced by
`classifyPlacement`.

### 2.2 Via `CARTESIAN_TRANSFORMATION_OPERATOR_3D` (new small builder)

The STEP entity that EXPLICITLY carries a scale/mirror operator:

```
#op = CARTESIAN_TRANSFORMATION_OPERATOR_3D('', #axis1/*X dir or $*/, #axis2/*Y dir or $*/,
                                            #localOrigin, scale/*real or $*/, #axis3/*Z dir or $*/);
```

`cartesianOperator(id) → optional<Transform>` builds the affine map: the (orthonormalised, or
as-given) axis triad forms the linear columns, `scale` (default 1) multiplies uniformly, the
local origin is the translation. A **non-uniform** operator
(`CARTESIAN_TRANSFORMATION_OPERATOR_3D_NON_UNIFORM` or a `scale1/scale2/scale3` triple with
unequal factors) → `nullopt` (DECLINE). The composed result is fed to the SAME
`classifyPlacement`. **The exact emitted keyword + arg order is confirmed against an OCCT-authored
scaled/mirrored fixture Diagnose (task 1.1)** before this arm is finalised — the shape above is
the ISO form; if OCCT emits the scale purely inside the `AXIS2_PLACEMENT_3D` frames (§2.1) and
never emits a `CARTESIAN_TRANSFORMATION_OPERATOR_3D`, this builder is added defensively but §2.1
carries the fixtures.

### 2.3 `classifyPlacement` — the honest affine gate

```cpp
enum class PlacementClass { Rigid, UniformScale, Mirror };
struct Placement { PlacementClass cls; double scale; };  // scale = k (Rigid ⇒ 1)

// Classify the composed affine linear part. Accepts the three CONFORMAL classes
// (rigid, uniform-scale, mirror); DECLINES non-uniform/shear (nullopt).
static std::optional<Placement> classifyPlacement(const math::Transform& t) {
  const math::Mat3& m = t.linear();
  const math::Mat3 g = m.transposed() * m;             // Gram: MᵀM
  // Conformal ⇔ MᵀM = k²·I : diagonal equal + off-diagonal zero.
  const double k2 = g(0,0);
  if (k2 <= 1e-18) return std::nullopt;                 // degenerate
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = 0; j < 3; ++j) {
      const double target = (i == j) ? k2 : 0.0;
      if (std::fabs(g(i,j) - target) > 1e-9 * (k2 + 1.0)) return std::nullopt;  // shear/non-uniform
    }
  const double k = std::sqrt(k2);
  const double det = m.determinant();                   // ≈ ±k³
  if (det > 0.0) return Placement{ std::fabs(k - 1.0) < 1e-9 ? PlacementClass::Rigid
                                                             : PlacementClass::UniformScale, k };
  return Placement{ PlacementClass::Mirror, k };         // det<0 ⇒ reflection (± uniform scale)
}
```

`Rigid` is exactly the old `isRigid==true` case (k≈1, det>0), so **the rigid path is
byte-unchanged**. `UniformScale` and `Mirror` are the new admitted classes; everything else
(a `MᵀM` that is not `k²·I`) DECLINES. The tolerance is scale-relative (`1e-9·(k²+1)`) so a
large-`k` placement is not spuriously rejected.

## 3. T1 — the tessellator interaction (why uniform-scale is free and mirror needs one complement)

`surface_eval.h::d1` produces `out.normal = cross(place(∂u), place(∂v))` and `face_mesher.h`
flips winding by `face.orientation() == Reversed`. Under a placed `Transform` with linear part
`L`:

- **Uniform scale `L = k·R`, k>0, R∈SO(3):** `place(∂u) = k·R·∂u`, `place(∂v) = k·R·∂v`, so
  `cross = k²·(R·∂u × R·∂v) = k²·R·(∂u × ∂v)` — same DIRECTION as the rigid case (R preserves
  the cross product, k²>0 preserves sign). Normal points outward, winding unchanged, volume
  scales by det = `k³`. **The tessellator renders it correctly with NO change.** This is why T1's
  uniform-scale sub-track adds nothing to the tessellator: the located-node machinery already
  applies `L` and the normal falls out right.

- **Mirror `L = k·M`, det<0 (M a reflection):** `cross(M·∂u, M·∂v) = det(M)·M·(∂u × ∂v)` — the
  `det(M)<0` FLIPS the normal relative to the rigid outward convention. So a mirrored solid
  meshed as-is has every face normal pointing INWARD → the solid-mesher computes a NEGATIVE
  enclosed volume → the engine's watertight+`volume>0` self-verify FAILS. Since the tessellator
  is **not modified**, the reader compensates at the topology level: for a Mirror-class component,
  after mapping the component solid at local coords, it **complements the orientation** of the
  solid's faces (equivalently, reverses the solid's shell orientation) using the existing
  `Orientation` algebra (`reversed`/`complemented` in `shape.h`) before applying the mirror
  `Location`. The double flip (reflection flips the geometric normal; the orientation complement
  flips the winding convention back) leaves the world normals pointing OUTWARD, so the mirrored
  solid self-verifies watertight with the CORRECT positive volume. No tessellator arm, no new
  topology primitive, no fabricated normal.

`mapManifoldBrep` is reused **unchanged** to build the solid at local coords; the orientation
complement is applied to the RESULT shape (a Solid is a shell of oriented faces; complementing the
solid's orientation propagates through the tessellator's `face.orientation()` read). The exact
mechanism (complement the top Solid orientation vs. rebuild the shell with reversed faces) is
chosen against the mirror fixture in task 3.x so the self-verify volume is positive; both use only
the existing algebra.

## 4. T2 — AP242 unit-context tolerance

`validateUnitContext()` today scans ALL `SI_UNIT` sub-records and requires a `.MILLI. .METRE.`
length; an AP242 file additionally carries `PLANE_ANGLE_UNIT` / `SOLID_ANGLE_UNIT` and PMI
representation-context unit assignments. The gate is refined to answer exactly ONE question —
"is the LENGTH unit millimetre?":

- iterate `SI_UNIT` sub-records; a length unit (`.METRE.` name) MUST be `.MILLI.` (unchanged
  decline on a non-mm length);
- a non-length SI_UNIT (`.RADIAN.`, `.STERADIAN.`, angle/PMI contexts) is **skipped** — its
  presence no longer disqualifies the file.

So an AP242 file with mm geometry + angle/PMI unit contexts passes the gate; a genuinely non-mm
file still declines. No tolerance is weakened — the mm gate is unchanged; only the additive PMI
unit contexts are ignored.

## 5. T2 — AP242 PMI / annotation skip policy

The brep ref-traversal is ALREADY PMI-blind (it walks down from `MANIFOLD_SOLID_BREP` roots and
never touches an unreferenced annotation entity). The only trip hazard is the GLOBAL
`hasNestedAssembly()` scan and the `assembly()` composer. This change scopes them to the
PRODUCT-PLACEMENT graph:

1. **A curated AP242 annotation/PMI keyword set is recognised + skipped** — e.g.
   `DRAUGHTING_MODEL`, `ANNOTATION_PLANE`, `ANNOTATION_OCCURRENCE`, `ANNOTATION_FILL_AREA*`,
   `*_GEOMETRIC_TOLERANCE` (position/flatness/…), `DATUM`, `DATUM_FEATURE`,
   `PLACED_DATUM_TARGET_FEATURE`, `DIMENSIONAL_CHARACTERISTIC_REPRESENTATION`,
   `DIMENSIONAL_LOCATION`, `GEOMETRIC_ITEM_SPECIFIC_USAGE`, `PRESENTATION_STYLE_ASSIGNMENT`,
   `DRAUGHTING_CALLOUT`, etc. The **exact** set is grounded against the OCCT-authored AP242
   fixture Diagnose (task 1.2) — the reader recognises the keywords the fixture actually emits and
   documents that the set is additive (an unrecognised annotation entity that is NOT referenced by
   any geometric root is still harmless because the traversal never visits it; a recognised one is
   explicitly skipped so it cannot trip a scan).
2. **`hasNestedAssembly()`** only returns true for a transform relationship that reaches a
   `MANIFOLD_SOLID_BREP` root (a PRODUCT placement), not for a `REPRESENTATION_RELATIONSHIP` /
   `MAPPED_ITEM` / `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` whose child representation is an
   annotation/draughting representation. A plain AP242 file with ONE solid + PMI therefore takes
   the single-solid / flat path, not the assembly path.
3. **`assembly()`** skips an annotation-graph relationship it does not need (its child rep reaches
   no geometric root brep) rather than declining, and computes its completeness gate over the
   GEOMETRIC root breps only. A genuinely uncomposable PRODUCT transform still DECLINES.

The result: an AP242 file whose SOLIDS are in slice imports its solids (single, flat, or placed
per T1); its PMI is silently dropped. No PMI is ever imported or turned into geometry.

## 6. `build()` / `assembly()` — the behavioural changes

```cpp
topo::Shape assembly() {
  // ... walk CDSR → (childRep, transform) as today, BUT:
  //   - skip an annotation-graph relationship (childRep reaches no geometric root) — §5
  //   - classifyPlacement(T) instead of isRigid(T): Rigid|UniformScale|Mirror → keep,
  //     nullopt (non-uniform/shear) → DECLINE — §3
  //   - remember each placed brep's PlacementClass
  // completeness gate over GEOMETRIC roots only (unchanged shape) — §5
  for (const int id : geometricBrepIds) {
    topo::Shape solid = mapManifoldBrep(id);               // local coords, UNCHANGED
    if (fail_ || solid.isNull()) return {};
    const Placement pl = placementOf(id);                  // Rigid|UniformScale|Mirror
    if (pl.cls == PlacementClass::Mirror)
      solid = complementOrientation(solid);                // §3 — existing algebra, no new prim
    solids.push_back(placed(id) ? solid.located(topo::Location{transformOf(id)}) : solid);
  }
  return solids.size() == 1 ? solids.front()
                            : topo::ShapeBuilder::makeCompound(std::move(solids));
}
```

`build()`'s single-solid + flat multi-solid paths are byte-unchanged; only the placement gate
(rigid → conformal) + the mirror complement + the AP242 skips are added.

## 7. Architecture / OCCT boundary (unchanged)

```
cc_step_import (facade, unchanged)
   └─ active engine
        ├─ NativeEngine::step_import(path)
        │     ├─ step_import_native(path)  (src/native/exchange, OCCT-FREE) → topo::Shape
        │     │     • transform tree present → assembly():
        │     │         classifyPlacement → Rigid|UniformScale|Mirror per component
        │     │         Mirror → complement face orientation (topology algebra)
        │     │         non-uniform/shear → NULL (DECLINE)
        │     │     • AP242 PMI/annotation entities + angle/PMI unit contexts → SKIPPED
        │     │     • flat multi-root → Compound ; one root → Solid (unchanged)
        │     │     • out-of-slice component / uncomposable structure / non-mm → NULL (DECLINE)
        │     ├─ NULL → OCCT fall-through
        │     └─ Solid/Compound → robustlyWatertightImport (per-member, volume>0) → wrap native
        │             └─ any member fails (incl. a wrongly-mirrored inward solid) → OCCT
        └─ OCCT STEPControl_Reader (fallback + oracle + fixture author)
   cc_iges_* / cc_step_export → unchanged
```

`src/native/**` stays OCCT-free (grep-gated). `step_writer.cpp` and the tessellator are NOT
modified. No `cc_*` ABI change; default engine stays OCCT (opt-in `cc_set_engine(1)`).

## 8. Honest scope (what still DECLINEs → OCCT)

- **Non-uniform scale / shear** placement transforms — DECLINE (only conformal
  rigid/uniform-scale/mirror composed).
- **PMI / GD&T / annotation SEMANTICS** — SKIPPED (never imported); an AP242 file's PMI is dropped,
  its geometry imported. No solid is ever invented from a PMI entity.
- **Deep multi-level nesting, external part refs, shared sub-assembly transform sub-trees** —
  single-level conformal placements only; a structure the reader cannot fully compose → DECLINE.
- **Out-of-slice component geometry** (`TOROIDAL_SURFACE`, `SURFACE_OF_REVOLUTION`,
  `TRIMMED_CURVE`, rational B-splines, `BEZIER`) inside any component → whole file DECLINES.
- **Non-mm length units** (the mm gate is unchanged), non-manifold / unhealable component
  B-reps — DECLINE.
- **IGES** import/export — stay OCCT `IGESControl_*`. **#8 `drop-occt`** stays blocked.

## 9. Cognitive complexity

`classifyPlacement` is a Gram-matrix conformality test with a det-sign branch (≤ ~10;
parser/systems band). `cartesianOperator` is a small ref-resolving builder (≤ ~8, mirrors
`axis2placement`). The `assembly()` additions (skip annotation relationships, remember placement
class, mirror complement) add a few guarded branches (target ≤ ~15; still within the
parser/systems band 25–35). `validateUnitContext` gains one skip branch (≤ ~8). All measured with
the `cognitive-complexity` skill before archive; nothing pushed to a higher band.

## 10. Verification plan

- **Gate 1 (host, OCCT-free)** — `tests/native/test_native_step_reader.cpp`:
  - **Classifier** — a uniform-scale linear part (`k·R`, k=2) → `UniformScale(2)`; a reflection
    (det<0) → `Mirror`; a rigid → `Rigid(1)`; a non-uniform diagonal (`diag(2,1,1)`) and a shear →
    DECLINE (nullopt).
  - **Scaled placed compound** — a two-component transform-tree buffer with a 2× uniform-scale
    component imports as a `Compound` of two `Solid`s; the scaled solid's volume is `8×` the
    unmscaled unit and its centroid sits at the scaled world placement; both valid + watertight.
  - **Mirrored placed compound** — a reflected-component buffer imports as a `Compound` whose
    mirrored solid is valid + watertight with POSITIVE volume and a reflected centroid (proves the
    orientation compensation; without it the volume would be negative / the solid non-watertight).
  - **AP242 tolerance** — a buffer whose `FILE_SCHEMA` is AP242 and which carries a solid + PMI /
    annotation entities + PLANE_ANGLE/PMI unit contexts imports the SOLID and drops the PMI (same
    solid as the AP203 equivalent).
  - **Non-uniform decline** — a non-uniform-scale / shear component placement DECLINES (NULL).
    Split the prior `decline_non_rigid_assembly_returns_null`: uniform-scale + mirror now IMPORT,
    non-uniform/shear RETAINS the decline.
  - **No regression** — the rigid assembly, FLAT multi-solid, single-solid, quadric, and
    bspline-face round-trip cases STILL pass.
- **Gate 2 (sim vs OCCT, OCCT linked)** — `tests/sim/native_step_import_parity.mm` +
  `scripts/run-sim-native-step-import.sh` via `cc_*` under `cc_set_engine(1)`;
  `xcrun simctl list devices booted` first:
  - **(A) scaled assembly parity** — OCCT authors a 2-component assembly with one component at 2×
    uniform scale; native `cc_step_import` returns a placed `Compound`; OCCT re-imports; assert
    same solid **count**, same **total volume** (scaled component contributing `k³·V₀`, rel tol),
    per-solid **bbox + centroid/placement** matched within tol.
  - **(B) mirrored assembly parity** — OCCT authors a reflected component; native import returns a
    placed `Compound`; assert same count / total volume / mirrored per-solid bbox + centroid, and
    that the native member is watertight with positive volume (correct outward orientation).
  - **(C) AP242 geometry** — OCCT (`STEPCAFControl_Writer` with PMI, or a hand-authored AP242
    header + PMI entities) writes a solid + PMI annotations; native `cc_step_import` imports the
    SOLID; OCCT `STEPControl_Reader` re-imports; assert identical solid (count / volume / bbox),
    PMI ignored on both sides.
  - **(D) non-uniform / shear** — OCCT authors an assembly with a non-uniform-scaled component;
    native `cc_step_import` DECLINES → OCCT and matches `cc_set_engine(0)`.
  - Own `main()`, on the `run-sim-suite.sh` SKIP list; default engine restored in teardown so the
    suite assertion count is unchanged.
- **Done** only when the relevant gates are green and every existing suite stays green at the OCCT
  default (prior import slices host round-trip + sim `[NIMPORT]` 33/33 incl. rigid assemblies,
  STEP export, healing, SSI S1–S5, native blends + #6/#7, marching, boolean, construct,
  tessellation, phase3 do NOT regress). Honestly reported: this adds uniform-scale + mirror
  single-level assembly import (placed compound, mirror compensated by the topology orientation
  algebra) + AP242 geometry import with PMI skipped; non-uniform/shear / deep-nested / out-of-slice
  transforms + PMI semantics stay OCCT; #8 `drop-occt` stays blocked.
