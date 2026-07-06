# Proposal — add-native-step-scaled-ap242

## Why

The native STEP import reader (`add-native-step-import` → `widen-native-step-import` →
`add-native-step-assemblies`, all archived) tokenizes an ISO-10303-21 (Part-21) file and maps
the in-slice B-rep subset to a native `topology::Shape`: a `Solid` for one root
`MANIFOLD_SOLID_BREP`, a **flat** `Compound` for several co-equal roots at world coordinates,
and — the slice just landed — a **placed** `Compound` for a single-level **RIGID** assembly
(each component `MANIFOLD_SOLID_BREP` placed by the composed rotation+translation its
`ITEM_DEFINED_TRANSFORMATION` / `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION` transform
tree carries). But it **DECLINES two things it can honestly handle**:

1. **A scaled or mirrored component placement.** `Mapper::isRigid()` gates every composed
   component `Transform` to orthonormal-with-det≈+1, so the moment a component is placed by a
   **uniform scale** (`k·I` rotation) or a **mirror/reflection** (det ≈ −1) the whole file
   declines → OCCT. That is correct-but-narrow: a uniform-scale or mirrored instance is still a
   plain **affine** placement the native topology already models exactly, and the tessellator
   already renders a scaled placed solid correctly — the reader is simply refusing to compose
   it.

2. **An AP242 file that carries PMI / GD&T / annotation entities.** The reader's two GLOBAL
   scans over the record table — `validateUnitContext()` (require SI mm) and
   `hasNestedAssembly()` (any `REPRESENTATION_RELATIONSHIP*` / `MAPPED_ITEM` /
   `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` → the assembly path) — see the extra entities an
   AP242 file emits (semantic PMI, geometric-tolerance, draughting/annotation, extra unit
   contexts for angle/plane-angle) and either fail the unit gate or drive the assembly path
   into a structure it cannot compose, declining the WHOLE file even though the **geometry**
   (the `MANIFOLD_SOLID_BREP` solids) is fully in slice. The PMI is not geometry we import; but
   its mere presence should not cost us the solids.

This slice **widens native import along two bounded, honest axes**:

- **(T1) UNIFORM-SCALE + MIRROR component placements.** The composed component `Transform` is
  already the exact affine map the file carries. `isRigid()` is replaced by an
  **`isSupportedPlacement()`** classifier that ACCEPTS three affine classes and DECLINES the
  rest:
  - **rigid** (rotation + translation) — unchanged, imports as today;
  - **uniform scale** (`R·(k·I)`, one positive scale factor `k` on an orthonormal rotation)
    — the placed solid's volume scales by `k³`, and the tessellator renders it correctly
    **unmodified** (it derives the world normal from the transformed tangents
    `cross(place(∂u), place(∂v))`, which under a uniform `k>0` scales in magnitude but keeps
    direction + winding — see `surface_eval.h`), so a uniform-scale placement rides the
    existing located-node machinery transparently;
  - **mirror / reflection** (an orthonormal linear part with det ≈ −1, optionally with a
    uniform scale) — a reflection is affine and the topology `Location` applies it faithfully,
    BUT `cross(place(∂u), place(∂v))` REVERSES relative to the local outward normal (a
    reflection flips handedness), so a naively-placed mirrored solid would mesh with INWARD
    normals → negative enclosed volume → fail the engine self-verify. Because the tessellator
    is **not modified**, the reader compensates at the **topology** level: it applies the
    mirror `Location` AND **complements the orientation** of the mirrored component's faces (the
    existing `Orientation` `complemented`/`reversed` algebra in `shape.h`), so the world normals
    point outward again and the solid self-verifies watertight with the correct (positive) volume.

  **Non-uniform scale / shear** (`scale1/2/3` differing, or a linear part that is neither a
  scaled-rotation nor a scaled-reflection) still **DECLINES → OCCT** — the honest boundary is
  unchanged; only the *uniform* affine classes are added.

- **(T2) AP242 FILE TOLERANCE — skip PMI, import the geometry.** The two global scans are
  taught to **ignore AP242 annotation / PMI / GD&T entities** instead of tripping on them:
  - `validateUnitContext()` — an AP242 file carries extra unit assignments (plane-angle,
    solid-angle, and the PMI representation contexts). The mm-length gate is unchanged (still
    require SI mm, still decline a non-mm LENGTH_UNIT), but the presence of angle / steradian /
    annotation unit contexts is no longer read as "non-mm" — they are skipped.
  - `hasNestedAssembly()` — the assembly-trigger scan is scoped so that
    `REPRESENTATION_RELATIONSHIP*` / `MAPPED_ITEM` / `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`
    entities that belong to the **PMI / annotation representation graph** (not to a
    product-placement transform tree that reaches a `MANIFOLD_SOLID_BREP` root) do NOT force the
    assembly path; and the assembly composer, when it encounters an annotation/PMI relationship
    it does not need, SKIPS it rather than declining, as long as every geometric root brep is
    still placed exactly once.
  - The brep traversal itself is **already** PMI-blind: it walks refs down from
    `MANIFOLD_SOLID_BREP` roots and never touches an unreferenced `DRAUGHTING_MODEL` /
    `ANNOTATION_*` / `*_GEOMETRIC_TOLERANCE` / `DIMENSIONAL_*` / semantic-PMI entity. This slice
    adds the **explicit skip policy** (a recognised, curated set of AP242 annotation keywords is
    ignored) + regression-pins it: an AP242 file whose geometry is in slice imports its solids;
    its PMI is silently dropped.

So the slice **replaces the rigid-only placement gate with a uniform-affine classifier** (rigid
+ uniform-scale + mirror, mirror compensated by face-orientation complement) and **relaxes the
two global scans to skip AP242 PMI/annotation entities** — importing the solids of an AP242 file
while dropping its PMI. It is **not** a general product-structure importer and **not** a PMI
importer: non-uniform/shear placements, deep-nested product structure, out-of-slice component
geometry, and all PMI *semantics* stay OCCT.

This does NOT unblock #8 `drop-occt` (a general STEP/AP242 reader + IGES + a general-curved
kernel still block it). It is an additive breadth widening of the working import slice.

## What changes

1. **Placement classifier (`step_reader.cpp`, `math`).** Replace the boolean `isRigid()` gate
   with `classifyPlacement(const math::Transform&) → optional<PlacementClass>` returning one of
   `{Rigid, UniformScale(k), Mirror(k)}` or `nullopt` (DECLINE):
   - **Rigid** — `MᵀM ≈ I` and det ≈ +1 (today's `isRigid`, unchanged behaviour).
   - **UniformScale** — the linear part factors as `R·(k·I)` for a single `k > 0` and an
     orthonormal `R` (i.e. `MᵀM ≈ k²·I`, det ≈ +k³ > 0). One positive scale, no shear.
   - **Mirror** — `MᵀM ≈ k²·I` with det ≈ −k³ < 0 (an orthonormal reflection, optionally
     uniformly scaled). Handedness-flipping but conformal.
   - **DECLINE** (`nullopt`) — `MᵀM` not a scalar multiple of `I` (non-uniform scale or shear),
     or a degenerate/singular linear part. Non-uniform/shear is out of the honest slice.
   The classifier is a small read-only predicate over the composed `Transform` (uses the
   existing `Mat3::transposed`/`determinant`); `math` gains no behavioural change. It is also
   fed the raw `CARTESIAN_TRANSFORMATION_OPERATOR_3D` form when the file carries one (see 3).
2. **Mirror orientation compensation (`step_reader.cpp`, topology).** When a component's
   placement classifies as **Mirror**, the reader still applies `Shape::located(Location{T})`
   (the affine reflection is applied faithfully by the topology + accessors), AND it complements
   the component solid's face orientation so the tessellator's tangent-derived normals point
   OUTWARD again (a reflection flips `cross(∂u,∂v)`). This is done with the EXISTING topology
   orientation algebra (`reversed`/`complemented` in `shape.h`) applied when the mirrored
   solid's shell/faces are assembled — no tessellator change, no new topology primitive. A
   mirrored solid that after compensation still fails the engine's watertight+positive-volume
   self-verify → OCCT (never a fabricated flip).
3. **Scale/mirror transform sources (`step_reader.cpp`).** The scale/mirror can arrive two ways;
   the reader composes whichever the file carries and classifies the RESULT:
   - **In the `AXIS2_PLACEMENT_3D` frames** — a mirrored/scaled `ITEM_DEFINED_TRANSFORMATION`
     frame pair already yields a scaled/reflected composed `Transform` (the existing
     `frameToWorld(to) ∘ frameToWorld(from)⁻¹` naturally carries it); the classifier reads it off
     the result. No new parse.
   - **A `CARTESIAN_TRANSFORMATION_OPERATOR_3D`** — the STEP entity that explicitly carries a
     `scale` (uniform) or `scale1/scale2/scale3` (non-uniform → DECLINE) plus an axis triad. A
     new `cartesianOperator(id) → optional<Transform>` reads
     `CARTESIAN_TRANSFORMATION_OPERATOR_3D('',#axis1,#axis2,#origin,scale[,#axis3])` (and the
     `_2D`/base forms as needed) into an affine `Transform`; a present `scale1/2/3` triple with
     unequal factors → DECLINE. The composer routes a transform operator through the same
     classifier + mirror-compensation. The exact emitted form is confirmed against an
     OCCT-authored fixture Diagnose (task 1.1) before the arm is finalised.
4. **AP242 unit-context tolerance (`step_reader.cpp`).** `validateUnitContext()` keeps the mm
   LENGTH gate (require SI mm; a non-mm length still declines) but no longer treats the presence
   of PLANE_ANGLE / SOLID_ANGLE / PMI-context unit assignments as disqualifying — they are
   skipped. The gate answers exactly one question: "is the LENGTH unit millimetre?" — additive
   PMI unit contexts do not change the answer.
5. **AP242 PMI/annotation skip policy (`step_reader.cpp`).** `hasNestedAssembly()` +
   `assembly()` are scoped to the PRODUCT-PLACEMENT transform graph:
   - a curated set of AP242 annotation/PMI keywords (`DRAUGHTING_MODEL`, `ANNOTATION_*`,
     `*_GEOMETRIC_TOLERANCE`, `DIMENSIONAL_*`, `PLACED_DATUM_TARGET_FEATURE`,
     `DATUM*`, `GEOMETRIC_ITEM_SPECIFIC_USAGE`, `PRESENTATION_*`, etc. — grounded against the
     fixture in task 1.2) is recognised and **skipped** — such entities never force the assembly
     path and never fail a scan;
   - a `REPRESENTATION_RELATIONSHIP` / `MAPPED_ITEM` / `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`
     that belongs to the annotation graph (does not reach a `MANIFOLD_SOLID_BREP` root through
     its child representation) is skipped by the composer rather than declined, provided every
     GEOMETRIC root brep is still placed exactly once (the completeness gate is unchanged; it is
     computed over the geometric roots only). If a genuinely uncomposable PRODUCT transform
     remains → DECLINE (unchanged honesty).
   The brep ref-traversal is already PMI-blind; this change makes the SKIP explicit + pinned so
   a future scan does not "helpfully" re-add a global decline.
6. **Native reader API + OCCT-free build (`step_reader.h` / `native_exchange.h`).**
   `step_import_native` signature unchanged (still one `topo::Shape`, now possibly a placed
   `Compound` with uniformly-scaled / mirrored members). Doc-comment updated: uniform-scale +
   mirror single-level assemblies import; AP242 files import geometry with PMI skipped;
   non-uniform/shear + PMI semantics still decline. OCCT-free, host-buildable.
7. **Engine hook + OCCT fallback (`native_engine.cpp`) — unchanged logic, wider input.**
   `step_import` still calls `step_import_native` then `robustlyWatertightImport` (per-member
   for a Compound). A uniformly-scaled placed member self-verifies with volume `k³·V₀ > 0`; a
   mirror-compensated member self-verifies with the correct positive volume. Any NULL parse or
   leaky/negative-volume member → OCCT `STEPControl_Reader` re-reads the SAME file. `iges_*` /
   `step_export` untouched.
8. **healShell / tessellator policy (unchanged).** No tessellator change. `readStepString` still
   does not planarize via `healShell` (that would destroy curved faces). A uniform-scale placed
   solid is watertight by construction (a conformal map preserves the 2-manifold); a mirrored
   solid is watertight after the orientation compensation. A placed solid that still leaves a
   gap fails the engine self-verify → OCCT.
9. **Verification** — extend `scripts/run-sim-native-step-import.sh` +
   `tests/sim/native_step_import_parity.mm` with (A) an OCCT-authored **scaled** assembly (a
   component at 2× uniform scale) → native placed-compound import vs OCCT re-import (count /
   total volume with the scaled component's volume `k³·V₀` / per-solid bbox+placement); (B) an
   OCCT-authored **mirrored** assembly (a reflected component) → native import vs OCCT re-import
   (same count / volume / mirrored bbox+placement, correct outward orientation); (C) an
   OCCT-authored **AP242** file (a solid + PMI annotations) → native import of the SOLID vs OCCT
   re-import (identical solid, PMI ignored); (D) a **non-uniform/shear** assembly → honest NULL →
   OCCT. Host CTest gains the classifier unit cases + mirror-compensation + AP242-skip cases; the
   `decline_non_rigid_assembly_returns_null` test is split: uniform-scale + mirror now IMPORT,
   non-uniform/shear RETAINS the decline.

Additive throughout; the `cc_*` ABI never changes; the default engine stays OCCT.

## Non-goals (DEFERRED — return NULL → OCCT, not implemented, not faked)

- **Non-uniform scale / shear placement transforms** — a `Location` would apply the linear
  part, but a non-uniform/shear instance is out of the honest slice (it distorts the solid
  non-conformally; the tessellator's tangent-derived normal + the watertight self-verify are not
  validated for it), and the classifier DECLINES → OCCT. Only uniform scale + reflection (+
  rigid) are composed.
- **PMI / GD&T / semantic annotation IMPORT** — AP242 annotation entities are SKIPPED (the
  geometry imports; the PMI is dropped), NEVER imported or turned into geometry. We do not
  invent a solid from a PMI/annotation entity.
- **Deep (multi-level) nested product structure, external part references, shared sub-assemblies
  with their own transform sub-trees** — single-level component placements only; a genuinely
  recursive assembly the reader cannot fully compose → DECLINE → OCCT.
- **Out-of-slice component geometry** (`TOROIDAL_SURFACE`, `SURFACE_OF_REVOLUTION`,
  `TRIMMED_CURVE`, rational/weighted B-splines, `BEZIER`) inside any component → the whole
  assembly / file DECLINES → OCCT (no partial import).
- **Non-mm units, non-manifold / unhealable component B-reps** — DECLINE (unchanged gates; the
  mm-length gate is unchanged, only additive PMI unit contexts are skipped).
- **Inventing a placement, a scale, a reflection, or a solid** — only transforms/solids present
  in the file are composed; a structure that cannot be composed to a supported placement for
  every geometric root DECLINES rather than defaulting a component to identity or fabricating
  geometry. A mirror is compensated by the topology orientation algebra, never by faking a
  normal or hand-flipping tessellation output.
- **Unblocking #8 `drop-occt`** — a general STEP/AP242 reader + IGES + a general-curved kernel
  still block it. Reported honestly.

## Impact

- `src/native/exchange/step_reader.cpp` — `isRigid` → `classifyPlacement`
  (`{Rigid,UniformScale,Mirror}` | DECLINE); mirror face-orientation compensation in the placed
  path; `cartesianOperator(id)` for `CARTESIAN_TRANSFORMATION_OPERATOR_3D`; `validateUnitContext`
  skips PMI unit contexts (mm gate unchanged); `hasNestedAssembly`/`assembly` skip AP242
  annotation/PMI entities. The rigid + flat multi-solid + single-solid paths are UNCHANGED for
  rigid/flat/single files. `step_reader.h` / `native_exchange.h` — doc-comment documents
  uniform-scale + mirror placements + AP242-geometry-with-PMI-skipped. OCCT-free, host-buildable.
  `step_writer.cpp` and the tessellator are NOT modified.
- `src/native/math/**` — the classifier is a read-only predicate (inline in the reader or a tiny
  helper); no behavioural change to existing `math` / `Transform`.
- `src/native/topology/**` — no new primitive; the mirror compensation uses the EXISTING
  `Orientation` `complemented`/`reversed` algebra + `Shape::located`.
- `src/engine/native/native_engine.cpp` — **unchanged logic** (`robustlyWatertightImport`
  per-member-verifies a Compound; a scaled/mirrored member is a Solid). `iges_*` / `step_export`
  unchanged.
- `tests/native/test_native_step_reader.cpp` — classifier unit cases (a uniform-scale frame pair
  → `UniformScale(k)`; a mirrored frame pair → `Mirror`; a non-uniform/shear → DECLINE); a placed
  scaled two-component buffer → a `Compound` whose scaled solid has `k³` volume; a placed mirrored
  buffer → a `Compound` whose mirrored solid is watertight with positive volume + reflected
  centroid; an AP242 buffer with PMI/annotation + extra unit contexts → imports the solid, ignores
  the PMI; a non-uniform assembly → NULL; the rigid / flat multi-solid / single-solid / bspline
  round-trips STILL pass.
- `tests/sim/native_step_import_parity.mm` + `scripts/run-sim-native-step-import.sh` — the (A)
  scaled, (B) mirrored, (C) AP242, (D) non-uniform-decline cases. Own `main()`, on the
  `run-sim-suite.sh` SKIP list; default engine restored in teardown.
- **No** `cc_kernel.h` / `cc_kernel.cpp` change; the `cc_*` ABI is unchanged; default engine
  stays OCCT. The prior import slices (flat multi-solid + bspline-face + rigid assemblies, sim
  `[NIMPORT]` 33/33), STEP export, healing, SSI S1–S5, blends/#6/#7, phase3 do NOT regress.

## Verification

1. **Host unit (OCCT-free).** `classifyPlacement` maps a uniform-scale linear part to
   `UniformScale(k)`, a reflection to `Mirror`, a rigid to `Rigid`, and a non-uniform/shear to
   DECLINE; a two-component transform-tree buffer with a 2× uniform-scale component imports as a
   `Compound` whose scaled solid's volume is `8×` the unit and whose centroid sits at the scaled
   world placement; a mirrored-component buffer imports as a `Compound` whose mirrored solid is
   valid + watertight with POSITIVE volume and a reflected centroid (proving the orientation
   compensation); an AP242 buffer (a solid + PMI + extra unit contexts) imports the solid and
   drops the PMI; a non-uniform/shear placement DECLINES (NULL); the rigid / flat / single /
   round-trip cases are unchanged.
2. **Sim vs OCCT (simulator, OCCT linked).** OCCT `STEPControl_Writer` /
   `STEPCAFControl_Writer` authors (A) a scaled assembly (a component at 2× uniform scale), (B) a
   mirrored assembly (a reflected component), (C) an AP242 file (a solid + PMI annotations); native
   `cc_step_import` (engine 1) imports each; OCCT `STEPControl_Reader` re-imports the same file;
   the two agree on solid **count**, **total volume** (the scaled component contributing `k³·V₀`),
   and per-solid **bbox + placement** within tolerance; the AP242 solid imports identically with
   PMI ignored. A (D) non-uniform/shear assembly DECLINES natively and imports via OCCT identical
   to `cc_set_engine(0)`.

Done only when the relevant gates pass and every existing suite stays green at the OCCT default.
Reported honestly: this adds **uniform-scale + mirror** single-level assembly import (placed
compound, mirror compensated by the topology orientation algebra) + **AP242 geometry import with
PMI skipped**; non-uniform/shear transforms, deep-nested / out-of-slice assemblies, and PMI
semantics stay OCCT; arbitrary / AP242-general / IGES import remain OCCT and #8 `drop-occt` stays
blocked.
