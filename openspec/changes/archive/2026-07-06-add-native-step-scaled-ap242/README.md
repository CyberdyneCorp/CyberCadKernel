# add-native-step-scaled-ap242

A **bounded, honest breadth slice** of Phase 4 capability **#7 `native-exchange`**
(`openspec/NATIVE-REWRITE.md`), sitting on the working native STEP import reader
(`add-native-step-import` → `widen-native-step-import` → `add-native-step-assemblies`, all
archived). Those slices import a single solid, a **flat** multi-solid file, and a single-level
**RIGID** assembly (each component `MANIFOLD_SOLID_BREP` placed by the composed rotation +
translation its transform tree carries), but **DECLINE two things they can honestly handle**: a
component placed by a **uniform scale** or a **mirror (reflection)** (`isRigid()` gates the
composed transform to orthonormal-det≈+1), and an **AP242** file that carries PMI / GD&T /
annotation entities (the two global scans — the mm unit gate + the assembly trigger — trip on the
extra entities and decline the whole file even though the geometry is in slice).

This change widens along two axes:

- **(T1) uniform-scale + mirror placements.** `isRigid()` becomes `classifyPlacement()` — a
  Gram-matrix conformality test (`MᵀM ≈ k²·I`) with a det-sign branch that ACCEPTS `Rigid`,
  `UniformScale(k>0)`, and `Mirror(det<0)` and DECLINES non-uniform/shear. The native
  `math::Transform` already models the full affine map, and the tessellator (which derives the
  world normal from the transformed tangents, `cross(place(∂u), place(∂v))`) renders a uniform
  scale **transparently** (`k²` magnitude, direction + winding preserved; volume scales `k³`). A
  mirror flips the tangent-cross normal, so the reader compensates at the **topology** level —
  complementing the mirrored component's face orientation with the existing `Orientation`
  algebra — so the mirrored solid meshes with OUTWARD normals and self-verifies watertight with
  the correct positive volume. **No tessellator change.**

- **(T2) AP242 file tolerance.** The mm unit gate is refined to ask only "is the LENGTH unit
  millimetre?" (the additive PLANE_ANGLE / PMI unit contexts are skipped, not read as non-mm), and
  the assembly-trigger scan + composer are scoped to the PRODUCT-PLACEMENT graph so AP242
  annotation / GD&T / draughting entities are SKIPPED, not fatal. The brep ref-traversal is
  already PMI-blind; this makes the skip explicit + regression-pinned. An AP242 file whose solids
  are in slice imports its solids; its PMI is silently dropped — never imported, never turned into
  geometry.

Honest scope: **uniform-scale + reflection single-level assemblies + AP242 geometry with PMI
skipped**. A **non-uniform / shear** placement transform, a **deep multi-level nested** product
structure, an **external part reference**, a component whose geometry is out of the import slice
(`TOROIDAL_SURFACE`, `SURFACE_OF_REVOLUTION`, rational B-spline, …), a placement chain the reader
cannot compose, PMI **semantics**, and any placed solid that fails the watertight+positive-volume
self-verify → **NULL → OCCT `STEPControl_Reader`**. No placement, scale, reflection, or solid the
file did not describe is ever invented; no tolerance is weakened; the mirror is compensated by the
topology orientation algebra, never by faking a normal or hand-editing tessellation output. It
does NOT change the `cc_*` ABI, the default engine (stays OCCT), `step_writer.cpp`, or the
tessellator, and does NOT unblock #8 `drop-occt`.

The correctness gate is **sim vs OCCT**: OCCT `STEPControl_Writer` / `STEPCAFControl_Writer`
authors (A) a scaled assembly (a component at 2× uniform scale), (B) a mirrored assembly (a
reflected component), and (C) an AP242 file (a solid + PMI annotations); native import is compared
vs OCCT `STEPControl_Reader` re-import — same solid count, same total volume (the scaled component
contributing `k³·V₀`), per-solid bbox / placement within tolerance; the AP242 solid imports
identically with PMI ignored; a (D) non-uniform/shear assembly declines honestly to OCCT. Plus
host unit cases (classifier + mirror-compensation + AP242-skip) and a NUMSCI-ON no-interaction
build. Every prior suite stays green at the OCCT default.
