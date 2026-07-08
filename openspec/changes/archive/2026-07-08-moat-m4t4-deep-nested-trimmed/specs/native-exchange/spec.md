# native-exchange

## ADDED Requirements

### Requirement: Import a RECTANGULAR_TRIMMED_SURFACE face by reducing it to its native basis surface, or DECLINE

The native STEP importer (`step_import_native`, OCCT-free) SHALL admit an
`ADVANCED_FACE` whose surface entity is a
`RECTANGULAR_TRIMMED_SURFACE('',#basis_surface,u1,u2,v1,v2,usense,vsense)` by
**unwrapping it to `#basis_surface`** — recursively resolving `surface(#basis_surface)`
and returning that basis `FaceSurface` unchanged — mirroring the landed `TRIMMED_CURVE`
→ basis-curve and `SURFACE_CURVE` / `SEAM_CURVE` → `curve_3d` unwraps. The face's
`EDGE_LOOP` (`FACE_OUTER_BOUND` / `FACE_BOUND` → `ORIENTED_EDGE` → `EDGE_CURVE`) SHALL
remain the AUTHORITATIVE trim, its analytic pcurves reconstructed on the basis surface
exactly as for a directly-referenced basis; the rectangular `(u,v)` box SHALL be used
ONLY to guard the reduction and SHALL NOT synthesise any geometry. The reduction SHALL
be admitted ONLY when `surface(#basis_surface)` resolves to a supported native
`FaceSurface::kind` AND the face carries a real boundary loop. The importer SHALL
`decline()` → OCCT when: the basis resolves to `nullopt` (a surface kind outside the
supported native set — e.g. an offset / swept / hyperboloid surface), any of
`u1,u2,v1,v2` is non-finite, the box is empty or reversed (`u2 ≤ u1` or `v2 ≤ v1`), or
the face would require the rectangular box to be SYNTHESISED into a boundary wire (a
bare, loop-less rect-trim). A rect-trim over a `TOROIDAL_SURFACE` that carries real trim
edges SHALL inherit the existing partial-torus decline unchanged. No tolerance SHALL be
weakened, no boundary or surface SHALL be fabricated, the reader SHALL stay OCCT-free
(`src/native/**`), and the `cc_*` ABI SHALL be unchanged (additive reader behaviour
only). An admitted face SHALL remain subject to the engine's mandatory watertight +
volume/area self-verify against the OCCT oracle downstream; a non-watertight or
off-volume native result SHALL be DISCARDED → OCCT.

#### Scenario: A RECTANGULAR_TRIMMED_SURFACE over a PLANE imports to the same solid as its basis (host, analytic)

- GIVEN an in-scope ISO-10303-21 buffer describing a manifold-solid B-rep whose one `ADVANCED_FACE` references a `RECTANGULAR_TRIMMED_SURFACE` over a `PLANE` basis, bounded by a real `EDGE_LOOP`, read on the host with no OCCT
- WHEN `step_import_native` resolves the surface and assembles the solid
- THEN the imported native `Shape` SHALL be equivalent (identical enclosed volume, surface area, and face/edge/vertex topology) to the solid imported from the byte-identical buffer that references the basis `PLANE` directly — an independent host equivalence computed without OCCT — proving the unwrap is exact and drops only the redundant parameter box

#### Scenario: A RECTANGULAR_TRIMMED_SURFACE over a CYLINDRICAL_SURFACE imports watertight and matches OCCT (sim, parity)

- GIVEN a STEP file carrying a manifold-solid B-rep whose face surface is a `RECTANGULAR_TRIMMED_SURFACE` over a `CYLINDRICAL_SURFACE` basis, bounded by a real `EDGE_LOOP`, imported on a booted simulator with OCCT linked
- WHEN the reader unwraps to the cylinder basis and the native tessellator meshes the resulting solid
- THEN the native solid SHALL be valid + watertight AND its triangle-envelope count, enclosed volume, bounding box, and centroid SHALL match the OCCT `STEPControl_Reader` + `BRepMesh_IncrementalMesh` oracle within tolerance, AND the sampled boundary-corner points `S(uᵢ,vⱼ)` SHALL lie on the closed-form analytic cylinder within a scale-relative tolerance that is NOT widened

#### Scenario: A RECTANGULAR_TRIMMED_SURFACE over an unsupported basis declines to OCCT (host)

- GIVEN an in-scope ISO-10303-21 buffer whose `ADVANCED_FACE` references a `RECTANGULAR_TRIMMED_SURFACE` over a basis surface kind outside the supported native set (e.g. an offset / swept surface `surface()` does not map), read on the host with no OCCT
- WHEN `step_import_native` resolves the surface
- THEN it SHALL return a NULL `Shape` (DECLINE) so the engine falls through to OCCT — no basis surface, boundary, or solid is fabricated, and the tolerance is NOT widened

#### Scenario: A RECTANGULAR_TRIMMED_SURFACE with an inverted or degenerate parameter box declines (host)

- GIVEN an in-scope ISO-10303-21 buffer whose `RECTANGULAR_TRIMMED_SURFACE` carries a non-finite bound or an empty/reversed box (`u2 ≤ u1` or `v2 ≤ v1`), read on the host with no OCCT
- WHEN `step_import_native` validates the rectangular box
- THEN it SHALL `decline()` (NULL → OCCT) rather than accept a malformed wrapper — no approximate or leaky face is emitted

### Requirement: Deep-nested (3+-level) rigid assemblies compose the full leaf→root chain

The native STEP importer SHALL compose the world placement of a leaf in an assembly of
ARBITRARY nesting depth by walking the `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` /
`REPRESENTATION_RELATIONSHIP` parent edges from the leaf shape-representation to its
unique root, composing `W = T₁ ∘ T₂ ∘ … ∘ Tₙ` with each level applied on the left
(`composeChain`). A length-1 chain SHALL reproduce the landed single-level placement
byte-identically, a length-2 chain the landed 2-level placement, and a length-N chain
(N ≥ 3) SHALL compose every ancestor transform — the depth SHALL be bounded only by the
chain data, never by a constant. The existing per-level conformality gate (rigid /
uniform-scale / mirror only), the cycle guard, the unique-root requirement, and the
all-but-one-root completeness gate SHALL apply unchanged at every depth; any dangling,
non-conformal, cyclic, or ambiguous (shared-sub-assembly, multi-parent) chain SHALL
DECLINE → OCCT as it does today. This requirement LOCKS already-shipped behaviour with a
depth ≥ 3 regression proof; it changes no code.

#### Scenario: A 3-level nested rigid assembly composes W = T₁ ∘ T₂ ∘ T₃ (host, analytic)

- GIVEN an in-scope ISO-10303-21 buffer describing a leaf `MANIFOLD_SOLID_BREP` placed into a sub-assembly SR by a rigid `T₃`, that sub-assembly placed into a second sub-assembly SR by a rigid `T₂`, and that placed into the ROOT SR by a rigid `T₁` (three `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`s), read on the host with no OCCT
- WHEN `step_import_native` walks the relationship chain from the leaf SR to its unique root
- THEN the imported leaf SHALL land at the world placement equal to the leaf geometry mapped by the INDEPENDENTLY-computed matrix product `T₁·T₂·T₃` (computed in the test without `composeChain`), confirming the deep chain composes every ancestor transform and drops none

#### Scenario: A 3-level nested assembly matches the OCCT oracle (sim, parity)

- GIVEN the same 3-level ISO-10303-21 buffer, imported on a booted simulator with OCCT linked
- WHEN the reader composes the chain and the native tessellator meshes the resulting solid
- THEN the native result SHALL match the OCCT `STEPControl_Reader` + `BRepMesh_IncrementalMesh` oracle on solid count, enclosed volume, bounding box, centroid, and watertight topology

#### Scenario: A shared sub-assembly (multi-parent) at depth still declines (host)

- GIVEN an in-scope ISO-10303-21 buffer in which one child shape-representation is placed into TWO distinct parents (a shared sub-assembly instanced twice), read on the host with no OCCT
- WHEN `step_import_native` builds the parent-edge forest
- THEN it SHALL return a NULL `Shape` (DECLINE) — the multi-parent instancing model is out of this slice and stays an honest, unchanged decline → OCCT, never a misplaced solid
