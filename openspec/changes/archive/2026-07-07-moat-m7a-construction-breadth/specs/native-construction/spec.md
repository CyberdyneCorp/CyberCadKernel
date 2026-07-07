# native-construction

## ADDED Requirements

### Requirement: N-section ruled loft via an additive facade entry (`cc_solid_loft_sections`)

The native construction library SHALL expose the already-landed OCCT-free N-section
ruled-loft builder (`src/native/construct/loft.h` `build_loft_sections`) through a NEW
ADDITIVE facade function `cc_solid_loft_sections(sectionsXYZ, counts, sectionCount)` —
the ≥3-section generalisation of `cc_solid_loft_wires`. `sectionsXYZ` holds the ordered
sections back to back as flat `(x, y, z)` triples; `counts[k]` (each ≥3) is the vertex
count of section `k`; `sectionCount` (≥2) is the number of sections. Consecutive sections
are skinned by `(sectionCount − 1)` RULED bands (one bilinear side face per corresponding
edge pair); the first and last sections are capped, internal sections are shared vertex
rings (not capped) → a closed watertight `Solid`. Sections MAY differ in vertex count
(made compatible by an arc-length-preserving resample). The change is ADDITIVE-ONLY: no
existing signature, POD layout, or enum value SHALL change, `cc_solid_loft` /
`cc_solid_loft_wires` and every other shipped op SHALL remain byte-identical, and the new
behaviour SHALL be reachable ONLY through the new `cc_solid_loft_sections` entry.

The engine SHALL keep the NATIVE result ONLY IF it self-verifies: the candidate
`build_loft_sections` solid is accepted iff it is non-null, `robustlyWatertight`, and has a
strictly positive `watertightVolume`; otherwise the SAME arguments SHALL be forwarded to
the OCCT oracle `OcctEngine::solid_loft_sections` (`BRepOffsetAPI_ThruSections`, solid +
ruled). The builder SHALL return NULL → OCCT for every out-of-slice input: fewer than 2
sections, any section with < 3 points, a NON-PLANAR or point-collapsed section, a
self-folding chain, a mismatched-count pair whose resampled caps cannot close watertight,
or a candidate whose mesh is not robustly watertight (e.g. an asymmetric
expand-then-contract spool whose two adjacent bands taper at different ratios and
T-junction the native tessellator's shared-ring seam — the solid's volume stays exact, but
the self-verify discards it rather than emit a non-watertight mesh). No tolerance SHALL be
weakened, the tessellator SHALL NOT be modified, and no leaky, volume-wrong, or
parity-mismatched solid SHALL ever be emitted. The native builder SHALL remain OCCT-free,
reference no OCCT / `IEngine` / `EngineShape` type, and keep `src/native/**` at zero OCCT
includes; OCCT `BRepOffsetAPI_ThruSections` is the verification ORACLE and the fallback
only, confined to `src/engine/occt`.

#### Scenario: An N-section planar loft builds natively with the closed-form volume (host)

- GIVEN an ordered chain of ≥3 PLANAR closed section polygons (equal or mismatched vertex counts) — e.g. a symmetric square spool (10×10 @z=0, 4×4 @z=6, 10×10 @z=12) or a stacked box — built on the host with NO OCCT linked
- WHEN `build_loft_sections` is computed and tessellated by `src/native/tessellate`
- THEN the result SHALL be a watertight `Solid` (`boundaryEdgeCount == 0`) whose enclosed volume equals the closed-form value (the prismatoid `h/3·(A1 + A2 + √(A1·A2))` per frustum band, or `A·H` for a straight stack) within the tessellation deflection bound — verified with no OCCT present

#### Scenario: The native N-section loft matches the OCCT ThruSections oracle through the facade (parity)

- GIVEN a planar ≥3-section loft on a booted iOS simulator (OCCT linked)
- WHEN `cc_solid_loft_sections` is called with the native engine active (`cc_set_engine(1)`) and the OCCT side builds `OcctEngine::solid_loft_sections` (`BRepOffsetAPI_ThruSections`, solid + ruled) on the same inputs through the SAME `cc_*` facade
- THEN the two shapes' mass properties, bounding box, watertightness, and face/edge topology SHALL agree within the deflection bound (planar sections agree to floating-point precision)

#### Scenario: An out-of-slice N-section loft declines honestly to OCCT (sim)

- GIVEN an N-section loft whose input is out of slice — a NON-PLANAR internal section, a point-collapsed or degenerate section, or a candidate whose mesh is not robustly watertight (e.g. an asymmetric 4×4→6×6→2×2 expand-then-contract spool) — with the native engine active
- WHEN `cc_solid_loft_sections` is invoked
- THEN the native builder SHALL return NULL (or the engine self-verify SHALL discard the candidate) AND the engine SHALL forward the SAME arguments to the OCCT `BRepOffsetAPI_ThruSections` oracle, returning that solid to floating-point precision — it SHALL NOT emit an approximate, self-intersecting, or non-watertight solid, and NO always-NULL dead builder SHALL be retained

#### Scenario: The additive entry leaves every shipped construction op byte-identical (host)

- GIVEN the pre-change baseline of `cc_solid_loft`, `cc_solid_loft_wires`, and the rest of the construction family
- WHEN `cc_solid_loft_sections` is added as a new additive facade entry with its OCCT oracle (`solid_loft_sections`) and its `IEngine` default that returns `engine_unsupported`
- THEN every other shipped construction op SHALL remain byte-identical (same signatures, POD layouts, results, and watertightness), the new code SHALL be reachable ONLY through the new entry, `include/cybercadkernel/cc_kernel.h` SHALL have NO deletions, and `src/native/**` SHALL keep zero OCCT includes
