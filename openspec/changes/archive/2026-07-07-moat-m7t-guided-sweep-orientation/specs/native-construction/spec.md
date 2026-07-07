# native-construction

## ADDED Requirements

### Requirement: Guide-oriented sweep via an additive facade entry (`cc_guided_orient_sweep`)

The native construction library SHALL expose, through a NEW ADDITIVE facade function
`cc_guided_orient_sweep(profileXY, profileCount, pathXYZ, pathCount, guideXYZ, guideCount)`,
a swept solid whose section **orientation** is constrained by a guide polyline, matching
OCCT `BRepOffsetAPI_MakePipeShell` with `SetMode(guideWire)` in **NoContact** mode. This
entry is DISTINCT from the existing scale-splay `cc_guided_sweep` (which uses the guide to
drive section SCALE via `BRepOffsetAPI_ThruSections`) and SHALL NOT repurpose it. The
change is ADDITIVE-ONLY: no existing signature, POD layout, or enum value SHALL change,
`cc_guided_sweep` and every other shipped op SHALL remain byte-identical, and the new
behaviour SHALL be reachable ONLY through the new `cc_guided_orient_sweep` entry;
`include/cybercadkernel/cc_kernel.h` SHALL have NO deletions.

For the **straight-spine** slice (all path points collinear), the native builder SHALL
reproduce OCCT's `SetMode(guideWire)` NoContact frame law reverse-engineered from the
oracle (`GeomFill_GuideTrihedronPlan::D0`, `GeomFill_LocationGuide::D0` `rotation == false`):
at each spine station `P` with the constant spine tangent `T`, the aim axis
`N = normalize(Pprime − P)` where `Pprime` is the guide point lying in the plane through
`P` **perpendicular to `T`** (a segment/plane intersection against the guide polyline — NO
guide surface), the co-axis `B = T × N`, and the centred profile placed rigidly as
`world(x, y) = P + x·N + y·B` with **NO scaling**. This is a per-station rigid frame whose
in-plane angle is set by the **perpendicular-plane correspondence**; it SHALL NOT use a
guide-parameter-fraction correspondence (the measured M7a trap that was volume-correct but
spatially ~7% wrong). OCCT `BRepOffsetAPI_MakePipeShell + SetMode(guideWire)` is the
verification ORACLE and the fallback only, confined to `src/engine/occt`; the native builder
SHALL remain OCCT-free, reference no OCCT / `IEngine` / `EngineShape` type, and keep
`src/native/**` at zero OCCT includes.

The engine SHALL keep the NATIVE result ONLY IF it passes a **SPATIAL** self-verify: the
candidate solid SHALL be accepted iff it is non-null, robustly watertight, has strictly
positive volume, AND matches the OCCT oracle within tolerance on volume **AND on a spatial
metric — bounding-box corner delta within the deflection bound, with watertightness and
mesh-volume convergence bounding shape fidelity**.
Acceptance on volume / watertightness ALONE SHALL be forbidden (a rigidly mis-rotated
section is volume-correct yet spatially wrong — the M7a failure mode). On any spatial-
tolerance miss, a non-straight spine, an empty or ambiguous perpendicular-plane∩guide, or a
degenerate profile / path / guide, the builder SHALL return NULL and the engine SHALL
forward the SAME arguments to the OCCT oracle, returning that solid. No tolerance SHALL be
weakened, the tessellator SHALL NOT be modified, and no spatially-wrong, leaky, or
parity-mismatched solid SHALL ever be emitted.

#### Scenario: The guide-oriented section orientation matches an independent computation (host, GATE a)

- GIVEN a straight `+Z` spine over `z ∈ [0, H]`, a rectangle profile with half-extents `a ≠ b`, and a guide rotating by total angle `Θ` (`guide(z) = (ρ·cos(Θz/H), ρ·sin(Θz/H), z)`), built on the host with NO OCCT linked
- WHEN the per-station frame is computed by the perpendicular-plane law (`N = normalize(guide(z) − P)`, `B = T × N`)
- THEN the section aim-axis angle `atan2(N_y, N_x)` SHALL equal the independent closed form `Θ·z/H` to floating-point precision AND the swept solid's enclosed volume SHALL converge to the closed form `4ab·H` under band refinement (the residual being the ruled-surface discretization deficit that the OCCT approximated surface shares)

#### Scenario: An offset (non-rotating) guide yields an exact prism volume (host, GATE a)

- GIVEN a straight `+Z` spine and a straight offset guide line `(ρ, 0, z)` with a rectangle profile, built on the host with NO OCCT
- WHEN `cc_guided_orient_sweep`'s native builder is computed and tessellated by `src/native/tessellate`
- THEN the aim axis `N` SHALL be the constant offset direction AND the solid SHALL be a watertight prism whose enclosed volume equals `4ab·H` within the fp64/deflection bound

#### Scenario: The native guide-oriented sweep matches the OCCT PipeShell oracle spatially (sim, GATE b)

- GIVEN a straight-spine guide-oriented sweep on a booted iOS simulator (OCCT linked)
- WHEN `cc_guided_orient_sweep` is called with the native engine active (`cc_set_engine(1)`) and the OCCT side builds `OcctEngine::guided_orient_sweep` (`BRepOffsetAPI_MakePipeShell` + `SetMode(guideWire)`, NoContact) on the same inputs through the SAME `cc_*` facade
- THEN the two shapes SHALL agree within the deflection bound on mass properties (volume, area), watertightness, and face/edge topology **AND on the spatial metric — bounding-box corner delta within the deflection bound** (with watertightness and mesh-volume convergence) — so that an orientation error (volume-correct but spatially shifted) would FAIL the gate rather than pass it

#### Scenario: An out-of-slice or spatial-miss guide sweep declines honestly to OCCT (sim)

- GIVEN a guide sweep whose input is out of slice — a NON-straight (kinked or curved) spine, an empty/ambiguous perpendicular-plane∩guide, or a degenerate profile/path/guide — OR a candidate whose bbox corner delta exceeds the spatial tolerance, with the native engine active
- WHEN `cc_guided_orient_sweep` is invoked
- THEN the native builder SHALL return NULL (or the SPATIAL self-verify SHALL discard the candidate) AND the engine SHALL forward the SAME arguments to the OCCT `MakePipeShell + SetMode(guideWire)` oracle, returning that solid — it SHALL NOT emit a spatially-wrong, self-intersecting, or non-watertight solid, and NO always-NULL dead builder SHALL be retained

#### Scenario: The additive entry leaves every shipped construction op byte-identical (host)

- GIVEN the pre-change baseline of `cc_guided_sweep` (scale-splay), `cc_solid_sweep`, `cc_twisted_sweep`, and the rest of the construction family
- WHEN `cc_guided_orient_sweep` is added as a new additive facade entry with its OCCT oracle (`guided_orient_sweep`) and its `IEngine` default that returns `engine_unsupported`
- THEN every other shipped construction op SHALL remain byte-identical (same signatures, POD layouts, results, and watertightness), the new code SHALL be reachable ONLY through the new entry, `include/cybercadkernel/cc_kernel.h` SHALL have NO deletions, and `src/native/**` SHALL keep zero OCCT includes
