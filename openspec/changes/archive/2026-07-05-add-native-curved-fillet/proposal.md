## Why

Feature **#6 CURVED BLENDS** is one of the two frontier features sitting on the
finished SSI / curved-boolean stack (`ROADMAP.md`; `SSI-ROADMAP.md` §Sequencing:
`S5 curved booleans → #6 blends`). The native blend library today
(`src/native/blend/fillet_edges.h`) fillets only a STRAIGHT convex edge shared by
two PLANAR faces: a rolling ball in a planar dihedral traces a **CYLINDER** (axis
∥ the straight crease). A CURVED adjacent face or a curved crease returns a NULL
Shape → OCCT `BRepFilletAPI_MakeFillet`. #6 is a ~months feature; this change lands
ONE concrete, verifiable **first slice** of it, honestly scoped.

The first slice is the simplest curved crease that closes in closed form: a
**constant-radius rolling-ball fillet on a CIRCULAR edge** — the rim where a
cylinder LATERAL face meets a PLANAR CAP (the top/bottom disc of a native cylinder
solid). This is the exact curved analogue of the planar fillet:

- In a planar dihedral the rolling ball sweeps a **cylinder** (canal surface of a
  straight crease). On a circular crease the ball centre travels a CIRCLE, so the
  canal surface it sweeps is a **TORUS** — the surface of revolution of the ball's
  cross-section circle about the cylinder axis. The kernel already has the native
  `math::Torus` (value / dU / dV / normal) and a native REVOLVE that emits an exact
  torus patch (`src/native/construct/residuals.h build_revolution_torus`).
- The two trim seams — where the torus is tangent to the cylinder lateral face and
  to the planar cap — are **CIRCLES**. torus∩cylinder (coaxial) and torus∩plane
  (axis-perpendicular) are BOTH in the shipped native SSI **S1** closed-form family
  (`src/native/ssi/`: `quadric_pairs.h`, `plane_torus.h`) and are analytically
  exact. So both tangent circles come from S1, not from marching.

So #6's first slice reuses everything the stack already ships: `math::Torus` +
native revolve for the patch, native SSI **S1** circle seams for the trim, and the
planar blend's `blend_geom.h` half-space clip + boolean `assembleSolid` weld +
faceted-arc discipline for a watertight result. It is the curved sibling of the
planar `fillet_edges` slice, gated by the SAME engine self-verify.

`cc_fillet_edges` ALREADY exists (planar native path + OCCT fallback). This change
adds a NATIVE path for the circular cylinder↔cap rim behind the SAME ABI — no
signature change — with the OCCT oracle as the fallback for the rest.

## What Changes

- Extend the native blend library (`src/native/blend/`) with a **circular-crease
  rolling-ball fillet** that runs NATIVELY when `cc_fillet_edges(body, {edge}, 1,
  r)` selects a CIRCULAR edge shared by exactly two faces — a `FaceSurface`-kind
  `Cylinder` lateral face and a `FaceSurface`-kind `Plane` cap — meeting at a CONVEX
  crease, with `r` small enough that both tangent circles stay inside their faces.
- Construct the blend as a **torus canal-surface patch**:
  - The rolling ball of radius `r` seated tangent to the cylinder (radius `R`, axis
    `A`) and the cap plane (at the cap height `z_cap`, normal ∥ axis) has its centre
    on a CIRCLE coaxial with the cylinder at radius `R − r` (convex external rim) and
    axial offset `r` from the cap. That centre-circle + the ball cross-section is a
    **torus**: major radius `R_t = R − r`, minor radius `r`, coaxial with the
    cylinder, centred at axial `z_cap ∓ r`. (Signs by rim orientation; documented.)
  - The tangent seam on the cylinder is the **circle** torus∩cylinder at radius `R`,
    axial `z_cap ∓ r` (coaxial quadric pair — native SSI S1 `quadric_pairs.h`).
  - The tangent seam on the cap is the **circle** torus∩plane at radius `R − r` in
    the cap plane (axis-perpendicular plane∩torus — native SSI S1 `plane_torus.h`).
  - Both seams are **self-verified** to lie on the torus AND on their respective
    original surface (cylinder / plane) within a scale-derived tolerance before use.
- **Trim + insert + weld** (reusing `blend_geom.h` + `assembleSolid`):
  - Trim the cylinder lateral face back to the cylinder tangent circle and the cap
    face back to the cap tangent circle (drop the rim wedge between the two seams).
  - Tessellate the torus blend patch (the quarter-tube swept between the two seam
    circles) into deflection-bounded facets (a ring of quad strips, minor-angle
    sweep chosen so the sagitta `r(1−cos Δθ/2) ≤ deflection`, and enough major-angle
    segments to bound the `R_t` sagitta) — the same facet discipline the planar
    fillet arc uses, but a torus (two curvatures) instead of a cylinder.
  - The patch shares its rim vertices with the two trimmed faces so the shell closes;
    weld + triangulate watertight through the boolean `assembleSolid`.
- **Self-verify → OCCT fallback in the engine** (`native_engine.cpp`): the existing
  `blendResultVerified` guard (watertight + a convex blend strictly REDUCES volume:
  `0 < Vr < Vo`) already gates the native `fillet_edges` dispatch; the circular-rim
  result flows through the SAME guard. If the native builder returns NULL (out of
  scope) OR the result fails the guard, the engine falls through to OCCT
  `BRepFilletAPI_MakeFillet`. No new guard is added; the discipline is unchanged.
- Native code stays **OCCT-free**. The torus patch uses `math::Torus` + native
  revolve; the seam circles use native SSI S1 (which is header-math for these
  closed-form coaxial/axis-perpendicular pairs and builds WITHOUT the NUMSCI
  substrate). No `cc_*` signature or POD struct changes.

**No `cc_*` ABI change.** `cc_fillet_edges` keeps its exact signature; the circular
cylinder↔cap rim is a new NATIVE path behind it, OCCT for the rest.

## Capabilities

### New Capabilities
<!-- none — #6's first slice EXTENDS the existing native-blends capability with a
     curved (circular-crease) fillet; it does not introduce a new capability. -->

### Modified Capabilities
- `native-blends`: extend the native constant-radius `cc_fillet_edges` from the
  STRAIGHT planar-dihedral slice (rolling-ball → cylinder) to its FIRST curved
  crease — a CIRCULAR edge where a cylinder lateral face meets a planar cap, whose
  rolling-ball canal surface is a **TORUS** and whose two tangent trim seams are
  **CIRCLES** (torus∩cylinder + torus∩plane, native SSI S1). The native patch is
  trimmed, inserted G1-tangent, and welded watertight through the boolean
  `assembleSolid`; the engine's existing mandatory watertight + volume-reducing
  self-verify accepts it or DISCARDS it → OCCT `BRepFilletAPI_MakeFillet`. Concave
  rims, variable radius, cyl↔cyl canal fillets, non-circular curved creases, and
  freeform stay OCCT-fallthrough (never faked). Delivered behind the unchanged
  `cc_fillet_edges` signature via `engine-adapter`.

## Impact

- **ABI**: none. `cc_fillet_edges` keeps its signature and POD structs; the circular
  rim is an internal native path with OCCT fallback.
- **Build**: adds a header (e.g. `src/native/blend/fillet_circular_edge.h`) to the
  OCCT-free native blend library; consumes `native-math` (`Torus`, `Cylinder`,
  `Plane`), `native-construction` (revolve/torus patch), `native-ssi` S1 (the two
  seam circles — solver-free, builds with NUMSCI OFF), `native-boolean`
  (`assembleSolid` weld), `native-tessellate` (watertight mesher — NOT modified).
- **Verification**: two gates. **host** (OCCT-free CTest): the torus patch + both
  seam circles lie on their surfaces within tol, the result meshes watertight, and
  its volume equals `|body| − (removed rim-wedge volume)` within the tessellation
  deflection bound. **sim** native-vs-OCCT parity (`run-sim-native-curved-fillet.sh`):
  the native fillet vs OCCT `BRepFilletAPI_MakeFillet` on the same cylinder↔cap rim
  (volume / area / watertight / G1-tangency at the two seams within tol).
- **Roadmap**: implements the `ROADMAP.md` #6 FIRST SLICE and the `SSI-ROADMAP.md`
  `S5 → #6 blends` on-ramp (S1 circle seams consumed by a curved blend). Concave /
  variable / cyl∩cyl-canal / non-circular-curved / freeform blends remain future
  #6 work.
- **Regression**: additive only. The planar `fillet_edges` (straight dihedral),
  chamfer/offset/shell, native booleans, SSI S1–S5, and all marching/boolean parity
  are untouched; the circular-rim path only fires for the named curved input and is
  gated by the existing self-verify.
- **Risk / honesty**: honest scope — the torus canal surface and its two circle
  seams are exact closed form; every seam is self-verified on both surfaces and the
  assembled solid is gated by the engine's watertight + volume-reducing guard.
  Anything unsafe or out-of-slice returns NULL → OCCT, so the slice can never emit a
  wrong or leaky curved blend. The measured OCCT-fallback gap is REPORTED, never
  masked with a weakened tolerance.
