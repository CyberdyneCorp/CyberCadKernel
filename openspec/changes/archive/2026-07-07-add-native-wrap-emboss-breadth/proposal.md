## Why

Feature **#7 WRAP-EMBOSS** has a native FIRST SLICE (`add-native-wrap-emboss`,
archived): behind the unchanged `cc_wrap_emboss` ABI, a native OCCT-free builder
(`src/native/feature/wrap_emboss.h`) embosses a RAISED RECTANGULAR pad on a
CYLINDER lateral face (`boss = 1`), verified vs the Phase-3 OCCT oracle
(`occt_wrap_emboss.cpp`, GitHub #290). Everything else — deboss, non-rectangular
profiles, non-cylindrical bases — still returns NULL → OCCT.

That slice deliberately covers ONE case to prove the pipeline (footprint
projection onto the cylinder → wrapped cap-and-side pad → shared-vertex weld →
mandatory watertight + volume-sign self-verify). This change WIDENS the native
path along three independent, honestly-scoped tracks, each reusing the proven
machinery and each gated by the SAME self-verify → OCCT fallback:

- **T1 DEBOSS** is a direct mirror of the raised pad: instead of an outer cap at
  `R + height` and side walls pushed OUT, cut a pocket with a floor at
  `R − depth` and side walls pushed IN. The base wall is retiled over the full
  turn with the same footprint window removed; the pad walls now close the pocket
  INWARD. Volume SHRINKS by ≈ footprint-area × depth. Highest confidence — the
  geometry is the existing build with a sign flip on the radial offset and the
  wall outward normals, plus a volume-DECREASING self-verify branch.

- **T2 NON-RECTANGULAR profile** generalises the 4-corner axis-aligned rectangle
  to an arbitrary CLOSED N-vertex POLYGON in the profile's `(px, py)` space. Each
  vertex projects to the cylinder by the SAME map (`u = px/R`, `v = py + vMid`);
  polygon edges that are neither axial nor circumferential become HELICAL curves
  on the cylinder. The outer cap is the triangulated polygon mapped to
  `R + height`; the side walls are one ruled strip per edge; the base wall is
  retiled over the full turn with the polygon WINDOW removed via a constrained
  triangulation that shares the polygon-boundary vertices. Convex polygons build
  robustly; simple non-convex (e.g. L-shaped) is attempted via ear-clipping and
  accepted ONLY if the mandatory self-verify passes, else NULL → OCCT.

- **T3 FREEFORM / non-cylindrical base** is the hardest track: a raised
  rectangular pad on a CONE lateral face. A cone's outward normal is not purely
  radial, so the pad is offset along the SURFACE NORMAL; the parallel surface at
  offset `h` of a cone is again a coaxial cone (same half-angle, shifted
  reference radius), so the cap stays an elementary patch and the same
  shared-vertex weld applies to the retiled lateral wall. SPHERE (full-ball
  closure + polar singularity) and every other freeform base stay OCCT. T3 is a
  NARROW slice with an explicit honest-decline: if the cone weld cannot be made
  watertight with the correct volume, the builder returns NULL → OCCT and the
  measured gap is REPORTED — no dead code, no faked tolerance.

The raised-rectangular-pad-on-cylinder emboss remains the UNTOUCHED control. All
three tracks reuse `native-math` (Cylinder/Cone frames), `native-boolean`
(`Polygon`/`Plane`/`assembleSolid`), and the native tessellator's watertight
self-verify. Native stays OCCT-free; the path is NUMSCI-independent (the weld is a
shared-vertex `assembleSolid`, no SSI substrate), so it builds in both the default
and `CYBERCAD_HAS_NUMSCI` configurations.

## What Changes

- **Native builder (`src/native/feature/wrap_emboss.h`, OCCT-free, additive).**
  - **T1 DEBOSS.** Accept `boss == 0` for the rectangular-footprint-on-cylinder
    case. Parametrise the radial offset: `rOut = boss ? R + height : R − depth`,
    guarded by `depth < R` (the floor stays off the axis). Flip the pocket floor
    and side-wall outward normals so the solid boundary faces INTO the pocket.
    Retile the base wall over the full turn with the footprint window removed
    (unchanged) and close it with the inward pad. Return NULL for `depth ≥ R` or
    any non-rectangular / non-cylindrical input.
  - **T2 NON-RECTANGULAR polygon.** Generalise `rectFootprint` to an N-vertex
    simple closed polygon (`count ≥ 3`, positive shoelace area, non-self-
    intersecting, fits within the face's angular `< 2π` and axial extent).
    Triangulate the polygon (fan for convex; ear-clip for simple non-convex),
    subdivide for the cylinder's angular sagitta, map to `R + height` for the
    cap; emit one ruled side wall per edge (tiled for helical edges); retile the
    wall-minus-polygon-window with a constrained triangulation sharing the
    polygon-boundary vertices. Return NULL for self-intersecting / degenerate
    polygons or when the constrained triangulation cannot close.
  - **T3 CONE base (narrow slice).** Recover a `FaceSurface::Kind::Cone` lateral
    face (frame, reference radius, half-angle). Project the rectangular footprint
    onto the cone `(u, v)`; build the pad offset along the surface NORMAL to the
    parallel coaxial cone at offset `height`; retile the cone lateral wall minus
    the window with the same shared-vertex weld; leave the cone's base disk /
    apex untouched. Return NULL for a sphere/BSpline/other base, or when the
    weld cannot close watertight (honest decline → OCCT).
- **Engine self-verify (`native_engine.cpp` `wrapEmbossVerified`).** Generalise
  the guard: (a) DERIVE the sign from `boss` — `boss != 0` requires the volume to
  GROW, `boss == 0` requires it to SHRINK, each by a magnitude within the
  documented plausible band; (b) compute the footprint area from the true
  SHOELACE polygon area (not the bounding box), so a non-rectangular profile is
  gated on its real area; (c) for a non-cylindrical base use the wrapped footprint
  area on the base SURFACE (the cone's local metric varies with `v`), within a
  deflection-bounded band. NULL builder OR a failing self-verify ⇒ OCCT
  `cc_wrap_emboss`. No tolerance is ever weakened; the measured gap is reported.
- **Native code stays OCCT-free**; the weld is the shared-vertex `assembleSolid`
  (NUMSCI-independent), so the widened path builds and runs in BOTH the default
  and `CYBERCAD_HAS_NUMSCI` configurations. No `cc_*` signature or POD struct
  changes.

**No `cc_*` ABI change.** `cc_wrap_emboss` keeps its exact signature and
semantics. The three tracks add NEW native paths behind it; OCCT remains the
oracle and the fallback for every case a track declines (sphere base, general
freeform base, self-intersecting profile, over-2π / off-end footprint, and every
input a self-verify rejects).

## Capabilities

### Modified Capabilities
- `native-wrap-emboss`: widen the native, OCCT-free wrap-emboss beyond the raised-
  rectangular-pad-on-cylinder first slice with THREE new native cases behind the
  unchanged `cc_wrap_emboss` — (T1) a RECESSED rectangular DEBOSS pocket on a
  cylinder (`boss = 0`, volume shrinks), (T2) a NON-RECTANGULAR closed POLYGON
  raised emboss on a cylinder, and (T3) a raised rectangular emboss on a CONE
  lateral face (narrow slice; sphere/other freeform bases decline). Each track is
  built by the same project → cap-and-side → shared-vertex weld pipeline, is
  OCCT-free and NUMSCI-independent, and is gated by a GENERALISED mandatory self-
  verify (watertight + signed volume change — grow for emboss, shrink for deboss —
  by the true wrapped footprint area × height/depth) that DISCARDS a bad result →
  OCCT `cc_wrap_emboss`. Anything outside a track's proven sub-slice returns NULL →
  OCCT, with the measured gap reported. The raised-rectangular-pad-on-cylinder
  emboss is preserved as an untouched control.
- `wrap-emboss`: record that `cc_wrap_emboss` now has native paths for deboss
  (rectangular pocket on a cylinder), non-rectangular polygon emboss on a
  cylinder, and rectangular emboss on a cone lateral face (behind the unchanged
  signature), with the Phase-3 OCCT cap-and-side + healed-sew builder remaining
  the oracle and the fallback for every other case (sphere / general freeform
  base, self-intersecting or dense profiles, over-2π / off-end footprints, and any
  input a self-verify rejects). The observable ABI contract and volume-change-sign
  semantics are unchanged.

## Impact

- **ABI**: none. `cc_wrap_emboss` keeps its signature and POD structs; the three
  tracks are internal native paths with OCCT fallback.
- **Build**: extends `src/native/feature/wrap_emboss.h` (OCCT-free, header-only)
  and generalises the `NativeEngine::wrap_emboss` self-verify. The weld stays the
  shared-vertex `assembleSolid`, so the native path remains NUMSCI-independent and
  builds in both configs. The tessellator is NOT modified (additive only).
- **Verification**: two gates. **host** (OCCT-free CTest): T1 deboss is watertight
  with volume reduced by ≈ footprint-area × depth; T2 polygon emboss is watertight
  with volume grown by the shoelace polygon area × height; T3 cone emboss (when it
  builds) is watertight with the cone-surface wrapped area × height; every
  declined case returns NULL. **sim** native-vs-OCCT parity
  (`scripts/run-sim-native-wrap-emboss.sh` + `.mm`, extended): each track's native
  `cc_wrap_emboss` vs the OCCT `cc_wrap_emboss` on the same input (volume / area /
  watertight / `IsValid` within a deflection-bounded tolerance; report the
  measured deltas).
- **Roadmap**: advances `ROADMAP.md` #7 and the `SSI-ROADMAP.md` `S5 → #7`
  on-ramp by widening the native wrap-emboss coverage (deboss + non-rectangular +
  cone) while keeping OCCT authoritative for the rest.
- **Regression**: additive only. The raised-rectangular-pad-on-cylinder emboss is
  the untouched control; the OCCT wrap-emboss (#290, Phase 3) is unchanged and
  remains the oracle + fallback; native booleans, SSI, and the native blends are
  untouched; each new native path only fires for its named sub-slice and is gated
  by the self-verify.
- **Risk / honesty**: honest per-track scope. T1 (deboss) is the highest-
  confidence mirror; T2 (polygon) is robust for convex and attempts simple
  non-convex under the self-verify gate; T3 (cone) is a narrow slice with an
  explicit honest-decline to OCCT if the weld cannot be made watertight. Anything
  unsafe or out-of-slice returns NULL → OCCT, so no track can ever emit a wrong or
  leaky embossed body, and the measured OCCT-fallback gap is REPORTED, never masked
  with a weakened tolerance.
