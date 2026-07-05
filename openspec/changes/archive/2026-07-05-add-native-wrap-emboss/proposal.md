## Why

Feature **#7 WRAP-EMBOSS** is the second frontier feature on the finished SSI /
curved-boolean stack (`ROADMAP.md`; `SSI-ROADMAP.md` §Sequencing: `S5 curved
booleans → #7 wrap-emboss`). `cc_wrap_emboss` — wrap a 2D profile onto a cylindrical
face and pad it radially to add (boss) / remove (deboss) material — ALREADY exists
on the ABI, but it is **OCCT-only**: the Phase-3 oracle (`occt_wrap_emboss.cpp`,
GitHub #290) builds the pad as an exact cap-and-side surface set and heals it with
`BRepBuilderAPI_Sewing` + `ShapeFix`. There is NO native wrap-emboss path yet
(`NativeEngine::wrap_emboss` calls `CC_NATIVE_BODY_UNSUPPORTED` and delegates to the
fallback).

#7 is a ~months feature; this change lands ONE concrete, verifiable **first slice**,
honestly scoped: **emboss a RECTANGULAR PAD onto a CYLINDER lateral face**
(`boss = 1`). It is the simplest wrap-emboss that exercises the whole pipeline —
footprint projection onto the cylinder, a wrapped raised pad, and a curved-boolean
fuse — while staying analytic:

- The footprint is a RECTANGLE, so the four projected boundary curves are two
  AXIAL segments (constant angle, straight in `z`) and two CIRCUMFERENTIAL arcs
  (constant `z`, a circular arc on the cylinder) — all closed-form on the cylinder.
- The raised pad is bounded by: two **wrapped side walls** that FOLLOW the cylinder
  (the axial edges swept radially out to `R + height` — each a small ruled patch on
  a radial plane through the axis), two **circumferential walls** (radial-plane
  strips at the constant-`z` boundaries), and an **outer cap** (a trimmed CYLINDER
  face at radius `R + height` spanning the same angular × axial footprint). This is
  exactly the cap-and-side decomposition the OCCT oracle uses, expressed with native
  `math::Cylinder` + planar strips.
- The pad's inner side (the wrapped footprint on the base cylinder at radius `R`) is
  the seam where the pad meets the base solid — the pad-side-wall ∩ cylinder
  intersection is a **curved-boolean / SSI** seam, exactly the machinery
  `src/native/boolean/ssi_boolean.cpp` + `src/native/ssi/` ships (S5 curved-boolean
  fuse; S4-f completeness guards the small feature loop).

So #7's first slice reuses the finished stack: `native-construction`
extrude/revolve to build the pad body, `native-math` `Cylinder` for the wrapped
walls and outer cap, native curved boolean / SSI for the pad∪base fuse, and the
watertight mesher for the weld + self-verify. It matches the OCCT oracle for the
pad-on-cylinder case behind the SAME `cc_wrap_emboss` ABI.

## What Changes

- Add a NATIVE wrap-emboss builder (`src/native/wrapemboss/`, OCCT-free) that runs
  when `cc_wrap_emboss(body, faceId, profileXY, count, height, boss)` selects a
  CYLINDER lateral face of a native body, `boss = 1`, and the profile is a simple
  closed **RECTANGLE** (4 points, positive area, fits within the face's angular ×
  axial extent):
  - **Project the footprint** onto the cylinder: arc-length `px → angle px/R`, axial
    `py` (centred on the face's V-mid) — the unchanged wrapping math the OCCT oracle
    uses.
  - **Build the raised pad** as an explicit surface set: an OUTER CAP = trimmed
    `Cylinder` face at `R + height` over the footprint's angular × axial box; two
    AXIAL side walls that follow the cylinder (radial-plane ruled strips from `R` to
    `R + height` along the two constant-angle edges); two CIRCUMFERENTIAL side walls
    (radial-plane strips along the two constant-`z` edges); tiled into
    deflection-bounded facets (the arcs get enough angular segments so the sagitta
    ≤ deflection) and welded into a watertight pad solid via the boolean
    `assembleSolid`.
  - **Fuse** the pad with the base cylinder solid via the native curved boolean
    (`ssi_boolean_solid` / `native-booleans` FUSE), whose pad-wall ∩ base-cylinder
    seam is the SSI intersection curve (the wrapped footprint on the cylinder). A
    small radial overlap makes the fuse a clean (non-coincident-face) boolean, as in
    the OCCT oracle.
- **Self-verify → OCCT fallback in the engine** (`native_engine.cpp`): add a native
  wrap-emboss dispatch that (a) calls the native builder, then (b) runs a MANDATORY
  self-verify — the result must be a valid watertight solid whose volume is strictly
  GREATER than the base body's (an emboss ADDS material) by a magnitude consistent
  with the wrapped footprint area × height (the same volume-sign discipline the blend
  guard uses, `wantGrow = true`). If the builder returns NULL (out of slice) OR the
  self-verify fails, the engine falls through to the OCCT `cc_wrap_emboss` oracle.
- Native code stays **OCCT-free**; the fuse uses the native SSI substrate, so the
  native path is compiled under `CYBERCAD_HAS_NUMSCI` (like the curved boolean) and a
  no-NUMSCI build simply keeps the OCCT-only behaviour. No `cc_*` signature or POD
  struct changes.

**No `cc_*` ABI change.** `cc_wrap_emboss` keeps its exact signature and semantics;
the rectangular-pad-on-cylinder emboss is a new NATIVE path behind it, OCCT for the
rest (all deboss, all non-rectangular / complex profiles, freeform base,
non-cylindrical faces).

## Capabilities

### New Capabilities
- `native-wrap-emboss`: a native, OCCT-free wrap-emboss FIRST SLICE — emboss a
  RECTANGULAR pad (`boss = 1`) onto a CYLINDER lateral face. It projects the
  rectangular footprint onto the cylinder (arc-length → angle, axial), builds the
  raised pad as a wrapped-side + outer-cap surface set (side walls following the
  cylinder + an outer cap at `R + height`), welds it watertight, and FUSES it with
  the base cylinder solid via the native curved boolean / SSI seam. The engine's
  MANDATORY watertight + volume-INCREASING self-verify accepts it or DISCARDS it →
  OCCT `cc_wrap_emboss` oracle. Consumes `native-construction`, `native-math`,
  `native-booleans`, and `native-ssi` (S5 curved-boolean fuse); compiled under
  `CYBERCAD_HAS_NUMSCI`. Delivered behind the unchanged `cc_wrap_emboss` signature
  via `engine-adapter`. Verified against the OCCT `cc_wrap_emboss` result
  (volume / area / watertight).

### Modified Capabilities
- `wrap-emboss`: record that `cc_wrap_emboss` now has a NATIVE path for the
  rectangular-pad-on-cylinder emboss slice (behind the unchanged signature), with the
  Phase-3 OCCT cap-and-side + healed-sew builder remaining the oracle and the
  fallback for every other case (deboss, non-rectangular / dense profiles, freeform
  base, non-cylindrical faces). The observable ABI contract and volume-change-sign
  semantics are unchanged.

## Impact

- **ABI**: none. `cc_wrap_emboss` keeps its signature and POD structs; the native
  slice is an internal path with OCCT fallback.
- **Build**: adds `src/native/wrapemboss/` (OCCT-free) and a native
  `NativeEngine::wrap_emboss` dispatch. The fuse consumes the SSI substrate, so the
  native path is compiled under `CYBERCAD_HAS_NUMSCI` (like the curved boolean); a
  build without the substrate keeps the current OCCT-only behaviour. The tessellator
  is NOT modified (additive only).
- **Verification**: two gates. **host** (OCCT-free CTest where the fuse is available
  under NUMSCI): the projected footprint lies on the cylinder, the pad + fuse mesh
  watertight, and the volume equals `|body| + wrappedFootprintArea × height` within
  the deflection bound. **sim** native-vs-OCCT parity
  (`scripts/run-sim-native-wrap-emboss.sh`, modelled on `run-sim-phase3-suite.sh` /
  the wrap-emboss harness): the native `cc_wrap_emboss` vs the OCCT `cc_wrap_emboss`
  result on the same rectangular-pad-on-cylinder input (volume / area / watertight
  within tol; report the measured deltas).
- **Roadmap**: implements the `ROADMAP.md` #7 FIRST SLICE and the `SSI-ROADMAP.md`
  `S5 → #7 wrap-emboss` on-ramp (the curved-boolean fuse consumed by an emboss).
  Deboss, complex/dense profiles, freeform-base and non-cylindrical wrap remain
  future #7 work (OCCT).
- **Regression**: additive only. The existing OCCT wrap-emboss (#290, Phase 3) is
  untouched and remains the oracle + fallback; native booleans, SSI S1–S5,
  marching/boolean parity, and the native blends are untouched; the native path only
  fires for the named slice and is gated by the self-verify.
- **Risk / honesty**: honest scope — only the rectangular-pad-on-cylinder emboss
  lands. Every pad face is exact (cylinder + radial planes), the fuse seam is the
  native SSI intersection, and the assembled solid is gated by the engine's
  watertight + volume-INCREASING self-verify. Anything unsafe or out-of-slice returns
  NULL → OCCT, so the slice can never emit a wrong or leaky embossed body. The
  measured OCCT-fallback gap is REPORTED, never masked with a weakened tolerance.
