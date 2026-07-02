## Why

`cc_wrap_emboss` wraps a 2D profile onto a cylindrical face and pads it in/out to
emboss (boss) or deboss the body (`ROADMAP.md` Phase 3; GitHub #290;
`occt-usage` §Offsets/sweeps/lofts). Today the pad is built with
`BRepOffsetAPI_ThruSections` between an inner and an outer wrapped wire (see
`occt_construct.cpp` / the app's `KernelBridge.mm`). ThruSections is fragile on a
**dense, high-curvature** wrapped profile: the two lofted section wires have many
near-tangent vertices around the cylinder, and the ruled loft frequently produces
a self-intersecting or non-manifold shape that `BRepCheck_Analyzer::IsValid`
rejects — so the emboss silently fails (returns `0`) exactly when the profile is
detailed enough to matter. The current code already falls back from the dense
wire to a coarse polygon, which regresses fidelity.

This change makes the wrap-emboss pad **robust**: build the pad as an explicit
cap-and-side surface set (inner cap, outer cap, and side walls between the two
wrapped boundaries) and **sew + heal** it into a closed solid
(`BRepBuilderAPI_Sewing` then `ShapeFix_Shell`/`ShapeFix_Solid`), instead of
relying on ThruSections to loft a valid solid in one shot. Sewing tolerant faces
and healing gaps is far more forgiving of the dense high-curvature case, so the
resulting pad is a VALID solid that fuses/cuts cleanly.

## What Changes

- Add an **internal robust wrap-emboss pad builder** in the OCCT adapter:
  - Wrap the (densified) profile onto the cylinder at the inner radius `rIn` and
    the outer radius `rOut` (unchanged wrapping math: arc-length `px` → angle
    `px/R`, axial `py`, centred on the face's V-mid).
  - Build the pad as explicit faces: an **inner cap** (the wrapped profile face
    at `rIn`), an **outer cap** (at `rOut`), and **side walls** — one ruled/filled
    face per profile edge connecting the `rIn` boundary to the `rOut` boundary
    (`BRepFill` / `BRepBuilderAPI_MakeFace` on the four corner points, or a
    `GeomFill` ruled surface).
  - **Sew** all faces (`BRepBuilderAPI_Sewing` at a tolerance tied to the feature
    size), then **heal** (`ShapeFix_Shell` + `ShapeFix_Solid`, orient the shell
    closed) into a solid, and gate on `BRepCheck_Analyzer::IsValid`.
- **Keep `cc_wrap_emboss` signature and semantics.** The robust builder replaces
  the ThruSections pad on the primary path; the existing ThruSections build is
  retained ONLY as a coarse fallback if the sewn build fails, so the operation
  never regresses relative to today (it can only get more robust).
- **Fuse (boss) / cut (deboss) unchanged**: `boss=1` → `rIn=R, rOut=R+depth`,
  fuse; `boss=0` → `rIn=R-depth, rOut=R`, cut — as today.
- Fall back cleanly: if neither the sewn pad nor the coarse ThruSections pad
  yields a valid solid, return `0` (as today) and record the reason in
  `cc_last_error`.

No C ABI change: `cc_wrap_emboss` keeps its exact signature. The robust path is
internal to the OCCT adapter, guarded by `#ifdef CYBERCAD_HAS_OCCT`; the host stub
remains a safe no-op (`0`).

## Capabilities

### New Capabilities
- `wrap-emboss`: a robust cylinder wrap-emboss that builds the pad as a
  cap-and-side surface set and sews + heals it (`BRepBuilderAPI_Sewing` +
  `ShapeFix`) into a valid closed solid before the boss/deboss boolean, so a
  dense / high-curvature wrapped profile still yields a watertight
  `BRepCheck_Analyzer::IsValid` result with the correct volume-change sign — with
  the previous `BRepOffsetAPI_ThruSections` build retained only as a coarse
  fallback and the honest-fallback rule applied when even that fails. Delivered
  behind the unchanged `cc_wrap_emboss` signature via `engine-adapter`.

## Impact

- **Contract**: strengthens `occt-usage` §Offsets/sweeps/lofts and closes #290
  without changing the `cc_wrap_emboss` observable signature; the result is a
  valid solid where the old path produced an invalid/failed shape.
- **App**: no code change — `cc_wrap_emboss` returns a valid embossed/debossed
  body for profiles that previously failed.
- **Build**: OCCT-only (sewing/ShapeFix are OCCT); guarded by `CYBERCAD_HAS_OCCT`;
  the host stub is unchanged (`0`).
- **Determinism**: the pad is built from a fixed face decomposition and a fixed
  sew order; results are reproducible.
- **Risk / honesty**: if a specific dense profile cannot be sewn into a valid
  solid, the operation FALLS BACK to the coarse ThruSections pad (still valid, at
  reduced fidelity) and that case is marked **deferred** with a note — never a
  faked pass.
