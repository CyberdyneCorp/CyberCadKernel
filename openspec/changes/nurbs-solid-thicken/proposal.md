# Proposal — nurbs-solid-thicken (NURBS roadmap Layer 5)

## Why

`docs/NURBS-SCOPE.md` records the NURBS roadmap as a dependency-ordered stack. Layer 5 **surface
OFFSET** (`src/native/math/bspline_offset.{h,cpp}`) is landed: given a NURBS surface `S(u,v)`, it
constructs the offset locus `O(u,v) = S(u,v) + d·N(u,v)` as a fitted non-rational NURBS surface, with
an honest self-intersection (fold) guard. The offset SURFACE is the panel from which **shell /
thicken / hollow** is built — but on its own it is not a solid.

This change adds the next bounded slice of Layer 5: **SOLID THICKEN / SHELL**. Given an OPEN NURBS
surface `S` and a signed thickness `d`, construct the CLOSED, watertight SOLID that `S` bounds when
offset by `d`:

    ∂(solid) = S  ∪  O  ∪  (four ruled SIDE WALLS joining the boundary loops of S and O)

where `O = offsetSurface(S, d)`. This is the core shell workflow (Shapr3D's shell): a thin-walled
solid produced by offsetting a face and stitching the walls into a closed boundary.

It is worth building **now** because it (a) is small and well-bounded (it composes the landed offset
+ evaluators + the kernel's existing closed-shell carrier `tessellate::Mesh`, sewing three panel
kinds that share exact boundary vertices), (b) inherits the offset layer's honest fold guard for
free (a thicken past the curvature radius declines because its offset panel folds), and (c) has an
**airtight, closed-form oracle**: the solid must be WATERTIGHT (χ = 2, no boundary edges), its
enclosed volume must equal `area·|d|` exactly for a flat box (and converge to it for a thin slab /
match the annular-wedge closed form for a cylinder shell), and its offset side must match
`offsetSurface` at distance `|d|`.

## What

A new OCCT-free module `src/native/math/bspline_thicken.{h,cpp}` (namespace
`cybercad::native::math`, beside `bspline_offset`), **numsci-gated** (`CYBERCAD_HAS_NUMSCI`, like
`bspline_offset.cpp`) because it composes `offsetSurface` (whose fit solves through the numsci
facade). It reuses the Layer-1 `BsplineSurfaceData` as input (the surface `S`, which may be rational)
and produces a CLOSED shell as a `tessellate::Mesh` (fp64 `Point3` vertices + indexed `Triangle`s —
the kernel's existing triangle-solid carrier, with `isWatertight` / `enclosedVolume` /
`boundaryEdgeCount` / `isConsistentlyOriented` verification primitives).

`thickenSurface(S, d, tol, gridU, gridV)`:

1. **Guards first** — well-formed input, `|d|` above the linear tolerance. Construct the OFFSET panel
   `O = offsetSurface(S, d, tol)`; a degenerate normal or a self-intersecting (fold) offset propagates
   as a decline (`DegenerateNormal` / `SelfIntersection`) — a thicken NEVER returns a folded solid.
2. **Sample + sew** — tessellate `S` and its offset locus `O = S + d·N` on a shared `(nu × nv)` grid
   into two cap panels; build four ruled side-wall strips joining `S`'s four boundary edges to `O`'s,
   REUSING the exact shared boundary vertices so every seam edge is used by exactly two triangles
   (watertight by construction).
3. **Orient + verify** — propagate a coherent winding across shared edges (BFS), fix global
   inside/out by the signed-volume sign, then VERIFY closure (`isWatertight`, χ = 2, zero boundary
   edges, `isConsistentlyOriented`). A shell that fails closure is DECLINED (`NotClosed`) — never
   returned open or leaky.

## Verification (HOST-analytic, the airtight-oracle is the whole point)

- **HOST (no OCCT), the primary gate** — `tests/native/test_native_nurbs_thicken.cpp`:
  1. **Watertight** — the thickened solid is a closed 2-manifold (`isWatertight`, χ = 2, zero
     boundary edges, `isConsistentlyOriented`) for a flat patch, a curved bump, and a rational
     quarter-cylinder, at both signs of `d`.
  2. **Volume** — enclosed volume == `Lx·Ly·|d|` EXACTLY (~1e-9) for a flat rectangle thickened by
     `d` (a box); converges to `area·|d|` as `|d| → 0` for a curved bump; matches the annular-wedge
     closed form for a cylinder shell.
  3. **Offset side** — every offset-cap vertex lies at distance `|d|` from `S` (projected via
     `closest_point_on_surface`), and the reported `offsetError` equals `offsetSurface`'s; the
     original cap sits on `S`.
  4. **Fold guard** — thickening a tight dome past its minimum radius of curvature is DECLINED
     (`SelfIntersection`, `ok=false`, empty solid), NOT returned folded; a degenerate near-null-normal
     patch declines; a safe small thicken of the same dome succeeds.
- **SIM native-vs-OCCT parity** — OPTIONAL cross-check against OCCT `BRepOffsetAPI_MakeThickSolid`
  (a separate track; HOST is primary and sufficient).

## Scope

- Adds `src/native/math/bspline_thicken.{h,cpp}` — OCCT-free, numsci-gated (`CYBERCAD_HAS_NUMSCI`),
  compiled into the core lib via the existing `src/native` glob.
- Adds `tests/native/test_native_nurbs_thicken.cpp` (host, numsci-gated) wired into CMake mirroring
  `test_native_nurbs_offset`.
- Only `#include`s `bspline.h`, `bspline_ops.h` (Layer 1), `bspline_offset.h` (Layer 5), and
  `tessellate/mesh.h` (the closed-shell carrier + checks) — it does NOT modify them.
- **`cc_*` ABI unchanged.** Layer 5 thicken is an internal geometry-algorithm library; its consumers
  are later shell features, not the app today. No ABI is added until a consumer needs it — consistent
  with the demand-driven policy.

## Non-goals

- **No robust self-intersecting-shell recovery** — recovering a valid thicken from a folded region by
  trimming (rather than declining) is materially harder and remains a documented residual. This module
  DECLINES a folded thicken honestly; it never returns folded geometry.
- **No multi-face shells** — thickening a whole multi-patch B-rep with mitred / rounded corners into a
  single closed solid is a distinct construction and a documented residual. This module thickens a
  SINGLE patch into a six-panel (2 caps + 4 walls) box-topology shell.
- **No rational offset panel** — the offset panel is fitted non-rationally (the input surface may be
  rational, but the offset does not fit weights), inherited from `bspline_offset`.
- No error-driven adaptive knot placement; no new `cc_*` ABI; no change to STEP admission, the
  boolean/blend/ssi/topology modules, or any evaluator signature.
