# Proposal — moat-b2st-smooth-trim-split (MOAT M2b / B2 smooth-trim)

## Why

The byte-frozen B2 face-split verb `splitFace` (`src/native/boolean/face_split.h`)
partitions a trimmed freeform/analytic face along an OPEN seam CHORD that enters one
boundary edge and exits another (`crossings == 2`, a convex straight-edged loop cut by
one clean chord). It DECLINES — `CrossingsNot2`, with the measured `crossings == 0` —
the seam that is a **CLOSED SMOOTH curve interior to the face**: a horizontal plane
slicing a bowl/dome cap, or a circle wrapping a curved wall. That closed-seam case is
exactly what a **curved-wall freeform operand** needs (cut the top off a dome; ring-cut
a wall), and the landed M2 roadmap names it the deferred next enabler:

> "the sole freeform wall of the reachable freeform-SOLID class carries a smooth CLOSED
> (circular) trim, but B2 `splitFace`'s first slice requires a convex straight-edged
> outer loop … the **B2 smooth-trim (closed/circular wall) generalisation remains the
> deferred next enabler.**"

The M2-FUSE / M2-multiseam waves SIDESTEPPED this by choosing operands whose freeform
wall carries a convex straight-edged loop; the smooth-trim case stayed declined. This
change lands it.

## What

One additive OCCT-free header-only verb `src/native/boolean/smooth_trim_split.h`
(`splitFaceSmoothTrim`) — an ADDITIVE SIBLING of `splitFace` (which stays
BYTE-FROZEN, its convex straight-edged path unchanged), mirroring how
`splitFaceJunction` (`junction_split.h`) was added for the valence-3 junction case.

`splitFaceSmoothTrim` partitions a trimmed freeform/analytic face along a **CLOSED
smooth seam interior to the face** into two genuinely-trimmed sub-faces:

- `faceInside` — the disk the seam ENCLOSES (outer wire = the seam loop);
- `faceOutside` — the parent MINUS that disk (the parent outer wire + the seam loop
  as a HOLE wire).

The seam loop is built as one short STRAIGHT edge per traced polyline segment (the
faithful representation of a polyline seam — a curvature-driven edge discretizer
UNDER-samples a single degree-1 B-spline standing in for the whole curved arc), and
laid onto both sub-faces with OPPOSITE orientation, so it is their BIT-EXACT common
boundary. Because the M0 mesher already keeps a triangle iff it is inside the outer
loop AND outside every hole (`trim.h` UVRegion), BOTH sub-faces mesh with NO
tessellator change, and the disk + annulus TILE the parent
(`areaInside + areaOutside == parentArea`).

Every predicate is a geometry test with a mandatory self-verify (closed interior loop,
simple polygon, non-degenerate sub-regions, bit-identical shared seam, rebuild tiling
residual under the SAME strict `SplitOptions` tolerances B2 uses, never weakened). The
still-hard cases DECLINE with a measured blocker: an open chord that crosses the
boundary (`SeamNotInterior`), a self-intersecting loop (`SelfIntersecting`), a
too-short seam (`SeamTooShort`), a non-closed seam (`SeamNotClosed`).

## Impact

- **Additive only.** New file `smooth_trim_split.h`; `face_split.h` (B2 convex path),
  `junction_split.h`, `half_space_cut.h`, `inter_solid_seam.h`, `two_operand.h`,
  `seam_graph.h`, `multi_seam.h`, M0/M1 and the whole tessellator are BYTE-IDENTICAL.
  `src/native/**` stays OCCT-free; no `cc_*` ABI change.
- **Gate A (HOST ANALYTIC, no OCCT)** — `tests/native/test_native_smooth_trim_split.cpp`
  (7/7). A quad-trimmed Bézier bowl sliced by the horizontal plane `z = c` yields a
  CLOSED CIRCULAR seam (the real S3 trace) interior to the quad; the split tiles to
  machine ε at the closed-form disk area `π·ρ²`, both sub-faces mesh watertight and
  their areas converge monotonically to the true curved area at multiple deflections;
  byte-frozen B2 `splitFace` still DECLINES the same seam (the contrast asserted).
- **Gate B (SIM native-vs-OCCT)** — an OCCT face-split oracle for a closed interior
  trim is not cleanly reachable via `BRepAlgoAPI` (it needs a splitter/section wire
  reconstruction, not a solid boolean), so this slice grounds the closed-form partition
  in Gate A and records the sim-oracle limitation honestly; the sim parity lands with
  the downstream weld verb that consumes `splitFaceSmoothTrim` (the freeform half-space
  cut of a dome by a horizontal plane).
- Unblocks the curved-wall freeform boolean whose seam is a closed/circular smooth
  curve — the named M2 enabler.
