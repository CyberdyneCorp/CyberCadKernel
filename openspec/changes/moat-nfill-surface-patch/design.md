# Design — moat-nfill-surface-patch

## The reachable slice

Filling an N-sided hole is, for the bounded analytic case, an **evaluate-and-weld** slice
built entirely on landed substrate — not new geometry infrastructure:

1. **Boundary sampling (analytic).** Each boundary edge is a straight segment or a circular
   arc. Sampling a segment is a lerp; sampling an arc is `center + R·(cos θ·û + sin θ·v̂)`
   over the arc's angular span. `gridN` samples per side gives the boundary polylines that
   both define the patch and weld it to the hole. No NURBS evaluation is required.

2. **Patch interpolation (Coons / Gregory).** For N=4 the discrete **Coons patch** is the
   textbook transfinite blend:
   `S(u,v) = (1−v)·C₀(u) + v·C₂(u) + (1−u)·C₃(v) + u·C₁(v) − bilinear(corners)`,
   evaluated on a `(gridN+1)²` grid. For N=3 one logical side degenerates to a corner
   (a triangular Coons patch). For N=5,6 there is no rectangular (u,v) domain, so we use a
   **Gregory-style convex combination**: parameterize the N-gon interior by generalized
   barycentric (mean-value) coordinates, and blend the per-side Coons contributions weighted
   by those coordinates. The result is a triangle-grid `tess::Mesh`. This is the ENTIRE
   surface machinery — a fixed transfinite formula over the sampled boundary curves, NOT a
   general NURBS solver.

3. **Bit-exact boundary.** The boundary rows/columns of the grid are the boundary samples
   themselves (assigned, not recomputed), so the patch shares the hole-boundary points to
   the last bit — the precondition for the weld to close.

4. **Weld + self-verify.** For `fillHoleSolid`, the shell's existing faces become
   `boolean::Polygon`s (via `boolean::extractPolygons`), the patch triangles become
   one-triangle `Polygon`s, and `boolean::assembleSolid` welds the whole bag into a Solid on
   its shared vertices (weld tol 1e-7 — the patch boundary samples are bit-exact so they land
   in the same weld cell as the shell's boundary corners). The `heal::self_verify` gate
   (watertight across the deflection ladder + positive volume) + `tess::isConsistentlyOriented`
   is the authoritative closure check — the builder never trusts its own bookkeeping.

## Why this is not a general NURBS kernel

The patch is only ever *evaluated to a mesh*. Nothing stores or re-fits a NURBS surface;
nothing solves a global surface-fit / energy-minimization; there is no trimmed-surface
representation. The only "surface" the kernel gains is a fixed Coons/Gregory formula
sampled on a grid. A boundary that is not made of straight/arc analytic edges cannot be
sampled by this machinery and is declined. This is the same discipline the campaign held
for every bounded slice: reuse the mesh/weld substrate, prove watertight, decline the tail.

## ABI shape: mesh-backed patch body

`cc_fill_ngon(boundaryXYZ, cornerCount, edgeKinds, arcMids, gridN)` returns a **mesh-backed
body** (the tessellated patch surface), exactly the body kind `wrapNativeMesh` already
serves for imported STL soups — `cc_mass_properties` (area), `cc_bounding_box`,
`cc_tessellate`, `cc_face_meshes` all branch on `holder->isMesh` and serve it directly.
This is honest: OCCT `BRepFill_Filling` also produces a FACE (an open surface), so the
parity oracle compares the patch SURFACE (area / bbox / boundary-coincidence), and the
watertight-solid completion is proven in the host gate via `fillHoleSolid`.

Boundary encoding across the ABI (all POD arrays, mirroring the point-array construct ops):
- `boundaryXYZ` — `cornerCount` corner points (x,y,z), the loop in order (implicitly closed).
- `edgeKinds[i]` — 0 = straight to next corner, 1 = circular arc to next corner.
- `arcMids` — for each arc edge, a mid-arc point (x,y,z) fixing the arc plane + bulge; NULL
  when there are no arcs.
- `gridN` — samples per side (≥ 2); the tessellation density the convergence gate sweeps.

## Two-gate verification plan

- **Host GATE (a)** — OCCT-free closed form. Planar quad hole in a box → `fillHoleSolid`
  restores the box volume EXACTLY (≤ 1e-9 rel) watertight/χ=2/oriented; planar N-gon → patch
  area = polygon area; saddle 4-sided analytic boundary → on-boundary residual ≤ 1e-9,
  deviation bounded and CONVERGING as `gridN` doubles; non-analytic / 7-sided / degenerate →
  measured `NGonDecline`.
- **SIM GATE (b)** — native vs OCCT on the booted simulator. Same boundary filled by
  `cc_fill_ngon` (native) and by `BRepFill_Filling` (OCCT, called directly in the `.mm`),
  compared on patch area (deflection-bounded for curved boundaries, exact for planar),
  bbox, and boundary-coincidence (every native boundary sample lies on the OCCT face
  boundary within tolerance).

## Honest-decline envelope (measured, → OCCT)

`NonAnalyticBoundary` (spline edge), `TooManySides` (N<3 or N>6), `DegenerateBoundary`
(zero-length / duplicate corner), `SelfIntersecting` (patch grid self-crosses or the weld
fails self-verify), `NotConverged` (a G2 request the bounded patch cannot honestly meet).
The engine returns 0 / NULL with the reason; a native void is NEVER forwarded to OCCT.

## Complexity note

The patch evaluator, the boundary sampler, and the loop tracer are each short linear
assemblers; the transfinite blend is a fixed formula. Cognitive complexity stays within the
compilers/geometry band (≤ ~25) — no systems-band function is introduced.
