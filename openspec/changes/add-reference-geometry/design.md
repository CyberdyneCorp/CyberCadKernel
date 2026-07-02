## Context

The app needs datum planes and axes to place sketches and features. These are
lightweight, exact, analytic constructions — a point plus a unit normal (plane)
or a point plus a unit direction (axis) — and do NOT require a B-rep result, so
they are returned as POD out-arrays (`double[6]`) with a `1`/`0` success flag,
matching the existing `cc_face_axis` / `cc_bounding_box` style. The kernel today
has only `cc_face_axis`; this change fills in the rest.

Two families:
- **Point-only (pure math)**: plane-from-3-points, offset-plane,
  axis-from-2-points. No topology needed → implemented with plain fp64 vector
  math so they work identically in the no-OCCT host build and the OCCT build.
- **Derived (needs geometry)**: plane-from-planar-face, axis-from-linear-edge,
  axis-from-cyl/cone-face. These read an existing body's surface/curve, so they
  go through `IEngine` and are implemented by the OCCT adapter (the cyl/cone case
  reusing the exact `cc_face_axis` logic); the stub leaves them unsupported.

Constraints:
- **ABI stability**: six additive entry points only; no existing signature or POD
  layout changes (`tests/test_abi.cpp` must still match `KernelBridgeAPI.h`).
- **Host build must stay green**: the point-only constructors must compile and
  pass with OCCT off; the derived ones return `0` in the stub, not crash.
- **Exactness**: results are checked against known analytic values, so the math
  must be exact fp64 (normalize with a guarded zero-length check).

## Goals / Non-Goals

Goals:
- Six additive constructors returning origin + unit normal (plane) or origin +
  unit direction (axis) as `double[6]`, `1`/`0` success.
- Point-only constructors correct in the host build (no OCCT).
- Derived constructors correct in the OCCT build, reusing `cc_face_axis` for the
  cyl/cone face axis.
- Exact analytic checks + degenerate-input failure checks.

Non-Goals:
- Any B-rep datum object / persistent datum feature in the shape registry (the
  app owns datum objects; the kernel only computes their geometry).
- Datum geometry from more exotic inputs (tangent-to-curve, angle-between-planes,
  three-tangent, etc.) — future additive constructors if needed.
- A `gp_Ax2`/frame (two axes) return; a single normal/direction is enough for the
  sketching use-case.

## Decisions

- **POD out-array + success flag, no handle.** Each constructor writes
  `double[6]` and returns `1`/`0`, mirroring `cc_face_axis`. `out6` for a plane is
  `[ox,oy,oz, nx,ny,nz]`; for an axis `[ox,oy,oz, dx,dy,dz]`. The normal/direction
  is always unit-length on success.
- **Plane from 3 points**: `origin = p0`, `n = normalize((p1-p0) x (p2-p0))`.
  If `|(p1-p0) x (p2-p0)| < eps` the points are colinear/coincident → return `0`.
- **Offset plane**: `n = normalize(normal)` (return `0` if `|normal| < eps`),
  `origin = origin + dist * n`. `dist` may be negative.
- **Axis from 2 points**: `origin = a`, `d = normalize(b-a)`; return `0` if
  `|b-a| < eps`.
- **Point-only constructors live facade-side (or a shared `ref_geometry` math
  helper).** They do not call `active_engine()`, so they work regardless of which
  engine is active (including the stub). This keeps them exact and build-agnostic.
- **Derived constructors route through `IEngine`.** New virtuals
  `ref_plane_from_face(body, faceId)`, `ref_axis_from_edge(body, edgeId)`,
  `ref_axis_from_face(body, faceId)` return `Result<std::vector<double>>` (6
  values), default `engine_unsupported`. The OCCT adapter overrides them in
  `occt_query.cpp`:
  - plane-from-face: `BRepAdaptor_Surface`; require `GeomAbs_Plane`; take
    `gp_Pln` position location + axis direction.
  - axis-from-edge: `BRepAdaptor_Curve`; require `GeomAbs_Line`; take the line's
    location + direction.
  - axis-from-cyl/cone-face: **reuse the existing `face_axis` implementation**
    (same code path as `cc_face_axis`) so the datum axis of a cylinder/cone is
    identical to the query result.
  The stub inherits the unsupported default → `0`.
- **Eps tolerance.** A single small absolute tolerance (e.g. `1e-9` on the
  cross-product / difference magnitude in mm) gates degeneracy; documented in the
  helper. Chosen so a genuinely tiny-but-valid triangle still passes while exactly
  colinear/coincident input fails.

## Risks / Trade-offs

- **Colinear tolerance choice.** Too loose rejects thin-but-valid triangles; too
  tight accepts near-degenerate input. Mitigated by tying the check to the
  cross-product magnitude (area∝) and documenting the constant; the checks
  include a clearly-colinear case that must fail and a clearly-valid case that
  must pass.
- **Face/edge constructors unavailable in host.** By design — they need geometry;
  the stub returns `0`. Documented so callers know the point-only trio is the
  host-portable subset.
- **Normal orientation.** The 3-point normal follows the winding `p0→p1→p2`
  right-hand rule; the plane-from-face normal follows the face's surface
  orientation. Documented so callers know the sign convention; magnitude/axis
  correctness is unaffected.

## Migration Plan

1. Add a shared `ref_geometry` fp64 math helper (cross/normalize with guarded
   zero-length) and wire the three point-only constructors into the facade.
2. Add the three derived `IEngine` virtuals (default `engine_unsupported`) and
   override them in the OCCT adapter (`occt_query.cpp`), reusing `face_axis` for
   the cyl/cone case.
3. Add the six entry points to `include/cybercadkernel/cc_kernel.h` +
   `src/facade/cc_kernel.cpp`; keep all existing signatures unchanged.
4. Host CTest: exact analytic checks for the point-only trio + degenerate-input
   failures; confirm the derived trio returns `0` in the stub.
5. iOS-sim: analytic checks for the derived trio (plane-from-planar-face on a box
   face equals the known face normal; axis-from-linear-edge equals the known edge
   direction; axis-from-cyl-face equals `cc_face_axis`).
6. `tests/test_abi.cpp` still matches `KernelBridgeAPI.h`.
7. `openspec validate --all --strict`; update `ROADMAP.md` Phase 3 status.

## Open Questions

- The exact degeneracy tolerance constant (`eps` on cross-product / difference
  magnitude) — pin empirically so the analytic checks pass and a clearly-colinear
  case fails; document alongside the helper.
- Whether the app also wants a two-axis frame (`gp_Ax2`) return in a follow-up;
  out of scope here (single normal/direction suffices for datum planes/axes).
