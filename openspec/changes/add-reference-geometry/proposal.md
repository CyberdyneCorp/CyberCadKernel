## Why

Sketching and feature placement in the app need **datum reference geometry** —
construction planes and axes derived from existing geometry or from raw points —
before a feature is built (`ROADMAP.md` Phase 3; cross-refs the CyberCad
`add-datum-plane-sketching` change). Today the kernel exposes only `cc_face_axis`
(the axis of a cylindrical/conical face); there is no way to get a datum plane
from three points, an offset plane, a plane from a planar face, or an axis from a
linear edge / two points. These are cheap, exact, analytic constructions that do
not need a B-rep result — they return a point plus a unit direction/normal — yet
the app currently has to recompute them ad hoc.

This is the **easiest** Phase-3 item and the natural first one: it is pure
double-precision math for the point-only constructors, so those work even in the
no-OCCT host build; only the face/edge-derived constructors need OCCT (to read the
surface/curve), and they reuse the existing `cc_face_axis` logic for the
cylinder/cone case.

## What Changes

- Add additive `cc_*` reference-geometry constructors that each fill a caller
  `double*` out-array and return `1` on success / `0` on failure (the same style
  as `cc_face_axis` / `cc_bounding_box`). No shape handle is created or returned.
  - **Plane from 3 points** — `cc_ref_plane_from_points(p0[3], p1[3], p2[3],
    out6)` → `out6 = [ox,oy,oz, nx,ny,nz]`: origin at `p0`, unit normal
    `normalize((p1-p0) x (p2-p0))`. Colinear / coincident points return `0`.
  - **Offset plane** — `cc_ref_plane_offset(origin[3], normal[3], dist, out6)`:
    same unit normal, origin moved by `dist` along the unit normal. A
    zero-length normal returns `0`.
  - **Plane from a planar face** — `cc_ref_plane_from_face(body, faceId, out6)`:
    the face's plane (origin + unit normal). Non-planar / unknown face returns `0`.
    OCCT-only.
  - **Axis from 2 points** — `cc_ref_axis_from_points(a[3], b[3], out6)` →
    `out6 = [ox,oy,oz, dx,dy,dz]`: origin `a`, unit direction `normalize(b-a)`.
    Coincident points return `0`.
  - **Axis from a linear edge** — `cc_ref_axis_from_edge(body, edgeId, out6)`:
    the edge's line origin + unit direction. Non-linear / unknown edge returns
    `0`. OCCT-only.
  - **Axis from a cylindrical/conical face** — `cc_ref_axis_from_face(body,
    faceId, out6)`: the surface axis, **reusing the `cc_face_axis` logic**.
    OCCT-only.
- **Pure-math constructors run in the host build**: `cc_ref_plane_from_points`,
  `cc_ref_plane_offset`, `cc_ref_axis_from_points` are implemented in the facade
  (or a shared math helper) with no engine dependency, so they return correct
  results in the no-OCCT host stub as well as the OCCT build.
- **Derived constructors route through `IEngine`**: `cc_ref_plane_from_face`,
  `cc_ref_axis_from_edge`, `cc_ref_axis_from_face` add new `IEngine` virtuals
  (default `engine_unsupported`), overridden by the OCCT adapter; the stub
  inherits the unsupported default so they return `0` in the host build.

C ABI change is **ADDITIVE only**: six new entry points; no existing `cc_*`
signature or POD struct layout changes, so `tests/test_abi.cpp` still matches
`KernelBridgeAPI.h`.

## Capabilities

### New Capabilities
- `reference-geometry`: analytic datum planes and axes returned as POD
  (origin + unit normal / origin + unit direction) with no shape handle —
  plane-from-3-points, offset-plane, plane-from-planar-face, axis-from-2-points,
  axis-from-linear-edge, and axis-from-cylindrical/conical-face (reusing
  `cc_face_axis`). The point-only constructors are exact double-precision math
  available in every build (including the no-OCCT host); the face/edge-derived
  constructors use OCCT and are unsupported (return `0`) in the stub.

## Impact

- **Contract**: purely additive; cross-refs the CyberCad
  `add-datum-plane-sketching` change and complements the existing
  `cc_face_axis`. No observable change to any existing `cc_*`.
- **App**: gains a stable kernel source of datum planes/axes it can build
  sketches and features against, instead of recomputing them app-side.
- **Build**: point-only constructors compile and pass in the host build; the
  face/edge constructors are guarded behind the OCCT adapter (`#ifdef
  CYBERCAD_HAS_OCCT` in the adapter TU) and are safe no-ops in the stub.
- **Precision / determinism**: exact fp64 analytic math; results are
  deterministic and independently checkable against known normals/directions.
- **Risk**: minimal — the math is closed-form; the only failure modes
  (degenerate/colinear input, non-planar face, non-linear edge) return `0` and
  are covered by the checks.
