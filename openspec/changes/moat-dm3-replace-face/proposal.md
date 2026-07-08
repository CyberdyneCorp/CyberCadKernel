# Proposal — moat-dm3-replace-face (MOAT M-DM, DM3 + DM4)

## Why

Two direct-modeling ops still hard-decline every native body and keep the LGPL
engine on their critical path:

- **DM3 — general `cc_replace_face(body, faceId, offset, tiltDeg)`** (the app's
  push/pull "move a face"): retarget a planar face by offsetting it along its own
  outward normal and tilting it about the face's parametric X-axis, trimming the
  solid to the new plane. Today `NativeEngine::replace_face` is a hard
  `CC_NATIVE_BODY_UNSUPPORTED` (Class-B readiness row 81); the sibling DM2
  `cc_replace_face_to_plane` is already native.
- **DM4 — the project tool**: drop a point onto a face's analytic surface and read
  the foot-of-perpendicular + distance. No `cc_*` entry exists yet; the app's
  project / measure path can only reach OCCT `GeomAPI_ProjectPointOnSurf`.

Both are bounded **assembly-of-landed-parts** slices, not new geometry:

- DM3 DERIVES the target plane from the picked face's own plane and re-solves via
  the already-landed, already-two-gate-verified DM2 `replaceFaceToPlane`
  (grow-then-trim = 1 Fuse + 1 Cut, each individually watertight self-verified).
- DM4 is the CLOSED-FORM normal projection onto the three analytic surface kinds
  (plane / cylinder / sphere), read straight off the native B-rep surface
  (`topology::surfaceOf`), mirroring OCCT `GeomAPI_ProjectPointOnSurf` on the
  untrimmed surface.

## What changes

- **New OCCT-free header `src/native/directmodel/replace_face_general.h`** —
  `replaceFaceOffsetTilt(solid, faceId, offset, tiltDeg)` derives the target plane
  `(o + n̂_F·offset, n̂_F)` for the pure-offset slice (tiltDeg ≈ 0) and re-solves via
  DM2. A non-zero tilt (OCCT face-parametrization X-axis — a foreign convention),
  a non-planar picked face / non-all-planar solid, and an unverifiable no-op
  offset honestly DECLINE → OCCT.
- **New OCCT-free header `src/native/directmodel/project.h`** —
  `projectPointOnFace(solid, faceId, p)` returns the closed-form foot + distance
  for a plane / cylinder / sphere face. Cone / torus / freeform, and the ambiguous
  poses (point on a cylinder axis / at a sphere centre), DECLINE.
- **`NativeEngine::replace_face`** now serves native bodies via
  `replaceFaceOffsetTilt` (was a hard decline), keeping the OCCT-body forward and
  the honest native decline; **`NativeEngine::project_point_on_face`** (new
  override) serves native bodies via `projectPointOnFace`.
- **Additive `cc_project_point_on_face` + `CCProjection`** in the C ABI; the OCCT
  adapter `OcctEngine::project_point_on_face` is the `GeomAPI_ProjectPointOnSurf`
  oracle. No existing `cc_*` signature changes.

## Impact

- DM1/DM2 (`split_plane.h`, `replace_face.h`) and M0/M1/M2 remain **byte-frozen**;
  the two new headers are additive siblings.
- `src/native/**` stays OCCT-free (descriptive comments only).
- Readiness row 81 (`replace_face`) flips **B → A**; a new project row lands A for
  the analytic slice; both honest-declined outside it.
