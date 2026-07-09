# Proposal — moat-m3af-analytic-fillet (MOAT M3, analytic-solid face fillets)

## Why

Three app-facing blend ops still hard-decline every native body and keep the LGPL
engine on their critical path (Class-B readiness rows 84–86):

- **`cc_fillet_face(body, faceId, radius)`** (7 app sites): round every edge that
  bounds the picked face at constant radius. Today `NativeEngine::fillet_face` is a
  hard `CC_NATIVE_BODY_UNSUPPORTED`.
- **`cc_full_round_fillet(body, faceId)`** and **`cc_full_round_fillet_faces(body,
  left, middle, right)`**: replace a narrow middle face with a full tangent round
  between its two neighbour walls (consuming the middle face). Both hard-decline.

The tractable ANALYTIC family here is a pure **assembly of the landed native blend
substrate**, not new geometry:

- **`full_round_fillet[_faces]`** on a PRISMATIC rib — a narrow planar middle face
  between two PARALLEL planar walls — is a cylindrical cap of radius r = strip-width/2,
  axis on the strip mid-plane at distance r from each wall, tangent (G1) to both. It
  is the r = w/2 special case of the rolling-ball construction; it consumes the middle
  face and welds tangent to the two walls via `nblend::fillet_edges` on the TWO
  OPPOSITE (non-corner-sharing) seam edges — a config the landed multi-edge blend
  welds robustly. **THIS LANDS.**

Measured decline (honest, gates on M2):

- **`fillet_face`** per the ABI/OCCT semantics rounds EVERY edge bounding the picked
  face. On an all-planar convex solid every face is a CLOSED loop of ≥3 edges that
  pairwise share CORNERS, and rounding two edges that meet at a corner needs a
  SPHERICAL corner patch between their two cylinder blends — the corner weld the
  landed multi-edge `fillet_edges` does NOT build (it welds only NON-adjacent edge
  sets; adjacent edges return NULL, MEASURED in the host gate). So a full-face fillet
  on a planar solid is **honestly DECLINED this wave with a measured reason** and
  forwards to OCCT. The native PATH is fully wired + self-verified, so `fillet_face`
  lands automatically the moment M2 supplies the corner-sphere weld — with no engine
  change.

## What changes

- **New OCCT-free header `src/native/blend/fillet_face.h`** — `fillet_face(solid,
  faceId, radius)` collects the convex bounding edges of the picked planar face and
  applies the landed `nblend::fillet_edges` multi-edge blend to them; declines a
  non-planar picked face, a non-all-planar solid, no convex edge, or a weld the
  substrate cannot close (`WeldGatesM2` — the corner-sphere patch, this wave's
  measured decline) (`FilletFaceDecline`). The path is ready; it lands on M2.
- **New OCCT-free header `src/native/blend/full_round.h`** — `full_round_fillet(solid,
  faceId)` auto-detects the two longest opposite edges of the middle face and their
  across-neighbours (mirroring the OCCT `resolveAuto`); `full_round_fillet_faces(solid,
  left, middle, right)` uses the shared seam edges. For two PARALLEL planar walls it
  builds the r = w/2 tangent-cylinder cap, consumes the middle strip, and welds. A
  DIHEDRAL (non-parallel) middle, a curved wall, a non-planar middle, or a
  closed-seam/annulus configuration DECLINES (`FullRoundDecline`).
- **`NativeEngine::fillet_face` / `full_round_fillet` / `full_round_fillet_faces`**
  now serve native bodies via these builders behind the existing facade seam, gated by
  the SAME `blendResultVerified` self-verify the landed fillet ops use (fillet_face:
  SHRINK; full_round: SHRINK). A native body outside the analytic domain returns the
  same honest decline the prior hard-decline produced (a native void is NEVER handed
  to OCCT). An OCCT body forwards to the OCCT oracle unchanged.

## Impact

- The landed blend substrate (`fillet_edges.h`, `curved_fillet.h`, `blend_geom.h`)
  and M0/M1/M2 remain **byte-frozen**; the two new headers are additive siblings.
- `src/native/**` stays OCCT-free (descriptive comments only).
- `cc_*` ABI unchanged (native paths only, behind existing facade seams).
- The tessellator (`src/native/tessellate/`) is consumed as-is, UNTOUCHED.
- Readiness rows 84–86 move the analytic families toward A; the freeform / dihedral /
  closed-seam residual stays OCCT (gated on M2 freeform booleans + closed-seam weld).
