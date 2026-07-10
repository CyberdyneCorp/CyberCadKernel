# Design — moat-draft-angle (MOAT feature — draft angle)

## The geometry: pivot on the trace with the neutral plane

A drafted planar face `F` has outward unit normal `n̂_F` and a point `o_F` on it. The
NEUTRAL plane `N` has origin `o_N` and unit normal = the PULL direction `p̂`. The face's
trace on the neutral plane is the line

    L = F ∩ N,   direction t̂ = normalize(n̂_F × p̂).

A draft PIVOTS `F` about `L`: the trace lies on the neutral plane (which does not move),
so it stays fixed, and the face tilts by the draft angle `θ` about `t̂`:

    n̂' = Rot(t̂, φ) · n̂_F,   φ = sign(h_C) · θ,

where `h_C = (centroid − o_N)·p̂` is the signed height of the face centroid above the
neutral plane. The sign makes a POSITIVE angle draw material IN as the face recedes from
the neutral plane along `+p̂` (the standard mold-release taper and OCCT
`BRepOffsetAPI_DraftAngle`'s convention). The tilted target plane passes through the foot
of the centroid on `N`, `footN = centroid − h_C·p̂`, which lies on both `N` and `F`, hence
on `L` — so the plane through `footN` with normal `n̂'` contains the whole pivot line.

## Why inward-trim-and-verify, not per-face DM2 re-solve

A draft only REMOVES stock, so every drafted face moves inward and drafting it is a pure
TRIM — no grow slab is needed. The first design re-used DM2 `replaceFaceToPlane` per face,
but DM2's output is a boolean result whose coplanar faces are TRIANGULATED; a second DM2
step re-reads a triangle fragment (area of a half-face) and its closed-form
`V₀ + A_F·d̄` volume oracle then rejects the correct solid (DIAGNOSE finding: the
distinct-plane-preserved candidate fails the area-based volume gate). The robust design
instead derives EVERY drafted plane up front from the UNTOUCHED original solid (ids stable,
faces un-fragmented), then applies them as a sequence of `nb::splitByPlane` inward cuts.
Each cut self-verifies watertight on its own (inherited DM1 robustness); the composite is
then re-audited: watertight closed 2-manifold, single lump (Euler χ = 2), consistently
oriented (`tess::isConsistentlyOriented`), and positive volume STRICTLY smaller than the
original (a draft removes material). Anything that fails DECLINES — never a leaky /
self-intersecting / grown solid.

## Two gates (OCCT is the oracle)

- **(a) HOST analytic (no OCCT):** a box side face drafted by θ about the base plane becomes
  a trapezoid at the exact taper — the removed material is a CLOSED-FORM WEDGE
  `½·H·(H·tanθ)·D`, so `V' = V₀ − ½·H·(H·tanθ)·D` is asserted fp-exact; all four side faces
  drafted give a truncated-pyramid FRUSTUM `(H/3)(A_bot + A_top + √(A_bot·A_top))`; an
  off-axis (rotated) box side face gives the same wedge (rigid-motion invariant). The
  honest-decline envelope (curved base / non-planar neutral / cap face perpendicular to the
  pull / degenerate angle) returns NULL with a measured reason.
- **(b) SIM native-vs-OCCT (booted simulator):** the SAME drafts vs the OCCT oracle
  `BRepOffsetAPI_DraftAngle` + `BRepGProp`, comparing volume / area / watertight / Euler χ /
  bbox / one-sided Hausdorff, including the off-axis fixture (the oracle is the axis-aligned
  draft rigidly rotated) and the cap-face honest decline.

## ABI

`CCShapeId cc_draft_faces(CCShapeId body, const int *faceIds, int faceCount, const double
*neutralOrigin, const double *pullDir, double angleDeg)`. Additive; signature-styled like
`cc_replace_face` / `cc_fillet_edges`. `angleDeg` in degrees (converted to radians at the
engine boundary). Returns 0 on an honest decline (with `cc_last_error` set). The native
engine serves an all-planar body; a curved/foreign body forwards to OCCT.

## Non-goals

Curved-base draft (a cylinder/cone/sphere wall tapered about a neutral plane), a non-planar
neutral surface, a face perpendicular to the pull axis (a cap — no trace line), a
degenerate or ≥90° angle, and a self-intersecting draft all remain OCCT — declined with a
measured reason, never faked. Variable per-face draft angles and stepped/parting-line draft
are out of this slice.
