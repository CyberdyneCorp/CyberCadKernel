# Tasks — add-native-wrap-emboss (#7 wrap-emboss, first slice)

Verification levels: **host** = OCCT-free host CTest (footprint-on-cylinder +
watertight + closed-form added-volume; the fuse path is available under
`CYBERCAD_HAS_NUMSCI`); **sim** = native-vs-OCCT `cc_wrap_emboss` parity on a booted
simulator (volume / area / watertight / `IsValid` within tol). No `cc_*` signature is
added — the native path lands behind the existing `cc_wrap_emboss`.

> Implementation note (honest deviation from the first-draft plan): the raised pad is
> NOT built as a standalone solid then fused via `ssi_boolean_solid`. The S5-a SSI fuse
> handles ONE transversal *elementary* curved solid pair; the pad carries planar walls +
> a cylindrical cap, so it is not a single elementary solid and the SSI fuse cannot drive
> it robustly. Instead — mirroring the proven #6 curved-fillet slice — the WHOLE embossed
> solid is rebuilt as one deflection-bounded planar-facet soup that shares vertices
> exactly along every seam and welds watertight through the SAME boolean `assembleSolid`.
> The pad-wall ∩ cylinder seam is expressed by shared footprint-boundary vertices (the
> base cylinder wall is retiled over the full turn with the footprint window removed), so
> there is no fragile boolean and NO doubled/coincident faces. This is header-only,
> OCCT-free (`src/native/feature/wrap_emboss.h`), and NUMSCI-independent (no S3 substrate
> needed), which is strictly better than the SSI fuse for this slice. Everything else in
> the plan (classify → wrap → cap-and-side walls → engine self-verify → OCCT fallback)
> holds.

## 1. Classify the rectangular-pad-on-cylinder emboss
- [x] 1.1 In the native wrap-emboss builder, classify the input: `boss == 1`, native
  body, `faceId` resolves to a `FaceSurface`-kind `Cylinder` lateral face (radius `R`,
  axis `A`, V-mid), and the profile is a closed RECTANGLE (4 pts, positive area). Any
  mismatch ⇒ return NULL (→ OCCT). (**host**)
- [x] 1.2 Guard: `height > 0`; footprint angular span `< 2π − ε`; axial span within
  the face's extent; no self-overlap. Any failure ⇒ NULL → OCCT. (**host**)

## 2. Project the footprint onto the cylinder
- [x] 2.1 Map each corner `(px,py)` to `u = px/R`, `v = py + vMid`; the two axial
  edges are straight in `v`, the two circumferential edges are circular arcs on the
  cylinder. (Corners lie on the cylinder by construction — `ringPoint` evaluates the
  cylinder surface.) (**host**)
- [x] 2.2 Tile the two circumferential arcs into deflection-bounded chords (angular
  sagitta `R(1−cos Δu/2) ≤ deflection`, `sagittaSteps`/`uSamples`). (**host**)

## 3. Build the raised pad (wrapped side walls + outer cap)
- [x] 3.1 OUTER CAP: a `Cylinder` patch at radius `R + height` over the footprint's
  `u × v` box, tiled into deflection-bounded triangles (`emitOuterCap`). (**host**)
- [x] 3.2 AXIAL side walls (×2): radial-plane strips from `R` to `R+height` along the
  two constant-angle edges (`emitAxialWalls`, planar — contain the axis). (**host**)
- [x] 3.3 CIRCUMFERENTIAL side walls (×2): planar strips at the constant-`z` edges,
  `ρ ∈ [R, R+height]`, tiled along `u` (`emitCircWalls`). (**host**)
- [x] 3.4 Close the embossed solid: the base cylinder wall is retiled over the full turn
  with the footprint WINDOW removed (`tileWallWithWindow`) + the two end caps, so the pad
  walls meet the base along the shared footprint boundary; the whole soup welds watertight
  via the boolean `assembleSolid`. (No separate inner cap / pad solid — see the note.) (**host**)

## 4. Weld the pad into the base cylinder (shared-vertex seam, no fragile boolean)
- [x] 4.1 The pad-wall ∩ cylinder seam is expressed by SHARED footprint-boundary
  vertices (the wall window and the pad's R-side edges reuse the same `uSamples`
  sequence), so `assembleSolid` welds it watertight directly — no `ssi_boolean_solid`
  fuse, no coincident faces. NUMSCI-independent. (**host** + **sim**)

## 5. Engine native dispatch + self-verify → OCCT fallback
- [x] 5.1 In `native_engine.cpp`, `NativeEngine::wrap_emboss` now tries the NATIVE path
  (`cybercad::native::feature::wrap_emboss`) for a native body; an OCCT body delegates to
  the OCCT oracle as before. (**host**)
- [x] 5.2 MANDATORY self-verify (`wrapEmbossVerified`): the candidate MUST be a valid
  watertight positive-volume solid with volume strictly GREATER than the base by ≈
  wrappedFootprintArea × height (arc-length × axial = the flat profile area), to a
  deflection-bounded tolerance (1% rel + floor). Failure OR NULL ⇒ OCCT `cc_wrap_emboss`.
  No tolerance weakened. (**host**)
- [x] 5.3 Out-of-slice inputs (deboss, non-rectangular/complex profile, freeform /
  non-cylindrical base, over-2π or off-end footprint) return NULL from the builder and
  fall through to the OCCT oracle unchanged (asserted by `wrap_emboss_scope_defers`). (**host** + **sim**)

## 6. Verification (two gates)
- [x] 6.1 Host suite (no OCCT): each corner on the cylinder by construction; the mesh is
  watertight; enclosed volume `= |cyl| + wrappedFootprintArea × height` within the
  deflection bound. New `tests/native/test_native_wrap_emboss.cpp` (4 cases). Builds in
  BOTH the default and NUMSCI configs (the native path is NUMSCI-independent). (**host**)
- [x] 6.2 Sim parity: `scripts/run-sim-native-wrap-emboss.sh` +
  `tests/sim/native_wrap_emboss_parity.mm` — native `cc_wrap_emboss` vs OCCT
  `cc_wrap_emboss` on the rectangular-pad-on-cylinder input over 3 configs: 6/6 PASS,
  vol rel ≤ 1.7e-3, area rel ≤ 7.3e-4, all watertight, mesh vol == B-rep vol. (**sim**)
- [x] 6.3 No regression: the existing OCCT wrap-emboss (`run-sim-phase3-suite.sh`, 70/0/0
  incl. all emboss checks), #6 curved fillet (`run-sim-native-curved-fillet.sh` 9/9),
  default CTest 27/27, NUMSCI CTest 34/34 all green. (**sim**)
- [x] 6.4 `openspec validate add-native-wrap-emboss --strict` green. (**host**)

## Deferred (NOT in this slice — honest NULL → OCCT)

- [ ] **Deboss** (`boss = 0`, the cut variant) → OCCT.
- [ ] **Non-rectangular / complex / dense / high-curvature profiles** (the OCCT
  healed-sew oracle's domain) → OCCT.
- [ ] **Freeform / non-cylindrical base face** (cone / sphere / planar / NURBS) → OCCT.
- [ ] **Footprint that wraps > 2π, self-overlaps, or exceeds the face's axial
  extent** → NULL → OCCT — never faked with a weakened tolerance.
