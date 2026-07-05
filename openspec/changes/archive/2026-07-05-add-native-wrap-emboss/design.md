# Design — add-native-wrap-emboss (#7 wrap-emboss, first slice)

## Context

`cc_wrap_emboss(body, faceId, profileXY, count, height, boss)` wraps a 2D profile
onto a cylindrical face and pads it radially to boss / deboss. Today it is
**OCCT-only**: the Phase-3 oracle (`src/engine/occt/occt_wrap_emboss.cpp`, GitHub
#290) builds the pad as an exact cap-and-side surface set — an inner cap and outer
cap (trimmed cylindrical faces whose boundaries are helical arcs in the surface's
own `(u,v)`), plus one ruled side wall per profile edge — and sews + heals it
(`BRepBuilderAPI_Sewing` + `ShapeFix`) into a watertight solid, then fuses (boss) /
cuts (deboss). `NativeEngine::wrap_emboss` currently just delegates to that oracle.

`ROADMAP.md` #7 and `SSI-ROADMAP.md` (§Sequencing: `S5 → #7 wrap-emboss`) place the
FIRST native slice directly on the finished curved-boolean stack. This design builds
the OCCT oracle's cap-and-side decomposition NATIVELY for the simplest input — a
RECTANGULAR pad embossed on a cylinder lateral face — and fuses it with the base
solid using the native SSI curved boolean, gated by a new engine self-verify → OCCT
fallback.

The method is **clean-room**: the wrapping math (arc-length → angle) and the
cap-and-side decomposition are re-derived from the geometry; the OCCT
`cc_wrap_emboss` result is the verification ORACLE only, never copied.

## Goals / Non-Goals

**Goals**
- A native, OCCT-free wrap-emboss: RECTANGULAR footprint, `boss = 1`, on a native
  body's CYLINDER lateral face, `height > 0`.
- Project the footprint onto the cylinder (arc-length `px → u = px/R`, axial
  `v = py + vMid`); build the raised pad as wrapped side walls + an outer cap; weld
  watertight; FUSE with the base cylinder via the native curved boolean / SSI seam.
- Gate with a NEW engine self-verify (valid watertight solid, volume strictly
  GREATER than the base by ≈ wrappedFootprintArea × height) → OCCT fallback.
- Behind the unchanged `cc_wrap_emboss` ABI; OCCT oracle for everything else.

**Non-Goals (return NULL → OCCT, never faked)**
- DEBOSS (`boss = 0`) — the cut variant; a later #7 slice.
- Non-rectangular / complex / dense / high-curvature profiles — the OCCT healed-sew
  oracle's domain; defer.
- Freeform / non-cylindrical base face (cone, sphere, planar, NURBS) — defer.
- A footprint that wraps > 2π, self-overlaps, or exceeds the face's axial extent —
  return NULL → OCCT.

## The rectangular-pad-on-cylinder geometry (clean-room)

Base: a native cylinder solid, lateral `Cylinder` face radius `R`, axis `A` (frame
Z), the selected `faceId`. The face's V-mid is the axial centre of the lateral face.
The profile is a closed RECTANGLE `[px0,px1] × [py0,py1]` in wrapped coordinates.

**Footprint projection.** Each profile corner `(px, py)` maps to a cylinder point at
`u = px / R` (angle), `v = py + vMid` (axial) — the same map the OCCT oracle uses. The
footprint's four edges become:

- two AXIAL edges (`u = u0` and `u = u1`, `v ∈ [v0,v1]`): straight segments parallel
  to the axis on the cylinder;
- two CIRCUMFERENTIAL edges (`v = v0` and `v = v1`, `u ∈ [u0,u1]`): circular ARCS on
  the cylinder (constant `z`), tiled into deflection-bounded chords (angular sagitta
  `R(1−cos Δu/2) ≤ deflection`).

**The raised pad** (`boss`, `rIn = R`, `rOut = R + height`), a cap-and-side set:

- OUTER CAP: a trimmed `Cylinder` face at radius `R + height` over `u ∈ [u0,u1]`,
  `v ∈ [v0,v1]` — tiled into quad facets following the outer cylinder.
- AXIAL side walls (×2, at `u = u0` and `u = u1`): each a planar strip on the RADIAL
  plane through the axis at that angle, spanning `ρ ∈ [R, R+height]`, `v ∈ [v0,v1]`.
- CIRCUMFERENTIAL side walls (×2, at `v = v0` and `v = v1`): each a planar strip on
  the plane `z = v`, spanning `ρ ∈ [R, R+height]`, `u ∈ [u0,u1]` (tiled along `u`).
- INNER footprint: the wrapped rectangle on the base cylinder at `R` — NOT a face of
  the standalone pad; it is the SEAM where the pad meets the base solid, resolved by
  the fuse.

**The fuse.** Build the pad as a standalone watertight solid (outer cap + 4 side
walls + an inner cap at `R` closing it for the boolean) and FUSE it with the base
cylinder via the native curved boolean (`ssi_boolean_solid`, Op::FUSE /
`native-booleans`). The pad-side-wall ∩ base-cylinder intersection is the wrapped
footprint curve — a native SSI seam (the axial walls cut the cylinder along axial
lines; the circumferential walls along `z`-circles; a small radial overlap keeps the
boolean non-coincident, as in the OCCT oracle). The result is the base cylinder with
the raised rectangular pad added.

## Module shape

```
src/native/wrapemboss/
  wrap_emboss.h            // NEW — the native rectangular-pad-on-cylinder builder
                           //   projectFootprint / buildPad / fusePad
  (reuses) native/construct, native/math (Cylinder), native/boolean (ssi_boolean,
           assembleSolid), native/ssi
src/engine/native/native_engine.cpp
                           // NEW native wrap_emboss dispatch + self-verify guard
```

`NativeEngine::wrap_emboss(body, faceId, p, c, d, boss)` currently delegates to the
fallback. It gains a native path: if `boss == 1`, the body is native, `faceId`
resolves to a `Cylinder` lateral face, and the profile is a rectangle, call
`nwrap::wrap_emboss(...)`; then run the self-verify; else (or on failure) delegate to
OCCT as today.

## Engine self-verify → OCCT fallback (NEW dispatch, existing discipline)

The native fillet path reuses `blendResultVerified(result, body, wantGrow)`
(watertight + volume-sign). The wrap-emboss dispatch applies the SAME idea with
`wantGrow = true`: the candidate MUST be a valid watertight positive-volume solid
whose volume is strictly GREATER than the base body's, by a magnitude consistent with
the wrapped footprint area × height (a documented plausible band, not merely
"changed"). If the native builder returns NULL OR the self-verify fails, the engine
falls through to the OCCT `cc_wrap_emboss` oracle. The engine SHALL NEVER emit an
unverified, leaky, or wrong-volume embossed solid. No tolerance is weakened to pass.

## Build pipeline (concrete)

1. **Classify.** `boss == 1`, native body, `faceId` → `FaceSurface`-kind `Cylinder`
   lateral face (radius `R`, axis `A`, V-mid), profile = closed rectangle (4 pts,
   positive area). Else NULL → OCCT.
2. **Guard.** `height > 0`; footprint angular span `(px1−px0)/R < 2π − ε`; footprint
   axial span within the face's axial extent; no self-overlap. Else NULL → OCCT.
3. **Project footprint** to `(u,v)` corners; tile the two circumferential arcs into
   deflection-bounded chords.
4. **Build the pad solid** — outer cap (trimmed cylinder at `R+height`), 4 side walls
   (2 radial-plane axial, 2 planar circumferential), an inner cap at `R` closing the
   pad for the boolean; weld watertight via `assembleSolid`.
5. **Fuse** pad ∪ base cylinder via `ssi_boolean_solid(base, pad, Op::FUSE)` (native
   curved boolean; the pad∩cylinder seam is the SSI intersection). Small radial
   overlap for a clean boolean.
6. **Return** the fused solid to `NativeEngine::wrap_emboss`, which runs the new
   watertight + volume-increasing self-verify; pass → native, fail → OCCT.

Cognitive-complexity: classify + guard is flat guard-clauses (backend band ≤15); the
pad tiling is isolated loops with a documented sagitta bound (systems band); the fuse
is one call into the existing curved-boolean substrate.

## Verification model (two gates)

- **Host (no OCCT; fuse available under NUMSCI):** build a native cylinder on the
  host; emboss a rectangular pad natively; assert (a) each projected footprint corner
  lies on the cylinder within tol, (b) the fused mesh is watertight
  (`boundaryEdgeCount == 0`) across the deflection ladder, and (c) the enclosed
  volume equals `|cyl| + wrappedFootprintArea × height` within the deflection bound
  (`wrappedFootprintArea` = the wrapped rectangle area on the mid-radius). A
  no-NUMSCI host build asserts the native path is skipped (OCCT-only behaviour
  unchanged).
- **Sim native-vs-OCCT parity** (`scripts/run-sim-native-wrap-emboss.sh`, modelled on
  `run-sim-phase3-suite.sh` / the wrap-emboss harness): on a booted simulator, emboss
  the SAME rectangular pad with `cc_wrap_emboss` under the native engine
  (`cc_set_engine(1)`) and under OCCT (`cc_set_engine(0)`); compare volume / surface
  area / watertightness / `BRepCheck_Analyzer::IsValid` against the OCCT
  `cc_wrap_emboss` result within tol, and report the measured deltas. Also confirm no
  regression: the existing OCCT wrap-emboss (#290, `run-sim-phase3-suite.sh`), native
  booleans, SSI, and `run-sim-suite.sh` stay green.

## Decisions

- **Rectangle first.** A rectangular footprint gives closed-form axial/circumferential
  boundaries and a trivially valid pad, exercising the full projection → pad → fuse
  pipeline without the dense-profile healing the OCCT oracle exists to handle.
- **Fuse via the native curved boolean, not a bespoke weld.** The pad∩cylinder seam
  IS an SSI intersection; routing the fuse through `ssi_boolean_solid` reuses the
  verified S5 curved-boolean machinery (and its S4-f completeness guard) rather than
  hand-rolling a coincident-face weld.
- **New self-verify with `wantGrow = true`.** An emboss ADDS material; the guard
  mirrors the blend guard's volume-sign discipline, gated additionally by the
  footprint-area × height plausibility band.
- **NUMSCI-gated native path.** The fuse consumes the SSI substrate, so the native
  path compiles under `CYBERCAD_HAS_NUMSCI` (like the curved boolean); a build without
  it keeps today's OCCT-only wrap-emboss.

## Risks / Trade-offs

- **Slice narrowness vs honesty.** Only the rectangular-pad-on-cylinder emboss
  (`boss = 1`) lands; deboss, complex profiles, freeform bases defer to OCCT.
  Accepted — faking a wrap-emboss is forbidden.
- **Fuse robustness on the wrapped seam.** The pad∩cylinder seam relies on the
  native curved-boolean fuse; if that declines (returns NULL) or the fused solid
  fails the self-verify, the whole op falls through to the OCCT oracle — never a
  leaky pad.
- **Footprint-area × height plausibility band.** The volume band uses the wrapped
  (curved) footprint area; a borderline near-2π wrap or a very thin pad returns NULL
  → OCCT rather than a fragile fuse, matching the roadmap's degeneracy stance.
