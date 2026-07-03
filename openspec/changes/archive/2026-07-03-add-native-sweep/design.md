# Design — add-native-sweep (Phase 4 #4b Tier C, sweep)

Clean-room native `cc_solid_sweep` for the tractable path shapes, on the #1–#3
foundations (`src/native/math`, `src/native/topology`, `src/native/tessellate`) + the
#4 / #4b assemblers (`src/native/construct/`). OCCT-free, host-buildable. OCCT
(`BRepOffsetAPI_MakePipe` / `MakePipeShell`) is a **reference oracle only** — consulted
to confirm the frame convention, face decomposition, and end-cap orientation; nothing
copied.

## 1. The contract (`cc_solid_sweep`)

```c
CCShapeId cc_solid_sweep(const double *profileXY, int profileCount,
                         const double *pathXYZ,   int pathCount);
```

- `profileXY` — a **closed 2D profile**, `(x,y)` pairs in its own local plane,
  conceptually centred on its **centroid** (the sweep places the centroid on the
  spine).
- `pathXYZ` — the **3D spine**, `(x,y,z)` triples (`pathCount ≥ 2`).
- Semantics (mirroring `BRepOffsetAPI_MakePipe`): the profile is placed
  **perpendicular to the path at its start** — the profile's local `+Z` aligns with
  the unit path tangent `T(0)` and its local X/Y span the plane ⟂ `T(0)` — then
  carried along the spine, sweeping a capped ("pipe") solid.

`cc_twisted_sweep` adds `twistRadians` (total twist accumulated end-to-end) and
`scaleEnd` (profile scale factor at the far end, linearly interpolated). `guided_sweep`
adds a guide curve; `loft_along_rail` skins two profiles along a rail — both **out of
scope** (fall-through).

## 2. Case split (what is native vs fall-through)

```
cc_solid_sweep(profile, path)
   │
   ├─ path is STRAIGHT (single seg, or all points collinear within tol)
   │      → directional extrude of the profile along the path vector.
   │        Volume EXACT = profileArea · |path|.  ── NATIVE
   │
   ├─ path is SMOOTH curved (C¹, gentle) and PASSES the curvature guard
   │      → RMF transport + swept-surface tiling + end caps.
   │        Deflection-bounded vs OCCT MakePipe.   ── NATIVE
   │
   └─ tight curvature (min radius < profile extent) / self-intersecting / degenerate
          → return NULL Shape.                     ── OCCT FALL-THROUGH (labelled)
```

`cc_twisted_sweep` follows the SAME split (its RMF path is the curved-native branch
with a post-composed twist/scale; a straight path + twist is still native — a linearly
twisted prism); if the split lands in the fall-through leg, twisted falls through too.
`cc_guided_sweep` and `cc_loft_along_rail` **always** fall through (labelled).

## 3. Straight path → directional extrude (EXACT)

Detect straight: fit a line to `pathXYZ`; if every point is within `kProfileTol` of it,
the sweep is a translation of the profile by the path vector `d = path[last] − path[0]`.

- Place the profile at the start: build the start frame `(X0, Y0, T0)` with `T0 = d̂`,
  and map each profile `(x,y)` to `P0 = start + x·X0 + y·Y0` (centroid on `path[0]`).
- The far section is `P0 + d`. This is exactly the #4 prism assembler generalised from
  `+Z` to an arbitrary direction `d`: reuse `build_prism`'s structure with an oriented
  frame → N `Plane` side faces (one per profile edge) + 2 `Plane` caps.
- **Volume is EXACT** (`profileArea · |d|`, a right prism along `d̂` since caps are
  ⟂ `d̂`). This is the analytic-parity anchor for Gate 1.

Reuse note: `construct.h build_prism` currently extrudes along `+Z`; factor a
`build_prism_dir(profile2D, startFrame, dir)` (or an affine placement wrapper) so the
straight sweep and the existing `+Z` extrude share one assembler — no duplicated
topology code.

## 4. Smooth curved path → transport + swept surface

> **IMPLEMENTATION CORRECTION (supersedes §4.2's RMF for the shipped code).** This
> section originally specified a rotation-minimizing frame (RMF) that keeps the section
> perpendicular to the tangent. During verification the RMF sweep was found to
> **disagree with the OCCT `BRepOffsetAPI_MakePipe` oracle**: MakePipe's default
> `GeomFill_CorrectedFrenet` law collapses to a **constant rotation on a PLANAR spine**
> (OCCT `GeomFill_CorrectedFrenet.cxx` — the `isPlanar` branch uses a `Law_Constant`), so
> it TRANSLATES the section with a fixed orientation rather than rotating it to stay
> perpendicular. The shipped `src/native/construct/sweep.h` therefore uses a **CONSTANT
> frame** (`detail::constantFrames`, not the double-reflection RMF), matching the oracle
> to fp precision (parity vol rel ≈ 1.7e-16) for the straight and smooth-planar cases.
> **Non-planar** curved spines (where MakePipe's law is genuinely non-constant) are
> **deferred to OCCT**. §4.2 below is retained as the original design rationale; read it
> as the rejected alternative. The volume law is `profileArea · |Δspine · n̂|` (fixed
> section normal), NOT the Pappus `profileArea · arcLength`.

### 4.1 Spine discretization
Sample the polyline spine (or fit a C¹ Catmull-Rom / centripetal spline through the
path points for a smooth tangent field) into `m` stations `s_0..s_{m-1}` with a
**deflection-adaptive** step (denser where curvature is high), so the tiled swept
surface stays within the target deflection of the ideal pipe. Compute the unit tangent
`T_k` at each station.

### 4.2 Rotation-minimizing frame (double-reflection RMF)
Frenet frames twist (and blow up at inflections); we want the **minimal-twist**
(rotation-minimizing) frame. Use the **double-reflection method** (Wang, Jüttler,
Zheng, Wang, *ACM TOG* 2008) — the standard, robust, second-order-accurate RMF:

```
Given frame (r_k, s_k, T_k) at x_k and the next point x_{k+1} with tangent T_{k+1}:
  v1   = x_{k+1} − x_k;          c1 = v1·v1
  rL   = r_k − (2/c1)(v1·r_k) v1      // reflect r across the plane ⟂ v1
  tL   = T_k − (2/c1)(v1·T_k) v1      // reflect T likewise
  v2   = T_{k+1} − tL;          c2 = v2·v2
  r_{k+1} = rL − (2/c2)(v2·rL) v2     // second reflection
  s_{k+1} = T_{k+1} × r_{k+1}
```

Seed `(r_0, s_0, T_0)` from the start frame in §3 (X0/Y0 ⟂ T0). The transported frame
`(r_k, s_k, T_k)` carries the profile with minimal twist; the accumulated twist over a
**planar** arc is ~0 (a Gate-1 analytic assertion of the RMF property, distinguishing
it from a Frenet frame).

### 4.3 Swept-surface tiling
At each station `k`, the profile vertex `i` maps to
`S_{k,i} = x_k + profileX_i · r_k + profileY_i · s_k`. Then:

- **One swept side patch per profile edge** `i→i+1`: a surface interpolating the two
  spine-adjacent section edges (`S_{k,i}→S_{k,i+1}` and `S_{k+1,i}→S_{k+1,i+1}`) across
  the stations. Build it as a `Bezier`/`BSpline` `FaceSurface` — degree-1 across the
  profile edge (u), degree matching the spine sampling across v — with poles the
  transported section-edge endpoints. (For a single spine segment this degenerates to
  the bilinear ruled patch already proven in `loft.h`; the curved case stacks patches
  station-to-station or fits one higher-degree BSpline in v.)
- **Two end caps**: the transported profile face at station 0 and at station m−1
  (`Plane` faces on the start/end frames).
- Assemble faces → `Shell` → `Solid`, oriented outward (shared vertex/edge nodes
  between adjacent side patches and caps → watertight, exactly as `loft.h` welds).

### 4.4 Swept-surface math helper (`src/native/math`)
Add a small helper that, given the transported section rings, produces the
`Bezier`/`BSpline` poles for one side patch (row-major, U outer, matching
`bezierSurfacePoint` / `bsplineSurfacePoint` so the native tessellator agrees). The
existing `FaceSurface` kinds suffice (a straight sweep edge → `Plane`; a curved sweep
edge → `Bezier`/`BSpline`); a named `SweptSurface` is only added if it reads cleaner
than raw pole construction.

## 5. Guards (honesty — return NULL → OCCT fall-through)

The native builder returns a **NULL `Shape`** (never a wrong solid) when:

- **Tight curvature.** At any station, the spine's curvature radius `ρ_k = 1/κ_k` is
  below the profile's **max radial extent** `R = max_i ‖(profileX_i, profileY_i)‖`
  (with a safety margin). The swept inner wall would fold / self-intersect — an invalid
  solid. Fall through to OCCT's trimmed pipe-shell.
- **Self-intersecting spine / section.** The spine crosses itself, or a transported
  section self-intersects (detected from `κ_k · R ≥ 1` locally, or an explicit
  segment-intersection test on adjacent sections).
- **Degenerate input.** `profileCount < 3`, `pathCount < 2`, zero profile area,
  zero-length path, or coincident consecutive path points collapsing the tangent.

Each guard is a Gate-1 host test (NULL asserted) and a Gate-2 fall-through proof
(identical to `cc_set_engine(0)`).

## 6. Twisted sweep

Reuse §3/§4 transport, then per station scale `s ∈ [0,1]` (normalised arc length):

- rotate the section in its `(r_k, s_k)` plane by `θ_k = twistRadians · s`;
- scale it by `f_k = 1 + (scaleEnd − 1)·s`.

i.e. `S_{k,i} = x_k + f_k·(cosθ_k·X_i − sinθ_k·Y_i)·r_k + f_k·(sinθ_k·X_i + cosθ_k·Y_i)·s_k`.
Same tiling + caps + guard. A straight path with a twist is still native (a linearly
twisted prism — the rotated-square loft in `loft.h` is the 2-station analogue). If the
base sweep is not applicable (guard trip), twisted falls through too — no separate path.

## 7. `NativeEngine` wiring

```cpp
ShapeResult NativeEngine::solid_sweep(const double* p, int pc,
                                      const double* path, int pathc) {
    ntopo::Shape solid = ncst::build_sweep(p, pc, path, pathc);
    if (solid.isNull()) return fallback().solid_sweep(p, pc, path, pathc); // OCCT
    return track(wrapNative(std::move(solid)));
}
ShapeResult NativeEngine::twisted_sweep(const double* p, int pc, const double* path,
                                        int pathc, double tw, double se) {
    ntopo::Shape solid = ncst::build_twisted_sweep(p, pc, path, pathc, tw, se);
    if (solid.isNull()) return fallback().twisted_sweep(p, pc, path, pathc, tw, se);
    return track(wrapNative(std::move(solid)));
}
// guided_sweep / loft_along_rail: PURE fall-through (Tier C — pipe-shell/guide),
// labelled, verified identical to cc_set_engine(0).
```

OCCT appears only under `CYBERCAD_HAS_OCCT` in the fallback wiring; the native builder
references no OCCT / `IEngine` / `EngineShape` type.

## 8. Cognitive complexity

- `doubleReflectionRMF` step — short, ~5 (irreducible vector algebra, documented).
- `build_sweep` — a dispatcher (straight vs curved vs guard) + linear assembler,
  target ≤ 15 (systems band); split the tiling loop into `sweptSideFace` + `capFace`
  helpers as `loft.h` does.
- Guards are guard-clauses at the top (each returns NULL) → low branching.
- Flag any genuinely irreducible station-loop as systems-band (≤ 25) per
  NATIVE-REWRITE.md.

## 9. Verification detail

- **Gate 1 (host, no OCCT):** straight sweep EXACT volume `A·L` + watertight + face
  count; curved sweep watertight + volume → `A·spineArcLength` within deflection bound
  (Pappus/centroid check); RMF planar-arc twist ~0; tight-curvature / self-intersect /
  degenerate → NULL.
- **Gate 2 (sim, OCCT oracle):** `cc_solid_sweep` native vs `BRepOffsetAPI_MakePipe`
  (straight: vol rel 0.0, structure exact; curved: vol/area/bbox within a documented
  deflection tolerance, all watertight; native F a k≥1 multiple of OCCT F);
  `cc_twisted_sweep` where applicable; and tight-curvature / `cc_guided_sweep` /
  `cc_loft_along_rail` asserted identical under both engines (fall-through proof).
  OCCT default restored in teardown; the parity test carries its own `main()` (on the
  suite's SKIP list) so `run-sim-suite.sh` 221/221 is unchanged.

## References (oracle only — not copied)

- OCCT `/Users/leonardoaraujo/work/OCCT/src`: `BRepOffsetAPI_MakePipe`,
  `BRepOffsetAPI_MakePipeShell`, `BRepFill_PipeShell`, `GeomFill_CorrectedFrenet`
  (the corrected-Frenet / RMF-equivalent OCCT uses to minimise pipe twist).
- Wang, Jüttler, Zheng, Wang, "Computation of Rotation Minimizing Frames", *ACM
  Transactions on Graphics* 27(1), 2008 — the double-reflection method (§4.2).
- *The NURBS Book* (Piegl & Tiller) — swept/ruled surface pole construction (§4.3),
  consistent with the existing `src/native/math` Bézier/BSpline evaluators.
