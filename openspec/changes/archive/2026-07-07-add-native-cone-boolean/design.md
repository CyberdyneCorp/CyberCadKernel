# Design — add-native-cone-boolean (SSI Stage S5 — CONE surface family, coaxial COMMON)

## Context

`src/native/boolean/ssi_boolean.cpp` (OCCT-free, `CYBERCAD_HAS_NUMSCI`-gated) already ships the
SSI-driven curved-boolean assembler for three surface families, all consuming a seam trace and
welding shared-pool planar facets:

- `buildCommon` / `buildFuse` / `buildCut` (~lines 753–1201) — through-drill **cyl∩cyl** (TWO rim
  seams: one operand full-circle on both = the tube, the other local = the pierced wall).
- `buildLensCommon` / `buildLensFuse` / `buildLensCut` (~lines 978–1129) — **sphere∩sphere** lens
  (ONE closed seam, BOTH operands `Sphere`).
- `buildSteinmetzCommon` / `Fuse` / `Cut` (~lines 1488–1668) — the branched **Steinmetz**
  bicylinder (four `BranchArc` arms, two branch poles).
- Shared machinery: `recogniseCurvedSolid` (folds a solid into a `CurvedSolid{kind, frame,
  radius, semiAngle, vLo, vHi, capPlanes}` — ALREADY handles `CurvedKind::Cone`), `classifyPoint`
  (the curved point-in-solid test — ALREADY handles the cone wall half-space
  `(‖w−axial·ẑ‖ − (R0 + axial·tanα))·cosα`), `appendDiskCap` (~line 630, the faceted disc-cap
  fan), `pushPlanarTri`, and the shared `VertexPool` (`native/boolean/assemble.h`).

The dispatcher `ssi_boolean_solid` (~line 1716) recognises both operands, traces the seam
(`ssi::trace_intersection`), gates on full transversality (`nearTangentGaps == 0`,
`branchPoints == 0`), builds `Seam`s from the WLines, and switches on op → the family builders. A
coaxial cone∩cylinder pair produces ONE clean analytic circle seam; `buildCommon` declines it (it
needs two seams) and `buildLensCommon` declines it (it needs two spheres), so the pair falls
through to OCCT.

The S1 analytic layer already closed-forms the seam: `src/native/ssi/quadric_pairs.h`
`intersectCylinderConeCoaxial` (~line 157) emits the circle(s) where the cone's signed radius
`R0 + v·sinα` equals `±Rc` (radius `Rc`, centred on the axis at `v·cosα`); the on-nappe circle
inside the frustum extent is the physical seam. `intersectSphereConeCoaxial` (~line 205) solves
the quadratic `z(v)² + r(v)² = Rs²` for up to two circles.

The GAP is the assembler for a cone face. This change adds the coaxial cone∩cylinder COMMON
builder (optionally cone∩sphere COMMON), reusing the S5-a/c planar-facet weld and the engine
self-verify pattern. It is a BOUNDED addition: one new family, COMMON only, the analytically-
clean coaxial case.

## The geometry

A frustum cone A and a coaxial cylinder B share ONE axis (unit `ẑ`, origin `O` on the axis line).
Measure axial height `h = (P − O)·ẑ`. The cone's cross-section radius is
`r_c(h) = R0 + (h − h0)·tanα` (from `math::Cone`: `value(u,v)` has axial component `v·cosα` and
radius `R0 + v·sinα`, so along the axis `dr/dh = sinα/cosα = tanα`). APEX-FREE gate:
`r_c(h) > margin` over the whole frustum extent — no apex in the solid, so the S4-e apex chart
singularity is never touched. The cylinder radius is `Rc`, constant.

The two solids' axial overlap is `[hBot, hTop] = [max(coneBottom, cylBottom),
min(coneTop, cylTop)]`. The walls cross at the SEAM height `h*` where `r_c(h*) = Rc` — the S1
analytic circle, radius `Rc`, centred on the axis at `h*`. GATE: `h*` strictly inside
`(hBot, hTop)` (a transversal circle, not a cap-edge tangent) and inside both extents.

On one side of `h*` the cone is the tighter (smaller-radius) wall, on the other the cylinder is.
The COMMON `A ∩ B` is the solid of revolution whose cross-section radius is the pointwise MINIMUM
`min(r_c(h), Rc)` over `[hBot, hTop]`:

| Region | Bound | Radius | Kept because |
|---|---|---|---|
| CONE band (`h` where `r_c ≤ Rc`) | the cone wall | `r_c(h)` | classifies INSIDE the cylinder |
| CYLINDER band (`h` where `Rc ≤ r_c`) | the cylinder wall | `Rc` | classifies INSIDE the cone |
| bottom cap (`h = hBot`) | disc, `−ẑ` outward | `min(r_c(hBot), Rc)` | terminal disc of the band that starts there |
| top cap (`h = hTop`) | disc, `+ẑ` outward | `min(r_c(hTop), Rc)` | terminal disc of the band that ends there |

The two bands meet at the seam circle (radius `Rc` at `h*`); the caps close the two axial ends.
Widening or narrowing cone are symmetric (swap which band is below `h*`); the assembler reads the
side from `classifyPoint`, not a hard-coded direction.

### Closed-form volume (the analytic oracle)

Split at `h*`. Let `hLoBand..hHiBand` be the CONE-tighter sub-band with cone radii `r(hBot)` at
its far end and `Rc` at `h*`, and the CYLINDER-tighter sub-band of height `hTop − h*`:

```
V(A ∩ B) = V_frustum(r(hBot) → Rc over Δh_cone) + π·Rc²·(hTop − h*)
V_frustum(ra → rb over Δh) = (π·Δh / 3)·(ra² + ra·rb + rb²)
```

For the reference fixture (cone `r0 = 0.4 @ h=0` → `r1 = 2.0 @ h=4`, so `r_c(h) = 0.4 + 0.4h`;
cylinder `Rc = 1.0`, `h ∈ [1, 5]`): seam `0.4 + 0.4h* = 1.0 → h* = 1.5`; overlap `[1, 4]`; cone
tighter on `[1, 1.5]` (`r: 0.8 → 1.0`), cylinder tighter on `[1.5, 4]`. So
`V_frustum(0.8 → 1.0 over 0.5) = (π·0.5/3)(0.64 + 0.8 + 1.0) = 1.2775` and
`π·1²·2.5 = 7.85398`, total `V(A ∩ B) = 9.1315`. That is the exact value OCCT `BRepAlgoAPI_Common`
converges to — the DUAL oracle.

The frustum small-end disc (radius `0.8` at `h = 1`) is the bottom cap and the cyl-segment disc
(radius `1.0` at `h = 4`) is the top cap, so the closed form already accounts for both caps.

## Goals / Non-Goals

**Goals**
- Add `buildConeCylCommon(A, B, seam)` — the coaxial cone∩cylinder COMMON: the frustum band welded
  to the cylinder-segment band along the single S1 seam circle, closed by two disc caps, selected
  by the inside-the-other rule, welded watertight through one `VertexPool`, returning a `Solid` or
  NULL → OCCT.
- Add `appendConeBand` — the planar-facet frustum band strip (cone analogue of `appendTubeBand`),
  outward radial-`cosα` normal, sharing terminal rings through the pool.
- Dispatch `Op::Common` → `buildConeCylCommon` after the through-drill / lens builders decline.
- Add the engine analytic oracle: extend `ssiCurvedBooleanVerified` (`op == 2`) with the coaxial
  cone∩cylinder closed form `V_frustum + π Rc²·(hTop − h*)`, mirroring the Steinmetz `16 R³/3`
  oracle. A wrong-volume / non-watertight candidate is DISCARDED → OCCT.
- (Optional) `buildConeSphereCommon` — the single-crossing coaxial cone∩sphere COMMON (frustum
  band + spherical-segment band), with its own closed-form oracle; the two-circle case declines.

**Non-Goals (deferred — never faked)**
- TRANSVERSAL (non-coaxial) cone∩cylinder / cone∩sphere — a genuine quartic space curve, NOT an
  analytic circle here (`intersectCylinderConeCoaxial` returns `notAnalytic`) → the general
  marcher / OCCT. UNCHANGED.
- APEX-CROSSING seams and a frustum whose extent INCLUDES the apex (`r_c → 0`) — the S4-e apex
  chart singularity → NULL → OCCT.
- The TWO-CIRCLE coaxial cone∩sphere crossing (both circles inside both extents) → NULL → OCCT
  (the first slice handles the single-crossing config only).
- cone∩cone (any configuration) → NULL → OCCT.
- Cone FUSE / CUT (any pair) → NULL → OCCT (the analytically-clean COMMON is the guaranteed first
  slice; FUSE/CUT follow only when they land watertight with the correct closed form).
- Any change to `src/native/tessellate`, the `cc_*` ABI, the planar BSP-CSG, the analytic
  `curved.h`, or the cyl / sphere / Steinmetz builders.
- Weakening ANY tolerance to force a pass. If the cone COMMON cannot be built watertight with the
  correct closed-form volume, return NULL → OCCT and report the measured gap.

## Module shape

```
src/native/boolean/ssi_boolean.cpp            [CYBERCAD_HAS_NUMSCI]
  appendDiskCap(...)                    // UNCHANGED — reused for the two caps
  appendConeBand(cone, hFrom, hTo, seamRing, pool, faces)   // NEW — frustum ring strip
  appendCylBand(cyl,  hFrom, hTo, seamRing, pool, faces)    // NEW/reuse — cylinder ring strip
  buildConeCylCommon(A, B, seam)        // NEW — frustum band + cyl band + 2 caps
  buildConeSphereCommon(A, B, seam)     // NEW (optional) — frustum band + sphere-segment band
  ssi_boolean_solid(...)                // dispatch: Op::Common → buildConeCylCommon (+ optional)

src/engine/native/native_engine.cpp
  ssiCurvedBooleanVerified(...)         // extend op==2 with the coaxial-cone closed-form oracle
```

No new files; reuses `recogniseCurvedSolid`, `classifyPoint`, `appendDiskCap`, `pushPlanarTri`,
`VertexPool`, `toSeam`, and the S1 `intersectCylinderConeCoaxial` seam. OCCT-free.

## Parametrization note (axial height vs surface v)

`math::Cone::value(u,v)` places a point at axial component `v·cosα` and radius `R0 + v·sinα`, so
the intrinsic surface param `v` and the axial height `h` relate by `h = v·cosα`. `CurvedSolid`
stores `vLo/vHi` as the AXIAL projection `dot(w, ẑ)` (the same convention as the cylinder), so the
assembler works in axial-height space `h` and converts to the intrinsic `v = h / cosα` when it
calls `cone.point(u, v)`. The seam ring nodes come directly from the traced circle (already the
correct 3D points), so no per-node param round-trip is needed for the weld — the intrinsic-v
conversion is used only to place the interior band-station rings. This keeps the band samples
exactly on the analytic wall.

## `appendConeBand` — the frustum ring strip (NEW)

```cpp
// Emit a planar-facet frustum band of `cone` between axial stations hFrom..hTo, as a
// sequence of ring strips; the seam-side terminal ring is the SHARED pooled seamRing (so it
// welds to the cylinder band), the other terminal ring is a fresh pooled ring (welds to a
// cap). Each facet's outward normal is the cone's outward wall normal (radial, tilted by α):
//   n ∝ (radial_out)·cosα + ẑ·(−sinα)   (the true cone normal, not the pure radial).
// Ring count from the frustum's slant-length / sagitta bound (like appendTubeBand); u-count
// (nu) matches the seam ring so the strip columns line up. Returns false → NULL upstream if a
// station would fall on the apex (r_c ≤ margin) or the band is degenerate.
bool appendConeBand(const CurvedSolid& cone, double hFrom, double hTo,
                    const std::vector<int>& seamRing, VertexPool& pool,
                    std::vector<topo::Shape>& faces);
```

The cylinder band reuses the same pattern with a constant radius and pure-radial normal (the
existing `appendTubeBand` covers a full-circle cylinder band between two rings; the seam ring is
one terminal, a fresh pooled ring the other).

## `buildConeCylCommon` — frustum band + cyl band + two caps (NEW)

```
gate:      recogniseCurvedSolid → one Cone (A or B) + one Cylinder; coaxial (sameAxis);
           EXACTLY ONE closed full-circle seam on BOTH walls; frustum apex-free over its extent
           (r_c(v) > margin ∀ v); seam h* strictly inside (hBot, hTop). Else → {} (→ OCCT).
seam ring: resample the traced circle into nu evenly-spaced 3D nodes; pool them ONCE (the shared
           seamRing indices). Snap so both bands reference IDENTICAL nodes.
overlap:   [hBot, hTop] = [max(coneBottom, cylBottom), min(coneTop, cylTop)]; empty → {}.
side:      classify a cone-wall sample and a cyl-wall sample on each side of h* to decide which
           band lives on which side (widening vs narrowing cone read from classifyPoint, not
           hard-coded).
cone band: appendConeBand(cone, <cone-side terminal h>, h*, seamRing, pool, faces), kept iff its
           mid-sample classifies INSIDE the cylinder (classifyPoint(cyl, mid) == 1; ON == 0 → {}).
cyl band:  appendCylBand(cyl, h*, <cyl-side terminal h>, seamRing, pool, faces), kept iff its
           mid-sample classifies INSIDE the cone (classifyPoint(cone, mid) == 1; ON == 0 → {}).
caps:      appendDiskCap at hBot (−ẑ) and hTop (+ẑ), terminal rims pooled with the bands.
assemble:  makeShell → makeSolid; too few faces / non-watertight → {}.
```

Both bands share the pooled seam ring → they weld along the seam circle; each band's outer
terminal ring is pooled and shared with its cap → the caps weld to the bands; the shell is
watertight. Volume = `V_frustum(r(hBot) → Rc) + π Rc²·(hTop − h*)`. The engine self-verify checks
this against the closed form.

## `buildConeSphereCommon` — frustum band + spherical-segment band (NEW, optional)

Same shape, gated to the SINGLE-crossing coaxial config: `intersectSphereConeCoaxial` returns
exactly ONE circle inside both extents (the two-circle case → `{}` → OCCT). The frustum band welds
to a spherical-segment band (the sphere-latitude strip inside the cone) along the seam circle,
closed by the terminal caps. Closed form `V = V_frustum + V_spherical-segment`, where the
spherical segment volume is `π h_s²(3Rs − h_s)/3` for a cap of height `h_s` (or the general
zone formula for a middle segment). Shipped only if it lands watertight in the verified envelope;
otherwise deferred → OCCT.

## Dispatch (`ssi_boolean_solid`) — extended

Recognition + trace + the transversality gate are UNCHANGED. Extend only the `Op::Common` arm:

```cpp
case Op::Common: {
  if (topo::Shape drill = buildCommon(*csA, *csB, seams); !drill.isNull()) return drill;
  if (topo::Shape lens  = buildLensCommon(*csA, *csB, seams); !lens.isNull()) return lens;
  if (topo::Shape cone  = buildConeCylCommon(*csA, *csB, seams); !cone.isNull()) return cone;
  return buildConeSphereCommon(*csA, *csB, seams);  // optional; else {} → OCCT
}
```

`buildConeCylCommon` returns `{}` for any non-(cone+coaxial-cylinder) pair or any declined gate,
so the through-drill and lens paths are untouched. `Op::Fuse` / `Op::Cut` are NOT extended for
cone pairs → the cone COMMON is the only new native op; fuse/cut cone pairs return NULL → OCCT.

## Engine self-verify — the coaxial-cone analytic oracle (mirror of Steinmetz)

`ssiCurvedBooleanVerified` (`native_engine.cpp` ~line 332) currently returns `{}` unless `op == 2`
AND the operands are an equal-R orthogonal Steinmetz cylinder pair (the `16 R³/3` oracle). Extend
the `op == 2` branch: if the operands recognise as one `Cone` + one coaxial `Cylinder` (apex-free,
single interior seam), compute

```
h*    = seam height where r_c(h*) = Rc
hBot  = max(coneBottom, cylBottom);  hTop = min(coneTop, cylTop)
Δcone = |h* − (cone-side terminal h)|;  rBot = r_c(cone-side terminal)
expected = (π·Δcone/3)(rBot² + rBot·Rc + Rc²) + π·Rc²·(hTop − h*)
accept iff watertight(result) AND |vr − expected| ≤ max(1e-2·expected, 1e-6)
```

Returns `{applicable=true, matched}`; a wrong-volume or non-watertight candidate → `matched=false`
→ DISCARDED → OCCT. `op != 2` (cone fuse/cut) stays `{}` (not applicable) and the generic
set-algebra guard runs — but the cone builder returns NULL for fuse/cut, so the engine ships OCCT
there anyway. The oracle lives in the ENGINE (next to the OCCT fallback), so `src/native/**` stays
OCCT-free.

## Verification plan

- **Host (no OCCT), closed-form dual oracle.** `V(A ∩ B) = V_frustum(r(hBot) → Rc) +
  π Rc²·(hTop − h*)`. The host test asserts, for the reference coaxial cone∩cylinder pair: a
  watertight shell (`boundaryEdgeCount == 0`, every edge shared by exactly two faces), enclosed
  volume matches the closed form within the deflection band, every seam-ring node on BOTH walls ≤
  tol, the seam ring pooled ONCE; an apex-in-extent / transversal / cap-tangent / two-circle
  fixture → NULL. Green with NUMSCI on AND off (cone path correctly absent off). The cyl / sphere
  / Steinmetz regression goldens stay byte-identical.
- **Sim native-vs-OCCT.** Extend `scripts/run-sim-native-ssi-curved-boolean.sh` +
  `tests/sim/native_ssi_curved_boolean_parity.mm` with a coaxial cone∩cylinder pair (reusing
  `makeCone`/`occtCone`, `makeCyl`/`occtCyl`); its COMMON becomes a NATIVE pass vs
  `BRepAlgoAPI_Common` (volume ≈ `9.1315`, surface area, watertight closed shell, valid shape),
  raising native-pass **12 → 13** (14 if cone∩sphere COMMON lands). FUSE/CUT for the cone pair
  stay honest fall-backs with the measured gap. Do NOT regress the 12 existing native passes.
- `openspec validate add-native-cone-boolean --strict` green; record the CONE family opening in
  `SSI-ROADMAP.md` / `ROADMAP.md` / `NATIVE-REWRITE.md` with the measured deltas. Confirm no SSI /
  blend / heal / import / marching / phase3 suite regresses.

## Cognitive complexity

`buildConeCylCommon` is a short linear composition (gate → seam ring → overlap → per-band classify
→ `appendConeBand` / `appendCylBand` → two `appendDiskCap` → shell/solid), ~14–16 in the systems
band, comparable to `buildCommon` / `buildLensCommon`. `appendConeBand` is a ring-strip loop with
a sagitta-bounded station count (~12), mirroring `appendTubeBand`. The engine oracle extension is
a straight-line closed-form computation (~8). No function is pushed above the documented systems
band; the reused helpers (`appendDiskCap`, `classifyPoint`) are UNCHANGED.
