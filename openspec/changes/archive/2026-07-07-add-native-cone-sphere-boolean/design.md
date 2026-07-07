# Design — add-native-cone-sphere-boolean (SSI Stage S5-f: coaxial cone∩sphere COMMON/FUSE/CUT)

## Context

The S5-e coaxial cone∩cylinder op-set is 3/3 native in `src/native/boolean/ssi_boolean.cpp`
(OCCT-free, `CYBERCAD_HAS_NUMSCI`-gated). Two mature families of machinery already exist there:

- **The cone-wall split** (`coneCylSetup` + `buildConeCyl{Common,Fuse,Cut}`, ~lines 1083–1380):
  `coneCylSetup` gates one `Cone` + one coaxial `Cylinder`, computes the single analytic crossing
  `s*`, cross-checks it against the traced seam, and exposes the shared frame `(O, ẑ, X, Y)`, the
  `rCone(s)` / `ring(r, s)` / `wallPoint(r, s)` functors, and the azimuth resolution `N`.
  `appendRevolvedBand` (a ring→ring quad band, outward-or-inward radial normal via `outwardSign`)
  and `appendDiskCap` (an axis-point → rim fan disc, `±ẑ` normal) weld the cone-side fragments
  through one `VertexPool`.
- **The spherical-cap fragment** (`appendSphereCap`, ~lines 892–936, and the lens builders
  `buildLens{Common,Fuse,Cut}` + `lensSetup`, ~lines 1382–1479): `appendSphereCap(sph, otherCentre,
  seam, rings, pool, faces, outer, reversed)` builds a slerped ring fan from the sphere's APEX
  (the near pole toward `otherCentre` when `outer == false`, the far pole when `outer == true`) out
  to the seam circle, with the facet normal OUTWARD (`reversed == false`) or INWARD
  (`reversed == true`, the CUT cavity cap). Its outer ring (`r == rings`) reuses the exact seam
  nodes (`seam.pts`), so a cap welds to whatever else shares those nodes through the pool.

The coaxial cone∩sphere pair is EXACTLY the composition of these two: the seam is a single analytic
circle (the S1 `intersectSphereConeCoaxial` quadratic's single in-extent root), the CONE side of
the seam is a cone-wall band + disc cap, and the SPHERE side is a spherical cap. This change is a
BOUNDED new pair: the SAME seam-ring / shared-pool weld discipline, with the cone-side and sphere-
side builders already written; only the prologue (`coneSphereSetup`), the three per-op fragment
selections, and one engine oracle arm are new.

The dispatcher `ssi_boolean_solid` (~line 2065) recognises both operands, traces the seam, gates on
full transversality (`nearTangentGaps == 0`, `branchPoints == 0`), builds `Seam`s, and switches on
op. Today the `Op::Common` arm ends `... buildConeCylCommon(...)` (declines a sphere operand → NULL
→ OCCT); `Op::Fuse` / `Op::Cut` end `... buildConeCylFuse/Cut(...)` (likewise NULL). This change
appends `buildConeSphere{Common,Fuse,Cut}` as the final call of each arm.

## The geometry

A frustum cone A (frame `fA`, apex-free, `r_c(s) = R0 + s·tanα` over `s ∈ [coneS0, coneS1]`) and a
sphere B (radius `Rs`, centre ON the cone axis at axial station `s_c`) share the cone axis. Both are
SOLIDS OF REVOLUTION about that axis, so `A ∩ B`, `A ∪ B`, and `A − B` are ALL solids of revolution
— the whole problem lives in the `(s, r)` half-plane. A point `(s, r)` is: inside A iff
`coneS0 ≤ s ≤ coneS1 ∧ r ≤ r_c(s)`; inside B iff `(s − s_c)² + r² ≤ Rs²`, i.e. `r ≤ r_s(s) =
√(Rs² − (s − s_c)²)` for `|s − s_c| ≤ Rs`. The sphere wall closes on the axis at its two poles
`s = s_c ± Rs` (`r_s → 0`).

**The seam.** `intersectSphereConeCoaxial` solves `z(v)² + r(v)² = Rs²` — a QUADRATIC in the cone
parameter `v` (axial `z = s − s_c`, radius `r(v) = r_c(s)`), giving up to TWO circles. The
analytically-clean SINGLE-crossing configuration (this change's scope) is when exactly ONE root
`s*` falls strictly inside both operand extents and does NOT cross the cone apex — the sphere is on
the FRUSTUM side. At `s*`, `r_c(s*) = r_s(s*)` = the single seam circle (radius `r_c(s*)`, centred
on the axis). On one side of `s*` the cone is the tighter (smaller-radius) wall, on the other the
sphere is; the assembler reads the side from the two radii (as the cone∩cylinder builders do), not
a hard-coded direction. When TWO roots fall inside (the sphere passes fully through the cone), or a
root sits on the apex, the builder declines → OCCT.

The three op boundaries are three radial profiles:

| Op | side `r_c ≤ r_s` (cone tighter) | side `r_s ≤ r_c` (sphere tighter) | caps |
|---|---|---|---|
| COMMON `A ∩ B` | cone wall band (inside sphere) | sphere INNER cap → near pole (inside cone) | cone terminal disc inside sphere |
| FUSE `A ∪ B` | sphere OUTER cap → far pole (outside cone) | cone OUTER wall band (outside sphere) | cone terminal disc bounding the union |
| CUT `A − B` (cone minuend) | — (cone removed here) | cone OUTER wall band (outside sphere) + sphere INNER cap REVERSED | cone terminal disc(s) outside sphere |

All three weld the cone-side band to the sphere-side cap along the SAME single pooled seam ring.

### Worked example (the sim fixture)

Cone A `r_c(s) = 0.5 + 0.5s`, `s ∈ [0,4]` (`r_c: 0.5 → 2.5`); sphere B centre on-axis at `s_c = 0`,
`Rs = 2` (poles at `s = −2` and `s = +2`; `r_s(s) = √(4 − s²)`). The seam solves `5s² + 2s − 15 =
0` → `s* ≈ 1.5436` (the other root `< 0` is outside the frustum → single crossing), seam radius
`r_c(s*) = r_s(s*) ≈ 1.2718`. Cone tighter on `s ∈ [0, s*]` (`r_c ≤ r_s`); sphere tighter on
`s ∈ [s*, 2]` (`r_s ≤ r_c`, closing to the north pole at `s = 2`, which is INSIDE the cone since
`r_c(2) = 1.5 > 0`). `V(A) = 32.463`, `V(B) = 33.510`.

**COMMON** `A ∩ B` over `s ∈ [0, 2]`:

| `s`-range | tighter wall | fragment |
|---|---|---|
| cone bottom disc @ `s = 0` (`r ≤ 0.5`, inside sphere) | — | `appendDiskCap` (`−ẑ`) |
| `[0, s*]` cone tighter | cone wall | `appendRevolvedBand(ring(0.5,0) → ringSeam)` outward |
| seam ring @ `s*` (`r ≈ 1.2718`) | — | pooled once |
| `[s*, 2]` sphere tighter | sphere INNER cap | `appendSphereCap(B, apex = north pole @ s=2, seam, inner, outward)` |

`V(A ∩ B) = V_frustum(0.5 → 1.2718 over [0, s*]) + V_spherical-segment(s* → 2) = 4.0464 + 1.2094 =
5.256`. The spherical segment is the sphere cap of height `h = 2 − s* ≈ 0.4564`: `V_cap =
π h²(3Rs − h)/3 = 1.2094`.

**FUSE** `A ∪ B` = the sphere OUTER cap (outside the cone) welded to the cone OUTER wall:

| fragment | detail |
|---|---|
| sphere OUTER cap | `appendSphereCap(B, apex = FAR pole @ s = −2, seam, outer, outward)` — the part of B outside the cone (south pole → seam, crossing the equator) |
| seam ring @ `s*` | the SAME pooled ring COMMON uses |
| cone OUTER wall band `[s*, 4]` | `appendRevolvedBand(ringSeam → ring(2.5, 4))` outward — the cone outside the sphere |
| cone top disc @ `s = 4` | `appendDiskCap` (`+ẑ`, `r = 2.5`) |

`V = V(A) + V(B) − V(A ∩ B) = 60.718`. A GROW (`Vr > max(V(A), V(B))`). The cone bottom disc (inside
the sphere) and the sphere north pole (inside the cone) are SWALLOWED — not on the union boundary.

**CUT** `A − B` (cone minuend) = the cone with a spherical dimple carved in its narrow end:

| fragment | detail |
|---|---|
| cone OUTER wall band `[s*, 4]` | `appendRevolvedBand(ringSeam → ring(2.5, 4))` outward — cone outside sphere |
| cone top disc @ `s = 4` | `appendDiskCap` (`+ẑ`) |
| sphere INNER cap REVERSED | `appendSphereCap(B, apex = north pole @ s = 2, seam, inner, reversed)` — inward normal, the dimple bounding the cavity, pinching to the seam ring |

`V = V(A) − V(A ∩ B) = 27.207`. A SHRINK. The removed region is exactly the COMMON. Unlike the
cone∩cylinder CUT (disconnected end-frustum + washer), this is CONNECTED — one closed component:
the cone wall from `s*` up, capped on top, with the reversed spherical cap sealing the bottom
cavity. `buildConeSphereCut` binds `A` to operand `a` (order-sensitive, matches
`BRepAlgoAPI_Cut(a, b)`); a `sphere − cone` (sphere minuend) declines → OCCT.

## Goals / Non-Goals

**Goals**
- Add `coneSphereSetup(A, B, seams)` — the shared gate/seam prologue mirroring `coneCylSetup`: one
  `Cone` + one coaxial `Sphere` (centre on the cone axis via `sameAxis`/`distancePointLine`), the
  single interior crossing `s*` (declines a two-circle or apex crossing), the analytic-vs-traced
  seam cross-check, the shared frame + `rCone`/`ring`/`wallPoint` functors + azimuth `N`, the two
  sphere poles classified against the cone (inner/outer), and ONE canonical pooled seam ring emitted
  as a `Seam` welding the cone band to the spherical cap.
- Add `buildConeSphereCommon` / `buildConeSphereFuse` / `buildConeSphereCut`, each reusing the
  cone-side (`appendRevolvedBand` + `appendDiskCap`) and sphere-side (`appendSphereCap`) builders
  and the SAME pooled seam ring, returning a `Solid` or NULL → OCCT.
- Dispatch `Op::Common` → `buildConeSphereCommon`, `Op::Fuse` → `buildConeSphereFuse`, `Op::Cut` →
  `buildConeSphereCut` in `ssi_boolean_solid`; recognition + trace + the transversality gate
  UNCHANGED.
- Add ONE coaxial cone∩sphere COMMON analytic arm to `ssiCurvedBooleanVerified` (`V_frustum +
  V_spherical-segment`), mirroring the Steinmetz and cone∩cylinder arms. FUSE / CUT reuse the
  EXISTING generic set-algebra guard (`va + vb − vc` / `va − vc`, `vc` = native cone∩sphere COMMON),
  with the correct per-op sign (fuse grows, cut shrinks).

**Non-Goals (deferred — never faked)**
- TWO-circle coaxial cone∩sphere crossings (the sphere passes fully through the cone / spans the
  apex, `intersectSphereConeCoaxial` returns two in-extent circles) → NULL → OCCT.
- TRANSVERSAL (non-coaxial) cone∩sphere — a genuine quartic space curve, NOT an analytic circle
  (`intersectSphereConeCoaxial` returns `notAnalytic`) → the general marcher / OCCT.
- APEX-CROSSING seams and a frustum whose extent INCLUDES the apex (`r_c → 0`) → NULL → OCCT.
- A `sphere − cone` CUT (sphere minuend) → NULL → OCCT (this change ships the cone-minuend cut only;
  the sphere-with-a-conical-bite is a follow-on).
- cone∩cone (any op), and every other curved-curved family (through-drill cyl∩cyl, sphere∩sphere
  lens, Steinmetz, coaxial cone∩cylinder) — UNCHANGED.
- Any change to `src/native/tessellate`, the `cc_*` ABI, the planar BSP-CSG, the analytic
  `curved.h`, the cyl / sphere / Steinmetz / cone∩cylinder builders, or the generic set-algebra
  self-verify.
- Weakening ANY tolerance to force a pass. If COMMON / FUSE / CUT cannot be built watertight with
  the correct volume, return NULL → OCCT and report the measured gap.

## Module shape

```
src/native/boolean/ssi_boolean.cpp             [CYBERCAD_HAS_NUMSCI]
  appendRevolvedBand(...)                  // UNCHANGED — ring→ring band, ±radial
  appendDiskCap(...)                       // UNCHANGED — axis-point → rim fan, ±ẑ
  appendSphereCap(...)                     // UNCHANGED — slerped ring fan, inner/outer, ±normal
  coneSphereSetup(A, B, seams) -> ConeSphereSetup   // NEW — the shared gate/seam prologue
  buildConeSphereCommon(A, B, seams)       // NEW — cone band + seam + sphere inner cap + cone disc
  buildConeSphereFuse(A, B, seams)         // NEW — sphere outer cap + seam + cone outer band + disc
  buildConeSphereCut(A, B, seams)          // NEW — cone outer band + disc + reversed sphere inner cap
  ssi_boolean_solid(...)                   // dispatch: each op arm → new builder as final call

src/engine/native/native_engine.cpp
  ssiCurvedBooleanVerified(...)            // NEW arm — coaxial cone∩sphere COMMON closed form
```

No new files; reuses `recogniseCurvedSolid`, `sameAxis`, `classifyPoint`, `appendRevolvedBand`,
`appendDiskCap`, `appendSphereCap`, `decimateSeam` / `seamNodeTarget`, `pushPlanarTri`,
`VertexPool`, and the S1 `intersectSphereConeCoaxial` seam. OCCT-free (the engine arm is native-
math only).

## Shared prologue `coneSphereSetup` (mirrors `coneCylSetup`)

```cpp
struct ConeSphereSetup {
  bool ok = false;
  const CurvedSolid* cone = nullptr;   // = A or B
  const CurvedSolid* sph  = nullptr;
  math::Point3 O; math::Vec3 zc, X, Y; // shared cone-axis frame
  double tanA = 0, Rs = 0, sc = 0;     // cone slope, sphere radius, sphere centre station
  double sStar = 0;                    // single interior crossing r_c(s*) = r_s(s*)
  double coneS0 = 0, coneS1 = 0;       // cone s-extent
  bool coneTighterBelow = false;       // side of s* on which the cone is the tighter wall
  math::Point3 innerPole, outerPole;   // sphere pole inside / outside the cone
  int N = 0;
  double rCone(double s) const;                                 // R0 + s·tanA
  double rSph(double s)  const;                                 // √(Rs² − (s − sc)²)
  std::vector<math::Point3> ring(double r, double s) const;     // N nodes at (r, s)
  math::Point3 wallPoint(double r, double s) const;             // azimuth-0 classifier sample
  Seam seamRing() const;                                        // the canonical pooled seam ring
};
ConeSphereSetup coneSphereSetup(const CurvedSolid& A, const CurvedSolid& B,
                                const std::vector<Seam>& seams);
```

The prologue GATES: exactly one `Cone` + one `Sphere`; the sphere centre on the cone axis
(`distancePointLine(sph.origin, cone.origin, ẑ) ≤ 1e-6`); a single closed full-circle seam
(`seams.size() == 1`, `seam.closed`); a non-degenerate `tanα`; an apex-free frustum. It re-solves
the `intersectSphereConeCoaxial` quadratic in the cone's `s`-coordinate, requires EXACTLY ONE root
`s*` strictly interior to both extents (declines two-in-extent-roots and apex crossings), and
CROSS-CHECKS the analytic seam (height `s*`, radius `r_c(s*)`) against the traced seam centroid +
mean radius. It resolves the azimuth `N` from the seam-circle chord sagitta (same `kCapSagitta`
bound as the caps), classifies the two sphere poles (`s_c ± Rs`) against the cone to set
`innerPole` (inside) / `outerPole` (outside), and builds ONE canonical seam ring `s.ring(r_c(s*),
s*)` returned as a `Seam` (`.pts` = the ring nodes) so BOTH the cone band and `appendSphereCap`
(whose outer ring is `seam.pts`) weld on the IDENTICAL nodes. `ok == false` on any decline.

## `buildConeSphereCommon` — cone band + sphere inner cap (min cross-section)

```
gate:      s = coneSphereSetup(A, B, seams); if (!s.ok) return {};
seam:      Seam seamR = s.seamRing();  // N pooled nodes at (r_c(s*), s*)
cone band: on the cone-tighter side, appendRevolvedBand(ring(rConeTerm, sTerm) → seamR.pts, outward)
           where sTerm is the cone terminal inside the sphere; kept iff its mid-sample classifies
           strictly INSIDE the sphere (classifyPoint(sph, mid) == 1).
cone cap:  appendDiskCap at that cone terminal (the cone end disc inside the sphere), ±ẑ.
sph cap:   appendSphereCap(sph, otherCentre = a point toward the cone interior, seamR,
                           ringsFor(sph, innerPole), pool, faces, outer=false, reversed=false);
           kept iff s.innerPole classifies strictly INSIDE the cone (classifyPoint(cone, innerPole)
           == 1); the cap's outer ring reuses seamR.pts → welds to the cone band.
assemble:  if (faces.size() < 4) return {}; makeShell → makeSolid.
```

Volume of the welded shell = `V_frustum(cone-tighter sub-band) + V_spherical-segment(sphere-tighter
sub-band)` = `V(A ∩ B)`.

## `buildConeSphereFuse` — sphere outer cap + cone outer band (max cross-section)

```
gate:      s = coneSphereSetup(A, B, seams); if (!s.ok) return {};
survival:  s.outerPole must classify strictly OUTSIDE the cone (classifyPoint(cone, outerPole)
           == -1); else {} → OCCT.
sph cap:   appendSphereCap(sph, otherCentre, seamR, ringsFor(sph, outerPole), pool, faces,
                           outer=true, reversed=false)  — the sphere OUTSIDE the cone (far pole → seam).
cone band: appendRevolvedBand(seamR.pts → ring(rConeTop, coneTop), outward) — the cone OUTSIDE the
           sphere; kept iff its mid-sample classifies strictly OUTSIDE the sphere
           (classifyPoint(sph, mid) == -1).
cone cap:  appendDiskCap at the cone terminal bounding the union, ±ẑ.
assemble:  if (faces.size() < 4) return {}; makeShell → makeSolid.
```

Volume = `V(A) + V(B) − V(A ∩ B)` (a GROW). The seam ring `seamR` is the SAME pooled ring COMMON
uses — in FUSE it welds the sphere outer cap to the cone outer band.

## `buildConeSphereCut` — cone outer band + reversed sphere inner cap (cone minuend)

```
gate:      s = coneSphereSetup(A, B, seams); if (!s.ok) return {};
           if (&A != s.cone) return {};  // A must be the cone minuend; sphere − cone → OCCT
cone band: appendRevolvedBand(seamR.pts → ring(rConeTop, coneTop), outward) — cone OUTSIDE sphere;
           kept iff its mid-sample classifies strictly OUTSIDE the sphere (== -1).
cone caps: appendDiskCap at the cone terminal(s) outside the sphere, ±ẑ.
sph cap:   appendSphereCap(sph, otherCentre, seamR, ringsFor(sph, innerPole), pool, faces,
                           outer=false, reversed=true)  — the sphere INNER cap REVERSED (inward
           normal), bounding the carved dimple, pinching to the seam ring; kept iff s.innerPole
           classifies strictly INSIDE A (== 1).
assemble:  if (faces.size() < 4) return {}; makeShell → makeSolid  (ONE connected component).
```

Volume = `V(A) − V(A ∩ B)` (a SHRINK). The reversed inner cap's normal points into the removed
material (the dimple). CUT is order-sensitive; `buildConeSphereCut` honours the operand order.

## Driver dispatch (`ssi_boolean_solid`) — extended

Recognition + trace + the transversality gate are UNCHANGED. Extend the three op arms (each grows
one final call after the through-drill / lens / cone∩cylinder builders decline):

```cpp
case Op::Common: {
  if (topo::Shape drill = buildCommon(*csA, *csB, seams);       !drill.isNull()) return drill;
  if (topo::Shape lens  = buildLensCommon(*csA, *csB, seams);   !lens.isNull())  return lens;
  if (topo::Shape ccyl  = buildConeCylCommon(*csA, *csB, seams);!ccyl.isNull())  return ccyl;
  return buildConeSphereCommon(*csA, *csB, seams);   // S5-f coaxial cone∩sphere common
}
case Op::Fuse: { ... buildConeCylFuse ...; return buildConeSphereFuse(*csA, *csB, seams); }
case Op::Cut:  { ... buildConeCylCut  ...; return buildConeSphereCut(*csA, *csB, seams); }
```

`buildConeSphere{Common,Fuse,Cut}` return `{}` for any non-(cone + coaxial-sphere) pair or any
declined gate, so every existing path is untouched — no regression.

## Engine self-verify — new COMMON arm + per-op sign (fuse grows, cut shrinks)

`native_engine.cpp` `ssiCurvedBooleanVerified` (~line 332) returns `{}` unless `op == 2` (COMMON),
so the new arm gates COMMON only; FUSE / CUT flow to the generic `booleanResultVerified`. Add a
coaxial cone∩sphere arm alongside the Steinmetz and cone∩cylinder arms:

```cpp
// ── S5-f arm: COAXIAL cone(frustum)∩sphere COMMON (single analytic circle). ──
const auto* cone = ... (the Cone operand); const auto* sph = ... (the Sphere operand);
if (cone && sph) {
    // gate: sphere centre on the cone axis; single interior crossing s*.
    // expected = V_frustum(cone-tighter sub-band) + V_spherical-segment(sphere-tighter sub-band).
    //   V_frustum(ra, rb, Δh) = (π Δh/3)(ra² + ra·rb + rb²)
    //   V_seg(s1, s2)         = π [Rs²(s2 − s1) − ((s2 − s_c)³ − (s1 − s_c)³)/3]
    const double vr = watertightVolume(result);
    if (vr < 0.0) return {true, false};             // not watertight
    const double tol = std::max(1e-2 * expected, 1e-6);   // deflection-bounded curved mesh
    return {true, std::fabs(vr - expected) <= tol};
}
```

For FUSE / CUT `booleanResultVerified` runs `vc = watertightVolume(boolean_solid(a, b, Op::Common))`
= the native `buildConeSphereCommon` = `V(A ∩ B)`, `expected = va + vb − vc` (fuse) / `va − vc`
(cut). For the fixture: `va = 32.463`, `vb = 33.510`, `vc = 5.256` → `expected(fuse) = 60.718`,
`expected(cut) = 27.207` — matching OCCT within the deflection-bounded tolerance. A mis-selected /
mis-oriented / non-watertight candidate fails and is DISCARDED → OCCT.

## Verification plan

- **Host (no OCCT), analytic inclusion-exclusion dual oracle.** `V(A ∩ B) = V_frustum(r_c(sLo) →
  r_c(s*)) + V_spherical-segment(s* → pole)`, `V(cone frustum) = (π Δh/3)(r0² + r0·r1 + r1²)`,
  `V(sphere) = 4/3·π Rs³`; `FUSE = V(A) + V(B) − V(A ∩ B)`, `CUT(A,B) = V(A) − V(A ∩ B)`. The host
  test asserts, for the reference coaxial cone∩sphere pair: a watertight shell (`boundaryEdgeCount
  == 0`, every edge shared by exactly two faces), enclosed volume matching the closed form within
  the deflection band, every seam-ring node on BOTH walls ≤ tol, the seam ring pooled ONCE; a two-
  circle / apex-in-extent / transversal / sphere-minuend fixture → NULL. Green with NUMSCI on AND
  off (the cone∩sphere path correctly absent off).
- **Sim native-vs-OCCT.** Add a `cone=sphere(coax)` fixture to
  `scripts/run-sim-native-ssi-curved-boolean.sh` + `tests/sim/native_ssi_curved_boolean_parity.mm`
  (cone `r_c = 0.5 + 0.5s` over `[0,4]`, sphere centre on-axis at the origin, `Rs = 2`) and compare
  native vs `BRepAlgoAPI_{Common,Fuse,Cut}` (volume ≈ `5.256` common / `60.718` fuse / `27.207` cut,
  surface area, watertight closed shell, valid shape), raising **native-pass 15 → 18**. Do NOT
  regress the 15 existing native passes. Any pair whose self-verify does not pass stays an honest
  fall-back with the measured gap reported — no tolerance weakened.
- `openspec validate add-native-cone-sphere-boolean --strict` green; note the coaxial cone∩sphere
  op-set now 3/3 native in `SSI-ROADMAP.md` / `ROADMAP.md` / `NATIVE-REWRITE.md` with the measured
  deltas. Confirm no SSI / blend / heal / import / marching / phase3 suite regresses.

## Cognitive complexity

`buildConeSphere{Common,Fuse,Cut}` are near-clones (shared `coneSphereSetup` prologue → cone-side
`appendRevolvedBand` + `appendDiskCap` → sphere-side `appendSphereCap` → shell/solid), each in the
systems band (~14–18, comparable to `buildConeCyl*` and `buildLens*`). `coneSphereSetup` carries the
gate/seam/pole prologue (~18–22, comparable to `coneCylSetup`). The new `ssiCurvedBooleanVerified`
arm is a straight closed-form evaluation (~12, mirroring the cone∩cylinder arm). No function is
pushed above the documented systems band; the reused helpers (`appendRevolvedBand`, `appendDiskCap`,
`appendSphereCap`, `classifyPoint`) are UNCHANGED.
