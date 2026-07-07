# Design — complete-cone-cyl-fuse-cut (SSI Stage S5-e completion)

## Context

`add-native-cone-boolean` (S5-e) shipped, in `src/native/boolean/ssi_boolean.cpp` (OCCT-free,
`CYBERCAD_HAS_NUMSCI`-gated):

- `buildConeCylCommon(A, B, seams)` (~line 1070) — the coaxial cone∩cylinder COMMON assembler. It
  (1) GATES: exactly one `Cone` + one `Cylinder`, coaxial (`ssidetail::sameAxis`), a single closed
  full-circle seam (`seams.size() == 1`, `seam.closed`), a non-degenerate `tanα`; (2) computes the
  cylinder axial extent in the cone's `s`-coordinate (`s = dot(P − O, ẑ)`, sign-corrected for
  antiparallel axes), the overlap `[sLo, sHi] = [max(coneLo, cylLo), min(coneHi, cylHi)]`, and the
  single interior crossing `s* = (Rc − R0)/tanα` (declines apex/edge crossings); (3) cross-checks
  the ANALYTIC seam (height `s*`, radius `Rc`) against the TRACED seam centroid + mean radius; (4)
  builds three rings via a `ring(r, s)` functor on the shared frame `(O, ẑ, X, Y)` — `ringBot =
  ring(min(r_c(sLo), Rc), sLo)`, `ringSeam = ring(Rc, s*)`, `ringTop = ring(min(r_c(sHi), Rc),
  sHi)`; (5) emits two `appendRevolvedBand`s (`ringBot→ringSeam`, `ringSeam→ringTop`, outward
  radial) + two `appendDiskCap`s (`−ẑ` at `sLo`, `+ẑ` at `sHi`) through ONE `VertexPool`; (6)
  `makeShell → makeSolid`, or `{}` on any decline.
- Shared machinery: `recogniseCurvedSolid` (folds a solid into `CurvedSolid{kind, frame, radius,
  semiAngle, vLo, vHi, capPlanes}` — handles `Cone`), `classifyPoint` (the curved point-in-solid
  test — scores the cone wall half-space and the cylinder wall half-space), `appendRevolvedBand`
  (~line 1039, a ring-to-ring quad-strip band with outward-radial planar-facet normals),
  `appendDiskCap` (~line 630, the axis-point → rim fan disc, `±axis` normal), `pushPlanarTri`, and
  the shared `VertexPool`.

The dispatcher `ssi_boolean_solid` (~line 1845) recognises both operands, traces the seam, gates
on full transversality (`nearTangentGaps == 0`, `branchPoints == 0`), builds `Seam`s, and switches
on op. Its `Op::Common` arm already dispatches the coaxial cone∩cylinder pair to
`buildConeCylCommon`. Its `Op::Fuse` arm runs `buildFuse` (through-drill, declines a single seam)
then `buildLensFuse` (declines a non-sphere operand) → NULL; its `Op::Cut` arm runs `buildCut` then
`buildLensCut` → NULL. So the coaxial cone∩cylinder FUSE / CUT fall through to OCCT.

`add-native-cone-boolean`'s Non-Goals defer cone FUSE / CUT to a follow-on. This change is that
follow-on. It is a BOUNDED completion: the SAME analytic circle seam, a DIFFERENT band selection +
cap handling. Nothing about the gate, the seam cross-check, the `s`-coordinate math, the ring
functor, or `classifyPoint` changes.

## The geometry

A frustum cone A (frame `fA`, apex-free, `r_c(s) = R0 + s·tanα` over `s ∈ [coneLo, coneHi]`) and a
coaxial cylinder B (radius `Rc`, over `s ∈ [cylLo, cylHi]` in the cone's `s`-coordinate) share one
axis. Both are SOLIDS OF REVOLUTION about the shared axis, so `A ∩ B`, `A ∪ B`, and `A − B` are ALL
solids of revolution — the whole problem lives in the `(s, r)` half-plane. A point `(s, r)` is:
inside A iff `coneLo ≤ s ≤ coneHi ∧ r ≤ r_c(s)`; inside B iff `cylLo ≤ s ≤ cylHi ∧ r ≤ Rc`.

The walls cross at the single seam height `s*` where `r_c(s*) = Rc` — the S1 analytic circle
(radius `Rc`, centred on the axis at `s*`), gated STRICTLY interior to the overlap `[sLo, sHi] =
[max(coneLo, cylLo), min(coneHi, cylHi)]`. On one side of `s*` the cone is the tighter (smaller-
radius) wall, on the other the cylinder is; the assembler reads the side from the two radii (as
COMMON does), not a hard-coded widening/narrowing direction.

The three op boundaries are three radial profiles over the relevant `s`-range:

| Op | `s`-range | Profile | Boundary |
|---|---|---|---|
| COMMON `A ∩ B` (done) | `[sLo, sHi]` | `min(r_c(s), Rc)` | inside bands + 2 disc caps |
| FUSE `A ∪ B` (new) | `[min(coneLo,cylLo), max(coneHi,cylHi)]` | `max(r_c(s), Rc)` where both present, else the present wall | outer bands + 2 terminal disc caps + annular step caps |
| CUT `A − B` (new) | inside A ∧ ¬inside B | A-outer ∪ reverse(B-inside) | A outer bands + A caps + A cap-annulus + reversed B inside band + reversed B cap disc |

### Worked example (the sim fixture)

Cone A `r_c(y) = 0.5 + 0.5y`, `[0,4]` (`r_c: 0.5 → 2.5`); cylinder B `Rc = 1.5`, `[1,5]`. Overlap
`[1,4]`; `s* = 2` (`r_c(2) = 1.5 = Rc`). Cone tighter on `[1,2]` (`r_c: 1.0 → 1.5`), cylinder
tighter on `[2,4]`. `V(A) = 32.463`, `V(B) = 28.274`, `V(A ∩ B) = V_frustum(1.0 → 1.5) + π·1.5²·2 =
4.974 + 14.137 = 19.111` (native `19.107`).

**FUSE outer profile** `max(r_c, Rc)` over the union extent `[0,5]`:

| `s`-range | outer wall | rings (`r @ s`) |
|---|---|---|
| `[0,1]` cone only (below cyl) | cone wall | `R0 = (0.5 @ 0)` → `R1i = (1.0 @ 1)` |
| step at `s = 1` (cyl bottom cap protrudes) | B-bottom annulus `−ẑ` | `R1i = (1.0 @ 1)` → `R1o = (1.5 @ 1)` |
| `[1,2]` cyl tighter-in-common ⇒ cyl OUTER | cyl wall | `R1o = (1.5 @ 1)` → `Rseam = (1.5 @ 2)` |
| `[2,4]` cone tighter-in-common ⇒ cone OUTER | cone wall | `Rseam = (1.5 @ 2)` → `R4o = (2.5 @ 4)` |
| step at `s = 4` (cone top cap protrudes) | A-top annulus `+ẑ` | `R4i = (1.5 @ 4)` ← `R4o = (2.5 @ 4)` |
| `[4,5]` cyl only (above cone) | cyl wall | `R4i = (1.5 @ 4)` → `R5 = (1.5 @ 5)` |

closed by the cone bottom disc cap (`R0`, `−ẑ`) at `s = 0` and the cyl top disc cap (`R5`, `+ẑ`) at
`s = 5`. `Rseam` is the SAME pooled seam circle COMMON uses — in FUSE it welds the cyl outer band
(below `s*`) to the cone outer band (above `s*`); in COMMON it welds the cone inner band to the cyl
inner band. Volume `= V(A) + V(B) − V(A ∩ B) = 41.626`. A GROW (`Vr > max(V(A), V(B))`).

**CUT `A − B`** = (∂A outside B) ∪ reverse(∂B inside A). For the fixture this is DISCONNECTED (the
cylinder scoops the cone fully through on `[1,2]`):

- Piece 1 — a small end frustum on `[0,1]` (A below the cylinder, capped where the cylinder's
  bottom disc carves it): cone wall `R0 → R1i` (`0.5 → 1.0`, outward) + cone bottom disc (`R0`,
  `−ẑ`) + B-bottom disc-inside-A REVERSED (the disc `r ≤ 1.0 @ s = 1`, fanned to `R1i`, outward
  `+ẑ` — B's bottom cap now faces up, out of the removed material). `V₁ = V_frustum(0.5 → 1.0) =
  1.833`.
- Piece 2 — a conical washer on `[2,4]` (A outside B, hollowed by B): cone wall `Rseam → R4o`
  (`1.5 → 2.5`, outward) + A-top annulus (`R4i → R4o`, `+ẑ`) + cyl wall `Rseam → R4i` (`1.5 @ 2 →
  1.5 @ 4`) emitted REVERSED (inward radial — B's inside band bounds the cavity). The outer cone
  band and the reversed inner cyl band both emanate from `Rseam` at `s* = 2` → the seam ring welds
  them (the washer pinches to the seam circle, no bottom cap). `V₂ = V_frustum(1.5 → 2.5) − π·1.5²·2
  = 25.656 − 14.137 = 11.519`.

`V(A − B) = V₁ + V₂ = 13.352 = V(A) − V(A ∩ B)`. A SHRINK (`Vr < V(A)`). CUT is order-sensitive
(`buildConeCylCut` binds `A` to operand `a`, matching `BRepAlgoAPI_Cut(a, b)`). The two closed
components are assembled into ONE face list → one shell → one solid; the divergence-theorem mesh
volume sums both and the watertight check holds per component (every edge shared by exactly two
faces within its component). No component special-casing is needed — disjoint pieces simply do not
share pooled rings.

## Goals / Non-Goals

**Goals**
- Reuse the S5-e seam/split/weld (`coneCylSetup` gate + seam ring + `ring` functor +
  `appendRevolvedBand` + `appendDiskCap`) verbatim; the only new logic is the per-op profile (MIN
  for COMMON, MAX for FUSE, A-outer ∪ reversed-B-inner for CUT), the survival rule per band, and
  (FUSE/CUT) the annular step caps + reversed fragments.
- Add `appendAnnulusCap` — a flat annular ring (washer) between two coaxial same-station rings,
  axial `±ẑ` normal, both rings pooled — closing a radial step where an end-cap disc protrudes.
- Add `buildConeCylFuse` (max-radius outer profile + beyond-overlap segments + terminal disc caps +
  annular step caps) and `buildConeCylCut` (A outer + A caps + A cap-annulus + reversed B inside
  band + reversed B cap disc), both sharing the SAME pooled seam ring as COMMON so they weld
  watertight, returning a `Solid` or NULL → OCCT.
- Dispatch `Op::Fuse` → `buildConeCylFuse`, `Op::Cut` → `buildConeCylCut` in `ssi_boolean_solid`;
  recognition + trace + the transversality gate UNCHANGED.
- Keep the ENGINE self-verify UNCHANGED — the generic set-algebra guard already covers fuse
  (`va + vb − vc`) and cut (`va − vc`) via the native cone∩cylinder COMMON (`vc = V(A ∩ B)`), with
  the correct per-op sign (fuse grows, cut shrinks). The `op == 2`-only analytic oracle does NOT
  intercept fuse/cut.

**Non-Goals (deferred — never faked)**
- TRANSVERSAL (non-coaxial) cone∩cylinder — a genuine quartic space curve, NOT an analytic circle
  here (`intersectCylinderConeCoaxial` returns `notAnalytic`) → the general marcher / OCCT.
  UNCHANGED.
- APEX-CROSSING seams and a frustum whose extent INCLUDES the apex (`r_c → 0`) — the S4-e apex
  chart singularity → NULL → OCCT (refused at the shared gate).
- A cap-edge-TANGENT seam (`s*` on a cap plane, not strictly interior) → NULL → OCCT.
- coaxial cone∩sphere (any op) and cone∩cone (any op) → NULL → OCCT.
- Any other curved-curved family (through-drill cyl∩cyl, sphere∩sphere lens, Steinmetz) —
  UNCHANGED.
- Any change to `src/native/tessellate`, the `cc_*` ABI, the planar BSP-CSG, the analytic
  `curved.h`, the cyl / sphere / Steinmetz builders, `buildConeCylCommon`, or the engine self-
  verify.
- Weakening ANY tolerance to force a pass. If FUSE or CUT cannot be built watertight with the
  correct volume, return NULL → OCCT and report the measured gap.

## Module shape

```
src/native/boolean/ssi_boolean.cpp             [CYBERCAD_HAS_NUMSCI]
  appendRevolvedBand(...)                  // UNCHANGED — ring→ring band, outward radial
  appendDiskCap(...)                       // UNCHANGED — axis-point → rim fan, ±axis
  appendAnnulusCap(ringIn, ringOut, axialOut, pool, faces)   // NEW — flat washer, ±axis
  coneCylSetup(A, B, seams) -> ConeCylSetup                  // NEW — the shared gate/seam prologue
  buildConeCylCommon(A, B, seams)          // UNCHANGED result — now calls coneCylSetup
  buildConeCylFuse(A, B, seams)            // NEW — max-profile outer + caps + annuli
  buildConeCylCut(A, B, seams)             // NEW — A outer + A caps + reversed B inside
  ssi_boolean_solid(...)                   // dispatch: Op::Fuse/Op::Cut → new builders
```

No new files; reuses `recogniseCurvedSolid`, `sameAxis`, `classifyPoint`, `appendRevolvedBand`,
`appendDiskCap`, `pushPlanarTri`, `VertexPool`, and the S1 `intersectCylinderConeCoaxial` seam.
OCCT-free. NO engine edit.

## Shared prologue (factored from `buildConeCylCommon`)

All three builders need the IDENTICAL prologue: the gate (one `Cone` + one coaxial `Cylinder`,
single strictly-interior full-circle apex-free seam), the analytic-vs-traced seam cross-check, the
`s`-coordinate extents, `s*`, the azimuth resolution `N`, the shared frame, the `ring(r, s)`
functor, and the pooled seam ring at `(Rc, s*)`. Today that prologue lives inline in
`buildConeCylCommon` (~lines 1071–1149). Factor it into a small helper so FUSE and CUT reuse it
byte-for-byte:

```cpp
struct ConeCylSetup {
  bool ok = false;
  const CurvedSolid* cone = nullptr;   // = A or B
  const CurvedSolid* cyl = nullptr;
  math::Point3 O; math::Vec3 zc, X, Y; // shared axis frame
  double Rc = 0, tanA = 0, sStar = 0;  // cyl radius, cone slope, seam height
  double sLo = 0, sHi = 0;             // axial overlap
  double coneLo = 0, coneHi = 0, cylLo = 0, cylHi = 0;  // per-operand s-extents
  int N = 0;                            // azimuth samples
  double rCone(double s) const;         // R0 + s·tanA
  std::vector<math::Point3> ring(double r, double s) const;  // N nodes at (r, s)
};
ConeCylSetup coneCylSetup(const CurvedSolid& A, const CurvedSolid& B,
                          const std::vector<Seam>& seams);
```

`buildConeCylCommon` calls it and its output is byte-identical to the current inline result (same
gate, same `N`, same rings) → COMMON stays byte-identical, native-pass 13 not regressed. FUSE and
CUT call the SAME helper → the SAME frame + `s*` + seam ring, so a FUSE outer band and the COMMON
inner band it abuts share IDENTICAL seam-circle nodes.

## `appendAnnulusCap` — a flat washer (NEW)

```cpp
// Emit a flat annular ring (washer) between two coaxial rings at the SAME axial station:
// ringIn (inner radius) and ringOut (outer radius), N nodes each, u-aligned. Planar facets
// with a FIXED axial outward normal (±ẑ). Both rings are drawn through the shared pool so the
// washer welds to the inner-edge wall/disc and the outer-edge wall. Mirrors appendRevolvedBand
// but forces the normal AXIAL (not radial) because the washer lies in a constant-s plane.
void appendAnnulusCap(const std::vector<math::Point3>& ringIn,
                      const std::vector<math::Point3>& ringOut,
                      const math::Vec3& axialOutward, VertexPool& pool,
                      std::vector<topo::Shape>& faces);
```

Used at `s = cylLo` when the cylinder's bottom cap protrudes past the cone (`Rc > r_c(cylLo)`) and
at `s = coneHi` when the cone's top cap protrudes past the cylinder (`r_c(coneHi) > Rc`), and
symmetrically for the other end arrangement. When the two radii are equal (no step, the cap plane
sits exactly at a wall crossing) the annulus is degenerate and is skipped (the walls meet
directly).

## `buildConeCylFuse` — the max-radius outer profile (mirror of COMMON)

```
gate:      s = coneCylSetup(A, B, seams); if (!s.ok) return {};
rings:     build the ordered profile stations from the sorted set {coneLo, cylLo, s*, coneHi,
           cylHi} restricted to the union extent [min(coneLo,cylLo), max(coneHi,cylHi)]; at each
           station the OUTER radius = max of the present operands' radii (r_c(s) if cone present,
           Rc if cyl present), with the pooled seam ring reused at s*.
bands:     for each consecutive station pair on the same operand's outer wall, appendRevolvedBand
           (outward radial); a wall segment is KEPT iff its mid-sample classifies strictly OUTSIDE
           the other solid (classifyPoint(other, mid) == -1; an ON verdict == 0 → {}).
annuli:    at each cap plane where one operand's disc protrudes past the other's wall radius,
           appendAnnulusCap(innerRing, outerRing, ±ẑ) — the protruding end-cap annulus.
disc caps: appendDiskCap at the two union terminals (min-s end −ẑ, max-s end +ẑ), each fanned to
           that end's outer ring.
assemble:  if (faces.size() < 4) return {}; makeShell → makeSolid.
```

The outer bands stitch the wider operand's wall across the union along the SAME pooled seam ring;
the annular caps close the end-cap steps; the two disc caps close the union's axial ends. Volume of
the welded shell `= V(A) + V(B) − V(A ∩ B)`. The engine self-verify checks this against
`va + vb − vc` (`vc` = native `buildConeCylCommon`).

## `buildConeCylCut` — A outer + A caps + reversed B inside (mirror of COMMON)

```
gate:      s = coneCylSetup(A, B, seams); if (!s.ok) return {}; A is the minuend.
A outer:   A's wall segments where A classifies OUTSIDE B (below/above the cylinder + the cone-
           tighter side where r_c > Rc), appendRevolvedBand (outward radial), kept iff the mid-
           sample classifies strictly OUTSIDE B (== -1).
A caps:    A's terminal disc cap(s) where the full disc is outside B (appendDiskCap ±ẑ), and A's
           cap-annulus where A's end-cap disc extends past B (appendAnnulusCap ±ẑ).
B inside:  B's wall band INSIDE A (the cyl band on the r_c ≥ Rc side, from s* to the overlap
           terminal) emitted REVERSED (appendRevolvedBand with the INWARD radial reference), kept
           iff its mid-sample classifies strictly INSIDE A (== 1; ON == 0 → {}) — welded at the
           pooled seam ring so the reversed inner band pinches to the seam circle.
B cap:     B's end-cap disc INSIDE A (the part r ≤ r_c at a cylinder cap plane inside A) emitted
           REVERSED (appendDiskCap with the opposite axial normal) — the cavity floor/ceiling.
assemble:  if (faces.size() < 4) return {}; makeShell → makeSolid (one shell may carry two
           disjoint closed components).
```

A's outer wall + A's caps form A's remaining outer shell; B's inside band reversed + B's cap disc
reversed bound the carved cavity (their normals point into the removed material). All fragments
share the pooled seam ring + the terminal rings → watertight (per component). Volume `= V(A) −
V(A ∩ B)`. CUT is order-sensitive: `buildConeCylCut` honours the operand order, matching
`BRepAlgoAPI_Cut(a, b)`.

## Driver dispatch (`ssi_boolean_solid`) — extended

Recognition + trace + the transversality gate are UNCHANGED. Extend only the `Op::Fuse` /
`Op::Cut` arms (each grows one final call after the through-drill / lens builders decline):

```cpp
case Op::Fuse: {
  if (topo::Shape drill = buildFuse(*csA, *csB, seams); !drill.isNull()) return drill;
  if (topo::Shape lens  = buildLensFuse(*csA, *csB, seams); !lens.isNull()) return lens;
  return buildConeCylFuse(*csA, *csB, seams);   // S5-e coaxial cone∩cylinder fuse
}
case Op::Cut: {
  if (topo::Shape drill = buildCut(*csA, *csB, seams); !drill.isNull()) return drill;
  if (topo::Shape lens  = buildLensCut(*csA, *csB, seams); !lens.isNull()) return lens;
  return buildConeCylCut(*csA, *csB, seams);    // S5-e coaxial cone∩cylinder cut (A minuend)
}
```

`buildConeCylFuse` / `buildConeCylCut` return `{}` for any non-(cone+coaxial-cylinder) pair or any
declined gate, so the through-drill and lens paths are untouched — every existing FUSE/CUT pass
keeps its result (no regression).

## Engine self-verify — per-op sign (fuse grows, cut shrinks), NO change

`native_engine.cpp` `ssiCurvedBooleanVerified` (~line 332) returns `{}` (not applicable) unless
`op == 2` (COMMON), so it does NOT intercept cone∩cylinder FUSE/CUT. `booleanResultVerified`
(~line 401) therefore runs the generic set-algebra guard for fuse/cut:

```
vc = watertightVolume(boolean_solid(a, b, Op::Common));   // native buildConeCylCommon = V(A∩B)
switch (op) { case 0: expected = va + vb − vc;   // fuse — GROWS
              case 1: expected = va − vc;         // cut  — SHRINKS
              case 2: expected = vc; }            // common (analytic oracle path)
accept iff |Vr − expected| <= max(1e-6·expected, 1e-9) AND watertight
```

For the fixture: `va = 32.463`, `vb = 28.274`, `vc = 19.107` (native common), so `expected(fuse) =
41.630` and `expected(cut) = 13.356` — matching the OCCT `volO` fuse `= 41.626` / cut `= 13.352`
within the deflection-bounded tolerance. No engine edit is needed; the correct per-op sign is
already wired. A mis-selected / mis-oriented / non-watertight candidate fails the check and is
DISCARDED → OCCT.

## Verification plan

- **Host (no OCCT), analytic inclusion-exclusion dual oracle.** `V(A ∩ B) = V_frustum(r(sLo) → Rc)
  + π Rc²·(sHi − s*)` (the native cone∩cylinder common), `V(cone frustum) = (π Δh/3)(r0² + r0·r1 +
  r1²)`, `V(cyl) = π Rc² L`; `FUSE = V(A) + V(B) − V(A ∩ B)`, `CUT(A,B) = V(A) − V(A ∩ B)`. The host
  test asserts, for the reference coaxial cone∩cylinder pair: a watertight shell
  (`boundaryEdgeCount == 0`, every edge shared by exactly two faces), enclosed (summed for a
  disconnected CUT) volume matching the closed form within the deflection band, every seam-ring node
  on BOTH walls ≤ tol, the seam ring pooled ONCE; an apex-in-extent / transversal / cap-tangent
  fixture → NULL. Green with NUMSCI on AND off (the cone fuse/cut path correctly absent off). The
  COMMON regression golden stays byte-identical.
- **Sim native-vs-OCCT.** `scripts/run-sim-native-ssi-curved-boolean.sh` +
  `tests/sim/native_ssi_curved_boolean_parity.mm` already run the `cone=cyl(coax)` pair across
  `{Fuse, Cut, Common}` and auto-detect native-vs-fall-back. After this change the cone∩cylinder
  FUSE + CUT resolve native (volume ≈ `41.626` fuse / `≈ 13.352` cut, surface area, watertight
  closed shell, valid vs `BRepAlgoAPI_{Fuse,Cut}`), raising **native-pass 13 → 15**. Do NOT regress
  the 13 existing native passes (through-drill cyl∩cyl + sphere∩sphere + Steinmetz + cone∩cylinder
  COMMON). Any pair whose self-verify does not pass stays an honest fall-back with the measured gap
  reported — no tolerance weakened.
- `openspec validate complete-cone-cyl-fuse-cut --strict` green; note the coaxial cone∩cylinder
  op-set now 3/3 native in `SSI-ROADMAP.md` / `ROADMAP.md` / `NATIVE-REWRITE.md` with the measured
  deltas. Confirm no SSI / blend / heal / import / marching / phase3 suite regresses.

## Cognitive complexity

`buildConeCylFuse` and `buildConeCylCut` are near-clones of `buildConeCylCommon` (shared
`coneCylSetup` prologue → per-station rings → outer/reversed `appendRevolvedBand` per band →
`appendAnnulusCap` / `appendDiskCap` → shell/solid), each in the systems band (~16–20, comparable
to `buildConeCylCommon` and to the Steinmetz fuse/cut builders). Factoring `coneCylSetup` out of
`buildConeCylCommon` LOWERS its complexity and removes the three-way duplication of the gate/seam
prologue. `appendAnnulusCap` is a thin ring-strip loop (~6), mirroring `appendRevolvedBand`. No
function is pushed above the documented systems band; the reused helpers (`appendRevolvedBand`,
`appendDiskCap`, `classifyPoint`) are UNCHANGED.
