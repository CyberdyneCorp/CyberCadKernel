# Design — complete-sphere-sphere-fuse-cut (SSI Stage S5-c completion)

## Context

`add-native-ssi-curved-boolean-wider` (S5-b + S5-c) shipped, in
`src/native/boolean/ssi_boolean.cpp` (OCCT-free, `CYBERCAD_HAS_NUMSCI`-gated):

- `buildLensCommon(A, B, seams)` (~line 961) — the sphere∩sphere COMMON assembler: one
  closed seam, both operands Sphere; `decimateSeam(seam, seamNodeTarget(seam))` produces a
  shared node subset, then TWO `appendSphereCap` calls (inner cap of A, inner cap of B)
  weld on the shared pooled seam → the lens.
- `appendSphereCap(sph, otherCentre, seam, rings, pool, faces)` (~line 880) — builds the
  INNER cap: apex = surface point of `sph` NEAREST `otherCentre`
  (`centre + R·unit(centre→other)`); interior ring points placed by SLERPing the unit
  radial from `apexDir` to each seam node's radial; every facet a PLANAR triangle oriented
  with the sphere OUTWARD radial normal.
- `ssi_boolean_solid(a, b, op)` (~line 1437) — recognise + trace + gate, then a
  `switch(op)`: `Op::Common` tries through-drill `buildCommon`, then `buildLensCommon`;
  `Op::Fuse` → `buildFuse`; `Op::Cut` → `buildCut` (the through-drill S5-b builders, which
  return NULL for the single-seam sphere pair → OCCT).

Its Non-Goals defer sphere∩sphere fuse / cut to a follow-on. This change is that
follow-on. It is a BOUNDED completion: the SAME single-seam geometry, a DIFFERENT cap
selection. Nothing about recognition, tracing, the seam, the decimation, or the ring/facet
math changes.

## The geometry

Two overlapping spheres A (centre `cA`, radius `RA`) and B (centre `cB`, radius `RB`)
trace as ONE closed seam circle **C**. C splits each sphere into two spherical caps:

| Cap | Apex | Bounds |
|---|---|---|
| INNER-A | `cA + RA·unit(cA→cB)` (nearest B) | the part of A INSIDE B |
| OUTER-A | `cA − RA·unit(cA→cB)` (far pole)   | the part of A OUTSIDE B |
| INNER-B | `cB + RB·unit(cB→cA)` (nearest A) | the part of B INSIDE A |
| OUTER-B | `cB − RB·unit(cB→cA)` (far pole)   | the part of B OUTSIDE A |

Note `unit(cB→cA) = −unit(cA→cB)`, so INNER-B's apex is `cB − RB·unit(cA→cB)` — i.e. the
existing `appendSphereCap(B, cA, …)` (otherCentre = cA) already builds INNER-B. OUTER-B's
apex is `cB + RB·unit(cA→cB)`. The op → cap selection:

| Op | Caps (shared seam C) | Volume |
|---|---|---|
| COMMON `A ∩ B` (done) | INNER-A (outward) + INNER-B (outward) | `V_cap(A) + V_cap(B) = lens` |
| FUSE `A ∪ B` (new)    | OUTER-A (outward) + OUTER-B (outward) | `V(A) + V(B) − lens` |
| CUT `A − B` (new)     | OUTER-A (outward) + INNER-B **reversed** (inward) | `V(A) − lens` |

CUT intuition: A minus B keeps all of A's outer shell (OUTER-A) and, where B scooped into
A, the cavity is bounded by B's inner cap — but now that surface faces INWARD (into the
removed material), so INNER-B is emitted with its normal FLIPPED. The two caps share C, so
the outer shell and the cavity wall meet watertight along the seam.

## Goals / Non-Goals

**Goals**
- Generalise `appendSphereCap` to emit inner OR outer caps, outward OR reversed, with no
  code duplication and the COMMON path byte-identical.
- Add `buildLensFuse` (two OUTER caps, outward) and `buildLensCut` (OUTER-A outward +
  INNER-B reversed), both sharing the SAME decimated seam as COMMON so they weld
  watertight, returning a `Solid` or NULL → OCCT.
- Dispatch `Op::Fuse` → `buildLensFuse`, `Op::Cut` → `buildLensCut` for the recognised
  single-seam sphere∩sphere lens; keep the through-drill builders tried first.
- Keep the ENGINE self-verify UNCHANGED — the existing generic set-algebra guard already
  covers fuse (`va + vb − vc`) and cut (`va − vc`) via the native lens COMMON (`vc`), with
  the correct per-op sign (fuse grows, cut shrinks).

**Non-Goals (deferred — never faked)**
- Any other curved-curved family (cyl∩cyl through-drill, Steinmetz, sphere/cone∩box,
  cyl∩cone, cyl∩sphere, cone∩cone, oblique cyl∩cyl) — UNCHANGED.
- Tangent / coincident / near-containment spheres (`nearTangentGaps > 0`, or an apex
  robustly ON/inside the other where the op's rule needs it outside) → NULL → OCCT.
- Any change to `src/native/tessellate`, the `cc_*` ABI, the planar BSP-CSG, the analytic
  `curved.h`, the through-drill `buildCommon/Fuse/Cut`, or the S5-c `buildLensCommon`.
- Weakening ANY tolerance to force a pass. If FUSE or CUT cannot be built watertight with
  the correct volume, return NULL → OCCT and report the measured gap.

## Module shape

```
src/native/boolean/ssi_boolean.cpp   [CYBERCAD_HAS_NUMSCI]
  appendSphereCap(sph, otherCentre, seam, rings, pool, faces,
                  bool outer=false, bool reversed=false)   // GENERALISED (inner default)
  buildLensCommon(A, B, seams)   // UNCHANGED — two inner, outward caps
  buildLensFuse(A, B, seams)     // NEW — two OUTER, outward caps
  buildLensCut(A, B, seams)      // NEW — OUTER-A outward + INNER-B reversed
  ssi_boolean_solid(a, b, op)    // dispatch: single-seam sphere pair → lens builders
```

No new files; reuses `decimateSeam`, `seamNodeTarget`, `slerpDir`, `pushPlanarTri`,
`classifyPoint`, `VertexPool`, `recogniseCurvedSolid`. OCCT-free.

## `appendSphereCap` generalisation

Add two defaulted bools so existing calls are unaffected:

```cpp
void appendSphereCap(const CurvedSolid& sph, const math::Point3& otherCentre,
                     const Seam& seam, int rings, VertexPool& pool,
                     std::vector<topo::Shape>& faces,
                     bool outer = false, bool reversed = false);
```

- **Apex direction.** Today `apexDir = unit(otherCentre − centre)` (inner apex). Generalise
  to `apexDir = outer ? −unit(otherCentre − centre) : +unit(otherCentre − centre)`. The far
  pole is diametrically opposite the near apex; SLERP from the outer apex to each seam
  node's radial sweeps the OUTER cap (the polar half-angle is `π − θ_inner`, so
  `ringsFor` — which uses the actual apex/seam angle — sizes it correctly for either cap
  with no special case).
- **Facet orientation.** Today the outward reference is `ctr − centre` (radial outward).
  Generalise to `reversed ? (centre − ctr) : (ctr − centre)`. `pushPlanarTri` orients the
  facet to match its `refOutward` argument, so passing the inward reference flips the whole
  cap's normal — exactly what CUT's cavity-bounding INNER-B needs. No winding logic
  changes; only the reference sign.

The ring/slerp loop, the seam-node outer ring (`r == rings` → `seam.pts[k]`), and the fan
are byte-identical. `buildLensCommon` calls with the defaults (`outer=false,
reversed=false`) → unchanged output.

## `buildLensFuse` — two OUTER caps (mirror of `buildLensCommon`)

```
gate:      seams.size()==1, seam.closed, ≥4 pts, both Sphere, |cA−cB|>eps
survival:  each far pole classifies OUTSIDE the other solid (transversal lens →
           the far pole of A is outside B and vice-versa). classifyPoint(other, farPole)
           != 1 (inside) required; a far pole INSIDE the other (containment) → NULL → OCCT.
seam:      capSeam = decimateSeam(seam, seamNodeTarget(seam))   // SAME as COMMON → weld
caps:      appendSphereCap(A, cB, capSeam, ringsFor(A, outerApexA), pool, faces,
                           /*outer=*/true, /*reversed=*/false)
           appendSphereCap(B, cA, capSeam, ringsFor(B, outerApexB), pool, faces,
                           /*outer=*/true, /*reversed=*/false)
assemble:  makeShell → makeSolid   (NULL if <4 faces)
```

`outerApexA = cA − RA·unit(cA→cB)`, `outerApexB = cB + RB·unit(cA→cB)`. `ringsFor` (the
existing lambda) takes the true apex→seam polar half-angle, so it sizes the (larger) outer
cap correctly. Volume of the welded shell = `V(A) + V(B) − lens` (each outer cap plus the
seam disc equals a full sphere minus the inner cap; the two outer caps sharing C give the
peanut). The engine self-verify checks this against `va + vb − vc`.

## `buildLensCut` — OUTER-A + reversed INNER-B (mirror of `buildLensCommon`)

```
gate:      same single-seam sphere gate
minuend:   A is the minuend (CUT is A − B, order-sensitive)
seam:      capSeam = decimateSeam(seam, seamNodeTarget(seam))   // SAME as COMMON
survival:  outer apex of A OUTSIDE B (as in fuse) AND inner apex of B INSIDE A
           (as in COMMON) — else NULL → OCCT (tangent/degenerate/containment)
caps:      appendSphereCap(A, cB, capSeam, ringsFor(A, outerApexA), pool, faces,
                           /*outer=*/true,  /*reversed=*/false)   // OUTER-A, outward
           appendSphereCap(B, cA, capSeam, ringsFor(B, innerApexB), pool, faces,
                           /*outer=*/false, /*reversed=*/true)    // INNER-B, INWARD
assemble:  makeShell → makeSolid   (NULL if <4 faces)
```

INNER-B reversed bounds the scooped cavity (its normal points into the removed material);
OUTER-A is the remaining outer shell of A. They share C → watertight. Volume of the shell
= `V(A) − lens` (A's outer shell closed by the inward cavity cap that removes the lens).
The engine self-verify checks this against `va − vc`. CUT is not symmetric: `buildLensCut`
honours the operand order (A = the shape passed as `a`), matching the through-drill
`buildCut` convention and `BRepAlgoAPI_Cut(a, b)`.

## Driver dispatch (`ssi_boolean_solid`) — extended

Gate + trace + seam construction UNCHANGED. Extend the `switch(op)`:

```cpp
case Op::Fuse: {
  const topo::Shape drill = buildFuse(*csA, *csB, seams);   // through-drill (NULL for 1 seam)
  if (!drill.isNull()) return drill;
  return buildLensFuse(*csA, *csB, seams);                  // single-seam sphere lens
}
case Op::Cut: {
  const topo::Shape drill = buildCut(*csA, *csB, seams);    // through-drill (NULL for 1 seam)
  if (!drill.isNull()) return drill;
  return buildLensCut(*csA, *csB, seams);                   // single-seam sphere lens
}
```

This mirrors the existing `Op::Common` dispatch (through-drill `buildCommon` first, then
`buildLensCommon`). The through-drill `buildFuse/buildCut` resolve their two-seam drill
roles and return NULL on the single-seam sphere pair, so the lens builders engage exactly
there — the S5-b through-drill fuse/cut passes are untouched. `buildLensFuse/Cut` gate on
`seams.size()==1` + both Sphere, so they decline on any non-sphere-lens input that reached
them.

## Engine self-verify — per-op sign (fuse grows, cut shrinks), no change

`native_engine.cpp` `booleanResultVerified` (~line 341) already runs, for the generic
(non-Steinmetz) case:

```
vc = watertightVolume(boolean_solid(a, b, Op::Common));   // native lens COMMON here
switch(op) { case 0: expected = va + vb − vc;   // fuse — GROWS: Vr > max(VA,VB)
             case 1: expected = va − vc;         // cut  — SHRINKS: Vr < VA
             case 2: expected = vc; }            // common
accept iff |Vr − expected| <= max(1e-6·expected, 1e-9) AND watertight
```

For the sphere lens, the native COMMON path is `buildLensCommon`, so `vc = lens`,
`expected(fuse) = V(A)+V(B)−lens` and `expected(cut) = V(A)−lens` — exactly the analytic
values. The `ssiCurvedBooleanVerified` Steinmetz oracle applies ONLY to `op == 2` equal-R
perpendicular cylinders, so it does not intercept the sphere fuse/cut. No engine edit is
needed; the correct per-op sign is already wired. A mis-welded or wrong-signed candidate
fails the check and is DISCARDED → OCCT.

## Verification plan

- **Host (no OCCT), analytic spherical-cap dual oracle.** `V_cap = π h² (3R − h) / 3`;
  for spheres at centre distance `d`, `h_A = RA − x`, `h_B = RB − (d − x)` where
  `x = (d² + RA² − RB²) / (2d)`; `lens = V_cap(A) + V_cap(B)`;
  `FUSE = 4/3·π(RA³+RB³) − lens`; `CUT = 4/3·π·RA³ − lens`. The host test asserts, for
  equal- and unequal-radius overlapping pairs: watertight shell (`boundaryEdgeCount==0`,
  every edge shared by exactly two faces), enclosed volume matches the closed form within
  the deflection band, seam nodes on both spheres ≤ tol; tangent/degenerate fixtures →
  NULL. Green with NUMSCI on AND off (the sphere path correctly absent off).
- **Sim native-vs-OCCT.** `scripts/run-sim-native-ssi-curved-boolean.sh` +
  `tests/sim/native_ssi_curved_boolean_parity.mm` already run the two sphere-lens pairs
  across `{Fuse, Cut, Common}` and auto-detect native-vs-fall-back. After this change the
  sphere FUSE + CUT resolve native (volume + area + watertight + valid vs
  `BRepAlgoAPI_{Fuse,Cut}`), raising **native-pass 6 → 8**. Any pair whose self-verify
  does not pass stays an honest fall-back with the measured gap reported — no tolerance
  weakened.

## Cognitive complexity

`buildLensFuse` and `buildLensCut` are near-clones of `buildLensCommon` (gate → survival
classify → decimate → two `appendSphereCap` → shell/solid), each in the systems band
(~10–14, comparable to `buildLensCommon`'s flagged ~13). The generalised `appendSphereCap`
adds two branch selectors (apex sign, reference sign) to an already-flagged ~13 function —
stays in the documented systems band for radial-ring cap generation; flagged, not split
(splitting the shared ring loop would duplicate the geometry, the very thing the
generalisation avoids).
