# Design — complete-steinmetz-fuse-cut (SSI Stage S5-d completion)

## Context

`add-native-ssi-branched-boolean` (S5-d) shipped, in `src/native/boolean/ssi_boolean.cpp`
(OCCT-free, `CYBERCAD_HAS_NUMSCI`-gated):

- `buildSteinmetzCommon(A, B, st)` (~line 1453) — the bicylinder COMMON assembler. It orients
  the four `BranchArc` arms pole0→pole1 (`orientArc`), RESAMPLES all four onto ONE cosine-
  clustered pole-axis grid (`resampleArcByAxis`) so lunes sharing an arc reference IDENTICAL 3D
  nodes, groups the four arcs into the clean 2+2 lune pairing per cylinder (`groupLunes` by
  folded mean u, ordered by mean v), keeps each lune whose centroid classifies INSIDE the other
  solid (`classifyPoint(other, centroid) == 1`), and emits it as a strip of planar triangles
  through the shared `VertexPool` (`appendLunePatch`, `outwardSign = +1`). The two poles are
  pooled ONCE; each arc's nodes are shared by its two owning lunes → the four lunes weld into a
  watertight shell.
- `appendLunePatch(cs, which, lo, hi, uSamples, outwardSign, pool, faces)` (~line 1316) — the
  dual-arc lockstep strip builder; `outwardSign` already selects the facet normal side (radial
  out `+1` / in `−1`), so a reversed (inward) lune needs NO new code.
- `groupLunes` (~line 1399), `arcMeanU` (~line 1298), `luneUSamples` (~line 1434),
  `orientArc` (~line 1248), `resampleArcByAxis` (~line 1269) — the reusable split machinery.
- `appendDiskCap(cs, v, rim, capOutward, pool, faces)` (~line 630) — the through-drill faceted
  disc-cap fan (axis-point → rim, planar facets, ±axis normal). The Steinmetz cap helper reuses
  its pattern.
- `tryBranchedSteinmetz(A, B, adA, adB, op)` (~line 1535) — the branched dispatch: `steinmetzPreGate`
  → branch-enabled re-trace → `recogniseSteinmetzTrace` → `switch(op)` with only `Op::Common`
  wired (`buildSteinmetzCommon`); `Op::Fuse` / `Op::Cut` fall to `return {}` (→ OCCT).

Its Non-Goals defer Steinmetz fuse / cut to a follow-on. This change is that follow-on. It is a
BOUNDED completion: the SAME branched trace, a DIFFERENT lune selection + cap handling. Nothing
about `steinmetzPreGate`, the branch-enabled re-trace, `recogniseSteinmetzTrace`, the arc
orient/resample, the lune grouping, or the strip math changes.

## The geometry

Two EQUAL cylinders A (frame `fA`, radius R, extent `v ∈ [vLo_A, vHi_A]`) and B (frame `fB`,
radius R, extent `[vLo_B, vHi_B]`), axes crossing orthogonally at the origin. The intersection
self-crosses at TWO branch points (poles `pole0, pole1` at `(0, ±R, 0)` for the R=1 Z⟂X
fixture) into FOUR `BranchArc` arms. On EACH cylinder the four arcs split the wall into TWO
INSIDE lune patches (inside the other cylinder) and the surrounding OUTSIDE region; `groupLunes`
returns the two inside lunes per cylinder (the 2+2 pairing). Because the intersection lives at
`|y| ≤ R`, and the cylinders extend far beyond (`|vLo|, |vHi| ≫ R`), each cylinder's two
original disc end caps at `v = vLo` and `v = vHi` are FULL circles UNTOUCHED by the arcs.

| Region | Bounds | Normal |
|---|---|---|
| INSIDE-lune (A or B) | the part of that wall inside the OTHER cylinder | radial out (`outwardSign +1`) |
| OUTSIDE-lune (A or B) | the part of that wall OUTSIDE the other cylinder | radial out (`outwardSign +1`) |
| end cap (A or B) | full disc at `v = vLo` / `v = vHi` | ∓axis (outward) |

`groupLunes` returns the INSIDE pair per cylinder; the OUTSIDE region of a cylinder is bounded
by the SAME four-arc/two-pole seam but faces the complementary side. The op → fragment
selection:

| Op | Fragments (shared four arcs + two poles) | Volume |
|---|---|---|
| COMMON `A ∩ B` (done) | 2 INSIDE lunes of A + 2 INSIDE lunes of B (all `+1`) | `16 R³/3` |
| FUSE `A ∪ B` (new)    | 2 OUTSIDE lunes of A + 2 OUTSIDE lunes of B (all `+1`) + 4 disc caps | `V(A)+V(B) − 16 R³/3` |
| CUT `A − B` (new)     | 2 OUTSIDE lunes of A (`+1`) + 2 A-caps + 2 INSIDE lunes of B REVERSED (`−1`) | `V(A) − 16 R³/3` |

CUT intuition: A minus B keeps A's whole outer wall (the two OUTSIDE lunes) closed by A's two
end caps; where B carved into A, the channel is bounded by B's inside lunes — but those now face
INWARD (into the removed material), so they are emitted with `outwardSign = −1`. All fragments
share the four arcs + two poles, so the outer wall and the channel meet watertight along them.

FUSE intuition: A ∪ B is the outer envelope — each cylinder keeps ONLY its OUTSIDE wall (the
INSIDE walls are interior to the union and vanish), the four arcs stitch A's outside to B's
outside, and each cylinder's original end caps close the two protruding tube ends.

## Goals / Non-Goals

**Goals**
- Reuse the S5-d branched split/weld (`orientArc` + `resampleArcByAxis` + `groupLunes` +
  `appendLunePatch`) verbatim; the only new selection logic is the survival rule per op
  (INSIDE vs OUTSIDE lune) and the `outwardSign` per fragment.
- Add `appendCylinderEndCaps` — the two full-circle faceted disc caps of a cylinder (outward
  ±axis), mirroring `appendDiskCap`, disjoint from the seam.
- Add `buildSteinmetzFuse` (both cylinders' OUTSIDE lunes + all four caps) and
  `buildSteinmetzCut` (A OUTSIDE lunes + A caps + B INSIDE lunes reversed), both sharing the SAME
  oriented/resampled four arcs + two pooled poles as COMMON so they weld watertight, returning a
  `Solid` or NULL → OCCT.
- Dispatch `Op::Fuse` → `buildSteinmetzFuse`, `Op::Cut` → `buildSteinmetzCut` in
  `tryBranchedSteinmetz`; recognition + re-trace UNCHANGED.
- Keep the ENGINE self-verify UNCHANGED — the generic set-algebra guard already covers fuse
  (`va + vb − vc`) and cut (`va − vc`) via the native bicylinder COMMON (`vc = 16 R³/3`), with
  the correct per-op sign (fuse grows, cut shrinks). The `op == 2`-only Steinmetz analytic
  oracle does NOT intercept fuse/cut.

**Non-Goals (deferred — never faked)**
- Any other curved-curved family (through-drill cyl∩cyl, sphere∩sphere lens, sphere/cone∩box,
  cyl∩cone, cyl∩sphere, cone∩cone, oblique cyl∩cyl) — UNCHANGED.
- Non-equal-radius, non-orthogonal, or non-crossing cylinder pairs (`steinmetzPreGate` false),
  and any branched trace `recogniseSteinmetzTrace` does not resolve to the 2-node / 4-arm
  clean 2+2 form → NULL → OCCT.
- SHORT cylinders whose end caps would clip the intersection band (a cap plane within the
  `|y| ≤ R` seam zone) → NULL → OCCT (the disjoint-cap assumption fails; not in scope).
- Any change to `src/native/tessellate`, the `cc_*` ABI, the planar BSP-CSG, the analytic
  `curved.h`, the through-drill / sphere-lens builders, or `buildSteinmetzCommon`.
- Weakening ANY tolerance to force a pass. If FUSE or CUT cannot be built watertight with the
  correct volume, return NULL → OCCT and report the measured gap.

## Module shape

```
src/native/boolean/ssi_boolean.cpp   [CYBERCAD_HAS_NUMSCI]
  appendLunePatch(...)                 // UNCHANGED — outwardSign already selects ±radial
  groupLunes(...)                      // UNCHANGED — returns the clean 2+2 inside pairing
  appendCylinderEndCaps(cs, pool, faces)   // NEW — two full-circle disc caps, ±axis outward
  buildSteinmetzCommon(A, B, st)       // UNCHANGED — four inside lunes (+1)
  buildSteinmetzFuse(A, B, st)         // NEW — outside lunes of both (+1) + four caps
  buildSteinmetzCut(A, B, st)          // NEW — A outside (+1) + A caps + B inside reversed (−1)
  tryBranchedSteinmetz(A, B, adA, adB, op)   // dispatch: Fuse/Cut → new builders
```

No new files; reuses `orientArc`, `resampleArcByAxis`, `groupLunes`, `arcMeanU`, `luneUSamples`,
`appendLunePatch`, `appendDiskCap` (pattern), `pushPlanarTri`, `classifyPoint`, `VertexPool`,
`recogniseSteinmetzTrace`, `steinmetzPreGate`. OCCT-free.

## Shared arc/pole setup (factored from `buildSteinmetzCommon`)

All three builders need the IDENTICAL prologue: orient the four arcs pole0→pole1, resample onto
ONE cosine-clustered pole-axis grid (`nn` stations), snap grid endpoints to the exact shared
poles. Today that prologue lives inline in `buildSteinmetzCommon` (~lines 1455–1481). Factor it
into a small helper so FUSE and CUT reuse it byte-for-byte:

```cpp
struct SteinmetzArcs { std::vector<BranchArcData> arcs; math::Vec3 axisU; double axisLen; };
std::optional<SteinmetzArcs> orientResampleArcs(const CurvedSolid& refCyl, const SteinmetzTrace& st);
// axisVec = pole1 − pole0; axisLen guard; nn from arc-sagitta bound clamped [24,180];
// cosine tvals; orientArc → resampleArcByAxis per arc; snap endpoints to poles.
```

`buildSteinmetzCommon` calls it and its output is byte-identical to the current inline result
(same `nn`, same `tvals`, same nodes) → COMMON stays byte-identical, native-pass 10 not
regressed. FUSE and CUT call the SAME helper → the SAME resampled arcs → the SAME weld nodes as
COMMON, so a FUSE outside-lune and the COMMON inside-lune it abuts share IDENTICAL arc nodes.

## `appendCylinderEndCaps` — the two disc caps (NEW)

```cpp
// Emit the two full-circle disc caps of `cs` at v=vLo and v=vHi, each a fan from the axis
// point to a fresh full-turn rim ring (uSamples around 2π), planar facets, outward ±axis
// normal. The rim ring is generated ON the cylinder wall (cs.point(u, vEnd)); the cap is
// disjoint from the arc seams (|y| ≤ R ≪ |vEnd|), so it needs no VertexPool weld to the lunes
// — it closes the outer wall tube end. Returns false (→ NULL upstream) if the cap plane falls
// within the seam band (|vEnd| ≤ R + margin — a short cylinder, out of scope).
bool appendCylinderEndCaps(const CurvedSolid& cs, VertexPool& pool,
                           std::vector<topo::Shape>& faces);
```

Uses `appendDiskCap`'s fan pattern: a full-turn rim ring of `nu` samples (from the same
chord-sagitta bound as the lune width), `appendDiskCap(cs, vLo, rimLo, −axis, …)` and
`appendDiskCap(cs, vHi, rimHi, +axis, …)`. Reuses the through-drill helper directly.

## `buildSteinmetzFuse` — outside lunes of both + four caps (mirror of COMMON)

```
gate:      recognised Steinmetz trace (from tryBranchedSteinmetz); axisLen > eps
arcs:      SteinmetzArcs = orientResampleArcs(A, st)      // SAME as COMMON
survival:  per cylinder, groupLunes → the 2 inside lunes; the FUSE keeps the OUTSIDE region,
           whose bounding arcs are the SAME arcs but whose facets face the outer wall. Keep a
           lune fragment iff its centroid classifies OUTSIDE the other solid
           (classifyPoint(other, centroid) == -1); an ON verdict (== 0) → NULL → OCCT.
lunes:     appendLunePatch(cs, which, lo, hi, uSamp, /*outwardSign=*/+1.0, pool, faces)
           for the two outside lunes on A and the two on B.
caps:      appendCylinderEndCaps(A, pool, faces); appendCylinderEndCaps(B, pool, faces);
           short-cylinder cap → return {}.
assemble:  makeShell → makeSolid   (NULL if too few faces)
```

The four outside lunes stitch A's outer wall to B's outer wall along the four arcs (shared pooled
nodes); the four disc caps close the four protruding tube ends. Volume of the welded shell =
`V(A) + V(B) − 16 R³/3` (each cylinder contributes its full wall + caps minus the inside lune it
donates to the shared common region). The engine self-verify checks this against `va + vb − vc`.

**Outside-lune node source.** The OUTSIDE lune of a cylinder is bounded by the same two arcs as
its abutting INSIDE lune (`groupLunes` pairs them), so its `lo`/`hi` arc endpoints are the SAME
pooled nodes — only the interior u-fold sweeps the complementary side of the wall (folded u the
long way round, `2π − span`). `appendLunePatch` already folds u contiguous per step, so passing
the outside lune's own (u,v) track (which the arcs carry) sweeps the outside strip with the same
machinery; the seam nodes remain the pooled arc nodes → weld to COMMON-side and to the caps'
tube (the caps' rim is a full circle sharing the wall, welding at the tube end is not required as
the cap+wall meet analytically at `v=vEnd`, both emitted from `cs.point`).

## `buildSteinmetzCut` — A outside + A caps + B inside reversed (mirror of COMMON)

```
gate:      recognised Steinmetz trace; A is the minuend (CUT is order-sensitive, matches
           BRepAlgoAPI_Cut(a, b)); axisLen > eps
arcs:      SteinmetzArcs = orientResampleArcs(A, st)      // SAME as COMMON
survival:  A's two OUTSIDE lunes centroid OUTSIDE B (== -1); B's two INSIDE lunes centroid
           INSIDE A (== 1) — else (ON, or wrong side) → NULL → OCCT.
frags:     appendLunePatch(A, LuneCyl::A, lo, hi, uSamp, /*outwardSign=*/+1.0, …)  // A outside
           appendCylinderEndCaps(A, pool, faces)                                    // A caps
           appendLunePatch(B, LuneCyl::B, lo, hi, uSamp, /*outwardSign=*/-1.0, …)  // B inside, INWARD
assemble:  makeShell → makeSolid   (NULL if too few faces)
```

A's outside lunes + A's caps form A's remaining outer shell; B's inside lunes reversed
(`outwardSign = −1`) bound the carved channel (their normals point into the removed material).
All fragments share the four arcs + two poles → watertight. Volume = `V(A) − 16 R³/3`. CUT is
not symmetric: `buildSteinmetzCut` honours the operand order, matching `BRepAlgoAPI_Cut(a, b)`.
`A` = the shape passed as `a`; the branched dispatch already binds A/B to the operand order.

## Driver dispatch (`tryBranchedSteinmetz`) — extended

Pre-gate + branch-enabled re-trace + `recogniseSteinmetzTrace` UNCHANGED. Extend only the
`switch(op)`:

```cpp
if (op == Op::Common) return buildSteinmetzCommon(A, B, *st);
if (op == Op::Fuse)   return buildSteinmetzFuse(A, B, *st);
return buildSteinmetzCut(A, B, *st);   // Op::Cut
```

The branched path fires ONLY on the decline edge (`trace.nearTangentGaps > 0`) AND ONLY when
`steinmetzPreGate` matches, so every single-seam S5-a/b/c pass keeps its default trace — no
re-trace, no regression. `buildSteinmetzFuse/Cut` consume the SAME resolved 2-node / 4-arm trace
`buildSteinmetzCommon` does; any non-Steinmetz branched pair returns NULL upstream in
`recogniseSteinmetzTrace` → OCCT.

## Engine self-verify — per-op sign (fuse grows, cut shrinks), no change

`native_engine.cpp` `ssiCurvedBooleanVerified` (~line 332) returns `{}` (not applicable) unless
`op == 2` (COMMON), so it does NOT intercept Steinmetz FUSE/CUT. `booleanResultVerified`
(~line 359) therefore runs the generic set-algebra guard for fuse/cut:

```
vc = watertightVolume(boolean_solid(a, b, Op::Common));   // native buildSteinmetzCommon = 16R³/3
switch(op) { case 0: expected = va + vb − vc;   // fuse — GROWS: Vr > max(VA,VB)
             case 1: expected = va − vc;         // cut  — SHRINKS: Vr < VA
             case 2: expected = vc; }            // common (analytic oracle path)
accept iff |Vr − expected| <= max(1e-6·expected, 1e-9) AND watertight
```

For the R=1, L=6 fixture: `V(A) = V(B) = π·1²·6 = 6π ≈ 18.850`, `vc = 16/3 ≈ 5.333`, so
`expected(fuse) = 32.366` and `expected(cut) = 13.516` — EXACTLY the OCCT `volO` the sim harness
already reports for these sub-cases. No engine edit is needed; the correct per-op sign is already
wired. A mis-selected / mis-oriented / non-watertight candidate fails the check and is
DISCARDED → OCCT.

## Verification plan

- **Host (no OCCT), analytic inclusion-exclusion dual oracle.** `V(A ∩ B) = 16 R³/3` (exact
  bicylinder common), `V(cyl) = π R² L`; `FUSE = π R²(L_A + L_B) − 16 R³/3`,
  `CUT(A,B) = π R² L_A − 16 R³/3`. The host test asserts, for the equal-R orthogonal Steinmetz
  pair: watertight shell (`boundaryEdgeCount == 0`, every edge shared by exactly two faces),
  enclosed volume matches the closed form within the deflection band, every arc node on BOTH
  cylinders ≤ tol, the two poles pooled ONCE; a short-cylinder / non-Steinmetz fixture → NULL.
  Green with NUMSCI on AND off (the Steinmetz path correctly absent off). COMMON regression
  golden unchanged (byte-identical).
- **Sim native-vs-OCCT.** `scripts/run-sim-native-ssi-curved-boolean.sh` +
  `tests/sim/native_ssi_curved_boolean_parity.mm` already run the `cyl=cyl(steinmetz)` pair
  across `{Fuse, Cut, Common}` and auto-detect native-vs-fall-back (runPair, `nativePass`
  auto-flag). After this change the Steinmetz FUSE + CUT resolve native (volume + area +
  watertight + valid vs `BRepAlgoAPI_{Fuse,Cut}`), raising **native-pass 10 → 12**. Do NOT
  regress the 10 existing native passes (incl. Steinmetz COMMON). Any pair whose self-verify
  does not pass stays an honest fall-back with the measured gap reported — no tolerance weakened.
- `openspec validate complete-steinmetz-fuse-cut --strict` green; note Steinmetz now 3/3 native
  in `SSI-ROADMAP.md` / `ROADMAP.md` / `NATIVE-REWRITE.md` with the measured deltas. Confirm no
  SSI / blend / heal / import / marching / phase3 suite regresses.

## Cognitive complexity

`buildSteinmetzFuse` and `buildSteinmetzCut` are near-clones of `buildSteinmetzCommon` (shared
`orientResampleArcs` prologue → per-cylinder `groupLunes` → survival classify → `appendLunePatch`
per fragment → optional caps → shell/solid), each in the systems band (~14–18, comparable to
`buildSteinmetzCommon`). Factoring `orientResampleArcs` out of `buildSteinmetzCommon` LOWERS its
complexity and removes the three-way duplication. `appendCylinderEndCaps` is a thin wrapper over
`appendDiskCap` (~6). `appendLunePatch` (flagged ~18, systems band) is UNCHANGED. No function is
pushed above the documented systems band; the reused `appendLunePatch` stays flagged, not split
(splitting the shared strip loop would duplicate the geometry, the very thing the reuse avoids).
