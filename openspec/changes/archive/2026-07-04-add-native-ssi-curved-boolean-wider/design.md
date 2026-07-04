# Design — add-native-ssi-curved-boolean-wider (SSI Stage S5-b + S5-c)

## Context

The archived `add-native-ssi-curved-boolean` (S5-a) shipped the SSI-curve-driven
split→classify→select→weld pipeline in `src/native/boolean/ssi_boolean.{h,cpp}`
(OCCT-free, `CYBERCAD_HAS_NUMSCI`-gated). It produces ONE verified native curved boolean:
the **through-drill cylinder∩cylinder COMMON** (a thin cylinder drilled clean through a
fat one; two disjoint closed rim seams; ΔV = 8.1e-04 vs OCCT). Its own honest-scope note
lists three remainders that do NOT depend on S4:

1. **Fuse / Cut** on that same through-drill pair DECLINE — `ssi_boolean_solid` returns
   NULL for `Op::Fuse` / `Op::Cut` (the outside-fragment + re-trimmed-cap weld was not yet
   robust).
2. **sphere∩sphere COMMON** DECLINES — `recogniseCurvedSolid` folds a sphere solid and S3
   traces the transversal seam (ONE closed circle), but `buildCommon` requires `seams.size()
   == 2` and a full-circle tube operand, so the one-seam lens hits the guard and returns NULL.
3. (sphere×box / cone×box have a PLANAR box operand — not a curved-curved pair; those stay
   deferred and are out of THIS change's scope.)

This change closes (1) and (2). Both are **assembler** gaps: recognition (`recogniseCurvedSolid`)
and tracing (`ssi::trace_intersection`) already work for both. The method stays clean-room;
OCCT `BRepAlgoAPI` is the verification ORACLE only.

## Goals / Non-Goals

**Goals**
- (S5-b) Compute native `fuse` (`A ∪ B`) and `cut` (`A − B`) for the through-drill
  cylinder∩cylinder topology `buildCommon` already recognises, reusing the SAME two rim
  seams, the SAME `VertexPool` weld, and the SAME planar-facet discipline, selecting the
  complementary fragments per the op set algebra. Return a watertight curved-faced `Solid`
  or NULL → OCCT.
- (S5-c) Compute native `common` (`A ∩ B`) for the transversal sphere∩sphere lens via a
  NEW single-seam / two-cap `buildLensCommon` path, welding two spherical caps along the
  ONE seam circle with the radial-ring planar-facet discipline. Return a watertight
  `Solid` or NULL → OCCT.
- Keep the ENGINE self-verify unchanged: the EXISTING generic set-algebra guard covers
  S5-b fuse/cut (inclusion–exclusion over the native COMMON) and S5-c common (the
  common-volume check), with closed-form host oracles.
- Report native-vs-OCCT parity for the new cases; call out what still declines.

**Non-Goals (deferred — never faked here)**
- **Near-tangent / coincident** curved pairs (`nearTangentGaps > 0`, tangent spheres,
  equal-radius orthogonal cyl∩cyl Steinmetz) → S4 + OCCT. Unchanged from S5-a.
- **sphere∩sphere fuse / cut** — S5-c ships COMMON only (the lens has the clean
  closed-form + the simplest two-cap weld); sphere fuse/cut (outer-cap union with the
  re-trimmed remainder of each sphere) is a follow-on. DECLINE → OCCT.
- **Other curved-curved families** — cyl∩cone, cyl∩sphere, cone∩cone, and any pair whose
  through-drill / single-lens topology `buildCommon` / `buildLensCommon` do not recognise
  → DECLINE → OCCT.
- **Oblique / multi-tube cyl∩cyl** piercings (seams not two clean full-circle rims) →
  DECLINE (S5-a's `buildCommon` gate already rejects these; S5-b inherits the gate).
- Any change to `src/native/tessellate` (the S5-a lesson) or to the `cc_*` ABI, the
  planar BSP-CSG, the analytic `curved.h`, or the S5-a `buildCommon` path itself.

## Module shape

```
src/native/boolean/ssi_boolean.cpp   [CYBERCAD_HAS_NUMSCI]
  buildCommon(A, B, seams)      // S5-a through-drill COMMON — UNCHANGED
  buildCut(A, B, seams)         // NEW S5-b: A − B for the through-drill topology
  buildFuse(A, B, seams)        // NEW S5-b: A ∪ B for the through-drill topology
  buildLensCommon(A, B, seam)   // NEW S5-c: sphere∩sphere COMMON, one seam, two caps
  ssi_boolean_solid(a, b, op)   // dispatch: pick the topology by seam count + kinds,
                                //   then the op-appropriate builder; else NULL → OCCT
```

Reuses (no new files needed): `VertexPool` (`assemble.h`), `appendTubeBand`,
`appendMouthCap`, `tubeTriFace`, `seamWire`, `classifyPoint`, `recogniseCurvedSolid`,
`isFullCircle`, `meanU/meanV/uSpan`, `wallSurface` — all already in
`ssi_boolean.{h,cpp}`. OCCT-free.

## Pipeline

### Dispatch (`ssi_boolean_solid`) — extended, still short + linear
Gate + trace are UNCHANGED from S5-a (recognise both operands as elementary curved
solids; obtain the `TraceSet`; require `nearTangentGaps == 0` and every consumed WLine
`Closed` / `BoundaryExit`; `≥ 1` seam). Then dispatch on **seam count + operand kinds +
op** rather than the S5-a `switch(op)` that blanket-declined fuse/cut:

| Seams | Kinds | Op | Builder |
|---|---|---|---|
| 2, through-drill | cyl / cyl | Common | `buildCommon` (S5-a, unchanged) |
| 2, through-drill | cyl / cyl | Cut | `buildCut` (S5-b) |
| 2, through-drill | cyl / cyl | Fuse | `buildFuse` (S5-b) |
| 1, closed | sphere / sphere | Common | `buildLensCommon` (S5-c) |
| anything else | — | — | NULL → OCCT |

Any builder returning NULL (topology not recognised, ON-band sample, degenerate weld)
falls through to NULL → OCCT. The dispatcher stays a short table; the geometry lives in
the flagged builders.

### S5-b — Fuse / Cut set algebra (the fragment-selection rule per op)

The through-drill COMMON boundary (S5-a) is: the piercing **tube band** (the thin wall
between the two rim seams, INSIDE the fat solid) + **two drill-mouth caps** (the fat-wall
patches bounded by each seam). The fragment inventory for the pair, over the SAME two
seams, is:

- **Tube band, inside part** `T_in` — the thin wall between the rims (already built by
  `appendTubeBand`); it is the piece of the thin wall INSIDE the fat solid.
- **Tube band, outside parts** `T_out` — the two remaining stretches of the thin wall
  (below the lower rim and above the upper rim) OUTSIDE the fat solid, each capped by the
  thin cylinder's own end disc.
- **Fat wall, outside part** `F_out` — the fat cylinder wall minus the two drill mouths
  (the fat wall OUTSIDE the thin tube).
- **Mouth caps** `M` — the two fat-wall patches bounded by the seams (built by
  `appendMouthCap`); each is the piece of the fat wall INSIDE the thin tube.
- **Fat end caps** `F_cap` and **thin end caps** `T_cap` — the operands' original planar
  discs.

The SAME set algebra as planar `booleanPolygons` maps op → surviving fragments:

- **Cut `A − B`** with `A = fat`, `B = thin` (drill the thin tunnel out of the fat body):
  keep **fat OUTSIDE thin** + **thin INSIDE fat, reversed**. Concretely:
  `F_out` (fat wall, unchanged) + `F_cap` re-trimmed to exclude the drilled disc region +
  `T_in` **reversed** (the tunnel wall, now an inward-facing boundary of the result) +
  the two mouth caps `M` **dropped** (they were fat material now removed). The shared seam
  welds the reversed tunnel wall `T_in` to the re-trimmed fat caps / fat wall along the
  two rims. Selection is by `classifyPoint`: keep a fat-wall fragment iff its interior
  sample is OUTSIDE the thin tube; take `T_in` (already the inside piece) and reverse its
  facet winding.
- **Fuse `A ∪ B`** (weld the two cylinders into one body):
  keep **fat OUTSIDE thin** + **thin OUTSIDE fat** + both operands' end caps, and DROP the
  two mouth caps `M` and the tube band `T_in` (both now interior to the union). Concretely
  `F_out` + `T_out` (the two outside thin-wall stretches with their `T_cap` discs) +
  `F_cap`, welded along the two shared rim seams where the thin wall exits the fat wall.
  Selection by `classifyPoint`: keep each fragment iff its interior sample is OUTSIDE the
  OTHER solid; the seam is the shared boundary where a kept thin-wall stretch meets the
  kept fat wall.

Both builders emit every seam-adjacent face as **planar-triangle facets** (the S5-a
watertight discipline): the re-trimmed fat caps and the reversed/kept walls that share a
rim seam with a differently tessellated neighbour take their rim from the EXACT traced
seam nodes in the shared `VertexPool`, so the shell welds. A face that shares NO seam
(e.g. an untouched fat end cap far from the drill) may keep its analytic surface. Any
fragment whose interior sample lands in the ON-band, or any weld that cannot close, aborts
→ NULL → OCCT.

**Volume identities (host ground truth, no OCCT):**
`vol(CUT) = vol(fat) − vol(COMMON)` and `vol(FUSE) = vol(fat) + vol(thin) − vol(COMMON)`,
where `vol(COMMON)` is the S5-a-pinned through-drill value (3.11685, ΔV 8.1e-4). These are
exactly the inclusion–exclusion relations the ENGINE's generic guard checks.

### S5-c — sphere∩sphere lens COMMON (single-seam / two-cap)

Two spheres A (centre `cA`, radius `rA`) and B (centre `cB`, radius `rB`) at centre
distance `d`, with `|rA − rB| < d < rA + rB` (transversal), intersect in ONE circle. S3
returns ONE `Closed` WLine tracing that circle, each node carrying `(u1,v1)` on A and
`(u2,v2)` on B and a SHARED 3D point. The COMMON (lens) boundary is exactly **two
spherical caps**:

- **Cap of A inside B** — the spherical patch of A on the side of the seam circle toward
  `cB` (the part of A's surface with `‖P − cB‖ < rB`).
- **Cap of B inside A** — the spherical patch of B on the side toward `cA`.

They meet along the ONE seam circle. `buildLensCommon`:

1. **Gate.** Require exactly one seam, `Closed`, both operands `CurvedKind::Sphere`. The
   seam's `(u,v)` track must NOT be a full sphere (a degenerate coincident case) — a
   proper sub-latitude band; else NULL.
2. **Cap pole.** For sphere A, the cap pole is A's surface point nearest `cB` (the axis
   `cA → cB` pierces A's surface there); symmetrically for B toward `cA`. Evaluate the
   pole strictly on the analytic sphere (`recogniseCurvedSolid` gives the frame + radius).
3. **Survival check (COMMON rule).** The A-cap survives iff its pole sample is INSIDE
   sphere B (`classifyPoint(csB, poleA) == inside`); the B-cap iff its pole is INSIDE A.
   Both must hold for a lens (each cap is the piece of one sphere inside the other). A pole
   in the ON-band (spheres tangent) → abort → NULL → OCCT.
4. **Cap weld (planar facets).** Emit each cap with the SAME radial-ring discipline as
   `appendMouthCap`: fan from the cap POLE out through concentric rings to the shared seam
   nodes, every ring node evaluated ON the analytic sphere, the OUTER ring being the EXACT
   traced seam nodes drawn from the shared `VertexPool`. Ring count from the seam's
   angular span / a sagitta step (the existing `kCapSagitta` bound), so the cap follows
   the sphere curvature to O(1/rings²) while staying planar-per-facet. Because BOTH caps'
   outer rings are the SAME pooled seam vertices, they weld watertight along the single
   seam. Orient each cap's facet normals OUTWARD (away from the lens interior = along the
   sphere's own outward radial at the pole).
5. **Assemble.** `makeShell(caps) → makeSolid`. Return the candidate; the ENGINE
   self-verifies.

**Volume ground truth (host, no OCCT):**
`V_lens = π (rA + rB − d)² (d² + 2 d·rB − 3 rB² + 2 d·rA + 6 rA·rB − 3 rA²) / (12 d)`.
Equal radii `r`, distance `d`: each cap height `h = r − d/2`, cap volume `π h² (3r − h)/3`,
lens `= 2 ×` that. Asserted within the curved-face deflection band.

## Curved point-in-solid classification (reused, unchanged)

Both slices reuse S5-a's `classifyPoint(CurvedSolid, P, tol)` unchanged: signed distance
to the curved wall (cylinder radial slab / sphere `‖P−c‖−r` / cone apex-angle) combined
with the cap half-spaces; `+1` inside, `−1` outside, `0` ON. An ON verdict aborts the
native path (coincident / tangent → OCCT), never a guessed side. S5-b uses it to select
outside-vs-inside wall fragments; S5-c uses it for the two cap-pole survival checks.

## Self-verify → OCCT fallback (ENGINE — NO new oracle)

`ssi_boolean_solid` returns the candidate `Solid` or NULL; the ENGINE decides
shippability, exactly as for S5-a. **No engine code change is required:**

- **S5-b fuse/cut** are caught by the EXISTING generic guard
  (`native_engine.cpp booleanResultVerified`, the branch after the Steinmetz/analytic
  checks): it measures the native COMMON volume `vc`, forms `expected = va + vb − vc`
  (fuse) / `va − vc` (cut), and DISCARDS a candidate whose watertight volume misses
  `expected` beyond tolerance → OCCT. The through-drill native COMMON (S5-a) is the `vc`
  it uses — an independent already-verified path.
- **S5-c common** is caught by the SAME guard's common branch (`expected = vc`), where the
  native COMMON for a sphere pair is the `buildLensCommon` result itself; the host test
  additionally pins it to the closed-form lens volume as the analytic oracle.

The `ssiCurvedBooleanVerified` Steinmetz special oracle (op == 2, equal-radius
perpendicular cylinders) is UNTOUCHED and does not fire for these cases (through-drill is
unequal radii; sphere∩sphere is not cylinders). The guard lives in the engine next to the
OCCT fallback; the library stays OCCT-free.

## Transversal-vs-deferred scope (honest)

| Configuration | behaviour |
|---|---|
| Through-drill cyl∩cyl, `nearTangentGaps == 0`, **Common** | `buildCommon` (S5-a) → native `Solid` |
| Through-drill cyl∩cyl, `nearTangentGaps == 0`, **Cut** | `buildCut` (S5-b) → native `Solid` (or discard → OCCT) |
| Through-drill cyl∩cyl, `nearTangentGaps == 0`, **Fuse** | `buildFuse` (S5-b) → native `Solid` (or discard → OCCT) |
| Transversal sphere∩sphere, ONE closed seam, **Common** | `buildLensCommon` (S5-c) → native `Solid` (or discard → OCCT) |
| Sphere∩sphere **Fuse / Cut** | DECLINE → NULL → OCCT (S5-c is COMMON only) |
| Equal-radius orthogonal cyl∩cyl (Steinmetz) | `nearTangentGaps > 0` → NULL → OCCT (S4) |
| Tangent / coincident spheres, or lens pole ON the other sphere | ON-band / `nearTangentGaps > 0` → NULL → OCCT (S4) |
| Oblique / multi-tube cyl∩cyl (not two clean full-circle rims) | `buildCommon`/`buildCut`/`buildFuse` gate rejects → NULL → OCCT |
| cyl∩cone, cyl∩sphere, cone∩cone, sphere∩box, freeform | DECLINE → NULL → OCCT |
| Self-verify (watertight or volume) fails on a candidate | ENGINE DISCARDS → OCCT, reported |

## Verification model (two gates)

- **Host (no OCCT), analytic oracles.** Extend
  `tests/native/test_native_ssi_curved_boolean.cpp`:
  - **S5-b Cut/Fuse:** on the S5-a through-drill fixture (fat r=2 Z-axis, thin r=0.5
    X-axis), `ssi_boolean_solid(fat, thin, Cut)` and `(…, Fuse)` are now non-NULL,
    watertight (`watertightMeshVolume > 0`), and their enclosed volumes match
    `vol(fat) − vol(COMMON)` and `vol(fat) + vol(thin) − vol(COMMON)` within the 1%
    deflection band (`vol(COMMON)` = the S5-a-pinned 3.11685). The S5-a monotone
    invariants still hold.
  - **S5-c sphere∩sphere Common:** two spheres (e.g. equal `r=1`, `d=1`; and an unequal
    `rA=1, rB=0.7, d=1.2` case) → `ssi_boolean_solid(sA, sB, Common)` non-NULL, watertight,
    volume equal to the closed-form lens within the band, every seam node on both surfaces
    ≤ tol. A **tangent** sphere pair (`d = rA + rB`) → NULL (deferred, no native solid).
  - Full CTest green NUMSCI ON and OFF (the new S5-b/S5-c assertions correctly absent with
    NUMSCI off, like the S5-a tests). No OCCT linked; no tolerance weakened.
- **Sim native-vs-OCCT — BRepAlgoAPI parity.** Extend
  `tests/sim/native_ssi_curved_boolean_parity.mm` +
  `scripts/run-sim-native-ssi-curved-boolean.sh` with: the through-drill **Fuse** and
  **Cut** vs `BRepAlgoAPI_{Fuse,Cut}`, and the sphere∩sphere **Common** vs
  `BRepAlgoAPI_Common` (built as OCCT `BRepPrimAPI_MakeSphere`). Compare volume, surface
  area, watertight closed shell, `BRepCheck` validity; report per-pair deltas and the
  count still deferred to OCCT (the S4 / out-of-family seam). Parity is a REPORTED figure;
  whatever does not pass falls back to OCCT and is reported with the measured gap.

## Decisions

- **Reuse `buildCommon`'s topology + weld helpers for fuse/cut, don't re-derive.** The two
  rim seams, `VertexPool`, `appendTubeBand`, `appendMouthCap`, and `tubeTriFace` already
  produce a watertight through-drill shell; S5-b only changes WHICH fragments survive and
  their winding (the set-algebra selection), keeping the planar-facet weld identical. This
  is why fuse/cut are an assembler slice, not new tracing.
- **Ship sphere∩sphere COMMON only in S5-c.** The lens is the simplest curved-curved
  single-seam case (two caps, one closed-form volume, the exact `appendMouthCap` weld);
  sphere fuse/cut need the re-trimmed remainder of each whole sphere — a larger weld —
  deferred honestly rather than shipped unverified.
- **No new engine oracle.** The generic set-algebra guard already computes fuse/cut via
  inclusion–exclusion over the native COMMON and checks the common volume directly, so
  S5-b/S5-c are guarded by shipped code; adding a special oracle would be redundant and
  risk masking a real weld defect.
- **Dispatch on topology (seam count + kinds), not just op.** The S5-a `switch(op)` was a
  temporary blanket decline; routing on seam count + operand kinds lets `buildCommon`
  (two-seam) and `buildLensCommon` (one-seam) coexist and keeps each builder's gate honest.
- **Planar facets on every seam-adjacent NEW/re-trimmed face.** The S5-a watertight lesson
  applies verbatim: any face sharing the seam with a differently tessellated neighbour
  emits planar-triangle facets through the shared pooled seam nodes; the tessellator is
  untouched.

## Risks / Trade-offs

- **Cut/Fuse cap re-trim hairline gap.** The re-trimmed fat caps (cut) or the dropped
  mouth region (fuse) can leave a hairline seam mismatch on a high-curvature mouth.
  Mitigation: emit the re-trimmed faces as planar facets through the SAME pooled seam
  nodes as the tube band, and let the ENGINE watertight check DISCARD any residual gap →
  OCCT. Never shipped leaky.
- **Sphere lens near-tangent pole.** A lens whose caps are shallow (spheres nearly
  tangent) puts the pole sample near the ON-band; `classifyPoint` returning 0 aborts →
  OCCT, and the engine correct-volume guard catches any that slips. Accepted.
- **Generic guard's native-COMMON dependency (fuse/cut).** The engine forms `expected`
  using the native COMMON; if that COMMON were itself unwatertight the guard falls back to
  trusting the watertight leg (existing behaviour). For the through-drill pair the native
  COMMON is the verified S5-a result, so `expected` is sound. Accepted.
- **Ring-count for the lens cap.** Picking too few rings under-resolves the cap volume;
  the sagitta-bound `kCapSagitta` sizing (same as `appendMouthCap`) keeps each facet's bow
  below the deflection, and the engine correct-volume guard is the backstop. A persistently
  failing cap declines → OCCT.
