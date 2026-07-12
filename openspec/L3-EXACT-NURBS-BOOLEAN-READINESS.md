# L3-EXACT-NURBS-BOOLEAN-READINESS — the NURBS-roadmap Layer-3 scoping map

**Scope of this document.** A **MEASUREMENT / SCOPING** doc (not an implementation) for
the NURBS roadmap **Layer 3 — the exact-NURBS B-rep boolean** (`docs/NURBS-SCOPE.md`
Layer-3 row: *"General exact-NURBS B-rep boolean, BOPAlgo-class, tolerant, non-manifold —
5–10 py, the hardest, the core of a B-rep kernel"*). L3 is the *deep frontier*, now
partially **unblocked by Layer 8** (`src/native/topology/trimmed_nurbs.{h,cpp}` — the
trimmed-NURBS data model + pcurve fidelity/construction).

It answers three questions, honestly:

1. **What does an exact-NURBS boolean need, stage by stage, and how far do the EXISTING
   kernel pieces already get?** (§2, the 5-stage readiness map — WORKS / PARTIAL / MISSING
   with measured evidence.)
2. **What is the FIRST tractable exact-NURBS boolean slice, and which pieces compose it?**
   (§3.)
3. **What is the honest gap list + py estimate to that first slice, and did Layer-8 / SSI
   recall actually move the needle?** (§4, §5.)

This doc mirrors the honesty discipline of [DROP-OCCT-READINESS.md](DROP-OCCT-READINESS.md):
every classification below is **measured**, not assumed. The measurement harness is
`tests/native/test_native_l3_boolean_readiness.cpp` (registered `test_native_l3_boolean_readiness`,
numsci build) — it drives real NURBS↔NURBS operand pairs through the existing pieces and
reports how far native gets per stage against an OCCT-FREE closed-form oracle (a rational
NURBS surface that *exactly* represents a quadric ⇒ the intersection is a known conic at
machine precision, the same known-answer trick as `test_native_ssi_exact_fuzz.cpp` leg 2).
`src/native` is **READ-ONLY** here (a concurrent healing track owns `topology/trimmed_nurbs`);
the harness modifies nothing in the kernel and adds no `cc_*` ABI.

**Method / discipline.** Each stage is classified exactly one of:
- **WORKS** — the existing piece serves the *typical in-domain* input at a measured bar,
  with an honest decline outside it.
- **PARTIAL** — a *degenerate/narrow slice* is served (and measured); the general case is
  declined honestly (never faked).
- **MISSING** — no native path; the case is declined (→ OCCT today).

---

## 1. The boolean pipeline (what an exact-NURBS boolean does, and where each stage lives)

A B-rep boolean `A ⊕ B` (fuse / cut / common) over two solids whose faces are trimmed
NURBS runs five stages. The existing kernel already has a working instance of this exact
pipeline for the **transversal ELEMENTARY** curved family — `src/native/boolean/ssi_boolean.h`
(SSI Stage **S5-a**) — driven off the S3 `TraceSet`. **L3 is the generalisation of that
same pipeline to NURBS↔NURBS operands.** The pieces, per stage:

| # | boolean stage | existing kernel piece | substrate |
|---|---|---|---|
| 1 | **Surface–surface intersection** (NURBS↔NURBS → 3-D curves) | `src/native/ssi/**` — `seed_intersection` (S2) + `trace_intersection`/`trace_from_seeds` (S3), via `makeNurbsAdapter` | numsci |
| 2 | **Pcurve construction** (3-D curve → each operand's (u,v)) | `topology/trimmed_nurbs.h` `constructPcurve` | numsci |
| 3 | **Face trimming / splitting** (split each operand face along the seam) | `boolean/face_split.h` (convex, 1 chord) + `boolean/smooth_trim_split.h` (closed interior seam); `trimmed_nurbs.h` `classify` = the inside-test | numsci (seam from S3) |
| 4 | **Region classification** (keep/discard per fuse/cut/common) | `trimmed_nurbs.h` `classify` (In/Out); `ssi_boolean` `classifyPoint` set-algebra (elementary) | numsci |
| 5 | **Reassembly / sewing** (stitch kept fragments watertight) | `boolean/assemble.h` weld (`SolidMesher`); `trimmed_nurbs.h` `pcurveFidelity` = the watertight-seam invariant | — |

---

## 2. STAGE-READINESS MAP (5 stages × readiness + measured evidence)

All residuals `file:line` are in `src/native/**`. The harness output cited is from
`test_native_l3_boolean_readiness` (numsci build, host, OCCT-free oracle).

### Stage 1 — Surface–surface intersection · **WORKS (transversal + closed interior loops)**

> **CORRECTION (Wave C, C2 investigation):** the earlier "1c closed interior loop → 0 seeds"
> below was a **flawed fixture, not a recall gap**. The `makeParaboloidPatch(-1.0, 1.2)` used for
> probe 1c has an actual surface maximum of z=0.30, while the probe plane sat at z=0.40 — so the
> true intersection is **empty**, and returning 0 seeds was **correct**. Re-measured with the
> plane *below* the dome's true max (z≈0.15–0.28): the seeder returns **1 seed → a fully-closed
> 446-point loop, radius 0.5, on-both-surfaces residual 2.0e-11** (the AABB-overlap prune fires
> on 3-D overlap, NOT on a domain-boundary crossing, so interior loops ARE bracketed). Co-resident
> interior loops already recover 2→2 via the landed scale-adaptive seeding. **Stage 1 closed
> interior loops WORK.** The genuine Stage-1 residual is the **near-tangent / merged multi-branch
> moat** (S4 marching/transversality), not a seeding-detector gap.

The `src/native/ssi/**` pipeline is **sound and now the L3 front-end**: `makeNurbsAdapter`
builds an adapter directly from a rational-NURBS pole/weight/knot grid, and
`seed_intersection` + `trace_from_seeds` produce on-both-surfaces WLines with fitted
B-spline seams. Measured on real NURBS operands:

| probe | operands | result | evidence |
|---|---|---|---|
| **1a exact** | rational-NURBS quarter-**cylinder** (R=1, exact `x²+y²=R²`) ∩ NURBS-**plane** z=1 | **WORKS** | 1 seed, 1 WLine; max on-both-surfaces residual **1.9e-11**; **max deviation from the exact analytic circle 5.6e-16** — the traced NURBS seam is the exact circle to machine precision, no OCCT |
| **1b transversal freeform** | two crossing bicubic NURBS "planes" (open line) | **WORKS** | 1 seed, 1 WLine; on-both-surfaces **4.2e-13** |
| **1c closed interior loop** | downward bicubic-NURBS **dome** ∩ NURBS **plane** (plane *below* the dome's true max) | **WORKS** (corrected) | **1 seed → fully-closed 446-point loop, radius 0.5, on-both-surfaces 2.0e-11.** The original "0 seeds" reading used a plane *above* the dome max (empty intersection) — a flawed fixture, not a gap. Co-resident interior loops recover 2→2. |

**Verdict 1: WORKS for NURBS operands with a transversal OR closed-interior-loop trace (exact,
freeform-open, and closed loops all land, on both surfaces).** The genuine Stage-1 residual is
the **near-tangent / merged multi-branch moat** (S4 marching/transversality) —
This corroborates the SSI-ROADMAP NURBS-Layer-2 empirical decline map: the general
NURBS↔NURBS decline is **≈13.9%** (canonical, DISAGREED==0), of which **~83% is
multi-branch / small-loop seeding-recall** — *not* the near-tangent marching moat (that
bucket measured **0%**; the marcher steps through freeform grazes cleanly). **The no-silent-
wrong invariant held: native never traced a curve off both surfaces.**

### Stage 2 — Pcurve construction · **PARTIAL**

`topology/trimmed_nurbs.h` `constructPcurve` is exactly the stage-2 verb: project sampled
3-D edge points to (u,v) via `numerics::closest_point_on_surface` and fit a 2-D B-spline,
then round-trip-verify `S(pcurve(t)) == C(t)`.

| probe | result | evidence |
|---|---|---|
| iso-curve `S(u, 0.5)` on a bicubic NURBS surface | **DECLINE (honest)** | projection residual (edge-on-S) **9.6e-5** (edge IS on S), but round-trip fidelity **maxDev 0.026** ≫ the scale-relative tol **2.1e-9** → `ok=false`, honest decline |

The decline is **honest and instructive**: the projection lands the feet on S, but the fit
re-parametrises by chord-length while fidelity re-evaluates at the edge's *original*
parameter — a **parametrisation mismatch** between `C(t)` and the fitted pcurve's `t`, plus
the fit being **non-rational** (a rational seam is approximated and its *true* deviation
reported, never a widened tolerance). So stage 2 **WORKS as a data model + fidelity guard**
(it detects a bad pcurve — see stage 5) but **constructPcurve does not yet reliably produce
a boolean-grade pcurve for a general NURBS seam** at the default bar. This is the
lowest-cost stage to strengthen (a parameter-aligned fit + a boolean-appropriate,
edge-length-relative fidelity tolerance).

**Verdict 2: PARTIAL** — the pcurve data model + fidelity guard land; robust construction
of a boolean-grade pcurve on a general NURBS operand is the residual.

### Stage 3 — Face trimming / splitting · **PARTIAL**

The split's inside-test primitive — `trimmed_nurbs.h` `classify` — **WORKS**:

| probe | verdict | evidence |
|---|---|---|
| interior (0.5,0.5) in a rectangular (u,v) trim | `In` | correct |
| exterior (0.05,0.5) | `Out` | correct |
| on-edge (0.2,0.5) | `OnBoundary` | correct (boolean cannot tolerate an either-way boundary — this is the whole point of Layer 8 over the mesher's `trim.h`) |
| open/degenerate loop | `Unknown` | honest decline, never a fabricated verdict |

The **split machinery itself is PARTIAL**: `boolean/face_split.h` tiles a **CONVEX** outer
loop cut by **ONE clean chord** (enters one boundary edge, exits another — no tangency, no
re-entry) with a host-checkable self-verify (`area(L1)+area(L2)==area(parent)`);
`boolean/smooth_trim_split.h` adds a **CLOSED interior seam** (disk + annulus). Both are
proven in isolation (`test_native_face_split`, `test_native_smooth_trim_split`). **MISSING:**
general multi-crossing / re-entrant / hole-crossing splits, and tolerant-topology healing
(auto-closing gapped loops, pinch-point resolution) — all declined today.

**Verdict 3: PARTIAL** — the inside-test primitive + two split slices (convex-1-chord,
closed-interior-seam) land; general multi-crossing splits + healing are MISSING.

### Stage 4 — Region classification · **PARTIAL**

The keep/discard verdict is the same `classify` In/Out primitive (WORKS on a single trimmed
face). The **fuse/cut/common set algebra exists** in `ssi_boolean` (`fuse = out∪out`,
`cut = out(A)∪in(B)ᴿ`, `common = in∩in`) — but only over **elementary** curved solids
recovered by `recogniseCurvedSolid` (Cylinder/Sphere/Cone). **MISSING:** a general
point-in-**SOLID** membership across *multiple* trimmed NURBS faces (the classifier that
tags a face fragment inside/outside the *other* NURBS solid). Today a NURBS/freeform operand
is declined at recognition (`ssi_boolean.h`: *"Anything richer — two distinct curved
surfaces, freeform, torus → nullopt → declined"*).

**Verdict 4: PARTIAL** — set-algebra + single-face In/Out land; general NURBS solid
membership is MISSING.

### Stage 5 — Reassembly / sewing · **PARTIAL**

The watertight-seam **invariant** — `trimmed_nurbs.h` `pcurveFidelity` — **WORKS and is
honest**:

| probe | verdict | evidence |
|---|---|---|
| faithful pcurve (v=0.5 iso-line seam) | **WELDS** | fidelity maxDev **2.8e-17** ≤ tol 2.0e-9 |
| drifted pcurve (v=0.6, off the edge) | **REJECTED** | fidelity maxDev **0.108** > tol → detected (a cracked seam is never passed) |

The **sew itself is PARTIAL**: `boolean/assemble.h` welds coincident vertices watertight and
the M0 tessellator has the **curved↔FLAT** seam pin (M0w) + the elementary curved seams
(`ssi_boolean` welds cyl/sphere/cone COMMON). **MISSING:** a general **curved-NURBS↔curved-
NURBS** watertight sew — two freeform faces meeting along a shared NURBS seam — which
`boolean/freeform_freeform_cut.h` **honest-declines to NULL today** (its two-curved-side
closed-seam weld is the fragility the M0w pin only *partially* resolves for the freeform↔freeform
case).

**Verdict 5: PARTIAL** — the seam-weld invariant lands; the general freeform↔freeform
watertight sew is MISSING.

> **UPDATE (L3-S3 landed, `nurbs-boolean-l3-s3`):** the general **freeform↔freeform** watertight
> sew is now **RESOLVED for the tractable COMMON single-transversal-seam pose**.
> `boolean/freeform_freeform_cut.h`'s two-curved-side closed-seam weld (split BOTH walls,
> membership select, orientation-coherence repair) — which the `ff_cut_honest_declines_never_leaky`
> probe above measured as declining for the naive weld — in fact WELDS the **COMMON lens**
> watertight once orientation coherence is repaired (the directed-edge invariant, exactly one cap
> reversed): proven for `Kind::Bezier` walls (`test_native_freeform_freeform_cut`) AND now for
> genuine `Kind::BSpline`/NURBS walls (`nurbs_freeform_split.h` `nurbsFaceFreeformSplit`, §3
> callout). The residual is the **CUT** leg (apex-ambiguous membership, honest-declined) + the
> multi-crossing / re-entrant split — NOT the curved↔curved COMMON sew itself.

> **UPDATE (CUT leg RESOLVED, track W — watertight curved-seam weld):** the **CUT (`A−B`)** leg
> now **WELDS watertight** and agrees with OCCT `BRepAlgoAPI_Cut` (DISAGREED=0). The measured
> re-diagnosis overturned the "mesher wall" framing: the two-curved-side closed seam
> (A-annulus's inner-hole boundary ↔ B-disk's outer boundary) already welds through the SAME
> M0w seam-chord pin the COMMON lens uses. **Measured on the canonical bowl-cup fixture at
> deflection 0.005:** the full CUT survivor set (A-annulus + B-disk + A-lid) meshes to
> **boundaryEdges = 0, χ = 2, consistently oriented**, with the 366 shared-seam nodes matched
> to **max ‖evalA − evalB‖ = 2.9e-14** — NOT the ~1788 open edges the naive framing predicted.
> The actual blocker was a **membership-probe bug, not a weld**: `subFaceCentroid3d` averaged
> the annulus's OUTER loop only, so its representative point was the disk CENTRE — which for
> the annulus (a face with the seam disk as a HOLE) lands in the REMOVED disk (the bowl apex at
> z=0), reading INSIDE B and declining `ClassifyAmbiguous`. Fix (bounded, in
> `freeform_freeform_cut.h`, M0 mesher UNTOUCHED): `subFaceInteriorReps` samples the sub-face
> respecting holes and votes membership over in-material points — every one of 125 annulus ring
> samples votes OUTSIDE B unanimously. **Result:** CUT meshed volume vs OCCT `BRepAlgoAPI_Cut`
> = 2.2%→1.5%→0.7% over deflection 0.01→0.005→0.0025 (monotone convergence, all watertight),
> host `ff_cut_welds_watertight_at_closed_form` + SIM `native_freeform_freeform_cut_parity.mm`
> `cut-welds-vs-occt` GREEN (SIM 14/14, DISAGREED=0). `src/native` stays OCCT-free; no `cc_*`
> ABI touched. The COMMON path is unchanged (its disks have no holes → the interior-sample vote
> agrees with the old centroid). The remaining L3 tail is the **multi-crossing / re-entrant /
> multi-seam** NURBS↔NURBS split and the closed-loop seeding recall — NOT the closed-seam
> curved↔curved weld, which is now resolved for BOTH legs of the single-transversal-seam pose.

### Summary table

| stage | readiness | one-line evidence |
|---|---|---|
| 1 Surface–surface intersection | **WORKS** | exact NURBS-cyl∩plane circle to **5.6e-16**; freeform open line **4.2e-13**; closed interior loop **1 seed → 446-pt loop 2.0e-11** (the "0 seeds" reading was a flawed empty-intersection fixture — corrected). Residual = near-tangent multi-branch moat (S4), ≈13.9% general decline |
| 2 Pcurve construction | **PARTIAL** | `constructPcurve` declines the iso-curve round-trip (parametrisation + non-rational fit); data model + fidelity guard land |
| 3 Face split | **PARTIAL** | `classify` inside-test WORKS; split = convex-1-chord + closed-interior-seam only; multi-crossing + healing MISSING |
| 4 Region classification | **PARTIAL** | single-face In/Out + elementary set-algebra land; general NURBS solid membership MISSING |
| 5 Reassembly / sew | **PARTIAL** | `pcurveFidelity` welds good / rejects drifted seam; general freeform↔freeform watertight sew MISSING |

---

## 3. The FIRST tractable slice (the simplest genuinely-exact-NURBS boolean within reach)

> **✅ LANDED (OpenSpec change `nurbs-boolean-l3-s1`).** This slice is now a shipped verb:
> `src/native/boolean/nurbs_plane_split.h` (`nurbsFacePlaneSplit`), composing exactly the
> pieces below. Two-gate proof: **host closed-form** `tests/native/test_native_nurbs_plane_split.cpp`
> (a genuine `Kind::BSpline` degree-2 bowl-cup reproducing `z=a·(x²+y²)` exactly, cut by z=c:
> CUT volume `π·ρ²·c/2` + COMMON `V(full)−that`, partition closure `V(below)+V(above)=V(full)`,
> seam fidelity `S(u,v)==C`=0 + on-both-surfaces ~7e-13 ⇒ DISAGREED=0, watertight χ=2, honest
> NULL declines) + **sim vs OCCT** `tests/sim/native_nurbs_plane_split_parity.mm` (native vs
> `BRepAlgoAPI_Cut` on a reconstructed `Geom_BSplineSurface` bowl-cup — volume/watertight/χ
> parity within the tessellation band). `src/native` stays OCCT-free; no `cc_*` ABI; `assemble.h`
> / `face_split.h` / `ssi` / `trimmed_nurbs` / `math` unmodified. The general NURBS↔NURBS boolean
> remains the §4 deep tail.
>
> **✅ ALSO LANDED — L3-S2 (OpenSpec change `nurbs-boolean-l3-s2`), the face∩CURVED-face
> extension.** `src/native/boolean/nurbs_curved_split.h` (`nurbsFaceCurvedSplit`) extends
> L3-S1 from a PLANE cutter to an **ANALYTIC CURVED** cutter (Cylinder/Sphere/Cone): the seam
> is a curve on BOTH curved surfaces and the sew is **curved-NURBS↔analytic-CURVED**, the
> stage-5 residual this doc named. It reuses L3-S1's `npsdetail::{makeWallAdapter, seamFidelity}`
> + S5-a's `ssidetail::{recogniseCurvedSolid, CurvedSolid::adapter, classifyPoint}` +
> `splitFaceSmoothTrim` + the S5-a `appendMouthCap` fan idiom (`assemble.h::{VertexPool,
> triangleFace}`), composing: NURBS-adapter ∩ curved-cutter-adapter **trace**[stage 1] →
> WLine-`(u,v)`-read fidelity on BOTH F and G (`S_F==C` AND `S_G==C`)[stage 2] →
> `splitFaceSmoothTrim`[stage 3] → **CURVED-solid membership** keep (`classifyPoint`, not a
> half-space)[stage 4] → **curved-G cap fan** synth (deflection-bounded planar-triangle fan on
> the true cutter surface, outer ring = exact seam nodes so it welds bit-for-bit to the NURBS
> disk)[stage 5] → watertight+volume self-verify. Two-gate proof: host closed-form
> `tests/native/test_native_nurbs_curved_split.cpp` (a `Kind::BSpline` paraboloid bowl cut by a
> genuine analytic SPHERE; the CUT/Below keep side is the closed-form **LENS**
> `2π[zc·ρ²/2 − a·ρ⁴/4] − (2π/3)[Rs³ − (Rs²−ρ²)^{3/2}]`, the meshed volume CONVERGING to it
> monotonely as the deflection refines — measured 7.2%→0.9% over 0.004→0.0005 — with χ=2,
> DISAGREED=0 [`S_F`=0, `S_G`~2e-11], honest NULL declines) + sim vs OCCT
> `tests/sim/native_nurbs_curved_split_parity.mm` (native vs `BRepAlgoAPI_Cut(cup, ball)` for
> the lens + `BRepAlgoAPI_Common(cup, ball)` for the inside piece — volume/watertight/χ parity
> within the tessellation band; 14/14). `src/native` stays OCCT-free; no `cc_*` ABI;
> `nurbs_plane_split.h` / `ssi_boolean.{h,cpp}` / `assemble.h` / `face_split.h` / `ssi` /
> `trimmed_nurbs` / `math` unmodified. **So stage 5 (the curved↔curved sew) is now resolved for
> the ANALYTIC curved cutter** — the general **freeform↔freeform** sew (both operands arbitrary
> NURBS) remains the §4 deep tail, alongside the closed-loop seeding recall and the
> multi-crossing split.
>
> **✅ ALSO LANDED — L3-S3 (OpenSpec change `nurbs-boolean-l3-s3`), the face∩FREEFORM-NURBS-face
> extension — the general freeform↔freeform sew, BOTH operands arbitrary NURBS.**
> `src/native/boolean/nurbs_freeform_split.h` (`nurbsFaceFreeformSplit`) removes the last analytic
> crutch: the cutter G is now a genuine **freeform NURBS** face (`Kind::BSpline`), so the kept-G cap
> is itself a curved NURBS sub-face (no closed-form fan) and the sew is **NURBS-disk↔NURBS-disk**
> along the shared curved seam — the **general M0 curved↔curved weld** this doc named as the
> **stage-5 deep-tail wall** (verdict 5 / the §4 tail row). The tractable slice reached it by
> COMPOSITION, not a re-implemented sew: `boolean/freeform_freeform_cut.h`
> (`freeformFreeformClosedSeamCut`) ALREADY performs the freeform↔freeform curved↔curved
> closed-seam weld (split BOTH walls, membership select, orientation-coherence repair) — proven
> watertight at the closed-form lens for the **COMMON** leg — but ONLY for `Kind::Bezier` walls.
> L3-S3 is that SAME weld with both walls left as genuine NURBS, reusing the surface-kind-agnostic
> `ffcdetail::{rekeyToB, pickByMembership, weldOrientationCoherent}` byte-identically and adding
> only the NURBS wall gate (`nfsdetail::nurbsWall`, requires `Kind::BSpline`) + the NURBS-adapter
> trace (`npsdetail::makeWallAdapter` on BOTH walls). It composes: recognise both NURBS operands →
> NURBS-adapter ∩ NURBS-adapter **trace**[stage 1] → WLine-`(u,v)`-read fidelity on BOTH F and G
> (`S_F==C` AND `S_G==C`)[stage 2] → `splitFaceSmoothTrim` on BOTH walls (bit-identical shared seam
> nodes)[stage 3] → **mesh-membership** keep (`classifyPointInMesh`)[stage 4] → **orientation-
> coherent** curved-NURBS↔curved-NURBS **weld** (the directed-edge invariant)[stage 5] →
> watertight+volume self-verify, landing the **COMMON (lens)** of two genuine-NURBS bowl-cups.
> Two-gate proof: host closed-form `tests/native/test_native_nurbs_freeform_split.cpp` (two
> `Kind::BSpline` paraboloid bowl-cups meeting in ONE closed circular seam; the COMMON lens
> **CONVERGING** to the closed form `V = π·H²/(4a)` monotonely **12.97%→1.87%** over deflection
> 0.01→0.00125, with χ=2, consistently oriented, DISAGREED=0 [`S_F`=0, `S_G`~2.8e-14,
> on-both-surfaces ~2.8e-14], honest NULL declines for a null operand / non-intersecting pair /
> the apex-ambiguous CUT leg) + sim vs OCCT
> `tests/sim/native_nurbs_freeform_split_parity.mm` (native vs `BRepAlgoAPI_Common(F, G)` on two
> reconstructed `Geom_BSplineSurface` cups — volume/watertight/orientation/χ parity within the
> tessellation band, OCCT cross-checked against the closed form; 9/9). `src/native` stays OCCT-free;
> no `cc_*` ABI; `nurbs_plane_split.h` / `nurbs_curved_split.h` / `freeform_freeform_cut.h` /
> `freeform_operand.h` / `freeform_membership.h` / `smooth_trim_split.h` / `ssi_boolean.{h,cpp}` /
> `assemble.h` / `face_split.h` / `ssi` / `topology` / `math` unmodified (all composed
> byte-identically). **So stage 5 (the general freeform↔freeform sew) is now resolved for the
> tractable COMMON single-transversal-seam pose** — the residual deep tail is now the **CUT
> (`F−G`) leg** (apex-ambiguous membership, honest-declined here too), **multi-crossing /
> re-entrant / multi-seam** NURBS↔NURBS splits, and the closed-loop seeding recall.

**Slice L3-S1 — a NURBS face SPLIT BY A PLANE, welded exact.** Concretely: **cut a single
trimmed NURBS solid by a half-space (planar cutter)**, keeping the exact-NURBS wall on the
kept side.

Why this is the first reachable genuinely-exact-NURBS case — it composes *only* pieces that
the harness measured as WORKS/near-WORKS, and it avoids every MISSING stage:

- **Stage 1** — the seam is **NURBS-wall ∩ plane**. When the plane cuts the wall in an
  **open transversal curve** that exits the (u,v) boundary, stage 1 **WORKS** (measured 1a:
  exact circle to 5.6e-16 on a rational-NURBS wall; 1b: freeform open line to 4.2e-13). It
  sidesteps the 1c closed-interior-loop seeding-recall gap.
- **Stage 2** — the plane side needs no NURBS pcurve (analytic), and the NURBS side's pcurve
  is the traced WLine's per-node `(u1,v1)` **read directly from S3** (each WLine node already
  carries its (u,v) on the NURBS operand) — so this slice **does not depend on
  `constructPcurve`'s general round-trip** (the stage-2 residual). The M1→B2 path in the
  existing freeform-boolean tests already consumes the WLine's (u,v) this way.
- **Stage 3** — the split is a **convex outer loop cut by ONE clean chord** →
  `boolean/face_split.h` (proven, `test_native_face_split`); or, for a closed rim, the
  **closed interior seam** → `boolean/smooth_trim_split.h` (proven, `test_native_smooth_trim_split`).
- **Stage 4** — keep/discard is a **half-space side test** (a plane is a closed-form
  half-space; `classify` + the plane's signed distance) — no general NURBS solid membership.
- **Stage 5** — the sew is **curved-NURBS-wall ↔ FLAT cap** — exactly the **M0w curved↔flat
  weld pin** that already welds watertight (`boolean/curved_wall_cut.h` does this today for
  the analytic-elementary wall; L3-S1 is the same weld with the wall's surface kind left as
  **BSpline/NURBS** instead of Cylinder). It avoids the MISSING freeform↔freeform sew.

**Pieces it composes:** `ssi::makeNurbsAdapter` + `makePlaneAdapter` → `seed_intersection` →
`trace_from_seeds` (stage 1) → WLine (u,v) read (stage 2, no constructPcurve) →
`face_split.h` / `smooth_trim_split.h` (stage 3) → half-space side test (stage 4) →
`assemble.h` M0w curved↔flat weld + `pcurveFidelity` gate (stage 5). This is the **exact
composition `curved_wall_cut.h` already performs for an elementary wall** — L3-S1 is that
verb with a genuine NURBS wall surface, gated by the two-gate discipline (host closed-form
volume where the wall reduces to a quadric + SIM vs OCCT `BRepAlgoAPI_Cut`).

**Explicitly deferred out of the first slice** (each a MISSING stage above): NURBS↔NURBS
where BOTH operands are curved (needs the freeform↔freeform sew, stage 5) · closed interior
seam loops that stage-1 seeding misses (stage 1 recall) · multi-crossing / re-entrant splits
(stage 3) · general NURBS solid membership (stage 4) · a boolean-grade general
`constructPcurve` (stage 2).

---

## 4. Honest gap list + py estimate to the first working exact-NURBS boolean slice (L3-S1)

> **✅ G1–G6 all LANDED** in `nurbs_plane_split.h` + the two-gate proof (OpenSpec change
> `nurbs-boolean-l3-s1`). The estimates below are retained as the historical scoping record.
> The dominant risk called out — **G5** (the NURBS-wall↔flat sew watertightness on a genuine
> NURBS grid) — resolved cleanly: the M0w curved↔flat pin welds the `Kind::BSpline` sub-face to
> the flat cap watertight (χ=2), volume-convergent, at the closed-form volume and at OCCT parity.

Ordered by the readiness map. Each is a *measured* gap, not a guess.

| # | gap | stage | what it needs | rough py |
|---|---|---|---|---|
| G1 | **NURBS-wall ∩ plane seam as an L3 operand** | 1 | wire `makeNurbsAdapter` from a topology NURBS `FaceSurface` (grid already there) into the boolean's operand recogniser; the seam trace already works (measured 1a/1b) | **0.25** |
| G2 | **Read the WLine (u,v) as the NURBS pcurve** (skip general `constructPcurve`) | 2 | reuse the M1→B2 (u,v)-read the freeform-boolean tests already use; NO general pcurve construction in this slice | **0.25** |
| G3 | **NURBS wall face split along the seam** | 3 | run `face_split.h` (convex/1-chord) or `smooth_trim_split.h` (closed rim) on a NURBS `FaceSurface` instead of a Bézier one — the split verbs are surface-kind-agnostic; verify tiling self-verify holds on a NURBS grid | **0.5** |
| G4 | **Half-space keep/discard on a NURBS fragment** | 4 | plane signed-distance side test at an interior UV sample (closed-form; no NURBS solid membership) | **0.25** |
| G5 | **Curved-NURBS-wall ↔ flat-cap watertight sew** | 5 | drive the M0w curved↔flat weld with the wall's surface kind = BSpline/NURBS; confirm watertight + χ=2 + closed-form volume where the wall reduces to a quadric | **0.5–1** |
| G6 | **Two-gate acceptance** (host closed-form volume + SIM vs OCCT `BRepAlgoAPI_Cut`) | all | the non-negotiable gate discipline, mirroring `curved_wall_cut.h` | **0.25** |

**py-to-first-slice (L3-S1) estimate: ≈ 2–2.5 py** (midpoint ~2.25). The dominant risk is
**G5** (the NURBS-wall↔flat sew watertightness across the tessellator on a genuine NURBS grid
— the M0w pin is proven for elementary walls, not yet exercised on a NURBS `FaceSurface`),
followed by **G3** (the split self-verify on a NURBS grid). G1/G2/G4/G6 are low-risk wiring.

**Beyond L3-S1 — the deeper L3 tail** (each a MISSING stage, NOT in the first slice):

| tail gap | stage | why it is the deep frontier | rough py |
|---|---|---|---|
| **Closed-interior-loop seeding recall** on freeform pairs | 1 | the ≈13.9% general NURBS decline, ~83% multi-branch/small-loop; a boolean seam is *usually* a closed loop; the SSI-ROADMAP measured a targeted-reseed campaign that landed 0.0 pt (hard) | **1–3** |
| **Boolean-grade general `constructPcurve`** | 2 | parameter-aligned + rational-capable fit at an edge-length-relative fidelity bar | **0.5–1** |
| **General multi-crossing / re-entrant face split + tolerant-topology healing** | 3 | the BOPAlgo-class combinatorial split + gapped-loop / pinch-point healing | **1–2** |
| **General NURBS solid membership** (point-in-trimmed-NURBS-solid) | 4 | ray-cast / winding across many trimmed NURBS faces, robust on tangencies | **0.5–1** |
| **General freeform↔freeform watertight sew** | 5 | the curved↔curved seam weld `freeform_freeform_cut.h` declines today | **1–2** |

**Deep-tail total ≈ 4–9 py** — consistent with `docs/NURBS-SCOPE.md`'s Layer-3 estimate of
**5–10 py** for the *general* exact-NURBS boolean. **L3-S1 (≈2–2.5 py) is the tractable first
verified slice; the general BOPAlgo-class boolean is the 5–10 py program the slices decompose.**

---

## 5. Did Layer-8 pcurve work + SSI recall move the needle for stages 1/2?

**Yes, materially — and the harness measures exactly how much.**

- **Stage 1 (SSI).** The SSI recall work (SSI-ROADMAP NURBS-Layer-2: scale-adaptive initial
  seeding 28.5%→18.8%, locus-coverage audit + freeform-pair seeding extension 18.8%→16.7%,
  seed-cluster distinct-branch split 16.7%→**13.9%**, all DISAGREED==0) **directly lowered the
  L3 stage-1 decline** and confirmed the honesty invariant (no fabricated traces). The harness
  independently reproduces the *shape* of the residual: transversal traces (open curves, both
  exact-NURBS and freeform) **land at machine precision** (1a 5.6e-16, 1b 4.2e-13), while the
  **closed interior loop** (1c) is the residual **0-seed recall gap**. So the recall work moved
  stage 1 from "unusable freeform front-end" to "usable for transversal open-curve seams
  (the first slice), with the closed-loop recall as the named deep-tail gap." **Net: stage 1
  is unblocked for L3-S1 *because of* the SSI recall work.**

- **Stage 2 (pcurve).** Layer 8 (`trimmed_nurbs`) is what makes stage 2 *exist* natively at
  all: the `TrimmedNurbsFace` data model, the **honest boolean-grade `classify`** (In/Out/
  OnBoundary/**Unknown** — the mesher's either-way boundary is unusable for a boolean, which
  is the whole reason Layer 8 was built over `tessellate/trim.h`), the **`pcurveFidelity`
  seam-weld guard** (measured: welds a good seam to 2.8e-17, rejects a drifted seam at 0.108),
  and **`constructPcurve`**. The harness shows the split verdict here is **nuanced but honest**:
  Layer 8 **fully unblocks stages 3/4/5's inside-test + weld-invariant** (all WORKS), but
  `constructPcurve` itself is **PARTIAL** at the boolean bar (the iso-curve round-trip
  declines) — *and the first slice L3-S1 deliberately routes around it by reading the WLine's
  (u,v) directly*, so **stage 2 does not block the first slice**. Layer 8 moved stage 2 from
  MISSING to a working data-model + fidelity guard, with general construction as the residual.

**Bottom line:** L8 + SSI recall turned stages 1 and 2 from "not started" into "the first
exact-NURBS boolean slice (a NURBS face split by a plane) is now composable from measured-
WORKS pieces (SSI transversal trace + WLine-(u,v) read + `face_split`/`smooth_trim_split` +
M0w curved↔flat sew + `pcurveFidelity`)." The general boolean remains the 5–10 py deep
frontier, gated on the **closed-loop seeding recall**, the **general freeform↔freeform sew**,
and the **multi-crossing split + healing** — the three named MISSING stages.

---

## 6. What this doc did NOT do (measurement discipline)

- **No kernel core modified.** `git diff HEAD -- src/native src/facade` is **empty**. The
  only changes are this doc + `tests/native/test_native_l3_boolean_readiness.cpp` + its CMake
  wiring. `cc_*` ABI byte-unchanged; `src/native` OCCT-free.
- **No boolean implemented.** The harness *composes and probes* existing pieces; it does not
  assemble a boolean result. A stage that declines honestly is a MEASUREMENT (the harness
  exits 0 on a clean run); it exits 1 only on a **broken invariant** (a fabricated trace off
  both surfaces, a non-round-tripping pcurve accepted, or a drifted seam passed as
  watertight) — so this doc's honesty is regression-guarded as the kernel evolves.
- **OCCT-free host oracle.** The stage-1 exactness (the 5.6e-16 circle) is proven against a
  closed-form analytic oracle (rational NURBS ≡ quadric), no OCCT. The differential SIM leg
  vs OCCT `BRepAlgoAPI_Fuse/Cut/Common` is the *acceptance* oracle for the eventual L3-S1
  implementation (per the two-gate discipline in §4 G6) — this doc scopes it, it does not run
  the boolean.

---

*This is a documentation / scoping artifact, not an OpenSpec change (mirroring
[DROP-OCCT-READINESS.md](DROP-OCCT-READINESS.md)). Parent roadmaps:
`docs/NURBS-SCOPE.md` (Layer-3 row), [SSI-ROADMAP.md](SSI-ROADMAP.md) (the S1–S5 SSI
pipeline + the NURBS-Layer-2 decline map), [NATIVE-REWRITE.md](NATIVE-REWRITE.md). The
Layer-8 prerequisite is the OpenSpec change `trimmed-nurbs-brep-model`.*
