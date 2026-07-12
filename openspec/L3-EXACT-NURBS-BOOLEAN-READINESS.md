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

### Stage 2 — Pcurve construction · **WORKS (parameter-aligned + rational; honest-declines the surface-nonlinearity residual)**

`topology/trimmed_nurbs.h` `constructPcurve` is exactly the stage-2 verb: project sampled
3-D edge points to (u,v) via `numerics::closest_point_on_surface` and fit a 2-D B-spline,
then round-trip-verify `S(pcurve(t)) == C(t)`.

> **✅ STRENGTHENED (L3-a, `L3-a-stage2-boolean-pcurve`).** The earlier decline was a
> **parametrisation mismatch**, now FIXED. The fit re-parametrised by CHORD-LENGTH while
> fidelity re-evaluated at the edge's ORIGINAL parameter → for a *curved* (u,v) seam the two
> drifted by ~0.30 at worst (the 0.026-class decline). The fix is a **parameter-aligned fit**:
> the edge C(t) is sampled at UNIFORM t and the projected feet are fitted with those SAME
> uniform parameters (new `math::interpolateCurveWithParams` — Piegl & Tiller §9.2 collocation
> with prescribed params, NOT a fresh chord-length reparam), knots remapped to [first,last], so
> `pcurve(t)` lands on `C(t)`'s foot at EVERY t, not just the sampled knots. **Measured, same
> host oracle:**

| probe | result | evidence |
|---|---|---|
| iso-curve `S(u, 0.5)` on a bicubic NURBS surface | **WELDS boolean-grade** | round-trip **maxDev 1.1e-15** (was 0.026) — the iso-line is affine-representable, reproduced to machine precision; second iso `S(u,0.35)` also ≤1e-9 |
| curved **freeform** seam on an **affine** surface (plane) | **WELDS boolean-grade** | chord-length fit drifts **0.30**; parameter-aligned welds **1.6e-16** (≥6 orders tighter) — the exact preimage is a genuine low-degree curve |
| **rational** circular seam on a **rational NURBS cylinder** (exact x²+y²=R²) | **WELDS boolean-grade** | round-trip **4.0e-16**; S(pcurve(t)) lies on the exact quadric to ≤1e-9 (was the old polynomial sag). A circular seam on a PLANE builds an EXACT **rational** pcurve (`interpolateRationalCurve`, weights non-empty), maxDev **0** |
| curved **freeform** seam on a **general bicubic** surface | **HONEST-DECLINE** | parameter-alignment slashes the drift **0.30 → ~2e-7** (≥4 orders), but the nonlinear-S preimage is NOT low-degree → the truncation residual stays above the 1e-9 bar → `ok=false`, TRUE deviation reported, **never a widened tolerance** |

The fidelity tolerance is **edge-length-relative** (`tol = absTol + relTol·L`, L the 3-D edge
length scale) so the guard stays honest: it welds a parameter-aligned seam and REJECTS a
genuinely drifted one. The residual is now sharply characterised: an **affine surface** (plane,
or a rational NURBS cylinder whose u-parameter IS the rational-quadratic parameter) reproduces
ANY polynomial/rational seam to machine precision; a **general bicubic** surface with a curved
seam retains an honest surface-nonlinearity truncation and DECLINES (never faked). This is the
**boolean-grade pcurve for the affine/rational seam family** the readiness map named as the
lowest-cost strengthening.

**Verdict 2: WORKS** — parameter-aligned + rational-capable `constructPcurve` welds a
boolean-grade pcurve on an affine or rational NURBS operand (iso-curve, curved freeform on a
plane, and a rational circular seam on a rational NURBS cylinder all ≤1e-9 / machine precision);
the general **curved-seam-on-a-nonlinear-surface** truncation is honestly declined with a
measured, orders-of-magnitude-reduced residual — never widened. Regression:
`test_native_trimmed_nurbs` (`testConstructionIsoCurve`, `testFreeformSeam`,
`testRationalCylinderSeam`).

### Stage 3 — Face trimming / splitting · **PARTIAL**

The split's inside-test primitive — `trimmed_nurbs.h` `classify` — **WORKS**:

| probe | verdict | evidence |
|---|---|---|
| interior (0.5,0.5) in a rectangular (u,v) trim | `In` | correct |
| exterior (0.05,0.5) | `Out` | correct |
| on-edge (0.2,0.5) | `OnBoundary` | correct (boolean cannot tolerate an either-way boundary — this is the whole point of Layer 8 over the mesher's `trim.h`) |
| open/degenerate loop | `Unknown` | honest decline, never a fabricated verdict |

The **split machinery** now has THREE proven slices: `boolean/face_split.h` tiles a
**CONVEX** outer loop cut by **ONE clean chord** (enters one boundary edge, exits another —
no tangency, no re-entry) with a host-checkable self-verify (`area(L1)+area(L2)==area(parent)`);
`boolean/smooth_trim_split.h` adds a **CLOSED interior seam** (disk + annulus); and (L3-b)
`boolean/multi_crossing_split.h` adds the **general MULTI-CROSSING / RE-ENTRANT / HOLE-
CROSSING** split into **N ≥ 2** sub-regions. The multi-crossing verb builds the planar
arrangement of boundary arcs (outer + holes) + seam arcs — splitting every arc at all
pairwise crossings (seam×outer, seam×hole, seam×seam) with the Wave-I `trim_boolean` segment-
crossing closed form — welds them into a DCEL, and walks the sub-region faces by the tightest-
clockwise rotational rule (the same orientation-coherent arc-walk family as `trim_boolean.cpp`:
outer CCW, holes CW). It self-verifies **Σ area(sub-regions) == area(parent)** and each
sub-region a simple loop; tangent/coincident seams honest-decline (`Degenerate`), non-cutting
seams `NoSubdivision` — never a wrong tiling. All three are proven in isolation
(`test_native_face_split`, `test_native_smooth_trim_split`, `test_native_multi_crossing_split`).

| oracle | verdict | area error |
|---|---|---|
| 2-chord (3 regions), convex face | `Ok` | Σ == parent, gap **0** (machine) |
| crossing chords (4 quadrants, each 0.25) | `Ok` | Σ == parent, gap **0** |
| hole-crossing seam (net-area tiling) | `Ok` | Σ net == parent (15), gap **0** |
| isolated-hole attribution | `Ok` | Σ net == parent, gap **0** |
| re-entrant U-cut (3 pieces) | `Ok` | Σ == parent (8), gap **0** |
| coincident-edge / non-cutting seam | **DECLINE (honest)** | `Degenerate` / `NoSubdivision`, no tiling |

**Still MISSING:** tolerant-topology healing (auto-closing gapped loops, pinch-point
resolution) — declined today.

**Verdict 3: MOSTLY LANDED** — the inside-test primitive + THREE split slices (convex-1-chord,
closed-interior-seam, general multi-crossing/re-entrant/hole-crossing) land with exact
(machine-precision) tiling on the closed-form oracles; only tolerant-topology healing remains.
The **split machinery itself is PARTIAL**: `boolean/face_split.h` tiles a **CONVEX** outer
loop cut by **ONE clean chord** (enters one boundary edge, exits another — no tangency, no
re-entry) with a host-checkable self-verify (`area(L1)+area(L2)==area(parent)`);
`boolean/smooth_trim_split.h` adds a **CLOSED interior seam** (disk + annulus). Both are
proven in isolation (`test_native_face_split`, `test_native_smooth_trim_split`). **MISSING:**
general multi-crossing / re-entrant / hole-crossing splits.
> **UPDATE (L3-HEAL landed) — the TOLERANT-TOPOLOGY HEALING PRE-PASS is now RESOLVED.**
> `boolean/split_healing.h` (`healTrimLoops`) repairs a RAW SSI-derived trim-loop set (small
> gaps, near-coincident vertices, pinch points) into split-ready valid simple loops OR declines
> honestly — the "auto-closing gapped loops, pinch-point resolution" this row named as MISSING.
> It is a bounded PRE-PASS by **COMPOSITION**, not a re-implemented healer: it drives the
> Wave-G/G5 healing ALREADY in `topology/trimmed_nurbs` byte-identically — `healTrimLoop`
> (gap-close / snap / large-gap decline diagnosis) + `splitTrimLoopAtPinches` (weld small gaps →
> resolve N-way / crossing pinches into simple sub-loops, region- AND signed-area-preserving) +
> `flattenTrimLoop` (the split's own seam-consistent flattener) — and ADDS one host-checkable
> gate: Σ signedArea(output) == Σ signedArea(input) within a scale-relative tolerance, else the
> whole set honest-declines (a split never consumes a partially-healed set). Four airtight oracles
> (`test_native_split_healing`, host, OCCT-free, always-on suite — the composed primitives are
> not numsci-gated): **(1) clean loop → byte-identical NO-OP** (0 real heals, arcs unchanged;
> ULP-level closing-dedup welds are floored out as noise); **(2) small-gap loop → welded CLOSED +
> SPLIT-READY** (signed area preserved ≤ 1e-12; the closed loop passes face_split's own
> simple-polygon / area-floor readiness predicates); **(3) figure-eight PINCH → two valid simple
> sub-loops** (reusing G5 `splitAtPinches`, signed area preserved); **(4) over-tolerance gap
> (≫ tol) → HONEST LargeGap DECLINE** — never force-welded, and the tolerance is never widened to
> force a heal (a wider-but-still-<gap tol still declines). 52 checks GREEN. `src/native` stays
> OCCT-free; no `cc_*` ABI; `trimmed_nurbs.{h,cpp}` / `face_split.h` / `smooth_trim_split.h`
> UNCHANGED (composed byte-identically). The residual is now the **general multi-crossing /
> re-entrant / hole-crossing SPLIT** itself — NOT the tolerant-topology healing, which is landed.
**Verdict 3: PARTIAL** — the inside-test primitive + two split slices (convex-1-chord,
closed-interior-seam) **+ the tolerant-topology healing pre-pass (L3-HEAL, `split_healing.h`)**
land; the general multi-crossing / re-entrant split is the remaining MISSING piece.

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

> **UPDATE (L3 Stage-4 point-in-NURBS-solid membership LANDED, track L3-c):** the general
> **point-in-SOLID membership across MULTIPLE trimmed NURBS faces** — the classifier that
> tags a face fragment inside/outside the OTHER freeform solid, named MISSING above — is now
> a shipped OCCT-free verb: `src/native/boolean/nurbs_solid_membership.h`
> (`pointInNurbsSolid` + `classifyFragmentVsSolid`). It composes exactly the two measured-
> WORKS Stage-4 primitives this doc lists — the H1 exact intersector
> `math/bspline_intersect.h intersectCurveSurface` + `topology/trimmed_nurbs.h classify` —
> into an EXACT (no-mesh) ray-cast: shoot a ray from the query point along a generic
> direction (a degenerate degree-1 line = the `cc`/`cs` curve), intersect it with each
> trimmed face's TRUE surface, and count as a crossing only the forward hits whose (u,v)
> classify `In` the face's trim loops; odd count ⇒ inside. A hit ON a trim edge
> (`OnBoundary`/`Unknown`), a TANGENTIAL surface hit, or a curve-on-surface `Coincident`
> decline makes the ray ambiguous, and it RE-CASTS in a fresh generic direction (8 fixed,
> non-parallel, non-axis-aligned) rather than miscounting; after K directions with no clean
> ray the verdict is an honest `Unknown`, NEVER a guessed In/Out. `classifyFragmentVsSolid`
> samples interior representatives of a fragment (respecting holes) and votes — the batch
> region classifier the keep/discard select consumes; a straddling fragment's interior-rep
> vote is well-defined (both In and Out votes, `straddles` flagged). **Proof (host closed-
> form gate `tests/native/test_native_nurbs_solid_membership.cpp`):** a genuine-NURBS-walled
> bowl-cup (BSpline paraboloid bowl `z=a(x²+y²)` + flat lid, UV mapping x=u−0.5, y=v−0.5 so
> the in/out is EXACT) classified on a **945-point membership grid to 100% crisp-correct (no
> silent-wrong, 0 On, 0 Unknown clear of the boundary)**; on-boundary/tangent points resolve
> to a defined verdict (On/In/Out) via re-cast, never a wrong crisp; a fragment entirely
> inside → In, entirely outside → Out, and a straddling fragment votes both (32 In / 20 Out,
> flagged). `src/native` stays OCCT-free (0 OCCT refs in the changed header); no `cc_*` ABI;
> header-only, numsci-gated ray-cast body with an honest-decline stub when the substrate is
> off; `bspline_intersect` / `trimmed_nurbs` / `ssi_boolean` / `assemble` unmodified.
> **So the Stage-4 general NURBS solid membership is now RESOLVED** — a freeform operand no
> longer has to decline at `recogniseCurvedSolid` for the keep/discard verdict. The residual
> Stage-4 tail is only near-boundary/coincident-face configurations that stay honest
> `Unknown` after every re-cast direction, and analytic-curved-wall solids (Cylinder/Sphere/
> Cone), which keep their existing closed-form `classifyPoint` path (`ssi_boolean.h`).

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

> **UPDATE (L3-d — MULTI-SEAM split RESOLVED; annulus↔annulus sew is a FROZEN-MESHER
> honest-decline, track `worktree-agent-a82ec06b23a4baf52`):** the **multi-seam SPLIT +
> CLASSIFY** machinery — the case where two freeform walls meet along **>1 closed seam** (the
> SSI returns several loops) — is now **RESOLVED**, and the annulus↔annulus **SEW** is a
> sharply-mapped **frozen-M0-mesher honest-decline** (never leaky). `boolean/freeform_freeform_multiseam.h`
> (`freeformFreeformMultiSeamCut`, additive; `freeform_freeform_cut.h` byte-unchanged) traces
> ALL closed seams (requires ≥2, else `NoMultiSeam` → track W), splits **BOTH walls by ALL
> seams** into a **nesting-aware** (loops+1) sub-region set (inner disk + middle annulus +
> background) that **tiles the parent EXACTLY** (UV gap 0), classifies **EACH** sub-region by
> W's hole-respecting `subFaceInteriorReps` vote, and sews the survivors with an
> orientation-coherence repair + a **mandatory M0 watertight/coherent/two-sided-volume
> self-verify**. **Measured on a genuine 2-seam pose** (a degree-4 VALLEY cup mirrored into a
> degree-4 DOME cup — walls meet in TWO concentric circles r₁≈0.131, r₂≈0.374, the SSI trace
> returning exactly **2 closed loops** at 2.0e-11 on-surface): the split tiles exactly
> (gap 0), the middle annulus of A votes INSIDE B and of B INSIDE A (the annular lens), and
> the **OUTER seam** (seam-as-OUTER on both annuli) **welds watertight** — but the **INNER
> seam** (seam-as-HOLE on both annuli) hits the **frozen M0 mesher's holed-curved-annulus weld
> gap** (the SAME residual the L3-S3 NURBS CUT leg named: the mesher's per-face
> curvature-driven CDT never welds two curved trimmed faces across a shared HOLE seam). So the
> verb **HONEST-DECLINES to NULL** (`NotWatertight`) with a **sharpened residual map**
> (`boundaryEdges` — COMMON≈59, CUT≈307 unpaired edges, ALL localized to the inner seam, out
> of ~10⁴ shell edges — the sew is complete but for the one holed seam), **NEVER a
> leaky/partial/wrong solid; no tolerance widened.** Two-gate proof: host closed-form
> `tests/native/test_native_freeform_freeform_multiseam.cpp` (8/8 GREEN — 2-loop trace, exact
> multi-seam tiling, per-region membership, the honest annulus-lens decline + residual map,
> the single-seam pose declined `NoMultiSeam`, **W's single-seam weld UNREGRESSED** [still
> welds watertight χ=2, be=0], non-operand declined) + SIM vs OCCT
> `tests/sim/native_freeform_freeform_multiseam_parity.mm` (**7/7 GREEN, DISAGREED=0**: OCCT
> confirms the two seams on BOTH degree-4 surfaces to 1.2e-11 and its `BRepAlgoAPI_Common` =
> the closed-form lens 0.007695 EXACTLY, while native **honest-declines** — native abstains,
> never fabricates a solid OCCT would contradict). `src/native` stays **OCCT-free**; **no
> `cc_*` ABI** touched; the M0 mesher, `freeform_freeform_cut.h`, `smooth_trim_split.h`, `ssi`,
> `topology`, `math` all **byte-unchanged**. **Net Stage-5 verdict: the multi-seam
> SPLIT+CLASSIFY is RESOLVED; the multi-seam annulus↔annulus watertight SEW is a bounded,
> airtight-honest-decline** gated by the frozen M0 mesher's holed-curved-annulus weld — the
> next real enabler (a mesher-level shared-seam-as-hole weld, out of this additive slice's
> scope). The residual L3 tail is now the **re-entrant** split shape (nesting handled; a
> genuinely self-re-entering seam untested) and that mesher-level holed-seam weld.

> **UPDATE (W2 — annulus↔annulus inner-seam weld MEASURED to mesher-rewrite scale; honest
> re-decline with a SHARPENED root-cause map, track `worktree-agent-a25f2777e6216cfa6`):** a
> measurement-first attack on the exact L3-d residual (the multi-seam INNER-seam weld) confirms
> it is a **genuine M0 CDT mesher rewrite, NOT a bounded topology weld**, and sharpens the map
> from "holed-curved-seam gap" to the precise failing mechanism. **Localization (host, d=0.0025,
> `SolidMesher`):** the COMMON survivor set (A's middle annulus + B's middle annulus) meshes to
> **boundaryEdges = 59, ALL 59 at the INNER seam** (r₁≈0.131, z=H/2=0.015; `nearOuter=0,
> other=0`) — the OUTER seam (r₂≈0.374) welds watertight (0 unpaired). CUT = 307, likewise all
> inner. **Root cause (measured, not assumed):** the two annuli's inner-seam boundary VERTICES
> are near-coincident (the seam NODES agree to **1.3e-11** across the two walls; the seam-chord
> pin places the subdivided samples within ULP), yet the two faces are meshed by **INDEPENDENT
> per-face constrained-Delaunay triangulations** whose inner-HOLE boundary tessellations
> disagree by one edge (A=822 vs B=823 hole-boundary edges) — the CDT's hole-culling
> (`triangles()` centroid-in-hole test) drops a thin near-boundary triangle on ONE annulus but
> not the other, because the two annuli bulge OPPOSITE ways off the shared flat chord, so a
> near-hole centroid lands inside the hole loop for one wall and outside for the other. The
> unpaired edge propagates around the ring. **The residual GROWS with refinement** — measured
> boundaryEdges **2 → 22 → 59 → 233 → 769** over deflection **0.01 → 0.005 → 0.0025 → 0.00125 →
> 0.000625** — the unambiguous signature of a per-face-CDT PARITY gap (more boundary samples ⇒
> more independent-CDT disagreement), not a boundary-placement gap (which would SHRINK with
> refinement, as the OUTER seam does). **A bounded topology fix was implemented and measured to
> NOT help:** unifying both walls' inner-seam 3-D chord geometry to bit-identical canonical
> poles (so `EdgeCache` hands both annuli the SAME `d.points`) leaves the residual UNCHANGED at
> 59 — proving the gap is in the CDT triangulation of the shared hole strip, NOT in boundary
> vertex placement. The genuine enabler is a **mesher-level shared-seam-as-hole weld** (mesh the
> shared inner-hole boundary strip ONCE, shared by both faces, instead of two independent
> per-face CDTs) — a change to the frozen M0 mesher CORE, explicitly out of the additive
> boolean slice's scope. **So W2 re-affirms the L3-d honest-decline** (`NotWatertight`, residual
> localized + refinement-growing) as a first-class, DISAGREED=0-safe outcome, with the residual
> now mapped to the exact CDT parity mechanism. `src/native` UNCHANGED (the bounded fix was
> measured and reverted as non-improving); no `cc_*` ABI touched; the 8/8 host gate + 7/7 SIM
> parity (DISAGREED=0) remain GREEN.

> **UPDATE (M0-WELD — the shared-seam-strip weld LANDED; the multi-seam annulus↔annulus inner
> seam now WELDS watertight, track `worktree-agent-aa3785779f27b219f`):** W2's exact root-cause
> map turned out to name a **bounded** fix, not a mesher rewrite. The failing mechanism was
> `ConstrainedDelaunay::triangles()`'s **per-triangle centroid-in-hole GEOMETRIC cull**: for a
> thin near-hole triangle whose centroid sits ~on the shared hole loop, the two annuli bulge
> opposite ways off the shared flat seam chord, so the centroid lands *inside* the hole for one
> wall and *outside* for the other — the two per-face CDTs drop a **different** near-boundary
> triangle and the residual GROWS with refinement (2→22→59→233→769). The fix (in
> `src/native/tessellate/uv_triangulate.h`, `ConstrainedDelaunay::triangles()` only) replaces
> that cull with a **TOPOLOGICAL flood fill**: the loop edges are constrained (never flipped) —
> a hard topological wall — and a triangle is kept iff it is reachable, across NON-constrained
> edges only, from a triangle incident to the OUTER boundary loop. The decision then depends
> ONLY on the constrained-edge topology (the **SAME** shared seam loop on both annuli), so the
> two faces cull the shared hole strip **IDENTICALLY** and it welds 2-manifold; a
> constrained-fold pass drops the zero-area hole-bridge slivers so each boundary edge borders
> exactly one ring triangle. **Measured (host, `SolidMesher`):** the multi-seam COMMON survivor
> set's inner-seam boundaryEdges go **59 → 0** at d=0.0025 (and 0 at d=0.01, 0.005), the residual
> now **SHRINKS** with refinement, and the meshed COMMON volume converges to the closed-form
> annular lens (relErr **0.139 → 0.081 → 0.045** over d 0.01→0.005→0.0025); CUT converges
> likewise (relErr ≤ 0.2% at d=0.005–0.00125). **Impact:** the L3-d multi-seam annulus↔annulus
> COMMON/CUT sew now **WELDS watertight** (χ correct, coherent, boundaryEdges 0) instead of
> honest-declining; the single-seam `nurbsFaceFreeformSplit` **CUT leg** (annulus↔disk shared
> seam-as-hole) now welds too; and the `nurbsSolidBoolean` orchestrator's multi-seam **CUT/COMMON
> weld** (FUSE, the outer-envelope compose, still honest-declines — not exposed by the sew verb).
> Two-gate proof: host `test_native_freeform_freeform_multiseam` (welds watertight + monotone
> convergence) + `test_native_nurbs_solid_boolean` + `test_native_nurbs_freeform_split` (CUT
> welds) GREEN; SIM `native_freeform_freeform_multiseam_parity.mm` now asserts native-vs-OCCT
> `BRepAlgoAPI_Common` **agreement** within the tessellation band (**DISAGREED=0 by AGREEMENT**,
> native answers + OCCT confirms — no longer by abstention). `src/native` stays **OCCT-free**; no
> `cc_*` ABI; **every existing tessellation/boolean/blend mesh is UNREGRESSED** (the flood fill
> reaches exactly the ring the centroid test kept for a hole-free or well-behaved holed region;
> `face_mesher` / `solid_mesher` / `freeform_freeform_cut` / `freeform_freeform_multiseam` /
> `smooth_trim_split` byte-unchanged). **Residual:** a small non-manifold count reappears at
> deflections FINER than the working band (d ≤ 0.00125) from hole-bridge sliver interactions in
> the shared ear-clip base — a distinct, smaller effect than the closed parity gap, out of this
> bounded slice's scope; the working deflection band [0.0025, 0.01] welds cleanly and converges.

> **UPDATE (MESH-FINE — the fine-deflection residuals MEASURED to the exact mechanism; honest
> re-decline with a SHARPENED root-cause map, track `worktree-agent-a41f35719341d5809`):** a
> measurement-first attack on the two named fine-deflection residuals (the M0-WELD inner-seam
> sliver at d ≤ 0.00125 and the BOOL-FUSE-MULTI outer-seam T-junction at d ≤ 0.004) confirms
> BOTH are the SAME rewrite-scale mechanism W2 named — a **per-face-CDT-in-UV vs shared-3-D-seam
> parity gap** — NOT bounded ear-clip/topology fixes, and sharpens the map to the precise
> failing geometry. **Localization (host, `SolidMesher`, raw welded edge-use histogram by
> radius):** COMMON/CUT are the two MIDDLE annuli sharing BOTH seams; CUT welds watertight to
> **d = 0.001** (0 non-manifold), COMMON welds to **d = 0.002** and reappears **NONMANIF = 4 → 9**
> at d = 0.00125 → 0.001, ALL at the INNER seam r₁ (the residual GROWS with refinement — the
> parity-gap signature). FUSE keeps each wall's OUTER background annulus (bounded by the rim) and
> reappears **NONMANIF = 1 → 5 → 19 → 57** at d = 0.0025 → 0.002 → 0.00125 → 0.001, ALL at the
> OUTER seam r₂, PLUS a broad `oth` open-boundary crack (8368 / 14128 edges at d = 0.002 / 0.00125)
> in the rim-to-r₂ background↔lid tessellation — a THIRD, coarser defect. **Root cause (measured,
> not assumed — per-face mesh dump at d = 0.001):** at fine deflection the CDT refinement inserts
> an interior curvature Steiner point whose (u,v) is genuinely INTERIOR to the annulus (it passes
> `region.inside` and `locate`), yet whose SURFACE value S(u,v) lands ~ON the shared seam CIRCLE
> (r = r₁ in 3-D) — the wall surface is steep/near-vertical there, so a UV-interior point maps onto
> the 3-D seam. Because the two annuli sharing the seam sample the SAME domain, the M0-WELD flood
> fill makes them place the **IDENTICAL** on-seam interior vertex (verified: A and B produce
> 77 = 77 seam-lying interior edges at d = 0.00125), and the interior edges to it lie on the seam,
> so the welded solid uses that shared 3-D edge FOUR times (two triangles per face) — a
> non-manifold edge. **Three bounded UV-space fixes were implemented and each measured to NOT help
> without a volume-convergence regression, then reverted:** (a) a grid-margin guard that keeps
> curvature samples a fraction of a cell off every loop, (b) a CDT `insert()` guard that skips a
> Steiner point within a UV margin of any hole loop, and (c) a `legalize()` guard that refuses a
> flip creating a boundary-hugging same-hole-loop chord — (a) at a broad margin FIXED the coarse
> defect but flipped CUT's strict monotone-convergence check (a real regression), and at a tight
> margin left tris byte-identical; (b) and (c) were **exact no-ops even at 3× the seam step**,
> because the offending point is FAR from the seam in UV (the surface distortion is exactly what
> makes a UV-interior point map onto the 3-D seam) — proving the gap is unreachable from any
> per-face UV-space reasoning. **The genuine enabler is unchanged from W2:** a **mesher-level
> shared-seam-strip weld** — mesh the seam-adjacent strip ONCE, shared by both faces (so the
> near-seam interior samples are common, not two independently-welded coincidences), instead of
> two independent per-face CDTs in each face's own (u,v) — a change to the frozen M0 mesher CORE,
> explicitly out of the additive boolean slice's scope. **So MESH-FINE re-affirms the fine-band
> honest-decline** (`NotWatertight`, residual localized to the shared seam + refinement-growing) as
> a first-class, DISAGREED=0-safe outcome, with the residual now mapped to the exact UV-interior-
> maps-onto-3-D-seam mechanism (a step past W2's "per-face-CDT parity" to the precise geometry).
> `src/native` UNCHANGED (the three bounded fixes were measured and reverted as non-improving or
> regressing); no `cc_*` ABI touched; `src/native` stays OCCT-free; the working deflection band
> ([0.002, 0.01] for COMMON, [0.001, 0.01] for CUT modulo an isolated d = 0.004 edge-discretization
> resonance, [0.005, 0.01] for FUSE) welds watertight + converges, and the full host + SIM gates
> remain GREEN.

### The COMPOSED two-freeform-solid NURBS boolean ORCHESTRATOR · **LANDED (BOOL-INT)**

> **UPDATE (BOOL-INT — the general two-freeform-solid boolean ORCHESTRATOR that COMPOSES all
> five landed stages, track `worktree-agent-a15cb7dd0b2f2a565`):** the flagship
> `nurbsSolidBoolean(A, B, op)` for **op ∈ {Fuse, Cut, Common}** over two general freeform
> NURBS solids is now a shipped verb: `src/native/boolean/nurbs_solid_boolean.h` (additive,
> header-only). It **COMPOSES** the five landed stage verbs — it re-implements NONE of them
> (the five stage-verb files stay **BYTE-UNCHANGED**): Stage 1 SSI `ssi::trace_intersection`
> over `makeBezierAdapter` walls; Stage 2 the WLine per-node `(u,v)` read directly (the
> `constructPcurve`-free tractable-slice path); Stage 3 `splitFaceSmoothTrim` (single seam) +
> `freeform_freeform_multiseam.h splitWallBySeams` (multi-seam nesting-aware); Stage 4
> `ffcdetail::subFaceHasMembership` hole-respecting interior-rep vote (the
> `classifyFragmentVsSolid` batch-select form); Stage 5 `freeformFreeformClosedSeamCut`
> (single-seam W weld) + `freeformFreeformMultiSeamCut` (L3-d multi-seam). The op-level
> dispatch: **SSI-trace the shared closed seams once**; on **ONE** closed transversal seam,
> COMMON/CUT **delegate byte-identically** to `freeformFreeformClosedSeamCut` and **FUSE**
> runs `nurbsSolidFuse` (the OUTER-envelope compose the single-seam verb did NOT expose — A's
> annulus+lid OUTSIDE B ∪ B's annulus+lid OUTSIDE A, welded through the SAME M0w seam pin with
> a **group-aware orientation-coherence repair** that flips the whole B- or A-group, since the
> two rim annuli wind oppositely across the shared seam); on **≥2** closed seams (genuine
> multi-seam) it routes to `freeformFreeformMultiSeamCut`, which splits+classifies exactly and
> **HONEST-DECLINES** the annulus↔annulus inner-seam sew (the frozen-M0-mesher holed-curved-seam
> gap, L3-d) with a residual map. Every op **self-verifies** (watertight + consistently-oriented
> + a per-op volume bound, TWO-SIDED vs the closed form when supplied); any abstaining sub-case
> returns a **NULL Shape** with a measured `SolidBoolReport` — **NEVER a leaky/partial/wrong
> solid; no tolerance widened.**
>
> **Two-gate proof.** Host closed-form `tests/native/test_native_nurbs_solid_boolean.cpp` (7/7
> GREEN): on the canonical single-transversal-seam bowl-cup, **COMMON / CUT / FUSE all WELD
> watertight** (χ=2, boundaryEdges=0, consistently oriented) at the closed-form volumes
> (π·H²/(4a) lens; V(A)−lens; V(A)+V(B)−lens envelope), each **CONVERGING** as the deflection
> refines; **op-algebra** V(fuse)+V(common)=V(A)+V(B) and V(cut)+V(common)=V(A) hold on the
> meshed volumes (≤2%); CUT is per-operand (A−B and B−A both weld, equal volume on the
> z-mirror-symmetric pose); the **MULTI-SEAM** pose (two degree-4 mirror cups, 2 seams)
> **HONEST-DECLINES** `MultiSeamDeclined` with the residual map (boundaryEdges localized to the
> inner seam, never leaky); a null operand declines `NotAdmittedA`. SIM vs OCCT
> `tests/sim/native_nurbs_solid_boolean_parity.mm` (**14/14 GREEN, DISAGREED=0**): native
> `nurbsSolidBoolean` COMMON/CUT/FUSE vs OCCT `BRepAlgoAPI_{Common,Cut,Fuse}` on the SAME
> reconstructed `Geom_BezierSurface` bowl-cups — all three weld watertight and match OCCT's
> volume within the tessellation band, **converging** (FUSE relErr 3.5%→2.1%→1.0%; CUT
> 2.2%→1.5%→0.7%; COMMON 13%→6.1%→3.4% over d 0.01→0.005→0.0025), and OCCT's own op-algebra
> V(fuse)+V(common)=V(A)+V(B) confirms the closed-form oracle. `src/native` stays **OCCT-free**
> (0 OCCT refs in the new header); **no `cc_*` ABI**; the M0 mesher, `freeform_freeform_cut.h`,
> `freeform_freeform_multiseam.h`, `smooth_trim_split.h`, `nurbs_solid_membership.h`, `ssi`,
> `topology`, `math` all **byte-unchanged**. **Net: the general single-transversal-seam
> two-freeform-solid NURBS boolean (Fuse/Cut/Common) is RESOLVED and OCCT-parity-verified; the
> multi-seam annulus↔annulus sew remains the bounded, airtight-honest-decline** gated by the
> frozen M0 mesher's holed-curved-seam weld (the same L3-d residual). The residual L3 tail is
> unchanged: the closed-loop seeding recall, the mesher-level holed-seam weld, and re-entrant
> split shapes.

### The boolean OUTPUT made RE-ADMISSIBLE as a boolean INPUT · **LANDED (BOOL-READMIT)**

> **UPDATE (BOOL-READMIT — the N-operand fold now RE-ADMITS the binary boolean's welded output
> for ≥3 operands, track `worktree-agent-afe5a62bce3b42fe4`):** the N-ary fold
> (`nurbs_solid_boolean_nary.h`) previously HONEST-DECLINED at fold index 2 because the binary
> boolean's welded output is NOT a pristine single-wall bowl-cup — it is a MULTI-freeform-wall
> solid whose walls are SEAM-SPLIT ANNULI (the shared seam is an interior HOLE loop), which the
> byte-frozen B1 gate `recogniseFreeformSolid` declines on TWO measured counts (`HoledFreeformFace`
> + its exactly-ONE-simply-trimmed-wall rule). `src/native/boolean/boolean_readmit.h` (additive,
> header-only; the frozen `recogniseFreeformSolid` / `freeformWall` and the five stage verbs stay
> **BYTE-UNCHANGED**) makes it re-admissible:
> - `recogniseFreeformSolidReadmit` — a strictly MORE-PERMISSIVE B1 sibling that ADMITS a holed
>   annulus wall (the seam is a legitimate trim loop, not a defect) and ANY number of freeform
>   walls, reusing `fodetail::{classifyFaceRole, faceOutwardNormal, foldAabb}` byte-identically and
>   accepting EITHER the topology edge-incidence audit OR a mesh-level `isWatertight` (the boolean
>   output shares its seam geometrically, not by vertex-identity — the same exactly-two-incidences
>   predicate the binary boolean already self-verified). **The operand is now ADMISSIBLE** — the
>   first measured blocker is cleared.
> - `nurbsSolidBooleanReadmit` — a pristine accumulator DEFERS bit-identically to the frozen
>   `nurbsSolidBoolean` (2-operand folds **UNREGRESSED**); a REDUNDANT operand (a re-applied part
>   CONTAINED in the union, a CUT tool DISJOINT from the remaining material, an INTERSECT operand
>   CONTAINING the acc) SHORT-CIRCUITS to `acc` **EXACTLY** by a COINCIDENCE-TOLERANT membership
>   witness — no weld, no synthesised geometry, so DISAGREED=0 is structural.
>
> **Impact.** The reachable idempotent ≥3-operand folds now **WELD watertight** (χ=2, be=0) at the
> inclusion-exclusion volume within the tessellation band: `UnionN({A,B,B})` → V(A)+V(B)−lens,
> `CutN(A,{B,B})` → V(A)−lens, `IntersectN({B,A,A})` → the lens. Host
> `tests/native/test_native_boolean_readmit.cpp` (5/5) + the updated
> `test_native_nurbs_solid_boolean_nary.cpp` (12/12) prove admission, 2-operand bit-identity, the
> three short-circuit welds, and the honest-decline. `src/native` stays **OCCT-free**; **no `cc_*`
> ABI**; the binary boolean flagship suite is **unregressed** (8/8).
>
> **The SHARPENED (narrower) residual boundary.** A GENUINELY-OVERLAPPING ≥3 fold whose second
> seam lands on an ALREADY-HOLED annulus needs a **MULTI-HOLE / multi-crossing face split**
> (splitting an annulus that already carries a seam-hole by a second seam): `splitFaceSmoothTrim`
> treats the face as simply-connected and does NOT preserve the existing hole, so the survivor
> sub-faces are geometrically incomplete and the weld HONEST-DECLINES `NotWatertight` (measured
> be≈768, never leaky). This is exactly the readiness table's **UNLANDED §4 multi-crossing / re-
> entrant split** (stage 3, PARTIAL). The boundary is now a full step NARROWER than pre-BOOL-READMIT:
> the operand is ADMITTED and its seam is TRACED — only the multi-hole split remains, and when it
> lands the SAME fold extends to the general genuine-overlap ≥3 case with no change to this header.

### Summary table

| stage | readiness | one-line evidence |
|---|---|---|
| 1 Surface–surface intersection | **WORKS** | exact NURBS-cyl∩plane circle to **5.6e-16**; freeform open line **4.2e-13**; closed interior loop **1 seed → 446-pt loop 2.0e-11** (the "0 seeds" reading was a flawed empty-intersection fixture — corrected). Residual = near-tangent multi-branch moat (S4), ≈13.9% general decline |
| 2 Pcurve construction | **WORKS** (L3-a) | parameter-aligned + rational `constructPcurve` welds the iso-curve **1.1e-15**, a curved freeform seam on a plane **1.6e-16**, a rational circular seam on a rational NURBS cylinder **4.0e-16** (was 0.026 decline); a curved seam on a nonlinear bicubic honestly declines its ~2e-7 surface-truncation residual (never widened) |
| 3 Face split | **PARTIAL** | `classify` inside-test WORKS; split = convex-1-chord + closed-interior-seam only; multi-crossing + healing MISSING |
| 4 Region classification | **WORKS** | single-face In/Out + elementary set-algebra land; general point-in-NURBS-solid membership across MULTIPLE trimmed faces now LANDED (`nurbs_solid_membership.h`, exact ray-cast: `intersectCurveSurface` ∩ `classify`, tangent/on-edge → re-cast → honest `Unknown`): 945-pt grid **100% crisp-correct**, fragment vote well-defined |
| 2 Pcurve construction | **PARTIAL** | `constructPcurve` declines the iso-curve round-trip (parametrisation + non-rational fit); data model + fidelity guard land |
| 3 Face split | **PARTIAL** | `classify` inside-test WORKS; split = convex-1-chord + closed-interior-seam; **tolerant-topology healing pre-pass LANDED** (`split_healing.h`, L3-HEAL: gap-close + snap + G5 pinch-resolve + area-preservation gate + honest over-gap decline); general multi-crossing / re-entrant split MISSING |
| 4 Region classification | **PARTIAL** | single-face In/Out + elementary set-algebra land; general NURBS solid membership MISSING |
| 5 Reassembly / sew | **PARTIAL** | `pcurveFidelity` welds good / rejects drifted seam; single-transversal-seam freeform↔freeform sew WELDS (tracks S3/W, both legs); **multi-seam split+classify RESOLVED (exact tiling + per-region vote), and the annulus↔annulus inner seam-as-hole sew now WELDS watertight** (M0-WELD, `uv_triangulate.h`: the CDT hole-cull is a TOPOLOGICAL flood fill so both annuli triangulate the shared strip identically — inner-seam boundaryEdges **59→0**, volume converges to the closed-form lens, DISAGREED=0 by OCCT-agreement); residual = a small non-manifold count only at deflections finer than the working band |
| **COMPOSED boolean (Fuse/Cut/Common)** | **LANDED (BOOL-INT)** | the general two-freeform-solid orchestrator `nurbsSolidBoolean(A,B,op)` (`nurbs_solid_boolean.h`) COMPOSES all five stages (byte-unchanged); single-transversal-seam **COMMON/CUT/FUSE all weld watertight** at the closed-form volumes, converging, **DISAGREED=0 vs OCCT `BRepAlgoAPI_{Common,Cut,Fuse}`** (SIM 14/14); FUSE is the group-flip outer-envelope compose; op-algebra V(fuse)+V(common)=V(A)+V(B) holds; the multi-seam annulus↔annulus sew honest-declines with the residual map (never leaky). Host 7/7 + SIM 14/14 |

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
| **Boolean-grade general `constructPcurve`** | 2 | ✅ **LANDED (L3-a)** for the affine/rational seam family: parameter-aligned (`interpolateCurveWithParams`) + rational-capable fit at an edge-length-relative fidelity bar — iso-curve **1.1e-15**, plane freeform seam **1.6e-16**, rational NURBS-cyl circle **4.0e-16**. Residual: the curved-seam-on-a-**nonlinear** surface truncation (~2e-7), honest-declined | **~0 done / 0.25–0.5 residual** |
| **General multi-crossing / re-entrant face split + tolerant-topology healing** | 3 | the BOPAlgo-class combinatorial split + gapped-loop / pinch-point healing | **1–2** |
| ~~**General NURBS solid membership** (point-in-trimmed-NURBS-solid)~~ **✅ LANDED (L3-c)** | 4 | ~~ray-cast / winding across many trimmed NURBS faces, robust on tangencies~~ — landed in `nurbs_solid_membership.h` (exact ray-cast over the true face surfaces, tangent/on-edge re-cast → honest `Unknown`); 945-pt grid 100% crisp-correct | ~~0.5–1~~ **done** |
| **Boolean-grade general `constructPcurve`** | 2 | parameter-aligned + rational-capable fit at an edge-length-relative fidelity bar | **0.5–1** |
| **General multi-crossing / re-entrant face split** | 3 | the BOPAlgo-class combinatorial split (the gapped-loop / pinch-point tolerant-topology **healing pre-pass is now LANDED** — `boolean/split_healing.h`, L3-HEAL — so this row narrows to the combinatorial multi-crossing split alone) | **1–1.5** |
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
