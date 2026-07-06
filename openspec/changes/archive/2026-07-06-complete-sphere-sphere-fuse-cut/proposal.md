## Why

`SSI-ROADMAP.md` S5 is **the payoff** — general curved booleans driven by the S3
`TraceSet`. The archived `add-native-ssi-curved-boolean-wider` (S5-b + S5-c) landed the
transversal sphere∩sphere **COMMON** natively: `buildLensCommon` welds the two
inside-the-other spherical caps along the ONE traced seam circle, verified vs OCCT
`BRepAlgoAPI_Common` (watertight, correct volume). But that change shipped **COMMON
only** for the sphere pair — its own honest-scope note (Non-Goals) explicitly defers
sphere∩sphere fuse / cut to a follow-on:

> **sphere∩sphere fuse / cut** — S5-c ships COMMON only (the lens has the clean
> closed-form + the simplest two-cap weld); sphere fuse/cut (outer-cap union with the
> re-trimmed remainder of each sphere) is a follow-on. DECLINE → OCCT.

So today the sphere∩sphere family is **1/3 native** (COMMON), while `ssi_boolean_solid`
dispatches `Op::Fuse` / `Op::Cut` for the sphere-lens case to the through-drill
`buildFuse` / `buildCut`, which resolve their drill roles on the single-seam sphere pair
and return NULL → OCCT (an honest, verified fallback — but a fallback).

The GAP is again the **assembler**, not recognition or tracing: `recogniseCurvedSolid`
already folds both spheres, S3 already traces the ONE closed transversal seam cleanly
(`nearTangentGaps == 0`), and `buildLensCommon` already welds two caps along it. The
same single-seam geometry, with a **different cap selection**, is exactly FUSE and CUT.
This change completes the set: sphere∩sphere becomes **3/3 native**.

## What Changes

The geometry (two overlapping spheres A, B; ONE seam circle C; C splits each sphere into
an INNER cap — apex nearest the other centre — and an OUTER cap — apex at the far pole):

- **COMMON** (already native, `buildLensCommon`) = inner-cap-of-A + inner-cap-of-B,
  sharing C (the lens). Volume `V(lens) = V_cap(A) + V_cap(B)`.
- **FUSE** `A ∪ B` = **OUTER**-cap-of-A + **OUTER**-cap-of-B, sharing C (the peanut/
  dumbbell outer shell). Volume `V(A) + V(B) − V(lens)`.
- **CUT** `A − B` = **OUTER**-cap-of-A + **INNER**-cap-of-B **REVERSED** (the inner cap
  of B now bounds the scooped cavity, normal flipped inward), sharing C. Volume
  `V(A) − V(lens)`.

Concretely:

- **Generalise `appendSphereCap`** to build EITHER cap: add a signed apex-direction
  selector (inner apex = `centre + R·unit(centre→other)`, the current behaviour; outer
  apex = `centre − R·unit(centre→other)`, the far pole) and a `reversed` flag that flips
  the facet's outward reference (radial-inward) so a cap can bound a cavity. No
  duplication — the ring/slerp/facet loop is unchanged; only the apex direction and the
  `pushPlanarTri` reference sign vary. The COMMON caller keeps building the inner,
  outward cap byte-identically.
- **Add `buildLensFuse(A, B, seams)`** — mirror `buildLensCommon`: same
  `decimateSeam` + `seamNodeTarget` shared seam, two `appendSphereCap` calls with the
  **OUTER** apex for both spheres (outward normal), welded on the shared pooled seam
  nodes. Survival rule: keep each sphere's cap that is OUTSIDE the other solid (the far
  pole classifies outside the other sphere for a transversal lens). A far-pole sample
  robustly ON/inside the other sphere (containment / tangent) → NULL → OCCT.
- **Add `buildLensCut(A, B, seams)`** — mirror `buildLensCommon`: the **OUTER** cap of A
  (outward normal) + the **INNER** cap of B emitted **REVERSED** (inward normal) so it
  bounds the scooped cavity, both on the SAME shared decimated seam. `A` is the minuend;
  CUT is orientation-sensitive, so the operand order is honoured. A degenerate/tangent
  apex → NULL → OCCT.
- **Driver dispatch** (`ssi_boolean_solid`): for the recognised single-seam sphere∩sphere
  lens (both operands Sphere, ONE closed seam), dispatch `Op::Fuse` → `buildLensFuse` and
  `Op::Cut` → `buildLensCut`, mirroring the existing `Op::Common` → `buildLensCommon`
  dispatch. The through-drill `buildFuse` / `buildCut` are TRIED FIRST (they resolve
  their two-seam drill roles and return NULL for the single-seam sphere pair), so the
  sphere-lens builders take over exactly on that NULL — no regression to the through-drill
  S5-b path.
- **Engine self-verify — NO new oracle, NO sign change needed.** The existing generic
  set-algebra guard (`booleanResultVerified`, `native_engine.cpp`) already computes
  `expected = va + vb − vc` (fuse, `op == 0`) and `va − vc` (cut, `op == 1`), where `vc`
  is the native COMMON = `buildLensCommon`. A FUSE therefore GROWS (`Vr > max(VA,VB)`)
  and a CUT SHRINKS (`Vr < VA`), verified against the SAME native lens COMMON the S5-c
  change shipped. A mis-welded / wrong-volume candidate is DISCARDED → OCCT, never faked.

## Capabilities

### New Capabilities
<!-- none — this change COMPLETES the living native-booleans sphere∩sphere op-set
(COMMON already native) by adding native FUSE + CUT, and re-affirms the native-ssi
single-closed-seam sphere∩sphere TraceSet contract as the input to all three ops. -->

### Modified Capabilities
- `native-booleans`: complete the transversal sphere∩sphere op-set — add native
  **Fuse** (two OUTER caps) and **Cut** (OUTER-A + REVERSED INNER-B) alongside the
  already-native **Common** (two inner caps), all welded along the ONE shared decimated
  seam circle with the radial-ring planar-facet discipline, guarded by the engine's
  EXISTING generic set-algebra self-verify (fuse grows, cut shrinks, vs the native lens
  COMMON) → OCCT fallback. Generalise `appendSphereCap` for inner/outer apex + a reversed
  normal; the COMMON path stays byte-identical. Other curved-curved pairs unchanged;
  tangent/degenerate spheres DECLINE → OCCT. No `cc_*` change.
- `native-ssi`: re-affirm the S3 single-closed-seam sphere∩sphere `TraceSet`
  (`nearTangentGaps == 0`, one `Closed` WLine with `(u1,v1,u2,v2)` per node) as the
  consumed input contract for ALL THREE sphere-lens ops — the SAME seam splits each
  sphere into inner/outer caps; COMMON/FUSE/CUT differ only in which caps survive and
  their orientation.

## Impact

- **Scope (bounded completion).** This change ONLY completes the sphere∩sphere op-set
  (COMMON already native; add FUSE + CUT). It is the same single-seam geometry with a
  different cap selection — NOT an open-ended new family. All other pairs (through-drill
  cyl∩cyl, Steinmetz, sphere/cone∩box, cyl∩cone, cyl∩sphere, cone∩cone, oblique/multi-tube
  cyl∩cyl, freeform) are UNCHANGED and keep their existing native/decline behaviour.
- **ABI**: none. Invoked behind the existing `cc_boolean` op codes; no `cc_*` entry
  point, signature, or POD struct changes. Additive only.
- **Build**: extends `src/native/boolean/ssi_boolean.cpp` only — generalise
  `appendSphereCap`, add `buildLensFuse` + `buildLensCut` (mirroring `buildLensCommon`),
  extend the dispatch. Reuses `decimateSeam`, `seamNodeTarget`, `slerpDir`,
  `pushPlanarTri`, `classifyPoint`, `VertexPool`, `recogniseCurvedSolid` — all already
  present. No new files. Compiled under `CYBERCAD_HAS_NUMSCI` (the sphere pair uses the
  S3 trace). `src/native/**` stays OCCT-free. No change to `src/native/tessellate`, the
  planar BSP-CSG, the analytic `curved.h`, the through-drill `buildCommon/Fuse/Cut`, or
  the S5-c `buildLensCommon`.
- **Verification (two gates, dual oracle, no weakened tolerance).**
  - **Host (no OCCT)** — the analytic spherical-cap volume `V_cap = π h² (3R − h) / 3`,
    with `lens = V_cap(A) + V_cap(B)`, `FUSE = V(A) + V(B) − lens`,
    `CUT(A,B) = V(A) − lens`. The host suite checks the assembled shell is watertight
    (`boundaryEdgeCount == 0`, every edge shared by exactly two faces), the enclosed
    volume matches the closed form within the deflection-sized band, and both caps' seam
    nodes lie on both spheres ≤ tol. Tangent/degenerate fixtures return NULL (deferred).
  - **Sim native-vs-OCCT** — `scripts/run-sim-native-ssi-curved-boolean.sh` already
    exercises the two sphere-lens pairs (equal + unequal radii) across
    `{Fuse, Cut, Common}`; today FUSE/CUT are honest fall-backs. After this change they
    become NATIVE passes vs `BRepAlgoAPI_{Fuse,Cut}` (volume + surface area + watertight
    closed shell + valid shape). The harness auto-detects native-vs-fall-back, so
    **native-pass rises 6 → 8** (the two sphere-lens pairs' FUSE + CUT resolving native;
    the harness counts the equal-radius pair's FUSE + CUT — the unequal pair remains an
    honest fall-back until its self-verify passes, and this change does not weaken its
    tolerance to force it).
- **Roadmap**: completes `SSI-ROADMAP.md` S5 sphere∩sphere to 3/3 native. Other S5
  families and the S4 near-tangent moat remain the tail.
- **Risk (honest)**: a FUSE far-pole sample or a CUT reversed-cap orientation that is
  wrong yields a self-intersecting or wrong-signed shell — caught by the engine's
  mandatory watertight + correct-volume (fuse grows / cut shrinks vs the native lens
  COMMON) self-verify, which DISCARDS the candidate → OCCT. A hairline seam gap on a
  high-curvature lens is caught by the watertight check at the mesher deflection ladder →
  discard → OCCT. If either op cannot be built watertight with the correct volume, the
  builder returns NULL → OCCT and the measured gap is reported. No case is faked,
  stubbed, or tolerance-weakened to pass.
