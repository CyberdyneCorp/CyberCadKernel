# Proposal — moat-fuse-two-operand-freeform-boolean (MOAT M2-FUSE)

## Why

The landed freeform boolean is **single-operand**: `freeformHalfSpaceCut`
(`src/native/boolean/half_space_cut.h`) cuts ONE recognised freeform operand (the
bowl-lidded convex-quad prism) by ONE analytic **PLANAR half-space**, composing
B1 `recogniseFreeformSolid` → M1 `trace_intersection` → B2 `splitFace` → B4
analytic-face-split + section-cap → M0 self-verify. It reaches CUT (`A ∩ {x ≤ 0}`)
and COMMON (`A ∩ {x ≥ 0}`) with a SINGLE curved seam and a SINGLE planar cap. The
native-booleans spec closes with an explicit, MEASURED decline:

> "FUSE with an INFINITE half-space is ill-defined (unbounded), so the honest FUSE
> target is FUSE with a FINITE cutter … a TWO-operand boolean `A ∪ B` … the next
> enabler [is] a two-operand inter-solid intersection verb (multi-seam SSI +
> two-operand face classification/merge)."

That verb is the entire distance between the landed single-operand machinery and a
real boolean algebra. A finite second operand `B` makes FUSE / CUT / COMMON all
well-defined, but it requires the four things the single-operand path never needed:
(1) an **inter-solid seam set** — the intersection curve(s) where `B`'s faces meet
`A`'s faces — not one surface∩surface seam but a graph of them; (2) splitting **BOTH**
operands' crossed faces along that seam set (B2 splits ONE face along ONE seam today);
(3) **classifying** the resulting fragments of BOTH shells inside/outside the other
solid (`A`-out-of-`B`, `B`-out-of-`A`, `A`-in-`B`, `B`-in-`A`); (4) **welding** the
op-specific survivor set from BOTH operands along their shared seam edges (FUSE =
`A`-out + `B`-out; CUT = `A`-out + `B`-in reversed; COMMON = `A`-in + `B`-in).

This change lands the **FIRST two-operand freeform boolean** at the SIMPLEST reachable
configuration: the bowl-lidded convex-quad prism `A` FUSED (and, by the same machinery,
CUT / COMMON) with a **finite axis-aligned analytic box** `B` positioned so that exactly
ONE of `B`'s planar faces slices `A`'s freeform bowl wall in a **single clean transversal
curved seam** — reusing the M1 plane∩Bézier trace UNCHANGED — while `B`'s remaining faces
meet `A`'s PLANAR faces along ordinary **plane∩plane straight segments**. This collapses
the multi-seam SSI to ONE reused curved seam plus analytic straight seams, keeping the
inter-solid intersection graph small and closed, so the two-operand verb is reachable
WITHOUT any new curved-SSI capability.

A general two-operand freeform boolean (arbitrary poses, multiple curved seams, branch
points, non-transversal contact) is a multi-person-year capability. This change is the
FIRST bounded slice, not the general verb. **OCCT remains the oracle and the fallback**:
the assembled result is admitted ONLY if it self-verifies WATERTIGHT and matches an
INDEPENDENT closed-form union/difference volume; otherwise the boolean DECLINES → OCCT.
If the general single-curved-seam FUSE is not robustly weldable this wave (the known
shared-curved-edge deflection fragility), the change LANDS the provable piece — the
inter-solid seam-set trace + the two-operand split/classify verbs proven in isolation —
and returns the next SHARPENED blocker honestly. A correct decline carrying the measured
next enabler is a first-class, expected outcome; no partial, overlapping, leaky, or
wrong-volume solid is ever emitted, and no dead FUSE stub is written.

## What Changes

1. **An additive inter-solid seam-set builder** — a new OCCT-free header
   `src/native/boolean/inter_solid_seam.h` that, GIVEN two recognised operands
   (`A` = `recogniseFreeformSolid`'s bowl-lidded prism; `B` = an axis-aligned analytic
   box admitted as an all-planar solid), assembles the **intersection seam set**: the
   ONE curved seam (`B`'s cutting plane ∩ `A`'s Bézier wall, traced by the EXISTING
   `ssi::trace_intersection` / `traceWallSeam` machinery, byte-unchanged) spliced to the
   plane∩plane straight segments where `B`'s other faces cross `A`'s planar walls/bottom.
   It verifies the seam set closes into ONE simple loop per crossed face pair and returns
   a typed DECLINE (no partial seam) when it does not.
2. **A two-operand face-split driver** — extends the split step to split BOTH operands'
   crossed faces: `A`'s Bézier wall along the curved seam via B2 `splitFace` **UNCHANGED**,
   and each crossed PLANAR face (of `A` and of `B`) along its straight seam segment via the
   landed analytic-face-split verb (`half_space_cut.h` B4 primitive) **reused**, not
   rewritten. Every split is verified (exactly-two-crossings, non-tangent, non-vertex) or
   DECLINES.
3. **A two-operand shell classifier** — classifies every resulting fragment of `A` against
   `B`'s M0 boundary mesh, and every fragment of `B` against `A`'s M0 boundary mesh, using
   B3 `classifyPointInMesh` at each fragment's interior centroid **UNCHANGED**, into
   `A`-out / `A`-in / `B`-out / `B`-in. An `On`/`Unknown` verdict or a fragment whose
   membership cannot be crisply resolved DECLINES (never guessed).
4. **A per-op two-operand weld** — selects the survivor set per the op (FUSE = `A`-out +
   `B`-out; CUT = `A`-out + `B`-in reversed; COMMON = `A`-in + `B`-in), welds the shared
   seam edges bit-exactly (the `assemble.h` `VertexPool` discipline), and welds into one
   shell → `Solid`. Behind a new entry point `freeformBooleanTwoOperand(A, B, op)` in
   `src/native/boolean/two_operand.h`.
5. **A MANDATORY two-operand self-verify → OCCT fallback** — before returning, the welded
   result MUST be WATERTIGHT (every edge shared by exactly two faces) AND its enclosed
   volume MUST match the INDEPENDENT closed-form value for the op
   (`V(A ∪ B) = V(A) + V(B) − V(A ∩ B)` for FUSE, etc.) within a scale-relative deflection
   band. A result that FAILS is DISCARDED (NULL `Shape` → OCCT). No tolerance is weakened;
   a leak is never emitted.
6. **The M1 SSI extension, if any, is strictly additive.** The single reused curved seam
   needs no new SSI. IF the seam-set assembly needs an additive helper on the marcher (e.g.
   a param-box union across the box face for full-seam coverage), it is added ADDITIVELY to
   `native-ssi` with every prior seeding/marching control BYTE-FROZEN. No analytic
   `recogniseCurvedSolid`, no landed `half_space_cut` CUT/COMMON path, and no B1/B2/B3/M0
   header is modified — all are consumed byte-identical.
7. **The honest-out is preserved end-to-end.** The two-operand assembler DECLINES (NULL
   `Shape` → OCCT) whenever any verb declines OR the self-verify fails, records WHICH verb
   declined and the measured gap, and — if the full FUSE weld is not robustly reachable this
   wave — LANDS the inter-solid seam-set trace + split/classify verbs proven in isolation
   and returns the next sharpened blocker (the shared-curved-edge single-sampling M0 weld
   fix named in the landed CUT/COMMON spec, or general multi-seam graph assembly). No FUSE
   stub is shipped.

## Capabilities

### Modified Capabilities

- `native-booleans`: ADDS the FIRST **two-operand** freeform boolean — a recognised
  freeform operand FUSED / CUT / COMMONED with a FINITE analytic box across a single reused
  curved seam plus analytic straight seams — via an inter-solid seam-set builder, a
  two-operand split of BOTH operands, a two-operand shell classifier, and a per-op weld,
  gated by a mandatory watertight + independent-closed-form-volume self-verify and an OCCT
  parity gate, with the honest decline retained and the landed single-operand CUT/COMMON
  path byte-identical.
- `native-ssi`: ADDS (only if required for full-seam coverage) a strictly ADDITIVE helper
  for tracing the box-face∩freeform-wall seam over the two-operand param box, proven to
  leave every prior seeding/marching control and every existing SSI parity figure
  byte-identical.

## Impact

- `src/native/boolean/inter_solid_seam.h` — NEW OCCT-free header: the inter-solid seam-set
  builder (reuses `ssi::trace_intersection` for the curved seam and analytic plane∩plane
  for the straight seams). Cognitive complexity kept in the backend band via per-pair
  helpers.
- `src/native/boolean/two_operand.h` — NEW OCCT-free header: `freeformBooleanTwoOperand(A,
  B, op)` composing seam-set → split-both → classify-both → per-op weld → self-verify, plus
  a typed `TwoOperandDecline`. Reuses B1/B2/B3/M0 and the `half_space_cut.h` analytic-split
  + `assemble.h` weld primitives UNCHANGED.
- `src/native/boolean/freeform_operand.h` — CONSUMED unchanged (`recogniseFreeformSolid`,
  `FreeformOperand`); the all-planar box `B` is admitted by the EXISTING planar-solid
  recogniser path (`native_boolean.h`), not a new gate.
- `src/native/ssi/marching.h` / `seeding.h` — CONSUMED unchanged; touched ONLY if a
  strictly additive full-seam-coverage helper is proven necessary, with prior controls
  byte-frozen.
- `src/engine/occt` + simulator proof harness — OCCT `BRepAlgoAPI_Fuse` / `_Cut` / `_Common`
  oracle for the SIM parity gate (volume / area / watertight / topology / spatial BBOX).
  OCCT is referenced ONLY here; `src/native/**` stays OCCT-free.
- **Zero-regression discipline (mandatory).** The landed `freeformHalfSpaceCut` CUT/COMMON,
  the analytic `recogniseCurvedSolid`/`classifyPoint`, and B1/B2/B3/M0 MUST be
  byte-identical; the native-booleans and native-ssi suites MUST pass with counts unchanged
  from the pre-change baseline; `src/native/**` MUST keep ZERO OCCT includes; the `cc_*` ABI
  MUST be unchanged (this is internal boolean behaviour).
- **Out of scope (declines, documented not faked):** general two-operand poses with MULTIPLE
  curved seams or branch points; non-transversal / tangent inter-solid contact; a non-box
  finite second operand with curved faces; robust deflection-independent welding of the
  shared curved seam (the M0 shared-curved-edge single-sampling fix, named as the next
  enabler). No `cc_*` ABI change; no CyberCad app change; no OCCT linked into `src/native/**`;
  no FUSE stub.
