# native-booleans

## ADDED Requirements

### Requirement: Freeform↔analytic-half-space COMMON as the complementary keep-side of the landed CUT

The native boolean library SHALL compute the **COMMON** of the bowl-lidded convex-quad
prism operand with an analytic PLANAR half-space — `A ∩ {x ≥ 0}`, the piece on the OTHER
side of the cut plane from the landed CUT (`A ∩ {x ≤ 0}`) — by driving the landed
`freeformHalfSpaceCut` at the COMPLEMENTARY keep side (`KeepSide::Above`) with NO new
geometry verb. The COMMON path SHALL reuse the SAME B1 recognise, M1 seam trace, B2 split,
B4 analytic-face-split + section-cap, and M0 self-verify, and SHALL flip only the
keep-side selection (which B2 sub-face and which analytic sub/whole faces survive) and the
section-cap outward normal — both already implemented in the landed verb. The consumed
`half_space_cut.h`, B1/B2/B3/M0/M1, and the analytic `recogniseCurvedSolid`/`classifyPoint`
SHALL remain BYTE-IDENTICAL; no `cc_*` ABI surface SHALL be added; `src/native/**` SHALL
keep zero OCCT includes; no tolerance SHALL be weakened.

The COMMON result SHALL be gated on the **complementary closed-form volume**
`∫∫_{Q ∩ {x ≥ 0}} (H0 + a·(x² + y²)) dA`: when the mandatory self-verify admits a result,
that result SHALL be WATERTIGHT (every edge shared by exactly two faces) and its enclosed
volume SHALL equal the complementary closed-form value within the scale-relative deflection
band. When the shared-edge weld does not coincide at the chosen deflection, the self-verify
SHALL DECLINE (NULL `Shape` → OCCT); a partial, overlapping, or leaky solid SHALL NEVER be
emitted.

#### Scenario: The bowl-lidded prism COMMON a planar half-space welds watertight at the complementary closed-form volume (host, no OCCT)

- GIVEN the bowl-lidded convex-quad prism operand and the cutting plane `x = 0`, on a host build with NO OCCT linked, and a deflection at which the shared-curved-edge weld coincides for both keep-sides
- WHEN `freeformHalfSpaceCut(A, P, KeepSide::Above)` composes B1 → M1 → B2 → B4 → M0 self-verify
- THEN it SHALL return a watertight `Solid` whose enclosed volume equals `∫∫_{Q ∩ {x ≥ 0}} (H0 + a·(x² + y²)) dA` within the scale-relative deflection band — the SECOND boolean operator reached with no new geometry verb, only the complementary keep-side selection and section-cap orientation

#### Scenario: COMMON reuses the landed verbs byte-identically with no new ABI (host)

- GIVEN the COMMON path and the consumed `half_space_cut.h`, B1/B2/B3/M0/M1, and analytic `recogniseCurvedSolid`/`classifyPoint` headers before and after this change
- WHEN COMMON is exercised and the headers are compared against the M2-assembly baseline
- THEN COMMON SHALL be `freeformHalfSpaceCut(A, P, KeepSide::Above)` with the consumed headers byte-identical, zero OCCT includes under `src/native/**`, no `cc_*` signature or POD change, and no weakened tolerance

### Requirement: CUT/COMMON partition-closure host oracle

The native boolean library SHALL provide a mesh-free, OCCT-free closed-form oracle that the
CUT keep-side and the COMMON keep-side PARTITION the operand: since the cut plane `x = 0`
partitions the quad `Q` into `Q ∩ {x ≤ 0}` and `Q ∩ {x ≥ 0}` (overlapping only on the
measure-zero chord) under the identical integrand `H0 + a·(x² + y²)`, the closed-form
identity `V(A ∩ {x ≤ 0}) + V(A ∩ {x ≥ 0}) = V(A)` SHALL hold to machine precision. The
complementary clip `clipXge0` SHALL be the Sutherland–Hodgman complement of the landed
`clipXle0`, and its polygon area plus the keep-side clip's area SHALL equal `area(Q)`. When
BOTH complementary keep-side solids weld watertight at a deflection, their MESH enclosed
volumes SHALL also sum to the operand volume within the doubled deflection band. This oracle
SHALL depend on NO mesher and NO OCCT and SHALL be the primary COMMON correctness check.

#### Scenario: The closed-form partition identity closes to machine precision (host, no OCCT)

- GIVEN the closed-form CUT volume `V_below = ∫∫_{Q ∩ {x ≤ 0}}` and COMMON volume `V_above = ∫∫_{Q ∩ {x ≥ 0}}`, and the full operand volume `V_full = ∫∫_Q`, all evaluated by the exact per-triangle quadratic-moment oracle with NO OCCT
- WHEN the partition identity is checked
- THEN `|V_below + V_above − V_full|` SHALL be `≤ 1e-12`, `0 < V_above < V_full` SHALL hold, and the areas of `clipXle0(Q)` and `clipXge0(Q)` SHALL sum to `area(Q)` — CUT and COMMON exactly partition the operand independent of any mesh

#### Scenario: The two complementary keep-side solids partition the operand at the mesh level (host)

- GIVEN a deflection at which BOTH `freeformHalfSpaceCut(A, P, KeepSide::Below)` and `KeepSide::Above` weld watertight, on a host build with NO OCCT
- WHEN both solids are meshed and their enclosed volumes summed
- THEN each SHALL be watertight and `vol(CUT) + vol(COMMON)` SHALL equal `V_full` within the doubled scale-relative deflection band — the mesh-level partition confirms the closed-form identity

### Requirement: The self-verify never emits a leak across deflections, documenting the shared-curved-edge weld fragility

The native boolean library's mandatory self-verify SHALL emit ONLY a watertight solid or a
NULL `Shape` — NEVER a partial, overlapping, or leaky solid — for BOTH keep-sides at EVERY
deflection. Over a deflection sweep, for both `KeepSide::Below` (CUT) and `KeepSide::Above`
(COMMON), every returned result SHALL satisfy `isNull()` OR mesh-`isWatertight()`. The
watertightness of the welded boolean SHALL be acknowledged as DEFLECTION-FRAGILE for BOTH
operators: the M0 mesher position-welds coincident boundaries by vertex identity, while a
shared CURVED edge (the degree-2 Bézier seam and the bowl's quad edges) is tessellated
INDEPENDENTLY on each incident face, so the two samplings — and the weld — coincide only at
deflections where their sample parameters align; at the scattered fragile deflections the
self-verify DECLINES rather than emit a hairline crack. This fragility SHALL be recorded as
a MEASURED, first-class fact (not tolerance-fudged and not hidden), with the enabling fix —
**shared-curved-edge single-sampling in M0** (tessellate each shared edge ONCE and reuse it
on both incident faces) — recorded as the next enabler for robust, deflection-independent
watertightness of CUT and COMMON alike. This change SHALL NOT modify M0.

#### Scenario: No leaky solid is emitted at any deflection for either keep-side (host, no OCCT)

- GIVEN the bowl-lidded prism operand and the cutting plane `x = 0`, swept over deflections `{0.03, 0.02, 0.01, 0.008, 0.005, 0.004}` for both `KeepSide::Below` and `KeepSide::Above`, on a host build with NO OCCT
- WHEN each `freeformHalfSpaceCut` result is meshed and audited
- THEN EVERY result SHALL be `isNull()` OR watertight (a non-watertight solid SHALL NEVER be returned), and the deflections at which each keep-side declines SHALL be attributable to the shared-curved-edge weld mismatch — the honest-decline discipline holds at 100% of the sweep

#### Scenario: The shared-curved-edge weld fragility is documented, not fudged (host)

- GIVEN the measured deflection sweep showing BOTH keep-sides oscillate between a watertight weld and a `NotWatertight` decline (the CUT is not special)
- WHEN the change is inspected for how COMMON is landed
- THEN the COMMON gate SHALL rely on the mesh-free partition identity plus a watertight case at a both-sides-weld deflection with the UNWEAKENED 2% volume band, NO tolerance SHALL be loosened to force a weld, and the shared-curved-edge single-sampling M0 fix SHALL be recorded as the next enabler — the fragility SHALL be a tested, documented fact

### Requirement: FUSE with a finite cutter is declined as the next breadth blocker

This change SHALL DECLINE FUSE with a finite cutter and record it as the next breadth
blocker. FUSE with an INFINITE half-space is ill-defined (unbounded), so the honest FUSE
target is FUSE with a FINITE cutter (a box or a second prism), which is a TWO-operand
boolean `A ∪ B`. The change SHALL record why FUSE is NOT robustly reachable from the landed
single-operand machinery: B1 admits ONE operand and there is no verb to intersect two
solids' boundaries; M1 traces ONE surface∩surface seam whereas a box FUSE needs several
plane∩(bowl+walls) seams assembled into a consistent intersection graph; B2 splits ONE face
along ONE seam whereas FUSE must split BOTH operands' crossed faces along MULTIPLE seams;
and B4 caps ONE planar section whereas FUSE must CLASSIFY each operand's faces inside/outside
the other and WELD the surviving faces from BOTH operands along shared intersection curves —
a two-operand face classification + merge verb absent from the M2 substrate. FUSE SHALL be
DECLINED as the next breadth blocker with the next enabler recorded (a two-operand
inter-solid intersection verb: multi-seam SSI + two-operand face classification/merge). NO
partial or stubbed FUSE path SHALL be written.

#### Scenario: FUSE is recorded as the next breadth blocker, not stubbed (host)

- GIVEN the landed single-operand / single-plane / single-seam / single-cap machinery (B1/M1/B2/B4) and the FUSE-with-a-finite-cutter target
- WHEN the change is inspected for FUSE handling
- THEN it SHALL contain NO FUSE code path (stubbed or otherwise), SHALL record FUSE as a TWO-operand boolean needing a two-operand inter-solid intersection verb (multi-seam SSI + two-operand face classification/merge) that the M2 substrate does not supply, and SHALL name that verb — plus the shared-curved-edge single-sampling M0 fix — as the measured next enablers, an honest decline that is a first-class outcome
