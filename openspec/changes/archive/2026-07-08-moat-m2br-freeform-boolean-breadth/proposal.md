# Proposal — moat-m2br-freeform-boolean-breadth (MOAT M2-breadth: COMMON, + FUSE diagnosis)

## Why

The M2-assembly slice (`moat-m2asm-first-freeform-boolean`) landed the FIRST end-to-end
freeform boolean: a bowl-lidded convex-quad prism **CUT** by ONE analytic planar
half-space — `A ∩ {x ≤ 0}` — composed `recognise[B1] → trace[M1] → split[B2] →
analytic-face-split + section-cap[B4] → self-verify[M0]`, host-gated at its closed-form
polynomial volume. That is ONE boolean OPERATOR (difference) on ONE operand family.

The breadth question is: does the SAME landed machinery reach the OTHER boolean
operators? The CUT verb `freeformHalfSpaceCut` already carries a `KeepSide` selector and
already orients the section cap for either side, so the **complementary keep-side** —
`A ∩ {x ≥ 0}`, the piece on the OTHER side of the same cut plane — IS the **COMMON** of
the bowl-lidded prism with the planar half-space `{x ≥ 0}`. Extending the moat from CUT
to COMMON needs NO new geometry verb: the same B4 analytic-face split, the same seam,
the complementary keep-side selection and cap orientation (both already coded), gated by
the **complementary closed-form volume** — with the partition identity
`V(A ∩ {x ≤ 0}) + V(A ∩ {x ≥ 0}) = V(A)` as a second, independent, mesh-free oracle.

**Diagnosis (measured on the host, no OCCT).** Driving the landed
`freeformHalfSpaceCut` at `KeepSide::Above` produces the correct complementary solid at
the correct complementary volume, and the closed-form partition identity closes to
machine precision (measured `|V_below + V_above − V_full| = 0`). BUT the welded result's
watertightness is **deflection-fragile** — and, crucially, **so is the CUT's**. A
deflection sweep (`{0.03 … 0.002}`) shows BOTH keep-sides oscillate between a watertight
weld and a self-verify DECLINE (`NotWatertight` → NULL) at scattered deflections (CUT
declines at 0.02 and 0.004; COMMON declines at 0.02, 0.01, 0.004, 0.002; both weld at the
majority, e.g. 0.03/0.015/0.012/0.011/0.009/0.008/0.007/0.006/0.005/0.003). The landed
CUT host test passes only because it happens to sample deflection 0.01, one of the points
where the CUT weld coincides. At NO deflection is a **leaky** solid emitted: the mandatory
self-verify yields either a watertight solid or NULL, never a leak.

The root cause is NOT in the boolean operator: it is that the M0 mesher **position-welds
coincident boundaries by vertex identity**, while a shared CURVED edge (the degree-2
Bézier seam / bowl quad edges) is tessellated INDEPENDENTLY on each incident face, so the
two samplings coincide (weld) only at deflections where their parameter samples happen to
align. This shared-curved-edge weld fragility is a **pre-existing M0 property EXPOSED, not
introduced, by the breadth extension** — the CUT already carried it. It is the sharpened
next blocker.

**Outcome.** COMMON LANDS at the host gate: at a deflection where the shared-edge weld
coincides for BOTH keep-sides, the bowl-lidded prism COMMON a planar half-space assembles
watertight at its complementary closed-form volume, and CUT + COMMON partition the operand
(mesh-level) — proving the second boolean operator reuses the landed verbs. The
deflection-free partition-closure identity is the primary COMMON oracle. **FUSE is
DECLINED as the next breadth blocker** (see below), and the shared-curved-edge
single-sampling weld is recorded as the enabling M0 fix for robust (deflection-independent)
watertightness of BOTH CUT and COMMON.

## What Changes

1. **A COMMON host gate** (new host test + a small additive fixture extension — NO change
   to `half_space_cut.h`, which already supports `KeepSide::Above`):
   - The **complementary closed-form oracle**: `clipXge0` + `polyVolume` over
     `Q ∩ {x ≥ 0}`, and the **partition-closure identity** `V_below + V_above = V_full`
     checked mesh-free to machine precision.
   - **COMMON lands watertight at the complementary volume**: `freeformHalfSpaceCut(A, P,
     KeepSide::Above)` at a deflection where the shared-edge weld coincides returns a
     watertight `Solid` whose enclosed volume equals `∫∫_{Q ∩ {x ≥ 0}} (H0 + a(x²+y²)) dA`
     within the deflection band, and CUT + COMMON meshes partition `V_full`.
   - **The honest-decline discipline is TESTED across deflections**: over a deflection
     sweep, for BOTH keep-sides, every result is `isNull()` OR watertight — a leaky solid
     is NEVER emitted. The measured shared-curved-edge weld fragility is documented, not
     hidden or tolerance-fudged.
2. **A FUSE breadth DIAGNOSIS (no code)** recording why FUSE with a finite cutter (box or
   second prism) is NOT robustly reachable from the landed single-operand, single-plane,
   single-seam, single-cap machinery, and what the next enabler is.
3. **Strictly additive.** `half_space_cut.h` is consumed BYTE-IDENTICAL (the `KeepSide`
   path already exists); B1/B2/B3/M0/M1 and the analytic `recogniseCurvedSolid`/
   `classifyPoint` stay byte-identical; `src/native/**` keeps 0 OCCT includes; no `cc_*`
   ABI change. The M2-assembly change and its tests are NOT modified — this change adds its
   OWN openspec change, host test, and fixture extension.

## Capabilities

### Added Requirements

- `native-booleans`: ADDS the **freeform↔analytic-half-space COMMON (complementary
  keep-side)** — the bowl-lidded convex-quad prism COMMON a planar half-space, computed by
  the landed `freeformHalfSpaceCut` at the complementary `KeepSide` with NO new geometry
  verb, host-gated at the complementary closed-form volume, or DECLINED (NULL → OCCT) via
  the same mandatory self-verify.
- `native-booleans`: ADDS the **CUT/COMMON partition-closure host oracle** — the
  mesh-free closed-form identity `V(A ∩ {x ≤ 0}) + V(A ∩ {x ≥ 0}) = V(A)` and the
  mesh-level partition of the operand by the two complementary keep-side solids.
- `native-booleans`: ADDS the **never-emit-a-leak-across-deflections discipline** — over a
  deflection sweep, for both keep-sides, the self-verify emits only a watertight solid or
  NULL, and the shared-curved-edge (Bézier-seam) weld fragility that makes watertightness
  deflection-dependent for BOTH CUT and COMMON is DOCUMENTED with its measured next enabler
  (shared-edge single-sampling in M0), not fudged.
- `native-booleans`: ADDS the **FUSE breadth-blocker diagnosis** — FUSE with a finite
  cutter is a TWO-operand boolean requiring inter-solid SSI, multi-seam splitting, and
  two-operand face classification/merge that the landed single-operand/single-plane/
  single-seam/single-cap machinery does not supply, recorded as the next breadth enabler
  (NOT stubbed).

## Impact

- `tests/native/first_freeform_boolean_breadth_fixture.h` — NEW, OCCT-free: the
  complementary `clipXge0`/`cutVolumeAbove` oracle + the partition identity, reusing the
  M2-assembly operand fixture UNCHANGED.
- `tests/native/test_native_freeform_boolean_breadth.cpp` — NEW host GATE (a): the COMMON
  watertight-at-complementary-volume case, the partition-closure oracle, the mesh-level
  partition, and the never-leak-across-deflections sweep.
- **Consumed UNCHANGED (proven byte-identical vs the M2-assembly baseline):**
  `boolean/half_space_cut.h` (B4 + `freeformHalfSpaceCut`, `KeepSide::Above` path),
  `boolean/freeform_operand.h` (B1), `boolean/face_split.h` (B2),
  `boolean/freeform_membership.h` (B3), `tessellate/solid_mesher.h` (M0),
  `ssi/marching.h` (M1), and the analytic `ssi_boolean.h` paths.
- **Gates.** (a) HOST ANALYTIC — no OCCT: the partition-closure identity (mesh-free,
  machine precision) + COMMON watertight at the complementary closed-form volume +
  never-leak-across-deflections. (b) SIM native-vs-OCCT (`BRepAlgoAPI_Common`) is a
  DOCUMENTED follow-up (a separate simulator harness), matching how the CUT slice landed
  host-first.
- **Deferred / declined (documented, not faked):** **FUSE** with a finite cutter (the next
  breadth operator — needs a two-operand inter-solid intersection verb); **robust
  deflection-independent watertightness** for CUT and COMMON (needs shared-curved-edge
  single-sampling in M0); freeform↔freeform booleans; multi-plane/box cutters; the SIM
  `BRepAlgoAPI_Common` parity harness. No `cc_*` ABI change; no OCCT under `src/native/**`.
