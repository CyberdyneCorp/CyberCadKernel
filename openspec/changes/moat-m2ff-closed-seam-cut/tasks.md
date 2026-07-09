# Tasks — moat-m2ff-closed-seam-cut (MOAT M2 freeform↔freeform CLOSED-SEAM CUT / COMMON)

Order: baseline capture → two-bowl-cup fixture + closed-form oracle → the composed verb
→ host analytic gate → sim native-vs-OCCT gate → byte-freeze + zero-regression proof →
docs. All new native code stays OCCT-free and host-buildable (`clang++ -std=c++20`),
namespace `cybercad::native::boolean`. No `cc_*` ABI change. Strictly ADDITIVE:
`smooth_trim_split.h`, B2 `splitFace`, `freeform_operand.h`, `freeform_membership.h`,
M0/M1 and every landed boolean header stay BYTE-IDENTICAL. No tolerance weakened; a
correct decline is first-class.

## STATUS

LANDED (BOTH gates) as an ENABLER + a sharpened MEASURED decline. The composed verb
`freeformFreeformClosedSeamCut` runs recognise[B1] → trace[M1] → split[B2 smooth-trim] →
classify[B3] → weld[M0] for two coaxial freeform bowl-cups over a shared CLOSED curved
seam, and HONEST-DECLINES CUT and COMMON to NULL (never a leaky solid). MEASURED root
cause of the decline: the two-CURVED-side closed-seam weld is gated on the byte-frozen M0
tessellator — the M0w seam pin welds a curved sub-face to a FLAT chord the other side
already sits on (curved↔flat welds watertight), but two independently-tessellated CURVED
sub-faces sharing a closed seam do NOT weld (measured: the lens of two disk caps leaves
open edges AT the seam that persist across seam decimation and deflection; a shared-edge
two-pcurve construction did not close either). This is the SAME class of tessellator work
the roadmap says CANNOT be landed byte-identically as a single additive slice — so the
verb correctly returns NULL → OCCT. Host gate 8/8; sim native-vs-OCCT gate 12/12.

## 1. Baseline + fixture

- [x] 1.1 Capture the byte-frozen substrate baseline: `test_native_curved_wall_cut`
  (13/13), `test_native_two_operand_freeform_boolean` (6/6) GREEN before the change.
- [x] 1.2 Add `tests/native/freeform_freeform_cut_fixture.h`: two coaxial paraboloid
  bowl-cups (UP bowl-cup `A` + DOWN dome-cup `B`, degree-2 separable Bézier walls + flat
  lids), the real M1 seam (`ssi::trace_intersection` of the two Béziers → a CLOSED circle
  ρ = √(H/2a) at z = H/2), and closed-form oracles `volA = π·a·R⁴/2`,
  `volCommon = π·H²/(4a)`, `volCut = volA − volCommon`.

## 2. The composed verb

- [x] 2.1 `freeform_freeform_cut.h`: `FfOp`, `FfCutDecline` + `ffCutDeclineName`, and
  `freeformFreeformClosedSeamCut`, composing B1 → M1 trace (two `makeBezierAdapter`) → B2
  smooth-trim split of BOTH walls (B's seam re-keyed `(u2,v2)→(u1,v1)`) → B3 membership
  survivor selection → M0 weld + mandatory watertight + op-volume self-verify → NULL on
  any decline.
- [x] 2.2 Keep the driver + helpers within the backend cognitive-complexity band.

## 3. Host GATE (a) — analytic, no OCCT

- [x] 3.1 Both operands B1-admit with one freeform wall + one analytic lid.
- [x] 3.2 The shared seam is a CLOSED curved circle radius ρ on BOTH walls' (u,v) to
  ~1e-13, on both surfaces to the trace residual, at z*.
- [x] 3.3 The closed-form partition is consistent (`volCut + volCommon = volA`), the lens
  is substantial.
- [x] 3.4 The pipeline B2-splits BOTH curved walls (each disk area = π·ρ²).
- [x] 3.5 CUT and COMMON HONEST-DECLINE to NULL (`NotWatertight`/`ClassifyAmbiguous`)
  across a deflection sweep — never a leaky solid.
- [x] 3.6 A non-operand declines `NotAdmittedA`; a non-intersecting operand declines
  `SeamUnusable`.

## 4. Sim GATE (b) — native-vs-OCCT

- [x] 4.1 `tests/sim/native_freeform_freeform_cut_parity.mm` + run script
  `scripts/run-sim-native-freeform-freeform-cut.sh`.
- [x] 4.2 OCCT reconstructs the SAME two bowl-cups (Geom_BezierSurface bowl + a planar lid
  on the shared rim edges, sewn, interior-classified outward).
- [x] 4.3 Each native seam node lies ON BOTH OCCT Bézier surfaces (BRepExtrema ≤ 1e-4,
  measured ~1e-14); OCCT `Cut` / `Common` match the closed-form volumes to ≤ 2% (Common
  EXACT); the native verb DECLINES CUT and COMMON to NULL (correct honest fallthrough); a
  non-intersecting operand → native `SeamUnusable` + OCCT no-op. 12/12 PASS.

## 5. Byte-freeze + zero-regression proof

- [x] 5.1 `include/` diff EMPTY (no `cc_*` change); `git diff src/native` EMPTY (only the
  NEW `freeform_freeform_cut.h`); `smooth_trim_split.h` / `face_split.h` /
  `freeform_operand.h` / `freeform_membership.h` / the tessellator / M1 BYTE-IDENTICAL.
- [x] 5.2 `src/native` has 0 new OCCT includes.
- [x] 5.3 Full host `ctest` GREEN with zero regression.

## 6. Docs

- [x] 6.1 Update `openspec/MOAT-ROADMAP.md` M2 status with the landed freeform↔freeform
  enabler, both gate numbers, and the sharpened next blocker (the two-curved-side
  closed-seam weld = a byte-frozen M0 tessellator change, out of the additive boolean-layer
  scope).
