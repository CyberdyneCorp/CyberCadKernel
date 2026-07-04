## Why

`SSI-ROADMAP.md` S5 is **the payoff** — general curved booleans driven by the S3
`TraceSet`. The archived change `add-native-ssi-curved-boolean` (**S5-a**) landed the
FIRST real native slice: the **through-drill cylinder∩cylinder COMMON** (unequal radii,
transversal two-loop trace), native and verified vs OCCT `BRepAlgoAPI_Common` —
watertight, ΔV = 8.1e-04. But S5-a is deliberately narrow, and its own honest scope note
lists exactly what remains:

- **Fuse / Cut on the same through-drill pair DECLINE** (`ssi_boolean_solid` returns NULL
  for `Op::Fuse` / `Op::Cut`) — the outside-wall fragments plus the operands' caps
  re-trimmed by the seam were not yet a robust watertight weld, so both ops fall through
  to OCCT. The COMMON tube band + two drill-mouth caps already exist and weld watertight;
  fuse/cut reuse the SAME seam and the SAME planar-facet weld discipline on the
  complementary fragment selection.
- **sphere∩sphere COMMON DECLINES.** `recogniseCurvedSolid` already recognises a sphere
  solid and builds its `SurfaceAdapter`, and S3 traces the transversal sphere∩sphere
  seam (ONE closed circle) cleanly. But `buildCommon` is hard-specialised to the
  through-drill topology — it REQUIRES exactly **two** seams with one operand full-circle
  on both (the tube). A sphere∩sphere lens has **one** seam and no tube, so it hits the
  `seams.size() != 2` guard and returns NULL.

The GAP in both cases is the **assembler**, not recognition or tracing — both already
work. This change extends S5-a with two additive assembler slices, ON TOP of the shipped
S5-a code, each verified vs OCCT before it ships and each declining honestly (NULL →
OCCT) outside its verified envelope.

## What Changes

- **(S5-b) Native Fuse + Cut for the through-drill cylinder∩cylinder pair.** Extend
  `ssi_boolean_solid`'s `switch(op)` so `Op::Fuse` and `Op::Cut` no longer blanket-DECLINE
  for the two-seam through-drill topology `buildCommon` already handles. Reuse the SAME
  two full-circle rim seams and the SAME planar-facet weld discipline, selecting the
  complementary fragments per the op's set algebra:
  - **Cut `A − B`** (drill the thin tube out of the fat cylinder): keep the fat wall
    OUTSIDE the thin tube + the fat solid's own caps re-trimmed to exclude the drilled
    region + the thin TUBE BAND reversed (the tunnel wall, now an inward boundary of the
    result). The seam is shared, so the tunnel wall welds to the re-trimmed mouths.
  - **Fuse `A ∪ B`** (weld the two cylinders into one body): keep the fat wall OUTSIDE
    the thin tube + the thin wall OUTSIDE the fat solid + each operand's caps, dropping
    the two drill-mouth patches that are now interior. The shared seam welds the two
    outer walls.
  Both build only the fragments whose interior sample passes the existing `classifyPoint`
  test for the op's survival rule (the SAME rule as planar `booleanPolygons`), and both
  emit PLANAR-TRIANGLE facets at any face that shares the seam with a differently
  tessellated neighbour (the S5-a watertight lesson). Anything that does not assemble a
  clean watertight candidate returns NULL → OCCT, reported.
- **(S5-c) Native COMMON for the transversal sphere∩sphere pair via a NEW single-seam
  / two-cap assembler.** Add a `buildLensCommon` path taken when the trace is ONE closed
  seam and BOTH operands are spheres (the through-drill `buildCommon` stays untouched and
  keeps owning the two-seam topology). The sphere∩sphere COMMON (a symmetric or asymmetric
  lens) is bounded by **two spherical caps**: the cap of sphere A that lies inside sphere
  B, and the cap of sphere B that lies inside sphere A, meeting along the ONE seam circle.
  Each cap is emitted with the SAME radial-ring planar-facet discipline `appendMouthCap`
  already uses (fan from the cap pole out to the shared seam nodes, outer ring = the EXACT
  traced seam nodes from the shared `VertexPool`), so the two caps weld watertight along
  the single seam. Cap survival is the COMMON rule (each cap is the piece of one sphere
  INSIDE the other, `classifyPoint == inside`); a pole sample landing ON the other sphere
  (tangent / coincident) aborts → NULL → OCCT.
- **Engine self-verify — no new oracle needed.** S5-b Fuse/Cut are already covered by the
  ENGINE's EXISTING generic set-algebra guard (`native_engine.cpp booleanResultVerified`):
  it computes `Vr ≈ |A|+|B|−|A∩B|` (fuse) / `|A|−|A∩B|` (cut) using the native COMMON and
  DISCARDS a mismatch → OCCT. S5-c sphere∩sphere COMMON is covered by the same guard's
  common branch (`Vr ≈ |A∩B|`), with the closed-form lens volume as the host analytic
  oracle. No `ssiCurvedBooleanVerified`-style special oracle is added; the Steinmetz
  branch is untouched.
- **Honest scope.** ONLY what verifies vs OCCT ships. S5-b covers the through-drill
  cyl∩cyl fuse/cut (the topology S5-a's `buildCommon` already recognises); other cyl∩cyl
  configurations, oblique piercings, and multi-tube cases still DECLINE. S5-c covers the
  transversal sphere∩sphere COMMON (one clean seam, both spheres, poles strictly inside);
  sphere∩sphere fuse/cut, tangent/coincident spheres (`nearTangentGaps > 0` or ON-band
  pole), unequal-topology and other curved-curved families (cyl∩cone, cyl∩sphere,
  sphere∩box) still DECLINE → OCCT. Near-tangent stays the S4 boundary. Nothing is faked,
  hand-tuned, or shipped unverified.

**No `cc_*` ABI change.** Invoked behind the existing `cc_boolean` op codes through the
same `boolean_solid` entry. `src/native/**` stays OCCT-free; the new assembler paths are
compiled under `CYBERCAD_HAS_NUMSCI` (they consume the S3 tracer), exactly like S5-a.
Additive only — the S5-a `buildCommon` path and its tests are unchanged.

## Capabilities

### New Capabilities
<!-- none — this change EXTENDS the living native-booleans capability (S5-a's
SSI-curve-driven curved boolean) with the through-drill fuse/cut ops and a single-seam
sphere∩sphere COMMON assembler, and EXTENDS native-ssi by declaring the single-closed-seam
TraceSet as a consumed S5-c input contract. -->

### Modified Capabilities
- `native-booleans`: extend the S5-a SSI-curve-driven curved boolean with (S5-b) native
  **Fuse** and **Cut** for the through-drill cylinder∩cylinder pair — the complementary
  fragment selection over the SAME two rim seams and planar-facet weld — and (S5-c) a NEW
  single-seam / two-cap assembler that computes native **COMMON** for the transversal
  sphere∩sphere lens. The engine's EXISTING generic set-algebra self-verify guards both
  (fuse/cut via inclusion–exclusion over the native COMMON; the sphere lens via the
  common-volume check with the closed-form lens as the host oracle) and DISCARDS a bad
  candidate → OCCT. Out-of-envelope configurations DECLINE → OCCT, reported not faked. No
  `cc_*` change.
- `native-ssi`: declare the single **closed** transversal `WLine` (`nearTangentGaps == 0`,
  `status == Closed`) as the consumed input contract for the S5-c sphere∩sphere COMMON —
  its per-node `(u1,v1,u2,v2)` tracks the seam circle on both spheres and its shared 3D
  nodes are the seam vertices both caps weld on. `nearTangentGaps > 0` (tangent /
  coincident spheres) remains the honest S4 boundary the path declines on.

## Impact

- **ABI**: none. Invoked behind the existing `cc_boolean` op codes; no `cc_*` entry
  point, signature, or POD struct change. Additive only.
- **Build**: edits `src/native/boolean/ssi_boolean.cpp` (add `buildCut` / `buildFuse` for
  the through-drill topology; add `buildLensCommon` + a single-seam sphere dispatch in
  `ssi_boolean_solid`); no new files strictly required (the seam/weld helpers —
  `appendTubeBand`, `appendMouthCap`, `tubeTriFace`, `seamWire`, `VertexPool` — are
  reused). Compiled under `CYBERCAD_HAS_NUMSCI`. `src/native/**` stays OCCT-free. NO
  change to `src/native/tessellate` (the S5-a lesson: a tessellator change that fixes one
  case breaks others — every watertight fix is assembler-side).
- **Verification**: two gates. **Host (no OCCT)** — extend
  `tests/native/test_native_ssi_curved_boolean.cpp`: (S5-b) the through-drill CUT volume
  equals `vol(fat) − vol(common)` and FUSE equals `vol(fat)+vol(thin)−vol(common)` (the
  S5-a-pinned COMMON value) within the deflection band, each watertight; (S5-c) the
  sphere∩sphere COMMON volume equals the closed-form lens `V = π(rA+rB−d)²(d²+2d·rB−3rB²+
  2d·rA+6rA·rB−3rA²)/(12d)` (equal radii: two caps of height `h = r − d/2`, cap volume
  `π h²(3r−h)/3`, lens `= 2×`) within the band, watertight, every seam node on both
  surfaces ≤ tol; and a tangent / coincident sphere pair returns NULL (deferred). No OCCT
  linked, no tolerance weakened. **Sim native-vs-OCCT** — extend
  `tests/sim/native_ssi_curved_boolean_parity.mm` + `run-sim-native-ssi-curved-boolean.sh`
  with the through-drill Fuse/Cut and the sphere∩sphere Common against
  `BRepAlgoAPI_{Fuse,Cut,Common}` (volume, area, watertight, validity); report per-pair
  deltas and the count still deferred to OCCT.
- **Roadmap**: advances `SSI-ROADMAP.md` S5 from S5-a (one COMMON) toward wider coverage
  (S5-b fuse/cut, S5-c sphere∩sphere) — the three explicitly-listed S5-a remainders that
  do NOT need S4. Near-tangent robustness (S4), oblique / multi-tube piercings, and the
  remaining curved-curved families (cyl∩cone, cyl∩sphere, sphere∩box) stay the tail.
- **Risk (honest)**: (a) the through-drill fuse/cut cap re-trim can leave a hairline seam
  gap on a high-curvature mouth — caught by the ENGINE watertight check → discard → OCCT,
  never shipped leaky; (b) a sphere∩sphere pole sample near-ON the other sphere
  (near-tangent lens) misclassifies — the ON-band abort + the engine correct-volume guard
  DISCARD it → OCCT; (c) the generic set-algebra guard's own native-COMMON call must
  itself be watertight for the fuse/cut check — if it is not, the guard trusts the
  watertight result conservatively (existing behaviour) and a wrong candidate is caught by
  the watertight leg. Whatever does not verify falls back to OCCT and is reported with the
  measured gap; no case is faked, stubbed, or hand-tuned to pass.
