# native-booleans

Extend the S5-a SSI-curve-driven curved boolean (`src/native/boolean/ssi_boolean.{h,cpp}`,
`openspec/SSI-ROADMAP.md` S5) with two additive assembler slices, both driven by the
already-shipped S3 `ssi::TraceSet` and both guarded by the engine's EXISTING generic
set-algebra self-verify → OCCT fallback:

- **S5-b** — native **Fuse** (`A ∪ B`) and **Cut** (`A − B`) for the through-drill
  cylinder∩cylinder topology `buildCommon` already recognises (two full-circle rim seams),
  by selecting the complementary fragments per the op set algebra over the SAME seams and
  the SAME planar-facet weld.
- **S5-c** — native **Common** (`A ∩ B`) for the transversal sphere∩sphere lens via a NEW
  single-seam / two-cap assembler that welds two spherical caps along the ONE seam circle.

Both DECLINE (NULL → OCCT) outside their verified envelope; near-tangent / coincident
pairs remain the S4 boundary. Internal: **no `cc_*` ABI change** — invoked behind the
existing `cc_boolean` op codes. `src/native/**` stays OCCT-free; the paths are compiled
under `CYBERCAD_HAS_NUMSCI`. No change to `src/native/tessellate` or to the S5-a
`buildCommon` path.

## ADDED Requirements

### Requirement: SSI-driven native Fuse and Cut for the through-drill cylinder∩cylinder pair

The native boolean library SHALL compute `cc_boolean(a, b, op)` — `op = 0` fuse
(`A ∪ B`), `op = 1` cut (`A − B`) — NATIVELY for the **through-drill
cylinder∩cylinder** topology the S5-a COMMON path (`buildCommon`) already recognises: two
transversal, fully-traced (`nearTangentGaps == 0`) closed rim seams, one operand
full-circle on both (the piercing tube), the other local on both (the pierced mouths). It
SHALL reuse the SAME two rim seams, the SAME shared `VertexPool` weld, and the SAME
planar-triangle facet discipline as the S5-a COMMON, and SHALL select the surviving
fragments per the op's face-survival rule — the SAME set algebra as the planar path:

- **Cut `A − B`** (`A` = the pierced cylinder, `B` = the piercing tube): the pierced wall
  and end caps OUTSIDE the tube (re-trimmed to exclude the drilled region) + the tube band
  INSIDE the pierced solid REVERSED as the inward tunnel wall; the two drill-mouth caps
  (removed material) SHALL be dropped.
- **Fuse `A ∪ B`**: the pierced wall OUTSIDE the tube + the tube wall OUTSIDE the pierced
  solid (each outside stretch with its own end-cap disc) + the operands' end caps; the two
  drill-mouth caps AND the inside tube band (now interior to the union) SHALL be dropped.

Every face that shares a rim seam with a differently tessellated neighbour SHALL be
emitted as PLANAR-TRIANGLE facets through the EXACT traced seam nodes drawn from the shared
`VertexPool`, so the shell welds watertight (the S5-a watertight discipline). Fragment
survival SHALL be decided by the S5-a curved point-in-solid test (`classifyPoint`) at an
interior sample; a sample that is robustly ON the other solid, an unrecognised
through-drill topology, or a weld that cannot close SHALL return a NULL `Shape` (→ OCCT).

The result SHALL be a native `topology::Shape` of type `Solid` carrying true curved face
kinds, watertight (every edge shared by exactly two faces), whose enclosed volume equals
the exact set-algebra value for the op within a relative tolerance sized to the
curved-face tessellation deflection: `Vr ≈ vol(A) + vol(B) − vol(A ∩ B)` (fuse) or
`Vr ≈ vol(A) − vol(A ∩ B)` (cut), where `vol(A ∩ B)` is the S5-a through-drill COMMON. The
builder SHALL remain OCCT-free and reference no OCCT / `IEngine` / `EngineShape` type, and
SHALL be compiled under `CYBERCAD_HAS_NUMSCI`. No `cc_*` entry point, signature, or POD
struct SHALL be added or changed, and the S5-a `buildCommon` COMMON path SHALL be
unchanged.

#### Scenario: The through-drill cut removes the tunnel with the correct volume (host)
- GIVEN a thin cylinder drilled clean through a fat one (the S5-a transversal through-drill
  fixture, `nearTangentGaps == 0`, two closed rim seams), built as native curved solids on
  the host with no OCCT
- WHEN `cc_boolean(fat, thin, 1)` (cut) is computed and tessellated by
  `src/native/tessellate`
- THEN the result SHALL be a watertight `Solid` (every edge shared by exactly two faces)
  AND its enclosed volume SHALL equal `vol(fat) − vol(fat ∩ thin)` within the curved-face
  deflection tolerance (`vol(fat ∩ thin)` = the S5-a-pinned through-drill COMMON volume)

#### Scenario: The through-drill fuse welds both cylinders with the correct volume (host)
- GIVEN the same transversal through-drill pair built as native curved solids on the host
  with no OCCT
- WHEN `cc_boolean(fat, thin, 0)` (fuse) is computed and tessellated
- THEN the result SHALL be a watertight closed 2-manifold `Solid` AND its enclosed volume
  SHALL equal `vol(fat) + vol(thin) − vol(fat ∩ thin)` within the curved-face deflection
  tolerance AND SHALL satisfy `fuse ≥ max(vol(fat), vol(thin))`

#### Scenario: Fuse and cut reuse the S5-a seam and weld, not a new trace (host)
- GIVEN the through-drill pair and its shipped S3 `TraceSet` (the SAME two rim seams the
  S5-a COMMON consumes)
- WHEN the native fuse and cut are assembled
- THEN they SHALL be built from the SAME two rim seams and the SAME shared-vertex
  planar-facet weld as the S5-a COMMON, with no new tracing and no hand-matched
  per-primitive result builder

### Requirement: SSI-driven native Common for the transversal sphere∩sphere lens

The native boolean library SHALL compute `cc_boolean(a, b, 2)` (common, `A ∩ B`) NATIVELY
for a **transversal sphere∩sphere** pair via a NEW single-seam / two-cap assembler
(`buildLensCommon`), taken when the S3 `TraceSet` is ONE `Closed` seam
(`nearTangentGaps == 0`) and BOTH operands are recognised as `Sphere` curved solids. The
sphere∩sphere COMMON (lens) is bounded by TWO spherical caps — the cap of sphere A inside
sphere B and the cap of sphere B inside sphere A — meeting along the ONE seam circle. The
assembler SHALL:

- **Gate.** Require exactly one closed seam and both operands spheres; a seam that is a
  full sphere (a coincident / degenerate case) SHALL return NULL.
- **Cap survival (COMMON rule).** Take each cap's POLE (sphere A's surface point nearest
  sphere B's centre, and symmetrically), evaluated on the analytic sphere; keep the A-cap
  ONLY IF its pole is INSIDE sphere B and the B-cap ONLY IF its pole is INSIDE sphere A
  (the S5-a `classifyPoint` test). A pole robustly ON the other sphere (tangent /
  coincident) SHALL abort the native path → NULL → OCCT, never a guessed side.
- **Cap weld.** Emit each cap with the SAME radial-ring planar-facet discipline as the
  S5-a drill-mouth cap (fan from the pole out through concentric rings to the OUTER ring =
  the EXACT traced seam nodes drawn from the shared `VertexPool`, every ring node on the
  analytic sphere, facet normals oriented outward). Because BOTH caps' outer rings are the
  SAME pooled seam vertices, the two caps SHALL weld watertight along the single seam.

The result SHALL be a native `topology::Shape` of type `Solid`, watertight (every edge
shared by exactly two faces), with every seam node on BOTH spheres within tolerance, whose
enclosed volume equals the closed-form lens volume within the curved-face deflection
tolerance:
`V = π (rA + rB − d)² (d² + 2 d·rB − 3 rB² + 2 d·rA + 6 rA·rB − 3 rA²) / (12 d)`
for centre distance `d` (equal radii `r`: two caps of height `h = r − d/2`, cap volume
`π h² (3r − h) / 3`, lens `= 2 ×`). The assembler SHALL remain OCCT-free and be compiled
under `CYBERCAD_HAS_NUMSCI`; no `cc_*` entry point SHALL be added or changed, and the S5-a
two-seam `buildCommon` path SHALL be unchanged.

#### Scenario: Equal-radius sphere∩sphere common matches the closed-form lens (host)
- GIVEN two equal-radius spheres (radius `r`, centre distance `0 < d < 2r`) built as native
  curved solids on the host with no OCCT, and their S3 `TraceSet` (one `Closed` seam,
  `nearTangentGaps == 0`)
- WHEN `cc_boolean(A, B, 2)` (common) is computed and tessellated
- THEN the result SHALL be a watertight `Solid` (every edge shared by exactly two faces)
  with every seam node on both spheres within tolerance AND its enclosed volume SHALL
  equal the lens `2 · π h² (3r − h) / 3`, `h = r − d/2`, within the curved-face deflection
  tolerance

#### Scenario: Unequal-radius sphere∩sphere common matches the closed-form lens (host)
- GIVEN two spheres of distinct radii `rA ≠ rB` with `|rA − rB| < d < rA + rB`, built as
  native curved solids on the host, and their one-seam S3 `TraceSet`
- WHEN `cc_boolean(A, B, 2)` (common) is computed and tessellated
- THEN the result SHALL be a watertight `Solid` whose enclosed volume equals the asymmetric
  lens closed form within the curved-face deflection tolerance

#### Scenario: A single-seam sphere pair is handled by the two-cap path, not the tube path (host)
- GIVEN a transversal sphere∩sphere pair (one closed seam) and a through-drill
  cylinder∩cylinder pair (two rim seams)
- WHEN each is dispatched
- THEN the sphere pair SHALL be assembled by the single-seam two-cap `buildLensCommon` and
  the cylinder pair by the two-seam `buildCommon`, each by its own topology gate, with no
  cross-contamination and no hand-matched per-primitive builder

### Requirement: The wider SSI curved booleans are guarded by the existing engine self-verify

The engine SHALL accept a native S5-b fuse / cut or S5-c sphere∩sphere common result as
native ONLY when it PASSES the EXISTING mandatory self-verify
(`native_engine.cpp booleanResultVerified`): a closed watertight 2-manifold with the
correct set-algebra volume — `Vr ≈ va + vb − vc` (fuse), `Vr ≈ va − vc` (cut), or
`Vr ≈ vc` (common) — within a relative tolerance sized to the curved-face tessellation
deflection, where `vc` is the native COMMON volume (the through-drill COMMON for the
cylinder pair; the lens for the sphere pair). NO new engine oracle SHALL be added: the
generic set-algebra guard already computes these, and the `ssiCurvedBooleanVerified`
Steinmetz special oracle (equal-radius perpendicular cylinders) SHALL remain untouched and
SHALL NOT fire for these cases. If the self-verify fails, the engine SHALL DISCARD the
native result and fall through to OCCT `BRepAlgoAPI` (OCCT operand) or report an honest
error (both operands native voids). The engine SHALL NEVER emit an unverified, leaky, or
wrong wider SSI curved boolean.

#### Scenario: A bad wider SSI curved boolean candidate is discarded (host)
- GIVEN a native S5-b fuse/cut or S5-c sphere-common candidate that is open / non-manifold
  OR whose enclosed volume is outside the deflection-sized band for its op, built on the
  host
- WHEN the existing generic set-algebra self-verify guard is applied
- THEN the guard SHALL reject the candidate AND the engine SHALL NOT emit a leaky or wrong
  curved solid (a native-native case reports an honest error; an OCCT-operand case falls
  through to OCCT)

#### Scenario: A verified fuse / cut / lens-common passes the existing guard (host)
- GIVEN a native through-drill fuse or cut whose watertight volume matches
  `va + vb − vc` / `va − vc`, OR a native sphere∩sphere common whose volume matches the
  closed-form lens, all within the deflection band
- WHEN the existing generic set-algebra guard is applied
- THEN the guard SHALL accept the candidate AND it SHALL be served natively with no OCCT
  fallback call AND the Steinmetz special oracle SHALL NOT have fired

### Requirement: Out-of-envelope wider curved pairs fall through to OCCT

The wider SSI curved boolean builders SHALL DECLINE (return a NULL `Shape`) for any case
outside the two shipped slices: (1) **sphere∩sphere fuse / cut** (S5-c ships COMMON only);
(2) **tangent / coincident spheres** (`nearTangentGaps > 0`, or a cap pole robustly ON the
other sphere); (3) **equal-radius orthogonal cylinder∩cylinder** (the Steinmetz pair —
`nearTangentGaps > 0`, an S4 case); (4) **oblique / multi-tube cylinder∩cylinder**
piercings whose seams are not two clean full-circle rims; (5) **other curved-curved
families** (cylinder∩cone, cylinder∩sphere, cone∩cone, sphere∩box, freeform). When either
operand is an OCCT body, each such case SHALL produce EXACTLY the OCCT `BRepAlgoAPI`
fallback result; when both operands are native voids OCCT cannot read, the engine SHALL
report an honest error. The change SHALL NOT fake, stub-out, hand-tune, or partially
implement any deferred case; `nearTangentGaps > 0` SHALL remain the honest S4 fallback
boundary, not consumed and not an error.

#### Scenario: A sphere∩sphere fuse or cut declines to OCCT (host)
- GIVEN a transversal sphere∩sphere pair with the native engine active
- WHEN `cc_boolean(A, B, 0)` (fuse) or `cc_boolean(A, B, 1)` (cut) is invoked
- THEN the wider builder SHALL return a NULL `Shape` (S5-c ships COMMON only) AND (with an
  OCCT operand) the result SHALL be identical to invoking the same call with the OCCT
  engine active, proving fall-through with no native interception

#### Scenario: Tangent spheres and the Steinmetz cylinder pair decline (host)
- GIVEN two tangent spheres (`d = rA + rB`) OR two equal-radius perpendicular cylinders
  (the Steinmetz pair), with the native engine active
- WHEN `cc_boolean` (common) is invoked
- THEN the wider builder SHALL return a NULL `Shape` (near-tangent → S4) AND the engine
  SHALL NOT emit a native result for that call

### Requirement: Wider SSI curved boolean parity with OCCT through the facade (simulator gate)

The wider SSI curved booleans' fidelity SHALL be reported as a MEASURED native-vs-OCCT
parity against `BRepAlgoAPI_{Fuse,Cut,Common}` on the simulator — volume, surface area,
watertightness (closed shell), and shape validity (`BRepCheck`) — on the through-drill
cylinder∩cylinder **fuse** and **cut** and the transversal sphere∩sphere **common** (built
as OCCT `BRepPrimAPI_MakeSphere` / cylinder solids), extending the S5-a harness
(`scripts/run-sim-native-ssi-curved-boolean.sh` +
`tests/sim/native_ssi_curved_boolean_parity.mm`) rather than adding a new harness. The
count of pairs still deferred to OCCT (sphere fuse/cut, near-tangent, out-of-family) SHALL
be REPORTED (the S4 / follow-on seam), not hidden or padded, and whatever the wider slices
cannot compute SHALL fall back to OCCT and be reported with the measured gap.

#### Scenario: native-vs-OCCT parity is reported per wider pair on the simulator
- GIVEN the through-drill fuse/cut pair and the transversal sphere∩sphere common pair,
  each built both as native curved solids and as OCCT `BRepPrimAPI` solids
- WHEN the native wider boolean and OCCT `BRepAlgoAPI_{Fuse,Cut,Common}` each compute the
  op on the simulator
- THEN the harness SHALL report the native-vs-OCCT volume delta, surface-area delta,
  watertight/closed-shell status, and shape validity within tolerance, compared at the
  `cybercad::native::boolean` C++ boundary
- AND no `cc_*` entry point SHALL have been added, and the count of pairs deferred to OCCT
  SHALL be reported, not hidden
