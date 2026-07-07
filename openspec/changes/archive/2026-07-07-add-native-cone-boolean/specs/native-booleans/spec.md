# native-booleans

Open the CONE surface family in the SSI-driven native curved-boolean assembler
(`src/native/boolean/ssi_boolean.cpp`, `openspec/SSI-ROADMAP.md` S5). The through-drill cyl∩cyl
(S5-a/b), sphere∩sphere lens (S5-c), and branched Steinmetz (S5-d) families are ALREADY native
(three ops each). `recogniseCurvedSolid` already folds a `CurvedKind::Cone` face and
`classifyPoint` already scores the cone wall half-space, and the S1 analytic layer
(`intersectCylinderConeCoaxial`) already closed-forms a coaxial cone∩cylinder seam as a single
circle — recognition AND tracing are in place. This change adds the ASSEMBLER for the
analytically-cleanest cone boolean: the COAXIAL cone(frustum)∩cylinder COMMON, whose seam is one
circle and whose enclosed volume is closed-form (a frustum band welded to a cylinder-segment
band), plus optionally the coaxial cone∩sphere COMMON.

The coaxial cone∩cylinder COMMON is the min-radius-profile solid of revolution over the axial
overlap `[hBot, hTop]`: the CONE band where the cone radius `r_c(h)` is the smaller (kept because
it classifies INSIDE the cylinder) welded along the single seam circle at `h*` (`r_c(h*) = Rc`) to
the CYLINDER band where `Rc` is the smaller (kept because it classifies INSIDE the cone), closed
by a bottom disc cap at `hBot` and a top disc cap at `hTop`. All fragments share the pooled seam
ring and their terminal rings and weld watertight through one `VertexPool` with the planar-facet
discipline the cyl / sphere / Steinmetz families use. The enclosed volume equals
`V_frustum(r(hBot) → Rc) + π Rc²·(hTop − h*)` — verified in the engine self-verify against that
closed form (a NEW analytic oracle mirroring the Steinmetz `16 R³/3` oracle).

The cone builder DECLINES (NULL → OCCT) outside its verified envelope: an apex-crossing seam or a
frustum whose extent reaches the apex (S4-e territory), a non-coaxial (transversal) cone pair (a
quartic space curve, not analytic here), a cap-edge-tangent seam, a two-circle coaxial cone∩sphere
crossing, cone∩cone, and cone FUSE/CUT all remain the OCCT boundary. Internal: **no `cc_*` ABI
change** — invoked behind the existing `cc_boolean` op codes. `src/native/**` stays OCCT-free; the
path is compiled under `CYBERCAD_HAS_NUMSCI`. No change to `src/native/tessellate`, the planar
BSP-CSG, the analytic `curved.h`, or the cyl / sphere / Steinmetz builders (all byte-identical).

## ADDED Requirements

### Requirement: SSI-driven native Common for the coaxial cone∩cylinder pair

The native boolean library SHALL compute `cc_boolean(a, b, op=2)` common (`A ∩ B`) NATIVELY for a
COAXIAL cone∩cylinder pair: one operand recognised as a `Cone` frustum solid (via
`recogniseCurvedSolid`), the other as a `Cylinder` solid whose axis is COLLINEAR with the cone
axis (`sameAxis`), whose S3/S1 seam trace is EXACTLY ONE closed full-circle seam on BOTH walls
(the S1 analytic circle from `intersectCylinderConeCoaxial`, `nearTangentGaps == 0`,
`branchPoints == 0`), where the frustum is APEX-FREE over its extent (`r_c(v) > margin` for all
`v`, so the S4-e apex chart singularity is never touched) and the seam height `h*` (where the cone
cross-section radius `r_c(h*)` equals the cylinder radius `Rc`) lies STRICTLY inside the axial
overlap `[hBot, hTop] = [max(coneBottom, cylBottom), min(coneTop, cylTop)]`.

The builder SHALL assemble the common boundary as the min-radius-profile solid of revolution: the
CONE wall band on the side of `h*` where the cone radius is the smaller, welded along the single
seam circle to the CYLINDER wall band on the side where `Rc` is the smaller, closed by two planar
disc caps (a bottom cap at `hBot` with outward normal `−ẑ`, a top cap at `hTop` with outward
normal `+ẑ`). It SHALL resample the traced seam circle into one full-turn ring pooled ONCE (the
shared seam ring), emit each band as a planar-facet ring strip drawn through the shared
`VertexPool` so the two bands weld along the seam ring and each band's outer terminal ring welds
to its disc cap, and use the cone's TRUE outward wall normal (`radial·cosα − ẑ·sinα`) for the cone
band and the pure outward radial normal for the cylinder band.

The builder SHALL keep the CONE band only if its interior sample classifies strictly INSIDE the
cylinder (`classifyPoint(cyl, mid) == 1`) AND the CYLINDER band only if its interior sample
classifies strictly INSIDE the cone (`classifyPoint(cone, mid) == 1`); an ON verdict (`== 0`, a
tangent / cap-edge seam), a frustum whose extent reaches the apex, a non-coaxial (transversal)
pair, a seam that is not exactly one interior full circle, or a weld that cannot close SHALL
return a NULL `Shape` (→ OCCT). The tolerance SHALL NOT be weakened to force a pass.

The result SHALL be a native `topology::Shape` of type `Solid` carrying true `Cone` (frustum band)
and `Cylinder` (cylinder band) and planar (cap) face kinds, watertight (every edge shared by
exactly two faces), whose enclosed volume equals
`V_frustum(r(hBot) → Rc) + π·Rc²·(hTop − h*)`, where
`V_frustum(ra → rb over Δh) = (π·Δh/3)(ra² + ra·rb + rb²)`, within a relative tolerance sized to
the curved-face tessellation deflection. The builder SHALL remain OCCT-free and reference no OCCT
/ `IEngine` / `EngineShape` type, SHALL be compiled under `CYBERCAD_HAS_NUMSCI`, and SHALL add or
change no `cc_*` entry point, signature, or POD struct.

#### Scenario: The coaxial cone∩cylinder common is the min-profile solid with the correct volume (host)
- GIVEN a frustum cone A (`r_c(h) = R0 + (h − h0)·tanα`, apex-free over its extent) and a coaxial
  cylinder B (radius `Rc`) whose walls cross at a single circle at height `h*` (`r_c(h*) = Rc`)
  strictly inside the axial overlap `[hBot, hTop]`, with a clean single-circle seam trace
  (`nearTangentGaps == 0`, `branchPoints == 0`)
- WHEN `ssi_boolean_solid(A, B, Op::Common)` runs (`buildCommon` declines the single seam,
  `buildLensCommon` declines the non-sphere operand, and `buildConeCylCommon` assembles the shell)
- THEN it returns a watertight `Solid` (`boundaryEdgeCount == 0`, every edge shared by exactly two
  faces) bounded by the CONE band (`r_c ≤ Rc` side, inside the cylinder) welded along the seam
  circle to the CYLINDER band (`Rc ≤ r_c` side, inside the cone), closed by a bottom disc cap at
  `hBot` and a top disc cap at `hTop`
- AND its enclosed volume equals `V_frustum(r(hBot) → Rc) + π·Rc²·(hTop − h*)` within the
  deflection-sized band (for the reference fixture `r_c(h)=0.4+0.4h`, `Rc=1`, `h∈[1,5]`: `h*=1.5`,
  overlap `[1,4]`, volume ≈ `9.1315`)
- AND every seam-ring node lies on BOTH walls within tolerance, and the seam ring is pooled ONCE
  (shared by both bands).

#### Scenario: An apex-crossing / transversal / cap-tangent cone pair declines to OCCT (host)
- GIVEN a coaxial cone∩cylinder pair whose frustum extent reaches the apex (`r_c → 0`), OR a
  NON-coaxial (transversal) cone∩cylinder pair whose seam is a quartic space curve
  (`intersectCylinderConeCoaxial` returns `notAnalytic`), OR a pair whose seam `h*` sits on a cap
  edge (a tangent, not a strictly-interior transversal circle)
- WHEN `ssi_boolean_solid(A, B, Op::Common)` runs
- THEN `buildConeCylCommon` refuses at the gate (apex, non-coaxial, or ON-edge seam) and returns a
  NULL `Shape`, and the engine falls through to OCCT `BRepAlgoAPI_Common` — reported, not faked,
  tolerance not weakened.

#### Scenario: The engine discards a wrong-volume cone common candidate (host)
- GIVEN a cone-common candidate whose welded shell volume does not match
  `V_frustum(r(hBot) → Rc) + π·Rc²·(hTop − h*)` (a mis-selected band, a mis-placed cap, or a
  hairline seam-ring gap)
- WHEN the engine's `ssiCurvedBooleanVerified` runs the coaxial-cone closed-form oracle (the
  `op == 2` branch, mirroring the Steinmetz `16 R³/3` oracle)
- THEN the candidate FAILS the watertight + closed-form-volume guard and is DISCARDED → OCCT; the
  engine never emits an unverified cone common.

#### Scenario: Cone Fuse / Cut and other cone pairs remain the OCCT boundary (host)
- GIVEN a coaxial cone∩cylinder pair with `Op::Fuse` or `Op::Cut`, OR a cone∩cone pair, OR a
  two-circle coaxial cone∩sphere crossing
- WHEN `ssi_boolean_solid` runs
- THEN the cone COMMON path is NOT wired for fuse/cut, cone∩cone, or the two-circle crossing, so it
  returns a NULL `Shape` and the engine ships OCCT `BRepAlgoAPI_{Fuse,Cut,Common}` — the
  analytically-clean coaxial cone∩cylinder (and single-crossing cone∩sphere) COMMON is the only
  new native cone op.

#### Scenario: The cyl / sphere / Steinmetz families are unchanged by the cone addition (host)
- GIVEN the existing through-drill cyl∩cyl, sphere∩sphere lens, and Steinmetz bicylinder fixtures
  across all three ops
- WHEN `ssi_boolean_solid` runs after the `Op::Common` dispatch grows the `buildConeCylCommon` arm
- THEN each existing family produces its byte-identical result (same volume, area, and vertices as
  before this change) — `buildConeCylCommon` returns `{}` for every non-(cone+coaxial-cylinder)
  pair, so the existing native passes do not regress.

### Requirement: SSI-driven native Common for the coaxial cone∩sphere pair (optional)

The native boolean library SHALL compute `cc_boolean(a, b, op=2)` common NATIVELY for a COAXIAL
cone∩sphere pair in the SINGLE-crossing configuration: one operand a `Cone` frustum, the other a
`Sphere` whose centre lies on the cone axis, whose S1 seam (`intersectSphereConeCoaxial`) is
EXACTLY ONE valid circle inside both operand extents. It SHALL assemble the frustum band welded
along that seam circle to the spherical-segment band (the sphere-latitude strip inside the cone),
closed by the terminal disc caps, selected by the same inside-the-other rule (`classifyPoint`) and
welded watertight through one `VertexPool`. When `intersectSphereConeCoaxial` returns TWO circles
inside both extents, or the frustum reaches the apex, or the pair is non-coaxial, the builder SHALL
return a NULL `Shape` (→ OCCT). The result SHALL be a watertight `Solid` whose enclosed volume
equals `V_frustum + V_spherical-segment` within the deflection-sized tolerance, verified by the
engine self-verify against that closed form. The builder SHALL remain OCCT-free, be compiled under
`CYBERCAD_HAS_NUMSCI`, and add or change no `cc_*` entry point, signature, or POD struct. This
requirement is OPTIONAL — shipped only if the coaxial cone∩sphere COMMON lands watertight in the
verified envelope; otherwise it is deferred → OCCT with the measured gap reported.

#### Scenario: The single-crossing coaxial cone∩sphere common has the correct closed-form volume (host)
- GIVEN a frustum cone A and a coaxial sphere B (centre on the cone axis) whose S1 seam is exactly
  ONE valid circle inside both extents
- WHEN `ssi_boolean_solid(A, B, Op::Common)` runs and `buildConeSphereCommon` assembles the shell
- THEN it returns a watertight `Solid` bounded by the frustum band welded along the seam circle to
  the spherical-segment band, closed by the terminal disc caps
- AND its enclosed volume equals `V_frustum + V_spherical-segment` within the deflection-sized band
- AND every seam-ring node lies on BOTH walls within tolerance, and the seam ring is pooled ONCE.

#### Scenario: A two-circle coaxial cone∩sphere crossing declines to OCCT (host)
- GIVEN a coaxial cone∩sphere pair whose `intersectSphereConeCoaxial` returns TWO circles inside
  both extents (the sphere passes fully through the cone)
- WHEN `ssi_boolean_solid(A, B, Op::Common)` runs
- THEN `buildConeSphereCommon` declines the two-circle config and returns a NULL `Shape`, and the
  engine ships OCCT `BRepAlgoAPI_Common` — reported, not faked (the first slice handles the
  single-crossing config only).
