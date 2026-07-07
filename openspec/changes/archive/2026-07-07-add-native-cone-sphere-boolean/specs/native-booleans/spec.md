# native-booleans

Open the coaxial cone∩sphere curved-boolean op-set in `src/native/boolean/ssi_boolean.cpp`
(`openspec/SSI-ROADMAP.md` S5-f) — the next cone-involving pair after the coaxial cone∩cylinder
op-set (now 3/3 native). A coaxial cone frustum A and a sphere B whose centre lies ON the cone axis
meet along ONE analytic CIRCLE seam (SSI S1 `intersectSphereConeCoaxial` — a QUADRATIC in the cone
parameter; the single-crossing configuration, where the sphere sits on the frustum side, gives
exactly ONE circle that does NOT cross the cone apex). Both operands are solids of revolution about
the shared axis, so `A ∩ B`, `A ∪ B`, and `A − B` are all solids of revolution described by a radial
profile over the axial station `s`. The seam circle at `s*` (`r_c(s*) = r_s(s*)`, cone radius
`r_c(s) = R0 + s·tanα`, sphere radius `r_s(s) = √(Rs² − (s − s_c)²)` about the on-axis centre `s_c`)
splits each wall into an INSIDE-the-other band and an OUTSIDE band; the cone carries its two disc end
caps, the sphere wall closes on the axis at its two poles `s_c ± Rs`.

This change adds native **Common**, **Fuse**, and **Cut** for that single seam circle, reusing TWO
existing families of machinery: the CONE side reuses the cone-wall split (`appendRevolvedBand` +
`appendDiskCap`, as `buildConeCylCommon` does), and the SPHERE side reuses the spherical-cap
fragment (`appendSphereCap`, with its inner/outer apex and reversed-normal flags, as the sphere-lens
builders do). COMMON = the min-cross-section overlap (the cone wall band inside the sphere welded
along the seam circle to the spherical segment inside the cone, closed by the cone terminal disc
inside the sphere). FUSE = the max-cross-section union (the sphere outer cap outside the cone welded
to the cone outer wall outside the sphere, closed by the cone terminal disc bounding the union).
CUT `A − B` (cone minuend) = A's outer wall outside B + A's disc cap(s) outside B, joined to the
sphere INNER cap emitted REVERSED (inward radial normal, the spherical dimple bounding the carved
cavity, pinching to the seam circle) — a CONNECTED frustum-with-a-spherical-dimple. All three share
the SAME single S1 analytic seam circle and weld watertight through one `VertexPool` (the cone band
and the spherical cap draw the identical pooled seam nodes).

All three DECLINE (NULL → OCCT) outside their verified envelope: a TWO-circle crossing (the sphere
passes fully through the cone / spans the apex), an apex-crossing seam or a frustum whose extent
reaches the apex, a non-coaxial (transversal) cone∩sphere pair (a quartic space curve, not analytic
here), a `sphere − cone` CUT (sphere minuend), and cone∩cone all remain the OCCT boundary. Internal:
**no `cc_*` ABI change** — invoked behind the existing `cc_boolean` op codes. `src/native/**` stays
OCCT-free; the path is compiled under `CYBERCAD_HAS_NUMSCI`. COMMON is guarded by a NEW analytic
closed-form oracle arm (`V_frustum + V_spherical-segment`) added to `ssiCurvedBooleanVerified`
(`src/engine/native/native_engine.cpp`), mirroring the existing Steinmetz and cone∩cylinder COMMON
arms; FUSE / CUT reuse the EXISTING generic set-algebra self-verify (fuse grows, cut shrinks, vs the
native cone∩sphere COMMON). No change to `src/native/tessellate`, the planar BSP-CSG, the analytic
`curved.h`, or the cyl / sphere / Steinmetz / cone∩cylinder builders.

## ADDED Requirements

### Requirement: SSI-driven native Common for the coaxial cone∩sphere pair

The native boolean library SHALL compute `cc_boolean(a, b, op=2)` common (`A ∩ B`) NATIVELY for the
COAXIAL cone∩sphere pair in the SINGLE-crossing configuration: one operand recognised as a `Cone`
frustum solid (via `recogniseCurvedSolid`), the other as a `Sphere` solid whose centre lies ON the
cone axis (`distancePointLine`/`sameAxis`), whose S1 seam trace is EXACTLY ONE closed full-circle
seam on BOTH walls (the S1 analytic circle from `intersectSphereConeCoaxial`, `nearTangentGaps ==
0`, `branchPoints == 0`), where the frustum is APEX-FREE over its extent and the single seam height
`s*` (where the cone cross-section radius `r_c(s*)` equals the sphere cross-section radius `r_s(s*)`)
lies STRICTLY inside both operand extents — the QUADRATIC's other root falling OUTSIDE the frustum
extent.

The builder SHALL reuse the SAME shared gate/seam prologue (`coneSphereSetup`: the coaxial gate, the
analytic-vs-traced seam cross-check, the axis frame, the crossing `s*`, the azimuth resolution, the
two sphere poles classified against the cone, and ONE canonical pooled seam ring at `(r_c(s*), s*)`),
the SAME shared `VertexPool` weld with the seam ring pooled ONCE, the cone-side planar-facet revolve
discipline (`appendRevolvedBand`, `appendDiskCap`), and the sphere-side cap builder
(`appendSphereCap`), and SHALL assemble the overlap boundary as the min-cross-section profile: the
CONE wall band on the `r_c ≤ r_s` side (inside the sphere) welded along the seam circle to the
SPHERE INNER cap on the `r_s ≤ r_c` side (the spherical segment inside the cone, closing to the
sphere pole that lies inside the cone), closed by the cone terminal disc that bounds the overlap (the
cone end disc inside the sphere).

The builder SHALL keep the cone band only if its interior sample classifies strictly INSIDE the
sphere AND the sphere inner cap only if its pole classifies strictly INSIDE the cone (the common
survival rule, via the curved point-in-solid test `classifyPoint`); a sample robustly ON the other
wall (`classifyPoint == 0`), a TWO-circle crossing (both roots inside both extents), a frustum whose
extent reaches the apex, a non-coaxial (transversal) pair, or a weld that cannot close SHALL return a
NULL `Shape` (→ OCCT). The tolerance SHALL NOT be weakened to force a pass.

The result SHALL be a native `topology::Shape` of type `Solid` carrying a true `Cone` (frustum band)
wall face kind, a true `Sphere` (spherical cap) wall face kind, and planar (disc cap) face kinds,
watertight (every edge shared by exactly two faces), whose enclosed volume equals the closed form
`V_frustum(cone-tighter sub-band) + V_spherical-segment(sphere-tighter sub-band)` within a relative
tolerance sized to the curved-face tessellation deflection — verified by the engine self-verify
against that closed form (a NEW analytic oracle arm mirroring the Steinmetz `16 r³/3` and
cone∩cylinder arms). The builder SHALL remain OCCT-free and reference no OCCT / `IEngine` /
`EngineShape` type, SHALL be compiled under `CYBERCAD_HAS_NUMSCI`, and SHALL add or change no `cc_*`
entry point, signature, or POD struct.

#### Scenario: The single-crossing coaxial cone∩sphere common has the correct closed-form volume (host)
- GIVEN a frustum cone A (`r_c(s) = R0 + s·tanα`, apex-free) and a coaxial sphere B (centre on the
  cone axis) whose walls cross at a single circle at `s*` (`r_c(s*) = r_s(s*)`) strictly inside both
  extents, with a clean single-circle seam trace (`nearTangentGaps == 0`, `branchPoints == 0`)
- WHEN `ssi_boolean_solid(A, B, Op::Common)` runs (the through-drill `buildCommon`, the sphere-lens
  `buildLensCommon`, and the cone∩cylinder `buildConeCylCommon` all decline, and
  `buildConeSphereCommon` assembles the overlap shell)
- THEN it returns a watertight `Solid` (`boundaryEdgeCount == 0`, every edge shared by exactly two
  faces) bounded by the cone wall band inside the sphere welded along the seam circle to the
  spherical segment inside the cone, closed by the cone terminal disc inside the sphere
- AND its enclosed volume equals `V_frustum(cone-tighter sub-band) + V_spherical-segment(sphere-
  tighter sub-band)` within the deflection-sized band (for the reference fixture `r_c(s) = 0.5 +
  0.5s`, sphere centre on-axis at the origin, `Rs = 2`: `s* ≈ 1.5436`, volume ≈ `5.256`)
- AND every seam-ring node lies on BOTH walls within tolerance, and the seam ring is pooled ONCE
  (shared by the cone band and the spherical cap).

#### Scenario: A two-circle / apex / transversal cone∩sphere pair declines common to OCCT (host)
- GIVEN a coaxial cone∩sphere pair whose `intersectSphereConeCoaxial` returns TWO circles inside
  both extents (the sphere passes fully through the cone), OR a frustum whose extent reaches the
  apex (`r_c → 0`), OR a NON-coaxial (transversal) cone∩sphere pair whose seam is a quartic space
  curve (`intersectSphereConeCoaxial` returns `notAnalytic`)
- WHEN `ssi_boolean_solid(A, B, Op::Common)` runs
- THEN `buildConeSphereCommon` refuses at the shared gate (two-circle, apex, or non-coaxial) and
  returns a NULL `Shape`, and the engine falls through to OCCT `BRepAlgoAPI_Common` — reported, not
  faked, tolerance not weakened.

#### Scenario: The engine discards a wrong-volume cone∩sphere common candidate (host)
- GIVEN a common candidate whose welded shell volume does not match `V_frustum + V_spherical-
  segment` (a mis-selected band or a mis-placed cap)
- WHEN the engine's NEW coaxial cone∩sphere COMMON analytic oracle arm in `ssiCurvedBooleanVerified`
  runs (`op == 2`)
- THEN the candidate FAILS the watertight + closed-form-volume guard and is DISCARDED → OCCT; the
  engine never emits an unverified cone∩sphere common.

### Requirement: SSI-driven native Fuse for the coaxial cone∩sphere pair

The native boolean library SHALL compute `cc_boolean(a, b, op=0)` fuse (`A ∪ B`) NATIVELY for the
coaxial cone∩sphere single-crossing pair the common path recognises (one `Cone` frustum + one
coaxial `Sphere` centre-on-axis, a single strictly-interior full-circle apex-free seam,
`nearTangentGaps == 0`). It SHALL reuse the SAME shared gate/seam prologue (`coneSphereSetup`),
`VertexPool` (seam ring pooled once), cone-side revolve discipline, and sphere-side cap builder as
`buildConeSphereCommon`, and SHALL assemble the union boundary as the max-cross-section outer
profile: the SPHERE OUTER cap (the part of the sphere outside the cone, `appendSphereCap` with the
FAR-pole apex, outward) welded along the SAME seam circle to the CONE OUTER wall band (the part of
the cone outside the sphere, outward radial), closed by the cone terminal disc(s) that bound the
union.

The builder SHALL keep the sphere outer cap only if its far pole classifies strictly OUTSIDE the
cone AND each cone outer band only if its interior sample classifies strictly OUTSIDE the sphere
(the fuse survival rule, via `classifyPoint`); a sample robustly ON the other wall
(`classifyPoint == 0`), a two-circle crossing, an apex-reaching frustum, a non-coaxial pair, or a
weld that cannot close SHALL return a NULL `Shape` (→ OCCT). The tolerance SHALL NOT be weakened.

The result SHALL be a native `topology::Shape` `Solid` with true `Cone` (wall), `Sphere` (cap), and
planar (disc cap) face kinds, watertight (every edge shared by exactly two faces), whose enclosed
volume equals `vol(A) + vol(B) − vol(A ∩ B)` within the deflection-sized relative tolerance, where
`vol(A ∩ B)` is the native cone∩sphere COMMON (`buildConeSphereCommon`). The builder SHALL remain
OCCT-free, reference no OCCT / `IEngine` / `EngineShape` type, be compiled under
`CYBERCAD_HAS_NUMSCI`, and add or change no `cc_*` entry point, signature, or POD struct.

#### Scenario: The coaxial cone∩sphere fuse grows to the outer envelope with the correct volume (host)
- GIVEN a frustum cone A and a coaxial sphere B (centre on the cone axis) whose walls cross at a
  single circle at `s*` strictly inside both extents, with a clean single-circle seam trace
- WHEN `ssi_boolean_solid(A, B, Op::Fuse)` runs (the through-drill `buildFuse`, the sphere-lens
  `buildLensFuse`, and the cone∩cylinder `buildConeCylFuse` all decline, and `buildConeSphereFuse`
  assembles the outer shell)
- THEN it returns a watertight `Solid` bounded by the sphere outer cap outside the cone welded along
  the seam circle to the cone outer wall outside the sphere, closed by the cone terminal disc
- AND its enclosed volume equals `vol(A) + vol(B) − vol(A ∩ B)` within the deflection-sized band
  (for the reference fixture volume ≈ `60.718`) — a GROW (`Vr > max(vol(A), vol(B))`)
- AND every seam-ring node lies on BOTH walls within tolerance, and the seam ring is pooled ONCE.

#### Scenario: The engine discards a wrong-volume cone∩sphere fuse candidate (host)
- GIVEN a fuse candidate whose welded shell volume does not match `vol(A) + vol(B) − vol(A ∩ B)` (a
  mis-selected outer band or a mis-oriented sphere cap)
- WHEN the engine's generic set-algebra self-verify runs (`expected = va + vb − vc`, `vc` = native
  cone∩sphere COMMON `buildConeSphereCommon`; the `op == 2`-only analytic oracle does NOT intercept
  fuse)
- THEN the candidate FAILS the watertight + correct-volume guard and is DISCARDED → OCCT; the engine
  never emits an unverified cone∩sphere fuse.

### Requirement: SSI-driven native Cut for the coaxial cone∩sphere pair

The native boolean library SHALL compute `cc_boolean(a, b, op=1)` cut (`A − B`, `A` the CONE
minuend) NATIVELY for the coaxial cone∩sphere single-crossing pair the common path recognises. It
SHALL reuse the SAME shared gate/seam prologue (`coneSphereSetup`), `VertexPool` (seam ring pooled
once), cone-side revolve discipline, and sphere-side cap builder as `buildConeSphereCommon`, and
SHALL assemble the difference boundary as A's OUTER wall band (the part of A outside B, outward
radial) plus A's terminal disc cap(s) outside B, plus the sphere INNER cap emitted REVERSED
(`appendSphereCap` with the near-pole apex and the inward normal — the spherical dimple bounding the
carved cavity, welded at the pooled seam circle so it pinches to the seam ring). Operand order SHALL
be honoured (CUT is not symmetric): `A` SHALL be the `Cone`; a `sphere − cone` (sphere minuend)
SHALL return a NULL `Shape` (→ OCCT). The single-crossing cone∩sphere cut SHALL be a CONNECTED solid
(one closed component — a frustum with a spherical dimple).

The builder SHALL keep each A outer band only if its interior sample classifies strictly OUTSIDE B
AND the reversed sphere inner cap only if its near pole classifies strictly INSIDE A (the cut
survival rule, via `classifyPoint`); a tangent / degenerate / wrong-side sample
(`classifyPoint == 0`), a two-circle crossing, an apex-reaching frustum, a non-coaxial pair, a
sphere minuend, or a weld that cannot close SHALL return a NULL `Shape` (→ OCCT). The tolerance
SHALL NOT be weakened.

The result SHALL be a native `topology::Shape` `Solid` with true `Cone` (wall), `Sphere` (dimple
cap), and planar (disc cap) face kinds, watertight (every edge shared by exactly two faces), whose
enclosed volume equals `vol(A) − vol(A ∩ B)` within the deflection-sized relative tolerance, where
`vol(A ∩ B)` is the native cone∩sphere COMMON. The builder SHALL remain OCCT-free, reference no OCCT
/ `IEngine` / `EngineShape` type, be compiled under `CYBERCAD_HAS_NUMSCI`, and add or change no
`cc_*` entry point, signature, or POD struct.

#### Scenario: The coaxial cone∩sphere cut carves a spherical dimple with the correct volume (host)
- GIVEN two coaxial operands A (a frustum cone, minuend) and B (a sphere centre-on-axis) whose walls
  cross at a single circle at `s*` strictly inside both extents, with a clean single-circle seam
  trace
- WHEN `ssi_boolean_solid(A, B, Op::Cut)` runs (the through-drill `buildCut`, the sphere-lens
  `buildLensCut`, and the cone∩cylinder `buildConeCylCut` all decline, and `buildConeSphereCut`
  assembles the shell)
- THEN it returns a watertight CONNECTED `Solid` bounded by A's OUTER wall band (outside B, outward)
  + A's disc cap(s) outside B + the sphere INNER cap REVERSED (inward, the dimple pinching to the
  seam circle)
- AND its enclosed volume equals `vol(A) − vol(A ∩ B)` within the deflection-sized band (for the
  reference fixture ≈ `27.207`) — a SHRINK (`Vr < vol(A)`)
- AND every seam-ring node lies on both walls within tolerance, and the seam ring is pooled ONCE.

#### Scenario: A sphere-minuend cut declines to OCCT (host)
- GIVEN a coaxial cone∩sphere pair with the SPHERE as the first operand (`sphere − cone`)
- WHEN `ssi_boolean_solid(sphere, cone, Op::Cut)` runs
- THEN `buildConeSphereCut` declines (`A` is not the `Cone` minuend) and returns a NULL `Shape`, and
  the engine ships OCCT `BRepAlgoAPI_Cut` — the sphere-with-a-conical-bite is a deferred follow-on,
  reported, not faked.

### Requirement: The existing curved-boolean families are unchanged by the cone∩sphere addition

The native boolean library SHALL keep every existing curved-boolean family (through-drill cyl∩cyl,
sphere∩sphere lens, Steinmetz bicylinder, coaxial cone∩cylinder) byte-identical when the cone∩sphere
op-set lands. `buildConeSphere{Common,Fuse,Cut}` SHALL return a NULL `Shape` for every non-(cone +
coaxial-sphere) pair so the existing builders and all their ops keep their existing results. The
dispatch SHALL grow only one final call per op arm (after the through-drill, lens, and cone∩cylinder
builders decline); recognition, tracing, the transversality gate, and the generic set-algebra
self-verify SHALL NOT change, and the NEW `ssiCurvedBooleanVerified` cone∩sphere arm SHALL return
not-applicable for every non-cone∩sphere pair and for `op != 2`.

#### Scenario: The cyl / sphere / Steinmetz / cone∩cylinder families are unchanged (host)
- GIVEN the existing through-drill cyl∩cyl, sphere∩sphere lens, Steinmetz bicylinder, and coaxial
  cone∩cylinder fixtures across all three ops
- WHEN `ssi_boolean_solid` runs after the `Op::Common` / `Op::Fuse` / `Op::Cut` arms grow the
  `buildConeSphere{Common,Fuse,Cut}` calls and the engine gains the cone∩sphere COMMON oracle arm
- THEN each existing family produces its byte-identical result (same volume, area, and vertices as
  before this change) — `buildConeSphere{Common,Fuse,Cut}` return `{}` for every non-(cone +
  coaxial-sphere) pair and the cone∩sphere oracle arm returns not-applicable for them, so the
  existing native passes do not regress.

#### Scenario: cone∩cone remains the OCCT boundary (host)
- GIVEN a cone∩cone pair with `Op::Common`, `Op::Fuse`, or `Op::Cut`
- WHEN `ssi_boolean_solid` runs
- THEN `buildConeSphere{Common,Fuse,Cut}` decline (the gate requires one `Cone` + one coaxial
  `Sphere`) and return a NULL `Shape`, and the engine ships OCCT `BRepAlgoAPI_{Common,Fuse,Cut}` —
  the coaxial cone∩cylinder and coaxial cone∩sphere are the only cone families with native booleans.
