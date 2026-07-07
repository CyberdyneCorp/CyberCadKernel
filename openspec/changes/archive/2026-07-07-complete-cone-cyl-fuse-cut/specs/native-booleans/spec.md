# native-booleans

Complete the coaxial cone∩cylinder curved-boolean op-set in
`src/native/boolean/ssi_boolean.cpp` (`openspec/SSI-ROADMAP.md` S5-e). The COMMON of a coaxial
frustum cone and cylinder is ALREADY native (`buildConeCylCommon` — the min-radius-profile solid of
revolution: the cone band inside the cylinder welded along the single S1 analytic seam circle to
the cylinder band inside the cone, closed by two disc caps, verified against the closed-form volume
`V_frustum(r(sLo) → Rc) + π Rc²·(sHi − s*)`). This change adds native **Fuse** and **Cut** for the
SAME analytic seam circle — the same crossing at `s*` with a DIFFERENT band selection plus the
operand end caps and the annular steps — so the family is 3/3 native.

Both A (a frustum cone, `r_c(s) = R0 + s·tanα`) and B (a coaxial cylinder, radius `Rc`) are solids
of revolution about the shared axis, so `A ∩ B`, `A ∪ B`, and `A − B` are all solids of revolution
described by a radial profile over `s`. The seam circle at `s*` (`r_c(s*) = Rc`) splits each wall
into an INSIDE-the-other band and an OUTSIDE band; each operand carries its two original disc end
caps at its axial extents. COMMON = the `min(r_c, Rc)` inside bands (the overlap). FUSE = the
`max(r_c, Rc)` outer bands over the full union extent (the wider operand's wall on each side, welded
at the seam) + the beyond-overlap wall segments + the two terminal disc caps + the annular step caps
where one operand's end-cap disc protrudes past the other. CUT `A − B` = A's outer wall (outside B)
+ A's disc caps + A's cap-annulus outside B, joined to B's INSIDE band emitted REVERSED (inward
radial normal, bounding the carved cavity, pinching to the seam circle) + B's cap disc inside A
emitted reversed. All three share the SAME single S1 analytic seam circle and weld watertight
through one `VertexPool` with the planar-facet revolve discipline (`appendRevolvedBand`,
`appendDiskCap`, and a new flat annular-cap helper). CUT is order-sensitive and may be a
DISCONNECTED result (assembled as one shell of two closed components sharing the pool).

Both new ops DECLINE (NULL → OCCT) outside their verified envelope: an apex-crossing seam or a
frustum whose extent reaches the apex (S4-e territory), a non-coaxial (transversal) cone∩cylinder
pair (a quartic space curve, not analytic here), a cap-edge-tangent seam, coaxial cone∩sphere, and
cone∩cone all remain the OCCT boundary. Internal: **no `cc_*` ABI change** — invoked behind the
existing `cc_boolean` op codes. `src/native/**` stays OCCT-free; the path is compiled under
`CYBERCAD_HAS_NUMSCI`. No change to `src/native/tessellate`, the planar BSP-CSG, the analytic
`curved.h`, the cyl / sphere / Steinmetz builders, the `buildConeCylCommon` COMMON path (which stays
byte-identical), or the engine self-verify (the generic set-algebra guard already covers fuse/cut
with the correct per-op sign against the native cone∩cylinder COMMON).

## ADDED Requirements

### Requirement: SSI-driven native Fuse for the coaxial cone∩cylinder pair

The native boolean library SHALL compute `cc_boolean(a, b, op=0)` fuse (`A ∪ B`) NATIVELY for the
COAXIAL cone∩cylinder pair the S5-e COMMON path (`buildConeCylCommon`) already recognises: one
operand recognised as a `Cone` frustum solid (via `recogniseCurvedSolid`), the other as a
`Cylinder` solid whose axis is COLLINEAR with the cone axis (`sameAxis`), whose S3/S1 seam trace is
EXACTLY ONE closed full-circle seam on BOTH walls (the S1 analytic circle from
`intersectCylinderConeCoaxial`, `nearTangentGaps == 0`, `branchPoints == 0`), where the frustum is
APEX-FREE over its extent and the seam height `s*` (where the cone cross-section radius `r_c(s*)`
equals the cylinder radius `Rc`) lies STRICTLY inside the axial overlap `[sLo, sHi]`.

The builder SHALL reuse the SAME shared gate/seam prologue (`coneCylSetup`: the coaxial gate, the
analytic-vs-traced seam cross-check, the axis frame, the crossing `s*`, the azimuth resolution, and
the pooled seam ring at `(Rc, s*)`), the SAME shared `VertexPool` weld with the seam ring pooled
ONCE, and the SAME planar-facet revolve discipline (`appendRevolvedBand`, `appendDiskCap`) as
`buildConeCylCommon`, and SHALL assemble the union boundary as the MAX-radius outer profile
`max(r_c(s), Rc)` over the union extent `[min(coneLo, cylLo), max(coneHi, cylHi)]`: the OUTER wall
of whichever operand is wider on each side of the seam (the cylinder wall on the cone-inner side,
the cone wall on the cyl-inner side, welded along the single seam circle), PLUS the wall segments
beyond the overlap (each operand's wall where the other is absent), closed by the two terminal disc
caps (outward `∓ẑ`) AND the annular step caps (flat washers with axial `±ẑ` normal) where one
operand's end-cap disc protrudes past the other's wall radius.

The builder SHALL keep each outer wall band only if its interior sample classifies strictly OUTSIDE
the other solid (the fuse survival rule, via the S5-a curved point-in-solid test `classifyPoint`);
a sample robustly ON the other wall (tangent, `classifyPoint == 0`), a frustum whose extent reaches
the apex, a non-coaxial (transversal) pair, a seam that is not exactly one strictly-interior full
circle, or a weld that cannot close SHALL return a NULL `Shape` (→ OCCT). The tolerance SHALL NOT be
weakened to force a pass.

The result SHALL be a native `topology::Shape` of type `Solid` carrying true `Cone` (frustum band)
and `Cylinder` (cylinder band) wall face kinds and planar (disc + annular cap) face kinds,
watertight (every edge shared by exactly two faces), whose enclosed volume equals `vol(A) + vol(B)
− vol(A ∩ B)` within a relative tolerance sized to the curved-face tessellation deflection, where
`vol(A ∩ B)` is the native cone∩cylinder COMMON (`buildConeCylCommon`). The builder SHALL remain
OCCT-free and reference no OCCT / `IEngine` / `EngineShape` type, SHALL be compiled under
`CYBERCAD_HAS_NUMSCI`, and SHALL add or change no `cc_*` entry point, signature, or POD struct.

#### Scenario: The coaxial cone∩cylinder fuse grows to the outer envelope with the correct volume (host)
- GIVEN a frustum cone A (`r_c(s) = R0 + s·tanα`, apex-free over its extent) and a coaxial cylinder
  B (radius `Rc`) whose walls cross at a single circle at `s*` (`r_c(s*) = Rc`) strictly inside the
  axial overlap, with a clean single-circle seam trace (`nearTangentGaps == 0`, `branchPoints == 0`)
- WHEN `ssi_boolean_solid(A, B, Op::Fuse)` runs (the through-drill `buildFuse` declines the single
  seam, the sphere `buildLensFuse` declines the non-sphere operand, and `buildConeCylFuse` assembles
  the outer shell)
- THEN it returns a watertight `Solid` (`boundaryEdgeCount == 0`, every edge shared by exactly two
  faces) bounded by the max-radius outer profile — the wider operand's wall on each side of the seam
  circle, the beyond-overlap wall segments, the two terminal disc caps, and the annular step caps
  where an end-cap disc protrudes
- AND its enclosed volume equals `vol(A) + vol(B) − vol(A ∩ B)` within the deflection-sized band (for
  the reference fixture `r_c(y) = 0.5 + 0.5y`, `Rc = 1.5`, cone `[0,4]`, cyl `[1,5]`: `s* = 2`,
  volume ≈ `41.626`) — a GROW (`Vr > max(vol(A), vol(B))`)
- AND every seam-ring node lies on BOTH walls within tolerance, and the seam ring is pooled ONCE
  (shared by the cylinder outer band below `s*` and the cone outer band above `s*`).

#### Scenario: An apex-crossing / transversal / cap-tangent cone pair declines fuse to OCCT (host)
- GIVEN a coaxial cone∩cylinder pair whose frustum extent reaches the apex (`r_c → 0`), OR a NON-
  coaxial (transversal) cone∩cylinder pair whose seam is a quartic space curve
  (`intersectCylinderConeCoaxial` returns `notAnalytic`), OR a pair whose seam `s*` sits on a cap
  edge (a tangent, not a strictly-interior transversal circle)
- WHEN `ssi_boolean_solid(A, B, Op::Fuse)` runs
- THEN `buildConeCylFuse` refuses at the shared gate (apex, non-coaxial, or ON-edge seam) and
  returns a NULL `Shape`, and the engine falls through to OCCT `BRepAlgoAPI_Fuse` — reported, not
  faked, tolerance not weakened.

#### Scenario: The engine discards a wrong-volume cone∩cylinder fuse candidate (host)
- GIVEN a fuse candidate whose welded shell volume does not match `vol(A) + vol(B) − vol(A ∩ B)` (a
  mis-selected outer band or a mis-placed annular cap)
- WHEN the engine's generic set-algebra self-verify runs (`expected = va + vb − vc`, `vc` = native
  cone∩cylinder COMMON `buildConeCylCommon`; the `op == 2`-only analytic oracle does NOT intercept
  fuse)
- THEN the candidate FAILS the watertight + correct-volume guard and is DISCARDED → OCCT; the engine
  never emits an unverified cone∩cylinder fuse.

### Requirement: SSI-driven native Cut for the coaxial cone∩cylinder pair

The native boolean library SHALL compute `cc_boolean(a, b, op=1)` cut (`A − B`, `A` the minuend)
NATIVELY for the coaxial cone∩cylinder pair the S5-e COMMON path already recognises (one `Cone`
frustum + one coaxial `Cylinder`, a single strictly-interior full-circle apex-free seam,
`nearTangentGaps == 0`). It SHALL reuse the SAME shared gate/seam prologue (`coneCylSetup`),
`VertexPool` (seam ring pooled once), and planar-facet revolve discipline as `buildConeCylCommon`,
and SHALL assemble the difference boundary as A's OUTER wall bands (the part of A outside B, outward
radial normal) plus A's terminal disc cap(s) outside B plus A's cap-annulus where A's end-cap disc
extends past B, plus B's INSIDE-A wall band emitted REVERSED (inward radial normal, welded at the
pooled seam circle so it bounds the carved cavity) plus B's end-cap disc inside A emitted REVERSED
(the cavity floor/ceiling). Operand order SHALL be honoured (CUT is not symmetric), matching
`BRepAlgoAPI_Cut(a, b)`. The result MAY be DISCONNECTED (two closed components — e.g. a small end
frustum plus a conical washer); it SHALL be assembled as one shell whose components share the pool
and whose summed mesh volume is verified.

The builder SHALL keep each A outer band only if its interior sample classifies strictly OUTSIDE B
AND each reversed B inside band only if its interior sample classifies strictly INSIDE A (the cut
survival rule, via `classifyPoint`); a tangent / degenerate / wrong-side sample (`classifyPoint ==
0`), a frustum whose extent reaches the apex, a non-coaxial (transversal) pair, a cap-edge-tangent
seam, or a weld that cannot close SHALL return a NULL `Shape` (→ OCCT). The tolerance SHALL NOT be
weakened.

The result SHALL be a native `topology::Shape` `Solid` with true `Cone` (wall) and `Cylinder`
(wall) and planar (disc + annular cap) face kinds, watertight per component (every edge shared by
exactly two faces), whose enclosed (summed) volume equals `vol(A) − vol(A ∩ B)` within the
deflection-sized relative tolerance, where `vol(A ∩ B)` is the native cone∩cylinder COMMON. The
builder SHALL remain OCCT-free, reference no OCCT / `IEngine` / `EngineShape` type, be compiled
under `CYBERCAD_HAS_NUMSCI`, and add or change no `cc_*` entry point, signature, or POD struct.

#### Scenario: The coaxial cone∩cylinder cut carves the cavity with the correct volume (host)
- GIVEN two coaxial operands A (a frustum cone, minuend) and B (a cylinder) whose walls cross at a
  single circle at `s*` strictly inside the axial overlap, with a clean single-circle seam trace
  (`nearTangentGaps == 0`)
- WHEN `ssi_boolean_solid(A, B, Op::Cut)` runs (the through-drill `buildCut` and sphere
  `buildLensCut` decline, and `buildConeCylCut` assembles the shell)
- THEN it returns a watertight `Solid` bounded by A's OUTER wall bands (outside B, outward) + A's
  disc cap(s) + A's cap-annulus outside B, plus B's INSIDE-A wall band REVERSED (inward, pinching to
  the seam circle) + B's cap disc inside A REVERSED (the cavity floor/ceiling), all sharing the
  single seam circle
- AND its enclosed (summed over components — e.g. an end frustum + a conical washer) volume equals
  `vol(A) − vol(A ∩ B)` within the deflection-sized band (for the reference fixture ≈ `13.352`) — a
  SHRINK (`Vr < vol(A)`)
- AND every seam-ring node lies on both walls within tolerance, and the seam ring is pooled ONCE.

#### Scenario: The reversed inner band bounds the cavity, verified against the native common (host)
- GIVEN the coaxial cone∩cylinder cut candidate above
- WHEN the engine's generic set-algebra self-verify runs (`expected = va − vc`, `vc` = native
  cone∩cylinder COMMON `buildConeCylCommon`; the `op == 2`-only analytic oracle does NOT intercept
  cut)
- THEN a candidate whose reversed INSIDE-B band is mis-oriented (outward, not bounding the cavity)
  yields the wrong enclosed volume, FAILS the guard, and is DISCARDED → OCCT — the correct candidate
  matches `vol(A) − vol(A ∩ B)` and is accepted native.

### Requirement: The COMMON path and other pairs are unchanged by the cone∩cylinder fuse/cut completion

The native boolean library SHALL keep the coaxial cone∩cylinder COMMON (`buildConeCylCommon`) and
every other curved-boolean family byte-identical when the fuse/cut completion lands. Factoring the
shared gate/seam prologue into `coneCylSetup` SHALL NOT change the COMMON result (same volume, area,
and vertices), and `buildConeCylFuse` / `buildConeCylCut` SHALL return a NULL `Shape` for every non-
(cone + coaxial-cylinder) pair so the through-drill cyl∩cyl, sphere∩sphere lens, and Steinmetz
bicylinder builders and all their ops keep their existing results. The dispatch SHALL grow only the
`Op::Fuse` and `Op::Cut` arms (one final call each after the through-drill and lens builders
decline); recognition, tracing, the transversality gate, and the engine self-verify SHALL NOT
change.

#### Scenario: The COMMON path is unchanged by the fuse/cut completion (host)
- GIVEN the SAME coaxial cone∩cylinder pair
- WHEN `ssi_boolean_solid(A, B, Op::Common)` runs after the shared gate/seam prologue is factored
  into `coneCylSetup`
- THEN `buildConeCylCommon` produces the byte-identical min-radius-profile common (same volume
  `vol(A ∩ B)`, area, and vertices as before this change) — the COMMON native pass does not regress.

#### Scenario: The cyl / sphere / Steinmetz families are unchanged by the cone∩cylinder fuse/cut addition (host)
- GIVEN the existing through-drill cyl∩cyl, sphere∩sphere lens, and Steinmetz bicylinder fixtures
  across all three ops
- WHEN `ssi_boolean_solid` runs after the `Op::Fuse` / `Op::Cut` arms grow the `buildConeCylFuse` /
  `buildConeCylCut` calls
- THEN each existing family produces its byte-identical result (same volume, area, and vertices as
  before this change) — `buildConeCylFuse` / `buildConeCylCut` return `{}` for every non-(cone +
  coaxial-cylinder) pair, so the existing native passes do not regress.

#### Scenario: Coaxial cone∩sphere and cone∩cone remain the OCCT boundary (host)
- GIVEN a coaxial cone∩sphere pair OR a cone∩cone pair, with `Op::Fuse` or `Op::Cut`
- WHEN `ssi_boolean_solid` runs
- THEN `buildConeCylFuse` / `buildConeCylCut` decline (the gate requires one `Cone` + one coaxial
  `Cylinder`) and return a NULL `Shape`, and the engine ships OCCT `BRepAlgoAPI_{Fuse,Cut}` — the
  coaxial cone∩cylinder is the only cone family with native fuse/cut.
