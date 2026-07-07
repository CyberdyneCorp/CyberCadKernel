# native-construction

This change (Phase 4 #4 native construction, the Tier-4 BREADTH batch) extends the living
`native-construction` capability with three tracks: (T1) a MISMATCHED-count RULED LOFT
(`cc_solid_loft` / `cc_solid_loft_wires` between an M-gon and an N-gon, `M ≠ N`, via an
arc-length-parameter vertex correspondence) — LANDED, EXACT for polygons; (T2) an
ORIENTATION-constraining GUIDED SWEEP (`cc_guided_sweep` with the section aimed at the guide)
OR its documented HONEST DECLINE; and (T3) a FINE-PITCH SELF-INTERSECTING THREAD
(`cc_helical_thread` / `cc_tapered_thread` whose radial-V flanks cross in 3D) OR its documented
HONEST DECLINE. All three land behind EXISTING facade entries — no new ABI. Every native result
is accepted ONLY through the engine's mandatory watertight + positive-volume self-verify and
DISCARDED → OCCT on failure. Everything outside the landed slices returns NULL → OCCT, and the
measured gap is REPORTED, never masked with a weakened tolerance.

## ADDED Requirements

### Requirement: Native mismatched-count ruled loft (vertex correspondence)

The native construction library SHALL compute a RULED LOFT `cc_solid_loft` /
`cc_solid_loft_wires` (and the N-section `build_loft_sections` chain) NATIVELY between two (or
more) CLOSED PLANAR polygon sections whose vertex counts DIFFER (an M-gon lofted to an N-gon,
`M ≠ N`), and SHALL return a NULL Shape for the cases it cannot robustly build so the engine
defers to the OCCT `BRepOffsetAPI_ThruSections` oracle.

The builder SHALL make the sections' vertex counts match BEFORE pairing by an ARC-LENGTH
PARAMETER correspondence: for each loop it SHALL compute the normalized cumulative arc-length
parameter of every vertex, resample BOTH loops at the sorted UNION of the loops' parameters
(inserting a point ON the straight edge of whichever loop lacks a given parameter), yielding
equal-count loops (count `K ≤ M + N`), and feed those to the existing equal-count aligner
(`detail::alignSectionB`, the rotational-start + traversal-direction pairing) and ruled-band
assembler (`detail::appendRuledBand`, one bilinear side face per corresponding edge pair). For
an N-section chain the correspondence SHALL propagate PAIRWISE down the chain (each section
resampled against its already-fixed predecessor). Because an inserted point at a parameter on a
STRAIGHT polygon edge is COLLINEAR, each resampled loop SHALL trace the SAME planar polygon as
its input (identical vertices in order, identical straight edges, identical area) — the resample
SHALL be GEOMETRY-PRESERVING — so the ruled surface between the resampled loops is the SAME
ruled surface between the original M-gon and N-gon, merely tiled with more bilinear bands, and
the enclosed volume SHALL EQUAL the true ruled-loft volume between the two polygons within the
tessellation deflection bound (EXACT for polygonal sections). This mirrors the compatibility
step (`BRepFill_CompatibleWires`) OCCT `ThruSections` runs before ruling; nothing is copied.

The EQUAL-count 2- and N-section ruled loft SHALL be the `K = M = N` special case of the same
builder and SHALL remain byte-identical in behaviour. The section guards SHALL be UNCHANGED:
every section PLANAR and non-degenerate (≥ 3 distinct points, non-null Newell normal); a
NON-PLANAR end section, a PUNCTUAL (all-coincident) section, a section with < 3 distinct points,
a zero-perimeter loop, or a chain whose RESAMPLED stacked skin SELF-INTERSECTS SHALL return a
NULL Shape → OCCT. The result SHALL be a native `topology::Shape` of type `Solid`, watertight,
accepted ONLY after the engine's mandatory watertight + positive-volume self-verify
(`robustlyWatertight` + `watertightVolume > 0`) — else DISCARDED → OCCT. This builder SHALL
remain OCCT-free and host-buildable and SHALL reference no OCCT / `IEngine` / `EngineShape` type;
`cc_solid_loft` / `cc_solid_loft_wires` signatures and POD layouts SHALL NOT change.

#### Scenario: An M-gon lofts to an N-gon watertight with the exact ruled-loft volume (host)

- GIVEN two closed PLANAR polygon sections with DIFFERENT vertex counts (an M-gon and an N-gon, `M ≠ N`, e.g. a triangle and a hexagon), built on the host with no OCCT
- WHEN `cc_solid_loft` (or `cc_solid_loft_wires`) is computed and the result is tessellated by `src/native/tessellate`
- THEN the arc-length correspondence SHALL resample both loops to a common vertex count preserving each polygon's shape and area, the result SHALL be a watertight `Solid` (`boundaryEdgeCount == 0`) of ruled bands + two planar end caps, AND its enclosed volume SHALL EQUAL the analytic ruled-loft volume between the M-gon and the N-gon within the tessellation deflection bound

#### Scenario: The equal-count ruled loft is the byte-identical special case (host)

- GIVEN two sections with the SAME vertex count `n` (the equal-count 2- or N-section loft)
- WHEN the mismatched-count builder runs the correspondence
- THEN the parameter union SHALL reduce to each section's own parameter set (`K = n`), the resampled loops SHALL equal the inputs, and the built solid SHALL be byte-identical to the existing equal-count ruled loft

#### Scenario: Native mismatched-count loft matches the OCCT ThruSections oracle (parity)

- GIVEN an M-gon→N-gon loft (`M ≠ N`) on a booted iOS simulator
- WHEN `cc_solid_loft` / `cc_solid_loft_wires` is called with the native engine active (`cc_set_engine(1)`) and the OCCT side builds `BRepOffsetAPI_ThruSections` (ruled) on the same two sections
- THEN the two shapes' mass properties, bounding box, and watertightness SHALL agree within the tessellation deflection bound (a TIGHT bound, since the polygon resample is EXACT); a fixture whose measured gap exceeds the bound SHALL be declared out of slice (NULL → OCCT) with the gap REPORTED

#### Scenario: A non-planar, punctual, or self-intersecting section defers to OCCT (host)

- GIVEN a loft whose end section is NON-PLANAR, PUNCTUAL (all points coincident), has < 3 distinct points, or whose resampled stacked skin SELF-INTERSECTS, with the native engine active
- WHEN `cc_solid_loft` / `cc_solid_loft_wires` is invoked
- THEN the native builder SHALL return a NULL Shape (or the engine self-verify SHALL discard the candidate) AND the engine SHALL defer to the OCCT `BRepOffsetAPI_ThruSections` oracle — it SHALL NOT emit an approximate or self-intersecting loft, and the measured gap SHALL be REPORTED

### Requirement: Native orientation-constraining guided sweep or honest decline (`cc_guided_sweep`)

The native construction library SHALL compute `cc_guided_sweep` with an ORIENTATION-constraining
guide — the swept section AIMED at the guide (its up-axis derived from the spine→guide
direction, the `BRepOffsetAPI_MakePipeShell` `SetMode(guideWire)` law) — NATIVELY for the
narrowest reproducible slice (a PLANAR spine and a PLANAR guide in the same plane, no twist)
ONLY IF such a solid can be built watertight AND matched to a REAL OCCT guide-orientation oracle
reachable behind the existing `cc_guided_sweep` entry; otherwise the orientation-constraining
guided sweep SHALL be an HONEST DECLINE (a NULL Shape → OCCT), with NO always-NULL dead builder
retained, and the measured OCCT-fallback gap SHALL be REPORTED. The existing SCALE-constraining
guided sweep (each station uniformly scaled by the guide splay `dist(path,guide)/d0`, matched to
the OCCT scale-splay `guided_sweep` oracle) SHALL remain native and UNCHANGED.

When landed, the orientation frame SHALL be a PURE per-station function of the local geometry
(up-axis `= normalize((guide(f) − path(f)) − tangent·((guide−path)·tangent))`,
`nrm = tangent × up`, section spanning `(nrm, up)` centred at `path(f)`) with NO accumulated
corrected-Frenet rotation, and the tube SHALL be ruled + capped watertight through the existing
`assembleRingTube`. The builder SHALL SELF-VERIFY watertightness and that the section up-axis
tracks the guide at every station; a NON-PLANAR spine or guide, an accumulated-twist guide, a
`guide − path` parallel to the tangent (the up-axis collapses), or an orientation solid that
does not match the real OCCT guide-orientation oracle SHALL return a NULL Shape → OCCT. The
builder SHALL remain OCCT-free and SHALL reference no OCCT / `IEngine` / `EngineShape` type; the
OCCT `BRepOffsetAPI_MakePipeShell` is the verification ORACLE only. The change SHALL NOT retain
an always-NULL builder purely to signal the track — an untractable T2 (no orientation oracle
behind the fixed scale-splay `cc_guided_sweep` semantics, which the project's no-ABI-break
discipline forbids overriding) is a documented OCCT-fallthrough with the gap REPORTED, not dead
code. `cc_guided_sweep`'s signature and POD layout SHALL NOT change.

#### Scenario: A reproducible planar orientation-guided sweep builds natively and is watertight (host)

- GIVEN a PLANAR spine and a PLANAR guide in the same plane with `guide − path` never parallel to the tangent, and a guide-orientation oracle reachable behind `cc_guided_sweep`, built on the host
- WHEN `cc_guided_sweep` is computed and the orientation slice is landed
- THEN the result SHALL be a watertight `Solid` whose every station section is AIMED at the guide (its up-axis equal to the guide direction projected perpendicular to the tangent), self-verified before assembly

#### Scenario: An untractable orientation-guided sweep declines honestly to OCCT (host + parity)

- GIVEN an orientation-constraining guided sweep for which no real OCCT guide-orientation oracle is reachable behind the fixed scale-splay `cc_guided_sweep` entry (adopting an orientation frame would break its semantics or fail parity vs the shipped scale oracle), OR an input outside the planar slice (non-planar spine/guide, accumulated twist, `guide ∥ tangent`), with the native engine active
- WHEN `cc_guided_sweep` is invoked
- THEN the native builder SHALL return a NULL Shape (or the track SHALL be a documented OCCT-fallthrough with no dead builder) AND the operation SHALL defer to the OCCT `BRepOffsetAPI_MakePipeShell` / scale-splay oracle, with the measured gap REPORTED — never a faked or parity-mismatched solid

### Requirement: Native fine-pitch self-intersecting thread or honest decline (`cc_helical_thread` / `cc_tapered_thread`)

The native construction library SHALL compute `cc_helical_thread` / `cc_tapered_thread`
NATIVELY for the narrowest ROBUST FINE-PITCH SELF-INTERSECTING slice — a thread whose adjacent
radial-V FLANKS cross in 3D at a steep helix lead (just past the `kMaxLeadRatio` fold guard) —
ONLY IF the crossing flanks can be TRIMMED to their intersection curve and the result built
watertight with the correct volume; otherwise the self-intersecting thread SHALL be an HONEST
DECLINE (a NULL Shape → OCCT) with NO always-NULL dead builder retained, and the measured
OCCT-fallback gap SHALL be REPORTED. The existing near-touching regime (turns whose V bases MEET,
resolved by the `resolveHalfBase` root flat) SHALL remain native and UNCHANGED.

When landed, the two overlapping flank helicoids SHALL be intersected and each TRIMMED to their
intersection curve (surface-surface intersection, `CYBERCAD_HAS_NUMSCI`-gated), re-tiled into
bilinear ruled bands + planar caps, and the slice SHALL be restricted to a shallow taper with
the root clear of the axis. The builder SHALL SELF-VERIFY watertightness and a volume consistent
with the analytic ridge (NOT the un-trimmed self-overlapping volume). When the crossing-flank
case cannot be built watertight with the correct volume without full Tier-4 SSI, when NUMSCI is
unavailable, or when the fold is beyond the narrow slice (deeper crossings, root-dive taper), the
builder SHALL return a NULL Shape → OCCT and the `kMaxLeadRatio` fold guard SHALL stay in force.
The builder SHALL remain OCCT-free and SHALL reference no OCCT / `IEngine` / `EngineShape` type;
the OCCT `BRepOffsetAPI_MakePipeShell` (aux-spine radial sweep) is the verification ORACLE only.
The change SHALL NOT retain an always-NULL builder purely to signal the track — an untractable T3
is a documented OCCT-fallthrough with the gap REPORTED, not dead code. `cc_helical_thread` /
`cc_tapered_thread` signatures and POD layouts SHALL NOT change.

#### Scenario: A robust crossing-flank fine-pitch thread builds natively and is watertight (host, NUMSCI-ON)

- GIVEN a fine-pitch thread whose adjacent radial-V flanks cross in 3D just past `kMaxLeadRatio`, with a shallow taper and the root clear of the axis, built on the host with NUMSCI ON
- WHEN `cc_helical_thread` (or `cc_tapered_thread`) is computed and the crossing-flank slice is landed
- THEN the overlapping flank helicoids SHALL be trimmed to their intersection curve, the result SHALL be a watertight `Solid`, AND its enclosed volume SHALL equal the analytic ridge volume (not the un-trimmed self-overlapping volume) within the tessellation deflection bound

#### Scenario: An untractable self-intersecting thread declines honestly to OCCT (host + parity)

- GIVEN a self-intersecting fine-pitch thread OUTSIDE the robust slice (deeper flank crossings, a root-dive taper, or a build with NUMSCI unavailable), with the native engine active
- WHEN `cc_helical_thread` / `cc_tapered_thread` is invoked
- THEN the native builder SHALL return a NULL Shape (or the track SHALL be a documented OCCT-fallthrough with no dead builder, the `kMaxLeadRatio` guard in force) AND the operation SHALL defer to the OCCT `BRepOffsetAPI_MakePipeShell` oracle, with the measured gap REPORTED — never a faked, self-overlapping, or volume-wrong solid

### Requirement: Tier-4 construction breadth is self-verified before it is kept native

The native builder SHALL, before any Tier-4 breadth construct (the mismatched-count loft, the
orientation-guided sweep, or the self-intersecting thread) is kept native, run the engine's
MANDATORY self-verify — `robustlyWatertight` across the deflection ladder AND a positive
`watertightVolume` (plus, where an analytic reference exists, the correct volume) — and SHALL
DISCARD the candidate (forward the SAME arguments to OCCT) if it fails. The builder SHALL NEVER
weaken a tolerance to pass, ship a leaky or volume-wrong native body, or fabricate a solid
outside its slice. This is a #4 instance of the roadmap's mandatory self-verify → OCCT-fallback
discipline, unchanged from the existing construction family.

#### Scenario: A Tier-4 construct that fails the self-verify falls through to OCCT (host)

- GIVEN a Tier-4 breadth construct (a mismatched-count loft whose resampled skin self-intersects, an orientation-guided sweep that fails parity, or a self-intersecting thread whose trimmed solid is not watertight or is volume-wrong), built on the host
- WHEN the corresponding native builder runs and the engine self-verify (`robustlyWatertight` + positive/correct volume) is applied
- THEN the candidate SHALL be DISCARDED and the engine SHALL forward the SAME arguments to the OCCT oracle — it SHALL NOT keep an unverified native body, and the measured gap SHALL be REPORTED

## MODIFIED Requirements

### Requirement: Deferred construction ops fall through to OCCT (honest scope)

The following construction operations SHALL fall through the `NativeEngine` to the fallback
(OCCT) engine: `wrap_emboss`. (`solid_sweep` — for a straight / smooth-planar / non-planar
(RMF) spine — `solid_loft` / `solid_loft_wires` — for a 2- or N-section EQUAL-count OR
MISMATCHED-count planar chain — `tapered_shank`, and the well-formed `helical_thread` /
`tapered_thread` are NOW native where their result self-verifies as a watertight, oracle-correct
solid; see the native residual requirements. `twisted_sweep` with a REAL twist/scale,
`guided_sweep` for the SCALE constraint (native) and its ORIENTATION extension, and
`loft_along_rail` were ATTEMPTED but land natively only where they self-verify oracle-correct and
otherwise fall through per the sub-cases below.) These construction ops are **attempted
natively** and are native ONLY where the result is verified a valid watertight solid with the
correct volume/geometry on BOTH gates; OTHERWISE they SHALL fall through to OCCT (labelled,
verified, never faked). In addition, these **sub-cases** SHALL fall through (the native builder
returns a NULL `Shape`, or the `NativeEngine` self-verify DISCARDS the candidate, and the engine
delegates):

- **Surface–surface intersection (SSI) — Tier 4, only the named slices attempted.** A
  **self-intersecting sweep** (the swept surface folds through itself), a **tight-curvature
  spine** whose section folds (curvature radius below the section's radial extent × the applied
  scale), a **hard pipe-shell rail** that cannot close without trimming two swept surfaces, and a
  **self-intersecting thread whose flanks cross in 3D beyond the robust crossing-flank slice**
  (the crest of one turn passes the next and the trimmed solid does not self-verify watertight +
  correct volume) SHALL fall through. This change attempts SSI ONLY for the named narrow slices
  (the crossing-flank thread, `CYBERCAD_HAS_NUMSCI`-gated) and declines the rest.
- **Sweeps:** a `twisted_sweep` beyond the twist/scale envelope (accumulated twist × section
  radius exceeding the station spacing so adjacent rings interpenetrate) and a self-crossing
  spine SHALL fall through. An ORIENTATION-constraining `guided_sweep` outside the reproducible
  planar slice — a non-planar spine or guide, an accumulated-twist guide, a `guide − path`
  parallel to the tangent, or (the realistic case) no real orientation oracle behind the fixed
  scale-splay `cc_guided_sweep` semantics — SHALL fall through (an honest decline, no dead code).
- **Loft:** a chain with a **NON-PLANAR end section**, a **punctual section**, a section with
  **< 3 distinct points**, or a **resampled stacked skin that self-intersects** SHALL fall
  through. A chain with **mismatched vertex counts** is NOW native via the arc-length vertex
  correspondence (an M-gon lofts to an N-gon, geometry-preserving, self-verified) — it is no
  longer in the fall-through-only list.
- **Profile ops:** a **spline-revolve** (a kind-3 segment in `solid_revolve_profile`, or a
  general B-spline surface of revolution — NOT attempted) and a **non-planar / self-
  intersecting spline** loop in `solid_extrude_profile` /
  `solid_extrude_profile_polyholes` SHALL fall through. (A kind-3 spline edge in a PLANAR
  extrude and a kind-1 off-axis-arc → TORUS revolve are native.)
- **Threads:** a thread requiring the **round-profile fallback** (the native builder returns
  NULL and OCCT applies that fallback — the native path does not fake it) SHALL fall through; the
  tapered tip-crossing and degenerate guards are unchanged, and a **crossing-flank fine-pitch
  thread beyond the robust slice** falls through (the `kMaxLeadRatio` fold guard in force).

Likewise every feature / boolean / tessellate-of-a-foreign-body / query / transform /
exchange op SHALL fall through. The change SHALL NOT fake, stub-out, or partially implement
any deferred op or sub-case, and SHALL attempt SSI ONLY for the named narrow slices; each
deferred call SHALL produce exactly the fallback engine's result.

#### Scenario: A deferred construction op yields the OCCT result through the native engine
- GIVEN the native engine active (`cc_set_engine(1)`) on an OCCT build
- WHEN a deferred op is invoked (e.g. `cc_wrap_emboss`)
- THEN the result SHALL be identical to invoking the same call with the OCCT engine active (`cc_set_engine(0)`), proving fall-through with no native interception

#### Scenario: An SSI sub-case outside the named slices falls through
- GIVEN the native engine active and an input whose valid B-rep would require surface–surface intersection outside the named slices (a self-intersecting sweep, a tight-curvature fold, a hard pipe-shell rail, or a self-intersecting thread whose trimmed solid does not self-verify)
- WHEN the corresponding `cc_*` op is invoked
- THEN the native builder SHALL return NULL (or the self-verify DISCARDS the candidate) AND the call SHALL fall through to OCCT unchanged (identical to `cc_set_engine(0)`), with no unverified SSI kept

#### Scenario: A deferred loft sub-case falls through
- GIVEN the native engine active and a loft chain with a NON-PLANAR end section, a punctual section, or a resampled skin that self-intersects (a MISMATCHED vertex count alone is NOT deferred — it is native via the correspondence)
- WHEN `cc_solid_loft` / `cc_solid_loft_wires` is invoked
- THEN the native builder SHALL return NULL AND the call SHALL fall through to OCCT unchanged (identical to `cc_set_engine(0)`)

#### Scenario: A deferred guided-sweep sub-case falls through
- GIVEN the native engine active and an orientation-constraining `cc_guided_sweep` outside the reproducible planar slice, or with no real orientation oracle behind the entry
- WHEN `cc_guided_sweep` is invoked
- THEN the native builder SHALL return NULL (an honest decline, no dead code) AND the call SHALL fall through to OCCT unchanged (identical to `cc_set_engine(0)`)

#### Scenario: A deferred profile sub-case falls through
- GIVEN the native engine active and an input hitting a deferred profile sub-case (a spline-revolve, or a non-planar / self-intersecting spline extrude loop)
- WHEN the corresponding `cc_solid_revolve_profile` / `cc_solid_extrude_profile` / `cc_solid_extrude_profile_polyholes` call is invoked
- THEN the native builder SHALL return NULL AND the call SHALL fall through to OCCT unchanged (identical to `cc_set_engine(0)`)

#### Scenario: A deferred thread sub-case falls through
- GIVEN the native engine active and a thread requiring the round-profile fallback OR a self-intersecting fine-pitch thread whose flanks cross beyond the robust crossing-flank slice
- WHEN `cc_helical_thread` / `cc_tapered_thread` is invoked
- THEN the native builder SHALL return NULL (no faked round profile, no unverified SSI) AND the call SHALL fall through to OCCT unchanged (identical to `cc_set_engine(0)`)
