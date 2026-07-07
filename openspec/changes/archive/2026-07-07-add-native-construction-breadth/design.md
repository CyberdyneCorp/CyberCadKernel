# Design — add-native-construction-breadth (#4 native construction, Tier-4 breadth)

## Context

The native construction family (`src/native/construct/`) skins closed sections and profiles
into watertight solids through one shared substrate: the bilinear ruled side face
(`detail::ruledSideFace`), the planar cap (`detail::planarFace`), the section aligner
(`detail::alignSectionB`), the sweep frames (`detail::constantFrames` /
`frenetSectionFrames` / `rmfFrames`), and the radial-V thread tiler (`detail::vStationAt`).
Every result is accepted only through the engine's mandatory self-verify — `robustlyWatertight`
(watertight across a deflection ladder) plus a positive `watertightVolume` — and DISCARDED →
OCCT on failure.

The remaining breadth is three Tier-4 cases. This change adds three tracks, each clean-room
(OCCT is the ORACLE only), each honest per-track:

- **T1** extends the ruled loft from EQUAL vertex counts to MISMATCHED counts via an
  arc-length vertex correspondence — the safest generalization, closed form, EXACT for
  polygons.
- **T2** generalizes the guided sweep from a SCALE constraint to an ORIENTATION constraint
  (the section aimed at the guide) — a harder frame law, landed as a narrow slice OR an
  honest decline.
- **T3** attempts the GENUINELY self-intersecting fine-pitch thread (crossing flanks) — the
  hardest, needing Tier-4 SSI trimming; a narrow slice OR an honest decline.

## Goals / Non-Goals

**Goals**
- T1: a native, OCCT-free MISMATCHED-count ruled loft (an M-gon lofted to an N-gon, `M ≠ N`)
  via an arc-length-parameter UNION correspondence — collinear point insertion, EXACT for
  polygons, welded watertight, self-verified, behind the UNCHANGED `cc_solid_loft` /
  `cc_solid_loft_wires`.
- T2: a native, OCCT-free ORIENTATION-constraining guided sweep on the narrowest
  reproducible slice (planar spine + planar guide, section aimed at the guide) — self-verified
  watertight and parity-matched to a real OCCT guide-orientation oracle, behind the UNCHANGED
  `cc_guided_sweep` — OR a documented HONEST DECLINE (no dead code, gap REPORTED).
- T3: a native, OCCT-free self-intersecting fine-pitch thread on the narrowest robust slice
  (crossing flanks trimmed, self-verified watertight + correct volume) behind the UNCHANGED
  `cc_helical_thread` / `cc_tapered_thread` — OR a documented HONEST DECLINE.
- All behind the mandatory self-verify; OCCT fallback for the rest.

**Non-Goals (return NULL → OCCT, never faked)**
- T1: a NON-PLANAR end section, a PUNCTUAL (all-coincident) section, a section with < 3
  distinct points, or a chain whose RESAMPLED stacked skin SELF-INTERSECTS.
- T2: a NON-PLANAR spine or guide, an accumulated-twist guide, a guide-orientation frame
  that is not reproducible per-station, or an orientation solid that does not match a real
  OCCT guide oracle behind the existing entry.
- T3: a crossing-flank thread whose trimmed solid does not self-verify watertight + correct
  volume, a root-dive taper, or any case needing full Tier-4 SSI beyond the narrow slice.
- Any track whose measured OCCT-parity gap exceeds the bound is declared out of slice
  (NULL → OCCT) and the gap REPORTED — never passed with a loosened tolerance.

## T1 — Mismatched-count ruled loft (clean-room, EXACT for polygons)

### The current constraint

`build_ruled_loft_sections(raw)` (`loft.h`) analyses every section, then:

```
const std::size_t n = secs.front().pts.size();
for (const auto& s : secs)
  if (s.pts.size() != n) return {};   // mismatched counts → OCCT (Tier C)
```

Equal counts flow into `alignSectionB` (rotational start + traversal direction that minimises
the summed paired distance) and `appendRuledBand` (one bilinear side face per corresponding
edge pair). T1 removes the hard `return {}` and inserts a CORRESPONDENCE step that makes the
counts match first.

### The arc-length parameter UNION correspondence

For each loop compute the normalized cumulative arc-length parameter of every vertex:

```
L      = Σ |p[i+1] − p[i]|                       (closed loop perimeter)
param(p[i]) = (Σ_{k<i} |p[k+1] − p[k]|) / L      ∈ [0, 1)
```

Given loops A (`M` verts, params `{α_i}`) and B (`N` verts, params `{β_j}`), form the sorted
UNION `U = {α_i} ∪ {β_j}` (dedup within `kProfileTol`), then RESAMPLE each loop at every
parameter `t ∈ U`:

```
resample(loop, t):
  find the edge [p[e], p[e+1]] whose param interval contains t
  local  = (t − param(p[e])) / (param(p[e+1]) − param(p[e]))
  return  p[e] + local · (p[e+1] − p[e])            // point ON the edge
```

Both resampled loops have the SAME count `K = |U| ≤ M + N`. Every ORIGINAL vertex survives (it
is a breakpoint of its own loop) and every inserted point is COLLINEAR on a straight edge of
the other loop. So:

- each resampled loop is the SAME planar polygon as its input (identical vertices in order,
  identical straight edges, identical area) — the resample is GEOMETRY-PRESERVING;
- the ruled surface between the two resampled loops is the SAME ruled surface between the
  original M-gon and N-gon, merely tiled into `K` bilinear bands instead of `M`/`N`;
- therefore the enclosed volume EQUALS the true ruled loft volume between the M-gon and the
  N-gon — EXACT for polygonal sections (deflection-bounded once meshed).

This mirrors OCCT `BRepFill_CompatibleWires` (the `SameNumberByACR` / polar compatibility step
`BRepOffsetAPI_ThruSections` runs before ruling): both wires are subdivided to a common edge
set at matched arc-length parameters. Nothing is copied; the construction is derived.

For an N-section chain the union is taken PAIRWISE down the chain propagation (section `k`
resampled against the already-fixed section `k−1`), reusing the existing predecessor-alignment
loop so the correspondence stays consistent along the whole chain. `alignSectionB` then fixes
the rotational origin/direction on the equal-`K` loops as today.

### Guards + self-verify

Unchanged section guards: PLANAR + non-degenerate (≥ 3 distinct points, non-null Newell
normal). A degenerate parameterization (a zero-perimeter loop) declines. The ENGINE runs the
mandatory self-verify: `robustlyWatertight(solid)` across the deflection ladder AND a positive
`watertightVolume`. A resampled skin that self-intersects (a badly-twisted M→N pairing the
alignment cannot resolve) fails watertight → DISCARDED → OCCT. `NativeEngine::solid_loft` /
`solid_loft_wires` gain this guard (they currently only null-check):

```cpp
ShapeResult NativeEngine::solid_loft(const double* b, int bc, const double* t, int tc, double d) {
  ntopo::Shape solid = ncst::build_loft(b, bc, t, tc, d);
  if (solid.isNull() || !robustlyWatertight(solid) || watertightVolume(solid) <= 0.0)
    return fallback().solid_loft(b, bc, t, tc, d);
  return track(wrapNative(std::move(solid)));
}
```

The equal-count path is the `K = M = N` special case (the union of identical param sets is
each set), so the existing 2- & N-section loft is BYTE-IDENTICAL in behaviour.

## T2 — Orientation-constraining guided sweep (clean-room — narrow slice OR honest decline)

### What already exists

`build_guided_sweep` (`sweep.h`) is native for the SCALE constraint: `scaleAt(f) =
dist(path(f), guide(f)) / d0`, sections placed through `frenetSectionFrames` (FIXED world up
`(0,1,0)`, `nrm = normalize(tangent × up)`), ruled + capped by `assembleRingTube`. This EXACTLY
matches the OCCT `guided_sweep` oracle (ThruSections through the guide-SCALED Frenet sections).

### The orientation law (harder frame math)

An ORIENTATION-constraining guide re-derives the section's up-axis from the guide instead of
world up:

```
for each station f:
  g   = normalize(guide(f) − path(f))            // spine → guide
  up  = normalize(g − tangent·(g·tangent))       // guide dir ⟂ tangent
  nrm = normalize(tangent × up)                  // section width axis
  section spans (nrm, up), centred at path(f)
```

so the section stays AIMED at the guide (the MakePipeShell `SetMode(guideWire)` law). This is a
pure per-station function of the LOCAL geometry (no accumulated corrected-Frenet rotation), so
it is reproducible — the necessary condition for a native match. Restrict to the narrow slice:
a PLANAR spine and a PLANAR guide in the same plane, no twist, `g` never parallel to the
tangent (else `up` collapses → decline).

### The honest gate (no dead code)

The load-bearing problem is the ORACLE, not the frame:

1. The shipped `cc_guided_sweep` facade entry is bound to the SCALE-splay oracle
   (`occt_construct.cpp guided_sweep`), which the native builder ALREADY matches. Its
   semantics are fixed.
2. An orientation frame produces a DIFFERENT solid. Shipping it behind `cc_guided_sweep`
   would either (a) CHANGE that entry's oracle — an ABI/semantics break the project forbids —
   or (b) FAIL parity vs the shipped scale oracle (exactly the RMF-vs-constant-frame mismatch
   `sweep.h` documents and the parity gate correctly rejects).
3. T2 is RETAINED as native ONLY if the narrow slice self-verifies watertight AND matches a
   REAL OCCT MakePipeShell guide-orientation oracle reachable behind an existing entry.
4. If it cannot (the realistic outcome — no orientation oracle behind the fixed scale-splay
   entry, and no ABI change permitted), T2 is an HONEST DECLINE: no builder retained,
   orientation-guided sweep stays a documented OCCT-fallthrough in `NativeEngine::guided_sweep`,
   the measured gap REPORTED. This mirrors the recent hard blend slices that correctly declined
   WITHOUT dead code.

Wiring (only if landed):

```cpp
// inside NativeEngine::guided_sweep, tried BEFORE the scale-splay path only when a
// distinct orientation oracle is bound behind the entry — else absent (honest decline):
ntopo::Shape solid = ncst::build_guided_sweep_oriented(p, pc, path, pathc, g, gc);
if (!solid.isNull() && robustlyWatertight(solid)) return track(wrapNative(std::move(solid)));
```

## T3 — Fine-pitch self-intersecting thread (hardest — narrow slice OR honest decline)

### What already exists

`build_thread` sweeps a radial-V section along the (tapered) pitch-line helix and tiles three
ruled bands per span. The FINE-PITCH RESOLVER (`resolveHalfBase`) opens a root flat so
near-touching turns (V bases MEET, `2·halfBase ≈ pitch`) weld a clean 2-manifold. What
`threadUnsafe` DEFERS is the GENUINE 3D fold: the helix LEAD ratio `pitch / (2π·pitchR)` at the
smallest pitch radius above `kMaxLeadRatio = 0.35`, where the radial-V flanks of ADJACENT turns
cross through each other — the crest of one turn passes the next.

### Why it is hard, and the honest gate

Crossing flanks are two helicoid surfaces that INTERSECT. A single radial-V ruled tiling emits
both flanks whole, so the mesh is self-overlapping — non-manifold no matter how vertices weld,
and its divergence-theorem volume diverges from the analytic ridge. Resolving it requires
surface-surface intersection: compute the intersection curve of the two overlapping flank
helicoids and TRIM each flank to it (a Tier-4 SSI operation the current thread builder does not
attempt). The gate:

1. Attempt ONLY a narrow slice (a fine pitch just past `kMaxLeadRatio` where the flank
   overlap is a single clean intersection curve, taper shallow, root clear of the axis).
2. Trim the crossing flanks to their intersection curve, re-tile, self-verify watertight +
   correct SHRINK-consistent volume + parity vs OCCT `BRepOffsetAPI_MakePipeShell` on the
   fixture.
3. If it passes → RETAIN the slice + raise the `kMaxLeadRatio` guard for exactly that slice.
4. If it does NOT pass robustly → DO NOT retain a dead builder. The self-intersecting thread
   stays a documented OCCT-fallthrough (the `kMaxLeadRatio` guard unchanged), T3 = honest
   decline, the measured gap REPORTED.

The realistic outcome is the HONEST DECLINE: robust flank-helicoid SSI trimming is genuinely
Tier-4 and outside this construction batch; the existing guard already defers the fold
correctly. Wiring stays as today — `NativeEngine::helical_thread` / `tapered_thread` already
forward on NULL / self-verify failure; T3 either adds a verified slice before that or leaves
the documented decline.

## Module shape

```
src/native/construct/
  loft.h     // T1. build_ruled_loft_sections gains detail::correspondByArcLength when
             //   counts differ (arc-length UNION resample → equal-K loops → existing
             //   alignSectionB + ruled bands). Equal-count path = K=M=N special case.
  sweep.h    // T2. NEW build_guided_sweep_oriented (+ detail::orientedGuideFrames) iff
             //   landed — the section aimed at the guide, planar slice, no twist. NOT
             //   retained if it cannot match a real guide-orientation oracle (honest decline).
  thread.h   // T3. NEW crossing-flank slice iff landed (SSI-trimmed flanks, NUMSCI-gated).
             //   NOT retained if the narrow slice does not self-verify (honest decline).
  native_construct.h  // doc-only: update the SUPPORTED/DEFERRED honesty block.
```

Cognitive-complexity: T1 is an arc-length resample loop + a union merge, both flat (systems
band, ≤ the current assembler). T2's oriented frame is a per-station closed-form loop (≤5), the
rest is the existing `assembleRingTube`. T3, if landed, is the SSI-trim tiler (systems band,
NUMSCI-gated, documented) — else absent.

## Engine wiring summary (the load-bearing change)

- `solid_loft` / `solid_loft_wires` — add the mandatory `robustlyWatertight` + positive-volume
  self-verify around the now-count-agnostic native loft (kept native only when verified, else
  forward to OCCT). The equal-count result is byte-identical (it already meshes watertight).
- `guided_sweep` — UNCHANGED structure (already self-verifies + forwards); T2 adds a verified
  orientation candidate before the fall-through IFF a real oracle is bound, else the documented
  decline.
- `helical_thread` / `tapered_thread` — UNCHANGED structure (already self-verify + forward);
  T3 adds a verified crossing-flank slice IFF it self-verifies, else the documented decline.
- A NULL builder result OR a failed self-verify DISCARDS the candidate and forwards the SAME
  arguments to the OCCT engine. No new guard type, no weakened tolerance.

## Verification model (two gates)

- **Host (no OCCT):**
  - T1: build two planar polygon loops with DIFFERENT vertex counts (e.g. a triangle and a
    hexagon, or a square and a pentagon); `build_loft` / `build_loft_wires`; assert the
    resampled sections trace the SAME polygons (area preserved), the solid is watertight
    (`boundaryEdgeCount == 0`), and its volume EQUALS the analytic ruled-loft volume between
    the two polygons within the deflection bound. The equal-count control is byte-identical.
    A non-planar / punctual / self-intersecting section declines → NULL. Default + NUMSCI-ON.
  - T2: the planar oriented-guide slice self-verifies watertight and the section up-axis tracks
    `normalize(guide − path)` at each station, OR the documented decline → NULL. Default +
    NUMSCI-ON.
  - T3: the crossing-flank slice self-verifies watertight + correct volume on its fixture
    (NUMSCI-ON), OR the documented decline → NULL with the gap REPORTED.
- **Sim native-vs-OCCT parity** (extend `run-sim-native-loft.sh` / `run-sim-native-sweep.sh` /
  `run-sim-native-thread.sh` + the `.mm`):
  - T1: `cc_solid_loft` / `cc_solid_loft_wires` native vs OCCT `BRepOffsetAPI_ThruSections` on
    an M-gon→N-gon fixture (`M ≠ N`), ≥ 2 mismatched-count fixtures. TIGHT bound — the polygon
    resample is EXACT (volume / bbox / watertight agree to fp precision).
  - T2: `cc_guided_sweep` native vs OCCT `BRepOffsetAPI_MakePipeShell` guide-orientation oracle
    on a planar fixture (iff landed); else the decline-parity fall-through (identical under both
    engines), gap REPORTED.
  - T3: `cc_helical_thread` native vs OCCT `BRepOffsetAPI_MakePipeShell` aux-spine on a
    fine-pitch crossing-flank fixture (iff landed); else the decline-parity fall-through, gap
    REPORTED.
  - No regression: `run-sim-native-loft.sh`, `run-sim-native-sweep.sh`,
    `run-sim-native-thread.sh`, and `run-sim-suite.sh` stay green.

## Decisions

- **T1 is the safest generalization — a geometry-preserving resample, EXACT for polygons.**
  Inserting collinear points changes no geometry, so the mismatched-count loft has the SAME
  volume as the true ruled loft; the correspondence reduces to the existing equal-count aligner
  on the resampled loops. Behind the UNCHANGED `cc_solid_loft` (no ABI change).
- **T2 is a narrow slice OR an honest decline — never a semantics break.** The shipped
  `cc_guided_sweep` oracle is scale-splay and already native; an orientation law has no oracle
  behind that fixed entry without breaking its semantics or failing parity, so T2 declines
  honestly unless a real guide-orientation oracle is reachable — no dead code, gap REPORTED.
- **T3 is a narrow slice OR an honest decline — never dead code.** Crossing flanks need Tier-4
  SSI trimming; the track is landed only if a narrow slice self-verifies watertight + correct
  volume, otherwise the self-intersecting thread stays OCCT-fallthrough behind the existing
  `kMaxLeadRatio` guard, gap REPORTED.
- **`robustlyWatertight` + positive volume — the SAME self-verify the family already uses.** No
  new guard type; T1 adds it to the loft dispatch (which lacked it); T2 / T3 reuse the existing
  guided/thread guards.

## Risks / Trade-offs

- **T1 alignment on a resampled skin.** A pathological M→N pairing (a thin star lofted to a
  rotated polygon) could produce a self-intersecting stacked skin; the arc-length union +
  `alignSectionB` minimises paired distance, and the engine watertight self-verify DISCARDS any
  residual self-intersection → OCCT. Bounded and honest.
- **T2 no orientation oracle behind the fixed entry.** The realistic outcome is an honest
  decline; attempting to ship an orientation frame behind the scale-splay `cc_guided_sweep`
  would break its fixed semantics — forbidden — so the track declines rather than fake a match.
- **T3 crossing-flank SSI.** Robust flank-helicoid intersection + trim is Tier-4; if the narrow
  slice cannot be built watertight with correct volume, the whole track declines honestly
  rather than emit a self-overlapping (non-manifold, volume-wrong) solid.
- **Slice narrowness vs honesty.** T1 ships the full mismatched-count polygon loft (broad and
  exact); T2 / T3 ship only their narrow slice (or decline). Accepted — the alternative is
  faking a non-tractable construct, which the project forbids. The measured OCCT-fallback gap
  is REPORTED, never masked with a weakened tolerance.
- **Unchanged ABI surface.** No new facade entry; all three tracks are native paths behind
  existing `cc_*` entries with OCCT fallback, so no consumer breaks.
