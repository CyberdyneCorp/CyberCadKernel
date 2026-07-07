## Why

Feature **#4 NATIVE CONSTRUCTION** has landed a wide swept-solid family: the prism / holed
/ typed-profile EXTRUDE, the line-segment + on-axis-arc + torus REVOLVE, the 2- and
N-section EQUAL-count RULED LOFT, the straight / smooth-planar / RMF SWEEP, the twisted and
SCALE-guided sweep, the straight-rail loft, the tapered SHANK, and the fine-pitch-RESOLVED
helical / tapered THREAD. Each is native only where it self-verifies watertight with the
correct volume; everything else is an honest OCCT-fallthrough.

The remaining construction breadth is three Tier-4 cases that today fall straight through to
OCCT:

- the ruled loft is EQUAL-count only — an M-gon lofted to an N-gon (`M ≠ N`) has an
  ambiguous vertex pairing and returns NULL → OCCT;
- the guided sweep constrains only SCALE (the section is uniformly scaled by the guide
  splay) — a guide that constrains ORIENTATION (the section aimed at the guide) is a harder
  frame law with no oracle behind the fixed `cc_guided_sweep` semantics;
- the helical thread RESOLVES near-touching turns (a root flat) but DEFERS the genuinely
  self-intersecting fine-pitch thread whose radial-V flanks cross in 3D (the lead-ratio
  guard) — that needs surface-surface intersection.

This change lands the construction-breadth batch as THREE honest tracks, each gated by the
engine's mandatory watertight + sane-volume self-verify, each returning NULL → OCCT (with
the measured gap REPORTED) outside its slice. All three land behind EXISTING facade entries
— no new ABI.

### T1 — Mismatched-count ruled loft (highest confidence, most tractable)

Extend `build_ruled_loft_sections` so an M-gon can loft to an N-gon (`M ≠ N`). The current
builder pairs vertex `k ↔ k` across sections (`detail::alignSectionB` chooses the rotational
start + traversal direction that minimises the summed paired distance) and REQUIRES every
section to share the same vertex count `n` — a mismatch returns NULL (`loft.h` line ≈358).

T1 adds a VERTEX CORRESPONDENCE that makes the counts match BEFORE pairing, mirroring OCCT
`BRepFill_CompatibleWires` (the compatibility step `ThruSections` runs before ruling):

- compute each loop's normalized cumulative arc-length parameters (loop A's `M` vertices at
  `{α_0=0, α_1, …}`, loop B's `N` vertices at `{β_0=0, β_1, …}`, each in `[0,1)`);
- resample BOTH loops at the sorted UNION of the two parameter sets, inserting a point on
  whichever loop lacks a given parameter — each resampled loop reaches the SAME count
  `K = |{α} ∪ {β}|` (`≤ M + N`);
- feed the two equal-`K` loops to the existing `alignSectionB` + ruled-band assembler.

An inserted point at parameter `t` on a STRAIGHT polygon edge is COLLINEAR — the resampled
loop traces the SAME planar polygon (identical vertices, identical edges as line segments,
identical area), so the ruled loft is UNCHANGED geometry, just tiled with more bands. The
enclosed volume therefore EQUALS the true ruled loft between the M-gon and the N-gon
(deflection-bounded; exact for polygonal sections). This is the tractable, EXACT track.

Behind the UNCHANGED `cc_solid_loft` / `cc_solid_loft_wires` (and the N-section
`build_loft_sections` chain). Guards unchanged: both sections PLANAR and non-degenerate;
the engine self-verify (watertight + positive volume, cross-checked vs OCCT) DISCARDS any
self-intersecting stacked skin. Declines (NULL → OCCT) a non-planar end section, a punctual
section, or a chain whose resampled skin self-intersects.

### T2 — Orientation-constraining guided sweep (medium/low confidence — narrow slice OR honest decline)

The shipped `cc_guided_sweep` (`build_guided_sweep`) is already NATIVE for a SCALE
constraint: each station's section is uniformly scaled by `dist(path(f), guide(f)) / d0`
and placed through the FIXED-world-up per-station Frenet frame (`frenetSectionFrames`) — an
exact match to the OCCT `guided_sweep` oracle (ThruSections through the guide-scaled Frenet
sections, `occt_construct.cpp`).

An ORIENTATION-constraining guide is a DIFFERENT operation: the section frame's up-axis is
derived from the spine→guide direction so the section stays AIMED at the guide as it sweeps
(the MakePipeShell `SetMode(guideWire)` law), not held to world up. This is the "harder
frame math". The honest gate:

- the orientation law is attempted ONLY for the narrowest reproducible slice — a PLANAR
  spine and a PLANAR guide in the same plane, no twist accumulation, the per-station up-axis
  `= normalize(guide(f) − path(f))` re-orthonormalized against the tangent — a pure
  per-station function of the local geometry (no accumulated corrected-Frenet rotation), so
  it is reproducible;
- it is RETAINED as native ONLY if it self-verifies watertight AND matches a REAL OCCT
  MakePipeShell guide-orientation oracle within the deflection bound.

The realistic outcome is an HONEST DECLINE: the existing `cc_guided_sweep` ABI entry is
bound to the SCALE-splay oracle (already native), and there is no separate
orientation-guide oracle behind that entry to verify against. Adopting the orientation frame
would either CHANGE that entry's fixed oracle (an ABI/semantics break — forbidden) or ship a
solid that FAILS parity vs the shipped scale oracle (the same RMF-vs-constant-frame mismatch
`sweep.h` already documents and correctly rejects). If the narrow slice cannot both
self-verify AND match a real guide-orientation oracle behind an existing entry, T2 is an
HONEST DECLINE — orientation-guided sweep stays OCCT-fallthrough, documented, the measured
gap REPORTED, with NO always-NULL dead builder retained.

### T3 — Fine-pitch self-intersecting thread (lowest confidence — narrow slice OR honest decline)

The helical / tapered thread (`build_thread`) already handles the near-touching regime: the
FINE-PITCH RESOLVER (`resolveHalfBase`) opens a small cylindrical root flat so adjacent
turns whose V bases would MEET weld a clean 2-manifold. What it DEFERS is the GENUINELY
self-intersecting fine-pitch thread — turns whose radial-V FLANKS cross through each other
in 3D at a steep helix lead (`threadUnsafe`: `pitch > kMaxLeadRatio·2π·pitchR`,
`kMaxLeadRatio = 0.35`). There the crest of one turn passes the next; the enclosed volume
diverges from the analytic ridge.

A single radial-V ruled tiling CANNOT represent crossing helicoid flanks watertight — the
mesh is non-manifold no matter how vertices weld. Resolving it requires surface-surface
intersection: trim the two overlapping flank helicoids against each other along their
intersection curve (a Tier-4 SSI operation, not attempted in the current thread builder).
T3 attempts a slice ONLY if it self-verifies watertight + correct volume vs OCCT; the
realistic outcome is an HONEST DECLINE — the self-intersecting thread stays a documented
OCCT-fallthrough (the `kMaxLeadRatio` guard), the measured gap REPORTED, with NO dead code.

## What Changes

- **T1 (loft, unchanged ABI).** Generalize `src/native/construct/loft.h`
  `build_ruled_loft_sections`: when the sections' vertex counts differ, run a new
  `detail::correspondByArcLength(sections)` that resamples every loop at the UNION of the
  loops' normalized arc-length parameters (collinear point insertion — geometry-preserving),
  yielding equal-`K` loops that flow into the existing `alignSectionB` + `appendRuledBand`
  assembler. The equal-count path is byte-unchanged (it is the `K = M = N` special case).
  `NativeEngine::solid_loft` / `solid_loft_wires` gain the mandatory self-verify
  (`robustlyWatertight` + positive `watertightVolume`) so the harder resampled tiling is
  kept native ONLY when it self-verifies, else forwarded to OCCT.

- **T2 (guided sweep, unchanged ABI, honest gate).** Add
  `detail::orientedGuideFrames` + `build_guided_sweep_oriented` in
  `src/native/construct/sweep.h` (the section up-axis derived from the spine→guide
  direction, planar-slice only, no twist) RETAINED only if it self-verifies watertight AND
  matches a real OCCT MakePipeShell guide-orientation oracle behind an existing entry. If it
  cannot (the shipped `cc_guided_sweep` oracle is scale-splay, already native; an
  orientation law breaks that fixed semantics or fails its parity), T2 is an HONEST DECLINE:
  no builder is retained, orientation-guided sweep stays a documented OCCT-fallthrough in
  `NativeEngine::guided_sweep`, the measured gap REPORTED.

- **T3 (thread, unchanged ABI, honest gate).** Attempt a self-intersecting-flank thread
  slice in `src/native/construct/thread.h` (raise the `kMaxLeadRatio` fold guard ONLY for a
  slice that trims the crossing flank helicoids and self-verifies watertight + correct
  volume). If the crossing-flank case cannot be built watertight with correct volume without
  Tier-4 SSI, T3 is an HONEST DECLINE: the `kMaxLeadRatio` guard stays, the self-intersecting
  thread remains a documented OCCT-fallthrough in `NativeEngine::helical_thread` /
  `tapered_thread`, the measured gap REPORTED, with NO dead code.

- **Engine wiring.** `NativeEngine::solid_loft` / `solid_loft_wires` add the self-verify
  around the (now count-agnostic) native loft. `guided_sweep` and `helical_thread` /
  `tapered_thread` are UNCHANGED in structure (they already self-verify `robustlyWatertight`
  and forward on NULL / failure); T2 / T3 either add a verified native candidate before that
  fall-through or leave it as the documented decline. A NULL builder result OR a failed
  self-verify DISCARDS the candidate and forwards the SAME arguments to the OCCT engine.

- **Native code stays OCCT-free.** T1 is closed-form polygon resampling + the existing
  bilinear ruled bands (no NUMSCI). T2's oriented frame is closed-form vector math. T3, if
  landed, needs the SSI trim substrate (`CYBERCAD_HAS_NUMSCI`-gated) OR is not retained.
  OCCT is referenced ONLY in the OCCT fallback engine.

**ABI:** UNCHANGED. `cc_solid_loft` / `cc_solid_loft_wires` / `cc_guided_sweep` /
`cc_helical_thread` / `cc_tapered_thread` keep their exact signatures; all three tracks are
new NATIVE paths behind those existing entries. No new facade entry, no POD struct change.

## Capabilities

### New Capabilities
<!-- none — this EXTENDS the living native-construction capability (mismatched-count loft +
     orientation-guided sweep / self-intersecting thread breadth). No new capability, and no
     kernel-facade change (all three tracks land behind existing cc_* entries). -->

### Modified Capabilities
- `native-construction`: add a native MISMATCHED-count ruled loft (`cc_solid_loft` /
  `cc_solid_loft_wires` between an M-gon and an N-gon, `M ≠ N`, via an arc-length-parameter
  UNION vertex correspondence — collinear point insertion, EXACT for polygons,
  self-verified watertight + volume); an ORIENTATION-constraining guided sweep
  (`cc_guided_sweep` with the section aimed at the guide) OR its documented HONEST DECLINE;
  and a FINE-PITCH self-intersecting thread (`cc_helical_thread` / `cc_tapered_thread` whose
  flanks cross in 3D) OR its documented HONEST DECLINE. Everything outside the landed slices
  (a non-planar / punctual / self-intersecting loft section, a non-reproducible /
  non-parity guide-orientation frame, a crossing-flank thread beyond the watertight slice)
  returns NULL → OCCT, and the measured gap is REPORTED. Delivered behind the UNCHANGED
  `cc_solid_loft` / `cc_solid_loft_wires` / `cc_guided_sweep` / `cc_helical_thread` /
  `cc_tapered_thread` — no ABI change.

## Impact

- **ABI**: unchanged. All three tracks are internal native paths behind existing `cc_*`
  entries; every path has OCCT fallback. No new entry, no POD layout change.
- **Build**: T1 generalizes `src/native/construct/loft.h` (arc-length resample +
  correspondence) — no new dependency, closed-form, NUMSCI-OFF. T2 adds an oriented-guide
  frame to `src/native/construct/sweep.h` — closed-form, NUMSCI-OFF — OR is not retained
  (honest decline). T3 needs the SSI trim substrate (`CYBERCAD_HAS_NUMSCI`-gated) OR is not
  retained (honest decline). `NativeEngine::solid_loft` / `solid_loft_wires` add the
  `robustlyWatertight` + positive-volume self-verify; `guided_sweep` / `helical_thread` /
  `tapered_thread` keep their existing self-verify. Consumes the two-stage tessellator (the
  watertight mesher — NOT modified) + the ruled-band assembler. Builds with NUMSCI OFF
  (T1 / T2) and NUMSCI ON (T1 / T2 regression + any T3 slice).
- **Verification**: two gates. **host** (OCCT-free CTest): T1 — an M-gon→N-gon loft
  (`M ≠ N`) meshes watertight with volume EQUAL to the analytic ruled-loft volume (the
  polygon resample is geometry-preserving), and the equal-count control is byte-identical;
  the non-planar / punctual / self-intersecting section still declines → NULL. T2 — the
  planar oriented-guide slice self-verifies watertight + on-guide orientation, OR the
  documented decline → NULL. T3 — the crossing-flank slice self-verifies watertight +
  correct volume, OR the documented decline → NULL with the gap REPORTED. **sim**
  native-vs-OCCT parity (`run-sim-native-loft.sh` / `run-sim-native-sweep.sh` /
  `run-sim-native-thread.sh` + the `.mm`): T1 vs `BRepOffsetAPI_ThruSections` on an
  M-gon→N-gon fixture (volume / bbox / watertight); T2 vs `BRepOffsetAPI_MakePipeShell`
  guide mode (or the decline-parity fall-through); T3 vs `BRepOffsetAPI_MakePipeShell`
  aux-spine on a fine-pitch fixture (or the decline-parity fall-through). Because T1's
  polygon resample is EXACT, its parity is TIGHT.
- **Roadmap**: advances `ROADMAP.md` #4 native-construction breadth (mismatched-count loft
  landed; guided-orientation + self-intersecting thread on the SSI on-ramp).
- **Regression**: additive only. The equal-count 2- & N-section loft
  (`run-sim-native-loft.sh`), the straight / planar / RMF / twisted / scale-guided sweep
  (`run-sim-native-sweep.sh`), the tapered shank + resolved helical / tapered thread
  (`run-sim-native-thread.sh`), the profile / holed extrude, native booleans, SSI, healing,
  import, and phase-3 suites are UNTOUCHED. The equal-count loft path is byte-identical
  (`K = M = N` special case); the new paths fire ONLY for the named mismatched /
  orientation / crossing-flank inputs, tried AFTER the existing paths, gated by the
  self-verify.
- **Risk / honesty**: T1 is an EXACT polygon resample (tight parity) — high confidence. T2
  is a narrow reproducible slice whose realistic outcome is an HONEST DECLINE (no
  orientation oracle behind the fixed scale-splay `cc_guided_sweep`). T3 is the hardest — a
  narrow slice OR an HONEST DECLINE (crossing flanks need Tier-4 SSI) with NO dead code and
  the measured gap REPORTED. NEVER a weakened tolerance, NEVER a faked solid, NEVER a leaky
  or volume-wrong native body accepted; everything outside a slice returns NULL → OCCT with
  the gap REPORTED.
