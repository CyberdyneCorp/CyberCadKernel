# Design — moat-m6m-hlr-fuzz (MOAT M6-breadth-13)

## Context

The native orthographic-HLR / drafting service (MOAT M-GS GS1) ships behind
`cc_hlr_project`: under `cc_set_engine(1)` the OCCT-free
`src/native/drafting/orthographic_hlr.h` projects a solid's straight edge set over the
M0 tessellation occluder and classifies each projected sample VISIBLE/HIDDEN, and
`src/native/drafting/silhouette.h` traces the closed-form silhouette (`n·view = 0`) of a
cylinder / sphere / cone / frustum / `Kind::Torus` face; a freeform (B-spline/Bézier) face
is honestly declined. Under `cc_set_engine(0)` the OCCT adapter drives `HLRBRep_Algo` /
`HLRBRep_HLRToShape` — the oracle. Both return `CCDrawing` in the SAME drawing-plane basis
(`right = normalize(view × up)`, `trueUp = right × view`), so the 2D segment sets are
directly comparable.

The curated `native_hlr_parity.mm` proves a handful of hand-picked solids from fixed
views; the geometry-services fuzzer holds only the OCCT-free HLR *core* to a box-corner
closed form. Neither searches the input space of the SHIPPING `cc_*` HLR differential.

## Goals / Non-Goals

**Goals**
- A DETERMINISTIC seeded batch of random solids at random rigid poses projected from
  random view directions, driving `cc_hlr_project` under BOTH engines and asserting ZERO
  silent wrong HLR results (DISAGREED = 0) over ≥2 seeds, N ≥ 60/seed.
- A closed-form silhouette-tangency arbiter (`n·view = 0`) for the cylinder + sphere
  families to attribute a native-vs-OCCT quadric gap to the correct engine.
- First-class honest decline for the freeform B-spline-band silhouette.

**Non-Goals**
- No change to `src/native/**`, `src/engine/**`, `include/**`, or the `cc_*` ABI (system
  under test). No new geometry capability — infrastructure only.
- Not a completeness proof of every view/solid; an ongoing, seed-extensible bar.

## Key decisions

### D1 — Drive the SHIPPING `cc_*` facade under both engines (not the internal core)
Like `native_hlr_parity.mm` / `native_directmodel_fuzz.mm`, the harness includes ONLY
`cybercadkernel/cc_kernel.h` and toggles `cc_set_engine`, so it tests the exact ABI +
engine path the CyberCad app uses. The runner therefore links the WHOLE kernel + the full
OCCT toolkit set incl. TKHLR (the `HLRBRep` oracle), mirroring `run-sim-native-hlr.sh`.
NO numsci — the HLR path is not `CYBERCAD_HAS_NUMSCI`-gated (unlike the direct-model split
seam trace), so this is the plain-kernel slice.

### D2 — Random rigid poses, built identically under both engines
Each solid is built from the same `cc_solid_extrude` / `cc_solid_revolve` /
`cc_solid_revolve_profile` parameters under both engines, then posed by the SAME
`cc_rotate_shape_about` + `cc_translate_shape` (rotate about a random unit axis by a random
angle, then translate). NO scale / mirror — the projected silhouette is then an exact
ISOMETRY of the base, so the closed-form silhouette-tangency arbiter transforms cleanly in
fp64 (Rodrigues rotation of the analytic silhouette by the same pose) and projected
lengths are preserved.

### D3 — Bidirectional partition coverage is the authoritative agreement test
The visible/hidden classification is compared as a LABELLED POINT SET:
- **native ⊆ oracle** — every native visible segment lies (both endpoints) on an oracle
  VISIBLE segment, and every native hidden segment on the oracle OUTLINE (visible∪hidden),
  within the tolerance. This catches a native segment drawn in the WRONG class or
  FABRICATED off the outline.
- **oracle ⊆ native** — the reverse, which catches a MISSING native arc (an incomplete
  outline).

For POLYHEDRAL solids (box / n-gon prism) the projection is exact, so this runs at a TIGHT
`1e-4 mm` tol and exact segment counts are additionally required. For QUADRIC silhouettes
(cylinder / sphere / cone / frustum) two independent discretizers place points at
different phases on the same smooth curve, so the coverage runs at a CURVE-sized `0.08 mm`
tol and counts are NOT required equal.

The total projected LENGTH (Σ visible, Σ hidden) is a SECONDARY, discretization-sensitive
PROXY: a grazing-view silhouette foreshortens, so the two discretizers' chord densities
diverge and the relative total-length can exceed a few percent while tracing the identical
locus. It is therefore a CORROBORATING signal for curved families (required tight for
polyhedral, where the projection is exact), NOT the gate. When only the length proxy trips
but the bidirectional partition holds, the outlines are provably the same visible/hidden
locus and the trial AGREES — verified empirically: seed `0x171313C0FFEE` cases 45/47
(grazing cylinders, visLen rel ≈ 3.2%) pass `bi=1 v⊆v=1 o⊆v=1` and are correctly AGREED.
The `0.08 mm` curve tol is FIXED and NEVER widened; the length band is FIXED at 3% and used
only as a corroborating signal for curved families.

### D4 — Closed-form silhouette-tangency arbiter (cylinder + sphere)
Where a face has an exact silhouette (`n·view = 0`), the harness reproduces `silhouette.h`'s
derivation in plain fp64: a CYLINDER's two generator lines at θ* = `atan2(−X·d, Z·d)` (the
base cylinder revolves about +Y, so its in-plane axes are +X and +Z; the view is rotated
into the base frame by the inverse pose), each posed and projected; a SPHERE's great circle
of radius R in the plane ⟂ view through the centre, discretized and projected. If a
native-vs-OCCT quadric mismatch survives the bidirectional partition, the arbiter decides:
native-on-arbiter ∧ ¬oracle-on-arbiter → ORACLE_UNRELIABLE (native vindicated — OCCT the
numeric outlier, as prior fuzzers repeatedly found); otherwise DISAGREED. This never
launders a native fault into a pass — a native outline off the closed form still DISAGREES.

### D5 — Classifier and the bar
Each trial is EXACTLY ONE of AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE_UNRELIABLE /
BOTH-DECLINED (+ an un-attributable BUILD_FAIL when neither engine could build). A native
build/pose DECLINE (e.g. `cc_rotate_shape_about` declining a revolve-built-frustum rigid
placement — a native PLACEMENT scope limit, not an HLR fault) with a non-empty OCCT oracle
is HONESTLY-DECLINED (native → OCCT fallback for the whole projection), matching the
drop-OCCT semantics. The freeform B-spline-band family is the deliberate decline probe. The
bar: **DISAGREED == 0**, proven across ≥2 seeds with real per-family + per-view coverage.

### D6 — Determinism
splitmix64 → xoshiro256** keyed ONLY by an explicit `FUZZ_SEED` (argv/env, fixed default);
no clock, no `rand()`. Same seed → byte-identical batch (verified: re-run diff empty).

## Risks / trade-offs
- **The length proxy over-fires at grazing views.** Mitigated by making bidirectional
  partition the authoritative curved-family gate (D3) — the length band cannot mask a real
  divergence because a wrong/missing arc fails the partition regardless of length.
- **The arbiter only covers cylinder + sphere.** Cone/frustum/torus silhouettes are
  arbitrated by the bidirectional partition against the OCCT oracle alone; a future slice
  can add their closed forms. Freeform is declined, never arbitrated.
