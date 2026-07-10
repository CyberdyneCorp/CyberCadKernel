# Proposal — moat-m6t-ngon-fill-fuzz (MOAT M6-breadth, the N-SIDED FILL domain)

## Why

The MOAT completeness bar (removing OCCT is gated by *proven* native correctness, not
merely by defaulting to it) has a growing set of landed differential-fuzzing domains —
curved booleans, STEP round-trip, construction/loft/sweep, blends, wrap/emboss, mesh
mass-properties, geometry-services, transform chains, reference/datum geometry,
direct-modeling, transformed-boolean, section curves, orthographic HLR, shape-healing,
curved-blend, draft-angle, and interference.

A NEW native surface landed: the bounded **N-SIDED FILL / surface patch**
(`src/native/surface/{ngon_fill,fill_solid}.h`) — evaluate a Coons / Gregory transfinite
interpolant of a 3–6-sided ANALYTIC (straight-segment + circular-arc) boundary loop to a
tessellated triangle-MESH patch, reached through the shipping `cc_fill_ngon` facade under
the NativeEngine. That path has a *curated* per-op parity harness
(`native_surfacing_parity.mm`, four hand-picked fixtures — planar square, planar hexagon,
saddle quad, one arc side — plus one heptagon decline) but **no seeded differential
fuzzer** that drives *random* 3–6-sided analytic boundary loops through the facade under
BOTH engines and classifies every trial. This change closes exactly that gap and
certifies the freshly-landed fill surface at the facade level.

The N-fill fuzzer is high-value for three reasons:

1. **It certifies a surface that just landed** — random 3–6-sided loops (random corner
   poses, per-side straight/arc, planar + non-planar/saddle configurations), none
   previously under a *randomised* facade-level bar.
2. **A wrong patch is a silent-wrong-result the user cannot detect** — a positive-area,
   boundary-coincident patch that nonetheless has the WRONG area hands the user a
   corrupted surface presented as a valid fill. The bar searches the parameter space for
   that.
3. **It is the SHIPPING path, both engines.** It calls the exact public `cc_fill_ngon`
   facade the app calls, once under OCCT (`cc_set_engine(0)` → `BRepFill_Filling`) and
   once under the NativeEngine (`cc_set_engine(1)` → `surface::fillNGon` with an honest
   OCCT fallback on decline). It shares NO code with the concurrent freeform-boolean,
   variable-sweep, or interference tracks; it fuzzes only the STABLE landed `cc_fill_ngon`.

## The oracles (closed-form the PRIMARY arbiter where it exists, OCCT + boundary residual)

OCCT's `BRepFill_Filling` is ENERGY-MINIMIZING, so its interior patch DIFFERS from the
native transfinite interpolant and can BULGE past the loop hull. So interior-vertex
identity is NEVER compared. Instead:

- **Planar families** (`planar-Ngon`, `planar-hole-completion`) — the native planar
  fast-path emits the EXACT ear-clipped polygon, so the patch AREA equals the EXACT 3-D
  polygon area (Newell shoelace). This is a closed-form ground truth exact for the ideal
  planar fill (and, for hole-completion, exactly the missing planar face area a
  `fillHoleSolid` weld restores). A native area matching it while OCCT is the outlier is
  logged ORACLE-INACCURATE (native vindicated), never a bar failure.
- **Non-planar families** (`saddle-4sided`, `arc-boundary`) — no closed-form area, so the
  arbiter is OCCT AREA within a FIXED deflection band + BBOX-CONTAINMENT (the native
  Coons patch stays in the loop hull, OCCT bulges) + an OCCT-INDEPENDENT analytic-boundary
  RESIDUAL (every native boundary sample lies on its straight/circular-arc curve). The
  saddle out-of-plane amplitude is bounded to the SMALL-WARP regime where the transfinite
  and energy-minimizing surfaces provably co-area within the fixed band — outside it the
  two are different valid surfaces and area stops discriminating (a GENERATOR SCOPE BOUND,
  not a tolerance widening).

## What changes

- **ADD** `tests/sim/native_ngon_fill_fuzz.mm` — a deterministic seeded (splitmix64 →
  xoshiro256**, keyed ONLY by `FUZZ_SEED`) differential fuzzer that, per trial: builds a
  random 3–6-sided analytic boundary loop for one of the four families {planar-Ngon,
  planar-hole-completion, saddle-4sided, arc-boundary} × N∈{3,4,5,6}; fills it through the
  public `cc_fill_ngon` facade under BOTH engines; measures patch area (`cc_mass_properties`)
  and bbox (`cc_bounding_box`); computes the analytic-boundary residual; and classifies
  AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE / ORACLE_UNRELIABLE /
  BOTH-DECLINED. A sparse (every-11th, N≥4) self-intersecting bowtie exercises the native
  NULL → OCCT honest-decline branch.
- **ADD** `scripts/run-sim-native-ngon-fill-fuzz.sh` — cloned from
  `run-sim-native-draft-faces-fuzz.sh` (links the WHOLE kernel — facade + core +
  engine[native+occt] + native math — plus the full OCCT toolkit set, `TKHLR`/`TKShHealing`
  retained). The native fill path is OCCT-FREE and NOT `CYBERCAD_HAS_NUMSCI`-gated, so —
  unlike the draft-faces runner — this needs NO numsci substrate. Seeded ONLY by
  `FUZZ_SEED`/argv; default N=72 per seed; runs TWO default seeds and fails if either fails.
- **ADD** `native_ngon_fill_fuzz.mm` to the `run-sim-suite.sh` SKIP list (own `main()`,
  `std::_Exit`).
- **UPDATE** `openspec/MOAT-ROADMAP.md` M6 row: breadth ×17 → ×18 (this domain; the
  concurrent freeform-boolean + variable-sweep tracks bump it further — reconcile to ×20
  at merge).

## The bar

DISAGREED == 0 and ORACLE_UNRELIABLE == 0 across ≥2 seeds, N≥60 trials/seed (the runner
fails if ANY seed fails), each of the four families with ≥1 AGREED trial. The FIXED
tolerances (planar native-vs-closed-form area ≤ 1e-4 — planar fill area is exact;
planar OCCT-vs-closed-form ≤ 1e-3; non-planar native-vs-OCCT area ≤ 1.2e-1; bbox-
containment slack ≤ 8e-2; boundary residual ≤ 1e-6) are NEVER widened to force a pass. An
honest native NULL → OCCT decline is first-class.

## Non-goals / discipline

- `src/native/**`, `src/engine/**`, `include/**`, and the `cc_*` ABI stay
  **byte-unchanged** (test infrastructure only). If the fuzzer surfaces a real native fill
  bug it is REPORTED, not fixed here; a native result more correct than OCCT at a numeric
  edge is classified ORACLE-INACCURATE (native vindicated), not DISAGREED.
- Does NOT fuzz the concurrent freeform-boolean / variable-sweep / interference tracks nor
  any other op — only the STABLE landed `cc_fill_ngon`.
- No new geometry capability; no widened tolerance; no `Date.now()`/`rand()`.
