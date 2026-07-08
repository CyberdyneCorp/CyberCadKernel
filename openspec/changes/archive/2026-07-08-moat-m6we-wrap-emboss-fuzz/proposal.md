# Proposal — moat-m6we-wrap-emboss-fuzz (MOAT M6-breadth-5, the completeness bar's FIFTH domain)

## Why

MOAT M6 landed the FIRST differential-fuzzing harness — a seeded batch driving random-
but-valid curved-boolean operands through BOTH the native path and the OCCT oracle,
asserting ZERO SILENT WRONG RESULTS (`tests/sim/native_boolean_fuzz.mm`). M6b extended
it to the native STEP reader (`native_step_import_fuzz.mm`), M6c to the native swept-solid
construction library (`native_construct_fuzz.mm`), and M6d to the native blend library
(`native_blend_fuzz.mm`). Four fuzzed domains are not a completeness bar: drop-occt needs
the discipline proven across MORE native domains, so a silent wrong result in ANY migrated
capability is caught by a seeded batch, not only by the handful of hand-picked fixtures the
curated parity harnesses carry.

The native **wrap-emboss** feature (`src/native/feature/wrap_emboss.h`) is the natural
fifth domain. It already has a curated round-trip parity harness
(`native_wrap_emboss_parity.mm`) that proves a *handful* of hand-picked pads/pockets agree
with the OCCT oracle. That is exactly the M6-shaped gap: a curated harness proves a few
fixtures; a fuzzer proves a *seeded batch*. This change turns the native wrap-emboss path
into a differential fuzzer so it is held to the same zero-silent-wrong bar as the boolean,
import, construction, and blend domains.

This is INFRASTRUCTURE — a test harness, not a new geometry capability. `src/native/**`
is untouched and stays OCCT-free; the `cc_*` ABI is unchanged. The harness is additive
test/sim code only.

## What Changes

1. **A new differential-fuzzing harness** `tests/sim/native_wrap_emboss_fuzz.mm` (own
   `main()`, seed-driven) that, for a seeded batch of N random-but-valid wrap-emboss inputs:
   - DETERMINISTICALLY generates a wrap-emboss input from the families the native path
     CLAIMS: a **rectangular PAD** (emboss, material ADDED), a **rectangular DEBOSS**
     pocket (material removed), and a **convex N-gon** (n=3..7) **emboss** and **deboss** —
     all wrapped onto the lateral face of a capped cylinder built through the SAME native
     construct entry point the `cc_solid_extrude_profile` facade uses
     (`build_prism_profile` on a full-circle profile → a capped cylinder with ONE Cylinder
     wall face). Plus SPARSE out-of-scope DECLINE-exercisers — a **non-cylindrical base**
     (a box), a **>2π footprint**, a **deboss depth ≥ R**, and a **self-intersecting**
     (pentagram) loop — to exercise the native NULL branch. The RNG is splitmix64-seeded
     xoshiro256** keyed ONLY by an explicit seed (argv/env) — no clock, no `rand()`.
   - BUILDS + MEASURES each input: the OCCT-free native builder
     (`feature::wrap_emboss`) is called DIRECTLY so a NULL / non-watertight result is an
     UNAMBIGUOUS native DECLINE (not a silent facade forward), measured by the native
     tessellator (mesh vol/area, watertight, solid count).
   - CLASSIFIES each trial: **AGREED** / **HONESTLY-DECLINED** / **DISAGREED** /
     **ORACLE-INACCURATE** / **BOTH-DECLINED**, at a FIXED tolerance (never widened).
   - Prints a per-family coverage summary; exits 0 IFF `DISAGREED == 0` and no out-of-scope
     guard leak. Any DISAGREE / ORACLE-INACCURATE prints the seed + case index +
     family/param tuple + all measurements as a reproducible record.

2. **The oracle** — because OCCT has NO single wrap-emboss API (as with the construction
   and blend fuzzers), the **PRIMARY** correctness oracle is the **closed-form
   curvature-corrected changed volume**. The native map is `u = px/R` (arc-length → angle),
   `v = py + vMid`, so a footprint of flat (shoelace) area `A` covers a (u,v) measure `A/R`
   and the changed volume is the radial shell over that measure:
   `ΔV = A·|Rout² − R²|/(2R)` (emboss `Rout = R+height`; deboss `Rout = R−depth`). This
   closed form is UNIVERSAL across the rectangle AND polygon footprints. The **SECONDARY**
   oracle reconstructs the SAME solid by an OCCT boolean of the base cylinder with a
   wrapped shell wedge (`BRepPrimAPI_MakeCylinder` sectors → `BRepAlgoAPI_Fuse` for a pad /
   `BRepAlgoAPI_Cut` for a pocket, measured exactly by `BRepGProp`) — CLEAN only for the
   rectangle families (a rectangle wraps to an EXACT angular sector) and used there to
   cross-check the closed form AND supply the only independent AREA oracle.

3. **A run script** `scripts/run-sim-native-wrap-emboss-fuzz.sh` — compiles the harness for
   the iOS simulator against the native feature + construct + tessellate + math TUs
   (OCCT-free, **no numsci**) and the OCCT oracle toolkits (`TKPrim` + `TKBO` + `TKGeomAlgo`
   + `TKTopAlgo`), then runs it in a booted simulator. Seed + N are argv/env overridable
   with fixed deterministic defaults.

4. **The new `.mm` is added to `scripts/run-sim-suite.sh`'s SKIP list** (own `main()`,
   OCCT-oracle slice), matching the siblings `native_boolean_fuzz.mm`,
   `native_step_import_fuzz.mm`, `native_construct_fuzz.mm`, `native_blend_fuzz.mm`.

5. **Delta to the `native-verification` capability spec** adding requirements for the
   wrap-emboss fuzz domain (generator, closed-form + OCCT-reconstruction oracle, five-way
   classifier, coverage bar with logged honest exclusions).

## Honest scope (logged, not silently dropped)

- **OCCT reconstruction is rectangle-only.** Reconstructing a wrapped POLYGON pad in OCCT
  would re-implement the feature (its arcs would need their own faceting), so it is NOT
  clean — the SECONDARY OCCT oracle is honestly DECLINED for the polygon families, whose
  oracle is the exact closed form alone (the OCCT reconstruction still transitively
  validates the SAME closed-form formula on the rectangle families). This is a first-class
  honest DECLINE at the ORACLE level, recorded here and in the spec delta.
- **Deflection-bounded sensitivity.** The native builder facets the whole cylinder into
  planar triangles, so its measured volume/area sit a small, deflection-bounded amount
  below the smooth closed-form / OCCT solid. A polygon pad cap is an ear-clipped inscribed
  facet fan whose few-triangle interior (a single triangle for n=3) is a deflection-
  INDEPENDENT inscribing floor; the generator therefore BOUNDS the polygon footprint /
  height (the same "bounded feature size" discipline the blend fuzz applies to rim/edge
  size) so that floor stays comfortably under the FIXED tolerance. Rectangle pads are
  sagitta-tiled and carry the exact OCCT sector oracle, so they stay generous. The measured
  max bias is logged in the summary so the margin is auditable.
- **Out-of-scope inputs** (non-cylindrical base, >2π footprint, depth ≥ R,
  self-intersecting loop) are generated SPARINGLY to exercise the native DECLINE branch
  for real, NOT to manufacture DISAGREE.

## Impact

- Affected specs: `native-verification` (ADDED requirements only).
- Affected code: additive test/sim only — `tests/sim/native_wrap_emboss_fuzz.mm`,
  `scripts/run-sim-native-wrap-emboss-fuzz.sh`, one line in `scripts/run-sim-suite.sh`.
- `src/native/**` UNTOUCHED and OCCT-free; `cc_*` ABI unchanged.
- Proven: `DISAGREED == 0` across 6 seeds (0x5745E6B055 N=120; 0xC0FFEE01, 0xABCDEF99,
  0x1337BEEF, 0x2, 0xDEADBEEF N=240–300; ~1500 trials total) with real per-family coverage
  (all four core families AGREE in the hundreds each), all four DECLINE-exercisers hitting
  BOTH-DECLINED, and 100+ OCCT rectangle reconstructions validating the closed form per
  seed. Max native-vs-oracle bias on AGREE ≤ 9.5e-3 (a ~2× margin under the fixed `2e-2`
  tolerance); byte-identical determinism across two runs of the same seed. Zero DISAGREED,
  zero ORACLE-INACCURATE, zero guard leaks.
