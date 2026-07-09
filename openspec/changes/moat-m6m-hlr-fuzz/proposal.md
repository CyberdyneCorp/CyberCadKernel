# Proposal — moat-m6m-hlr-fuzz (MOAT M6-breadth-13, the THIRTEENTH domain)

## Why

The MOAT completeness bar (dropping OCCT is gated by *proven* native correctness) has a
growing set of landed differential-fuzzing domains — curved booleans
(`native_boolean_fuzz.mm`), STEP round-trip (`native_step_import_fuzz.mm`),
construction/loft/sweep (`native_construct_fuzz.mm`), blends (`native_blend_fuzz.mm`),
wrap/emboss (`native_wrap_emboss_fuzz.mm`), mesh mass-properties
(`native_mass_props_fuzz.mm`), geometry services (`native_geometry_services_fuzz.mm`),
rigid/similarity transforms (`native_transform_fuzz.mm`), reference/datum geometry
(`native_reference_geometry_fuzz.mm`), direct-modeling (`native_directmodel_fuzz.mm`), and
transformed-boolean (`native_transformed_boolean_fuzz.mm`).

The **ORTHOGRAPHIC HLR / DRAFTING service** — the native path behind the CyberCad app's
2D drawing views (`cc_hlr_project`, `src/native/drafting/orthographic_hlr.h` +
`silhouette.h`) — landed as MOAT M-GS GS1 with a CURATED per-solid parity harness
(`native_hlr_parity.mm`: a handful of hand-picked solids from fixed views) and the
geometry-services fuzzer holds the OCCT-free HLR *core* to a box-corner closed form. But
neither turns the SHIPPING `cc_*` HLR differential into a **seeded batch of random solids
at random rigid poses projected from random view directions**, classifying every trial.

An HLR pass that silently mis-labels a hidden edge as visible (or fabricates an outline,
or drops a silhouette) is a **silent wrong drawing the user's dimensioned view is built
on**, and nothing today searches the input space for one. This change extends the
completeness bar to that THIRTEENTH native domain. Among the roadmap's REMAINING candidate
domains, HLR is the highest-value unclosed one:

- **Highest-value gap:** GS1 HLR has *curated* parity but no *fuzz* domain — exactly the
  "no fuzz domain yet" the roadmap flags for a shipping native capability.
- **Drives the SHIPPING PATH under both engines.** Like `native_hlr_parity.mm` /
  `native_directmodel_fuzz.mm` (and UNLIKE the internal-C++ fuzzers), this harness drives
  the public `cc_*` facade under BOTH `cc_set_engine(1)` (NativeEngine, the OCCT-free
  orthographic-HLR + silhouette core) and `cc_set_engine(0)` (OCCT adapter →
  `HLRBRep_Algo` / `HLRBRep_HLRToShape`, the oracle), so it verifies the exact ABI +
  engine-toggle path the app uses, in the SAME drawing-plane basis.
- **A closed-form silhouette-tangency arbiter where one exists.** A CYLINDER side face's
  silhouette is the two generator lines at the angles θ* where the radial normal ⟂ view
  (`n·view = 0`, θ* = `atan2(−X·d, Y·d)`); a SPHERE's is the great circle in the plane ⟂
  view through the centre. These have an EXACT closed form (`silhouette.h`'s own
  derivation), projected into the drawing plane in plain fp64. This THIRD, engine-
  independent ground truth attributes a native-vs-OCCT gap instead of reflexively blaming
  either engine — and, per prior fuzzers (native has repeatedly proven MORE correct than
  OCCT at numeric edges), lets a native-vs-OCCT quadric mismatch that the closed form
  confirms native is right about be logged ORACLE_UNRELIABLE (native vindicated), never a
  bar failure.

The native drafting service is verified in source to be OCCT-free: `orthographic_hlr.h`
and `silhouette.h` include only `src/native/math` and consume the M0 tessellation
occluder. The polyhedral core traces box / prism edge sets exactly; the analytic
silhouette tracer handles cylinder / sphere / cone / frustum / `Kind::Torus`; a FREEFORM
(revolve-built B-spline-band) face has no closed-form silhouette and is honestly declined.

## What Changes

1. **A new HLR / drafting differential fuzzer** `tests/sim/native_hlr_fuzz.mm`
   (iOS-simulator, whole kernel + OCCT incl. TKHLR linked), reusing the landed harness
   machinery (splitmix64/xoshiro256** RNG, coverage tally, `std::_Exit` epilogue). Per
   trial it:
   - **deterministically generates** a random-but-VALID base solid from six families —
     `BOX`, `NGON` prism (3..8 sides), `CYLINDER`, `CONE`/frustum, `SPHERE`, and a
     `FREEFORM` (B-spline-meridian revolve → `Kind::BSpline` bands, the honest-decline
     probe) — built IDENTICALLY under both engines via `cc_solid_extrude` /
     `cc_solid_revolve` / `cc_solid_revolve_profile`, applies a random RIGID pose
     (`cc_rotate_shape_about` + `cc_translate_shape`; NO scale/mirror, so the projected
     silhouette is an exact isometry of the base), and picks a random VIEW direction + a
     non-parallel up hint. The RNG is keyed ONLY by an explicit `FUZZ_SEED` (argv/env,
     fixed default `0x4D6F617436`) — NO clock, NO `rand()`; same seed → byte-identical
     batch.
   - **projects the posed solid** through `cc_hlr_project` under BOTH engines
     (`cc_set_engine(0)` OCCT `HLRBRep_Algo` oracle, `cc_set_engine(1)` NativeEngine) and
     compares the visible/hidden 2D drawing-plane segment SETS: (a) COUNTS — for the
     polyhedral convex families (BOX / NGON) the visible+hidden counts are deterministic
     and MUST match; (b) total LENGTH — Σ visible and Σ hidden length within a deflection-
     matched relative band (tight for polyhedral, curve-sized for quadric silhouettes);
     (c) PARTITION — every native segment's endpoints lie on a SAME-CLASS oracle segment
     within tolerance (native visible ⊆ oracle visible, native hidden ⊆ oracle outline),
     and — for polyhedral — vice versa (identical labelled point set). A misclassified or
     fabricated segment lands off these and fails.
   - **arbitrates quadric silhouettes with the closed form.** For the CYLINDER and SPHERE
     families it builds the analytic silhouette (`n·view = 0`) in fp64, poses it by the
     same rigid transform, projects it into the drawing plane, and requires every native
     visible segment to lie on it within a curve tol. A native result matching the closed
     form while OCCT is the outlier is ORACLE_UNRELIABLE (native vindicated), not a fault.
   - **classifies** each trial into EXACTLY ONE of AGREED / HONESTLY-DECLINED / DISAGREED /
     ORACLE_UNRELIABLE / BOTH-DECLINED at FIXED tolerances (never widened). Prints a
     per-family + per-view-regime coverage summary; exits 0 IFF DISAGREED == 0. Any
     DISAGREED prints seed + case index + full param tuple.
2. **A runner** `scripts/run-sim-native-hlr-fuzz.sh` mirroring `run-sim-native-hlr.sh`
   (whole kernel + the full OCCT toolkit set incl. TKHLR, the `HLRBRep` oracle). It runs
   ≥2 seeds by default (`0x4D6F617436` / `0x171313C0FFEE`) and fails if any seed fails; the
   new `.mm` is added to the `run-sim-suite.sh` SKIP list. NO numsci: the HLR path is not
   `CYBERCAD_HAS_NUMSCI`-gated.
3. **Nothing in `src/native/**` or `src/engine/**` changes.** The native drafting +
   `cc_*` facade path is the SYSTEM UNDER TEST and stays byte-unchanged; the `cc_*` ABI is
   unchanged. If the fuzzer surfaces a real native disagreement it is reported with its
   seed — not silenced.

## Capabilities

### Modified Capabilities

- `native-verification`: ADDS the thirteenth differential-fuzzing domain — an
  orthographic-HLR / drafting harness that drives random valid solids (box / n-gon prism /
  cylinder / cone-frustum / sphere / freeform-decline) at random rigid poses from random
  view directions through the native `cc_hlr_project` under BOTH engines via
  `cc_set_engine`, comparing the visible/hidden 2D segment sets to a deflection-matched
  tolerance and arbitrating the cylinder/sphere silhouettes with a closed-form
  silhouette-tangency ground truth, with the freeform B-spline-band silhouette as a
  first-class honest decline.

## Impact

- `tests/sim/native_hlr_fuzz.mm` — NEW test harness (infrastructure).
- `scripts/run-sim-native-hlr-fuzz.sh` — NEW runner.
- `scripts/run-sim-suite.sh` — the new `.mm` added to the SKIP list (one line).
- `openspec/MOAT-ROADMAP.md` — M6 breadth row updated (×12 → ×13).
- **Zero production-code change.** `src/native/**`, `src/engine/**`, `include/**`, and the
  `cc_*` ABI stay BYTE-UNCHANGED. No tolerance is weakened; no result is silently capped or
  dropped.
- **Honest scope / decline (documented, not faked):** a FREEFORM (revolve-built B-spline-
  band) solid has no closed-form silhouette; the native core returns an EMPTY drawing and
  the OCCT oracle ships — counted HONESTLY-DECLINED, NEVER a DISAGREE. A degenerate view
  (exactly down a quadric axis, where the whole side is silhouette) that BOTH engines
  decline is BOTH-DECLINED. A quadric mismatch the closed form confirms native is right
  about (OCCT the numeric outlier) is ORACLE_UNRELIABLE (native vindicated), never a bar
  failure.
